#!/usr/bin/env python3
"""10 Hz YOLO sign detection; Bool meanings follow docs/architecture.md."""
from pathlib import Path
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool
import torch
from ultralytics import YOLO


MODEL_PATH = Path(__file__).resolve().parents[1] / 'model/test_in_e87.pt'
IMAGE_TOPIC = '/camera/color/image_raw/compressed'
PUBLISH_HZ = 10.0
IMAGE_SIZE = 640
MAX_IMAGE_AGE_SEC = 0.30
CONFIRM_FRAMES = 2  # Require this many consecutive new frames, not timer ticks.

# Tune each class independently here. These are initial values, not calibrated.
# min_area_ratio = bounding-box area / original image area (NOT resized input).
# Example: 0.002 at 640x480 means at least 614.4 pixels of bounding-box area.
CLASS_THRESHOLDS = {
    'bump':      {'confidence': 0.55, 'min_area_ratio': 0.0015},
    'crosswalk': {'confidence': 0.60, 'min_area_ratio': 0.0020},
    'left':      {'confidence': 0.65, 'min_area_ratio': 0.0020},
    'parking':   {'confidence': 0.60, 'min_area_ratio': 0.0020},
    'right':     {'confidence': 0.65, 'min_area_ratio': 0.0020},
    'slow':      {'confidence': 0.55, 'min_area_ratio': 0.0015},
}

PRESENCE_TOPICS = {
    # Both speed-bump and zigzag slow sections use this single topic.
    '/yolo/slow_sign': ('bump', 'slow'),
    '/yolo/crosswalk_sign': ('crosswalk',),
    '/yolo/parking_sign': ('parking',),
}
LR_TOPIC = '/yolo/LR_sign'  # False=LEFT/A, True=RIGHT/B. No detection: no message.


def filter_detections(rows, names, width, height):
    """Return class names passing their own confidence AND size thresholds."""
    detected = set()
    for x0, y0, x1, y1, confidence, class_id in rows:
        if not np.isfinite((x0, y0, x1, y1, confidence, class_id)).all():
            continue
        name = names[int(class_id)].strip().lower()
        threshold = CLASS_THRESHOLDS.get(name)
        if threshold is None:
            continue
        x0, x1 = np.clip((x0, x1), 0, width)
        y0, y1 = np.clip((y0, y1), 0, height)
        area_ratio = max(0., x1-x0)*max(0., y1-y0)/(width*height)
        if confidence >= threshold['confidence'] and area_ratio >= threshold['min_area_ratio']:
            detected.add(name)
    return detected


class DetectionState:
    def __init__(self):
        self.reset()

    def reset(self):
        self.counts = dict.fromkeys(CLASS_THRESHOLDS, 0)

    def update(self, detected):
        for name in self.counts:
            self.counts[name] = min(CONFIRM_FRAMES, self.counts[name]+1) if name in detected else 0
        confirmed = {name for name, count in self.counts.items() if count >= CONFIRM_FRAMES}
        values = {topic: True for topic, classes in PRESENCE_TOPICS.items()
                  if confirmed.intersection(classes)}
        # Two opposite signs passing the thresholds is an ambiguous direction.
        if ('left' in confirmed) != ('right' in confirmed):
            values[LR_TOPIC] = 'right' in confirmed
        return values


class YoloDetect(Node):
    def __init__(self):
        super().__init__('yolo_detect')
        self.declare_parameter('model_path', str(MODEL_PATH))
        self.declare_parameter('image_topic', IMAGE_TOPIC)
        self.declare_parameter('device', 'auto')
        model_path = Path(self.get_parameter('model_path').value).expanduser().resolve()
        if not model_path.is_file():
            raise FileNotFoundError(f'Model not found: {model_path}')
        device = self.get_parameter('device').value
        self.device = ('0' if torch.cuda.is_available() else 'cpu') if device == 'auto' else device
        self.model = YOLO(str(model_path), task='detect')
        self.names = self.model.names
        model_classes = {name.strip().lower() for name in self.names.values()}
        missing = set(CLASS_THRESHOLDS)-model_classes
        if missing:
            raise ValueError(f'Model is missing configured classes: {sorted(missing)}')
        self.minimum_confidence = min(t['confidence'] for t in CLASS_THRESHOLDS.values())
        self.state = DetectionState()
        self.latest_image = None
        self.latest_received = 0.
        self.last_stamp = None
        self.last_processed_time = 0.
        output_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.VOLATILE)
        self.publishers_by_topic = {topic: self.create_publisher(Bool, topic, output_qos)
                                    for topic in (*PRESENCE_TOPICS, LR_TOPIC)}
        # Keep only the latest compressed frame. Best Effort accepts the real
        # camera's Reliable publisher and SensorDataQoS publishers alike.
        input_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                               durability=DurabilityPolicy.VOLATILE)
        self.subscription = self.create_subscription(
            CompressedImage, self.get_parameter('image_topic').value, self.on_image, input_qos)
        self.timer = self.create_timer(1./PUBLISH_HZ, self.detect)
        self.get_logger().info(f'Model: {model_path}; device: {self.device}; output: {PUBLISH_HZ:g} Hz')

    def on_image(self, message):
        self.latest_image = message
        self.latest_received = time.monotonic()

    def fresh(self, stamp_ns, received):
        age = (self.get_clock().now().nanoseconds-stamp_ns)/1e9
        return -.05 <= age <= MAX_IMAGE_AGE_SEC and time.monotonic()-received <= MAX_IMAGE_AGE_SEC

    def detect(self):
        message, received = self.latest_image, self.latest_received
        if message is None:
            return
        stamp_ns = message.header.stamp.sec*1_000_000_000+message.header.stamp.nanosec
        if self.last_stamp is not None and stamp_ns <= self.last_stamp:
            if time.monotonic()-self.last_processed_time > MAX_IMAGE_AGE_SEC:
                self.state.reset()
            return
        if not self.fresh(stamp_ns, received):
            self.state.reset()
            return
        self.last_stamp = stamp_ns
        if time.monotonic()-self.last_processed_time > MAX_IMAGE_AGE_SEC:
            self.state.reset()
        started = time.monotonic()
        try:
            image = cv2.imdecode(np.frombuffer(message.data, dtype=np.uint8), cv2.IMREAD_COLOR)
            if image is None:
                raise ValueError('Cannot decode compressed camera image')
            result = self.model.predict(source=image, imgsz=IMAGE_SIZE, conf=self.minimum_confidence,
                                        device=self.device, verbose=False, save=False)[0]
            rows = result.boxes.data.cpu().numpy() if result.boxes is not None else []
            detected = filter_detections(rows, self.names, image.shape[1], image.shape[0])
        except (cv2.error, ValueError, RuntimeError) as exc:
            self.state.reset()
            self.get_logger().error(f'YOLO frame failed: {exc}', throttle_duration_sec=5.)
            return
        # Inference can take time. Never renew an expired acquisition as a new
        # Bool, and never replay the same frame just to maintain a message rate.
        if not self.fresh(stamp_ns, received):
            self.state.reset()
            return
        self.last_processed_time = time.monotonic()
        values = self.state.update(detected)
        for topic, value in values.items():
            self.publishers_by_topic[topic].publish(Bool(data=value))
        elapsed = time.monotonic()-started
        if elapsed > 1./PUBLISH_HZ:
            self.get_logger().warning(
                f'Inference took {elapsed:.3f}s; cannot sustain {PUBLISH_HZ:g} Hz on this device.',
                throttle_duration_sec=5.)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = YoloDetect()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
