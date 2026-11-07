#!/usr/bin/env python3
"""10 Hz YOLO sign detection; Bool meanings follow docs/architecture.md."""
from pathlib import Path
import os
import time

# Apply before ROS logging initializes, including direct and ros2 run execution.
os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '{message}'

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Bool
import torch
from ultralytics import YOLO
from sign_ocr import DirectionOCR


def default_model_path():
    # Keep direct source execution working, including --symlink-install builds.
    source_model = Path(__file__).resolve().parents[1] / 'model/best.engine'
    if source_model.is_file():
        return source_model
    try:
        return Path(get_package_share_directory('yolo_detect')) / 'model/best.engine'
    except PackageNotFoundError:
        return source_model


MODEL_PATH = default_model_path()
IMAGE_TOPIC = '/camera/color/image_raw/compressed'
SHOW_IMAGE = True  # True: show detection window; False: disable GUI.
LEFT_RIGHT_YOLO = True  # False: ignore A/B overrides and use original YOLO classes.
# - true: A/B가 읽히면 해당 bbox를 left/right로 지정
# - false: A/B 강제 지정을 무시하고 기존 YOLO 결과 사용
OCR_MIN_SCORE = 0.80  # Glyph similarity, not a probability.
OCR_MIN_MARGIN = 0.08  # Minimum separation between the A and B scores.
WINDOW_NAME = 'YOLO Detection'
PUBLISH_HZ = 10.0
IMAGE_SIZE = 640
MAX_IMAGE_AGE_SEC = 0.30
CONFIRM_FRAMES = 2  # Require this many consecutive new frames, not timer ticks.

# Per-class minimum bounding-box area in original-image pixels squared.
# Image resizing for inference does not change these thresholds.
CLASS_THRESHOLDS = {
    'bump': {'confidence': 0.55, 'min_area_pixels': 3800},
    'crosswalk': {'confidence': 0.60, 'min_area_pixels': 3200},
    'left': {'confidence': 0.65, 'min_area_pixels': 3400},
    'parking': {'confidence': 0.60, 'min_area_pixels': 3900},
    'right': {'confidence': 0.65, 'min_area_pixels': 3400},
    'slow': {'confidence': 0.55, 'min_area_pixels': 3600},
}

PRESENCE_TOPICS = {
    '/yolo/bump_sign': ('bump',),
    '/yolo/slow_sign': ('slow',),
    '/yolo/crosswalk_sign': ('crosswalk',),
    '/yolo/parking_sign': ('parking',),
}
LR_TOPIC = '/yolo/LR_sign'  # False=LEFT/A, True=RIGHT/B. No detection: no message.


def filter_detections(rows, names, width, height):
    """Return highest confidence per class passing confidence and size thresholds."""
    detected = {}
    for x0, y0, x1, y1, confidence, class_id in rows:
        if not np.isfinite((x0, y0, x1, y1, confidence, class_id)).all():
            continue
        name = names[int(class_id)].strip().lower()
        threshold = CLASS_THRESHOLDS.get(name)
        if threshold is None:
            continue
        x0, x1 = np.clip((x0, x1), 0, width)
        y0, y1 = np.clip((y0, y1), 0, height)
        area_pixels = max(0., x1-x0)*max(0., y1-y0)
        if confidence >= threshold['confidence'] and area_pixels >= threshold['min_area_pixels']:
            detected[name] = max(detected.get(name, 0.), float(confidence))
    return detected


class DetectionState:
    def __init__(self):
        self.reset()

    def reset(self):
        self.counts = dict.fromkeys(CLASS_THRESHOLDS, 0)

    def update(self, detected, immediate_direction=False):
        for name in self.counts:
            self.counts[name] = min(CONFIRM_FRAMES, self.counts[name]+1) if name in detected else 0
        confirmed = {name for name, count in self.counts.items() if count >= CONFIRM_FRAMES}
        if immediate_direction:
            confirmed.difference_update(('left', 'right'))
            confirmed.update(name for name in ('left', 'right') if name in detected)
        values = {topic: bool(confirmed.intersection(classes))
                  for topic, classes in PRESENCE_TOPICS.items()}
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
        self.declare_parameter('image_transport', 'compressed')
        self.declare_parameter('camera_timeout_sec', 2.0)
        self.declare_parameter('left_right_yolo', LEFT_RIGHT_YOLO)
        self.left_right_yolo = self.get_parameter('left_right_yolo').value
        if not isinstance(self.left_right_yolo, bool):
            raise ValueError('left_right_yolo must be true or false')
        self.image_transport = self.get_parameter('image_transport').value
        if self.image_transport not in ('raw', 'compressed'):
            raise ValueError('image_transport must be raw or compressed')
        self.camera_timeout = float(self.get_parameter('camera_timeout_sec').value)
        if not np.isfinite(self.camera_timeout) or self.camera_timeout <= 0:
            raise ValueError('camera_timeout_sec must be finite and positive')
        self.bridge = CvBridge()
        self.direction_ocr = DirectionOCR(
            Path(__file__).resolve().parent / 'ocr/ab_templates.npz',
            min_score=OCR_MIN_SCORE, min_margin=OCR_MIN_MARGIN,
        ) if self.left_right_yolo else None
        self.show_image = SHOW_IMAGE
        self.preview_frame = None
        self.preview_dirty = False
        self.window_open = False
        if self.show_image and not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
            self.get_logger().warning('No graphical display available; preview disabled.')
            self.show_image = False
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
        self.presence_values = dict.fromkeys(PRESENCE_TOPICS, False)
        self.latest_image = None
        self.latest_received = 0.
        self.last_stamp = None
        self.last_processed_time = 0.
        output_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.VOLATILE)
        self.publishers_by_topic = {topic: self.create_publisher(Bool, topic, output_qos)
                                    for topic in (*PRESENCE_TOPICS, LR_TOPIC)}
        # Keep only the latest frame. Best Effort accepts the real
        # camera's Reliable publisher and SensorDataQoS publishers alike.
        input_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                               durability=DurabilityPolicy.VOLATILE)
        self.subscription = self.create_subscription(
            Image if self.image_transport == 'raw' else CompressedImage,
            self.get_parameter('image_topic').value, self.on_image, input_qos)
        self.camera_started = time.monotonic()
        self.camera_missing = False
        self.last_camera_error = None
        # Detect outages even when simulation time is paused.
        self.watchdog_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.preview_timer = self.create_timer(
            0.05, self.update_preview, clock=self.watchdog_clock)
        self.camera_watchdog = self.create_timer(
            0.5, self.check_camera, clock=self.watchdog_clock)
        self.get_logger().info(
            f'Camera input: {self.subscription.topic_name}; transport: {self.image_transport}')
        self.timer = self.create_timer(1./PUBLISH_HZ, self.detect, clock=self.watchdog_clock)
        self.get_logger().info(f'Model: {model_path}; device: {self.device}; output: {PUBLISH_HZ:g} Hz')

    def on_image(self, message):
        if self.latest_image is None or self.camera_missing:
            self.get_logger().info(f'Camera receiving: {self.subscription.topic_name}')
        self.camera_missing = False
        self.last_camera_error = None
        self.latest_image = message
        self.latest_received = time.monotonic()

    def update_preview(self):
        if not self.show_image or self.preview_frame is None:
            return
        try:
            if not self.window_open:
                cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
                self.window_open = True
            elif cv2.getWindowProperty(WINDOW_NAME, cv2.WND_PROP_VISIBLE) < 1:
                self.close_preview()
                return
            if self.preview_dirty:
                cv2.imshow(WINDOW_NAME, self.preview_frame)
                self.preview_dirty = False
            if cv2.waitKey(1) & 0xFF in (27, ord('q')):
                self.close_preview()
        except cv2.error as exc:
            self.get_logger().error(f'Preview disabled: {exc}')
            self.close_preview()

    def close_preview(self):
        self.show_image = False
        self.preview_frame = None
        if self.window_open:
            try:
                cv2.destroyWindow(WINDOW_NAME)
            except cv2.error:
                pass
            self.window_open = False

    def destroy_node(self):
        self.close_preview()
        return super().destroy_node()

    def check_camera(self):
        now = time.monotonic()
        last_received = self.latest_received if self.latest_image is not None else self.camera_started
        silence = now - last_received
        if silence < self.camera_timeout:
            return
        self.camera_missing = True
        self.state.reset()
        if self.last_camera_error is None or now - self.last_camera_error >= 5.0:
            self.get_logger().error(
                f'No camera messages: topic={self.subscription.topic_name}; '
                f'transport={self.image_transport}; silence={silence:.1f}s. '
                'Check camera publisher, message type, ROS_DOMAIN_ID and QoS.')
            self.last_camera_error = now

    def log_detections(self, detected, values, reason):
        for name, confidence in sorted(detected.items()):
            topic = next((topic for topic, classes in PRESENCE_TOPICS.items()
                          if name in classes), LR_TOPIC)
            status = f'published={values[topic]}' if topic in values else f'not published ({reason})'
            self.get_logger().info(
                f'Detection: class={name}; output_topic={topic}; '
                f'confidence={confidence:.3f}; {status}')

    def classify_detections(self, rows, image):
        """A/B overrides the YOLO class, while retaining the direction pixel-area gate."""
        height, width = image.shape[:2]
        detected, annotations, rejected = {}, [], []
        for row in rows:
            if not np.isfinite(row).all():
                continue
            yolo_name = self.names[int(row[5])].strip().lower()
            candidates = filter_detections([row], self.names, width, height)
            confidence = float(row[4])
            name = yolo_name
            x0, y0, x1, y1 = row[:4]
            x0, x1 = np.clip((int(x0), int(x1)), 0, width)
            y0, y1 = np.clip((int(y0), int(y1)), 0, height)
            bbox_width = max(0, int(x1 - x0))
            bbox_height = max(0, int(y1 - y0))
            self.get_logger().info(
                f'BBox: class={yolo_name}; confidence={confidence:.3f}; '
                f'width={bbox_width}px; height={bbox_height}px; '
                f'area={bbox_width * bbox_height}px^2')
            label = f'{name} YOLO:{confidence:.2f}'
            if self.direction_ocr is not None:
                reading = self.direction_ocr.read(image[y0:y1, x0:x1])
                if reading.letter:
                    name = {'A': 'left', 'B': 'right'}[reading.letter]
                    label = f'{name} / {reading.letter} YOLO:{confidence:.2f} OCR:{reading.score:.2f}'
                    self.get_logger().info(
                        f'Direction OCR: letter={reading.letter}; class={name}; '
                        f'yolo_candidate={yolo_name}; confidence={confidence:.3f}; '
                        f'ocr_score={reading.score:.3f}; output_topic={LR_TOPIC}')
                    if bbox_width * bbox_height < CLASS_THRESHOLDS[name]['min_area_pixels']:
                        rejected.append((name, confidence, 'bbox area below direction threshold', reading.score))
                        annotations.append(((int(x0), int(y0), int(x1), int(y1)), label))
                        continue
                    detected[name] = max(detected.get(name, 0.), confidence)
                    annotations.append(((int(x0), int(y0), int(x1), int(y1)), label))
                    continue
                if yolo_name in ('left', 'right'):
                    rejected.append((yolo_name, confidence, reading.reason, reading.score))
                    continue
            # No A/B: retain ordinary YOLO handling for non-direction classes.
            if candidates:
                detected[name] = max(detected.get(name, 0.), confidence)
            elif yolo_name in ('left', 'right'):
                continue
            annotations.append(((int(x0), int(y0), int(x1), int(y1)), label))
        return detected, annotations, rejected

    def render_preview(self, image, annotations, rejected):
        preview = image.copy()
        for (x0, y0, x1, y1), label in annotations:
            cv2.rectangle(preview, (x0, y0), (x1, y1), (0, 220, 0), 2)
            cv2.putText(preview, label, (max(0, x0), max(18, y0-6)),
                        cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 220, 0), 2, cv2.LINE_AA)
        if rejected:
            cv2.putText(preview, f'OCR unreadable: {len(rejected)} direction candidate(s)',
                        (8, 20), cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 180, 255), 1, cv2.LINE_AA)
        self.preview_frame = preview
        self.preview_dirty = True

    def fresh(self, stamp_ns, received):
        age = (self.get_clock().now().nanoseconds-stamp_ns)/1e9
        return -.05 <= age <= MAX_IMAGE_AGE_SEC and time.monotonic()-received <= MAX_IMAGE_AGE_SEC

    def publish_presence(self, clear=False):
        """Presence is a continuous Bool state; direction remains an event."""
        if clear:
            self.state.reset()
            self.presence_values = dict.fromkeys(PRESENCE_TOPICS, False)
        for topic, value in self.presence_values.items():
            self.publishers_by_topic[topic].publish(Bool(data=value))

    def detect(self):
        message, received = self.latest_image, self.latest_received
        if message is None:
            self.publish_presence(clear=True)
            return
        stamp_ns = message.header.stamp.sec*1_000_000_000+message.header.stamp.nanosec
        if not self.fresh(stamp_ns, received):
            self.publish_presence(clear=True)
            return
        if self.last_stamp is not None and stamp_ns <= self.last_stamp:
            self.publish_presence(
                clear=time.monotonic()-self.last_processed_time > MAX_IMAGE_AGE_SEC)
            return
        self.last_stamp = stamp_ns
        if time.monotonic()-self.last_processed_time > MAX_IMAGE_AGE_SEC:
            self.state.reset()
        started = time.monotonic()
        try:
            if self.image_transport == 'raw':
                image = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
            else:
                image = cv2.imdecode(np.frombuffer(message.data, dtype=np.uint8), cv2.IMREAD_COLOR)
            if image is None:
                raise ValueError('Cannot decode compressed camera image')
            result = self.model.predict(source=image, imgsz=IMAGE_SIZE, conf=self.minimum_confidence,
                                        device=self.device, verbose=False, save=False)[0]
            rows = result.boxes.data.cpu().numpy() if result.boxes is not None else []
            detected, annotations, rejected = self.classify_detections(rows, image)
            for name, confidence, reason, score in rejected:
                self.get_logger().info(
                    f'Direction OCR: yolo_candidate={name}; confidence={confidence:.3f}; '
                    f'ocr_score={score:.3f}; no direction assigned ({reason}); not published')
            if self.show_image:
                self.render_preview(image, annotations, rejected)
        except (CvBridgeError, cv2.error, ValueError, RuntimeError) as exc:
            self.publish_presence(clear=True)
            self.get_logger().error(f'YOLO frame failed: {exc}', throttle_duration_sec=5.)
            return
        # An expired acquisition clears presence and never emits a direction.
        if not self.fresh(stamp_ns, received):
            self.log_detections(detected, {}, 'image expired after inference')
            self.publish_presence(clear=True)
            return
        self.last_processed_time = time.monotonic()
        values = self.state.update(detected, immediate_direction=self.direction_ocr is not None)
        self.presence_values = {topic: values[topic] for topic in PRESENCE_TOPICS}
        for topic, value in values.items():
            self.publishers_by_topic[topic].publish(Bool(data=value))
        self.log_detections(detected, values, 'awaiting consecutive frames or ambiguous direction')
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
