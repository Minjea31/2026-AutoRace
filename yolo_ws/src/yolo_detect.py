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
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.clock import Clock, ClockType
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Bool, Float32MultiArray
import torch
from ultralytics import YOLO


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
WINDOW_NAME = 'YOLO Detection'
PUBLISH_HZ = 10.0
IMAGE_SIZE = 640
MAX_IMAGE_AGE_SEC = 0.30
CONFIRM_FRAMES = 2  # Require this many consecutive new frames, not timer ticks.

# Per-class minimum bounding-box area in original-image pixels squared.
# Image resizing for inference does not change these thresholds.
CLASS_THRESHOLDS = {
    'bump': {'confidence': 0.55, 'min_area_pixels': 2500},
    'crosswalk': {'confidence': 0.60, 'min_area_pixels': 2500},
    # left/right (A/B): 3400 -> 1000 (2026-11-10). 3400 confirmed the sign only ~0.7 m before
    # the island, too late for a lane change. The YOLO class decides A/B (no OCR); in the
    # 2026-11-10 car-06 bag it was 'left' with confidence >= 0.85 from ~20 px boxes.
    'left': {'confidence': 0.65, 'min_area_pixels': 1000},
    'parking': {'confidence': 0.60, 'min_area_pixels': 2000},
    'right': {'confidence': 0.65, 'min_area_pixels': 1000},
    'slow': {'confidence': 0.55, 'min_area_pixels': 2500},
    'red': {'confidence': 0.60, 'min_area_pixels': 2000},
    'green': {'confidence': 0.60, 'min_area_pixels': 2000},
}

PRESENCE_TOPICS = {
    '/yolo/bump_sign': ('bump',),
    '/yolo/slow_sign': ('slow',),
    '/yolo/crosswalk_sign': ('crosswalk',),
    '/yolo/parking_sign': ('parking',),
}
LR_TOPIC = '/yolo/LR_sign'  # False=LEFT/A, True=RIGHT/B. No detection: no message.
SIGNAL_TOPIC = '/yolo/signal'  # False=red, True=green; no message on absence/ambiguity.
SIGNAL_CLASS_IDS = {6: 'red', 7: 'green'}
# Published with every /yolo/LR_sign: the box that confirmed it, for logs and
# the lane fork snapshot. [right(1)/left(0), area_px2, width, height, yolo_confidence,
# x0, y0, x1, y1, stamp_sec, stamp_nanosec] in original-image pixels.
LR_DETAIL_TOPIC = '/yolo/LR_detail'

BBOX_COLOURS = {
    'bump': (70, 170, 255), 'slow': (90, 220, 150),
    'parking': (255, 175, 80), 'crosswalk': (80, 220, 255),
    'left': (240, 140, 180), 'right': (170, 220, 80),
    'red': (70, 70, 255), 'green': (70, 220, 70),
}
UNPUBLISHED_COLOUR = (140, 140, 140)


def bbox_is_published(box, values):
    if not box['eligible']:
        return False
    name = box['class']
    if name in ('left', 'right'):
        return LR_TOPIC in values and values[LR_TOPIC] == (name == 'right')
    if name in ('red', 'green'):
        return SIGNAL_TOPIC in values and values[SIGNAL_TOPIC] == (name == 'green')
    topic = next((topic for topic, classes in PRESENCE_TOPICS.items() if name in classes), None)
    return bool(topic and values.get(topic, False))


def draw_bbox_publications(image, boxes, values):
    preview = image.copy()
    height, width = preview.shape[:2]
    for box in boxes:
        colour = BBOX_COLOURS.get(box['class'], (100, 220, 150)) if bbox_is_published(box, values) else UNPUBLISHED_COLOUR
        x0, y0, x1, y1 = box['xyxy']
        cv2.rectangle(preview, (x0, y0), (max(x0, x1 - 1), max(y0, y1 - 1)), colour, 2)
        label = f"{box['class']} {box['width']}x{box['height']} {box['area']}px^2"
        (text_width, text_height), baseline = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, .35, 1)
        text_x = max(0, min(x0, width - text_width - 5))
        text_y = min(height - baseline - 3, y1 + text_height + 5)
        cv2.rectangle(preview, (text_x, text_y - text_height - 3),
                      (min(width - 1, text_x + text_width + 4), text_y + baseline + 2), (18, 18, 18), -1)
        cv2.putText(preview, label, (text_x + 2, text_y), cv2.FONT_HERSHEY_SIMPLEX, .35, colour, 1, cv2.LINE_AA)
    return preview


def detection_class_name(class_id, names):
    """Use the eight-class model's fixed light IDs and reject invalid class IDs."""
    if not np.isfinite(class_id) or class_id < 0 or class_id != int(class_id):
        return None
    class_id = int(class_id)
    if class_id not in names:
        return None
    if class_id in SIGNAL_CLASS_IDS:
        return SIGNAL_CLASS_IDS[class_id]
    name = names[class_id].strip().lower()
    return None if name in ('red', 'green') else name


def filter_detections(rows, names, width, height):
    """Return highest confidence per class passing confidence and size thresholds."""
    detected = {}
    for x0, y0, x1, y1, confidence, class_id in rows:
        if not np.isfinite((x0, y0, x1, y1, confidence, class_id)).all():
            continue
        name = detection_class_name(class_id, names)
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

    def update(self, detected):
        for name in self.counts:
            self.counts[name] = min(CONFIRM_FRAMES, self.counts[name]+1) if name in detected else 0
        # An ambiguous light must acquire two fresh unambiguous frames again.
        if 'red' in detected and 'green' in detected:
            self.counts['red'] = self.counts['green'] = 0
        confirmed = {name for name, count in self.counts.items() if count >= CONFIRM_FRAMES}
        values = {topic: bool(confirmed.intersection(classes))
                  for topic, classes in PRESENCE_TOPICS.items()}
        # Two opposite signs passing the thresholds is an ambiguous direction.
        if ('left' in confirmed) != ('right' in confirmed):
            values[LR_TOPIC] = 'right' in confirmed
        if ('red' in confirmed) != ('green' in confirmed):
            values[SIGNAL_TOPIC] = 'green' in confirmed
        return values


class YoloDetect(Node):
    def __init__(self):
        super().__init__('yolo_detect')
        self.declare_parameter('model_path', str(MODEL_PATH))
        self.declare_parameter('image_topic', IMAGE_TOPIC)
        self.declare_parameter('device', 'auto')
        self.declare_parameter('image_transport', 'compressed')
        self.declare_parameter('camera_timeout_sec', 2.0)
        self.declare_parameter('show_image', SHOW_IMAGE)
        self.image_transport = self.get_parameter('image_transport').value
        if self.image_transport not in ('raw', 'compressed'):
            raise ValueError('image_transport must be raw or compressed')
        self.camera_timeout = float(self.get_parameter('camera_timeout_sec').value)
        if not np.isfinite(self.camera_timeout) or self.camera_timeout <= 0:
            raise ValueError('camera_timeout_sec must be finite and positive')
        self.bridge = CvBridge()
        self.show_image = self.get_parameter('show_image').value
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
        if any(self.names.get(ident, '').strip().lower() != name
               for ident, name in SIGNAL_CLASS_IDS.items()):
            raise ValueError('Expected the eight-class model with class 6=red and 7=green')
        model_classes = {name.strip().lower() for name in self.names.values()}
        missing = set(CLASS_THRESHOLDS)-model_classes
        if missing:
            raise ValueError(f'Model is missing configured classes: {sorted(missing)}')
        self.minimum_confidence = min(t['confidence'] for t in CLASS_THRESHOLDS.values())
        # Create/warm the backend before timers start, so the first camera image
        # does not expire while TensorRT initializes. Keep the engine's 640x640 input.
        self.model.predict(source=np.zeros((IMAGE_SIZE, IMAGE_SIZE, 3), np.uint8),
                           imgsz=IMAGE_SIZE, rect=False, conf=self.minimum_confidence,
                           device=self.device, verbose=False, save=False)
        self.state = DetectionState()
        self.presence_values = dict.fromkeys(PRESENCE_TOPICS, False)
        # Replace the pair together: inference must never combine an old image
        # with the receipt time of a new image from the camera callback thread.
        self.latest_frame = (None, 0.)
        self.last_stamp = None
        self.last_processed_time = 0.
        output_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.VOLATILE)
        self.publishers_by_topic = {topic: self.create_publisher(Bool, topic, output_qos)
                                    for topic in (*PRESENCE_TOPICS, LR_TOPIC, SIGNAL_TOPIC)}
        self.bbox_pub = self.create_publisher(CompressedImage, '/yolo/debug/compressed', output_qos)
        self.lr_detail_pub = self.create_publisher(Float32MultiArray, LR_DETAIL_TOPIC, output_qos)
        self.box_details = []
        # Keep only the latest frame. Best Effort accepts the real
        # camera's Reliable publisher and SensorDataQoS publishers alike.
        input_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                               durability=DurabilityPolicy.VOLATILE)
        self.camera_callback_group = MutuallyExclusiveCallbackGroup()
        self.subscription = self.create_subscription(
            Image if self.image_transport == 'raw' else CompressedImage,
            self.get_parameter('image_topic').value, self.on_image, input_qos,
            callback_group=self.camera_callback_group)
        self.camera_started = time.monotonic()
        self.camera_missing = False
        self.last_camera_error = None
        # Detect outages even when simulation time is paused.
        self.watchdog_clock = Clock(clock_type=ClockType.STEADY_TIME)
        # main() pumps the GUI on its own thread; Qt windows must not move
        # between the executor's camera/inference worker threads.
        self.camera_watchdog = self.create_timer(
            0.5, self.check_camera, clock=self.watchdog_clock)
        self.get_logger().info(
            f'Camera input: {self.subscription.topic_name}; transport: {self.image_transport}')
        self.timer = self.create_timer(1./PUBLISH_HZ, self.detect, clock=self.watchdog_clock)
        self.get_logger().info(f'Model: {model_path}; device: {self.device}; output: {PUBLISH_HZ:g} Hz')

    def on_image(self, message):
        received = time.monotonic()
        if self.latest_frame[0] is None or self.camera_missing:
            self.get_logger().info(f'Camera receiving: {self.subscription.topic_name}')
        self.camera_missing = False
        self.last_camera_error = None
        self.latest_frame = (message, received)

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
        message, received = self.latest_frame
        last_received = received if message is not None else self.camera_started
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

    def log_detections(self, detected, values):
        for name, confidence in sorted(detected.items()):
            topic = SIGNAL_TOPIC if name in ('red', 'green') else next(
                (topic for topic, classes in PRESENCE_TOPICS.items() if name in classes), LR_TOPIC)
            if topic not in values:
                continue
            if topic in PRESENCE_TOPICS and not values[topic]:
                continue
            if topic == LR_TOPIC and values[topic] != (name == 'right'):
                continue
            if topic == SIGNAL_TOPIC and values[topic] != (name == 'green'):
                continue
            self.get_logger().info(
                f'Detection: class={name}; output_topic={topic}; '
                f'confidence={confidence:.3f}; published={values[topic]}')

    def classify_detections(self, rows, image):
        """Per-class confidence and original-image pixel-area gates; boxes kept for the views."""
        height, width = image.shape[:2]
        detected, annotations = {}, []
        self.box_details = []
        for row in rows:
            if not np.isfinite(row).all():
                continue
            name = detection_class_name(row[5], self.names)
            if name is None:
                continue
            confidence = float(row[4])
            x0, y0, x1, y1 = row[:4]
            x0, x1 = np.clip((int(x0), int(x1)), 0, width)
            y0, y1 = np.clip((int(y0), int(y1)), 0, height)
            bbox_width = max(0, int(x1 - x0))
            bbox_height = max(0, int(y1 - y0))
            eligible = bool(filter_detections([row], self.names, width, height))
            self.box_details.append({'class': name, 'xyxy': (int(x0), int(y0), int(x1), int(y1)),
                                     'width': bbox_width, 'height': bbox_height,
                                     'area': bbox_width * bbox_height, 'eligible': eligible,
                                     'confidence': confidence})
            annotations.append(((int(x0), int(y0), int(x1), int(y1)), f'{name} YOLO:{confidence:.2f}'))
            if eligible:
                detected[name] = max(detected.get(name, 0.), confidence)
        return detected, annotations

    def publish_lr_detail(self, stamp, values):
        if LR_TOPIC not in values:
            return
        name = 'right' if values[LR_TOPIC] else 'left'
        boxes = [b for b in self.box_details if b['eligible'] and b['class'] == name]
        if not boxes:
            return
        b = max(boxes, key=lambda box: box['area'])
        x0, y0, x1, y1 = b['xyxy']
        self.lr_detail_pub.publish(Float32MultiArray(data=[
            float(name == 'right'), float(b['area']), float(b['width']), float(b['height']),
            float(b['confidence']), float(x0), float(y0), float(x1), float(y1),
            float(stamp.sec), float(stamp.nanosec)]))

    def publish_bbox_view(self, image, header, values):
        if not self.bbox_pub.get_subscription_count():
            return
        try:
            preview = draw_bbox_publications(image, self.box_details, values)
            ok, encoded = cv2.imencode('.jpg', preview, [cv2.IMWRITE_JPEG_QUALITY, 82])
            if ok:
                self.bbox_pub.publish(CompressedImage(
                    header=header, format='bgr8; jpeg compressed bgr8', data=encoded.tobytes()))
        except (cv2.error, ValueError) as error:
            self.get_logger().error(f'YOLO debug image failed: {error}', throttle_duration_sec=5.)

    def render_preview(self, image, annotations):
        preview = image.copy()
        for (x0, y0, x1, y1), label in annotations:
            cv2.rectangle(preview, (x0, y0), (x1, y1), (0, 220, 0), 2)
            cv2.putText(preview, label, (max(0, x0), max(18, y0-6)),
                        cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 220, 0), 2, cv2.LINE_AA)
        self.preview_frame = preview
        self.preview_dirty = True

    def fresh(self, stamp_ns, received):
        age = (self.get_clock().now().nanoseconds-stamp_ns)/1e9
        return -.05 <= age <= MAX_IMAGE_AGE_SEC and time.monotonic()-received <= MAX_IMAGE_AGE_SEC

    def publish_presence(self, clear=False):
        """Repeat presence only; lights/direction require a fresh confirmed image."""
        if clear:
            self.state.reset()
            self.presence_values = dict.fromkeys(PRESENCE_TOPICS, False)
        for topic, value in self.presence_values.items():
            self.publishers_by_topic[topic].publish(Bool(data=value))

    def detect(self):
        message, received = self.latest_frame
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
            result = self.model.predict(source=image, imgsz=IMAGE_SIZE, rect=False, conf=self.minimum_confidence,
                                        device=self.device, verbose=False, save=False)[0]
            rows = result.boxes.data.cpu().numpy() if result.boxes is not None else []
            detected, annotations = self.classify_detections(rows, image)
            if self.show_image:
                self.render_preview(image, annotations)
        except (CvBridgeError, cv2.error, ValueError, RuntimeError) as exc:
            self.publish_presence(clear=True)
            self.get_logger().error(f'YOLO frame failed: {exc}', throttle_duration_sec=5.)
            return
        # An expired acquisition clears presence and never emits a direction.
        if not self.fresh(stamp_ns, received):
            self.publish_bbox_view(image, message.header, {})
            self.publish_presence(clear=True)
            return
        self.last_processed_time = time.monotonic()
        values = self.state.update(detected)
        self.presence_values = {topic: values[topic] for topic in PRESENCE_TOPICS}
        for topic, value in values.items():
            self.publishers_by_topic[topic].publish(Bool(data=value))
        self.publish_lr_detail(message.header.stamp, values)
        self.publish_bbox_view(image, message.header, values)
        self.log_detections(detected, values)
        elapsed = time.monotonic()-started
        if elapsed > 1./PUBLISH_HZ:
            self.get_logger().warning(
                f'Inference took {elapsed:.3f}s; cannot sustain {PUBLISH_HZ:g} Hz on this device.',
                throttle_duration_sec=5.)


def main(args=None):
    rclpy.init(args=args)
    node, executor = None, None
    try:
        node = YoloDetect()
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.02)
            node.update_preview()
    except KeyboardInterrupt:
        pass
    finally:
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
