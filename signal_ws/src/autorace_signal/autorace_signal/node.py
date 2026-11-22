"""Stop after a white floor mark; release on green or a fixed red deadline."""
# 시간 설정 (초) — 이 값을 수정한 뒤 빌드하고 노드를 다시 실행하세요.
STOP_DELAY_SEC = 2.0        # 빨간불 확인·흰선 감지 후 정지 시작까지 주행
SIGNAL_GAP_TIMEOUT_SEC = 0.5  # 같은 신호 확인 횟수를 초기화하는 수신 간격
LANE_TIMEOUT_SEC = 0.25
FUTURE_TOLERANCE_SEC = 0.05

CONFIRM_FRAMES = 2
PUBLISH_RATE_HZ = 30.0


import copy
import math
import time
from pathlib import Path
from dataclasses import dataclass

import cv2
import numpy as np
import yaml

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rcl_interfaces.srv import GetParameters
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import Bool, Float32
from sensor_msgs.msg import CompressedImage

from .core import SignalState


# Keep the floor detector in this module; no separate untracked helper is required.
def load_lane_white(path):
    config = yaml.safe_load(Path(path).read_text())['lane_lite']
    return validate_white(config['white_s_max'], config['white_v_min'])


def validate_white(saturation_max, value_min):
    values = (saturation_max, value_min)
    if any(isinstance(v, bool) or not isinstance(v, (int, float))
           or not math.isfinite(v) or not 0 <= v <= 255 for v in values):
        raise ValueError('Lane white HSV thresholds must be finite values from 0 to 255')
    return tuple(int(v) for v in values)


@dataclass(frozen=True)
class WhiteROI:
    left: float = .05
    right: float = .95
    top: float = .961
    bottom: float = 1.0
    minimum_ratio: float = .40

    def __post_init__(self):
        if (not all(math.isfinite(v) for v in (self.left, self.right, self.top, self.bottom, self.minimum_ratio))
                or not 0 <= self.left < self.right <= 1
                or not 0 <= self.top < self.bottom <= 1
                or not 0 < self.minimum_ratio <= 1):
            raise ValueError('Invalid normalized white ROI bounds or pixel ratio')

    def measure(self, image, saturation_max, value_min):
        height, width = image.shape[:2]
        x0, x1 = int(width * self.left), min(width, math.ceil(width * self.right))
        y0, y1 = int(height * self.top), min(height, math.ceil(height * self.bottom))
        crop = image[y0:y1, x0:x1]
        if crop.size == 0:
            raise ValueError('White ROI contains no pixels')
        # Match the lane colour threshold, before its lane-shape rejection filters.
        hsv = cv2.cvtColor(cv2.GaussianBlur(crop, (5, 5), 0), cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (0, 0, value_min), (179, saturation_max, 255))
        ratio = float(np.count_nonzero(mask)) / mask.size
        return ratio, (x0, y0, x1, y1), mask

    def preview(self, image, bounds, ratio, mask, armed):
        output = image.copy()
        x0, y0, x1, y1 = bounds
        positive = ratio >= self.minimum_ratio
        colour = (90, 220, 110) if positive else (60, 190, 245)
        patch = output[y0:y1, x0:x1]
        tint = patch.copy()
        tint[mask > 0] = (90, 220, 110)
        output[y0:y1, x0:x1] = cv2.addWeighted(patch, .65, tint, .35, 0)
        cv2.rectangle(output, (x0, y0), (x1 - 1, y1 - 1), colour, 2)
        text = f'WHITE {ratio:.1%} / {self.minimum_ratio:.1%} | armed: {", ".join(armed) or "none"}'
        cv2.rectangle(output, (0, 0), (output.shape[1], 27), (20, 25, 25), -1)
        cv2.putText(output, text, (8, 18), cv2.FONT_HERSHEY_SIMPLEX, .42, colour, 1, cv2.LINE_AA)
        return output



class SignalNode(Node):
    def __init__(self, **kwargs):
        super().__init__('signal_node', **kwargs)
        defaults = dict(confirm_frames=CONFIRM_FRAMES,
                        signal_gap_timeout_sec=SIGNAL_GAP_TIMEOUT_SEC,
                        publish_rate_hz=PUBLISH_RATE_HZ, lane_timeout_sec=LANE_TIMEOUT_SEC,
                        future_tolerance_sec=FUTURE_TOLERANCE_SEC, stop_delay_sec=STOP_DELAY_SEC)
        for key, default in defaults.items():
            value = self.declare_parameter(key, default, ParameterDescriptor(read_only=True)).value
            if (isinstance(value, bool) or not isinstance(value, (int, float))
                    or not math.isfinite(value) or value < 0
                    or (key not in ('future_tolerance_sec', 'stop_delay_sec') and value == 0)
                    or (key == 'confirm_frames' and not isinstance(value, int))):
                raise ValueError(f'Invalid {key}: {value}')
            setattr(self, key, value)
        descriptor = ParameterDescriptor(read_only=True)
        self.white_roi_enabled = self.declare_parameter('white_roi_enabled', True, descriptor).value
        if not isinstance(self.white_roi_enabled, bool):
            raise ValueError('white_roi_enabled must be a Bool')
        self.white_roi = WhiteROI(**{
            name: float(self.declare_parameter('white_roi_' + name, default, descriptor).value)
            for name, default in (('left', .05), ('right', .95), ('top', .961), ('bottom', 1.0), ('minimum_ratio', .40))
        })
        self.white_image_timeout = float(self.declare_parameter('white_image_timeout_sec', .3, descriptor).value)
        self.white_image_hz = float(self.declare_parameter('white_image_hz', 10., descriptor).value)
        if (not math.isfinite(self.white_image_timeout) or self.white_image_timeout <= 0
                or not math.isfinite(self.white_image_hz) or self.white_image_hz <= 0):
            raise ValueError('White image timeout and processing rate must be finite and positive')
        self.last_white_stamp = None
        self.last_white_wall = -math.inf
        self.white_ratio = None
        self.white_arm_stamp = None
        self.state = SignalState(self.confirm_frames, self.signal_gap_timeout_sec,
                                 require_white_mark=self.white_roi_enabled, stop_delay_sec=self.stop_delay_sec)
        self.lane = None
        self.lane_received = None
        self.last_ros_ns = None
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        self.command_pub = self.create_publisher(AckermannDriveStamped, '/mission/signal/command_02', qos)
        self.start_signal_pub = self.create_publisher(Bool, '/start_signal', qos)
        self.reported_stop_generation = 0
        self.signal_sub = self.create_subscription(Bool, '/yolo/signal', self.on_signal, qos)
        self.lane_sub = self.create_subscription(AckermannDriveStamped, '/mission/lane', self.on_lane, qos)
        self.white_query = None
        self.white_client = None
        if self.white_roi_enabled:
            config_file = self.declare_parameter('lane_white_config', '', descriptor).value
            if not config_file:
                try:
                    config_file = str(Path(get_package_share_directory('autorace_lane_deep_lite')) / 'config/real/mission_vision.yaml')
                except PackageNotFoundError:
                    config_file = next((str(parent / 'lane_fusion_ws/src/autorace_lane_deep_lite/config/real/mission_vision.yaml')
                                        for parent in Path(__file__).resolve().parents
                                        if (parent / 'lane_fusion_ws/src/autorace_lane_deep_lite/config/real/mission_vision.yaml').is_file()), '')
            if not config_file:
                raise ValueError('Cannot find the lane white HSV config; set lane_white_config')
            self.white_s_max, self.white_v_min = load_lane_white(config_file)
            driver = self.declare_parameter('lane_white_node', '', descriptor).value
            if driver:
                self.white_client = self.create_client(GetParameters, driver.rstrip('/') + '/get_parameters')
            camera = self.declare_parameter('white_image_topic', '/camera/color/image_raw/compressed', descriptor).value
            self.white_sub = self.create_subscription(CompressedImage, camera, self.on_white_image, qos_profile_sensor_data)
            self.white_debug_pub = self.create_publisher(CompressedImage, '/signal/white_roi/debug/compressed', qos_profile_sensor_data)
            self.white_ratio_pub = self.create_publisher(Float32, '/signal/white_roi/ratio', 1)
            self.white_detected_pub = self.create_publisher(Bool, '/signal/white_roi/detected', 1)
            if self.white_client is not None:
                self.white_timer = self.create_timer(1., self.poll_lane_white)
            stop_timing='즉시 정지' if self.stop_delay_sec==0 else f'{self.stop_delay_sec:.1f}초 뒤 정지'
            self.get_logger().info(
                f'신호등: 빨간불 {self.confirm_frames}회와 새 흰선 확인 시 {stop_timing}; '
                f'ROI x={self.white_roi.left:.2f}..{self.white_roi.right:.2f}, '
                f'y={self.white_roi.top:.2f}..{self.white_roi.bottom:.2f}, '
                f'white >= {self.white_roi.minimum_ratio:.0%}; '
                f'lane HSV S<={self.white_s_max}, V>={self.white_v_min}')
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(1/self.publish_rate_hz, self.publish, clock=self.steady_clock)
        self.get_logger().info('신호등: 초록불 확인 시 해제')

    def ros_now(self):
        now = self.get_clock().now().nanoseconds
        if self.last_ros_ns is not None and now < self.last_ros_ns:
            self.lane = None
            self.last_white_stamp = None
            self.white_arm_stamp = now if self.state.armed else None
        self.last_ros_ns = now
        return now

    @staticmethod
    def stamp_ns(message):
        return message.header.stamp.sec*1_000_000_000+message.header.stamp.nanosec

    def stamp_valid(self, message, ros_now):
        stamp = message.header.stamp
        age = (ros_now-self.stamp_ns(message))/1e9
        return (stamp.sec >= 0 and 0 <= stamp.nanosec < 1_000_000_000
                and self.stamp_ns(message) > 0
                and -self.future_tolerance_sec <= age < self.lane_timeout_sec)

    def on_lane(self, message):
        now = self.ros_now()
        drive = message.drive
        if (not all(math.isfinite(v) for v in (drive.speed, drive.steering_angle,
                drive.steering_angle_velocity, drive.acceleration, drive.jerk)) or drive.speed < 0):
            self.lane = None
            return
        if self.lane is not None and 0 < self.stamp_ns(message) < self.stamp_ns(self.lane):
            return  # An old packet cannot erase or refresh a newer lane command.
        if not self.stamp_valid(message, now):
            self.lane = None
            return
        self.lane, self.lane_received = copy.deepcopy(message), time.monotonic()

    def log_transition(self, was_stopping):
        if was_stopping != self.state.stopping:
            if self.state.stopping:
                self.get_logger().info('신호등: 빨간불 정지')
            else:
                self.get_logger().info('신호등: 초록불 확인, 차선 추종 복귀')

    def on_signal(self, message):
        was_stopping, was_armed = self.state.stopping, self.state.armed
        self.state.receive(message.data, time.monotonic())
        if self.state.armed and not was_armed:
            self.white_arm_stamp = self.get_clock().now().nanoseconds
            self.get_logger().info('신호등: 빨간불 확인, 흰선 대기')
        elif not self.state.armed:
            self.white_arm_stamp = None
        self.log_transition(was_stopping)

    def poll_lane_white(self):
        if self.white_client is None:
            return
        if self.white_query is not None:
            future, started = self.white_query
            if future.done():
                try:
                    values = future.result().values
                    if len(values) == 2 and all(v.type in (2, 3) for v in values):
                        thresholds = validate_white(*(v.integer_value if v.type == 2 else v.double_value for v in values))
                        if thresholds != (self.white_s_max, self.white_v_min):
                            self.white_s_max, self.white_v_min = thresholds
                            self.get_logger().info(f'Lane white HSV updated: S<={thresholds[0]}, V>={thresholds[1]}')
                except (ValueError, AttributeError, RuntimeError) as error:
                    self.get_logger().warning(f'Cannot update lane white HSV: {error}', throttle_duration_sec=5.)
                self.white_query = None
            elif time.monotonic() - started > 2:
                future.cancel()
                self.white_query = None
        if self.white_query is None and self.white_client.service_is_ready():
            future = self.white_client.call_async(GetParameters.Request(names=['white_s_max', 'white_v_min']))
            self.white_query = future, time.monotonic()

    def on_white_image(self, message):
        wall, now = time.monotonic(), self.get_clock().now().nanoseconds
        stamp = self.stamp_ns(message)
        if (stamp <= 0 or (self.last_white_stamp is not None and stamp <= self.last_white_stamp)
                or not -self.future_tolerance_sec <= (now - stamp) / 1e9 <= self.white_image_timeout
                or wall - self.last_white_wall < 1. / self.white_image_hz):
            return
        self.last_white_stamp, self.last_white_wall = stamp, wall
        try:
            image = cv2.imdecode(np.frombuffer(message.data, dtype=np.uint8), cv2.IMREAD_COLOR)
            if image is None:
                raise ValueError('Cannot decode camera JPEG')
            ratio, bounds, mask = self.white_roi.measure(image, self.white_s_max, self.white_v_min)
            if (self.get_clock().now().nanoseconds - stamp) / 1e9 > self.white_image_timeout:
                return
            self.white_ratio = ratio
            hit = ratio >= self.white_roi.minimum_ratio
            armed = ['red'] if self.state.armed else []
            if (self.white_arm_stamp is not None and stamp >= self.white_arm_stamp
                    and self.state.receive_white(hit, time.monotonic())):
                self.white_arm_stamp = None
                stop_timing='즉시 정지' if self.stop_delay_sec==0 else f'{self.stop_delay_sec:.1f}초 뒤 정지'
                self.get_logger().info(f'신호등: 흰선 확인, {stop_timing}')
            self.white_ratio_pub.publish(Float32(data=ratio))
            self.white_detected_pub.publish(Bool(data=hit))
            if self.white_debug_pub.get_subscription_count():
                preview = self.white_roi.preview(image, bounds, ratio, mask, armed)
                ok, encoded = cv2.imencode('.jpg', preview, [cv2.IMWRITE_JPEG_QUALITY, 80])
                if ok:
                    self.white_debug_pub.publish(CompressedImage(
                        header=message.header, format='bgr8; jpeg compressed bgr8', data=encoded.tobytes()))
        except (ValueError, cv2.error) as error:
            self.get_logger().warning(f'White ROI image rejected: {error}', throttle_duration_sec=5.)

    def publish(self):
        now, ros_now = time.monotonic(), self.ros_now()
        was_stopping = self.state.stopping
        self.state.tick(now)
        if not self.state.armed:
            self.white_arm_stamp = None
        self.log_transition(was_stopping)
        if not self.state.stopping:
            return
        if (self.lane is not None and 0 <= now-self.lane_received < self.lane_timeout_sec
                and self.stamp_valid(self.lane, ros_now)):
            message = copy.deepcopy(self.lane)
        else:
            self.lane = None
            message = AckermannDriveStamped()
            message.header.frame_id = 'base_link'
        message.drive.speed = 0.0
        message.header.stamp = self.get_clock().now().to_msg()
        self.command_pub.publish(message)
        # Announce each red-stop episode only after its first stop command.
        if self.reported_stop_generation != self.state.stop_generation:
            self.start_signal_pub.publish(Bool(data=True))
            self.reported_stop_generation = self.state.stop_generation


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SignalNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
