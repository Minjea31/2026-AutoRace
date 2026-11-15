"""Two sign inputs, fresh lane commands and one slow mission output."""
import copy
from functools import partial
import math
import signal
from threading import Event
import time
from pathlib import Path

import cv2
import numpy as np

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.srv import GetParameters
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool, Float32
from sensor_msgs.msg import CompressedImage

from .core import SlowState
from .white_roi import WhiteROI, load_lane_white, validate_white


class SlowNode(Node):
    def __init__(self, **kwargs):
        super().__init__('slow_node', **kwargs)
        defaults = dict(confirm_frames=2, hold_sec=8., slow_hold_sec=15., signal_gap_timeout_sec=.5,
                        speed_mps=.2, publish_rate_hz=20.,
                        lane_timeout_sec=.25, future_tolerance_sec=.05)
        for name, default in defaults.items():
            value = self.declare_parameter(name, default, ParameterDescriptor(read_only=True)).value
            if (isinstance(value, bool) or not isinstance(value, (int, float))
                    or not math.isfinite(value) or value < 0
                    or (name != 'future_tolerance_sec' and value == 0)
                    or (name == 'confirm_frames' and not isinstance(value, int))):
                raise ValueError(f'Invalid {name}: {value}')
            setattr(self, name, value)
        descriptor = ParameterDescriptor(read_only=True)
        self.white_roi_enabled = self.declare_parameter('white_roi_enabled', True, descriptor).value
        if not isinstance(self.white_roi_enabled, bool):
            raise ValueError('white_roi_enabled must be a Bool')
        self.white_roi = WhiteROI(**{
            name: float(self.declare_parameter('white_roi_' + name, default, descriptor).value)
            for name, default in (('left', .05), ('right', .95), ('top', .961), ('bottom', 1.0), ('minimum_ratio', .08))
        })
        self.white_image_timeout = float(self.declare_parameter('white_image_timeout_sec', .3, descriptor).value)
        self.white_image_hz = float(self.declare_parameter('white_image_hz', 10., descriptor).value)
        if (not math.isfinite(self.white_image_timeout) or self.white_image_timeout <= 0
                or not math.isfinite(self.white_image_hz) or self.white_image_hz <= 0):
            raise ValueError('White image timeout and processing rate must be finite and positive')
        self.last_white_stamp = None
        self.last_white_wall = -math.inf
        self.white_ratio = None
        self.white_arm_stamps = {}
        self.state = SlowState(self.confirm_frames, self.hold_sec, self.signal_gap_timeout_sec,
                               require_white_mark=self.white_roi_enabled, slow_hold_sec=self.slow_hold_sec)
        self.lane = self.lane_received = self.last_ros_ns = None
        self.last_mode = None
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        self.command_pub = self.create_publisher(AckermannDriveStamped, '/mission/slow/command_08', qos)
        self.lane_sub = self.create_subscription(AckermannDriveStamped, '/mission/lane', self.on_lane, qos)
        self.sign_subs = [self.create_subscription(Bool, topic, partial(self.on_sign, kind), qos)
                          for kind, topic in (('slow', '/yolo/slow_sign'), ('bump', '/yolo/bump_sign'))]
        self.crosswalk_sub = self.create_subscription(Bool, '/start_crosswalk', self.on_crosswalk, qos)
        self.signal_stop_sub = self.create_subscription(Bool, '/start_signal', self.on_signal_stop, qos)
        self.white_query = None
        self.white_client = None
        if self.white_roi_enabled:
            config_file = self.declare_parameter('lane_white_config', '', descriptor).value
            if not config_file:
                try:
                    config_file = str(Path(get_package_share_directory('autorace_lane_fusion')) / 'config/real/lane.yaml')
                except PackageNotFoundError:
                    config_file = next((str(parent / 'lane_fusion_ws/src/autorace_lane_fusion/config/real/lane.yaml')
                                        for parent in Path(__file__).resolve().parents
                                        if (parent / 'lane_fusion_ws/src/autorace_lane_fusion/config/real/lane.yaml').is_file()), '')
            if not config_file:
                raise ValueError('Cannot find the lane white HSV config; set lane_white_config')
            self.white_s_max, self.white_v_min = load_lane_white(config_file)
            driver = self.declare_parameter('lane_white_node', '/lane_fusion_node', descriptor).value
            self.white_client = self.create_client(GetParameters, driver.rstrip('/') + '/get_parameters')
            camera = self.declare_parameter('white_image_topic', '/camera/color/image_raw/compressed', descriptor).value
            self.white_sub = self.create_subscription(CompressedImage, camera, self.on_white_image, qos_profile_sensor_data)
            self.white_debug_pub = self.create_publisher(CompressedImage, '/slow/white_roi/debug/compressed', qos_profile_sensor_data)
            self.white_ratio_pub = self.create_publisher(Float32, '/slow/white_roi/ratio', 1)
            self.white_detected_pub = self.create_publisher(Bool, '/slow/white_roi/detected', 1)
            self.white_timer = self.create_timer(1., self.poll_lane_white)
            self.get_logger().info(
                f'Slow/bump floor gate: YOLO {self.confirm_frames}회 후 새 ROI 1회; '
                f'ROI x={self.white_roi.left:.2f}..{self.white_roi.right:.2f}, '
                f'y={self.white_roi.top:.2f}..{self.white_roi.bottom:.2f}, '
                f'white >= {self.white_roi.minimum_ratio:.0%}; '
                f'lane HSV S<={self.white_s_max}, V>={self.white_v_min}')
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(1/self.publish_rate_hz, self.publish, clock=self.steady_clock)
        self.get_logger().info(f'Slow ready: confirm={self.confirm_frames}, speed={self.speed_mps} m/s, '
                               f'renewable hold={self.hold_sec}s (steady time)')

    @staticmethod
    def stamp_ns(message):
        return message.header.stamp.sec*1_000_000_000+message.header.stamp.nanosec

    def ros_now(self):
        now = self.get_clock().now().nanoseconds
        if self.last_ros_ns is not None and now < self.last_ros_ns:
            self.lane = self.lane_received = None
            self.last_white_stamp = None
            self.white_arm_stamps = {kind: now for kind, mission in self.state.missions.items() if mission.armed}
        self.last_ros_ns = now
        return now

    def stamp_valid(self, message, ros_now):
        stamp = message.header.stamp
        age = (ros_now-self.stamp_ns(message))/1e9
        return (stamp.sec >= 0 and 0 <= stamp.nanosec < 1_000_000_000
                and self.stamp_ns(message) > 0
                and -self.future_tolerance_sec <= age < self.lane_timeout_sec)

    def on_lane(self, message):
        ros_now, now = self.ros_now(), time.monotonic()
        if self.lane is not None and 0 < self.stamp_ns(message) < self.stamp_ns(self.lane):
            return
        drive = message.drive
        if (not self.stamp_valid(message, ros_now) or drive.speed < 0
                or not all(math.isfinite(value) for value in (drive.speed, drive.steering_angle,
                    drive.steering_angle_velocity, drive.acceleration, drive.jerk))):
            self.lane = self.lane_received = None
            return
        self.lane, self.lane_received = copy.deepcopy(message), now

    def stop_slow(self, reason):
        mission = self.state.missions['slow']
        was_pending = mission.deadline is not None or mission.armed or mission.count > 0
        self.state.stop_slow()
        self.white_arm_stamps.pop('slow', None)
        if was_pending:
            self.get_logger().info(f'서행 종료: {reason}')

    def on_crosswalk(self, message):
        if message.data:
            self.stop_slow('횡단보도 시작')

    def on_signal_stop(self, message):
        if message.data:
            self.stop_slow('빨간불 정지')

    def on_sign(self, kind, message):
        self.state.tick(time.monotonic())
        was_armed = self.state.missions[kind].armed
        self.state.receive(kind, message.data, time.monotonic())
        if not was_armed and self.state.missions[kind].armed:
            self.white_arm_stamps[kind] = self.get_clock().now().nanoseconds
            self.get_logger().info(f'{kind.upper()}: YOLO {self.confirm_frames}회 확인, 바닥 흰색 ROI 대기')

    def poll_lane_white(self):
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
            armed = [kind for kind, mission in self.state.missions.items() if mission.armed]
            eligible = [kind for kind in armed if stamp >= self.white_arm_stamps.get(kind, stamp)]
            for kind in self.state.receive_white(hit, time.monotonic(), eligible):
                self.white_arm_stamps.pop(kind, None)
                self.get_logger().info(f'{kind.upper()}: 바닥 흰색 {ratio:.1%} >= {self.white_roi.minimum_ratio:.1%}, 미션 시작')
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
        mode = self.state.selected(now)
        if mode != self.last_mode:
            self.get_logger().info(f'Slow: {mode.upper() if mode else "PASS"}')
            self.last_mode = mode
        if mode is None:
            return
        fresh = (self.lane is not None and 0 <= now-self.lane_received < self.lane_timeout_sec
                 and self.stamp_valid(self.lane, ros_now))
        if fresh:
            message = copy.deepcopy(self.lane)
            message.drive.speed = self.speed_mps
        else:
            message = AckermannDriveStamped()
            message.header.frame_id = 'base_link'
        message.header.stamp = self.get_clock().now().to_msg()
        self.command_pub.publish(message)


def main(args=None):
    # ros2 launch can forward a second SIGINT while the node is cleaning up.
    stopped = Event()
    previous_handlers = {}
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        for signum in (signal.SIGINT, signal.SIGTERM):
            previous_handlers[signum] = signal.signal(signum, lambda *_: stopped.set())
        node = SlowNode()
        while rclpy.ok() and not stopped.is_set():
            rclpy.spin_once(node, timeout_sec=.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for signum, previous in previous_handlers.items():
            signal.signal(signum, previous)
