"""Two signs and a new white floor image trigger a stop with a cooldown."""
# 시간 설정 (초) — 이 값을 수정한 뒤 빌드하고 노드를 다시 실행하세요.
STOP_DELAY_SEC = 1.5    # 흰선 감지 후 정지 시작까지 주행
STOP_HOLD_SEC = 3.0     # 실제 정지 시작 후 정지 유지
COOLDOWN_SEC = 10.0     # 정지 종료 후 다음 표지판을 받기까지의 간격


import copy
import math
from pathlib import Path
import time

import cv2
import numpy as np
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.srv import GetParameters
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, Float32

from .core import CrosswalkState
from .white_roi import WhiteROI, load_lane_white, validate_white


class CrosswalkNode(Node):
    def __init__(self, **kwargs):
        super().__init__('crosswalk_node', **kwargs)
        self.lane = None
        descriptor = ParameterDescriptor(read_only=True)
        for name, default in (('stop_delay_sec', STOP_DELAY_SEC),
                              ('stop_hold_sec', STOP_HOLD_SEC),
                              ('cooldown_sec', COOLDOWN_SEC)):
            value = self.declare_parameter(name, default, descriptor).value
            if (isinstance(value, bool) or not isinstance(value, (int, float))
                    or not math.isfinite(value) or value < 0
                    or (name == 'stop_hold_sec' and value == 0)):
                raise ValueError(f'Invalid {name}: {value}')
            setattr(self, name, float(value))
        self.state = CrosswalkState(stop_delay_sec=self.stop_delay_sec,
                                    stop_hold_sec=self.stop_hold_sec, cooldown_sec=self.cooldown_sec)
        self.white_roi = WhiteROI(**{
            name: float(self.declare_parameter('white_roi_' + name, default, descriptor).value)
            for name, default in (('left', .05), ('right', .95), ('top', .961),
                                  ('bottom', 1.0), ('minimum_ratio', .40))
        })
        self.white_image_timeout = float(self.declare_parameter(
            'white_image_timeout_sec', .3, descriptor).value)
        self.white_image_hz = float(self.declare_parameter('white_image_hz', 10., descriptor).value)
        if (not math.isfinite(self.white_image_timeout) or self.white_image_timeout <= 0
                or not math.isfinite(self.white_image_hz) or self.white_image_hz <= 0):
            raise ValueError('White image timeout and processing rate must be finite and positive')
        config_file = self.declare_parameter('lane_white_config', '', descriptor).value
        if not config_file:
            try:
                config_file = str(Path(get_package_share_directory('autorace_lane_deep_lite'))
                                  / 'config/real/mission_vision.yaml')
            except PackageNotFoundError:
                config_file = next((str(parent / 'lane_fusion_ws/src/autorace_lane_deep_lite/config/real/mission_vision.yaml')
                                    for parent in Path(__file__).resolve().parents
                                    if (parent / 'lane_fusion_ws/src/autorace_lane_deep_lite/config/real/mission_vision.yaml').is_file()), '')
        if not config_file:
            raise ValueError('Cannot find lane white HSV config; set lane_white_config')
        self.white_s_max, self.white_v_min = load_lane_white(config_file)
        self.last_white_stamp = None
        self.last_white_wall = -math.inf
        self.white_arm_stamp = None
        self.last_ros_ns = None
        self.white_query = None
        driver = self.declare_parameter('lane_white_node', '', descriptor).value
        self.white_client = (self.create_client(GetParameters, driver.rstrip('/') + '/get_parameters')
                             if driver else None)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        self.command_pub = self.create_publisher(
            AckermannDriveStamped, '/mission/crosswalk/command_01', qos)
        self.start_crosswalk_pub = self.create_publisher(Bool, '/start_crosswalk', qos)
        self.sign_sub = self.create_subscription(
            Bool, '/yolo/crosswalk_sign', self.on_sign, qos)
        self.lane_sub = self.create_subscription(
            AckermannDriveStamped, '/mission/lane', self.on_lane, qos)
        camera = self.declare_parameter(
            'white_image_topic', '/camera/color/image_raw/compressed', descriptor).value
        self.white_sub = self.create_subscription(
            CompressedImage, camera, self.on_white_image, qos_profile_sensor_data)
        self.white_debug_pub = self.create_publisher(
            CompressedImage, '/crosswalk/white_roi/debug/compressed', qos_profile_sensor_data)
        self.white_ratio_pub = self.create_publisher(Float32, '/crosswalk/white_roi/ratio', 1)
        self.white_detected_pub = self.create_publisher(Bool, '/crosswalk/white_roi/detected', 1)
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(1.0 / 30.0, self.publish, clock=self.steady_clock)
        self.white_timer = (self.create_timer(1., self.poll_lane_white, clock=self.steady_clock)
                            if self.white_client is not None else None)
        self.get_logger().info(f'횡단보도: 표지판 2회·흰선 확인 후 {self.stop_delay_sec:.1f}초 뒤 정지')

    def ros_now(self):
        now = self.get_clock().now().nanoseconds
        if self.last_ros_ns is not None and now < self.last_ros_ns:
            self.last_white_stamp = None
            if self.state.phase == 'waiting_white':
                self.white_arm_stamp = now
        self.last_ros_ns = now
        return now

    def on_lane(self, message):
        self.lane = copy.deepcopy(message)

    def on_sign(self, message):
        # Apply the cooldown boundary before counting this new message.
        self.publish()
        previous = self.state.phase
        self.state.receive(message.data)
        if previous == 'waiting' and self.state.phase == 'waiting_white':
            self.white_arm_stamp = self.ros_now()
            self.get_logger().info('Crosswalk: sign confirmed, waiting for a new white floor image')
        self.publish()

    def poll_lane_white(self):
        if self.white_client is None:
            return
        if self.white_query is not None:
            future, started = self.white_query
            if future.done():
                try:
                    values = future.result().values
                    self.white_s_max, self.white_v_min = validate_white(*(
                        v.integer_value if v.type == 2 else v.double_value for v in values))
                except (ValueError, TypeError, AttributeError) as error:
                    self.get_logger().warning(f'Cannot update lane white HSV: {error}', throttle_duration_sec=5.)
                self.white_query = None
            elif time.monotonic() - started > 2.:
                self.white_client.remove_pending_request(future)
                self.white_query = None
        if self.white_query is None and self.white_client.service_is_ready():
            future = self.white_client.call_async(
                GetParameters.Request(names=['white_s_max', 'white_v_min']))
            self.white_query = future, time.monotonic()

    def on_white_image(self, message):
        wall, now = time.monotonic(), self.ros_now()
        stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        if (stamp <= 0 or (self.last_white_stamp is not None and stamp <= self.last_white_stamp)
                or not -.05 <= (now - stamp) / 1e9 <= self.white_image_timeout
                or wall - self.last_white_wall < 1. / self.white_image_hz):
            return
        self.last_white_stamp, self.last_white_wall = stamp, wall
        try:
            image = cv2.imdecode(np.frombuffer(message.data, dtype=np.uint8), cv2.IMREAD_COLOR)
            if image is None:
                raise ValueError('Cannot decode camera image')
            ratio, bounds, mask = self.white_roi.measure(image, self.white_s_max, self.white_v_min)
            if (self.ros_now() - stamp) / 1e9 > self.white_image_timeout:
                return
            hit = ratio >= self.white_roi.minimum_ratio
            armed = self.state.phase == 'waiting_white'
            if armed and self.white_arm_stamp is not None and stamp >= self.white_arm_stamp:
                if self.state.receive_white(hit, time.monotonic()):
                    self.white_arm_stamp = None
                    self.get_logger().info(f'횡단보도: 흰선 확인, {self.stop_delay_sec:.1f}초 뒤 정지')
                self.publish()
            self.white_ratio_pub.publish(Float32(data=ratio))
            self.white_detected_pub.publish(Bool(data=hit))
            if self.white_debug_pub.get_subscription_count():
                preview = self.white_roi.preview(image, bounds, ratio, mask,
                                                 ['crosswalk'] if armed else [])
                ok, encoded = cv2.imencode('.jpg', preview, [cv2.IMWRITE_JPEG_QUALITY, 80])
                if ok:
                    self.white_debug_pub.publish(CompressedImage(
                        header=message.header, format='jpeg', data=encoded.tobytes()))
        except (ValueError, cv2.error) as error:
            self.get_logger().warning(f'White ROI image rejected: {error}', throttle_duration_sec=5.)

    def publish(self):
        self.ros_now()
        previous = self.state.phase
        stopping = self.state.tick(time.monotonic(), self.lane is not None)
        if previous != self.state.phase:
            if self.state.phase == 'stopping':
                self.start_crosswalk_pub.publish(Bool(data=True))
            if self.state.phase == 'waiting':
                self.white_arm_stamp = None
            self.get_logger().info(f'Crosswalk: {self.state.phase}')
        if not stopping or self.lane is None:
            return
        message = copy.deepcopy(self.lane)
        message.drive.speed = 0.0
        message.header.stamp = self.get_clock().now().to_msg()
        self.command_pub.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CrosswalkNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
