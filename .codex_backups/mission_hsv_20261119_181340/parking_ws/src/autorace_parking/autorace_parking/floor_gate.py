"""Confirm the parking sign, then require a new camera observation of white floor."""
import math
from pathlib import Path
import time

import cv2
import numpy as np
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.srv import GetParameters
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CompressedImage

from .white_roi import WhiteROI, load_lane_white, validate_white
from .core import Activation


class ParkingFloorGate:
    def __init__(self, node, start):
        self.node, self.start = node, start
        descriptor = ParameterDescriptor(read_only=True)
        def parameter(name, default):
            return node.declare_parameter(name, default, descriptor).value
        self.roi = WhiteROI(**{
            name: float(parameter('white_roi_' + name, default))
            for name, default in (('left', .05), ('right', .95), ('top', .961),
                                  ('bottom', 1.0), ('minimum_ratio', .08))})
        self.timeout = float(parameter('white_image_timeout_sec', .3))
        self.hz = float(parameter('white_image_hz', 10.))
        self.approach_speed = float(parameter('white_approach_speed', .2))
        self.approach_timeout = float(parameter('white_approach_timeout_sec', 20.))
        self.approach_max_distance = float(parameter('white_approach_max_distance', 3.))
        if (not all(math.isfinite(v) and v>0 for v in
                    (self.approach_speed,self.approach_timeout,self.approach_max_distance))
                or self.approach_speed>.2):
            raise ValueError('White-line approach requires a speed <= 0.2 m/s and positive limits')
        if not all(math.isfinite(v) and v > 0 for v in (self.timeout, self.hz)):
            raise ValueError('White image timeout and rate must be finite and positive')
        config = parameter('lane_white_config', '')
        if not config:
            try:
                config = str(Path(get_package_share_directory('autorace_lane_fusion')) / 'config/real/lane.yaml')
            except PackageNotFoundError:
                config = next((str(p / 'lane_fusion_ws/src/autorace_lane_fusion/config/real/lane.yaml')
                               for p in Path(__file__).resolve().parents
                               if (p / 'lane_fusion_ws/src/autorace_lane_fusion/config/real/lane.yaml').is_file()), '')
        if not config:
            raise ValueError('Cannot find lane white HSV config; set lane_white_config')
        self.s_max, self.v_min = load_lane_white(config)
        self.line_frames = int(parameter('white_line_confirmations',1))
        self.line_min_span = float(parameter('white_line_min_span',0.))
        self.line_max_angle = float(parameter('white_line_max_angle',.44))
        if not 1 <= self.line_frames <= 5 or not 0 <= self.line_min_span <= 1 or not 0 < self.line_max_angle <= math.pi/2:
            raise ValueError('Invalid parking start line confirmation settings')
        self.line_count = 0
        self.line_stamp = None
        self.confirmation = Activation(node.required_detections, node.sign_timeout)
        self.armed_stamp = None
        self.armed_wall = None
        self.approach_distance = 0.
        self.last_wheel = None
        self.failure = ''
        self.ratio = None
        self.last_stamp = None
        self.last_wall = -math.inf
        self.last_ros = None
        self.query = None
        driver = parameter('lane_white_node', '/lane_fusion_node')
        self.client = node.create_client(GetParameters, driver.rstrip('/') + '/get_parameters')
        self.timer = node.create_timer(1., self.poll_thresholds)
        camera = parameter('white_image_topic', '/camera/color/image_raw/compressed')
        # 11-19: raw subscription — the 30 Hz JPEG is deserialized only after the sign is confirmed
        # (before that every frame was decoded into a Python message and dropped). Same topic, no extra delay.
        self.subscription = node.create_subscription(CompressedImage, camera, self.on_image, qos_profile_sensor_data, raw=True)
        node.get_logger().info(
            f'주차 바닥 조건: YOLO {node.required_detections}회 후 새 흰색 ROI 1회, '
            f'흰색 비율 >= {self.roi.minimum_ratio:.0%} (주차 내부 HSV 계산)')

    @property
    def armed(self):
        return self.armed_stamp is not None

    def reset(self):
        self.confirmation.reset()
        self.armed_stamp = None
        self.armed_wall = None
        self.approach_distance = 0.
        self.last_wheel = None
        self.failure = ''
        self.ratio = None
        self.line_count = 0
        self.line_stamp = None

    def update_approach(self):
        if not self.armed or self.failure or self.node.activation.active:
            return
        wall,now=time.monotonic(),self.node.now_seconds()
        wheel=self.node.wheel_sample
        if (wheel is not None and -.05<=now-wheel[0]<=.3 and 0<=wall-wheel[3]<=.3):
            if self.last_wheel is not None:
                dt=wheel[0]-self.last_wheel[0]
                if 0<dt<=.3:
                    self.approach_distance+=.5*(abs(wheel[1])+abs(self.last_wheel[1]))*dt
            if self.last_wheel is None or wheel[0]>self.last_wheel[0]:
                self.last_wheel=wheel
        if wall-self.armed_wall>=self.approach_timeout:
            self.failure=f'흰선 탐색 {self.approach_timeout:g}초 한도에 도달했습니다. 정지·재출발 확인 후 새 표지와 흰선으로 재시도합니다.'
        elif self.approach_distance>=self.approach_max_distance-1e-8:
            self.failure=f'흰선 탐색 {self.approach_max_distance:g}m 한도에 도달했습니다. 정지·재출발 확인 후 새 표지와 흰선으로 재시도합니다.'
        if self.failure:
            self.confirmation.reset()
            self.node.get_logger().warning(self.failure)

    def clock(self):
        now = self.node.get_clock().now().nanoseconds
        if self.last_ros is not None and now < self.last_ros:
            self.reset()
            self.last_stamp = None
        self.last_ros = now
        return now

    def on_sign(self, message):
        now = self.clock()
        self.update_approach()
        if self.armed and not self.failure:
            return
        self.confirmation.required = self.node.required_detections
        self.confirmation.timeout = self.node.sign_timeout
        self.confirmation.expire(now*1e-9)
        self.confirmation.last=now*1e-9
        self.confirmation.count = self.confirmation.count + 1 if message.data else 0
        if self.confirmation.count >= self.node.required_detections:
            self.on_confirmed_sign()

    def on_confirmed_sign(self):
        # The node confirmed the set already; do not count through sign_timeout again.
        if self.armed and not self.failure:return
        now=self.clock()
        self.reset()
        self.node.begin_floor_approach()
        self.armed_stamp=now
        self.armed_wall=time.monotonic()
        wheel=self.node.wheel_sample
        self.last_wheel=(self.node.now_seconds(),wheel[1],wheel[2],wheel[3]) if wheel is not None else None
        self.node.get_logger().info(f'PARKING: 표지 묶음 확인, {self.approach_speed:.2f}m/s 이하로 흰선에 접근합니다.')

    def observe(self, stamp, ratio, span, angle):
        now = self.clock()
        self.update_approach()
        if (not self.armed
                or self.failure
                or time.monotonic() < self.node.sign_ignore_until
                or stamp <= self.armed_stamp or (self.last_stamp is not None and stamp <= self.last_stamp)
                or not -.05 <= (now-stamp)/1e9 <= self.timeout):
            return
        self.last_stamp = stamp
        self.ratio = ratio
        hit = ratio >= self.roi.minimum_ratio
        if self.line_stamp is None or (stamp-self.line_stamp)/1e9 > .25:
            self.line_count = 0
        self.line_count = self.line_count+1 if hit else 0
        self.line_stamp = stamp
        if self.line_count >= 1:
            if not self.node.parking_stop_line_allowed():
                # Signal/crosswalk stops can see a white line too. Keep waiting
                # for a new parking-line image while lane owns the approach.
                self.line_count = 0
                return
            self.reset()
            self.start()

    def on_image(self, message):
        now, wall = self.clock(), time.monotonic()
        self.update_approach()
        if not self.armed or self.failure or wall < self.node.sign_ignore_until:
            return
        if isinstance(message, (bytes, bytearray, memoryview)):
            message = deserialize_message(bytes(message), CompressedImage)
        stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        if (stamp <= 0 or stamp <= self.armed_stamp
                or (self.last_stamp is not None and stamp <= self.last_stamp)
                or not -.05 <= (now - stamp) / 1e9 <= self.timeout
                or wall - self.last_wall < 1. / self.hz):
            return
        self.last_wall = wall
        try:
            image = cv2.imdecode(np.frombuffer(message.data, dtype=np.uint8), cv2.IMREAD_COLOR)
            if image is None:
                raise ValueError('Cannot decode camera JPEG')
            ratio, _, mask = self.roi.measure(image, self.s_max, self.v_min)
            # Re-check age after decoding so a delayed callback cannot start parking.
            age = (self.node.get_clock().now().nanoseconds - stamp) / 1e9
            if not -.05 <= age <= self.timeout:
                return
            self.observe(stamp,ratio,0.,0.)
        except (ValueError, cv2.error) as error:
            self.node.get_logger().warning(f'Parking white ROI image rejected: {error}', throttle_duration_sec=5.)

    def poll_thresholds(self):
        if self.query is not None:
            future, started = self.query
            if future.done():
                try:
                    values = future.result().values
                    if len(values) == 2 and all(v.type in (2, 3) for v in values):
                        self.s_max, self.v_min = validate_white(*(
                            v.integer_value if v.type == 2 else v.double_value for v in values))
                except (ValueError, AttributeError, RuntimeError) as error:
                    self.node.get_logger().warning(f'Cannot update parking white HSV: {error}', throttle_duration_sec=5.)
                self.query = None
            elif time.monotonic() - started > 2.:
                future.cancel()
                self.query = None
        if self.query is None and self.client.service_is_ready():
            self.query = (self.client.call_async(GetParameters.Request(names=['white_s_max', 'white_v_min'])),
                          time.monotonic())
