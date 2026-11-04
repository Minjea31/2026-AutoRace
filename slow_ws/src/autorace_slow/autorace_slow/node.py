"""Two sign inputs, fresh lane commands and one slow mission output."""
import copy
from functools import partial
import math
import signal
from threading import Event
import time

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool

from .core import SlowState


class SlowNode(Node):
    def __init__(self, **kwargs):
        super().__init__('slow_node', **kwargs)
        defaults = dict(confirm_frames=3, hold_sec=8., signal_gap_timeout_sec=.5,
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
        self.state = SlowState(self.confirm_frames, self.hold_sec, self.signal_gap_timeout_sec)
        self.lane = self.lane_received = self.last_ros_ns = None
        self.last_mode = None
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        self.command_pub = self.create_publisher(AckermannDriveStamped, '/mission/slow/command_08', qos)
        self.lane_sub = self.create_subscription(AckermannDriveStamped, '/mission/lane', self.on_lane, qos)
        self.sign_subs = [self.create_subscription(Bool, topic, partial(self.on_sign, kind), qos)
                          for kind, topic in (('slow', '/yolo/slow_sign'), ('bump', '/yolo/bump_sign'))]
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

    def on_sign(self, kind, message):
        self.state.receive(kind, message.data, time.monotonic())

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
            if mode == 'slow':
                # TODO: IMU 토픽으로 변경 예정. 현재 서행 조향각은 0으로 고정한다.
                message.drive.steering_angle = 0.0
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
