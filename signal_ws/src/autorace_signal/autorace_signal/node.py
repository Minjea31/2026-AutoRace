"""Stop on confirmed red; release on confirmed green or a red-free timeout."""
import copy
import math
import time

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool

from .core import SignalState

# Tune here; launch does not override these defaults. Restart after changes.
CONFIRM_FRAMES = 2
MAX_RED_WAIT_SEC = 10.0  # Restarted by every red received while stopping.
SIGNAL_GAP_TIMEOUT_SEC = 0.5
PUBLISH_RATE_HZ = 30.0
LANE_TIMEOUT_SEC = 0.25
FUTURE_TOLERANCE_SEC = 0.05


class SignalNode(Node):
    def __init__(self, **kwargs):
        super().__init__('signal_node', **kwargs)
        defaults = dict(confirm_frames=CONFIRM_FRAMES, max_red_wait_sec=MAX_RED_WAIT_SEC,
                        signal_gap_timeout_sec=SIGNAL_GAP_TIMEOUT_SEC,
                        publish_rate_hz=PUBLISH_RATE_HZ, lane_timeout_sec=LANE_TIMEOUT_SEC,
                        future_tolerance_sec=FUTURE_TOLERANCE_SEC)
        for key, default in defaults.items():
            value = self.declare_parameter(key, default, ParameterDescriptor(read_only=True)).value
            if (isinstance(value, bool) or not isinstance(value, (int, float))
                    or not math.isfinite(value) or value < 0
                    or (key != 'future_tolerance_sec' and value == 0)
                    or (key == 'confirm_frames' and not isinstance(value, int))):
                raise ValueError(f'Invalid {key}: {value}')
            setattr(self, key, value)
        self.state = SignalState(self.confirm_frames, self.max_red_wait_sec, self.signal_gap_timeout_sec)
        self.lane = None
        self.lane_received = None
        self.last_ros_ns = None
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        self.command_pub = self.create_publisher(AckermannDriveStamped, '/mission/signal/command_02', qos)
        self.signal_sub = self.create_subscription(Bool, '/yolo/signal', self.on_signal, qos)
        self.lane_sub = self.create_subscription(AckermannDriveStamped, '/mission/lane', self.on_lane, qos)
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(1/self.publish_rate_hz, self.publish, clock=self.steady_clock)
        self.get_logger().info(f'Signal ready: confirm={self.confirm_frames}, red inactivity timeout={self.max_red_wait_sec}s')

    def ros_now(self):
        now = self.get_clock().now().nanoseconds
        if self.last_ros_ns is not None and now < self.last_ros_ns:
            self.lane = None
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
            self.get_logger().info('Signal: STOPPING (red confirmed)' if self.state.stopping
                                   else f'Signal: PASS ({self.state.release_reason})')

    def on_signal(self, message):
        was_stopping = self.state.stopping
        self.state.receive(message.data, time.monotonic())
        self.log_transition(was_stopping)

    def publish(self):
        now, ros_now = time.monotonic(), self.ros_now()
        was_stopping = self.state.stopping
        self.state.tick(now)
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
