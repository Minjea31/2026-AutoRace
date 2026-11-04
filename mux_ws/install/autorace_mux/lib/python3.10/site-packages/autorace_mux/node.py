"""Seven Ackermann inputs -> priority arbitration -> /cmd_vel Twist."""

from dataclasses import fields
from functools import partial
import math
import signal
from threading import Event
import time

from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions

from autorace_mux.core import Arbitrator, Config, INPUTS, TwistConverter


class MuxNode(Node):
    def __init__(self, **kwargs):
        super().__init__('mux', **kwargs)
        descriptor = ParameterDescriptor(read_only=True)
        defaults = Config()
        values = {
            f.name: self.declare_parameter(
                f.name, getattr(defaults, f.name), descriptor).value
            for f in fields(Config)
        }
        self.config = Config(**values)
        self.arbitrator = Arbitrator(self.config)
        self.converter = TwistConverter(self.config)
        topics = {
            name: self.declare_parameter(f'topics.{name}', topic, descriptor).value
            for name, topic in INPUTS
        }
        output = self.declare_parameter('output_topic', '/cmd_vel', descriptor).value
        if not all(topics.values()) or not output or len(set(topics.values())) != len(topics):
            raise ValueError('Input topics must be nonempty and distinct; output must be nonempty')
        if output in topics.values():
            raise ValueError('Output topic must differ from every input topic')
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE)
        self._publisher = self.create_publisher(Twist, output, qos)
        self._subscriptions = [
            self.create_subscription(
                AckermannDriveStamped, topic, partial(self._receive, name), qos)
            for name, topic in topics.items()
        ]
        self._last_source = None
        self._last_warning = {}
        # Continue the watchdog even if /clock pauses when use_sim_time=true.
        self._steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self._timer = self.create_timer(
            1.0 / self.config.publish_rate_hz, self._publish, clock=self._steady_clock)
        self.get_logger().info(
            f'Mux ready: default={topics["lane"]}, output={output}, '
            f'timeout={self.config.input_timeout_sec:.3f}s')

    def _warn_rejected(self, source, reason):
        now = time.monotonic()
        if now - self._last_warning.get(source, -math.inf) >= 1.0:
            self.get_logger().warning(f'Ignoring {source}: {reason}')
            self._last_warning[source] = now

    def _receive(self, source, message):
        drive = message.drive
        values = (drive.speed, drive.steering_angle, drive.steering_angle_velocity,
                  drive.acceleration, drive.jerk)
        stamp = message.header.stamp
        if not all(math.isfinite(value) for value in values) or not (
                stamp.sec >= 0 and 0 <= stamp.nanosec < 1_000_000_000):
            self.arbitrator.discard(source)
            self._warn_rejected(source, 'invalid numeric field or header stamp')
            return
        accepted, reason = self.arbitrator.receive(
            source, drive.speed, drive.steering_angle,
            stamp.sec * 1_000_000_000 + stamp.nanosec,
            time.monotonic(), self.get_clock().now().nanoseconds)
        if not accepted:
            self._warn_rejected(source, reason)

    def _publish(self):
        now = time.monotonic()
        candidate = self.arbitrator.select(now, self.get_clock().now().nanoseconds)
        speed, yaw_rate = self.converter.convert(candidate, now)
        message = Twist()
        message.linear.x = speed
        message.angular.z = yaw_rate
        self._publisher.publish(message)
        source = candidate.source if candidate else 'STOP (no fresh input)'
        if source != self._last_source:
            self.get_logger().info(f'Selected: {source}')
            self._last_source = source

    def stop(self):
        self._timer.cancel()
        self._publisher.publish(Twist())
        self._publisher.wait_for_all_acked(Duration(seconds=0.2))


def main(args=None):
    # Keep the context alive until the final zero Twist has been published.
    stopped = Event()
    previous_handlers = {}
    node = None
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    try:
        for signum in (signal.SIGINT, signal.SIGTERM):
            previous_handlers[signum] = signal.signal(signum, lambda *_: stopped.set())
        node = MuxNode()
        while rclpy.ok() and not stopped.is_set():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            if rclpy.ok():
                node.stop()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for signum, previous in previous_handlers.items():
            signal.signal(signum, previous)
