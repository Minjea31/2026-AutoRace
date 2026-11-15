"""Seven Ackermann inputs -> priority arbitration -> /cmd_vel Twist."""

from dataclasses import fields
from functools import partial
import math
import json
import signal
from threading import Event
import time

from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist
from std_msgs.msg import Header, String
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions

from autorace_mux.parking_guard import ParkingGuard
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
        source_topic = self.declare_parameter(
            'active_source_topic', '/mux/active_source', descriptor).value
        if not source_topic or source_topic in (*topics.values(), output):
            raise ValueError('Active source topic must be nonempty and differ from command topics')
        source_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._source_publisher = self.create_publisher(String, source_topic, source_qos)
        self._input_topics = topics
        self._output_topic = output
        self._subscriptions = [
            self.create_subscription(
                AckermannDriveStamped, topic, partial(self._receive, name), qos)
            for name, topic in topics.items()
        ]
        self._parking_release_stamp = 0
        self._last_release_clock = None
        handoff = self.declare_parameter('parking_handoff_topic', '/parking/lane_handoff', descriptor).value
        self._parking_handoff_subscription = self.create_subscription(Header, handoff, self._release_parking, qos)
        self.parking_guard = ParkingGuard()
        self.parking_guard_enabled = self.declare_parameter('parking_guard_enabled',False,descriptor).value
        self.create_subscription(String,'/parking/control_state',self._parking_state,qos)
        self.create_subscription(Header,'/parking/approach_start',self._parking_start,qos)
        self._last_source = None
        self._last_warning = {}
        # Continue the watchdog even if /clock pauses when use_sim_time=true.
        self._steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self._timer = self.create_timer(
            1.0 / self.config.publish_rate_hz, self._publish, clock=self._steady_clock)
        self.get_logger().info(
            f'Mux ready: default={topics["lane"]}, output={output}, '
            f'timeout={self.config.input_timeout_sec:.3f}s, active_source={source_topic}')

    def _parking_start(self, message):
        if self.parking_guard_enabled:
            self.parking_guard.arm(message.stamp.sec+message.stamp.nanosec*1e-9,
                time.monotonic(),self._clock_ns()*1e-9)

    def _parking_state(self, message):
        if self.parking_guard_enabled:
            try:self.parking_guard.receive(json.loads(message.data),time.monotonic(),self._clock_ns()*1e-9)
            except (ValueError,TypeError):pass

    def _warn_rejected(self, source, reason):
        now = time.monotonic()
        if now - self._last_warning.get(source, -math.inf) >= 1.0:
            self.get_logger().warning(f'Ignoring {source}: {reason}')
            self._last_warning[source] = now

    def _clock_ns(self):
        now = self.get_clock().now().nanoseconds
        if self._last_release_clock is not None and now < self._last_release_clock:
            self._parking_release_stamp = 0
        self._last_release_clock = now
        return now

    def _release_parking(self, message):
        now = self._clock_ns()
        stamp = message.stamp.sec * 1_000_000_000 + message.stamp.nanosec
        if (message.stamp.sec < 0 or not 0 <= message.stamp.nanosec < 1_000_000_000
                or stamp <= self._parking_release_stamp
                or not -self.config.future_tolerance_sec <= (now - stamp) / 1e9 < self.config.input_timeout_sec):
            return
        self._parking_release_stamp = stamp
        self.arbitrator.discard_through('parking', stamp)
        self._publish()

    def _receive(self, source, message):
        now = self._clock_ns()
        stamp_ns = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        if source == 'parking' and self._parking_release_stamp > 0 and 0 <= stamp_ns <= self._parking_release_stamp:
            return
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
            time.monotonic(), now)
        if not accepted:
            self._warn_rejected(source, reason)

    def _publish(self):
        now = time.monotonic()
        candidate = self.arbitrator.select(now, self._clock_ns())
        if self.parking_guard_enabled:
            candidate = self.parking_guard.apply(candidate,now,self._clock_ns()*1e-9)
        speed, yaw_rate = self.converter.convert(candidate, now)
        message = Twist()
        message.linear.x = speed
        message.angular.z = yaw_rate
        self._publisher.publish(message)
        source = candidate.source if candidate else 'stop'
        self._source_publisher.publish(String(data=source))
        if source != self._last_source:
            if candidate:
                label, input_topic = source, self._input_topics[source]
            else:
                reason = 'parking guard' if self.parking_guard_enabled and self.parking_guard.armed else 'no fresh input'
                label, input_topic = f'STOP ({reason})', 'none'
            self.get_logger().info(
                f'Selected: {label} | input={input_topic} | output={self._output_topic} | '
                f'speed={speed:.3f} m/s, yaw_rate={yaw_rate:.3f} rad/s')
        self._last_source = source

    def stop(self):
        self._timer.cancel()
        self._publisher.publish(Twist())
        self._source_publisher.publish(String(data='stop'))
        if self._last_source != 'stop':
            self.get_logger().info('Selected: STOP (shutdown)')
        self._last_source = 'stop'
        self._publisher.wait_for_all_acked(Duration(seconds=0.2))
        self._source_publisher.wait_for_all_acked(Duration(seconds=0.2))


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
