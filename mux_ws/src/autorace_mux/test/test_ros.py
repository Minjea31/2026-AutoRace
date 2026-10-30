"""Real DDS pub/sub checks on private topics, with no vehicle output."""

import math
import os
import time
from uuid import uuid4

import pytest

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('ackermann_msgs.msg')
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile
from rclpy.signals import SignalHandlerOptions

from autorace_mux.core import INPUTS
from autorace_mux.node import MuxNode


class Harness:
    def __init__(self, simulated=False):
        self.context = Context()
        rclpy.init(context=self.context, domain_id=180 + os.getpid() % 20,
                   signal_handler_options=SignalHandlerOptions.NO)
        prefix = '/mux_test_' + uuid4().hex
        self.topics = {name: prefix + '/' + name for name, _ in INPUTS}
        parameters = {
            'output_topic': prefix + '/output', 'publish_rate_hz': 60.0,
            'input_timeout_sec': 0.18,
            'max_acceleration_mps2': 100.0, 'max_deceleration_mps2': 100.0,
            **{'topics.' + key: value for key, value in self.topics.items()},
        }
        if simulated:
            parameters.update(use_sim_time=True, require_stamp=False)
        self.mux = MuxNode(
            context=self.context,
            parameter_overrides=[Parameter(key, value=value) for key, value in parameters.items()])
        self.probe = Node('mux_probe_' + uuid4().hex, context=self.context)
        qos = QoSProfile(depth=1)
        self.publishers = {key: self.probe.create_publisher(AckermannDriveStamped, value, qos)
                           for key, value in self.topics.items()}
        self.samples = []
        self.subscription = self.probe.create_subscription(
            Twist, prefix + '/output',
            lambda msg: self.samples.append((msg.linear.x, msg.angular.z)), qos)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.mux)
        self.executor.add_node(self.probe)
        try:
            self.until(lambda: all(p.get_subscription_count() for p in self.publishers.values())
                       and self.probe.count_publishers(self.subscription.topic_name) == 1)
        except BaseException:
            self.close()
            raise

    def until(self, predicate, commands=None, timeout=3.0):
        deadline = time.monotonic() + timeout
        next_publish = 0.0
        while time.monotonic() < deadline:
            now = time.monotonic()
            if commands and now >= next_publish:
                for key, (speed, angle) in commands.items():
                    msg = AckermannDriveStamped()
                    msg.header.stamp = self.mux.get_clock().now().to_msg()
                    msg.drive.speed = float(speed)
                    msg.drive.steering_angle = float(angle)
                    self.publishers[key].publish(msg)
                next_publish = now + 0.02
            self.executor.spin_once(timeout_sec=0.01)
            if predicate():
                return
        pytest.fail(f'Timed out waiting for mux output; recent samples={self.samples[-5:]}')

    def output(self, speed, yaw, commands=None):
        self.samples.clear()
        self.until(lambda: bool(self.samples) and
                   self.samples[-1] == pytest.approx((speed, yaw), abs=1e-5), commands)

    def close(self):
        self.mux.stop()
        self.executor.shutdown()
        self.probe.destroy_node()
        self.mux.destroy_node()
        self.context.shutdown()


def test_real_ros_lane_mission_preemption_reverse_and_timeout():
    h = Harness()
    try:
        lane = {'lane': (0.4, 0.1)}
        h.output(0.0, 0.0)  # Startup watchdog.
        h.output(0.4, 0.4 * math.tan(0.1) / 0.2, lane)
        h.output(0.2, 0.0, {**lane, 'slow': (0.2, 0.0)})
        h.output(0.0, 0.0, {**lane, 'slow': (0.2, 0.0), 'crosswalk': (0.0, 0.2)})
        h.output(0.4, 0.4 * math.tan(0.1) / 0.2, lane)  # Expired missions -> lane.
        h.output(0.4, 0.4 * math.tan(0.1) / 0.2, {**lane, 'signal': (-0.2, 0.0)})
        h.output(-0.2, -0.2 * math.tan(0.1) / 0.2, {**lane, 'parking': (-0.2, 0.1)})
        h.output(0.0, 0.0)  # All inputs expire, including lane.
    finally:
        h.close()


def test_watchdog_keeps_running_without_simulated_clock_messages():
    h = Harness(simulated=True)
    try:
        assert h.mux.get_clock().now().nanoseconds == 0
        h.output(0.3, 0.0, {'lane': (0.3, 0.0)})
        h.output(0.0, 0.0)
    finally:
        h.close()
