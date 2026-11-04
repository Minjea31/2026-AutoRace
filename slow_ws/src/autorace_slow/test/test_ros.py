import copy
import math
import os
import time
from uuid import uuid4

import pytest
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Twist
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.signals import SignalHandlerOptions
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Bool

from autorace_slow.node import SlowNode
from autorace_mux.core import INPUTS
from autorace_mux.node import MuxNode


class Harness:
    def __init__(self, hold=2., sim=False):
        self.context = Context()
        rclpy.init(context=self.context, domain_id=180+os.getpid()%20,
                   signal_handler_options=SignalHandlerOptions.NO)
        prefix = '/slow_test_'+uuid4().hex
        self.topics = {key: prefix+'/'+key for key, _ in INPUTS}
        self.sim = sim
        common = [Parameter('use_sim_time', value=sim)]
        clock_args = ['--ros-args', '-r', '/clock:='+prefix+'/clock']
        self.node = SlowNode(context=self.context, parameter_overrides=common+[
            Parameter('hold_sec', value=hold)], cli_args=clock_args+[
                '-r', '/mission/lane:='+self.topics['lane'],
                '-r', '/mission/slow/command_08:='+self.topics['slow'],
                '-r', '/yolo/slow_sign:='+prefix+'/sign',
                '-r', '/yolo/bump_sign:='+prefix+'/bump'])
        self.mux = None if sim else MuxNode(context=self.context, parameter_overrides=common+[
            Parameter(k, value=v) for k, v in {
                **{'topics.'+key: topic for key, topic in self.topics.items()},
                'output_topic': prefix+'/output', 'max_acceleration_mps2': 100.,
            }.items()], cli_args=clock_args)
        self.probe = Node('slow_probe', context=self.context, parameter_overrides=common, cli_args=clock_args)
        self.clock_pub = self.probe.create_publisher(Clock, prefix+'/clock', 1)
        self.lane_pub = self.probe.create_publisher(AckermannDriveStamped, self.topics['lane'], 1)
        self.signs = {kind: self.probe.create_publisher(Bool, prefix+'/'+topic, 1)
                      for kind, topic in (('slow', 'sign'), ('bump', 'bump'))}
        self.commands, self.outputs = [], []
        self.subs = [self.probe.create_subscription(AckermannDriveStamped, self.topics['slow'], self.commands.append, 1),
                     self.probe.create_subscription(Twist, prefix+'/output', self.outputs.append, 1)]
        self.executor = SingleThreadedExecutor(context=self.context)
        self.nodes = [node for node in (self.node, self.mux, self.probe) if node is not None]
        for node in self.nodes:
            self.executor.add_node(node)
        self.lane_enabled, self.angle = False, .05
        deadline = time.monotonic()+3.
        while (self.lane_pub.get_subscription_count() < (1 if sim else 2)
               or any(pub.get_subscription_count() < 1 for pub in self.signs.values())):
            assert time.monotonic() < deadline, 'DDS discovery timed out'
            self.pump(.02)
        self.pump(.1)

    def message(self):
        message = AckermannDriveStamped()
        message.header.stamp = self.probe.get_clock().now().to_msg()
        message.header.frame_id = 'base_link'
        message.drive.speed, message.drive.steering_angle = .4, self.angle
        message.drive.steering_angle_velocity = .2
        message.drive.acceleration, message.drive.jerk = .3, .6
        return message

    def pump(self, duration):
        deadline, next_lane, next_clock = time.monotonic()+duration, 0., 0.
        while time.monotonic() < deadline:
            if self.sim and time.monotonic() >= next_clock:
                self.clock_pub.publish(Clock(clock=Time(sec=100)))
                next_clock = time.monotonic()+.02
            if self.lane_enabled and time.monotonic() >= next_lane:
                self.lane_pub.publish(self.message())
                next_lane = time.monotonic()+.02
            self.executor.spin_once(timeout_sec=.003)

    def detect(self, kind, count=3):
        for _ in range(count):
            self.signs[kind].publish(Bool(data=True))
            self.pump(.015)

    def close(self):
        self.executor.shutdown()
        for node in self.nodes:
            node.destroy_node()
        self.context.shutdown()


def test_dds_historical_steering_is_frozen_and_bump_uses_latest_lane():
    h = Harness()
    try:
        h.lane_enabled = True
        h.pump(.25)
        assert h.outputs[-1].linear.x == pytest.approx(.4)
        h.angle = .12
        h.pump(.025)
        h.detect('slow', 2)
        assert not h.commands
        h.detect('slow', 1)
        assert h.node.frozen_angle == pytest.approx(.05)
        h.pump(.15)
        assert h.commands[-1].drive.speed == pytest.approx(.2)
        assert h.commands[-1].drive.steering_angle == pytest.approx(.05)
        assert h.outputs[-1].linear.x == pytest.approx(.2)
        h.angle = .24
        h.pump(.12)
        frozen = h.node.frozen_angle
        h.detect('slow', 1)
        h.pump(.06)
        assert h.node.frozen_angle == frozen
        assert h.commands[-1].drive.steering_angle == pytest.approx(.05)
        h.detect('bump')
        h.pump(.06)
        message = h.commands[-1]
        assert message.drive.speed == pytest.approx(.2)
        assert message.drive.steering_angle == pytest.approx(.24)
        assert (message.drive.steering_angle_velocity, message.drive.acceleration, message.drive.jerk) == pytest.approx((.2, .3, .6))
        assert message.header.frame_id == 'base_link'
        assert h.outputs[-1].angular.z == pytest.approx(.2*math.tan(.24)/.2)
    finally:
        h.close()


@pytest.mark.parametrize('sim', [False, True])
def test_renewal_expiry_with_system_mux_or_paused_sim_clock(sim):
    # The simulator-time case isolates SlowNode: existing MuxNode replaces the
    # Node subscription list and loses executor access to its /clock input.
    h = Harness(hold=.6, sim=sim)
    try:
        h.lane_enabled = True
        h.pump(.2)
        h.detect('bump')
        started_deadline = h.node.state.missions['bump'].deadline
        h.pump(.3)
        h.detect('bump', 1)
        assert h.node.state.missions['bump'].deadline > started_deadline
        h.pump(.35)
        assert time.monotonic() > started_deadline
        assert h.node.state.selected(time.monotonic()) == 'bump'
        assert h.commands[-1].drive.speed == pytest.approx(.2)
        if not sim:
            assert h.outputs[-1].linear.x == pytest.approx(.2)
        h.pump(.3)
        count = len(h.commands)
        assert h.node.state.selected(time.monotonic()) is None
        h.pump(.35)
        assert len(h.commands) == count
        if sim:
            assert h.node.get_clock().now().nanoseconds == 100_000_000_000
        else:
            assert h.outputs[-1].linear.x == pytest.approx(.4)
    finally:
        h.close()


def test_missing_stale_invalid_lane_and_history_wait_are_neutral_stops():
    h = Harness()
    try:
        h.detect('slow')
        h.pump(.06)
        assert h.commands[-1].drive.speed == 0.
        h.lane_enabled = True
        h.pump(.06)
        assert h.node.frozen_angle is None
        h.pump(.15)
        assert h.commands[-1].drive.speed == pytest.approx(.2)
        assert h.node.frozen_angle == pytest.approx(.05)
        h.lane_enabled = False
        h.pump(.3)
        assert h.commands[-1].drive.speed == h.commands[-1].drive.steering_angle == 0.
        valid = h.message()
        original = copy.deepcopy(valid)
        h.node.on_lane(valid)
        h.node.publish()
        assert valid == original
        received = h.node.lane_received
        old = copy.deepcopy(valid)
        ns = h.node.stamp_ns(old)-1
        old.header.stamp = Time(sec=ns//1_000_000_000, nanosec=ns%1_000_000_000)
        h.node.on_lane(old)
        assert h.node.lane_received == received
        for field in ('speed', 'steering_angle', 'steering_angle_velocity', 'acceleration', 'jerk'):
            invalid = h.message()
            setattr(invalid.drive, field, float('nan'))
            h.node.on_lane(invalid)
            assert h.node.lane is None
        for kind in ('empty', 'old', 'future', 'reverse'):
            invalid = h.message()
            if kind == 'empty':
                invalid.header.stamp = Time()
            elif kind == 'reverse':
                invalid.drive.speed = -.1
            else:
                invalid.header.stamp.sec += -2 if kind == 'old' else 2
            h.node.on_lane(invalid)
            assert h.node.lane is None
    finally:
        h.close()
