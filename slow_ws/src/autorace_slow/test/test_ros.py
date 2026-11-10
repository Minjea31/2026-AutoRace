import copy
import math
import os
import time
import cv2
import numpy as np
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
from sensor_msgs.msg import CompressedImage

from autorace_slow.node import SlowNode
from autorace_mux.core import INPUTS
from autorace_mux.node import MuxNode


class Harness:
    def __init__(self, hold=2., sim=False, white_gate=False):
        self.context = Context()
        rclpy.init(context=self.context, domain_id=180+os.getpid()%20,
                   signal_handler_options=SignalHandlerOptions.NO)
        prefix = '/slow_test_'+uuid4().hex
        self.topics = {key: prefix+'/'+key for key, _ in INPUTS}
        self.sim = sim
        common = [Parameter('use_sim_time', value=sim)]
        clock_args = ['--ros-args', '-r', '/clock:='+prefix+'/clock']
        self.node = SlowNode(context=self.context, parameter_overrides=common+[
            Parameter('hold_sec', value=hold),
            Parameter('white_roi_enabled', value=white_gate)], cli_args=clock_args+[
                '-r', '/mission/lane:='+self.topics['lane'],
                '-r', '/mission/slow/command_08:='+self.topics['slow'],
                '-r', '/yolo/slow_sign:='+prefix+'/sign',
                '-r', '/yolo/bump_sign:='+prefix+'/bump',
                '-r', '/camera/color/image_raw/compressed:='+prefix+'/image'])
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
        self.camera = self.probe.create_publisher(CompressedImage, prefix+'/image', 1)
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
               or any(pub.get_subscription_count() < 1 for pub in self.signs.values())
               or (white_gate and self.camera.get_subscription_count() < 1)):
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

    def detect(self, kind, count=2):
        for _ in range(count):
            self.signs[kind].publish(Bool(data=True))
            self.pump(.015)

    def floor(self, white, age=0.):
        frame = np.full((240, 320, 3), 255 if white else 0, dtype=np.uint8)
        ok, encoded = cv2.imencode('.jpg', frame)
        assert ok
        message = CompressedImage(format='jpeg', data=encoded.tobytes())
        stamp = self.probe.get_clock().now().nanoseconds - int(age * 1e9)
        message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
        self.camera.publish(message)
        self.pump(.13)
        return message

    def close(self):
        self.executor.shutdown()
        for node in self.nodes:
            node.destroy_node()
        self.context.shutdown()


def test_floor_gate_requires_yolo_two_then_new_fresh_white_camera_frame():
    h = Harness(white_gate=True)
    try:
        h.lane_enabled = True
        h.pump(.25)
        old_white = h.floor(True)
        h.detect('slow')
        assert h.node.state.missions['slow'].armed
        assert not h.commands
        h.camera.publish(old_white)
        h.pump(.12)
        assert not h.commands
        h.floor(False)
        assert not h.commands
        h.floor(True, age=1.)
        assert not h.commands
        h.floor(True)
        assert h.commands[-1].drive.speed == pytest.approx(.2)
        assert h.commands[-1].drive.steering_angle == pytest.approx(h.angle)
    finally:
        h.close()


def test_bump_can_wait_after_sign_disappears_then_keep_latest_lane_steering():
    h = Harness(white_gate=True)
    try:
        h.lane_enabled = True
        h.angle = .22
        h.pump(.25)
        h.detect('bump')
        h.signs['bump'].publish(Bool(data=False))
        h.pump(.1)
        assert h.node.state.missions['bump'].armed and not h.commands
        h.floor(True, age=.15)
        # This image is fresh enough, but was acquired before confirmation.
        # A newer image, not a remembered white result, is required.
        assert not h.commands
        h.floor(True)
        assert h.commands[-1].drive.speed == pytest.approx(.2)
        assert h.commands[-1].drive.steering_angle == pytest.approx(.22)
    finally:
        h.close()


def test_dds_slow_and_bump_preserve_latest_lane_steering():
    h = Harness()
    try:
        h.lane_enabled = True
        h.pump(.25)
        assert h.outputs[-1].linear.x == pytest.approx(.4)
        h.angle = .12
        h.pump(.025)
        h.detect('slow', 1)
        assert not h.commands
        h.detect('slow', 1)
        h.pump(.15)
        assert h.commands[-1].drive.speed == pytest.approx(.2)
        assert h.commands[-1].drive.steering_angle == pytest.approx(h.angle)
        drive = h.commands[-1].drive
        assert (drive.steering_angle_velocity, drive.acceleration, drive.jerk) == pytest.approx((.2, .3, .6))
        assert h.node.lane.drive.speed == pytest.approx(.4)
        deadline = time.monotonic() + .6
        while abs(h.outputs[-1].linear.x - .2) > .001 and time.monotonic() < deadline:
            h.pump(.025)
        assert h.outputs[-1].linear.x == pytest.approx(.2, abs=.001)
        h.angle = .24
        h.pump(.12)
        h.detect('slow', 1)
        h.pump(.06)
        assert h.commands[-1].drive.steering_angle == pytest.approx(h.angle)
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


def test_missing_stale_invalid_lane_stops_and_first_fresh_lane_resumes_tracking():
    h = Harness()
    try:
        h.detect('slow')
        h.pump(.06)
        assert h.commands[-1].drive.speed == 0.
        h.lane_enabled = True
        h.pump(.08)
        assert h.commands[-1].drive.speed == pytest.approx(.2)
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
