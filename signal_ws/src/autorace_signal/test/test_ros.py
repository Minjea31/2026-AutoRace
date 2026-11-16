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
from geometry_msgs.msg import Twist
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool
from sensor_msgs.msg import CompressedImage

from autorace_signal.node import SignalNode
from autorace_mux.core import INPUTS
from autorace_mux.node import MuxNode


class Harness:
    def __init__(self, max_wait=None, white_gate=False, stop_delay=0.):
        self.context = Context()
        rclpy.init(context=self.context, domain_id=180+os.getpid()%20,
                   signal_handler_options=SignalHandlerOptions.NO)
        prefix = '/signal_test_'+uuid4().hex
        self.topics = {key: prefix+'/'+key for key, _ in INPUTS}
        overrides = [Parameter('white_roi_enabled', value=white_gate)]
        if stop_delay is not None:
            overrides.append(Parameter('stop_delay_sec', value=stop_delay))
        if max_wait is not None:
            overrides.append(Parameter('max_red_wait_sec', value=max_wait))
        self.signal = SignalNode(context=self.context, parameter_overrides=overrides,
            cli_args=['--ros-args', '-r', '/yolo/signal:='+prefix+'/yolo',
                      '-r', '/mission/lane:='+self.topics['lane'],
                      '-r', '/mission/signal/command_02:='+self.topics['signal'],
                      '-r', '/camera/color/image_raw/compressed:='+prefix+'/image',
                      '-r', '/start_signal:='+prefix+'/start'])
        self.mux = MuxNode(context=self.context, parameter_overrides=[
            Parameter(k, value=v) for k, v in {
                **{'topics.'+key: topic for key, topic in self.topics.items()},
                'output_topic': prefix+'/output', 'max_acceleration_mps2': 100.,
            }.items()])
        self.probe = Node('signal_probe', context=self.context)
        self.lane_pub = self.probe.create_publisher(AckermannDriveStamped, self.topics['lane'], 1)
        self.light_pub = self.probe.create_publisher(Bool, prefix+'/yolo', 1)
        self.camera = self.probe.create_publisher(CompressedImage, prefix+'/image', 1)
        self.commands, self.outputs, self.starts = [], [], []
        self.subs = [self.probe.create_subscription(AckermannDriveStamped, self.topics['signal'],
                                                  self.commands.append, 1),
                     self.probe.create_subscription(Twist, prefix+'/output', self.outputs.append, 1),
                     self.probe.create_subscription(Bool,prefix+'/start',self.starts.append,1)]
        self.executor = SingleThreadedExecutor(context=self.context)
        for node in (self.signal, self.mux, self.probe):
            self.executor.add_node(node)
        self.lane_enabled = False
        deadline = time.monotonic()+3.
        while (self.lane_pub.get_subscription_count() < 2 or self.light_pub.get_subscription_count() < 1
               or (white_gate and self.camera.get_subscription_count() < 1)):
            assert time.monotonic() < deadline, 'DDS discovery timed out'
            self.pump(.02)

    def lane_message(self):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.probe.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.drive.speed, msg.drive.steering_angle = .3, .12
        msg.drive.steering_angle_velocity, msg.drive.acceleration, msg.drive.jerk = .2, .4, .6
        return msg

    def pump(self, duration):
        deadline, next_lane = time.monotonic()+duration, 0.
        while time.monotonic() < deadline:
            if self.lane_enabled and time.monotonic() >= next_lane:
                self.lane_pub.publish(self.lane_message())
                next_lane = time.monotonic()+.02
            self.executor.spin_once(timeout_sec=.005)

    def light(self, green, count):
        for _ in range(count):
            self.light_pub.publish(Bool(data=green))
            self.pump(.05)

    def floor(self, white, age=0.):
        ok, encoded = cv2.imencode('.jpg', np.full((240,320,3),255 if white else 0,np.uint8))
        assert ok
        message = CompressedImage(format='jpeg', data=encoded.tobytes())
        stamp = self.probe.get_clock().now().nanoseconds-int(age*1e9)
        message.header.stamp.sec,message.header.stamp.nanosec=divmod(stamp,1_000_000_000)
        self.camera.publish(message)
        self.pump(.13)
        return message

    def close(self):
        self.executor.shutdown()
        for node in (self.signal, self.mux, self.probe):
            node.destroy_node()
        self.context.shutdown()


def test_dds_confirmation_preserves_fields_and_mux_returns_to_lane():
    h = Harness()
    try:
        assert h.signal.confirm_frames == 2
        assert h.signal.max_red_wait_sec == 10.0
        h.lane_enabled = True
        h.pump(.3)
        assert h.outputs[-1].linear.x == pytest.approx(.3)
        h.light(False, 1)
        assert not h.commands
        h.light(False, 1)
        h.pump(.1)
        assert h.outputs[-1].linear.x == 0.
        msg = h.commands[-1]
        assert msg.drive.speed == 0. and msg.drive.steering_angle == pytest.approx(.12)
        assert (msg.drive.steering_angle_velocity, msg.drive.acceleration, msg.drive.jerk) == pytest.approx((.2, .4, .6))
        assert msg.header.frame_id == 'base_link'
        h.light(True, 1)
        assert h.signal.state.stopping
        h.light(True, 1)
        h.pump(.12)
        count = len(h.commands)
        h.pump(.35)
        assert len(h.commands) == count
        assert h.outputs[-1].linear.x == pytest.approx(.3)
        assert h.outputs[-1].angular.z == pytest.approx(.3*math.tan(.12)/.2)
    finally:
        h.close()


def test_missing_stale_and_invalid_lane_use_neutral_stop():
    h = Harness()
    try:
        h.light(False, 2)
        h.pump(.1)
        assert h.commands[-1].drive.steering_angle == 0.
        valid = h.lane_message()
        original = copy.deepcopy(valid)
        h.signal.on_lane(valid)
        h.pump(.06)
        assert h.commands[-1].drive.steering_angle == pytest.approx(.12)
        assert valid == original
        old = copy.deepcopy(valid)
        old.header.stamp.nanosec = max(0, old.header.stamp.nanosec-1)
        received = h.signal.lane_received
        h.signal.on_lane(old)
        assert h.signal.lane_received == received
        h.pump(.3)
        assert h.commands[-1].drive.steering_angle == 0.
        for field in ('speed', 'steering_angle', 'steering_angle_velocity', 'acceleration', 'jerk'):
            invalid = h.lane_message()
            setattr(invalid.drive, field, float('nan'))
            h.signal.on_lane(invalid)
            assert h.signal.lane is None
        for stamp in ('missing', 'old', 'future'):
            invalid = h.lane_message()
            invalid.header.stamp.sec += {'missing': 0, 'old': -2, 'future': 2}[stamp]
            if stamp == 'missing':
                invalid.header.stamp.sec = invalid.header.stamp.nanosec = 0
            h.signal.on_lane(invalid)
            assert h.signal.lane is None
    finally:
        h.close()


def test_timeout_stops_publishing_and_requires_two_new_reds():
    h = Harness(max_wait=.35)
    try:
        h.light(False, 2)
        h.pump(.1)
        assert h.commands and h.signal.state.stopping
        h.pump(.4)
        count = len(h.commands)
        h.pump(.1)
        assert not h.signal.state.stopping and len(h.commands) == count
        h.light(False, 1)
        assert not h.signal.state.stopping
        h.light(False, 1)
        assert h.signal.state.stopping
    finally:
        h.close()


def test_repeated_red_does_not_extend_timeout_and_mux_returns_to_lane():
    h=Harness(max_wait=.8)
    try:
        h.lane_enabled=True
        h.pump(.3)
        h.light(False,2)
        first=h.signal.state.red_started_at
        h.pump(.3)
        h.light(False,1)
        assert h.signal.state.red_started_at==first
        while time.monotonic()<first+.9:
            h.light(False,1)
        assert not h.signal.state.stopping
        count=len(h.commands)
        for _ in range(12):h.light(False,1)
        assert len(h.commands)==count and len(h.starts)==1
        assert h.outputs[-1].linear.x==pytest.approx(.3)
    finally:
        h.close()


def test_default_signal_stops_after_two_seconds_and_releases_ten_seconds_from_red():
    h=Harness(white_gate=True,stop_delay=None)
    try:
        assert h.signal.stop_delay_sec==2.
        assert h.signal.max_red_wait_sec==10.
        h.lane_enabled=True
        h.pump(.3)
        old_white=h.floor(True)
        h.light(False,2)
        assert h.signal.state.armed and not h.commands and not h.starts
        h.camera.publish(old_white)
        h.pump(.13)
        assert h.signal.state.armed and not h.commands
        h.floor(False)
        h.floor(True,age=1.)
        assert h.signal.state.armed and not h.commands
        first=time.monotonic()
        h.floor(True)
        deadline=h.signal.state.stop_at
        assert deadline>=first+2.
        assert not h.commands and not h.starts
        h.floor(True)
        assert h.signal.state.stop_at==deadline
        h.pump(max(0.,deadline-time.monotonic()-.06))
        assert not h.commands and not h.starts
        h.pump(.20)
        assert h.commands and all(m.drive.speed==0. for m in h.commands)
        assert len(h.starts)==1 and h.starts[0].data
        assert h.outputs[-1].linear.x==0.
        red_deadline=h.signal.state.red_started_at+10.
        while time.monotonic()<red_deadline-.15:
            h.light(False,1)
        assert h.signal.state.stopping
        h.pump(max(0.,red_deadline+.10-time.monotonic()))
        assert not h.signal.state.stopping and h.signal.state.timed_out
        count=len(h.commands)
        for _ in range(12):h.light(False,1)
        assert len(h.commands)==count and len(h.starts)==1
        assert h.outputs[-1].linear.x==pytest.approx(.3)
    finally:
        h.close()


def test_green_cancels_white_delay_and_retry_requires_a_new_white():
    h=Harness(white_gate=True,stop_delay=.5)
    try:
        h.lane_enabled=True
        h.light(False,2)
        h.floor(True)
        assert h.signal.state.stop_at is not None
        h.light(True,2)
        h.pump(.55)
        assert not h.commands and not h.starts
        assert h.signal.state.stop_at is None and not h.signal.state.armed
        h.light(False,2)
        h.pump(.55)
        assert h.signal.state.armed and not h.commands
        h.floor(True)
        h.pump(max(0.,h.signal.state.stop_at-time.monotonic())+.15)
        assert h.commands and len(h.starts)==1
    finally:
        h.close()
