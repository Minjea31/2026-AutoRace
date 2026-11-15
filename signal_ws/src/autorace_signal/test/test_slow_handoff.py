"""Red-stop events cancel slow through DDS; test traffic never reaches the vehicle."""
import os
from pathlib import Path
import sys
import time
from uuid import uuid4

import cv2
import numpy as np
import pytest
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool

from autorace_signal.node import SignalNode

sys.path.insert(0, str(Path(__file__).resolve().parents[4] / 'slow_ws/src/autorace_slow'))
from autorace_slow.node import SlowNode


@pytest.mark.parametrize('bump_active', [False, True])
def test_red_stop_event_clears_slow_once_and_allows_a_fresh_second_mission(bump_active):
    context = Context()
    rclpy.init(context=context, domain_id=225, signal_handler_options=SignalHandlerOptions.NO)
    prefix = '/signal_slow_' + uuid4().hex
    topics = {
        '/start_signal': 'start', '/start_crosswalk': 'crosswalk',
        '/mission/lane': 'lane', '/yolo/signal': 'light',
        '/mission/signal/command_02': 'signal', '/mission/slow/command_08': 'slow',
        '/yolo/slow_sign': 'slow_sign', '/yolo/bump_sign': 'bump_sign',
        '/camera/color/image_raw/compressed': 'camera',
    }
    args = ['--ros-args']
    for source, target in topics.items():
        args += ['-r', source + ':=' + prefix + '/' + target]
    signal = SignalNode(context=context, cli_args=args, parameter_overrides=[
        Parameter('white_roi_enabled', value=False), Parameter('stop_delay_sec', value=0.)])
    slow = SlowNode(context=context, cli_args=args, parameter_overrides=[Parameter('start_delay_sec', value=0.)])
    probe = Node('signal_slow_probe', context=context)
    lane_pub = probe.create_publisher(AckermannDriveStamped, prefix + '/lane', 10)
    lights = probe.create_publisher(Bool, prefix + '/light', 10)
    starts, stops, slow_commands = [], [], []
    subscriptions = [
        probe.create_subscription(Bool, prefix + '/start', starts.append, 10),
        probe.create_subscription(AckermannDriveStamped, prefix + '/signal', stops.append, 10),
        probe.create_subscription(AckermannDriveStamped, prefix + '/slow', slow_commands.append, 10),
    ]
    executor = SingleThreadedExecutor(context=context)
    for node in (signal, slow, probe):
        executor.add_node(node)

    def pump(duration):
        end, next_lane = time.monotonic() + duration, 0.
        while time.monotonic() < end:
            if time.monotonic() >= next_lane:
                lane = AckermannDriveStamped()
                lane.header.stamp = probe.get_clock().now().to_msg()
                lane.drive.speed, lane.drive.steering_angle = .4, .12
                lane_pub.publish(lane)
                next_lane = time.monotonic() + .02
            executor.spin_once(timeout_sec=.005)

    def light(green, count):
        for _ in range(count):
            lights.publish(Bool(data=green))
            pump(.06)

    def white():
        ok, data = cv2.imencode('.jpg', np.full((240, 320, 3), 255, np.uint8))
        assert ok
        message = CompressedImage(format='jpeg', data=data.tobytes())
        message.header.stamp = slow.get_clock().now().to_msg()
        slow.on_white_image(message)
        pump(.12)

    def activate(kind):
        for _ in range(slow.confirm_frames):
            slow.on_sign(kind, Bool(data=True))
        white()
        assert slow.state.missions[kind].deadline is not None

    try:
        deadline = time.monotonic() + 5.
        while (signal.start_signal_pub.get_subscription_count() < 2
               or lights.get_subscription_count() < 1
               or signal.command_pub.get_subscription_count() < 1
               or slow.command_pub.get_subscription_count() < 1
               or lane_pub.get_subscription_count() < 2):
            assert time.monotonic() < deadline, 'DDS discovery timed out'
            pump(.02)
        activate('slow')
        assert any(abs(m.drive.speed - .2) < 1e-6 for m in slow_commands)
        if bump_active:
            activate('bump')
        # A pending renewal and its camera eligibility must be cancelled as well.
        for _ in range(slow.confirm_frames):
            slow.on_sign('slow', Bool(data=True))
        assert slow.state.missions['slow'].armed and 'slow' in slow.white_arm_stamps
        light(False, 1)
        assert not starts and not stops
        assert slow.state.missions['slow'].deadline is not None
        light(False, 1)
        pump(.12)
        assert len(starts) == 1 and starts[0].data is True
        assert stops and all(m.drive.speed == 0. for m in stops)
        mission = slow.state.missions['slow']
        assert mission.deadline is None and not mission.armed and mission.count == 0
        assert 'slow' not in slow.white_arm_stamps
        light(False, 3)
        assert len(starts) == 1  # Additional red messages keep the same fixed-deadline stop.
        sent = len(slow_commands)
        pump(.18)
        assert len(starts) == 1
        if bump_active:
            assert slow.state.selected(time.monotonic()) == 'bump'
            assert slow.state.missions['bump'].deadline is not None
        else:
            assert len(slow_commands) == sent
        light(True, 2)
        assert not signal.state.stopping and len(starts) == 1
        # A floor image alone, or only one new sign, cannot revive the cancelled mission.
        white()
        assert slow.state.missions['slow'].deadline is None
        slow.on_sign('slow', Bool(data=True))
        white()
        assert slow.state.missions['slow'].deadline is None
        slow.on_sign('slow', Bool(data=True))
        white()
        assert slow.state.missions['slow'].deadline is not None
        deadline = slow.state.missions['slow'].deadline
        slow.on_signal_stop(Bool(data=False))
        assert slow.state.missions['slow'].deadline == deadline
        light(False, 2)
        pump(.10)
        assert len(starts) == 2 and all(m.data for m in starts)
        assert slow.state.missions['slow'].deadline is None
    finally:
        executor.shutdown()
        for node in (signal, slow, probe):
            node.destroy_node()
        context.shutdown()
