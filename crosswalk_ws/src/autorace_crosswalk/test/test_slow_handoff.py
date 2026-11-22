"""Actual DDS handoff: emit only when stopping starts, then cancel slow output."""
import os
from pathlib import Path
import sys
import time
from uuid import uuid4

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool

from autorace_crosswalk.node import CrosswalkNode

sys.path.insert(0, str(Path(__file__).resolve().parents[4] / 'slow_ws/src/autorace_slow'))
from autorace_slow.node import SlowNode


def test_stop_start_signal_cancels_slow_via_dds():
    context = Context()
    rclpy.init(context=context, domain_id=180 + os.getpid() % 20,
               signal_handler_options=SignalHandlerOptions.NO)
    prefix = '/handoff_' + uuid4().hex
    args = ['--ros-args', '-r', '/start_crosswalk:=' + prefix + '/start',
            '-r', '/mission/crosswalk/command_01:=' + prefix + '/stop',
            '-r', '/mission/slow/command_04:=' + prefix + '/slow',
            '-r', '/mission/lane:=' + prefix + '/lane',
            '-r', '/camera/color/image_raw/compressed:=' + prefix + '/camera',
            '-r', '/yolo/crosswalk_sign:=' + prefix + '/crosswalk_sign',
            '-r', '/yolo/slow_sign:=' + prefix + '/slow_sign',
            '-r', '/yolo/bump_sign:=' + prefix + '/bump_sign']
    crosswalk = CrosswalkNode(context=context, cli_args=args)
    slow = SlowNode(context=context, cli_args=args, parameter_overrides=[
        Parameter('white_roi_enabled', value=False)])
    probe = Node('handoff_probe', context=context)
    starts, stops, slow_commands = [], [], []
    subscriptions = [
        probe.create_subscription(Bool, prefix + '/start', starts.append, 10),
        probe.create_subscription(AckermannDriveStamped, prefix + '/stop', stops.append, 10),
        probe.create_subscription(AckermannDriveStamped, prefix + '/slow', slow_commands.append, 10)]
    executor = SingleThreadedExecutor(context=context)
    for node in (crosswalk, slow, probe):
        executor.add_node(node)

    def pump(duration):
        until = time.monotonic() + duration
        while time.monotonic() < until:
            executor.spin_once(timeout_sec=.005)

    try:
        end = time.monotonic() + 5.
        while (crosswalk.start_crosswalk_pub.get_subscription_count() < 2
               or crosswalk.command_pub.get_subscription_count() < 1
               or slow.command_pub.get_subscription_count() < 1):
            assert time.monotonic() < end
            pump(.02)
        lane = AckermannDriveStamped()
        lane.header.stamp = slow.get_clock().now().to_msg()
        lane.drive.speed = .4
        slow.on_lane(lane)
        slow.on_sign('slow', Bool(data=True))
        slow.on_sign('slow', Bool(data=True))
        slow.publish()
        pump(.07)
        assert any(abs(m.drive.speed - .2) < 1e-6 for m in slow_commands)
        crosswalk.state.receive(True)
        crosswalk.state.receive(True)
        crosswalk.publish()
        pump(.05)
        assert not starts
        crosswalk.state.receive_white(True, time.monotonic())
        crosswalk.publish()
        pump(.05)
        assert crosswalk.state.phase == 'ready' and not starts
        # No lane: neither the stop nor its event can start yet.
        crosswalk.on_lane(lane)
        crosswalk.publish()
        pump(max(0.,crosswalk.state.ready_at-time.monotonic()-.06))
        assert not starts and not stops
        assert slow.state.missions['slow'].deadline is not None
        pump(.20)
        assert len(starts) == 1 and starts[0].data is True
        assert stops and all(m.drive.speed == 0. for m in stops)
        assert slow.state.missions['slow'].deadline is None
        # Drain DDS deliveries and prove silence while the stop keeps publishing.
        pump(.1)
        count = len(slow_commands)
        for _ in range(3):
            crosswalk.publish()
            slow.publish()
            pump(.05)
        assert len(starts) == 1
        assert len(slow_commands) == count
    finally:
        executor.shutdown()
        for node in (crosswalk, slow, probe):
            node.destroy_node()
        context.shutdown()
