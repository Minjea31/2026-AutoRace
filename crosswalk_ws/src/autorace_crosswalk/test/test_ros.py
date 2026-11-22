import copy
import os
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
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool

from autorace_crosswalk.node import CrosswalkNode


def test_dds_stop_cooldown_and_new_sign_and_floor_required_to_repeat():
    context = Context()
    rclpy.init(context=context, domain_id=180 + os.getpid() % 20,
               signal_handler_options=SignalHandlerOptions.NO)
    prefix = '/crosswalk_test_' + uuid4().hex
    node = CrosswalkNode(context=context, cli_args=[
        '--ros-args', '-r', '/yolo/crosswalk_sign:=' + prefix + '/sign',
        '-r', '/mission/lane:=' + prefix + '/lane',
        '-r', '/camera/color/image_raw/compressed:=' + prefix + '/camera',
        '-r', '/mission/crosswalk/command_01:=' + prefix + '/command',
        '-r', '/start_crosswalk:=' + prefix + '/start'])
    probe = Node('crosswalk_probe', context=context)
    lane_pub = probe.create_publisher(AckermannDriveStamped, prefix + '/lane', 1)
    sign_pub = probe.create_publisher(Bool, prefix + '/sign', 1)
    camera_pub = probe.create_publisher(CompressedImage, prefix + '/camera', 1)
    starts = []
    start_subscription = probe.create_subscription(Bool, prefix + '/start', starts.append, 10)
    commands = []
    subscription = probe.create_subscription(
        AckermannDriveStamped, prefix + '/command', commands.append, 10)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    executor.add_node(probe)
    lane = AckermannDriveStamped()
    lane.header.frame_id = 'lane_frame'
    lane.drive.speed, lane.drive.steering_angle = 1.0, .2
    lane.drive.steering_angle_velocity = .3
    lane.drive.acceleration, lane.drive.jerk = .4, .5

    def pump(duration):
        deadline = time.monotonic() + duration
        next_lane = 0.
        while time.monotonic() < deadline:
            if time.monotonic() >= next_lane:
                lane.header.stamp = probe.get_clock().now().to_msg()
                lane_pub.publish(lane)
                next_lane = time.monotonic() + .02
            executor.spin_once(timeout_sec=.005)

    def sign(value):
        sign_pub.publish(Bool(data=value))
        pump(.08)

    def image(white=True, age=0.):
        frame = np.full((200, 400, 3), 255 if white else 0, dtype=np.uint8)
        ok, encoded = cv2.imencode('.jpg', frame)
        assert ok
        message = CompressedImage(format='jpeg', data=encoded.tobytes())
        stamp = probe.get_clock().now().nanoseconds - int(age * 1e9)
        message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
        return message

    def floor(message):
        camera_pub.publish(message)
        pump(.14)

    try:
        deadline = time.monotonic() + 5.
        while (lane_pub.get_subscription_count() < 1
               or sign_pub.get_subscription_count() < 1
               or camera_pub.get_subscription_count() < 1
               or node.command_pub.get_subscription_count() < 1
               or node.start_crosswalk_pub.get_subscription_count() < 1):
            assert time.monotonic() < deadline, 'DDS discovery timed out'
            pump(.02)
        pump(.1)
        old_white = image()
        floor(old_white)
        assert not commands
        sign(True)
        assert not commands
        sign(False)
        sign(True)
        assert not commands
        old_white = image()
        sign(True)
        assert not commands
        assert node.state.phase == 'waiting_white'
        # A pre-confirmation frame remains ineligible even if it is still fresh.
        floor(old_white)
        floor(image(age=1.))
        floor(image(age=-1.))
        broken = image()
        broken.data = b'not an image'
        floor(broken)
        floor(image(white=False))
        assert not commands
        assert not starts
        assert node.stop_delay_sec == 1.5
        assert node.stop_hold_sec == 3.0
        fresh_white = image()
        detected_after = time.monotonic()
        floor(fresh_white)
        delay_end = node.state.ready_at
        assert delay_end >= detected_after + 1.5
        assert not commands and not starts and node.state.phase == 'ready'
        floor(image())
        assert node.state.ready_at == delay_end
        pump(max(0., delay_end - time.monotonic() - .06))
        assert not commands and not starts
        pump(.20)
        assert commands
        assert len(starts) == 1 and starts[0].data is True
        message = commands[-1]
        assert message.drive.speed == 0.
        assert message.drive.steering_angle == pytest.approx(.2)
        assert (message.drive.steering_angle_velocity, message.drive.acceleration,
                message.drive.jerk) == pytest.approx((.3, .4, .5))
        assert message.header.frame_id == 'lane_frame'
        original = copy.deepcopy(lane)
        node.on_lane(lane)
        node.publish()
        assert lane == original
        lane.drive.steering_angle = -.25
        sign(False)
        pump(.15)
        assert commands[-1].drive.steering_angle == pytest.approx(-.25)
        assert node.state.phase == 'stopping'
        # Repeated/new white observations and signs cannot extend the stop.
        started_at = node.state.started_at
        floor(fresh_white)
        floor(image())
        sign(True)
        assert node.state.started_at == started_at
        remaining = node.state.started_at + 3.15 - time.monotonic()
        pump(max(0., remaining))
        assert node.state.phase == 'cooldown'
        assert len(starts) == 1
        # Drain any previously published commands before testing silence.
        pump(.1)
        count = len(commands)
        sign(False)
        sign(True)
        sign(True)
        floor(image())
        pump(.15)
        assert len(commands) == count
        assert len(commands) >= 60
        # Ignore continuous detections throughout the ten-second cooldown.
        while time.monotonic() < node.state.completed_at + 9.5:
            sign(True)
            floor(image())
        count = len(commands)
        cooldown_end = node.state.completed_at + 10.
        pump(max(0., cooldown_end + .1 - time.monotonic()))
        assert node.state.phase == 'waiting'
        assert node.white_arm_stamp is None
        assert len(commands) == count
        # White alone and one sign cannot reuse confirmations from cooldown.
        floor(image())
        sign(True)
        assert node.state.phase == 'waiting'
        assert len(commands) == count
        sign(True)
        assert node.state.phase == 'waiting_white'
        assert len(commands) == count
        floor(image())
        assert node.state.phase == 'ready' and len(commands) == count
        pump(1.55)
        assert node.state.phase == 'stopping'
        assert len(starts) == 2 and all(m.data for m in starts)
        assert len(commands) > count
        assert commands[-1].drive.speed == 0.
        assert commands[-1].drive.steering_angle == pytest.approx(-.25)
    finally:
        executor.shutdown()
        probe.destroy_subscription(subscription)
        probe.destroy_subscription(start_subscription)
        node.destroy_node()
        probe.destroy_node()
        context.shutdown()
