"""Departure handoff validation and ROS arbitration on a private DDS domain."""
from concurrent.futures import Future
import time

import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.signals import SignalHandlerOptions
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float32MultiArray, Header

from autorace_parking.lane_handoff import ExitLaneGate
from autorace_parking.node import ParkingNode
from autorace_mux.node import MuxNode

VALID = [1., 0., 0., 1., 0., .9, 2., 0., 0., 0., .3]


def test_only_fresh_observations_after_actual_reverse_can_release():
    gate = ExitLaneGate()
    gate.observe_target(VALID, 1.)
    gate.observe_command(.3, .1, 10_000_000_000, 1.)
    gate.begin(10_000_000_000, 1.)
    assert not gate.ready(10_100_000_000, 1.1)
    gate.observe_target(VALID, 1.1)
    gate.observe_command(.3, .1, 10_100_000_000, 1.1)
    assert gate.ready(10_100_000_000, 1.1)
    assert not gate.ready(10_400_000_000, 1.4)
    assert not gate.ready(9_900_000_000, 1.1)
    gate.reset()
    assert not gate.ready(10_100_000_000, 1.1)


@pytest.mark.parametrize('values', [[], [0.] * 8,
    [1., 0., 0., 1., 0., .9, 1., 1.],  # held boundaries only
    [1., 0., 0., 1., 0., .05, 2., 0.],
    [1., float('nan'), 0., 1., 0., .9, 2., 0.]])
def test_invalid_low_confidence_and_held_targets_do_not_release(values):
    gate = ExitLaneGate()
    gate.begin(10_000_000_000, 1.)
    gate.observe_command(.3, .1, 10_100_000_000, 1.1)
    gate.observe_target(values, 1.1)
    assert not gate.ready(10_100_000_000, 1.1)


@pytest.mark.parametrize('speed,steering,stamp', [(0., .1, 10_100_000_000),
    (-.2, .1, 10_100_000_000), (.3, float('nan'), 10_100_000_000),
    (.3, .1, 10_000_000_000), (.3, .1, 11_000_000_000)])
def test_invalid_stopped_old_or_future_lane_commands_do_not_release(speed, steering, stamp):
    gate = ExitLaneGate()
    gate.begin(10_000_000_000, 1.)
    gate.observe_target(VALID, 1.1)
    gate.observe_command(speed, steering, stamp, 1.1)
    assert not gate.ready(10_100_000_000, 1.1)


class Harness:
    def __init__(self):
        self.context = Context()
        rclpy.init(context=self.context, domain_id=218, signal_handler_options=SignalHandlerOptions.NO)
        self.parking = ParkingNode(context=self.context, parameter_overrides=[
            Parameter('localization_source', value='scan'), Parameter('white_roi_enabled', value=False)])
        self.parking.control_timer.cancel()
        self.mux = MuxNode(context=self.context, parameter_overrides=[
            Parameter('output_topic', value='/handoff_test/cmd_vel')])
        self.probe = Node('exit_handoff_test_probe', context=self.context)
        self.lane_pub = self.probe.create_publisher(AckermannDriveStamped, '/mission/lane', 1)
        self.target_pub = self.probe.create_publisher(Float32MultiArray, '/lane_fusion/target', 1)
        self.releases = []
        self.release_sub = self.probe.create_subscription(Header, '/parking/lane_handoff', self.releases.append, 1)
        self.executor = SingleThreadedExecutor(context=self.context)
        for n in (self.parking, self.mux, self.probe):
            self.executor.add_node(n)
        end = time.monotonic() + 3.
        while (self.lane_pub.get_subscription_count() < 2 or self.target_pub.get_subscription_count() < 1
               or self.parking.command_pub.get_subscription_count() < 1
               or self.parking.lane_handoff_pub.get_subscription_count() < 2):
            assert time.monotonic() < end
            self.pump(.02)

    def lane(self):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.probe.get_clock().now().to_msg()
        msg.drive.speed, msg.drive.steering_angle = .4, .1
        self.lane_pub.publish(msg)

    def pump(self, duration):
        end, next_lane = time.monotonic() + duration, 0.
        while time.monotonic() < end:
            if time.monotonic() >= next_lane:
                self.lane()
                next_lane = time.monotonic() + .02
            self.executor.spin_once(timeout_sec=.003)

    def close(self):
        self.executor.shutdown()
        for n in (self.parking, self.mux, self.probe):
            n.destroy_node()
        self.context.shutdown()


@pytest.fixture
def h():
    harness = Harness()
    try:
        yield harness
    finally:
        harness.close()


def test_reverse_departure_releases_mux_immediately_and_completion_stays_terminal(h):
    n = h.parking
    n.activation.active = True
    n.follower.parked = True
    n.follower.path = [(0., 0., 0.), (-1., 0., 0.)]
    n.had_control_path = True
    h.target_pub.publish(Float32MultiArray(data=VALID))
    n.publish_command(-.2, .1)  # parked hold / an entry reverse must not arm the gate
    h.pump(.06)
    assert not n.follower.exited and n.exit_lane_gate.started_ros_ns is None
    n.follower.exiting = True
    n.publish_command(-.2, .1)
    h.pump(.05)
    assert h.mux._last_source == 'parking'
    assert not n.follower.exited  # old detection must not be reused
    n.pending = (Future(), n.epoch, (0., 0., 0.), 1.)
    n.exit_pending = (Future(), n.epoch, n.follower, (0., 0., 0.))
    started = time.monotonic()
    h.target_pub.publish(Float32MultiArray(data=VALID))
    h.pump(.10)
    assert time.monotonic() - started < h.mux.config.input_timeout_sec
    assert n.follower.exited and n.exit_lane_handed_off and not n.follower.path
    assert n.pending is None and n.exit_pending is None
    assert h.mux._last_source == 'lane' and len(h.releases) == 1
    assert not n.had_control_path
    for _ in range(3):
        n.tick()
    h.pump(.05)
    assert n.state == 'EXITED' and h.mux._last_source == 'lane' and len(h.releases) == 1


def test_delayed_parking_packets_are_ignored_and_next_mission_can_take_control(h):
    mux = h.mux
    h.pump(.08)
    now = h.probe.get_clock().now().to_msg()
    old = AckermannDriveStamped()
    old.header.stamp = now
    old.drive.speed = -.2
    mux._receive('parking', old)
    mux._publish()
    assert mux._last_source == 'parking'
    mux._release_parking(Header(stamp=h.probe.get_clock().now().to_msg()))
    assert mux._last_source == 'lane'
    mux._receive('parking', old)
    mux._publish()
    assert mux._last_source == 'lane'
    h.pump(.01)
    new = AckermannDriveStamped()
    new.header.stamp = h.probe.get_clock().now().to_msg()
    new.drive.speed = -.1
    mux._receive('parking', new)
    mux._publish()
    assert mux._last_source == 'parking'


@pytest.mark.parametrize('parked,exiting,recovering', [(False, False, True),
    (False, True, False), (True, False, False)])
def test_entry_recovery_and_parked_hold_do_not_handoff(h, parked, exiting, recovering):
    n = h.parking
    n.activation.active = True
    n.follower.parked, n.follower.exiting, n.follower.recovering = parked, exiting, recovering
    n.publish_command(-.2, .1)
    h.pump(.05)
    h.target_pub.publish(Float32MultiArray(data=VALID))
    h.pump(.06)
    assert not n.follower.exited and n.exit_lane_gate.started_ros_ns is None
    assert not h.releases and h.mux._last_source == 'parking'


def test_stale_future_and_late_release_do_not_cancel_a_newer_mission(h):
    mux = h.mux
    command = AckermannDriveStamped()
    command.header.stamp = h.probe.get_clock().now().to_msg()
    command.drive.speed = -.2
    mux._receive('parking', command)
    mux._publish()
    for delta in (-1_000_000_000, 1_000_000_000):
        event = Header()
        ns = h.probe.get_clock().now().nanoseconds + delta
        event.stamp.sec, event.stamp.nanosec = divmod(ns, 1_000_000_000)
        mux._release_parking(event)
        assert mux._last_source == 'parking' and mux._parking_release_stamp == 0
    # A release from an older mission may arrive after the next mission command.
    event = Header()
    ns = command.header.stamp.sec * 1_000_000_000 + command.header.stamp.nanosec - 1
    event.stamp.sec, event.stamp.nanosec = divmod(ns, 1_000_000_000)
    mux._release_parking(event)
    assert mux._last_source == 'parking'


def test_release_preserves_other_mission_priority_and_clock_reset_allows_new_parking(h):
    mux = h.mux
    h.pump(.04)
    command = AckermannDriveStamped()
    command.header.stamp = h.probe.get_clock().now().to_msg()
    command.drive.speed = 0.
    mux._receive('signal', command)
    mux._release_parking(Header(stamp=h.probe.get_clock().now().to_msg()))
    assert mux._last_source == 'signal'
    mux.arbitrator.discard('signal')
    now = h.probe.get_clock().now().nanoseconds
    mux._parking_release_stamp = now + 1_000_000_000
    mux._last_release_clock = now + 2_000_000_000
    command.header.stamp = h.probe.get_clock().now().to_msg()
    command.drive.speed = -.1
    mux._receive('parking', command)
    mux._publish()
    assert mux._parking_release_stamp == 0 and mux._last_source == 'parking'


@pytest.mark.parametrize('field', ['steering_angle_velocity', 'acceleration', 'jerk'])
def test_lane_command_rejected_by_mux_cannot_complete_departure(h, field):
    n = h.parking
    n.activation.active = True
    n.follower.parked = n.follower.exiting = True
    n.publish_command(-.2, .1)
    n.on_exit_lane_target(Float32MultiArray(data=VALID))
    command = AckermannDriveStamped()
    command.header.stamp = h.probe.get_clock().now().to_msg()
    command.drive.speed, command.drive.steering_angle = .4, .1
    setattr(command.drive, field, float('nan'))
    n.on_exit_lane_command(command)
    assert not n.follower.exited and n.exit_lane_gate.command is None
