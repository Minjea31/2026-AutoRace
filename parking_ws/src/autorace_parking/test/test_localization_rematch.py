import json
import math
import time

import pytest
import rclpy
from rclpy.parameter import Parameter
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String

from autorace_parking.node import ParkingNode


@pytest.fixture
def scenario():
    rclpy.init()
    node = ParkingNode(parameter_overrides=[Parameter('localization_source', value='fused')])
    clock = [100.]
    node.now_seconds = lambda: clock[0]
    commands = []
    node.publish_command = lambda v, a: commands.append((v, a))
    try:
        for _ in range(node.required_detections): node.on_sign(Bool(data=True))
        yield node, clock, commands
    finally:
        node.destroy_node(); rclpy.shutdown()


def odom(t, x=0.):
    msg = Odometry()
    ns = round(t*1e9)
    msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(ns, 10**9)
    msg.header.frame_id, msg.child_frame_id = 'parking_odom', 'base_link'
    msg.pose.pose.position.x = x
    msg.pose.pose.orientation.w = 1.
    for i in (0, 7, 35): msg.pose.covariance[i] = .001
    return msg


def health(t):
    return String(data=json.dumps({'stamp': t, 'ready': True}))


def scan(t):
    msg = LaserScan(); ns = round(t*1e9)
    msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(ns, 10**9)
    msg.header.frame_id = 'laser_link'
    msg.angle_min, msg.angle_increment = -math.pi, math.pi/180
    msg.range_min, msg.range_max = .01, 12.
    msg.ranges = [10.]*361
    return msg


def test_stale_ekf_discards_old_frame_and_recovers_with_new_match(scenario):
    node, clock, commands = scenario
    node.on_localization(odom(100.))
    node.on_scan(scan(100.))
    assert node.tracker.valid and node.tracker.matcher is not None
    old_tracker = node.tracker
    node.follower.path = [(0., 0., 0.), (.5, 0., 0.)]
    node.slot_map = {'old': True}
    node.pending_scans.append(scan(100.01))
    node.recovery_attempts, node.recovery_remaining = 2, .7
    node.blind_reverse_remaining = .1
    node.planning_cache = object()
    node.approach_reference.origin = (9., 8., .2)
    node.approach_reference.stamp = 99.
    clock[0] = 100.5
    node.on_fusion_health(health(100.5))
    attempt = node.attempt
    assert not node.localization_ready(clock[0])
    assert node.localization_recovering
    assert not node.odometry.samples and not node.pending_scans
    assert node.tracker is not old_tracker and node.tracker.matcher is None
    assert not node.follower.path and node.slot_map is None and node.planning_cache is None
    assert node.fusion_health is None
    assert node.attempt == attempt and node.activation.active
    assert (node.recovery_attempts, node.recovery_remaining, node.blind_reverse_remaining) == (2, .7, .1)
    assert node.approach_reference.origin == (9., 8., .2)
    assert commands[-1] == (0., 0.)
    cutoff = node.localization_not_before
    node.tick(); node.tick()
    assert node.localization_restart_count == 1 and node.localization_not_before == cutoff
    node.on_localization(odom(100.49)); node.on_scan(scan(100.49)); node.on_fusion_health(health(100.49))
    assert not node.odometry.samples and node.fusion_health is None
    clock[0] = 100.6
    node.on_localization(odom(100.6, 5.))  # A new absolute pose must not match the old frame.
    node.on_fusion_health(health(100.6))
    assert not node.localization_ready(clock[0])  # Requires a NEW aligned scan too.
    node.on_scan(scan(100.6))
    assert node.localization_ready(clock[0]) and not node.localization_recovering
    assert node.tracker.global_pose[0] == 5.
    assert node.tracker.pose == (0., 0., 0.)


@pytest.mark.parametrize('stamp', [99., 101.])
def test_invalid_timestamp_restarts_once_and_keeps_cutoff_fixed(scenario, stamp):
    node, clock, commands = scenario
    node.on_localization(odom(100.)); node.on_scan(scan(100.))
    node.on_localization(odom(stamp))
    assert node.localization_recovering and not node.odometry.samples
    cutoff = node.localization_not_before
    clock[0] += .1
    node.on_localization(odom(stamp))
    node.on_fusion_health(health(stamp))
    assert node.localization_restart_count == 1 and node.localization_not_before == cutoff


def test_health_callback_timeout_also_forces_new_match_and_holds_stop(scenario):
    node, clock, commands = scenario
    node.on_localization(odom(100.)); node.on_scan(scan(100.)); node.on_fusion_health(health(100.))
    node.fusion_health_received = time.monotonic()-1.
    assert not node.localization_ready(clock[0])
    node.tick()
    assert node.localization_recovering and commands[-1] == (0., 0.)
    assert node.activation.active and node.state == 'WAITING_LOCALIZATION'


def test_fresh_callback_after_gap_cannot_reuse_old_matching_keyframe(scenario):
    node, clock, commands = scenario
    node.on_localization(odom(100.)); node.on_scan(scan(100.))
    tracker = node.tracker
    clock[0] = 101.
    node.on_localization(odom(101., 1.))
    assert node.tracker is not tracker and node.tracker.matcher is None
    assert node.localization_recovering
    assert node.odometry.samples[-1].pose[0] == 1.


def test_completed_mission_stays_terminal(scenario):
    node, clock, commands = scenario
    node.follower.exited = True
    tracker = node.tracker
    node.on_localization(odom(99.))
    assert node.follower.exited and node.tracker is tracker
    assert not node.localization_recovering
