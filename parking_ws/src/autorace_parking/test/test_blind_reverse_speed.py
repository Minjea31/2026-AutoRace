"""User-cleared blind reverse is bounded; turning cruise must not creep."""
from concurrent.futures import Future
from dataclasses import replace
import math

import pytest

from autorace_parking.core import CollisionChecker, Gap, Scan, Settings, advance
from autorace_parking.control import Follower, build_plan_report
from autorace_parking.diagnostics import planning_text


def front_scan():
    return Scan([10.]*221, math.radians(-110), math.pi/180, .01, 12., (0., 0., 0.))


def full_scan():
    return Scan([10.]*361, -math.pi, math.pi/180, .01, 12., (0., 0., 0.))


def blind_config():
    return replace(Settings(), allow_reverse=True, initial_clear_radius=0.,
                   blind_reverse_distance=.25, recovery_speed=.2)


def test_explicitly_cleared_rear_allows_only_a_bounded_straight_reverse():
    cfg = blind_config()
    report = build_plan_report(front_scan(), cfg)
    assert report['diagnostics']['reason_code'] == 'recovery_ready'
    assert report['goal'] is None and not report['final']
    assert .20 <= -report['path'][-1][0] <= .25
    assert all(y == yaw == 0 for x, y, yaw in report['path'])
    assert report['diagnostics']['blind_reverse_distance_m'] <= .25
    # The permission is limited to recovery, rather than making rear space visible.
    assert not CollisionChecker(front_scan(), cfg).free((-.2, 0., 0.))
    assert not build_plan_report(front_scan(), cfg, blind_reverse_remaining=0.)['path']


def test_measured_obstacles_override_the_blind_reverse_permission():
    cfg = blind_config()
    scan = front_scan(); scan.points.append((-.15, 0.))
    report = build_plan_report(scan, cfg)
    assert not report['path']
    scan = front_scan(); scan.points.append((-.3, 0.))
    report = build_plan_report(scan, cfg)
    assert not report['path'] or report['path'][-1][0] > -.1


def test_recovery_uses_point_two_cruise_and_stops_after_about_twenty_cm():
    cfg = blind_config(); scan = front_scan()
    report = build_plan_report(scan, cfg)
    follower = Follower(cfg, speed=.5, require_parking_heading=False, min_turn_speed=.2)
    follower.accept(None, report['path'], False, (0., 0., 0.), recovery=True)
    pose = (0., 0., 0.); speeds = []
    for i in range(300):
        speed, steering, state = follower.command(scan, pose, new_scan=i % 2 == 0, now=i*.05)
        speeds.append(speed)
        pose = advance(pose, speed*.05, math.tan(steering)/cfg.wheelbase)
        if state == 'REOBSERVE':
            break
    assert min(speeds) == pytest.approx(-.2)
    assert -.25 <= pose[0] <= -.19
    assert state == 'REOBSERVE' and not follower.parked
    follower.accept(None, report['path'], False, pose, recovery=True)
    blocked = front_scan(); blocked.points.append((-.3, 0.))
    assert follower.command(blocked, pose, new_scan=True, now=20.)[0] == 0.


def test_turning_speed_floor_replaces_steering_settling_creep():
    cfg = replace(Settings(), max_steering=math.pi/4, min_turn_radius=.2)
    path = [advance((0., 0., 0.), .6*math.pi/2*i/60, -1/.6) for i in range(61)]
    target = Gap(*path[-1], .85, 0, 1)
    before = Follower(cfg, speed=.5, max_steering_rate=.5)
    after = Follower(cfg, speed=.5, max_steering_rate=.5, min_turn_speed=.2)
    for follower in (before, after):follower.accept(target, path, True, (0., 0., 0.))
    assert 0 < before.command(full_scan(), (0., 0., 0.), now=0.)[0] < .02
    assert after.command(full_scan(), (0., 0., 0.), now=0.)[0] == pytest.approx(.2)


def test_final_braking_and_arrival_still_override_the_turn_speed_floor():
    cfg = Settings()
    follower = Follower(cfg, speed=.5, parking_position_tolerance=.03, min_turn_speed=.2)
    follower.accept(Gap(.15, 0., 0., .85, 0, 1), [(i*.01, 0., 0.) for i in range(16)], True, (0., 0., 0.))
    speed = follower.command(full_scan(), (.1, 0., 0.), new_scan=True, now=0.)[0]
    assert 0 < speed < .2
    assert follower.command(full_scan(), (.15, 0., 0.), new_scan=True, now=.1)[0] == 0.


def test_recovery_message_no_longer_requests_unsupported_360_degree_lidar():
    text = planning_text({'reason_code':'no_observed_path', 'candidates':1,
                          'recovery_blocked_by':'unknown_space','scan_fov_deg':220.,
                          'blind_reverse_remaining_m':0.})
    assert '360' not in text and '잔여: 0cm' in text


def test_ros_acceptance_spends_budget_without_inventing_an_observed_corridor():
    rclpy = pytest.importorskip('rclpy')
    from rclpy.parameter import Parameter
    from autorace_parking.node import ParkingNode
    rclpy.init()
    node = ParkingNode(parameter_overrides=[Parameter('localization_source', value='scan'),
                                           Parameter('allow_reverse', value=True),
                                           Parameter('blind_reverse_distance', value=.25)])
    try:
        node.activation.active = True
        node.scan=front_scan()
        report = build_plan_report(front_scan(), node.cfg, blind_reverse_remaining=node.blind_reverse_remaining)
        future = Future(); future.set_result(report)
        node.pending = future, node.epoch, (0., 0., 0.), node.now_seconds()
        node.poll_plan()
        assert 0 <= node.blind_reverse_remaining < .02
        assert not node.observed_corridor
        assert not build_plan_report(front_scan(), node.cfg, origin=(-.24, 0., 0.),
                                     blind_reverse_remaining=node.blind_reverse_remaining)['path']
        node.reset_motion()
        assert node.blind_reverse_remaining == .25
    finally:
        node.activation.active = False
        node.destroy_node(); rclpy.shutdown()
