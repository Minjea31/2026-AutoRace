"""Maximum-angle reference paths, left-rear recovery and LDF visualization."""
from dataclasses import fields, replace
import math
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml

from autorace_parking.core import CollisionChecker, Gap, Scan, Settings, advance, sample_candidate, wrap
from autorace_parking.control import Follower, build_plan_report


def real_profile():
    params = yaml.safe_load((Path(__file__).resolve().parents[1]/'config/parking_real.yaml').read_text())['parking_node']['ros__parameters']
    return Settings(**{f.name: params[f.name] for f in fields(Settings) if f.name in params}), params


def clear_scan():
    return Scan([math.inf]*361, -math.pi, math.pi/180, .01, 12., (0., 0., 0.))


def front_scan():
    return Scan([10.]*221, math.radians(-110), math.pi/180, .01, 12., (0., 0., 0.))


def optional_twenty_cm_max_profile():
    cfg, params = real_profile()
    return replace(cfg, steering_response_scale=1.,steering_gain=1.,min_turn_radius=.2, max_steering=math.pi/4,
                   max_inner_steering=math.atan(.2/(.2-.172/2)), preferred_turn_radius=0.,
                   max_steering_paths_only=True, max_turn_arc_length=.4), params


def assert_max_angle_arcs(path, cfg):
    run, previous = 0., 0.
    for a, b in zip(path, path[1:]):
        chord = math.hypot(b[0]-a[0], b[1]-a[1])
        yaw = wrap(b[2]-a[2])
        if abs(yaw) < 1e-7:
            run, previous = 0., 0.
            continue
        curvature = 2*math.sin(yaw/2)/chord
        assert abs(curvature) == pytest.approx(1/cfg.turn_radius, abs=1e-6)
        # Signed geometric curvature also includes reverse motion; magnitude is exact.
        arc = abs(yaw)*cfg.turn_radius
        sign = 1 if curvature > 0 else -1
        run = run+arc if sign == previous else arc
        previous = sign
        assert run <= cfg.max_turn_arc_length+1e-7


@pytest.mark.parametrize('side', [-1, 1])
def test_optional_paths_use_only_straights_and_maximum_steering_arcs(side):
    cfg, _ = optional_twenty_cm_max_profile()
    assert cfg.max_steering_paths_only and cfg.turn_radius == pytest.approx(.2)
    report = build_plan_report(clear_scan(), cfg, Gap(1., side*.4, side*math.pi/2, .85, 0, 1))
    assert report['final'] and report['path']
    assert_max_angle_arcs(report['path'], cfg)


def test_optional_forty_cm_arc_limit_is_enforced_before_accepting_a_candidate():
    cfg, _ = optional_twenty_cm_max_profile()
    checker = CollisionChecker(clear_scan(), cfg)
    assert sample_candidate(checker, [(.4, 1/cfg.turn_radius)], cfg)[1]
    assert not sample_candidate(checker, [(.42, 1/cfg.turn_radius)], cfg)[1]
    assert not sample_candidate(checker, [(.2, .5/cfg.turn_radius)], cfg)[1]


def test_right_bay_recovery_goes_straight_then_left_rear():
    cfg, _ = real_profile();cfg = replace(cfg, steering_response_scale=1.,initial_clear_radius=0., recovery_left_when_goal_right=True,
                                               recovery_on_front_obstacle_only=False, assume_unobserved_free=False)
    report = build_plan_report(front_scan(), cfg, Gap(1., -.5, -math.pi/2, .85, 0, 1))
    assert report['diagnostics']['recovery_mode'] == 'straight_then_left_reverse'
    assert report['diagnostics']['reason_code'] == 'recovery_ready'
    end = report['path'][-1]
    assert end[0] < -.15 and end[1] > .03 and end[2] < 0
    assert all(abs(y) < 1e-9 and abs(yaw) < 1e-9 for x, y, yaw in report['path'][:6])
    assert_max_angle_arcs(report['path'], cfg)
    blocked = front_scan();blocked.points.append((-.15, 0.))
    assert not build_plan_report(blocked, cfg, Gap(1., -.5, -math.pi/2, .85, 0, 1))['path']


@pytest.mark.parametrize('side', [-1, 1])
def test_real_recovery_profile_backs_straight_thirty_cm_on_either_side(side):
    cfg, _ = real_profile();cfg = replace(cfg, initial_clear_radius=0.,clearance=.04)
    assert not cfg.recovery_left_when_goal_right
    assert cfg.recovery_speed == .12 and cfg.blind_reverse_distance == .3
    scan = front_scan();scan.points.append((cfg.vehicle_length/2+.08,0.))
    report = build_plan_report(scan, cfg, Gap(1., side*.5, side*math.pi/2, .85, 0, 1))
    assert report['diagnostics']['recovery_mode'] == 'straight_reverse'
    assert report['path'][-1][0] == pytest.approx(-.3)
    assert all(y == yaw == 0. for x, y, yaw in report['path'])


def test_real_profile_limits_steering_reversal_and_settling_speed():
    cfg, p = real_profile()
    assert p['max_steering_rate'] == 1.2 and cfg.respect_steering_settle_speed
    follower = Follower(cfg, speed=.5, lookahead=p['lookahead'],
                        max_steering_rate=p['max_steering_rate'], min_turn_speed=p['min_turn_speed'])
    path = [advance((0., 0., 0.), cfg.turn_radius*math.pi/2*i/60, -1/cfg.turn_radius) for i in range(61)]
    follower.accept(Gap(*path[-1], .85, 0, 1), path, True, (0., 0., 0.))
    previous=0.
    for i in range(12):
        speed,steering,_=follower.command(clear_scan(),(0.,0.,0.),now=i*.05)
        assert abs(steering-previous)<=p['max_steering_rate']*.05+1e-8
        assert 0<speed<=.5
        if i==0:assert speed<.1
        previous=steering
    assert steering==pytest.approx(-math.atan(cfg.wheelbase/cfg.turn_radius),abs=2e-4)


def test_ldf_tracks_actual_pursuit_radius_and_selected_point():
    cfg, p = real_profile()
    follower = Follower(cfg, speed=.5, lookahead=p['lookahead'])
    path = [(i*.01, 0., 0.) for i in range(101)]
    follower.accept(Gap(1., 0., 0., .85, 0, 1), path, True, (0., 0., 0.))
    follower.command(clear_scan(), (0., 0., 0.), new_scan=True, now=0.)
    assert follower.lookahead_distance == p['lookahead']
    assert follower.lookahead_point[0] >= follower.lookahead_distance
    follower.command(clear_scan(), (.025, 0., 0.), new_scan=True, now=.05)
    assert follower.lookahead_distance == pytest.approx(p['lookahead']+.1)
    assert follower.lookahead_point[0] >= follower.lookahead_distance
    follower.stop('PLANNING', .1)
    assert follower.lookahead_point is None


def test_ros_ldf_topic_and_rviz_markers_match_the_actual_controller_target():
    rclpy = pytest.importorskip('rclpy')
    from rclpy.parameter import Parameter
    from autorace_parking.node import ParkingNode
    rclpy.init()
    node = ParkingNode(parameter_overrides=[Parameter('localization_source', value='scan'),
                                           Parameter('lookahead', value=.25)])
    try:
        distances, batches = [], []
        node.ldf_pub = SimpleNamespace(publish=distances.append)
        node.lookahead_pub = SimpleNamespace(publish=batches.append)
        node.activation.active = True
        node.scan = clear_scan()
        node.follower.accept(Gap(1., 0., 0., .85, 0, 1), [(i*.01, 0., 0.) for i in range(101)], True, (0., 0., 0.))
        node.follower.command(node.scan, (0., 0., 0.), now=0.)
        node.publish_lookahead()
        assert distances[-1].data == .25
        shapes = {m.ns:m for m in batches[-1].markers if m.ns}
        assert set(shapes) == {'ldf_circle','ldf_target','ldf_ray'}
        assert shapes['ldf_target'].pose.position.x == node.follower.lookahead_point[0]
        assert shapes['ldf_target'].header.frame_id == 'base_link'
        assert len(shapes['ldf_circle'].points) == 33
        node.follower.stop('PLANNING', .1)
        node.publish_lookahead()
        assert len(batches[-1].markers) == 1  # stale target is cleared
    finally:
        node.activation.active = False
        node.destroy_node();rclpy.shutdown()
