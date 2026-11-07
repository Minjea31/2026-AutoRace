"""Gentle curves must relax arrival heading without losing tight-space access."""
from dataclasses import fields, replace
import math
from pathlib import Path

import pytest
import yaml

from autorace_parking.core import CollisionChecker, Gap, Scan, Settings, advance, wrap
from autorace_parking.control import Follower, build_plan_report


def optional_gentle_profile():
    p = Path(__file__).resolve().parents[1]/'config/parking_real.yaml'
    params = yaml.safe_load(p.read_text())['parking_node']['ros__parameters']
    cfg = Settings(**{f.name: params[f.name] for f in fields(Settings) if f.name in params})
    cfg = replace(cfg, steering_response_scale=1.,steering_gain=1.,preferred_turn_radius=.4, max_steering_paths_only=False, max_steering_on_curves=False,
                  min_turn_radius=.2, max_steering=math.pi/4,
                  max_inner_steering=math.atan(.2/(.2-.172/2)), require_goal_heading=False)
    params['max_steering_rate'] = .5
    return cfg, params


def clear_scan():
    return Scan([10.]*361, -math.pi, math.pi/180, .01, 12., (0., 0., 0.))


def peak_curvature(path):
    return max(2*abs(math.sin(wrap(b[2]-a[2])/2))/math.hypot(b[0]-a[0], b[1]-a[1])
               for a, b in zip(path, path[1:]))


@pytest.mark.parametrize('side', [-1, 1])
def test_relaxed_arrival_uses_wider_curve_and_oblique_final_heading(side):
    cfg, _ = optional_gentle_profile()
    target = Gap(1., side*.4, side*math.pi/2, .85, 0, 1)
    scan = clear_scan()
    before = build_plan_report(scan, replace(cfg, preferred_turn_radius=0., require_goal_heading=True), target)
    after = build_plan_report(scan, cfg, target)
    assert before['final'] and after['final']
    assert peak_curvature(after['path']) <= peak_curvature(before['path'])*.51
    assert 1/peak_curvature(after['path']) >= .4
    final = after['path'][-1]
    assert final[:2] == pytest.approx((target.x, target.y))
    assert math.degrees(abs(final[2])) < 60
    assert after['goal'].yaw == pytest.approx(final[2])
    assert all(CollisionChecker(scan, cfg).free(p) for p in after['path'])


def test_explicit_heading_requirement_still_reaches_the_requested_direction():
    cfg, _ = optional_gentle_profile()
    cfg = replace(cfg, require_goal_heading=True)
    report = build_plan_report(clear_scan(), cfg, Gap(1., -.4, -math.pi/2, .85, 0, 1))
    assert report['final']
    assert report['path'][-1][2] == pytest.approx(-math.pi/2)


def test_twenty_cm_turn_remains_available_when_wider_arcs_do_not_fit():
    cfg, _ = optional_gentle_profile()
    report = build_plan_report(clear_scan(), cfg, Gap(.2, -.2, -math.pi/2, .85, 0, 1))
    assert report['final'] and report['path'][-1][:2] == pytest.approx((.2, -.2))
    assert peak_curvature(report['path']) == pytest.approx(5.)


@pytest.mark.parametrize('side', [-1, 1])
def test_gentle_real_profile_tracks_parks_and_reverses_its_recorded_turn(side):
    cfg, p = optional_gentle_profile()
    scan = clear_scan()
    report = build_plan_report(scan, cfg, Gap(1., side*.4, side*math.pi/2, .85, 0, 1))
    follower = Follower(cfg, speed=p['parking_speed'], lookahead=p['lookahead'],
                        hold_seconds=p['parking_hold_seconds'],
                        max_yaw_rate=p['max_yaw_rate'],
                        require_parking_heading=p['require_parking_heading'],
                        parking_position_tolerance=p['parking_position_tolerance'],
                        max_steering_rate=p['max_steering_rate'], min_turn_speed=p['min_turn_speed'])
    follower.accept(report['goal'], report['path'], report['final'], (0., 0., 0.))
    pose = (0., 0., 0.)
    previous = None
    for i in range(1800):
        speed, steering, state = follower.command(scan, pose, new_scan=i % 2 == 0, now=i*.05)
        assert abs(steering) <= cfg.steering_limit+1e-9
        if previous is not None and speed and previous[0]:
            assert abs(steering-previous[1]) <= p['max_steering_rate']*.05+1e-9
        pose = advance(pose, speed*.05, math.tan(steering)/cfg.wheelbase)
        previous = speed, steering
        if follower.exited:
            break
    assert follower.parked and follower.exited
