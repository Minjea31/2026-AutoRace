"""Regression coverage for measured LIMO Pro parking geometry."""
import json
import math
from dataclasses import replace
from pathlib import Path

import pytest

from autorace_parking.core import (Cluster, CollisionChecker, Scan, Settings, cluster_points,
                                   find_gaps, straight_turn_candidates, wrap)
from autorace_parking.control import Follower, build_plan, build_plan_report, forward_turn_target
from autorace_parking.tracking import compose, inverse


def test_wego_inner_wheel_limit_is_stricter_than_nominal_turning_radius():
    cfg = replace(Settings(), max_steering=math.atan(.2/.4), min_turn_radius=.4,
                  max_inner_steering=.48869, steering_track=.172)
    assert cfg.turn_radius == pytest.approx(.46214728)
    assert cfg.steering_limit == pytest.approx(.408427116, abs=1e-6)
    inner_angle = math.atan(cfg.wheelbase / (cfg.turn_radius-cfg.steering_track/2))
    assert inner_angle <= cfg.max_inner_steering+1e-9
    assert cfg.steering_limit < inner_angle
    assert replace(cfg, min_turn_radius=.6).turn_radius == .6


@pytest.mark.parametrize('origin', [(0., 0., 0.), (.4, -.2, -math.pi/2), (.4, -.7, math.pi-.01)])
def test_front_face_target_depth_remains_in_the_parking_row_frame(origin):
    front_faces = [Cluster([(.55, -.27), (.65, -.27), (.65, -.35), (.55, -.35)]),
                   Cluster([(1.43, -.29), (1.57, -.29), (1.57, -.46), (1.43, -.46)])]
    local = [Cluster([compose(inverse(origin), (x, y, 0.))[:2] for x, y in group.points])
             for group in front_faces]
    cfg = replace(Settings(), row_yaw=-origin[2], goal_from_observed_front=True)
    gap = find_gaps(local, cfg, origin)[0]
    goal = compose(origin, (gap.x, gap.y, gap.yaw))
    assert goal == pytest.approx((1.04, -.491, -math.pi/2))
    # The rear of the perpendicular body plus clearance is behind both faces.
    assert goal[1]+cfg.vehicle_length/2+cfg.clearance <= -.29+1e-9


@pytest.mark.parametrize('radius', [.2, .46214728])
@pytest.mark.parametrize('sample', range(6))
def test_real_limo_front_faces_require_a_retained_parking_pair(sample, radius):
    from dataclasses import fields
    import yaml
    package = Path(__file__).resolve().parents[1]
    params = yaml.safe_load((package/'config/parking_real.yaml').read_text())['parking_node']['ros__parameters']
    cfg = Settings(**{f.name: params[f.name] for f in fields(Settings) if f.name in params})
    cfg = replace(cfg, min_turn_radius=radius, blind_reverse_distance=0.,
                  max_turn_arc_length=max(cfg.max_turn_arc_length,radius*math.pi/2+.01))
    raw = json.loads((package/'test/data/limo_pro_stationary_scans.json').read_text())['scans'][sample]
    scan = Scan(**raw)
    filtered = scan.filtered_obstacles(cfg)
    retained = find_gaps(cluster_points(filtered.points), cfg)
    report = build_plan_report(scan,cfg)
    goal,path,final = report['goal'],report['path'],report['final']
    if not retained:
        # The foreground sign was the only apparent pair in these older views.
        # Removing it must not leave its former fake bay available for parking.
        assert filtered.ignored_small_clusters > 0
        assert goal is None and path == [] and not final
        return
    shallow = replace(cfg, goal_from_observed_front=False)
    gap = find_gaps(cluster_points(filtered.points), shallow)[0]
    assert gap.y < 0
    if -gap.y < cfg.turn_radius:
        assert not straight_turn_candidates(gap, shallow)
    if goal is None:
        # An eligible pair does not guarantee a collision-free Ackermann route.
        assert not path and not final
        assert report['diagnostics']['reason_code'] in ('start_blocked','no_observed_path','budget_exceeded')
        return
    assert goal.y < 0
    assert any(abs(goal.x-candidate.x)<1e-6 for candidate in retained)
    assert len(path) > 10
    assert path[-1][0] > .13
    checker = CollisionChecker(scan, cfg)
    assert all(checker.free(pose) for pose in path)
    for a, b in zip(path, path[1:]):
        distance = math.hypot(b[0]-a[0], b[1]-a[1])
        assert 2*abs(math.sin(wrap(b[2]-a[2])/2))/distance <= 1/cfg.turn_radius+1e-8
    follower = Follower(cfg, speed=.5, require_parking_heading=False, max_steering_rate=.8)
    follower.accept(goal, path, final, (0., 0., 0.))
    speed, steering, state = follower.command(scan, (0., 0., 0.), now=0.)
    assert speed > 0 and state in ('APPROACHING','PARKING')
    assert abs(steering) <= cfg.steering_limit


@pytest.mark.parametrize('heading', [-math.pi/2, -1.2, -.8])
def test_shallow_target_fits_the_turn_without_changing_bay_alignment(heading):
    from autorace_parking.core import Gap
    cfg = replace(Settings(), min_turn_radius=.5)
    goal = Gap(1., -.1, heading, .8, 0, 1)
    target = forward_turn_target(goal, cfg)
    dx, dy = target.x-goal.x, target.y-goal.y
    assert dx*math.cos(heading)+dy*math.sin(heading) > 0
    assert dx*math.sin(heading)-dy*math.cos(heading) == pytest.approx(0.)
    assert straight_turn_candidates(target, cfg)
    assert forward_turn_target(target, cfg).x == pytest.approx(target.x)
    assert forward_turn_target(target, cfg).y == pytest.approx(target.y)


@pytest.mark.parametrize('side', [-1, 1])
def test_explicit_twenty_cm_option_generates_and_tracks_tight_turns(side):
    from dataclasses import fields
    import yaml
    from autorace_parking.core import Gap, advance
    package = Path(__file__).resolve().parents[1]
    params = yaml.safe_load((package/'config/parking_real.yaml').read_text())['parking_node']['ros__parameters']
    cfg = Settings(**{f.name: params[f.name] for f in fields(Settings) if f.name in params})
    cfg = replace(cfg, steering_response_scale=1.,steering_gain=1.,min_turn_radius=.2, max_steering=math.pi/4,
                  max_inner_steering=math.atan(.2/(.2-.172/2)))
    assert cfg.turn_radius == pytest.approx(.2)
    assert cfg.steering_limit == pytest.approx(math.pi/4)
    scan = Scan([10.]*361, -math.pi, math.pi/180, .01, 12., (0., 0., 0.))
    target = Gap(.2, side*.2, side*math.pi/2, .8, 0, 1)
    goal, path, final = build_plan(scan, cfg, goal=target)
    assert final and len(path) > 10
    curvatures = []
    for a, b in zip(path, path[1:]):
        distance = math.hypot(b[0]-a[0], b[1]-a[1])
        curvatures.append(2*abs(math.sin(wrap(b[2]-a[2])/2))/distance)
    assert max(curvatures) == pytest.approx(5.)
    assert all(CollisionChecker(scan, cfg).free(p) for p in path)
    follower = Follower(cfg, speed=params['parking_speed'],
                        require_parking_heading=params['require_parking_heading'],
                        parking_position_tolerance=params['parking_position_tolerance'],
                        max_steering_rate=params['max_steering_rate'])
    follower.accept(goal, path, final, (0., 0., 0.))
    pose = (0., 0., 0.)
    for i in range(1200):
        speed, steering, state = follower.command(scan, pose, new_scan=i % 2 == 0, now=i*.05)
        assert abs(steering) <= cfg.steering_limit+1e-9
        pose = advance(pose, speed*.05, math.tan(steering)/cfg.wheelbase)
        if follower.parked:
            break
    assert follower.parked
    assert math.hypot(pose[0]-goal.x, pose[1]-goal.y) <= params['parking_position_tolerance']
