"""Reverse recovery, gear cusps, and rear visibility are motion contracts."""
from dataclasses import replace
import math

import pytest

from autorace_parking.core import CollisionChecker, Gap, Scan, Settings, advance, plan_path
from autorace_parking.control import Follower, build_plan_report


def front_scan():
    return Scan([10.]*221, math.radians(-110), math.pi/180, .01, 12., (0., 0., 0.))


def full_scan():
    return Scan([10.]*361, -math.pi, math.pi/180, .01, 12., (0., 0., 0.))


def test_blind_rear_is_not_a_recovery_route():
    cfg = replace(Settings(), allow_reverse=True, initial_clear_radius=0.)
    report = build_plan_report(front_scan(), cfg, known_free_poses=[(0., 0., 0.)])
    assert not report['path']
    assert report['diagnostics']['recovery_blocked_by'] == 'unknown_space'


def test_visited_corridor_allows_bounded_reverse_and_live_obstacles_win():
    cfg = replace(Settings(), allow_reverse=True, initial_clear_radius=0.)
    known = [(-i/100, 0., 0.) for i in range(41)]
    scan = front_scan()
    report = build_plan_report(scan, cfg, known_free_poses=known)
    assert report['diagnostics']['reason_code'] == 'recovery_ready'
    assert report['goal'] is None and not report['final']
    assert .20 <= -report['path'][-1][0] <= cfg.recovery_reverse_distance
    checker = CollisionChecker(scan, cfg, known)
    assert all(checker.free(p) for p in report['path'])
    scan.points.append((-.25, 0.))
    report = build_plan_report(scan, cfg, known_free_poses=known)
    assert not report['path']
    assert report['diagnostics']['recovery_blocked_by'] == 'obstacle'


def test_startup_bubble_stays_fixed_in_the_mission_frame():
    cfg = replace(Settings(), allow_reverse=True, initial_clear_radius=.4)
    assert build_plan_report(front_scan(), cfg)['path']
    report = build_plan_report(front_scan(), cfg, origin=(-.2, 0., 0.), known_free_poses=[(0., 0., 0.)])
    assert not report['path']


def test_exhausted_recovery_does_not_keep_backing_up():
    cfg = replace(Settings(), allow_reverse=True)
    report = build_plan_report(full_scan(), cfg, recovery_allowed=False)
    assert not report['path']
    assert report['diagnostics']['recovery_limit_reached']


def test_hybrid_planner_can_reach_a_goal_behind_the_vehicle():
    cfg = replace(Settings(), allow_reverse=True, max_range=.8)
    scan = full_scan(); scan.max_range = .8
    goal = Gap(-.4, 0., 0., .8, 0, 1)
    path = plan_path(scan, goal, cfg)
    assert path
    assert path[-1][0] == pytest.approx(goal.x, abs=cfg.goal_tolerance)
    assert any(b[0] < a[0] for a, b in zip(path, path[1:]))
    assert all(CollisionChecker(scan, cfg).free(p) for p in path)


def test_reverse_then_forward_stops_at_the_cusp_before_parking():
    cfg = replace(Settings(), allow_reverse=True)
    follower = Follower(cfg, speed=.3)
    path = [(-i/100, 0., 0.) for i in range(31)]
    path += [(-.3+i/100, 0., 0.) for i in range(1, 111)]
    follower.accept(Gap(.8, 0., 0., .8, 0, 1), path, True, (0., 0., 0.))
    pose = (0., 0., 0.); scan = full_scan(); min_x = 0.; signs = []; zeros = 0
    for i in range(600):
        speed, _, state = follower.command(scan, pose, new_scan=i % 2 == 0, now=i*.05)
        if speed:
            sign = 1 if speed > 0 else -1
            if signs and signs[-1] != sign:
                assert zeros >= 3
            signs.append(sign); zeros = 0
        else:
            zeros += 1
        pose = (pose[0]+speed*.05, 0., 0.); min_x = min(min_x, pose[0])
        if follower.parked:
            break
    assert follower.parked and min_x < -.24
    assert -1 in signs and 1 in signs
    assert pose[0] > .7


def test_recovery_without_a_bay_never_counts_as_completed_parking():
    cfg = replace(Settings(), allow_reverse=True)
    follower = Follower(cfg, speed=.5, require_parking_heading=False)
    path = [(-i/100, 0., 0.) for i in range(25)]
    follower.accept(None, path, False, (0., 0., 0.), recovery=True)
    pose = (0., 0., 0.)
    for i in range(200):
        speed, _, state = follower.command(full_scan(), pose, new_scan=i % 2 == 0, now=i*.05)
        assert speed >= -cfg.recovery_speed-1e-9
        pose = (pose[0]+speed*.05, 0., 0.)
        if state == 'REOBSERVE':
            break
    assert state == 'REOBSERVE' and pose[0] < -.20
    assert not follower.parked and follower.goal is None


def test_new_obstacle_cancels_an_accepted_reverse_recovery():
    cfg = replace(Settings(), allow_reverse=True)
    follower = Follower(cfg)
    follower.accept(None, [(-i/100, 0., 0.) for i in range(25)], False, (0., 0., 0.), recovery=True)
    scan = full_scan(); scan.points.append((-.3, 0.))
    speed, steering, state = follower.command(scan, (0., 0., 0.), new_scan=True, now=0.)
    assert (speed, steering, state) == (0., 0., 'RECOVERY_BLOCKED')
    assert not follower.path and not follower.parked


def test_reverse_reposition_then_curved_parking_and_recorded_exit():
    cfg = replace(Settings(), allow_reverse=True, goal_tolerance=.05, goal_yaw_tolerance=.15)
    follower = Follower(cfg, speed=.3, max_steering_rate=.8)
    path = [(-i/100, 0., 0.) for i in range(25)]
    path += [advance(path[-1], .6*math.pi/2*i/60, -1/.6) for i in range(1, 61)]
    end = path[-1]
    path += [advance(end, i/100, 0.) for i in range(1, 21)]
    follower.accept(Gap(*path[-1], .8, 0, 1), path, True, (0., 0., 0.))
    pose = (0., 0., 0.); states = set()
    for i in range(800):
        speed, steering, state = follower.command(full_scan(), pose, new_scan=i % 2 == 0, now=i*.05)
        states.add(state)
        assert abs(steering) <= cfg.steering_limit+1e-9
        pose = advance(pose, speed*.05, math.tan(steering)/cfg.wheelbase)
        if follower.exited:
            break
    assert follower.parked and follower.exited
    assert {'REVERSING', 'CHANGING_DIRECTION', 'PARKING', 'PARKED', 'EXITING', 'EXITED'} <= states
