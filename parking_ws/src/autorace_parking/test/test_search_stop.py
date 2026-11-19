"""Explore at the real configured speed, then stop and plan from a new pose."""
from concurrent.futures import Future
from dataclasses import replace
import math
import time

import pytest
import rclpy
from rclpy.context import Context
from rclpy.parameter import Parameter

from autorace_parking.core import Gap, Scan
from autorace_parking.node import ParkingNode


@pytest.fixture
def node():
    context = Context()
    rclpy.init(context=context, domain_id=188)
    n = ParkingNode(context=context, parameter_overrides=[
        Parameter('localization_source', value='scan'),
        Parameter('search_enabled', value=True),
        Parameter('search_speed', value=.2),
        Parameter('search_resume_scans', value=1),
        Parameter('assume_unobserved_free', value=True),
        Parameter('continuous_replanning', value=True),
    ])
    n.control_timer.cancel()
    n.activation.active = n.tracker.valid = True
    n.tracker.pose = (0., 0., 0.)
    n.scan = Scan([math.inf]*361, -math.pi, math.pi/180, .01, 12.)
    n.planning_scan = lambda: n.scan
    clock = [100.]
    n.now_seconds = lambda: clock[0]
    n.scan_time = clock[0]
    n.scan_sequence = 1
    n.planning_info = {'reason_code': 'no_gap'}
    n.commands = []
    n.publish_command = lambda v, a: n.commands.append((v, a))
    try:
        yield n, clock
    finally:
        n.destroy_node()
        context.shutdown()


def plan(n, clock, *, origin=(0., 0., 0.), started=None, path=True):
    goal = Gap(.8-origin[0], 0., 0., .5, 0, 1, confirmed=True)
    future = Future()
    future.set_result({'goal': goal,
        'path': [(0., 0., 0.), (.2, 0., 0.), (goal.x, 0., 0.)] if path else [],
        'final': True, 'diagnostics': {
            'reason_code': 'path_ready' if path else 'no_observed_path',
            'target': {'x': goal.x, 'y': 0., 'yaw': 0.},
            'goal_confirmed': True, 'candidates': 1,
        }})
    n.pending = future, n.epoch, origin, clock[0] if started is None else started
    n.poll_plan()


def measurement(n, clock, stamp, speed=0., yaw_rate=0., age=0.):
    clock[0] = stamp
    n.wheel_sample = stamp, speed, yaw_rate, time.monotonic()-age
    return n.search_command(stamp, n.tracker.pose, n.scan)


def test_real_search_policy_ignores_zero_returns_but_stops_for_obstacles_and_stale_inputs(node):
    n, clock = node
    n.scan = Scan([0.]*361, -math.pi, math.pi/180, .01, 12.)
    assert n.search_command(clock[0], n.tracker.pose, n.scan) == (.2, 0.)
    n.cfg = replace(n.cfg, assume_unobserved_free=False)
    n.scan_sequence += 1
    assert n.search_command(clock[0], n.tracker.pose, n.scan) == (0., 0.)
    n.cfg = replace(n.cfg, assume_unobserved_free=True)
    n.scan.points = [(.35, 0.)]
    n.scan_sequence += 1
    assert n.search_command(clock[0], n.tracker.pose, n.scan) == (0., 0.)
    assert n.search_block_detail['kind'] == 'obstacle'
    n.scan.points = []
    n.scan_sequence += 1
    assert n.search_command(clock[0]+1., n.tracker.pose, n.scan) == (0., 0.)


@pytest.mark.parametrize('moving_path', [True, False])
def test_candidate_stops_search_then_only_a_new_stopped_pose_plan_can_drive(node, moving_path):
    n, clock = node
    assert n.search_command(clock[0], n.tracker.pose, n.scan) == (.2, 0.)
    measurement(n, clock, 100.01, speed=.2)
    plan(n, clock, path=moving_path)
    assert n.search_target_phase == 'STOPPING'
    assert not n.follower.path and n.commands[-1] == (0., 0.)
    assert n.tracked_goal.x == pytest.approx(.8)
    assert measurement(n, clock, 100.02, speed=.1) == (0., 0.)
    n.tracker.pose = (.04, 0., 0.)
    for stamp in (100.04, 100.06):
        assert measurement(n, clock, stamp) == (0., 0.)
        assert n.search_target_phase == 'STOPPING'
    assert measurement(n, clock, 100.08) == (0., 0.)
    assert n.search_target_phase == 'PLANNING'
    plan(n, clock, started=100.01)
    assert not n.follower.path
    clock[0] = n.scan_time = 100.1
    plan(n, clock, origin=n.tracker.pose)
    assert n.search_target_phase is None
    assert n.follower.path[0] == n.tracker.pose
    assert n.follower.goal.x == pytest.approx(.8)
    assert n.search_command(clock[0], n.tracker.pose, n.scan) is None
    n.localization_ready = lambda now: True
    n.plan_period = 1e9
    n.last_plan_wall = time.monotonic()
    n.tick()
    assert n.commands[-1][0] > 0.


def test_duplicate_stale_or_rolling_wheel_data_cannot_finish_the_search_stop(node):
    n, clock = node
    n.search_started = 99.
    plan(n, clock)
    measurement(n, clock, 100.02)
    for _ in range(5):
        n.search_command(clock[0], n.tracker.pose, n.scan)
    assert n.search_stop_count == 1
    measurement(n, clock, 100.04, age=1.)
    assert n.search_stop_count == 0
    measurement(n, clock, 100.06)
    measurement(n, clock, 100.08, yaw_rate=.03)
    assert n.search_stop_count == 0
    for stamp in (100.1, 100.12, 100.14):
        measurement(n, clock, stamp)
    assert n.search_target_phase == 'PLANNING'
    n.wheel_sample = clock[0], .1, 0., time.monotonic()
    plan(n, clock)
    assert not n.follower.path and n.search_target_phase == 'STOPPING'


def test_search_budget_does_not_block_planning_after_a_candidate_is_found(node):
    n, clock = node
    n.search_started = 79.9
    n.search_distance = n.search_max_distance
    plan(n, clock)
    assert n.search_target_phase == 'STOPPING'
    for stamp in (100.02, 100.04, 100.06):
        assert measurement(n, clock, stamp) == (0., 0.)
    assert n.search_target_phase == 'PLANNING'
    n.scan_time = clock[0]
    plan(n, clock)
    assert n.follower.path


def test_reset_clears_the_detected_target_stop(node):
    n, clock = node
    n.search_started = 99.
    plan(n, clock)
    n.reset_motion()
    assert n.search_target_phase is None and n.search_replan_after is None
    assert n.search_stop_count == 0 and n.tracked_goal is None


def test_tick_waits_for_standstill_and_a_post_stop_scan_before_requesting_a_path(node, monkeypatch):
    n, clock = node
    n.search_started = 99.
    plan(n, clock)
    n.localization_ready = lambda now: True
    calls = []

    def submit(*args):
        calls.append(args)
        return Future()

    monkeypatch.setattr(n.worker, 'submit', submit)
    n.wheel_sample = clock[0], .2, 0., time.monotonic()
    n.tick()
    assert not calls and n.commands[-1] == (0., 0.)
    n.tracker.pose = (.04, 0., 0.)
    for stamp in (100.02, 100.04, 100.06):
        clock[0] = stamp
        n.wheel_sample = stamp, 0., 0., time.monotonic()
        n.scan_sequence += 1
        n.tick()
        assert not calls and n.commands[-1] == (0., 0.)
    assert n.search_target_phase == 'PLANNING'
    clock[0] = n.scan_time = 100.08
    n.wheel_sample = clock[0], 0., 0., time.monotonic()
    n.tick()
    assert len(calls) == 1 and calls[0][4] == n.tracker.pose
    assert n.commands[-1] == (0., 0.)
