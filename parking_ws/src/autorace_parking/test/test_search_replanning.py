"""Initial search limits must not suppress recovery of an accepted route."""
from concurrent.futures import Future
from dataclasses import replace
import math
from unittest.mock import Mock

import pytest

from test_operator_ros import node
from test_route_block_loop import prepare_node, proposal, scan


def expired_search(node, elapsed=25.):
    node.search_enabled = True
    node.search_started = node.now_seconds() - elapsed
    node.search_distance = .24


def capture_planning(node):
    node.localization_ready = lambda now: True
    node.steering_model_ready = lambda: True
    node.worker.shutdown(wait=True)
    node.worker = Mock()
    future = Future()
    node.worker.submit.return_value = future
    node.pending = None
    node.last_plan_wall = -math.inf
    return future


def test_accepted_forward_path_finishes_search_and_clears_corridor_cache(node):
    goal, path = prepare_node(node, [])
    expired_search(node)
    node.search_check_sequence = 10
    node.search_clear = True
    node.search_clear_count = 2
    node.search_check_pose = (0., 0., 0.)
    node.search_block_detail = {'kind': 'obstacle'}
    proposal(node, goal, path)
    assert node.follower.path and not node.follower.recovering
    assert node.search_started is None
    assert node.search_last_pose is None
    assert node.search_distance == 0.
    assert node.search_check_sequence == -1
    assert not node.search_clear and node.search_clear_count == 0
    assert node.search_check_pose is None and node.search_block_detail is None


def test_rejected_path_does_not_refresh_an_expired_search_budget(node):
    goal, path = prepare_node(node, [(.4, 0.)])
    expired_search(node)
    started = node.search_started
    result = proposal(node, goal, path)
    assert result['diagnostics']['candidate_path_rejected']['kind'] == 'obstacle'
    assert node.search_started == started and node.search_distance == .24
    assert node.follower.route_blocked


@pytest.mark.parametrize('elapsed', [19., 20., 25., 120.])
def test_blocked_accepted_path_replans_even_after_initial_search_deadline(node, elapsed):
    prepare_node(node, [(.39, 0.)])
    expired_search(node, elapsed)
    future = capture_planning(node)
    commands = []
    node.publish_command = lambda speed, steering: commands.append((speed, steering))
    try:
        node.tick()
        assert node.state == 'ROUTE_BLOCKED'
        assert commands[-1] == (0., 0.)
        assert node.follower.last_collision['kind'] == 'obstacle'
        assert node.worker.submit.call_count == 1
    finally:
        future.cancel()


def test_distance_search_limit_does_not_disable_replanning_an_existing_path(node):
    prepare_node(node, [(.39, 0.)])
    expired_search(node, elapsed=1.)
    node.search_distance = node.search_max_distance
    future = capture_planning(node)
    try:
        node.tick()
        assert node.state == 'ROUTE_BLOCKED'
        assert node.worker.submit.call_count == 1
    finally:
        future.cancel()


@pytest.mark.parametrize('limit', ['time', 'distance'])
def test_initial_search_still_stops_and_does_not_restart_planning_at_its_limit(node, limit):
    prepare_node(node, [])
    node.follower.path = []
    node.follower.goal = None
    node.follower.final_approach = False
    node.planning_info = {'reason_code': 'no_gap'}
    expired_search(node, elapsed=25. if limit == 'time' else 1.)
    if limit == 'distance':
        node.search_distance = node.search_max_distance
    future = capture_planning(node)
    try:
        node.tick()
        assert node.state == 'SEARCH_LIMIT'
        assert node.worker.submit.call_count == 0
    finally:
        future.cancel()


@pytest.mark.parametrize('elapsed', [25., 120.])
def test_expired_search_does_not_interrupt_safe_confirmed_entry(node, elapsed):
    goal, path = prepare_node(node, [])
    node.cfg = replace(node.cfg, replan_while_following=False)
    node.follower.cfg = node.cfg
    node.follower.accept(replace(goal, confirmed=True), path, True, (0., 0., 0.))
    node.state = 'PARKING'
    expired_search(node, elapsed)
    future = capture_planning(node)
    commands = []
    node.publish_command = lambda speed, steering: commands.append((speed, steering))
    try:
        node.tick()
        assert node.state == 'PARKING' and commands[-1][0] > 0.
        assert node.worker.submit.call_count == 0
    finally:
        future.cancel()


def test_stale_scan_still_stops_an_existing_path_without_planning(node):
    prepare_node(node, [])
    expired_search(node)
    node.scan_time = node.now_seconds() - node.scan_timeout - .1
    future = capture_planning(node)
    commands = []
    node.publish_command = lambda speed, steering: commands.append((speed, steering))
    try:
        node.tick()
        assert node.state == 'WAITING_SCAN' and commands[-1] == (0., 0.)
        assert node.worker.submit.call_count == 0
    finally:
        future.cancel()


def test_a_later_search_starts_with_new_observations_and_its_own_limit(node):
    goal, path = prepare_node(node, [])
    expired_search(node)
    proposal(node, goal, path)
    node.follower.path = []
    node.follower.route_blocked = False
    node.planning_info = {'reason_code': 'no_gap'}
    node.scan = scan()
    node.scan_time = node.last_scan_received_stamp = 40.
    node.scan_sequence += 1
    assert node.search_command(40., (0., 0., 0.), node.scan) == (0., 0.)
    assert node.search_started == 40.
    # A cached pre-route corridor cannot authorize immediate search movement.
    assert node.search_clear_count < node.search_resume_scans
    assert node.search_command(40. + node.search_timeout, (0., 0., 0.), node.scan) == (0., 0.)
    assert node.state == 'SEARCH_LIMIT'
