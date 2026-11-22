"""Slow observation steps require a fresh, fully checked stopping corridor."""

import math

import pytest

from autorace_parking.core import Scan
from test_search import search


@pytest.mark.parametrize(
    "reason",
    [
        "no_gap",
        "no_observed_path",
        "budget_exceeded",
        "center_pose_blocked",
        "entry_departure_unavailable",
    ],
)
def test_missing_parking_path_uses_bounded_slow_search(search, reason):
    node, scan = search
    node.planning_info = {"reason_code": reason}
    assert node.search_command(100.0, (0.0, 0.0, 0.0), scan) == (0.1, 0.0)
    assert node.state == "SEARCHING"
    assert node.search_command(100.0 + node.search_timeout, (0.0, 0.0, 0.0), scan) == (
        0.0,
        0.0,
    )
    assert node.state == "SEARCH_LIMIT"


def short_corridor():
    return Scan([0.5] * 181, -math.pi / 2, math.pi / 180, 0.01, 3.0)


def test_observed_prefix_can_allow_slow_search_without_requiring_thirty_cm(search):
    node, _ = search
    node.planning_info = {"reason_code": "no_observed_path"}
    assert node.search_command(100.0, (0.0, 0.0, 0.0), short_corridor()) == (0.1, 0.0)
    assert node.search_clear


def test_actual_speed_requires_longer_braking_space_even_with_slow_command(search):
    node, scan = search
    scan.points.append((0.55, 0.0))
    node.follower.measured_speed = 0.4
    assert node.search_command(100.0, (0.0, 0.0, 0.0), scan) == (0.0, 0.0)
    assert (
        node.state == "SEARCH_BLOCKED"
        and node.search_block_detail["kind"] == "obstacle"
    )


def test_search_never_replaces_a_current_guided_path(search):
    node, scan = search
    node.planning_info = {"reason_code": "no_observed_path"}
    node.follower.path = [(0.0, 0.0, 0.0), (0.5, 0.0, 0.0)]
    assert node.search_command(100.0, (0.0, 0.0, 0.0), scan) is None


def test_blind_sector_still_blocks_slow_search(search):
    node, scan = search
    scan.ranges = [0.0] * len(scan.ranges)
    scan.points = []
    assert node.search_command(100.0, (0.0, 0.0, 0.0), scan) == (0.0, 0.0)
    assert node.state == "SEARCH_BLOCKED"
