"""Foreground sign filtering must retain the actual parking pair and raw sensing."""
from dataclasses import fields, replace
import json
import math
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml

from autorace_parking.core import CollisionChecker, Scan, Settings, cluster_points, find_gaps
from autorace_parking.control import build_plan_report


def real_config():
    package = Path(__file__).resolve().parents[1]
    params = yaml.safe_load((package/'config/parking_real.yaml').read_text())['parking_node']['ros__parameters']
    return replace(Settings(**{f.name: params[f.name] for f in fields(Settings) if f.name in params}),
                   min_cluster_size=.08,enclosed_space_detection=False,continuous_replanning=False)


def sign_scan():
    package = Path(__file__).resolve().parents[1]
    return Scan(**json.loads((package/'test/data/limo_pro_sign_scan.json').read_text())['scan'])


def test_live_foreground_sign_is_removed_and_only_distant_parking_pair_remains():
    # Preserve the wider recorded scene to isolate sign-filter behavior.
    cfg, raw = replace(real_config(),max_gap=1.1), sign_scan()
    original = find_gaps(cluster_points(raw.points), cfg)
    assert len(original) == 2 and original[0].x < 1.2
    filtered = raw.filtered_obstacles(cfg)
    goals = find_gaps(cluster_points(filtered.points), cfg)
    assert len(goals) == 1
    assert goals[0].x == pytest.approx(2.040046, abs=1e-5)
    assert goals[0].width == pytest.approx(.854309, abs=1e-5)
    assert filtered.ignored_small_clusters == 2 and filtered.ignored_small_points == 15
    assert not any(.6 < x < .7 and -.35 < y < -.25 for x, y in filtered.points)
    assert any(1.45 < x < 1.62 and -.51 < y < -.32 for x, y in filtered.points)
    assert any(2.46 < x < 2.61 and -.48 < y < -.37 for x, y in filtered.points)
    report = build_plan_report(raw, cfg)
    assert report['diagnostics']['candidates'] == 1
    assert report['diagnostics']['ignored_small_points'] == 15


def test_filter_preserves_raw_points_visibility_and_repeated_filter_counts():
    cfg, raw = real_config(), sign_scan()
    original_points, original_ranges = raw.points.copy(), raw.ranges.copy()
    filtered = raw.filtered_obstacles(cfg)
    assert raw.points == original_points and raw.ranges == original_ranges
    assert filtered.ranges == original_ranges
    assert filtered.filtered_obstacles(cfg) is filtered
    assert filtered.ignored_small_points == 15
    # Removing a sign must not claim its hidden space was measured clear.
    for x, y in [(i*.1, j*.1) for i in range(-5, 26) for j in range(-15, 16)]:
        assert filtered.visible(x, y) == raw.visible(x, y)
    assert raw.filtered_obstacles(replace(cfg, obstacle_min_cluster_size=0.)) is raw


def test_collision_filter_ignores_compact_clusters_but_retains_large_and_isolated_returns():
    cfg = replace(Settings(), obstacle_min_cluster_size=.1)
    raw = Scan([math.inf]*361, -math.pi, math.pi/180, .01, 12., (0., 0., 0.))
    sign = [(.49, 0.), (.5, .01), (.51, 0.)]
    obstacle = [(1.5, -.08+i*.02) for i in range(9)]
    isolated = [(2., .5), (2.01, .5)]
    raw.points = sign+obstacle+isolated
    assert not CollisionChecker(raw, replace(cfg, obstacle_min_cluster_size=0.)).free((.5, 0., 0.))
    checker = CollisionChecker(raw, cfg)
    assert checker.free((.5, 0., 0.))
    assert not checker.free((1.5, 0., 0.)) and checker.last_block == 'obstacle'
    assert not checker.free((2., .5, 0.)) and checker.last_block == 'obstacle'


def test_ros_obstacles_are_filtered_while_tracker_receives_raw_scan():
    rclpy = pytest.importorskip('rclpy')
    from rclpy.parameter import Parameter
    from sensor_msgs.msg import LaserScan
    from autorace_parking.node import ParkingNode
    rclpy.init()
    node = ParkingNode(parameter_overrides=[Parameter('localization_source', value='scan'),
                                           Parameter('obstacle_min_cluster_size', value=.12),
                                           Parameter('small_cluster_filter_max_distance', value=1.)])
    try:
        node.activation.active = True
        tracked, batches = [], []
        node.tracker.update = lambda points, stamp: tracked.append(points) or True
        node.obstacles_pub = SimpleNamespace(publish=batches.append)
        raw = sign_scan()
        msg = LaserScan()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = 'laser_link'
        msg.ranges = raw.ranges
        msg.angle_min, msg.angle_increment = raw.angle_min, raw.angle_increment
        msg.range_min, msg.range_max = raw.range_min, raw.range_max
        node.on_scan(msg)
        assert len(tracked) == 1 and tracked[0] == raw.points
        assert len(node.scan.points) == len(raw.points)-15
        boxes = [m for m in batches[-1].markers if m.ns == 'cluster_box']
        assert len(boxes) == 4  # distant small returns stay obstacles; the near sign is removed
        assert all(m.action != 3 for m in batches[-1].markers)
        # A shrinking scan removes absent IDs without blanking surviving boxes.
        clusters = cluster_points(node.scan.points,node.cfg.cluster_distance,node.cfg.cluster_min_points)
        node.publish_obstacles(clusters[:1])
        assert {(m.ns,m.id) for m in batches[-1].markers if m.action==2} == {('cluster_box',i) for i in range(1,4)}
        assert [(m.ns,m.id) for m in batches[-1].markers if m.action==0] == [('cluster_box',0)]
    finally:
        node.activation.active = False
        node.destroy_node()
        rclpy.shutdown()


def test_twelve_cm_filter_only_removes_near_signs_and_preserves_small_distant_cars():
    cfg = real_config()
    scan = Scan([math.inf]*361, -math.pi, math.pi/180, .01, 12., (0., 0., 0.))
    sign = [(.5, -.455+i*.011) for i in range(11)]
    first_car = [(1.5, -.455+i*.011) for i in range(11)]
    second_car = [(2.1, -.475+i*.015) for i in range(11)]
    scan.points = sign+first_car+second_car
    assert max(p[1] for p in sign)-min(p[1] for p in sign) > .1
    filtered = scan.filtered_obstacles(cfg)
    assert filtered.points == first_car+second_car
    gaps = find_gaps(cluster_points(filtered.points), cfg)
    assert len(gaps) == 1 and gaps[0].x == pytest.approx(1.8)


def test_cached_foreground_gap_is_replaced_when_filtered_scene_shows_the_distant_pair():
    cfg, scan = replace(real_config(),max_gap=1.1), sign_scan()
    previous = find_gaps(cluster_points(scan.points), cfg)[0]
    assert previous.x < 1.2
    report = build_plan_report(scan, cfg, previous, validate_goal=True)
    assert report['diagnostics']['goal_reselected']
    # The selected goal remains visible in diagnostics even if no safe route exists.
    assert report['diagnostics']['target']['x'] > 1.8
    if report['goal'] is not None:
        assert report['goal'].x > 1.8
    assert report['diagnostics']['candidates'] == 1


def test_occluded_pair_does_not_erase_an_anchored_target():
    from autorace_parking.core import Gap
    cfg = real_config()
    scan = Scan([math.inf]*361, -math.pi, math.pi/180, .01, 12., (0., 0., 0.))
    goal = Gap(1., -.4, -math.pi/2, .85, 0, 1)
    report = build_plan_report(scan, cfg, goal, validate_goal=True)
    assert not report['diagnostics']['goal_reselected']
    assert report['goal'].x == goal.x and report['goal'].y == goal.y


def test_matching_bay_preserves_anchored_depth_and_relaxed_heading():
    from autorace_parking.core import Gap
    from autorace_parking.control import same_gap
    from autorace_parking.tracking import compose
    for angle in (0., .7, -1.2):
        goal = Gap(*compose((0., 0., angle), (2., -.7, -.5)), .85, 0, 1)
        candidate = Gap(*compose((0., 0., angle), (2.02, -.4, -math.pi/2)), .85, 0, 1)
        assert same_gap(goal, candidate, angle)
