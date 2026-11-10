"""A parked mission retains its route through health/odometry interruptions."""
from concurrent.futures import Future
import json
import time

import pytest
import rclpy
from rclpy.context import Context
from rclpy.parameter import Parameter
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String

from autorace_parking.control import Follower
from autorace_parking.core import Gap, Settings, Scan
from autorace_parking.node import ParkingNode
from test_localization_rematch import odom, scan, health


@pytest.fixture
def parked():
    context = Context()
    rclpy.init(context=context, domain_id=226, signal_handler_options=SignalHandlerOptions.NO)
    n = ParkingNode(context=context, parameter_overrides=[
        Parameter('localization_source', value='fused'),
        Parameter('continuous_replanning', value=True),
        Parameter('require_complete_entry_path', value=True),
        Parameter('assume_unobserved_free', value=True),
        Parameter('exit_to_grid_edge', value=False)])
    n.control_timer.cancel()
    clock = [100.]
    n.now_seconds = lambda: clock[0]
    commands = []
    n.publish_command = lambda v, a: commands.append((v, a))
    n.activation.active = True
    n.on_localization(odom(100.)); n.on_scan(scan(100.)); n.on_fusion_health(health(100.))
    f = n.follower
    f.accept(Gap(-.5, 0., 0., .6, 0, 1), [(0., 0., 0.), (-.5, 0., 0.)], True, (0., 0., 0.))
    f.parked = f.exiting = f.exit_prepared = True
    f.last_drive_direction = -1
    f.parked_since, f.exit_started_at = 99., 99.
    f.entry_trace = [(-.5, 0., 0.), (0., 0., 0.)]
    n.lock_departure_localization()
    try:
        yield n, clock, commands
    finally:
        n.destroy_node(); context.shutdown()


def advance(n, clock, t, x=0.):
    clock[0] = t
    n.on_localization(odom(t, x))
    n.on_scan(scan(t))


@pytest.mark.parametrize('kind', ['missing', 'stale', 'not_ready'])
def test_health_does_not_discard_parked_route_or_block_mux(parked, kind):
    n, clock, commands = parked
    f, tracker = n.follower, n.tracker
    route, goal, trace = f.path, f.goal, f.entry_trace
    if kind == 'missing':
        n.fusion_health = n.fusion_health_received = None
    elif kind == 'stale':
        n.fusion_health_received = time.monotonic()-10.
    else:
        n.on_fusion_health(String(data=json.dumps({'stamp': 100.01, 'ready': False})))
    advance(n, clock, 102.)
    assert n.localization_ready(clock[0])
    n.tick()
    assert commands[-1][0] < 0 and n.state == 'EXITING'
    assert n.follower is f and n.tracker is tracker
    assert f.path is route and f.goal is goal and f.entry_trace is trace
    assert f.parked_since == 99. and n.localization_restart_count == 0
    states = []
    n.guard_pub = type('Capture', (), {'publish': lambda _, msg: states.append(json.loads(msg.data))})()
    n.publish_guard()
    assert states[-1]['phase'] == 'DRIVE' and states[-1]['ready']


def test_odom_outage_holds_route_then_resumes_without_matching(parked, monkeypatch):
    n, clock, commands = parked
    f, tracker = n.follower, n.tracker
    route = f.path
    origin, offset, matcher = tracker.origin, tracker.offset, tracker.matcher
    monkeypatch.setattr(matcher, 'match', lambda *_: pytest.fail('Departure must not run ICP'))
    clock[0] = 102.
    n.tick()
    assert commands[-1] == (0., 0.) and n.state == 'WAITING_LOCALIZATION'
    assert n.follower is f and f.path is route and f.parked_since == 99.
    advance(n, clock, 102.1, -.05)
    n.tick()
    assert commands[-1][0] < 0 and n.state == 'EXITING'
    assert tracker.origin == origin and tracker.offset == offset and tracker.matcher is matcher
    assert tracker.pose[0] == pytest.approx(-.05)
    assert n.localization_restart_count == 0


def test_pose_jump_stops_without_reanchoring_until_reset(parked):
    n, clock, commands = parked
    f, tracker = n.follower, n.tracker
    clock[0] = 100.1
    n.on_localization(odom(100.1, 5.))
    n.tick()
    assert commands[-1] == (0., 0.) and n.odometry.discontinuity
    assert n.follower is f and n.tracker is tracker and f.goal is not None
    advance(n, clock, 100.2, .01)
    n.tick()
    assert commands[-1] == (0., 0.)
    n.reset_state()
    assert not n.follower.parked and not n.tracker.odometry_only


def test_stale_scan_stops_but_preserves_route(parked):
    n, clock, commands = parked
    route = n.follower.path
    clock[0] = 102.
    n.on_localization(odom(102.))
    n.tick()
    assert commands[-1] == (0., 0.) and n.state == 'WAITING_SCAN'
    assert n.follower.path is route
    n.on_scan(scan(102.)); n.tick()
    assert commands[-1][0] < 0


def test_scan_acquired_before_parking_cannot_replace_locked_tracker(parked):
    n, _, _ = parked
    n.tracker.odometry_only = False
    old = Future()
    n.scan_job = (old, n.epoch, scan(100.), time.monotonic())
    entry_plan = Future(); n.pending = (entry_plan, n.epoch, (0., 0., 0.), 100.)
    n.lock_departure_localization()
    assert old.cancelled() and n.scan_job is None and entry_plan.cancelled() and n.pending is None
    assert n.tracker.odometry_only


def test_async_scan_keeps_fixed_odometry_transform(parked, monkeypatch):
    n, clock, commands = parked
    n.async_scan_matching = True
    origin, offset = n.tracker.origin, n.tracker.offset
    monkeypatch.setattr(n.tracker.matcher, 'match', lambda *_: pytest.fail('ICP during departure'))
    monkeypatch.setattr(n.tracker.matcher, 'stationary_scene_changed',
                        lambda *_: pytest.fail('Keyframe matching during departure'))
    advance(n, clock, 100.1, -.01)
    deadline = time.monotonic()+2.
    while n.scan_job is not None:
        assert time.monotonic() < deadline
        n.poll_scan_match()
        time.sleep(.001)
    n.tick()
    assert n.tracker.odometry_only and n.tracker.origin == origin and n.tracker.offset == offset
    assert n.tracker.pose[0] == pytest.approx(-.01) and commands[-1][0] < 0


def test_completion_tick_locks_transform_before_next_health_callback(parked):
    n, _, _ = parked
    f = n.follower
    f.parked = f.exiting = f.exit_prepared = False
    n.tracker.odometry_only = False
    f.goal = Gap(0., 0., 0., .6, 0, 1)
    f.path = [(0., 0., 0.), (.01, 0., 0.)]
    f.arrival_count = 2
    n.controlled_sequence = -1
    n.tick()
    assert f.parked and f.parked_since == 100. and n.tracker.odometry_only
    n.on_fusion_health(String(data=json.dumps({'stamp': 100.01, 'ready': False})))
    assert n.follower is f and f.goal is not None and n.localization_restart_count == 0


def test_missing_goal_holds_instead_of_crashing_or_replanning():
    f = Follower(Settings())
    f.parked = True
    source = Scan([10.]*361, -3.14, .02, .01, 12.)
    assert f.command(source, (0., 0., 0.), True, now=1.) == (0., 0., 'EXIT_PATH_ERROR')
    assert f.exit_route_reason == 'missing_parking_goal'


def test_new_obstacle_still_stops_departure(parked):
    n, _, commands = parked
    n.tick()
    assert commands[-1][0] < 0
    n.scan.points.append((-.1, 0.))
    n.planning_cache_key = None
    n.scan_sequence += 1
    n.tick()
    assert commands[-1] == (0., 0.) and n.state == 'ROUTE_BLOCKED'
    assert n.follower.parked and n.follower.path
