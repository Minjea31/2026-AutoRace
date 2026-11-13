"""Concurrency and stale-result regression tests for the independent planner."""
from concurrent.futures import Future
import time

import pytest
from geometry_msgs.msg import TransformStamped

from cone_lane_follower.async_planner import AsyncPlanner, compute_path
from cone_lane_follower.node import CENTER_DEFAULTS
from test_roi import harness


class HeldExecutor:
    def __init__(self):
        self.jobs = []

    def submit(self, fn, payload):
        future = Future()
        self.jobs.append((payload, future))
        return future

    def shutdown(self, **kwargs):
        pass


def test_busy_worker_replaces_pending_snapshot_and_drops_old_mission():
    executor = HeldExecutor()
    planner = AsyncPlanner(executor)
    planner.request('first', 1)
    planner.request('second', 2)
    planner.request('latest', 3)
    assert [p for p, _ in executor.jobs] == ['first']
    executor.jobs[0][1].set_result(([], {}))
    assert planner.poll() == (1, ([], {}))
    assert [p for p, _ in executor.jobs] == ['first', 'latest']
    planner.reset()
    executor.jobs[1][1].set_result((['old mission'], {}))
    assert planner.poll() is None
    planner.close()


def payload(budget=.06):
    left = [(i*.05, .3, 0.) for i in range(21)]
    right = [(i*.05, -.3, 0.) for i in range(21)]
    measured = [(i*.05, 0., 0.) for i in range(21)]
    return left, right, [], measured, .6, CENTER_DEFAULTS, budget


def test_worker_budget_retains_measured_path():
    data = payload(-1.)
    path, diagnostics = compute_path(data)
    assert path == data[3]
    assert 'planning_budget' in diagnostics


def test_real_worker_process_computes_without_blocking_poll():
    planner = AsyncPlanner()
    try:
        planner.request(payload(), 'snapshot')
        end = time.monotonic()+12
        result = None
        while result is None and time.monotonic() < end:
            result = planner.poll()
            time.sleep(.01)
        assert result is not None
        metadata, (path, diagnostics) = result
        assert metadata == 'snapshot'
        assert path
        assert all(abs(y) < 1e-6 for x, y, z in path)
        assert 'planning_ms' in diagnostics
    finally:
        planner.close()


@pytest.mark.parametrize('age,changed,expected', [(.1, False, 'Unified (worker)'),
    (.6, False, 'Measured (planner pending)'), (.1, True, 'Measured (planner pending)')])
def test_async_result_freshness_and_parameter_change(harness, monkeypatch, age, changed, expected):
    roi, *_ = harness
    executor = HeldExecutor()
    roi.async_planner = AsyncPlanner(executor)
    roi.control_scan_stamp = roi.get_clock().now().nanoseconds
    roi.measured_center_path = [(.1, 0., 0.), (.8, 0., 0.)]
    planned = [(.1, .05, 0.), (.8, .05, 0.)]
    tf = TransformStamped()
    tf.transform.rotation.w = 1.
    monkeypatch.setattr(roi, 'memory_transform', lambda *args: tf)
    p = {name: roi.get_parameter(name).value for name in CENTER_DEFAULTS}
    if changed:
        p['center_sample_step'] *= 2
    stamp = roi.get_clock().now().nanoseconds-int(age*1e9)
    roi.async_planner.request(None, (stamp, tf, p, time.monotonic()-age))
    executor.jobs[0][1].set_result((planned, {}))
    roi.use_async_path()
    assert roi.center_path_mode == expected
    assert roi.center_path == (planned if expected == 'Unified (worker)' else roi.measured_center_path)


def test_pending_or_failed_worker_keeps_current_measured_path(harness):
    roi, *_ = harness
    executor = HeldExecutor()
    roi.async_planner = AsyncPlanner(executor)
    roi.control_scan_stamp = roi.get_clock().now().nanoseconds
    roi.measured_center_path = [(.1, 0., 0.), (.8, 0., 0.)]
    roi.async_planner.request(None, (0, None, {}, time.monotonic()))
    roi.use_async_path()
    assert roi.center_path == roi.measured_center_path
    executor.jobs[0][1].set_exception(RuntimeError('worker failed'))
    roi.use_async_path()
    assert roi.center_path == roi.measured_center_path
    assert 'planner_error' in roi.path_diagnostics


def test_worker_path_is_reprojected_into_latest_scan_frame(harness, monkeypatch):
    roi, *_ = harness
    executor = HeldExecutor()
    roi.async_planner = AsyncPlanner(executor)
    roi.control_scan_stamp = roi.get_clock().now().nanoseconds
    roi.measured_center_path = [(.1, 0., 0.), (.8, 0., 0.)]
    source_to_odom = TransformStamped()
    source_to_odom.transform.rotation.w = 1.
    source_to_odom.transform.translation.x = 1.
    odom_to_scan = TransformStamped()
    odom_to_scan.transform.rotation.w = 1.
    odom_to_scan.transform.translation.x = -1.2
    def lookup(target, source, stamp):
        assert (target, source) == ('base_link', 'odom')
        assert stamp.nanoseconds == roi.control_scan_stamp
        return odom_to_scan
    monkeypatch.setattr(roi, 'memory_transform', lookup)
    parameters = {name: roi.get_parameter(name).value for name in CENTER_DEFAULTS}
    metadata = (roi.control_scan_stamp, source_to_odom, parameters, time.monotonic())
    roi.async_planner.request(None, metadata)
    executor.jobs[0][1].set_result(([ (.4, .1, 0.), (.8, .1, 0.) ], {}))
    roi.use_async_path()
    assert [q[0] for q in roi.center_path] == pytest.approx([.2, .6])


def test_missing_worker_result_tf_keeps_measured_path(harness, monkeypatch):
    from tf2_ros import TransformException
    roi, *_ = harness
    executor = HeldExecutor()
    roi.async_planner = AsyncPlanner(executor)
    roi.control_scan_stamp = roi.get_clock().now().nanoseconds
    roi.measured_center_path = [(.1, 0., 0.), (.8, 0., 0.)]
    tf = TransformStamped()
    tf.transform.rotation.w = 1.
    def missing(*args):
        raise TransformException('missing')
    monkeypatch.setattr(roi, 'memory_transform', missing)
    p = {name: roi.get_parameter(name).value for name in CENTER_DEFAULTS}
    roi.async_planner.request(None, (roi.control_scan_stamp, tf, p, time.monotonic()))
    executor.jobs[0][1].set_result(([ (.4, .1, 0.), (.8, .1, 0.) ], {}))
    roi.use_async_path()
    assert roi.center_path == roi.measured_center_path
    assert 'planner_tf' in roi.path_diagnostics
