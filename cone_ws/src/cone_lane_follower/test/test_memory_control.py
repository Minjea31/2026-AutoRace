import math
import time

import pytest
from geometry_msgs.msg import TransformStamped
from rclpy.time import Time
from tf2_ros import TransformException

# Reuse the existing isolated ROS node fixture.
from test_roi import harness


class MovingTF:
    def __init__(self, roi, yaw):
        self.roi = roi
        self.x, self.y, self.yaw = 2., 3., yaw
        self.fail = False
        self.stale = False

    def lookup_transform(self, target, source, stamp):
        if self.fail:
            raise TransformException('test: odometry unavailable')
        tf = TransformStamped()
        tf.header.frame_id, tf.child_frame_id = target, source
        now = self.roi.get_clock().now().nanoseconds
        tf.header.stamp = Time(nanoseconds=now-1_000_000_000 if self.stale else now).to_msg()
        if target == 'odom':
            assert source == 'base_link'
            assert stamp.nanoseconds == self.roi.control_scan_stamp
            x, y, yaw = self.x, self.y, self.yaw
        else:
            assert target == 'base_link' and source == 'odom'
            yaw = -self.yaw
            x = -math.cos(self.yaw)*self.x-math.sin(self.yaw)*self.y
            y = math.sin(self.yaw)*self.x-math.cos(self.yaw)*self.y
        tf.transform.translation.x, tf.transform.translation.y = x, y
        tf.transform.rotation.z, tf.transform.rotation.w = math.sin(yaw/2), math.cos(yaw/2)
        return tf


def prepare(roi, monkeypatch, yaw=0.):
    tf = MovingTF(roi, yaw)
    monkeypatch.setattr(roi, 'tf_buffer', tf)
    roi.mission_cone = roi.control_engaged = True
    roi.candidate_clusters = [[], []]
    roi.center_path = [(x, 0., 0.) for x in (.1, .3, .5, .7, .9, 1.1)]
    messages = []

    class Capture:
        def publish(self, msg):
            messages.append(msg)

    roi.command_pub = Capture()
    tick(roi)
    assert roi.control_mode == 'Memory Path Feedback'
    assert roi.path_memory.points and roi.memory_status == 'stored'
    return tf, messages


def tick(roi):
    roi.control_received = time.monotonic()
    roi.control_scan_stamp = roi.get_clock().now().nanoseconds
    roi.control_tick()


@pytest.mark.parametrize('yaw', [0., math.pi/2])
def test_saved_path_tracks_translation_rotation_and_stops_at_end(harness, monkeypatch, yaw):
    roi, *_ = harness
    tf, messages = prepare(roi, monkeypatch, yaw)
    stored = roi.path_memory.points[:]
    roi.center_path = []
    for travel in (.35, .6, .85):
        tf.x, tf.y = 2.+travel*math.cos(yaw), 3.+travel*math.sin(yaw)
        tf.yaw = yaw+.1
        tick(roi)
        assert roi.control_mode == 'Memory Path Feedback'
        assert 0. < messages[-1].drive.speed <= .15
        assert roi.control_heading_error == pytest.approx(-.1)
        assert roi.path_memory.points == stored  # Do not move the stored goal with the car.
        assert roi.mission_exit_since is None  # A usable memory path keeps the mission alive.
    tf.x, tf.y = 2.+1.06*math.cos(yaw), 3.+1.06*math.sin(yaw)
    tick(roi)
    assert roi.control_mode == 'Stopped'
    assert roi.control_stop_reason == 'memory_goal_reached'
    assert messages[-1].drive.speed == messages[-1].drive.steering_angle == 0.
    tick(roi)
    assert roi.control_stop_reason == 'memory_goal_reached'


@pytest.mark.parametrize('failure', ['missing_tf', 'stale_tf', 'scan_timeout', 'scan_tf'])
def test_memory_cannot_override_sensor_or_transform_faults(harness, monkeypatch, failure):
    roi, *_ = harness
    tf, messages = prepare(roi, monkeypatch)
    roi.center_path = []
    if failure == 'missing_tf':
        tf.fail = True
    elif failure == 'stale_tf':
        tf.stale = True
    elif failure == 'scan_timeout':
        roi.control_received -= 1.
    else:
        roi.control_fault = True
    roi.control_tick()
    assert roi.control_mode == 'Stopped'
    assert messages[-1].drive.speed == 0.
    if failure in ('missing_tf', 'stale_tf'):
        assert roi.control_stop_reason == 'memory_tf_unavailable_or_stale'
    elif failure == 'scan_timeout':
        assert roi.path_memory.points and roi.memory_status == 'paused_scan_timeout'
    else:
        assert not roi.path_memory.points


def test_shorter_or_shifted_observation_cannot_replace_saved_path(harness, monkeypatch):
    roi, *_ = harness
    prepare(roi, monkeypatch)
    roi.center_path = []
    tick(roi)
    assert roi.memory_following
    roi.center_path = [(.1, .4, 0.), (.8, .4, 0.)]
    tick(roi)
    assert roi.control_mode == 'Memory Path Feedback'
    assert roi.memory_status == 'kept_no_aligned_extension'
    roi.center_path = [(.1, .03, 0.), (.8, .03, 0.)]
    tick(roi)
    assert roi.control_mode == 'Memory Path Feedback'
    assert roi.memory_following and roi.memory_status == 'kept_no_aligned_extension'
    assert roi.control_lateral_error == pytest.approx(0.)


def test_memory_extends_goal_but_ignores_shorter_live_paths(harness, monkeypatch):
    roi, *_ = harness
    tf, messages = prepare(roi, monkeypatch)
    original = roi.path_memory.points[:]
    roi.center_path = [(.1, .1, 0.), (.5, .1, 0.)]
    tick(roi)
    assert roi.path_memory.points == original
    assert roi.control_lateral_error == pytest.approx(0.)
    # The new live route observes beyond the old 1.1 m goal.
    roi.center_path = [(.5, 0., 0.), (.9, 0., 0.), (1.2, 0., 0.), (1.5, 0., 0.)]
    tick(roi)
    assert roi.memory_status == 'extended'
    assert roi.path_memory.points[:len(original)] == original
    assert roi.path_memory.points[-1] == pytest.approx((3.5, 3., 0.))
    assert roi.control_mode == 'Memory Path Feedback'
    # Seeing only the old part must not reset the endpoint or path order.
    tf.x = 2.4
    roi.center_path = [(.1, 0., 0.), (.5, 0., 0.)]
    tick(roi)
    assert roi.path_memory.points[-1] == pytest.approx((3.5, 3., 0.))
    assert 0. < messages[-1].drive.speed <= .15


@pytest.mark.parametrize('failure', ['receive', 'stamp'])
def test_scan_timeout_preserves_path_and_resumes_only_with_fresh_tf(harness, monkeypatch, failure):
    roi, *_ = harness
    tf, messages = prepare(roi, monkeypatch)
    stored = roi.path_memory.points[:]
    roi.center_path = []
    if failure == 'receive':
        roi.control_received = time.monotonic()-.6
    else:
        roi.control_scan_stamp = roi.get_clock().now().nanoseconds-600_000_000
    roi.control_tick()
    assert roi.control_mode == 'Stopped' and messages[-1].drive.speed == 0.
    assert roi.control_stop_reason == ('scan_receive_timeout' if failure == 'receive' else 'scan_stamp_timeout')
    assert roi.path_memory.points == stored
    assert roi.memory_status == 'paused_scan_timeout'
    assert roi.mission_exit_since is None
    # Sensor recovery alone cannot override an unavailable odometry transform.
    tf.fail = True
    tick(roi)
    assert messages[-1].drive.speed == 0.
    assert roi.control_stop_reason == 'memory_tf_unavailable_or_stale'
    assert roi.path_memory.points == stored
    tf.fail = False
    tf.x += .35
    tick(roi)
    assert roi.control_mode == 'Memory Path Feedback'
    assert roi.path_memory.points == stored and roi.path_memory.index > 0
    assert 0. < messages[-1].drive.speed <= .15


@pytest.mark.parametrize('live_path', [
    [(.2, -.3, 0.), (.2, .3, 0.)],
    [(.8, 0., 0.), (.1, 0., 0.)],
    [(-.8, 0., 0.), (-.1, 0., 0.)],
    [(.1, .8, 0.), (.8, .8, 0.)],
    [(.1, 0., 0.)],
])
def test_unusable_nonempty_live_path_falls_back_to_valid_memory(harness, monkeypatch, live_path):
    roi, *_ = harness
    _, messages = prepare(roi, monkeypatch)
    stored = roi.path_memory.points[:]
    roi.center_path = live_path
    assert roi.path_errors(live_path, .25) is None
    tick(roi)
    assert roi.control_mode == 'Memory Path Feedback'
    assert roi.memory_status == 'following'
    assert roi.control_stop_reason is None
    assert roi.path_memory.points == stored
    assert roi.memory_local_path
    assert 0. < messages[-1].drive.speed <= .15
    assert roi.mission_exit_since is None


def test_invalid_live_and_memory_paths_report_memory_failure(harness, monkeypatch):
    roi, *_ = harness
    tf, messages = prepare(roi, monkeypatch)
    roi.center_path = [(.2, -.3, 0.), (.2, .3, 0.)]
    tf.yaw = math.pi/2
    tick(roi)
    assert roi.control_mode == 'Stopped'
    assert roi.control_stop_reason == 'memory_path_no_usable_forward_segment'
    assert roi.memory_status == roi.control_stop_reason
    assert messages[-1].drive.speed == messages[-1].drive.steering_angle == 0.


def test_invalid_live_path_cannot_override_memory_tf_error(harness, monkeypatch):
    roi, *_ = harness
    tf, messages = prepare(roi, monkeypatch)
    roi.center_path = [(.2, -.3, 0.), (.2, .3, 0.)]
    tf.fail = True
    tick(roi)
    assert roi.control_stop_reason == 'memory_tf_unavailable_or_stale'
    assert messages[-1].drive.speed == 0.


def test_update_failure_remains_visible_after_goal_status_overwrites_memory(harness, monkeypatch):
    roi, *_ = harness
    prepare(roi, monkeypatch)
    roi.path_memory.finished = True
    roi.center_path = [(.3, 0., 0.), (.4, 0., 0.)]
    tick(roi)
    assert roi.memory_status == 'memory_goal_reached'
    assert roi.control_stop_reason == 'memory_goal_reached'
    assert roi.memory_update_reason == 'recovery_too_far_from_vehicle'
    summary = roi.control_log_summary()
    assert 'memory=memory_goal_reached' in summary
    assert 'Path update: recovery_too_far_from_vehicle' in summary
    roi.center_path = []
    tick(roi)
    assert roi.memory_update_reason == 'not_attempted: empty_path'


def test_update_log_reports_unusable_new_path_and_tf_failure(harness, monkeypatch):
    roi, *_ = harness
    tf, _ = prepare(roi, monkeypatch)
    roi.center_path = [(.2, -.3, 0.), (.2, .3, 0.)]
    tick(roi)
    assert roi.memory_update_reason == 'not_attempted: path_no_usable_forward_segment'
    roi.clear_path_memory()
    roi.center_path = [(.1, 0., 0.), (.5, 0., 0.)]
    tf.fail = True
    tick(roi)
    assert roi.memory_update_reason == 'save_tf_unavailable_or_stale'


def test_two_points_on_both_boundaries_use_existing_memory(harness, monkeypatch):
    roi, *_ = harness
    _, messages = prepare(roi, monkeypatch)
    roi.left_boundary = [(.1, .3, 0.), (.8, .3, 0.)]
    roi.right_boundary = [(.1, -.3, 0.), (.8, -.3, 0.)]
    roi.update_center_path(roi.get_clock().now().to_msg())
    assert not roi.center_path
    tick(roi)
    assert roi.control_mode == 'Memory Path Feedback'
    assert roi.memory_status == 'following'
    assert 0. < messages[-1].drive.speed <= .15


@pytest.mark.parametrize('bad_memory', ['lateral', 'too_far', 'finished'])
def test_unusable_memory_is_replaced_by_connected_live_path(harness, monkeypatch, bad_memory):
    roi, *_ = harness
    tf, messages = prepare(roi, monkeypatch)
    old = roi.path_memory.points[:]
    if bad_memory == 'lateral':
        tf.yaw = math.pi/2
    elif bad_memory == 'too_far':
        tf.y += 1.
    else:
        roi.path_memory.finished = True
    roi.center_path = [(.1, 0., 0.), (.3, 0., 0.), (.6, 0., 0.)]
    tick(roi)
    assert roi.control_mode == 'Memory Path Feedback'
    assert roi.memory_update_reason == 'replaced_unusable_memory'
    assert roi.path_memory.points != old and not roi.path_memory.finished
    assert roi.control_stop_reason is None
    assert 0. < messages[-1].drive.speed <= .15


@pytest.mark.parametrize('live_path,reason', [
    ([(.3, 0., 0.), (.8, 0., 0.)], 'recovery_too_far_from_vehicle'),
    ([(.05, .05, 0.), (.25, .4, 0.)], 'recovery_direction_mismatch'),
])
def test_unusable_memory_and_disconnected_live_path_stop(harness, monkeypatch, live_path, reason):
    roi, *_ = harness
    tf, messages = prepare(roi, monkeypatch)
    old = roi.path_memory.points[:]
    tf.yaw = math.pi/2
    roi.center_path = live_path
    assert roi.path_errors(live_path, .25) is not None
    tick(roi)
    assert roi.control_mode == 'Stopped' and messages[-1].drive.speed == 0.
    assert roi.path_memory.points == old
    assert roi.memory_update_reason == reason


def test_valid_live_path_cannot_replace_memory_during_tf_failure(harness, monkeypatch):
    roi, *_ = harness
    tf, messages = prepare(roi, monkeypatch)
    old = roi.path_memory.points[:]
    tf.fail = True
    roi.center_path = [(.1, 0., 0.), (.5, 0., 0.)]
    tick(roi)
    assert roi.path_memory.points == old
    assert roi.control_stop_reason == 'memory_tf_unavailable_or_stale'
    assert messages[-1].drive.speed == 0.


@pytest.mark.parametrize('length', [.1, .2])
def test_completed_memory_can_switch_to_short_new_path(harness, monkeypatch, length):
    roi, *_ = harness
    _, messages = prepare(roi, monkeypatch)
    roi.path_memory.finished = True
    roi.center_path = [(.1, 0., 0.), (.1+length, 0., 0.)]
    tick(roi)
    assert roi.memory_update_reason == 'replaced_unusable_memory'
    assert roi.control_mode == 'Memory Path Feedback'
    assert not roi.path_memory.finished
    assert 0. < messages[-1].drive.speed <= .15


def test_scan_tolerates_277ms_gap_while_memory_tf_limit_stays_200ms(harness, monkeypatch):
    roi, *_ = harness
    tf, messages = prepare(roi, monkeypatch)
    roi.center_path = []
    roi.control_received = time.monotonic()-.277
    roi.control_scan_stamp = roi.get_clock().now().nanoseconds-277_000_000
    roi.control_tick()
    assert roi.control_mode == 'Memory Path Feedback'
    assert messages[-1].drive.speed > 0.
    assert roi.get_parameter('control_timeout').value == .5
    assert roi.get_parameter('control_memory_tf_timeout').value == .2
    lookup = tf.lookup_transform

    def delayed_tf(*args):
        result = lookup(*args)
        result.header.stamp = Time(nanoseconds=roi.get_clock().now().nanoseconds-300_000_000).to_msg()
        return result

    monkeypatch.setattr(tf, 'lookup_transform', delayed_tf)
    tick(roi)
    assert roi.control_stop_reason == 'memory_tf_unavailable_or_stale'
    assert messages[-1].drive.speed == 0.
