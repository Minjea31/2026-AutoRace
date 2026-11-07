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
    assert roi.control_mode == 'Path Feedback'
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
    else:
        assert not roi.path_memory.points


def test_memory_rejoins_only_continuous_new_path(harness, monkeypatch):
    roi, *_ = harness
    prepare(roi, monkeypatch)
    roi.center_path = []
    tick(roi)
    assert roi.memory_following
    roi.center_path = [(.1, .4, 0.), (.8, .4, 0.)]
    tick(roi)
    assert roi.control_mode == 'Memory Path Feedback'
    assert roi.memory_status == 'rejoin_rejected'
    roi.center_path = [(.1, .03, 0.), (.8, .03, 0.)]
    tick(roi)
    assert roi.control_mode == 'Path Feedback'
    assert not roi.memory_following and roi.memory_status == 'stored'
