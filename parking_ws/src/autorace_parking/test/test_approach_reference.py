import math

import numpy as np
import pytest

from autorace_parking.approach_reference import ApproachReference, relative_covariance
from autorace_parking.localization import OdomSample, PoseHistory


def sample(t, pose):
    return OdomSample(t, pose, .2, 0., (.001, .001, .001))


def test_waits_for_bracketing_ekf_and_rotates_relative_position():
    history = PoseHistory()
    history.add(sample(10., (2., 3., math.pi/2)))
    reference = ApproachReference(.3)
    assert reference.request(10.05, 10.06)
    assert not reference.resolve(history, 10.06)
    history.add(sample(10.1, (2., 3.02, math.pi/2)))
    assert reference.resolve(history, 10.1)
    assert reference.origin == pytest.approx((2., 3.01, math.pi/2))
    assert reference.relative(sample(10.05, reference.origin)) == pytest.approx((0., 0., 0.))
    assert reference.relative(sample(11., (2., 4.01, math.pi/2+.2))) == pytest.approx((1., 0., .2))
    assert len(history.samples) == 2
    assert not reference.request(10.15, 10.15)
    assert reference.relative(sample(9., (0., 0., 0.))) is None


def test_expiry_reset_and_invalid_signals_do_not_create_an_origin():
    h = PoseHistory()
    ref = ApproachReference(.3)
    for stamp in (9., 11., float('nan')):
        assert not ref.request(stamp, 10.)
    assert ref.request(10., 10.)
    assert not ref.request(10.1, 10.1)
    assert not ref.resolve(h, 10.31)
    assert ref.pending is None and ref.origin is None and 'expired' in ref.reason
    ref.reset(11.)
    assert not ref.request(10.9, 11.)
    h.add(sample(11.1, (0., 0., 0.)))
    assert ref.request(11.1, 11.1) and ref.resolve(h, 11.1)
    ref.reset(12.)
    assert ref.origin is None


def test_gaps_discontinuity_and_yaw_wrap():
    h = PoseHistory()
    h.add(sample(10., (0., 0., math.pi-.01)))
    h.add(sample(10.1, (0., 0., -math.pi+.01)))
    ref = ApproachReference(.3)
    assert ref.request(10.05, 10.1) and ref.resolve(h, 10.1)
    assert abs(abs(ref.origin[2])-math.pi) < 1e-10
    assert ref.relative(sample(10.1, (0., 0., -math.pi+.01)))[2] == pytest.approx(.01)
    ref.reset()
    h.discontinuity = True
    assert ref.request(10.1, 10.1)
    assert not ref.resolve(h, 10.1) and ref.origin is None


def test_covariance_rotation_preserves_cross_terms_and_variance():
    covariance = np.diag([1., 4., 3., 2., 5., .1])
    covariance[0, 5] = covariance[5, 0] = .02
    rotated = np.array(relative_covariance(covariance.reshape(-1), math.pi/2)).reshape(6, 6)
    assert rotated[0, 0] == pytest.approx(4.)
    assert rotated[1, 1] == pytest.approx(1.)
    assert rotated[1, 5] == pytest.approx(-.02)
    assert rotated[3, 3] == pytest.approx(5.)
    assert rotated[4, 4] == pytest.approx(2.)
    assert rotated == pytest.approx(rotated.T)


def test_ros_adapter_publishes_separate_odometry_and_reset_rearms():
    import rclpy
    from rclpy.parameter import Parameter
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Header
    from types import SimpleNamespace
    from autorace_parking.node import ParkingNode

    rclpy.init()
    node = ParkingNode(parameter_overrides=[Parameter('localization_source', value='fused')])
    emitted = []
    node.approach_pub = SimpleNamespace(publish=emitted.append)
    clock = [100.]
    node.now_seconds = lambda: clock[0]
    def odom(t, x):
        msg = Odometry()
        ns = round(t*1e9)
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(ns, 10**9)
        msg.header.frame_id, msg.child_frame_id = 'parking_odom', 'base_link'
        msg.pose.pose.position.x = x
        msg.pose.pose.orientation.w = 1.
        for i in (0, 7, 35): msg.pose.covariance[i] = .001
        msg.twist.twist.linear.x = .2
        return msg
    def event(t):
        msg = Header(); ns = round(t*1e9)
        msg.stamp.sec, msg.stamp.nanosec = divmod(ns, 10**9)
        return msg
    try:
        tracker = node.tracker
        node.on_localization(odom(100., 2.))
        assert not emitted
        clock[0] = 100.06
        node.on_approach_start(event(100.05))
        assert node.approach_reference.origin is None
        clock[0] = 100.1
        original = odom(100.1, 2.02)
        node.on_localization(original)
        assert emitted[-1].header.frame_id == 'parking_approach'
        assert emitted[-1].child_frame_id == 'base_link'
        assert emitted[-1].pose.pose.position.x == pytest.approx(.01)
        assert emitted[-1].twist.twist.linear.x == .2
        assert original.header.frame_id == 'parking_odom'
        assert original.pose.pose.position.x == 2.02
        assert node.tracker is tracker and len(node.odometry.samples) == 2
        node.on_approach_start(event(100.1))
        assert node.approach_reference.stamp == 100.05
        node.reset_state()
        assert node.approach_reference.origin is None
        node.on_approach_start(event(100.05))
        assert node.approach_reference.pending is None
        clock[0] = 100.2
        node.on_localization(odom(100.2, 2.04))
        node.on_approach_start(event(100.2))
        assert node.approach_reference.stamp == 100.2
    finally:
        node.destroy_node()
        rclpy.shutdown()
