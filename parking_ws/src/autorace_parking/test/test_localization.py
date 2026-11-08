import math
from dataclasses import replace

import numpy as np
import pytest

from autorace_parking.core import advance, wrap
from autorace_parking.localization import FusedTracker, OdomSample, PoseHistory
from autorace_parking.tracking import compose, inverse
from fusion_math import GyroCalibration


def sample(t, pose=(0., 0., 0.), speed=0., yaw_rate=0.):
    return OdomSample(t, pose, speed, yaw_rate, (.001, .001, .001))


def scene():
    return [(x, 1.2) for x in np.linspace(-1.5, 3., 120)] + [(2.5, y) for y in np.linspace(-2., 1., 100)]


def test_history_aligns_scan_time_across_yaw_wrap_and_rejects_outages():
    history = PoseHistory()
    assert history.add(sample(10., (0., 0., math.pi-.01)))
    assert history.add(sample(10.1, (.02, 0., -math.pi+.01)))
    pose = history.at(10.05).pose
    assert pose[:2] == pytest.approx((.01, 0.))
    assert abs(abs(pose[2])-math.pi) < 1e-6
    assert history.at(9.) is None and history.at(10.2) is None
    assert not history.healthy(11.)
    assert not history.add(replace(sample(10.2), variance=(0., 0., 0.)))
    assert not history.add(sample(10.2, (2., 0., 0.)))
    assert history.discontinuity


def test_stationary_scan_noise_does_not_accumulate_or_record_fake_motion():
    rng = np.random.default_rng(41)
    points = np.asarray(scene())
    history = PoseHistory()
    tracker = FusedTracker(history)
    for i in range(100):
        t = i*.1
        # EKF's small stationary measurement jitter must not accumulate either.
        pose = (.0001*i, -.00005*i, .00003*i)
        assert history.add(sample(t, pose))
        assert tracker.update(points+rng.normal(0., .004, points.shape), t)
        assert tracker.pose == pytest.approx((0., 0., 0.), abs=1e-10)
    assert tracker.corrections == 0 and tracker.stationary
    assert tracker.key_stamp == 0.


def test_lidar_corrects_biased_wheel_prediction_during_forward_and_reverse():
    for direction in (1., -1.):
        points = scene()
        history = PoseHistory()
        tracker = FusedTracker(history)
        truth = raw = (0., 0., 0.)
        for i in range(101):
            if i:
                truth = compose(truth, advance((0., 0., 0.), direction*.015, .3))
                raw = compose(raw, advance((0., 0., 0.), direction*.015*1.04, .3))
            assert history.add(sample(i*.1, raw, direction*.15, direction*.045))
            observed = [compose(inverse(truth), (*p, 0.))[:2] for p in points]
            assert tracker.update(observed, i*.1)
        fused_error = math.hypot(tracker.pose[0]-truth[0], tracker.pose[1]-truth[1])
        raw_error = math.hypot(raw[0]-truth[0], raw[1]-truth[1])
        assert tracker.corrections > 50
        assert fused_error < raw_error*.6
        assert fused_error < .03


def test_degenerate_lidar_keeps_measured_ekf_motion_without_scan_only_fallback():
    history = PoseHistory()
    tracker = FusedTracker(history)
    points = [(x, 1.) for x in np.linspace(-1., 2., 100)]
    for i in range(10):
        assert history.add(sample(i*.1, (.01*i, 0., 0.), .1))
        assert tracker.update(points, i*.1)
    assert tracker.pose[0] == pytest.approx(.09)
    assert 'Degenerate' in tracker.lidar_status
    assert not tracker.healthy(2.)
    assert not tracker.valid
    assert not tracker.update(points, 2.)


def test_outlier_scan_cannot_drag_pose_away_from_wheel_and_imu():
    history = PoseHistory()
    tracker = FusedTracker(history)
    points = scene()
    assert history.add(sample(0.))
    assert tracker.update(points, 0.)
    assert history.add(sample(.1, (.01, 0., 0.), .1))
    wrong = [(x-.12, y) for x, y in points]
    assert tracker.update(wrong, .1)
    assert tracker.pose == pytest.approx((.01, 0., 0.))
    assert tracker.rejected == 1


def test_gyro_bias_requires_continuous_stationary_samples_and_removes_bias():
    calibration = GyroCalibration(duration=2.)
    for i in range(15):
        assert not calibration.update(i*.1, .015, 0., 0.)
    assert not calibration.update(1.5, .015, .1, 0.)
    for i in range(21):
        ready = calibration.update(2.+i*.1, .015, 0., 0.)
    assert ready and calibration.bias == pytest.approx(.015)


def test_unmeasured_manual_translation_latches_instead_of_inventing_a_pose():
    history = PoseHistory()
    tracker = FusedTracker(history)
    points = scene()
    assert history.add(sample(0.))
    assert tracker.update(points, 0.)
    shifted = [(x-.12, y) for x, y in points]
    for i in range(1,4):
        assert history.add(sample(i*.1))
        valid = tracker.update(shifted, i*.1)
    assert not valid and tracker.latched
    assert 'inconsistent' in tracker.reason


def test_current_control_pose_compensates_scan_latency_without_outage_extrapolation():
    history = PoseHistory()
    tracker = FusedTracker(history)
    assert history.add(sample(0.))
    assert tracker.update(scene(), 0.)
    assert history.add(sample(.1, (.01,0.,0.), .1))
    assert tracker.pose == (0.,0.,0.)  # geometry still belongs to the old scan
    assert tracker.current_pose(.15) == pytest.approx((.015,0.,0.))
    assert tracker.current_pose(10.) == pytest.approx((.015,0.,0.))
    assert not tracker.healthy(10.)


def test_imu_mounting_rotation_handles_an_upside_down_sensor():
    from fusion_math import rotate_vector
    assert rotate_vector((0.,0.,1.), (1.,0.,0.,0.)) == pytest.approx((0.,0.,-1.))
    assert rotate_vector((0.,0.,1.), (0.,math.sqrt(.5),0.,math.sqrt(.5))) == pytest.approx((1.,0.,0.))
    with pytest.raises(ValueError): rotate_vector((0.,0.,1.), (0.,0.,0.,0.))


@pytest.mark.parametrize('variance,accepted', [
    ((1.0, 1.0, .25), True),
    ((1.0001, .2, .1), False),
    ((.2, 1.0001, .1), False),
    ((.2, .2, .2501), False),
])
def test_one_metre_xy_limit_keeps_original_half_radian_yaw_limit(variance, accepted):
    history = PoseHistory(max_std=1., max_yaw_std=.5)
    assert history.add(replace(sample(1.), variance=variance)) is accepted
