from types import SimpleNamespace
import pytest
from autorace_parking.approach_calibration import ApproachCalibration
from fusion_math import GyroCalibration


def healthy(t, generation=1, v=0.):
    return dict(stamp=t,ready=True,calibrating=False,calibration_generation=generation,
                wheel_speed_m_s=v,wheel_yaw_rate_rad_s=0.,wheel_age_s=.01,imu_age_s=.01,gyro_bias_rad_s=.015)


def test_one_request_and_failed_attempt_cannot_auto_rearm_or_accept_late_success():
    job=ApproachCalibration(timeout=3.)
    assert job.start(100.,0.)
    job.requested=True
    for i in range(100):
        assert not job.start(100.+i*.01,i*.01)
    job.response(True,1,.1)
    job.update(3.,103.,healthy(103.),SimpleNamespace(stamp=103.))
    assert job.phase=='FAILED'
    for i in range(100):
        job.response(True,1,4.+i)
        job.update(4.+i,104.+i,healthy(104.+i),SimpleNamespace(stamp=104.+i))
        assert not job.start(104.+i,4.+i) and job.phase=='FAILED'


@pytest.mark.parametrize('kind',['moving','stale','old_generation','missing','bad_number'])
def test_only_current_stationary_calibration_can_release(kind):
    job=ApproachCalibration();job.start(100.,0.);job.response(True,2,.1)
    h=healthy(101.,2)
    if kind=='moving':h['wheel_speed_m_s']=.3
    elif kind=='stale':h['imu_age_s']=1.
    elif kind=='old_generation':h['calibration_generation']=1
    elif kind=='missing':del h['gyro_bias_rad_s']
    else:h['wheel_speed_m_s']=float('nan')
    job.update(1.,101.,h,SimpleNamespace(stamp=101.))
    assert job.phase=='CALIBRATING'


def test_release_needs_two_distinct_health_samples_and_a_new_ekf_sample():
    job=ApproachCalibration();job.start(100.,0.);job.response(True,1,.1)
    for _ in range(5):job.update(.2,100.2,healthy(100.2),SimpleNamespace(stamp=100.2))
    assert job.ready_samples==1 and job.pending
    job.update(.3,100.3,healthy(100.3),SimpleNamespace(stamp=100.25))
    assert job.pending
    job.update(.32,100.32,healthy(100.3),SimpleNamespace(stamp=100.31))
    assert job.phase=='READY'
    assert not job.start(100.,2.)


def test_calibration_requires_continuous_stationary_data_after_a_gap():
    c=GyroCalibration(2.)
    for i in range(19):assert not c.update(i*.01,.015,0.,0.)
    assert not c.update(10.,.015,0.,0.)
    assert len(c.samples)==1
    for i in range(1,202):ready=c.update(10.+i*.01,.015,0.,0.)
    assert ready and c.bias==pytest.approx(.015)


def test_motion_and_noisy_calibration_never_complete():
    c=GyroCalibration(.3)
    for i in range(100):assert not c.update(i*.01,.015,.3,0.)
    for i in range(100):assert not c.update(2.+i*.01,.04*(-1)**i,0.,0.)
