"""One bounded stationary calibration per approach; failures require reset."""
import math


class ApproachCalibration:
    def __init__(self, timeout=10., sensor_timeout=.3):
        if not all(math.isfinite(v) and v > 0 for v in (timeout, sensor_timeout)):
            raise ValueError('Calibration timeouts must be positive')
        self.timeout, self.sensor_timeout = timeout, sensor_timeout
        self.phase = 'IDLE'
        self.stamp = -math.inf
        self.deadline = self.ready_at = None
        self.generation = None
        self.requested = False
        self.reason = ''
        self.ready_samples = 0
        self.last_health_stamp = -math.inf

    @property
    def pending(self):
        return self.phase in ('WAITING_SERVICE', 'CALIBRATING')

    def start(self, stamp, now):
        if self.pending or self.phase == 'FAILED' or stamp <= self.stamp:
            return False
        self.phase, self.stamp = 'WAITING_SERVICE', stamp
        self.deadline = now+self.timeout
        self.ready_at = self.generation = None
        self.requested = False
        self.ready_samples = 0
        self.last_health_stamp = -math.inf
        self.reason = 'Waiting for stationary gyro calibration'
        return True

    def fail(self, reason):
        self.phase, self.reason = 'FAILED', reason

    def response(self, success, generation, now):
        if self.phase != 'WAITING_SERVICE':
            return
        if now >= self.deadline:
            self.fail('Parking calibration timed out; reset required')
        elif not success or not isinstance(generation, int) or isinstance(generation, bool):
            self.fail('Parking calibration request failed; reset required')
        else:
            self.phase, self.generation = 'CALIBRATING', generation

    def update(self, now, ros_now, health, odometry):
        if not self.pending:
            return
        if now >= self.deadline:
            self.fail('Parking calibration timed out; reset required')
            return
        if self.phase != 'CALIBRATING' or health is None or odometry is None:
            return
        try:
            stamp = health['stamp']
            numbers = [stamp, health['wheel_speed_m_s'], health['wheel_yaw_rate_rad_s'],
                       health['wheel_age_s'], health['imu_age_s'], health['gyro_bias_rad_s']]
            valid = (all(math.isfinite(v) for v in numbers)
                     and health['calibration_generation'] == self.generation
                     and health['ready'] is True and not health['calibrating']
                     and 0 <= ros_now-stamp <= self.sensor_timeout
                     and 0 <= health['wheel_age_s'] <= self.sensor_timeout
                     and 0 <= health['imu_age_s'] <= self.sensor_timeout
                     and abs(health['wheel_speed_m_s']) < .01
                     and abs(health['wheel_yaw_rate_rad_s']) < .02)
        except (KeyError, TypeError, ValueError):
            valid = False
        if not valid:
            self.ready_samples = 0
            return
        if stamp > self.last_health_stamp:
            self.ready_samples += 1
            self.last_health_stamp = stamp
        if (self.ready_samples >= 2 and odometry.stamp >= self.last_health_stamp
                and 0 <= ros_now-odometry.stamp <= self.sensor_timeout):
            self.phase, self.ready_at, self.reason = 'READY', now, ''
