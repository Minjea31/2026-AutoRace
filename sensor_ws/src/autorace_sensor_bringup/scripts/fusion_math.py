"""ROS-independent IMU mounting and stationary bias calibration."""
import math
import numpy as np


def rotate_vector(vector, quaternion):
    v, q = np.asarray(vector, dtype=float), np.asarray(quaternion, dtype=float)
    if not np.isfinite(v).all() or not np.isfinite(q).all() or abs(float(q@q)-1.) > .01:
        raise ValueError('Invalid gyro mounting rotation')
    return v + 2*np.cross(q[:3], np.cross(q[:3], v)+q[3]*v)


class GyroCalibration:
    def __init__(self, duration=2., max_bias=.05, max_gap=.2):
        self.duration, self.max_bias = duration, max_bias
        self.max_gap = max_gap
        self.samples = []
        self.bias = None

    def update(self, stamp, gyro, wheel_speed, wheel_yaw_rate):
        if not all(math.isfinite(v) for v in (stamp, gyro, wheel_speed, wheel_yaw_rate)):
            return False
        if self.bias is not None:
            return True
        if abs(wheel_speed) > .01 or abs(wheel_yaw_rate) > .02 or abs(gyro) > self.max_bias:
            self.samples.clear()
            return False
        if self.samples and stamp <= self.samples[-1][0]:
            return False
        if self.samples and stamp-self.samples[-1][0] > self.max_gap:
            self.samples.clear()
        self.samples.append((stamp, gyro))
        if stamp-self.samples[0][0] < self.duration or len(self.samples) < 20:
            return False
        values = [value for _, value in self.samples]
        if np.std(values) > .01:
            self.samples.clear()
            return False
        self.bias = float(np.mean(values))
        return True
