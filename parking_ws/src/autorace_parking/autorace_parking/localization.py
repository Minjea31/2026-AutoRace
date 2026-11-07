"""Time-aligned EKF poses with bounded, keyframe-based LiDAR corrections.

No ROS imports: the sensor/clock and geometry contracts can be exercised offline.
Wheel/IMU prediction remains usable when scan geometry is degenerate. A failed
scan fit never replaces a valid pose with an unconstrained ICP result.
"""
from bisect import bisect_left
from collections import deque
from dataclasses import dataclass
import math

import numpy as np

from .core import wrap, advance
from .tracking import ScanTracker, compose, inverse


def interpolate_pose(a, b, fraction):
    return (a[0] + fraction * (b[0] - a[0]), a[1] + fraction * (b[1] - a[1]),
            wrap(a[2] + fraction * wrap(b[2] - a[2])))


@dataclass(frozen=True)
class OdomSample:
    stamp: float
    pose: tuple
    speed: float
    yaw_rate: float
    variance: tuple


class PoseHistory:
    def __init__(self, timeout=.3, max_speed=1.5, max_yaw_rate=3., max_std=.5):
        if not all(math.isfinite(v) and v > 0 for v in (timeout, max_speed, max_yaw_rate, max_std)):
            raise ValueError('Localization time, velocity and covariance bounds must be positive')
        self.samples = deque(maxlen=400)
        self.timeout, self.max_speed = timeout, max_speed
        self.max_yaw_rate, self.max_std = max_yaw_rate, max_std
        self.reason = ''
        self.discontinuity = False

    def add(self, sample):
        values = (sample.stamp, *sample.pose, sample.speed, sample.yaw_rate, *sample.variance)
        if not all(math.isfinite(v) for v in values) or min(sample.variance) <= 0:
            self.reason = 'Invalid localization values/covariance'
            return False
        if max(sample.variance) > self.max_std ** 2:
            self.reason = 'Localization covariance exceeds configured limit'
            return False
        if abs(sample.speed) > self.max_speed or abs(sample.yaw_rate) > self.max_yaw_rate:
            self.reason = 'Localization velocity exceeds physical bound'
            return False
        if self.samples:
            previous = self.samples[-1]
            dt = sample.stamp - previous.stamp
            if dt <= 0:
                return False
            if (math.hypot(sample.pose[0]-previous.pose[0], sample.pose[1]-previous.pose[1]) > self.max_speed*dt+.03
                    or abs(wrap(sample.pose[2]-previous.pose[2])) > self.max_yaw_rate*dt+.05):
                self.discontinuity = True
                self.reason = 'Localization pose jumped; reset parking after checking sensors'
                return False
        self.samples.append(sample)
        self.reason = ''
        return True

    def at(self, stamp):
        if not self.samples or stamp < self.samples[0].stamp:
            return None
        samples = list(self.samples)
        i = bisect_left([s.stamp for s in samples], stamp)
        if i < len(samples) and abs(samples[i].stamp-stamp) < 1e-8:
            return samples[i]
        # Do not project the last velocity across a sensor outage.
        if i == len(samples):
            if stamp-samples[-1].stamp > .03:
                return None
            return samples[-1]
        a, b = samples[i-1], samples[i]
        if b.stamp-a.stamp > self.timeout:
            return None
        f = (stamp-a.stamp)/(b.stamp-a.stamp)
        return OdomSample(stamp, interpolate_pose(a.pose, b.pose, f),
                          a.speed+f*(b.speed-a.speed), a.yaw_rate+f*(b.yaw_rate-a.yaw_rate),
                          tuple(x+f*(y-x) for x, y in zip(a.variance, b.variance)))

    def healthy(self, now):
        if self.discontinuity:
            return False
        if not self.samples:
            self.reason = 'Waiting for calibrated wheel/IMU EKF odometry'
            return False
        if not -.05 <= now-self.samples[-1].stamp <= self.timeout:
            self.reason = 'EKF odometry is stale or future-dated'
            return False
        return not self.reason


def sparse_points(points, count=240):
    array = np.asarray(points, dtype=float).reshape(-1, 2)
    array = array[np.isfinite(array).all(axis=1)]
    return array[np.linspace(0, len(array)-1, count, dtype=int)] if len(array) > count else array


class KeyframeMatcher:
    def __init__(self, points):
        self.reference = sparse_points(points)
        self.target, self.normals = ScanTracker.surfaces(self.reference)

    def stationary_scene_changed(self, points, prior):
        current = sparse_points(points)
        if len(current) < 12 or len(self.reference) < 12:
            return False
        c, s = math.cos(prior[2]), math.sin(prior[2])
        moved = current @ np.array([[c, -s], [s, c]]).T + prior[:2]
        nearest = ((moved[:, None, :]-self.reference[None, :, :])**2).sum(axis=2).min(axis=1)
        return float(np.mean(nearest > .08**2)) > .30

    def match(self, points, prior):
        current = sparse_points(points)
        if len(self.target) < 12 or len(current) < 12:
            raise ValueError('Insufficient reference surfaces')
        c, s = math.cos(prior[2]), math.sin(prior[2])
        rotation = np.array([[c, -s], [s, c]])
        translation = np.asarray(prior[:2], dtype=float)
        error = math.inf
        for _ in range(12):
            moved = current @ rotation.T + translation
            distances = ((moved[:, None, :]-self.target[None, :, :])**2).sum(axis=2)
            nearest = distances.argmin(axis=1)
            normals = self.normals[nearest]
            residual = ((moved-self.target[nearest])*normals).sum(axis=1)
            mask = (distances[np.arange(len(moved)), nearest] < .18**2) & (abs(residual) < .06)
            if mask.sum() < max(12, len(current)*.45):
                raise ValueError('Insufficient keyframe overlap')
            a, n, r = moved[mask], normals[mask], residual[mask]
            jacobian = np.column_stack((n[:, 0], n[:, 1], -n[:, 0]*a[:, 1]+n[:, 1]*a[:, 0]))
            weights = np.sqrt(np.minimum(1., .01/np.maximum(abs(r), 1e-8)))
            jacobian, residual = jacobian*weights[:, None], r*weights
            singular = np.linalg.svd(jacobian, compute_uv=False)
            if singular[-1] < .05 or singular[0]/singular[-1] > 100:
                raise ValueError('Degenerate keyframe geometry')
            dx, dy, yaw = np.linalg.lstsq(jacobian, -residual, rcond=None)[0]
            step = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
            translation = step @ translation + (dx, dy)
            rotation = step @ rotation
            error = float(np.sqrt(np.mean(residual**2)))
            if math.hypot(dx, dy) < 1e-5 and abs(yaw) < 1e-5:
                break
        if error > .02:
            raise ValueError('Keyframe residual exceeds 2 cm')
        return (float(translation[0]), float(translation[1]), math.atan2(rotation[1, 0], rotation[0, 0])), error


class FusedTracker:
    def __init__(self, history, correction_gain=.15, max_correction=.04, max_yaw_correction=.05):
        self.history = history
        self.gain, self.max_correction, self.max_yaw_correction = correction_gain, max_correction, max_yaw_correction
        self.pose = (0., 0., 0.)
        self.global_pose = (0., 0., 0.)
        self.offset = (0., 0., 0.)
        self.origin = None
        self.valid, self.reason, self.error = False, 'Waiting for EKF and scan', 0.
        self.matcher = None
        self.key_pose = None
        self.key_stamp = None
        self.last_stamp = None
        self.lidar_status = 'waiting'
        self.corrections = self.rejected = 0
        self.stationary = False
        self.sample = None
        self.latched = False
        self.stationary_disagreements = 0

    def update(self, points, stamp):
        if self.latched or self.history.discontinuity:
            self.valid = False
            self.reason = self.history.reason or self.reason
            return False
        sample = self.history.at(stamp)
        if sample is None:
            self.valid, self.reason = False, 'Waiting for EKF pose at scan acquisition time'
            return False
        self.sample = sample
        predicted = compose(self.offset, sample.pose)
        self.stationary = abs(sample.speed) < .012 and abs(sample.yaw_rate) < .012
        if self.origin is None:
            self.origin = predicted
            self.global_pose = predicted
        if self.matcher is not None:
            prior = compose(inverse(self.key_pose), predicted)
            if self.stationary and self.matcher.stationary_scene_changed(points, prior):
                self.stationary_disagreements += 1
                if self.stationary_disagreements >= 3:
                    self.latched, self.valid = True, False
                    self.reason = 'LiDAR detects motion inconsistent with stationary wheel/IMU; reset required'
                    return False
            else:
                self.stationary_disagreements = 0
            try:
                relative, self.error = self.matcher.match(points, prior)
                candidate = compose(self.key_pose, relative)
                innovation = (candidate[0]-predicted[0], candidate[1]-predicted[1], wrap(candidate[2]-predicted[2]))
                position_error = math.hypot(*innovation[:2])
                if position_error > self.max_correction or abs(innovation[2]) > self.max_yaw_correction:
                    self.rejected += 1
                    self.lidar_status = 'rejected: disagrees with wheel/IMU prediction'
                elif self.stationary:
                    # Validate against the fixed keyframe, but never integrate its
                    # measurement noise while wheel and IMU report no movement.
                    self.lidar_status = 'stationary: correction held'
                else:
                    predicted = (predicted[0]+self.gain*innovation[0], predicted[1]+self.gain*innovation[1],
                                 wrap(predicted[2]+self.gain*innovation[2]))
                    self.corrections += 1
                    self.lidar_status = 'accepted'
            except ValueError as exc:
                self.rejected += 1
                self.lidar_status = 'EKF only: '+str(exc)
        if self.stationary and self.last_stamp is not None:
            predicted = self.global_pose
        self.global_pose = predicted
        self.offset = compose(predicted, inverse(sample.pose))
        self.pose = compose(inverse(self.origin), predicted)
        if (self.matcher is None or (not self.stationary and
                (math.hypot(predicted[0]-self.key_pose[0], predicted[1]-self.key_pose[1]) > .20
                 or abs(wrap(predicted[2]-self.key_pose[2])) > .20 or stamp-self.key_stamp > 2.))):
            self.matcher = KeyframeMatcher(points)
            self.key_pose, self.key_stamp = predicted, stamp
        self.last_stamp = stamp
        self.valid, self.reason = True, ''
        return True

    def healthy(self, now):
        if not self.history.healthy(now):
            self.valid, self.reason = False, self.history.reason
            return False
        return self.valid

    def current_pose(self, now):
        """Predict only the short EKF publication interval, not LiDAR latency."""
        if self.origin is None or not self.history.samples:
            return self.pose
        sample = self.history.samples[-1]
        if self.stationary and abs(sample.speed) < .012 and abs(sample.yaw_rate) < .012:
            return self.pose
        dt = max(0., min(.05, now-sample.stamp))
        if abs(sample.speed) > 1e-8:
            delta = advance((0., 0., 0.), sample.speed*dt, sample.yaw_rate/sample.speed)
        else:
            delta = (0., 0., sample.yaw_rate*dt)
        predicted = compose(self.offset, compose(sample.pose, delta))
        return compose(inverse(self.origin), predicted)

    def diagnostics(self, now):
        return {'source': 'wheel_imu_ekf_lidar', 'valid': self.valid,
                'reason': self.reason, 'stationary': self.stationary,
                'pose': self.pose, 'global_pose': self.global_pose,
                'pose_stamp': self.last_stamp, 'control_pose': self.current_pose(now),
                'ekf_age_s': now-self.history.samples[-1].stamp if self.history.samples else None,
                'variance': self.sample.variance if self.sample else None,
                'lidar_status': self.lidar_status, 'lidar_residual_m': self.error,
                'lidar_corrections': self.corrections, 'lidar_rejections': self.rejected}


