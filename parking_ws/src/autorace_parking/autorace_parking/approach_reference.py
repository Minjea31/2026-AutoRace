"""An independent parking approach origin; never changes EKF or map coordinates."""
import math

import numpy as np

from .tracking import compose, inverse


class ApproachReference:
    def __init__(self, timeout):
        self.timeout = timeout
        self.reset()

    def reset(self, now=-math.inf):
        self.not_before = now
        self.pending = None
        self.origin = None
        self.stamp = None
        self.reason = ''

    def request(self, stamp, now):
        if self.origin is not None or self.pending is not None:
            return False
        if (not math.isfinite(stamp) or not math.isfinite(now)
                or stamp < self.not_before or not 0 <= now-stamp <= self.timeout):
            self.reason = 'Rejected stale, future or pre-reset approach start'
            return False
        self.pending = stamp
        self.reason = 'Waiting for EKF pose at approach start'
        return True

    def resolve(self, history, now):
        if self.pending is None:
            return False
        if now < self.pending or now-self.pending > self.timeout or history.discontinuity:
            self.pending = None
            self.reason = 'Approach start expired or EKF discontinuity detected'
            return False
        # Require actual samples bracketing the event; do not extrapolate.
        if not history.samples or history.samples[-1].stamp < self.pending:
            return False
        sample = history.at(self.pending)
        if sample is None:
            return False
        self.origin, self.stamp = sample.pose, self.pending
        self.pending = None
        self.reason = ''
        return True

    def relative(self, sample):
        if self.origin is None or sample.stamp < self.stamp:
            return None
        return compose(inverse(self.origin), sample.pose)


def relative_covariance(covariance, origin_yaw):
    # The captured origin defines a fixed frame, not another independent sample.
    c, s = math.cos(origin_yaw), math.sin(origin_yaw)
    transform = np.eye(6)
    transform[:2, :2] = ((c, s), (-s, c))
    transform[3:5, 3:5] = ((c, s), (-s, c))
    result = transform @ np.asarray(covariance).reshape(6, 6) @ transform.T
    return result.reshape(36).tolist()
