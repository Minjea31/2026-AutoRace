"""Short-range scan registration; does not subscribe to odom, IMU or TF."""
import math
import numpy as np
from scipy.spatial import cKDTree

from .core import wrap


def compose(a, b):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], wrap(a[2] + b[2]))


def inverse(pose):
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return -c * x - s * y, s * x - c * y, -yaw


def scale_motion(pose, factor):
    """Extrapolate a constant planar body velocity over a different interval."""
    x, y, yaw = pose
    if abs(yaw) < 1e-8:
        return x * factor, y * factor, yaw * factor
    angle = (factor - 1) * yaw / 2
    scale = math.sin(factor * yaw / 2) / math.sin(yaw / 2)
    c, s = math.cos(angle), math.sin(angle)
    return scale * (c * x - s * y), scale * (s * x + c * y), wrap(yaw * factor)


class ScanTracker:
    """Point-to-line ICP, using local surface normals rather than ray indices.

    Wall-only/degenerate geometry, low overlap, large jumps or high residual
    cause a latched failure. The caller stops and requires a mission reset.
    """
    def __init__(self, max_speed=1.0, max_yaw_rate=2.5, max_interval=.5):
        self.previous = None
        self.pose = (0.0, 0.0, 0.0)
        self.error = 0.0
        self.valid = True
        self.last_motion = (0.0, 0.0, 0.0)
        self.reason = ''
        self.last_stamp = None
        self.interval = .1
        self.previous_interval = .1
        self.max_speed, self.max_yaw_rate, self.max_interval = max_speed, max_yaw_rate, max_interval

    def fail(self, reason):
        self.valid, self.reason = False, reason
        return False

    @staticmethod
    def surfaces(points):
        if len(points) < 3:
            return np.empty((0,2)),np.empty((0,2))
        distances, indices = cKDTree(points).query(points,k=min(7,len(points)),workers=1)
        valid=distances < .14
        counts=valid.sum(axis=1)
        neighbors=points[indices]
        means=(neighbors*valid[:,:,None]).sum(axis=1)/np.maximum(counts[:,None],1)
        centered=neighbors-means[:,None,:]
        covariance=np.einsum('nki,nkj,nk->nij',centered,centered,valid)/np.maximum(counts[:,None,None],1)
        values,vectors=np.linalg.eigh(covariance)
        keep=(counts>=3)&(values[:,1]>=.00003)&(values[:,0]<=values[:,1]*.12)
        return points[keep],vectors[keep,:,0]

    def update(self, points, stamp=None):
        if not self.valid:
            return False
        if stamp is not None:
            if self.last_stamp is not None:
                self.interval = stamp - self.last_stamp
            self.last_stamp = stamp
        else:
            self.interval = .1
        current = np.asarray(points, dtype=float).reshape(-1, 2)
        if len(current) < 12:
            return self.fail(f'Only {len(current)} scan points')
        if self.previous is None:
            self.previous = current
            return True
        if not 0 < self.interval <= self.max_interval + 1e-8:
            return self.fail(f'Invalid scan interval: {self.interval:.3f}s')
        target, normals = self.surfaces(self.previous)
        if len(target) < 12:
            return self.fail(f'Only {len(target)} reference surface points')
        # Use acquisition timestamps, since sensor delivery can skip a frame.
        # Both extrapolation and admissible movement scale with elapsed time.
        dx, dy, yaw = scale_motion(self.last_motion, self.interval / self.previous_interval)
        total_r = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
        total_t = np.array([dx, dy])
        moved = current @ total_r.T + total_t
        for _ in range(20):
            distances = ((moved[:, None, :] - target[None, :, :]) ** 2).sum(axis=2)
            nearest = distances.argmin(axis=1)
            n = normals[nearest]
            residual = ((moved - target[nearest]) * n).sum(axis=1)
            valid = (distances[np.arange(len(moved)), nearest] < .12 ** 2) & (abs(residual) < .05)
            if valid.sum() < max(12, len(current) * .35):
                return self.fail(f'Insufficient overlap: {int(valid.sum())}/{len(current)}, dt={self.interval:.3f}s')
            a, n, r = moved[valid], n[valid], residual[valid]
            jacobian = np.column_stack((n[:, 0], n[:, 1], -n[:, 0] * a[:, 1] + n[:, 1] * a[:, 0]))
            weight = np.sqrt(np.minimum(1.0, .01 / np.maximum(abs(r), 1e-8)))
            jacobian, residual = jacobian * weight[:, None], r * weight
            singular = np.linalg.svd(jacobian, compute_uv=False)
            if singular[-1] < .05 or singular[0] / singular[-1] > 100:
                return self.fail(f'Degenerate scan geometry, dt={self.interval:.3f}s')
            dx, dy, yaw = np.linalg.lstsq(jacobian, -residual, rcond=None)[0]
            rotation = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
            translation = np.array([dx, dy])
            moved = moved @ rotation.T + translation
            total_t = rotation @ total_t + translation
            total_r = rotation @ total_r
            self.error = float(np.sqrt(np.mean(residual ** 2)))
            if math.hypot(dx, dy) < 1e-5 and abs(yaw) < 1e-5:
                break
        yaw = math.atan2(total_r[1, 0], total_r[0, 0])
        if (self.error > .025 or np.linalg.norm(total_t) > self.max_speed * self.interval + .02 or
                abs(yaw) > self.max_yaw_rate * self.interval + .03):
            return self.fail(f'ICP limits: error={self.error:.4f}m, translation={np.linalg.norm(total_t):.3f}m, '
                             f'yaw={yaw:.3f}rad, dt={self.interval:.3f}s')
        self.last_motion = (float(total_t[0]), float(total_t[1]), yaw)
        self.pose = compose(self.pose, self.last_motion)
        self.previous_interval = self.interval
        self.previous = current
        return True
