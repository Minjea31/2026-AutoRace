"""도로 좌표계와 주행 가능 경계.

도로 좌표 (s, d): 활성화 순간의 odom 자세를 원점으로, s = 전방, d = 왼쪽.
3차로 구간이 직선이라는 가정만 쓴다. 경계는 `limits(s)` 인터페이스로 분리해
나중에 카메라 노란 실선 검출 모듈로 교체할 수 있게 한다.
"""
import math

import numpy as np


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class RoadFrame:
    def __init__(self, x0, y0, yaw0):
        self.x0, self.y0, self.yaw0 = float(x0), float(y0), float(yaw0)
        self.c, self.s = math.cos(yaw0), math.sin(yaw0)

    def to_road(self, x, y):
        dx, dy = np.asarray(x) - self.x0, np.asarray(y) - self.y0
        return self.c * dx + self.s * dy, -self.s * dx + self.c * dy

    def to_odom(self, s, d):
        s, d = np.asarray(s), np.asarray(d)
        return self.x0 + self.c * s - self.s * d, self.y0 + self.s * s + self.c * d

    def pose_to_road(self, x, y, yaw):
        s, d = self.to_road(x, y)
        return float(s), float(d), wrap(yaw - self.yaw0)

    def yaw_to_odom(self, psi):
        return np.asarray(psi) + self.yaw0


class PriorBoundary:
    """사전 설정 도로 경계(시뮬레이터 도로 폭). d_right < 0 < d_left.

    교체용 인터페이스: limits(s) -> (right, left), s_max.
    """

    def __init__(self, s_pts, left_pts, right_pts):
        self.s_pts = np.asarray(s_pts, float)
        self.left = np.asarray(left_pts, float)
        self.right = np.asarray(right_pts, float)
        if not (len(self.s_pts) == len(self.left) == len(self.right) >= 2):
            raise ValueError('boundary arrays must have equal length >= 2')
        if np.any(np.diff(self.s_pts) <= 0) or np.any(self.left <= self.right):
            raise ValueError('boundary s must increase and left > right')
        self.s_min, self.s_max = float(self.s_pts[0]), float(self.s_pts[-1])

    def limits(self, s):
        s = np.asarray(s)
        return np.interp(s, self.s_pts, self.right), np.interp(s, self.s_pts, self.left)
