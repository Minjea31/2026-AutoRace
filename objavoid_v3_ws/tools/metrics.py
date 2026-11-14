"""평가 전용 기하 함수(정답 좌표 사용). 회피 노드는 이 모듈을 쓰지 않는다."""
import math

import numpy as np


class Rect:
    def __init__(self, x, y, yaw, length, width):
        self.x, self.y, self.yaw, self.length, self.width = x, y, yaw, length, width

    def corners(self):
        p = np.array([[1, 1], [-1, 1], [-1, -1], [1, -1]]) * [self.length / 2, self.width / 2]
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return p @ np.array([[c, s], [-s, c]]) + [self.x, self.y]


def _seg_point(a, b, p):
    ab = b - a
    t = np.clip(np.dot(p - a, ab) / max(np.dot(ab, ab), 1e-12), 0, 1)
    return np.linalg.norm(a + t * ab - p)


def _inside(poly, p):
    s = None
    for i in range(4):
        a, b = poly[i], poly[(i + 1) % 4]
        c = (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0])
        if s is None:
            s = c > 0
        elif (c > 0) != s:
            return False
    return True


def rect_distance(a, b):
    """두 직사각형(.corners() 제공) 사이 최소 거리. 겹치면 0."""
    pa, pb = a.corners(), b.corners()
    if any(_inside(pa, q) for q in pb) or any(_inside(pb, q) for q in pa):
        return 0.0
    best = np.inf
    for P, Q in ((pa, pb), (pb, pa)):
        for i in range(4):
            for q in Q:
                best = min(best, _seg_point(P[i], P[(i + 1) % 4], q))
    return float(best)


def section_obstacles(cfg, x_range=(3.9, 8.6), y_range=(0.0, 1.9)):
    """장애물 구간의 정차 차량 직사각형(평가용)."""
    v = cfg['vehicle']
    out = []
    for o in cfg['world']['obstacles']:
        x, y, yaw = o['pose']
        if x_range[0] <= x <= x_range[1] and y_range[0] <= y <= y_range[1]:
            out.append(Rect(x, y, yaw, v['length'], v['width']))
    return out
