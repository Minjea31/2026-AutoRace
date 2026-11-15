"""LiDAR 장애물 검출·추적과 관측 격자(Local Map).

- 스캔 점을 스캔 시각의 odom 자세로 도로 좌표에 고정한다.
- 도로 ROI 안 점만 각도 순서로 묶고, 클러스터를 도로 축 정렬 박스(AABB)로 만든다.
- 박스는 LIMO 실치수보다 작아지지 않도록 가려진 쪽으로 확장한다.
- 추적 박스는 도로(=odom 고정) 좌표에 저장하므로 회전·가림에도 유지된다.
- 관측 격자는 레이저가 실제로 지나간 칸만 '빈 공간'으로 기록한다(미관측 != 빈 공간).
"""
import math
from dataclasses import dataclass

import numpy as np


@dataclass
class Box:
    s0: float
    s1: float
    d0: float
    d1: float

    @property
    def cs(self):
        return 0.5 * (self.s0 + self.s1)

    @property
    def cd(self):
        return 0.5 * (self.d0 + self.d1)

    def union(self, o):
        return Box(min(self.s0, o.s0), max(self.s1, o.s1), min(self.d0, o.d0), max(self.d1, o.d1))

    def gap_to(self, o, shrink=0.02):
        ds = max(o.s0 - self.s1 + shrink, self.s0 - o.s1 + shrink, 0.0)
        dd = max(o.d0 - self.d1 + shrink, self.d0 - o.d1 + shrink, 0.0)
        return math.hypot(ds, dd)


@dataclass
class Track:
    id: int
    box: Box
    hits: int
    first_seen: float
    last_seen: float
    confirmed: bool = False
    ghost_time: float = 0.0
    hist: list = None  # 최근 관측 박스(합집합 창)


def scan_points(ranges, angle_min, angle_inc, lidar_pose, p):
    """유효 빔의 (인덱스, 거리, odom x, y)."""
    r = np.asarray(ranges, dtype=float)
    idx = np.arange(len(r))
    ok = np.isfinite(r) & (r > p.scan_range_min)
    lx, ly, lyaw = lidar_pose
    th = angle_min + idx * angle_inc + lyaw
    x = lx + r * np.cos(th)
    y = ly + r * np.sin(th)
    return idx, r, ok, th, x, y


def correct_extent(lo, hi, sensor, min_size, occluded_lo, occluded_hi):
    """관측 범위 [lo, hi]를 min_size 이상으로. 가려진 쪽(또는 센서 반대쪽)으로 확장."""
    if hi - lo >= min_size:
        return lo, hi
    if occluded_lo and not occluded_hi:
        return hi - min_size, hi
    if occluded_hi and not occluded_lo:
        return lo, lo + min_size
    if sensor <= lo:
        return lo, lo + min_size
    if sensor >= hi:
        return hi - min_size, hi
    c = 0.5 * (lo + hi)
    return c - min_size / 2, c + min_size / 2


def detect(ranges, angle_min, angle_inc, lidar_pose, frame, boundary, s_vehicle, p):
    """스캔 -> (클러스터 박스 목록, ROI 점 (N,2) 도로좌표)."""
    idx, r, ok, th, x, y = scan_points(ranges, angle_min, angle_inc, lidar_pose, p)
    s, d = frame.to_road(x, y)
    right, left = boundary.limits(s)
    roi = (ok & (r < p.scan_range_max) & (s > s_vehicle - 0.6) & (s < p.section_end + p.roi_beyond_end)
           & (d > right - p.roi_margin) & (d < left + p.roi_margin))
    sel = np.flatnonzero(roi)
    pts = np.column_stack((s[sel], d[sel]))
    if len(sel) == 0:
        return [], pts
    ls, ld = frame.to_road(lidar_pose[0], lidar_pose[1])
    ls, ld = float(ls), float(ld)
    # 각도 순서 인접 점 간격으로 분할
    jumps = np.hypot(np.diff(pts[:, 0]), np.diff(pts[:, 1])) > p.cluster_jump
    gaps = np.diff(sel) > 3  # ROI 밖 빔이 여러 개 끼면 분리
    cut = np.flatnonzero(jumps | gaps) + 1
    boxes = []
    for part in np.split(np.arange(len(sel)), cut):
        if len(part) < p.cluster_min_points:
            continue
        cs, cd = pts[part, 0], pts[part, 1]
        # 클러스터 양 끝 바로 바깥 빔이 더 가까우면 그쪽이 가려진 것
        i0, i1 = sel[part[0]], sel[part[-1]]
        occ0 = _occluded(r, ok, i0 - 1, r[i0])
        occ1 = _occluded(r, ok, i1 + 1, r[i1])
        # 빔 순서(오른쪽->왼쪽)의 앞쪽 끝이 d 작은 쪽인지 판단
        first_low = cd[0] <= cd[-1]
        occ_lo, occ_hi = (occ0, occ1) if first_low else (occ1, occ0)
        s0, s1 = cs.min(), cs.max()
        d0, d1 = cd.min(), cd.max()
        if p.obstacle_fixed_size:
            if ls <= s0:
                s1 = s0 + p.obstacle_min_length
            elif ls >= s1:
                s0 = s1 - p.obstacle_min_length
            else:
                center = (s0 + s1) / 2
                s0, s1 = center - p.obstacle_min_length / 2, center + p.obstacle_min_length / 2
            d0, d1 = correct_extent(d0, d1, ld, p.obstacle_min_width, occ_lo, occ_hi)
            center = (d0 + d1) / 2
            d0, d1 = center - p.obstacle_min_width / 2, center + p.obstacle_min_width / 2
        elif ls < s0:
            # 앞에서 볼 때만 최소 크기 보정: 뒷부분은 자기 가림 -> 뒤로, 폭은 가려진 쪽으로.
            # 옆·뒤에서는 앞에서 본 누적 박스가 크기를 보장한다(그때 보정하면 박스가 계속 커짐).
            face = cs < s0 + 0.03
            if face.sum() >= 2 and np.ptp(cd[face]) >= 0.08:
                # 앞면이 실제로 보일 때만 뒤로 확장(앞이 가려진 조각이면 앞끝을 모르므로 확장 금지)
                s1 = max(s1, s0 + p.obstacle_min_length)
            d0, d1 = correct_extent(d0, d1, ld, p.obstacle_min_width, occ_lo, occ_hi)
        boxes.append(Box(s0, s1, d0, d1))
    return boxes, pts


def _occluded(r, ok, j, rr):
    if j < 0 or j >= len(r):
        return False
    return bool(ok[j] and r[j] < rr - 0.08)


class Tracker:
    def __init__(self, p):
        self.p = p
        self.tracks = []
        self.next_id = 1
        self.passed = 0
        self.passed_ids = []
        self.version = 0          # 확정 장애물 집합이 바뀔 때 증가

    def update(self, boxes, t, s_rear, ranges=None, beam=None):
        p = self.p
        used = set()
        for b in boxes:
            # 겹치거나 중심이 가까운 트랙만 같은 장애물(0.47 m 간격 연속 차량을 합치지 않도록)
            best, best_d = None, np.inf
            for k, tr in enumerate(self.tracks):
                dc = math.hypot(tr.box.cs - b.cs, tr.box.cd - b.cd)
                if (tr.box.gap_to(b, shrink=0.0) < 0.06 or dc < p.assoc_distance) and dc < best_d:
                    best, best_d = k, dc
            if best is None:
                if b.s1 + p.pass_margin < s_rear:
                    continue  # 이미 지나간 차의 뒷면 재검출: 새 트랙으로 만들지 않음
                self.tracks.append(Track(self.next_id, b, 1, t, t, hist=[b]))
                self.next_id += 1
                continue
            tr = self.tracks[best]
            used.add(tr.id)
            tr.hist = (tr.hist or []) + [b]
            del tr.hist[:-p.track_window]
            # 추정: 최근 track_window회 관측의 합집합(전체 누적은 실차 자세 오차로 부풀었음).
            # 기억: 확정 뒤에는 추정이 track_lock_tol보다 크게 달라질 때만 바꾼다(작은 흔들림으로 경로를 버리지 않음).
            est = tr.hist[0]
            for x in tr.hist[1:]:
                est = est.union(x)
            if est.s1 - est.s0 > p.obstacle_max_length or est.d1 - est.d0 > p.obstacle_max_width:
                est = b
            tr.box = b if p.obstacle_fixed_size else est
            tr.hits += 1
            tr.last_seen = t
            tr.ghost_time = 0.0
            if not tr.confirmed and tr.hits >= p.confirm_hits:
                tr.confirmed = True
                self.version += 1
        keep = []
        for tr in self.tracks:
            if s_rear > tr.box.s1 + p.pass_margin:
                if tr.confirmed and tr.box.s0 < p.section_end:
                    self.passed += 1
                    self.passed_ids.append(tr.id)
                    self.version += 1
                continue
            if not tr.confirmed and t - tr.last_seen > p.tentative_timeout:
                continue
            if tr.confirmed and beam is not None and tr.id not in used and self._see_through(tr, ranges, beam):
                tr.ghost_time += beam['dt']
                if tr.ghost_time > p.ghost_time:
                    self.version += 1
                    continue
            keep.append(tr)
        self.tracks = keep

    def _see_through(self, tr, ranges, beam):
        """박스 중심 방향 빔들이 박스보다 확실히 멀리 반사되면 관통(유령)."""
        x, y = beam['frame'].to_odom(tr.box.cs, tr.box.cd)
        lx, ly, lyaw = beam['lidar_pose']
        dist = math.hypot(x - lx, y - ly)
        if dist > 2.5 or dist < 0.2:
            return False
        a = math.atan2(y - ly, x - lx) - lyaw
        a = (a + math.pi) % (2 * math.pi) - math.pi
        i = int(round((a - beam['angle_min']) / beam['angle_inc']))
        n = len(ranges)
        if i < 3 or i >= n - 3:
            return False
        w = np.asarray(ranges[i - 2:i + 3], float)
        far = (~np.isfinite(w)) | (w <= self.p.scan_range_min) | (w > dist + 0.20)
        return int(far.sum()) >= 4

    def confirmed(self):
        return [tr for tr in self.tracks if tr.confirmed]

    def section_tracks(self):
        """구간 안(갈림길 표지판 등 다음 구간 물체 제외) 확정 장애물."""
        return [tr for tr in self.tracks if tr.confirmed and tr.box.s0 < self.p.section_end]


class VisibilityGrid:
    """도로 좌표 고정 격자. 레이저가 지나간 칸 = 관측된 빈 공간."""

    def __init__(self, p, s_min, s_max, d_half=1.0):
        self.res = p.vis_res
        self.s_min, self.d_min = s_min, -d_half
        self.ns = int(math.ceil((s_max - s_min) / self.res))
        self.nd = int(math.ceil(2 * d_half / self.res))
        self.seen = np.zeros((self.ns, self.nd), bool)
        self.p = p
        self._steps = np.arange(0.06, p.vis_range, self.res * 0.8)

    def update(self, ranges, angle_min, angle_inc, lidar_pose, frame):
        p = self.p
        r = np.asarray(ranges, float)
        n = len(r)
        step = max(1, int(round(math.radians(p.vis_beam_step_deg) / max(angle_inc, 1e-6))))
        i = np.arange(0, n, step)
        rr = r[i]
        ok = np.isfinite(rr) & (rr > p.scan_range_min)
        free = np.where(ok, np.minimum(rr - 0.04, p.vis_range), 0.0)  # 무반사 빔은 미관측
        lx, ly, lyaw = lidar_pose
        th = angle_min + i * angle_inc + lyaw
        t = self._steps
        mask = t[None, :] < free[:, None]
        x = lx + np.cos(th)[:, None] * t[None, :]
        y = ly + np.sin(th)[:, None] * t[None, :]
        s, d = frame.to_road(x[mask], y[mask])
        a = ((s - self.s_min) / self.res).astype(int)
        b = ((d - self.d_min) / self.res).astype(int)
        v = (a >= 0) & (a < self.ns) & (b >= 0) & (b < self.nd)
        self.seen[a[v], b[v]] = True

    def reframed(self, old, new):
        """새 도로 좌표계로 옮긴 복사본(칸 중심을 옛 좌표로 되돌려 최근접 칸 조회)."""
        g = VisibilityGrid.__new__(VisibilityGrid)
        g.__dict__.update(self.__dict__)
        a = (np.arange(self.ns) + 0.5) * self.res + self.s_min
        b = (np.arange(self.nd) + 0.5) * self.res + self.d_min
        S, D = np.meshgrid(a, b, indexing='ij')
        os_, od = old.to_road(*new.to_odom(S, D))
        g.seen = self.is_seen(os_, od)
        return g

    def mark_free(self, s0, s1, d0, d1):
        a0, a1 = self._ia(s0), self._ia(s1)
        b0, b1 = self._ib(d0), self._ib(d1)
        self.seen[max(a0, 0):max(a1 + 1, 0), max(b0, 0):max(b1 + 1, 0)] = True

    def _ia(self, s):
        return int(math.floor((s - self.s_min) / self.res))

    def _ib(self, d):
        return int(math.floor((d - self.d_min) / self.res))

    def is_seen(self, s, d):
        s, d = np.asarray(s), np.asarray(d)
        a = np.floor((s - self.s_min) / self.res).astype(int)
        b = np.floor((d - self.d_min) / self.res).astype(int)
        v = (a >= 0) & (a < self.ns) & (b >= 0) & (b < self.nd)
        out = np.zeros(np.shape(s), bool)
        out[v] = self.seen[a[v], b[v]]
        return out
