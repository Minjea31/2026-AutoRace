"""카메라 노란 실선 경계(퓨전 차선 인식 결과 재사용).

입력: 차 기준(base_link: x 전방, y 왼쪽) 노란선 점과, 그 프레임에서 카메라가 실제로 본 바닥 영역.
  - 노란 점: /lane_fusion/deep_mask 노란 클래스 ∧ 같은 프레임 원본의 HSV 노란색(퓨전 lane.yaml 임계값)
  - 바닥 투영: 퓨전 lane.yaml의 BEV 보정값(이미지 사다리꼴 -> 미터 직사각형)
누적: odom 고정 격자(관측 횟수/노란 횟수). 차가 움직이거나 선이 잠깐 안 보여도 기억을 쓴다.
출력:
  - limits(s): 도로 좌표 노란 경계(관측된 곳은 실제 선, 안 본 곳은 사전 경계) + 관측 여부
  - align(): 직선 노란선 방향·위치로 도로 좌표계(방향, 중심) 추정
  - island_s(): 가운데 근처에 나타나는 노란선(갈림길 섬 끝) = 구간 끝
"""
import math

import numpy as np


def _perspective(src, dst):
    """4점 원근 변환 3x3 (cv2.getPerspectiveTransform과 같은 식)."""
    A, b = [], []
    for (x, y), (u, v) in zip(src, dst):
        A.append([x, y, 1, 0, 0, 0, -u * x, -u * y])
        A.append([0, 0, 0, x, y, 1, -v * x, -v * y])
        b += [u, v]
    h = np.linalg.solve(np.asarray(A, float), np.asarray(b, float))
    return np.append(h, 1.0).reshape(3, 3)


class GroundProjection:
    """퓨전 lane.yaml(lane_lite 섹션)의 BEV 정의를 그대로 써서 이미지 픽셀 -> base_link 바닥 좌표."""

    def __init__(self, cfg, max_range=2.5, max_lateral=1.5, homography=None):
        c = cfg
        w, h = int(c['width']), int(c['height'])
        self.w, self.h = w, h
        src = [(c['bev_top_left'] * (w - 1), c['bev_top_y'] * (h - 1)),
               (c['bev_top_right'] * (w - 1), c['bev_top_y'] * (h - 1)),
               (c['bev_bottom_right'] * (w - 1), c['bev_bottom_y'] * (h - 1)),
               (c['bev_bottom_left'] * (w - 1), c['bev_bottom_y'] * (h - 1))]
        dst = [(0, 0), (w - 1, 0), (w - 1, h - 1), (0, h - 1)]
        H = _perspective(src, dst)
        mpx = c['lane_width_m'] / c['lane_width_px']
        mrow = (c['bev_far_m'] - c['bev_near_m']) / (h - 1)
        cu = c.get('vehicle_center_u', 0.5) * (w - 1)
        M = np.array([[0, -mrow, c['bev_near_m'] + (h - 1) * mrow], [-mpx, 0, cu * mpx], [0, 0, 1]])
        self.G = M @ H                      # 이미지 px -> (전방, 왼쪽) m
        if homography is not None:          # 차별 보정값(예: 04번 bag 보정)이 있으면 그것을 쓴다
            self.G = np.asarray(homography, float).reshape(3, 3)
        self.Ginv = np.linalg.inv(self.G)
        self.near, self.far = c['bev_near_m'], c['bev_far_m']
        self.roi = (c.get('roi_left', 0.0), c.get('roi_right', 1.0), c.get('roi_top', 0.0), c.get('roi_bottom', 1.0))
        self.hsv = (c.get('yellow_h_min', 15), c.get('yellow_h_max', 40), c.get('yellow_s_min', 80),
                    c.get('yellow_v_min', 80))
        self.white_hsv = (c.get('white_s_max', 70), c.get('white_v_min', 120))  # 흰 점선: 채도 낮고 밝음
        # 같은 바닥 투영식은 BEV 직사각형 밖 픽셀에도 성립 -> 영상 ROI 전체(거리 max_range까지)를 관측 영역으로.
        # (BEV 직사각형만 쓰면 좌우 ±0.73 m라 3차로 외곽선 ±0.77 m가 시작 위치에서 안 보였다)
        self.max_range, self.max_lateral = max_range, max_lateral
        xs = np.arange(0.2, max_range, 0.03)
        ys = np.arange(-max_lateral, max_lateral, 0.03)
        X, Y = np.meshgrid(xs, ys)
        pts = np.column_stack((X.ravel(), Y.ravel()))
        u, v = self.to_image(pts)
        l, r, t, b = self.roi
        ok = (u >= l * (w - 1)) & (u <= r * (w - 1)) & (v >= t * (h - 1)) & (v <= b * (h - 1))
        self.scale = np.array([1.0, 1.0])  # 차별 바닥 거리 보정(전방, 좌우) — set_scale()
        self.view_points = pts[ok]
        self.near = float(self.view_points[:, 0].min())

    def set_scale(self, sx, sy):
        """투영 결과에 곱할 보정 계수(예: 04번 좌우 0.877 — 실측 차로 0.50 m를 카메라가 0.57 m로 잼)."""
        self.view_points = self.view_points / self.scale * np.array([sx, sy])
        self.scale = np.array([sx, sy], float)

    def to_ground(self, u, v):
        p = self.G @ np.vstack((u, v, np.ones_like(u)))
        return np.column_stack((p[0] / p[2], p[1] / p[2]))

    def to_image(self, pts):
        p = self.Ginv @ np.vstack((pts[:, 0], pts[:, 1], np.ones(len(pts))))
        return p[0] / p[2], p[1] / p[2]

    def white_points(self, mask, bgr=None, white_class=2, step=2):
        """마스크 흰 점선 클래스(∧ 원본 HSV 흰색) -> base_link 바닥 점."""
        return self.yellow_points(mask, bgr, white_class, step, white=True)

    def yellow_points(self, mask, bgr=None, yellow_class=1, step=2, white=False):
        """마스크 노란 클래스(∧ 원본 HSV 노란색) 픽셀 -> base_link 바닥 점. 마스크·영상 크기가 달라도 비례 변환."""
        import cv2
        mh, mw = mask.shape[:2]
        sel = mask[::step, ::step] == yellow_class
        if bgr is not None:
            # 원본을 표본 격자 크기로 맞춰 HSV 확인(축소 디코딩 320x240 + 마스크 2칸 간격이면 그대로)
            img = bgr if bgr.shape[:2] == sel.shape else cv2.resize(bgr, (sel.shape[1], sel.shape[0]),
                                                                    interpolation=cv2.INTER_NEAREST)
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            if white:
                s1, v0 = self.white_hsv
                sel &= (hsv[..., 1] <= s1) & (hsv[..., 2] >= v0)
            else:
                h0, h1, s0, v0 = self.hsv
                sel &= (hsv[..., 0] >= h0) & (hsv[..., 0] <= h1) & (hsv[..., 1] >= s0) & (hsv[..., 2] >= v0)
        vv, uu = np.nonzero(sel)
        u = uu * step * (self.w / mw)
        v = vv * step * (self.h / mh)
        l, r, t, b = self.roi
        ok = (u >= l * (self.w - 1)) & (u <= r * (self.w - 1)) & (v >= t * (self.h - 1)) & (v <= b * (self.h - 1))
        g = self.to_ground(u[ok].astype(float), v[ok].astype(float)) * self.scale
        keep = (g[:, 0] >= self.near - 0.02) & (g[:, 0] <= self.max_range) & (np.abs(g[:, 1]) <= self.max_lateral)
        return g[keep]


class YellowMap:
    """odom 고정 노란선 격자. 칸마다 (카메라가 본 횟수, 노란색이었던 횟수)."""

    def __init__(self, p, x0, y0, half=7.0):
        self.p = p
        self.res = p.lane_cell
        self.x0, self.y0 = x0 - half, y0 - half
        self.n = int(2 * half / self.res)
        self.views = np.zeros((self.n, self.n), np.int16)
        self.hits = np.zeros((self.n, self.n), np.int16)
        self.frames = 0
        self.version = 0

    def _idx(self, xy):
        i = ((xy[:, 0] - self.x0) / self.res).astype(int)
        j = ((xy[:, 1] - self.y0) / self.res).astype(int)
        ok = (i >= 0) & (i < self.n) & (j >= 0) & (j < self.n)
        return i[ok], j[ok]

    @staticmethod
    def _to_odom(pts, pose):
        x, y, yaw = pose
        c, s = math.cos(yaw), math.sin(yaw)
        return np.column_stack((x + c * pts[:, 0] - s * pts[:, 1], y + s * pts[:, 0] + c * pts[:, 1]))

    def update(self, yellow_base, view_base, pose, occluders=()):
        """occluders: 도로 좌표가 아닌 odom AABB 대신, 차 기준 (전방, 왼쪽) 다각형 목록(차량 박스) — 가려진 칸은 관측 제외."""
        vw = self._to_odom(view_base, pose)
        i, j = self._idx(vw)
        if occluders:
            keep = np.ones(len(view_base), bool)
            for poly in occluders:
                keep &= ~_shadow(view_base, poly)
            i, j = self._idx(vw[keep])
        cells = np.unique(i * self.n + j)
        np.add.at(self.views.ravel(), cells, 1)
        if len(yellow_base):
            yi, yj = self._idx(self._to_odom(yellow_base, pose))
            hc = np.unique(yi * self.n + yj)
            np.add.at(self.hits.ravel(), hc, 1)
        self.frames += 1
        self.version += 1

    def confirmed(self):
        p = self.p
        ok = (self.hits >= p.lane_confirm) & (self.hits >= p.lane_ratio * np.maximum(self.views, 1))
        i, j = np.nonzero(ok)
        return np.column_stack((self.x0 + (i + 0.5) * self.res, self.y0 + (j + 0.5) * self.res))

    def seen(self, xy):
        i = ((xy[:, 0] - self.x0) / self.res).astype(int)
        j = ((xy[:, 1] - self.y0) / self.res).astype(int)
        ok = (i >= 0) & (i < self.n) & (j >= 0) & (j < self.n)
        out = np.zeros(len(xy), bool)
        out[ok] = self.views[i[ok], j[ok]] >= self.p.lane_confirm
        return out


def _shadow(pts, poly):
    """카메라 원점에서 볼 때 다각형(차 기준 점들) 뒤에 숨는 점."""
    ang = np.arctan2(poly[:, 1], poly[:, 0])
    near = np.min(np.hypot(poly[:, 0], poly[:, 1]))
    a = np.arctan2(pts[:, 1], pts[:, 0])
    return (a >= ang.min()) & (a <= ang.max()) & (np.hypot(pts[:, 0], pts[:, 1]) > near)


class LaneBoundary:
    """사전 경계(PriorBoundary)와 같은 인터페이스. 관측된 노란선이 있는 s 구간은 실제 선을 쓴다."""

    def __init__(self, prior, p):
        self.prior = prior
        self.p = p
        self.s_min, self.s_max = prior.s_min, prior.s_max
        self.bin = 0.05
        self.s_bins = np.arange(self.s_min, self.s_max + 1e-9, self.bin)
        r, l = prior.limits(self.s_bins)
        self.right, self.left = r.copy(), l.copy()
        self.known_r = np.zeros(len(self.s_bins), bool)
        self.known_l = np.zeros(len(self.s_bins), bool)
        self.cells_sd = np.zeros((0, 2))
        self.island = None

    def limits(self, s):
        s = np.asarray(s)
        return np.interp(s, self.s_bins, self.right), np.interp(s, self.s_bins, self.left)

    def known(self, s, side):
        k = self.known_l if side > 0 else self.known_r
        i = np.clip(np.round((np.asarray(s) - self.s_min) / self.bin).astype(int), 0, len(self.s_bins) - 1)
        return k[i]

    def rebuild(self, cells_sd, s_section_end):
        """확정 노란 칸(도로 좌표) -> s 칸별 좌/우 경계. 관측된 칸은 실제 선(선 반폭만큼 안쪽)."""
        p = self.p
        r0, l0 = self.prior.limits(self.s_bins)
        self.right, self.left = r0.copy(), l0.copy()
        self.known_r[:] = False
        self.known_l[:] = False
        self.cells_sd = cells_sd
        if len(cells_sd) == 0:
            return
        s, d = cells_sd[:, 0], cells_sd[:, 1]
        k = np.round((s - self.s_min) / self.bin).astype(int)
        ok = (k >= 0) & (k < len(self.s_bins)) & (np.abs(d) < p.lane_max_d)
        k, d = k[ok], d[ok]
        half = p.lane_line_half
        # 가운데(d=0) 기준 왼쪽에서 가장 안쪽 선 / 오른쪽에서 가장 안쪽 선
        left = d > p.lane_center_gap
        right = d < -p.lane_center_gap
        lv = np.full(len(self.s_bins), np.inf)
        rv = np.full(len(self.s_bins), -np.inf)
        np.minimum.at(lv, k[left], d[left] - half)
        np.maximum.at(rv, k[right], d[right] + half)
        # 관측된 곳은 실제 선, 안 본 곳은 사전 경계. 실제 선이 사전보다 바깥이면(사전 오차) 실제 선을 따른다.
        self.known_l = np.isfinite(lv)
        self.known_r = np.isfinite(rv)
        self.left[self.known_l] = lv[self.known_l]
        self.right[self.known_r] = rv[self.known_r]
        # 한쪽만 보이는 칸: 반대쪽은 사전 경계 유지(임의로 넓히지 않음)


def line_segments(points_sd, res, max_lines=5, tol=0.02, iters=150, seed=0, max_gap=0.10):
    """노란 칸(도로 좌표) -> RANSAC 직선 조각 [(각, 길이, 법선 위치, 중앙 s, 두께, 칸 수)].
    꺾인 선(입구 깔때기 대각선 + 외곽 직선)도 직선 조각으로 나눈다. 조각은 축 방향으로 10 cm 넘게 끊기면 분리."""
    pts = np.asarray(points_sd, float)
    rng = np.random.default_rng(seed)
    out = []
    for _ in range(max_lines):
        if len(pts) < 8:
            break
        idx = rng.integers(0, len(pts), (iters, 2))
        a, b = pts[idx[:, 0]], pts[idx[:, 1]]
        dv = b - a
        L = np.hypot(dv[:, 0], dv[:, 1])
        ok = L > 0.1
        if not ok.any():
            break
        a, dv, L = a[ok], dv[ok], L[ok]
        nrm = np.column_stack((-dv[:, 1], dv[:, 0])) / L[:, None]
        dist = np.abs((pts[None, :, :] - a[:, None, :]) @ nrm[:, :, None])[..., 0]
        cnt = (dist < tol).sum(1)
        k = int(cnt.argmax())
        inl = dist[k] < tol
        q = pts[inl]
        m = q.mean(0)
        _, _, vt = np.linalg.svd(q - m, full_matrices=False)
        dirv = vt[0] if vt[0][0] >= 0 else -vt[0]
        proj = (q - m) @ dirv
        order = np.argsort(proj)
        gaps = np.flatnonzero(np.diff(proj[order]) > max_gap)
        runs = np.split(order, gaps + 1)
        run = max(runs, key=len)  # 가장 긴 연속 구간만 이 조각으로
        r = q[run]
        rm = r.mean(0)
        pr = (r - rm) @ dirv
        ang = math.atan2(dirv[1], dirv[0])
        thick = float(np.std((r - rm) @ np.array([-dirv[1], dirv[0]])))
        nr = -rm[0] * math.sin(ang) + rm[1] * math.cos(ang)
        out.append((ang, float(pr.max() - pr.min()), nr, float(rm[0]), thick, len(r)))
        keep = np.ones(len(pts), bool)
        keep[np.flatnonzero(inl)[run]] = False
        pts = pts[keep]
    return out


def fit_road(points_sd, p, s_from, s_to, prior=None, widths=None, white_sd=None):
    """노란·흰 점선 칸(도로 좌표) -> (방향 보정각 a, 중심 보정 dc 또는 None, 선 위치들) 또는 None.

    방향: 길고 얇은 직선 조각(노란 실선, 흰 점선은 25 cm 틈까지 이음) 중 지금 도로 축과 ±lane_align_max 안의
          것들을 평행 묶음으로 나눠, 총 길이가 가장 긴 묶음의 길이 가중 평균(입구 대각선·곡선·노이즈 배제).
    중심: 같은 종류 두 선의 짝 — 흰-흰(차로 폭 0.52), 노-노(도로 폭 1.537/입구 0.52) — 의 중점.
          짝이 없으면 가장 긴 흰 점선을 ±lane_dash_offset, 노란선을 사전 모델(prior)과 비교(±lane_single_tol).
    """
    def pick(pts, kind, gap):
        if pts is None or len(pts) == 0:
            return []
        sel = (pts[:, 0] > s_from) & (pts[:, 0] < s_to) & (np.abs(pts[:, 1]) < p.lane_max_d)
        return [g + (kind,) for g in line_segments(pts[sel], p.lane_cell, max_gap=gap)
                if g[1] >= p.lane_seg_min_len and g[4] <= p.lane_seg_max_thick and abs(g[0]) <= p.lane_align_max]
    segs = pick(points_sd, 'y', 0.10) + pick(white_sd, 'w', p.lane_white_gap)
    groups = []
    for g in sorted(segs, key=lambda q: -q[1]):
        for grp in groups:
            if abs(g[0] - grp[0][0]) < 0.06:
                grp.append(g)
                break
        else:
            groups.append([g])
    if not groups:
        return None
    segs = max(groups, key=lambda grp: sum(q[1] for q in grp))
    total = sum(g[1] for g in segs)
    if total < p.lane_align_min_len:
        return None
    a = sum(g[0] * g[1] for g in segs) / total
    lines = []
    for g in segs:
        ang, L, nrm, sm, kind = g[0], g[1], g[2], g[3], g[6]
        dm = (nrm + sm * math.sin(ang)) / math.cos(ang)   # 조각 중심의 d
        lines.append((-sm * math.sin(a) + dm * math.cos(a), sm, L, kind))
    center = None
    pair_w = {'w': [2 * p.lane_dash_offset], 'y': widths or p.lane_pair_widths}
    for i in range(len(lines)):
        for j in range(i + 1, len(lines)):
            if lines[i][3] != lines[j][3]:
                continue
            w = abs(lines[j][0] - lines[i][0])
            if any(abs(w - ww) < p.lane_width_tol for ww in pair_w[lines[i][3]]):
                c = 0.5 * (lines[i][0] + lines[j][0])
                if abs(c) <= p.lane_center_max and (center is None or abs(c) < abs(center)):
                    center = c
    if center is None and lines:
        whites = [q for q in lines if q[3] == 'w']
        if whites:
            c = max(whites, key=lambda q: q[2])[0]
            dc = min((c - p.lane_dash_offset, c + p.lane_dash_offset), key=abs)
            if abs(dc) <= p.lane_single_tol:
                center = dc
        elif prior is not None:
            c, sm, _, _ = max(lines, key=lambda q: q[2])
            r, l = prior.limits(np.array([sm]))
            cand = [c - (float(l[0]) + p.lane_line_half), c - (float(r[0]) - p.lane_line_half)]
            dc = min(cand, key=abs)
            if abs(dc) <= p.lane_single_tol:
                center = dc
    return a, center, [q[0] for q in lines]
