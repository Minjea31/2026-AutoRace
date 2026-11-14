"""ROS 독립 미션 코어: 인지 -> 재계획 정책 -> 경로 유지, 안전 정지, DONE 판정.

호출 구조
  on_odom()      : 센서 콜백(50 Hz). 자세 이력 저장.
  process_scan() : 작업 스레드(10 Hz, 최신 스캔만). 인지 + 필요할 때만 계획.
  control()      : 고정 주기 타이머(25 Hz). 저장된 경로를 PD로 추종, 계획 중에도 끊기지 않음.
경로 객체는 불변이고 참조 교체만 하므로 두 스레드가 잠금 없이 읽는다.
"""
import bisect
import math
import threading
import time
from collections import deque

import numpy as np

from .controller import PathTracker
from .lane_boundary import LaneBoundary, YellowMap, fit_road
from .perception import Tracker, VisibilityGrid, detect
from .params import side_radius
from .planner import Path, Planner
from .road import PriorBoundary, RoadFrame

IDLE, RUN, STOP, DONE, REVERSE = 'IDLE', 'RUN', 'STOP', 'DONE', 'REVERSE'
STUCK = ('no_path', 'path_blocked_ahead', 'emergency', 'path_deviation')  # 후진 탈출 대상 정지 이유


class Stats:
    def __init__(self):
        self.v = []

    def add(self, x):
        self.v.append(float(x))

    def summary(self):
        if not self.v:
            return {'n': 0}
        a = np.asarray(self.v)
        return {'n': len(a), 'mean': round(float(a.mean()), 3), 'p95': round(float(np.percentile(a, 95)), 3),
                'max': round(float(a.max()), 3)}


def splice_path(old, s_c, tail):
    """old 경로의 s_c까지 + tail(s_c에서 시작) 이어 붙인 경로."""
    k = int(np.searchsorted(old.s, s_c, side='left'))
    j = 1 if len(tail.s) > 1 and abs(tail.s[0] - (old.s[k - 1] if k > 0 else -1e9)) < 1e-6 else 0

    def cat(a, b):
        if a is None or b is None:
            return None
        return np.concatenate([a[:k], b[j:]])
    return Path(s=cat(old.s, tail.s), d=cat(old.d, tail.d), d1=cat(old.d1, tail.d1), d2=cat(old.d2, tail.d2),
                psi=cat(old.psi, tail.psi), kappa=cat(old.kappa, tail.kappa), clear=cat(old.clear, tail.clear),
                cost=tail.cost, complete=tail.complete, nodes=tail.nodes, stamp=tail.stamp,
                dsteer=cat(old.dsteer, tail.dsteer), bclear=cat(old.bclear, tail.bclear))


class ObjAvoidCore:
    def __init__(self, p, clock=time.perf_counter):
        if p.observe_mode:  # 관찰 모드: 실차 한계로 계획(자동주행은 보수적 한계 유지)
            p.plan_min_radius, p.max_heading = p.observe_min_radius, p.observe_max_heading
            p.plan_min_radius_left = p.plan_min_radius_right = 0.0
        self.p = p
        self.wall = clock
        self.prior = PriorBoundary(p.boundary_s, p.boundary_left, p.boundary_right)
        # 카메라 노란 실선(퓨전 차선 결과)을 쓰면 관측된 곳은 실제 선, 안 본 곳은 사전 경계
        self.boundary = LaneBoundary(self.prior, p) if p.use_lane else self.prior
        self.planner = Planner(p, self.boundary)
        self.section_end0 = p.section_end
        self.swap = threading.Lock()  # 좌표계 재정렬 시 (frame, path, grid) 원자적 교체
        self.ctrl = PathTracker(p)
        self.proj = PathTracker(p)  # 작업 스레드 전용 투영(제어 스레드 상태와 분리)
        self.odom = deque(maxlen=200)
        self.odom_t = []
        self.lock = threading.Lock()
        self.state, self.reason = IDLE, 'not_activated'
        self.frame = None
        self.reset_metrics()

    # ------------------------------------------------------------ 상태
    def reset_metrics(self):
        self.path = None
        self.path_version = -1
        self.last_plan_t = -1e9
        self.last_scan_t = None
        self.emergency_t = -1e9
        self.clear_ahead = False
        self.center_since = None
        self.replans = 0
        self.route_changes = 0
        self.plan_failures = 0
        self.plan_reasons = {}
        self.stop_events = []
        self.plan_ms = Stats()
        self.percep_ms = Stats()
        self.scan_age_ms = Stats()
        self.scan_queue_ms = Stats()
        self.edges = Stats()
        self.ey = Stats()
        self.epsi = Stats()
        self.last_info = {}
        self.last_plan_reason = ''
        self.cmd = (0.0, 0.0)
        self.done_reason = ''
        self.verified_s = None
        self.last_fail = None
        self.fail_checks = {}
        self.lane_frames = 0
        self.lane_lidar_dropped = 0
        self.rev = None          # 후진 중이면 {'t0', 'x0', 'y0', 'sigma', 'target'}
        self.rev_count = 0
        self.rev_anchor_s = None
        self.reverses = 0
        self.stuck_since = None
        self.lane_version = -1
        self.blocked_count = 0
        self.turn_ratio = None   # 실제 회전(IMU 각속도/속도) / 명령 회전(tan δ / L) EMA: 1보다 작으면 덜 돔
        self.cmd_hist = deque(maxlen=50)
        self.lane_ms = Stats()
        self.aligned = False
        self.align_count = 0
        self.last_align_t = -1e9
        self.align_info = None
        self.align_prev = None
        self.island = None

    def set_state(self, state, reason, t):
        if state == STOP and reason != 'first_plan' and (self.state != STOP or self.reason != reason):
            self.stop_events.append((round(t, 2), reason))
        self.state, self.reason = state, reason

    # ------------------------------------------------------------ 입력
    def on_odom(self, t, x, y, yaw, v, w):
        with self.lock:
            if self.odom_t and t <= self.odom_t[-1]:
                if t < self.odom_t[-1] - 0.5:  # 시계 되감김(시뮬 재시작)
                    self.odom.clear()
                    self.odom_t = []
                else:
                    return
            self.odom.append((t, x, y, yaw, v, w))
            self.odom_t.append(t)
            if len(self.odom_t) > self.odom.maxlen:
                self.odom_t = self.odom_t[-self.odom.maxlen:]

    def pose_at(self, t, extrapolate=None):
        """t 시각 odom 자세 (보간, 최신 이후는 제한 시간까지 외삽). 없으면 None."""
        lim = self.p.odom_extrapolate_max if extrapolate is None else extrapolate
        with self.lock:
            if not self.odom:
                return None
            ts = self.odom_t
            k = bisect.bisect_left(ts, t)
            if k == 0:
                if ts[0] - t > 0.05:
                    return None
                return self.odom[0][1:]
            if k >= len(ts):
                t0, x, y, yaw, v, w = self.odom[-1]
                dt = t - t0
                if dt > lim:
                    return None
                yaw2 = yaw + w * dt
                return (x + v * dt * math.cos(yaw + w * dt / 2), y + v * dt * math.sin(yaw + w * dt / 2),
                        yaw2, v, w)
            a, b = self.odom[k - 1], self.odom[k]
            u = (t - a[0]) / max(b[0] - a[0], 1e-9)
            dyaw = (b[3] - a[3] + math.pi) % (2 * math.pi) - math.pi
            return (a[1] + u * (b[1] - a[1]), a[2] + u * (b[2] - a[2]), a[3] + u * dyaw, b[4], b[5])

    def latest_odom_t(self):
        with self.lock:
            return self.odom_t[-1] if self.odom_t else None

    # ------------------------------------------------------------ 활성화
    def activate(self, t):
        pose = self.pose_at(t, extrapolate=1.0)
        if pose is None:
            return False
        x, y, yaw = pose[:3]
        self.frame = RoadFrame(x, y, yaw)
        p = self.p
        self.tracker = Tracker(p)
        self.grid = VisibilityGrid(p, self.boundary.s_min, self.boundary.s_max)
        self.grid.mark_free(-1.0, p.body_front + 0.05, -0.5 * p.body_width - 0.05, 0.5 * p.body_width + 0.05)
        self.ctrl.reset()
        self.reset_metrics()
        self.p.section_end = self.section_end0
        self.ymap = YellowMap(p, x, y) if p.use_lane else None
        self.wmap = YellowMap(p, x, y) if p.use_lane else None  # 흰 점선(넘을 수 있음): 정렬·노란 노이즈 제거용
        if p.use_lane:
            self.boundary.rebuild(np.zeros((0, 2)), p.section_end)
        self.activated_t = t
        self.occ_scan = None
        self.state, self.reason = STOP, 'first_plan'  # 첫 스캔 전 대기(정지 이벤트로 세지 않음)
        return True

    def on_lane(self, stamp, yellow_base, view_base, white_base=None):
        """카메라 노란선 점(차 기준)과 그 프레임의 관측 바닥 영역 -> odom 노란선 격자."""
        if self.ymap is None or self.frame is None or self.state in (IDLE, DONE):
            return False
        pose = self.pose_at(stamp)
        if pose is None:
            return False
        w0 = self.wall()
        yellow_base = self._lidar_ground_filter(yellow_base, pose)
        if white_base is not None:
            white_base = self._lidar_ground_filter(white_base, pose)
        self.ymap.update(yellow_base, view_base, pose[:3])
        if white_base is not None:
            self.wmap.update(white_base, view_base, pose[:3])
        self.lane_frames += 1
        self.lane_ms.add((self.wall() - w0) * 1000)
        return True

    def _lidar_ground_filter(self, pts_base, pose):
        """카메라 바닥 투영점 중 같은 방향 LiDAR 거리보다 먼 점을 버린다. 바닥 테이프는 LiDAR 빔이 위로 지나가므로
        LiDAR 거리가 더 멀고, 서 있는 물체(노란·연두 라바콘)의 픽셀은 바닥으로 투영하면 물체 뒤로 길게 늘어나 LiDAR 거리보다 멀어진다."""
        p = self.p
        sc = getattr(self, 'occ_scan', None)
        if not p.lane_lidar_filter or sc is None or not len(pts_base):
            return pts_base
        ranges, amin, ainc, (lx, ly, lyaw) = sc
        xy = YellowMap._to_odom(pts_base, pose[:3])
        dx, dy = xy[:, 0] - lx, xy[:, 1] - ly
        rng = np.hypot(dx, dy)
        idx = np.round(((np.arctan2(dy, dx) - lyaw - amin + math.pi) % (2 * math.pi) - math.pi) / ainc).astype(int)
        n = len(ranges)
        k = p.lane_lidar_beams
        near = np.full(len(idx), np.inf)
        for o in range(-k, k + 1):
            j = idx + o
            ok = (j >= 0) & (j < n)
            near[ok] = np.minimum(near[ok], ranges[j[ok]])
        keep = rng < near - p.lane_lidar_margin
        self.lane_lidar_dropped += int((~keep).sum())
        return pts_base[keep]

    def _lane_update(self, veh, t):
        """노란선 격자 -> (필요하면 도로 좌표계 재정렬) -> 경계·구간 끝 갱신. 작업 스레드."""
        p = self.p
        if self.ymap is None or (self.ymap.version + self.wmap.version) == self.lane_version:
            return veh
        self.lane_version = self.ymap.version + self.wmap.version
        cells = self.ymap.confirmed()
        wcells = self.wmap.confirmed()
        if len(cells) and len(wcells):
            # 흰 점선 근처(lane_white_exclusion)에 잡힌 노란 칸은 노이즈(점선 오분류·반사)로 버림
            wgrid = (self.wmap.hits >= p.lane_confirm) & (self.wmap.hits >= p.lane_ratio * np.maximum(self.wmap.views, 1))
            r = int(math.ceil(p.lane_white_exclusion / self.wmap.res))
            near = np.zeros_like(wgrid)
            for di in range(-r, r + 1):
                for dj in range(-r, r + 1):
                    near |= np.roll(np.roll(wgrid, di, 0), dj, 1)
            ii = ((cells[:, 0] - self.ymap.x0) / self.ymap.res).astype(int)
            jj = ((cells[:, 1] - self.ymap.y0) / self.ymap.res).astype(int)
            cells = cells[~near[ii, jj]]
        self.white_cells = wcells
        self.lane_cells = cells
        s, d = self.frame.to_road(cells[:, 0], cells[:, 1]) if len(cells) else (np.zeros(0), np.zeros(0))
        sd = np.column_stack((s, d))
        ws, wd = self.frame.to_road(wcells[:, 0], wcells[:, 1]) if len(wcells) else (np.zeros(0), np.zeros(0))
        wsd = np.column_stack((ws, wd))
        # 재정렬은 구간 입구에서만(활성화 후 lane_align_s_max 이내). 갈림길 근처의 섬·분기 선으로 중심이
        # 잘못 보정되는 것을 막는다. 처음 정렬은 위치와 무관하게 시도.
        allow = (not self.aligned) or veh[0] < p.lane_align_s_max
        if p.lane_align and allow and t - self.last_align_t >= p.lane_align_period and len(sd):
            near_end = min(veh[0] + 2.2, p.section_end - 0.1)  # 갈림길 섬 선은 정렬에 쓰지 않음
            if not self.aligned:   # 첫 정렬: 입구 차로 쌍·도로 쌍·한 선+사전 모델 모두 허용
                fit = fit_road(sd, p, veh[0] - 0.5, near_end, self.prior, white_sd=wsd)
            else:                  # 이후: 3차로 양쪽 외곽선 쌍으로만 중심 보정, 한 선이면 방향만
                fit = fit_road(sd, p, veh[0] - 0.5, near_end, None, widths=[p.lane_pair_widths[-1]], white_sd=wsd)
            waited = t - self.activated_t >= p.lane_align_wait
            if fit is not None:
                a, center, _ = fit
                if center is None and (self.aligned or waited):
                    center = 0.0  # 선 쌍·사전 모델로 중심을 못 잡으면 방향만 보정
                dc = 0.0 if center is None else center
                big = abs(a) > p.lane_align_tol or abs(dc) > p.lane_center_tol
                prev, self.align_prev = self.align_prev, (a, dc)
                consistent = (prev is not None and abs(prev[0] - a) < 0.02 and abs(prev[1] - dc) < 0.03)
                if center is not None and ((big and consistent) or not self.aligned):
                    self._reframe(a, dc, t)
                    self.last_align_t = t + (p.lane_realign_period - p.lane_align_period)
                    self.align_prev = None
                    self.align_info = {'t': round(t, 2), 'dpsi': round(a, 3), 'dcenter': round(dc, 3),
                                       'pair': center is not None}
                    s, d = self.frame.to_road(cells[:, 0], cells[:, 1]) if len(cells) else (np.zeros(0), np.zeros(0))
                    sd = np.column_stack((s, d))
                    veh = self.vehicle(t) or veh
                if center is not None:
                    self.aligned = True
        # 갈림길 섬 끝: 가운데 근처(|d|<0.3)에 나타나는 노란선 -> 구간 끝(관측 기반)
        lo, hi = self.section_end0 - p.lane_island_window, self.section_end0 + p.lane_island_window
        isl = sd[(np.abs(sd[:, 1]) < p.lane_island_d) & (sd[:, 0] > max(lo, veh[0] + 0.3)) & (sd[:, 0] < hi)] \
            if len(sd) and self.aligned and p.lane_island else sd[:0]
        if len(isl) >= 3:
            ss = np.sort(isl[:, 0])
            for i in range(len(ss) - 2):
                if ss[i + 2] - ss[i] < 0.15:
                    # 한 번 찾은 섬 끝은 앞당기기만(차가 다가가 앞쪽 칸이 판정 범위에서 빠져도 뒤로 밀리지 않게)
                    self.island = float(ss[i]) if self.island is None else min(self.island, float(ss[i]))
                    p.section_end = float(np.clip(self.island - 0.02, lo, hi))
                    break
        # 구간 끝 너머 노란선과, 입구 이후 가운데 근처(|d| < lane_island_d) 노란선(= 갈림길 섬)은 다음 미션 구간:
        # 이 미션의 좌우 외곽 경계에서는 제외(섬의 선을 외곽선으로 오인하면 차가 경계 밖으로 판정됨)
        if len(sd):
            island = (sd[:, 0] > 1.0) & (np.abs(sd[:, 1]) < p.lane_island_d)
            sd = sd[(sd[:, 0] < p.section_end) & ~island]
        self.boundary.rebuild(sd, p.section_end)
        return veh

    def _lane_blocking(self, t):
        """활성화 직후 lane_align_wait 동안은 노란 실선으로 도로 방향·중심을 잡을 때까지 출발 보류.
        그 안에 못 잡으면(외곽선이 차에 가려짐, 차선 노드 없음) 차 자세 기준으로 진행하고, 입구 1 m 안에서는
        계속 정렬을 시도한다. 관측된 노란선은 그대로 넘을 수 없는 경계로 적용된다."""
        if self.ymap is None or self.aligned:
            return False
        return t - self.activated_t < self.p.lane_align_wait

    def _reframe(self, a, dc, t):
        """도로 좌표계를 노란 실선 방향(a)·중심(dc)에 맞춘다. 추적 박스·관측 격자는 새 좌표로 옮기고 경로는 새로 만든다."""
        old = self.frame
        yaw = old.yaw0 + a
        x0 = old.x0 - dc * math.sin(yaw)
        y0 = old.y0 + dc * math.cos(yaw)
        new = RoadFrame(x0, y0, yaw)
        if abs(a) > self.p.lane_reset_tracks_angle:
            # 회전한 박스를 축정렬 박스로 감싸면 폭이 L*sin(a)만큼 부푼다(0.19 rad -> +6 cm, 좁은 틈을 막음).
            # 큰 재정렬에서는 박스를 버리고 새 좌표계에서 다시 검출(정지 장애물은 두 스캔이면 재확정).
            self.tracker.tracks = []
            self.tracker.version += 1
        for tr in self.tracker.tracks:
            b = tr.box
            cs = np.array([b.s0, b.s1, b.s1, b.s0])
            cd = np.array([b.d0, b.d0, b.d1, b.d1])
            ns, nd = new.to_road(*old.to_odom(cs, cd))
            tr.box = type(b)(float(ns.min()), float(ns.max()), float(nd.min()), float(nd.max()))
        grid = self.grid.reframed(old, new)
        with self.swap:
            self.frame, self.grid, self.path = new, grid, None
        self.align_count += 1
        self.last_plan_t = -1e9

    def deactivate(self):
        self.state, self.reason = IDLE, 'not_activated'
        self.frame = None

    def vehicle(self, t):
        pose = self.pose_at(t)
        if pose is None or self.frame is None:
            return None
        s, d, psi = self.frame.pose_to_road(*pose[:3])
        return s, d, psi, pose[3]

    # ------------------------------------------------------------ 작업 스레드
    def process_scan(self, stamp, ranges, angle_min, angle_inc, t_now):
        if self.frame is None or self.state in (IDLE, DONE):
            return None
        p = self.p
        w0 = self.wall()
        self.scan_age_ms.add((t_now - stamp) * 1000)
        pose = self.pose_at(stamp)
        if pose is None:
            return None
        x, y, yaw = pose[:3]
        c, sn = math.cos(yaw), math.sin(yaw)
        lp = (x + c * p.lidar_x - sn * p.lidar_y, y + sn * p.lidar_x + c * p.lidar_y, yaw + p.lidar_yaw)
        sv, dv, psiv = self.frame.pose_to_road(x, y, yaw)
        # 가장 뒤 모서리(크게 꺾여 있으면 안쪽 뒷모서리가 중심선보다 (폭/2)·|sin ψ| 더 뒤): 지난 장애물을 이보다 먼저 지우지 않음
        s_rear = sv - p.body_rear * abs(math.cos(psiv)) - 0.5 * p.body_width * abs(math.sin(psiv))
        boxes, pts = detect(ranges, angle_min, angle_inc, lp, self.frame, self.boundary, sv, p)
        dt = 0.1 if self.last_scan_t is None else max(0.0, stamp - self.last_scan_t)
        beam = {'frame': self.frame, 'lidar_pose': lp, 'angle_min': angle_min, 'angle_inc': angle_inc, 'dt': dt}
        self.tracker.update(boxes, stamp, s_rear, ranges, beam)
        self.grid.update(ranges, angle_min, angle_inc, lp, self.frame)
        if self._emergency(ranges, angle_min, angle_inc, pose[3]):
            self.emergency_t = stamp
        self.last_scan_t = stamp
        self.last_points = pts
        self.last_boxes = boxes
        r = np.asarray(ranges, float)
        r = np.where(np.isfinite(r) & (r > 0.02), r, np.inf)
        self.occ_scan = (r, angle_min, angle_inc, lp)
        self.percep_ms.add((self.wall() - w0) * 1000)

        veh = self.vehicle(t_now)
        if veh is None:
            return None
        veh = self._lane_update(veh, t_now)
        if self._lane_blocking(t_now):
            return True  # 노란 실선으로 도로 방향·중심을 잡을 때까지 계획 보류(틀린 방향으로 출발하지 않음)
        if self.rev is not None:
            return True  # 후진 중에는 인지만 갱신하고, 멈춘 뒤 새로 계획
        self._maybe_replan(veh, t_now)
        if self.path is not None:
            self.verified_s = self.planner.verified_until(self.path, veh[0], self.grid,
                                                          self.boundary if self.ymap is not None else None)
        self._update_clear_ahead(veh)
        return True

    def _emergency(self, ranges, angle_min, angle_inc, v):
        p = self.p
        r = np.asarray(ranges, float)
        th = angle_min + np.arange(len(r)) * angle_inc + p.lidar_yaw
        ok = np.isfinite(r) & (r > p.scan_range_min)
        x = p.lidar_x + r * np.cos(th)
        y = p.lidar_y + r * np.sin(th)
        reach = p.body_front + p.emergency_distance + max(v, 0.0) * p.emergency_time
        hit = ok & (x > p.body_front - 0.02) & (x < reach) & (np.abs(y) < 0.5 * p.body_width + 0.02)
        return int(hit.sum()) >= p.emergency_points

    def _start_state(self, veh, w=None):
        s, d, psi, v = veh
        if self.path is not None:
            _, _, _, ey, _ = self.proj.errors(self.path, s, d, psi)
            if abs(ey) < self.p.stitch_error and self.path.s[0] <= s <= self.path.end_s - 0.05:
                d0, v0, a0 = self.path.state_at(s)
                return (s, d0, v0, a0), ey
        else:
            ey = 0.0
        v0 = math.tan(psi)
        # 시작 곡률: 실제 운동(odom 각속도/속도). 정지 중이면 마지막 명령 조향(직접 몰던 경우)만, 아니면 0.
        # (이전: 항상 자기 명령 조향 -> 수동 조종·경로 이탈 후 곡률이 -5 1/m 같은 불가능 값이 되어 모든 후보 탈락)
        if w is not None and abs(v) > 0.05:
            k = w / v
        elif self.state == RUN:
            k = math.tan(self.ctrl.delta) / self.p.wheelbase
        else:
            k = 0.0
        rl, rr = side_radius(self.p, 'plan')
        k = max(-1.0 / rr, min(1.0 / rl, k))
        return (s, d, v0, k * (1 + v0 * v0) ** 1.5), ey

    def _maybe_replan(self, veh, t):
        p = self.p
        s = veh[0]
        boxes = [tr.box for tr in self.tracker.confirmed()]
        s_lim = self.planner.plan_limit(boxes) - 0.05
        reason, hard = None, False
        if self.path is None:
            reason, hard = 'no_path', True
        else:
            _, _, _, ey, _ = self.proj.errors(self.path, veh[0], veh[1], veh[2])
            c_obs, c_b = self.planner.path_clearance(self.path, s - p.body_rear, boxes)
            hit = c_obs > p.blocked_tol or c_b > p.blocked_tol  # 계획 당시 여유보다 blocked_tol 넘게 나빠짐
            self.blocked_count = self.blocked_count + 1 if hit else 0
            if self.blocked_count >= p.blocked_scans:  # 박스가 한 스캔 흔들린 것으로 경로를 버리지 않음
                reason, hard = 'blocked', True
            elif abs(ey) > p.deviation_replan:
                reason, hard = 'deviation', True
            elif (self.path.end_s - s < p.replan_remaining and self.path.end_s < s_lim - 0.05
                  and t - self.last_plan_t >= (0.3 if self.path.complete else 0.5)):
                reason = 'extend'
            elif t - self.last_plan_t >= 0.3 and self.tracker.version != self.path_version:
                reason = 'new_obstacle'
            elif not self.path.complete and t - self.last_plan_t >= 0.5:
                reason = 'incomplete'
            elif (self.verified_s is not None and self.verified_s - s < 1.0
                  and self.path.end_s - self.verified_s > 0.1 and t - self.last_plan_t >= p.replan_period):
                reason = 'unverified'
        if reason is None:
            return
        pose = self.pose_at(t)
        start, ey0 = self._start_state(veh, pose[4] if pose is not None else None)
        w0 = self.wall()
        new = None
        old = self.path
        s_c = s + p.commit_len
        if (not hard and p.commit_len > 0 and old is not None and abs(ey0) < p.stitch_error
                and old.s[0] <= s and old.end_s >= s_c + 0.1):
            # 차 앞 commit_len은 기존 경로를 그대로 두고 그 끝 상태에서 새로 계획해 이어 붙임(앞부분이 매번 바뀌지 않게)
            d_c, d1_c, d2_c = old.state_at(s_c)
            tail = self.planner.plan((s_c, d_c, d1_c, d2_c), boxes, self.grid, old, stamp=t)
            if tail is not None:
                new = splice_path(old, s_c, tail)
        if new is None:
            new = self.planner.plan(start, boxes, self.grid, None if hard else self.path,
                                    w_cons=0.0 if hard else None, stamp=t)
        if new is None and (hard or abs(start[3]) > 1.0 or abs(start[2]) > 0.8):
            # 차 자세가 계획 한계 밖(수동 조종으로 급하게 꺾음 등): 시작 곡률 0, 운동학 완화 구간을 늘려 재시도.
            # 지금 진행 방향에서 시작하는 경로를 내되 충돌·경계 검사는 그대로.
            relax0 = p.start_relax
            p.start_relax = p.start_relax_retry
            try:
                new = self.planner.plan((start[0], start[1], start[2], 0.0), boxes, self.grid, None, w_cons=0.0,
                                        stamp=t)
            finally:
                p.start_relax = relax0
            if new is not None:
                self.plan_reasons['relaxed_start'] = self.plan_reasons.get('relaxed_start', 0) + 1
        if new is None or (not new.complete and new.end_s - start[0] < p.escape_use_len):
            # 5차 노드 경로가 없거나 짧음: 사람처럼 꺾고-유지-풀기 조향 후보로 더 멀리 갈 수 있는 쪽
            # (예: 전방이 막히고 오른쪽에 노란선이 보이면 왼쪽 후보만 길게 살아남는다)
            pose = self.pose_at(t)
            delta0 = self.ctrl.delta
            if pose is not None and abs(pose[3]) > 0.05:
                delta0 = math.atan(p.wheelbase * pose[4] / pose[3])
            esc = self.planner.escape(veh[0], veh[1], veh[2], delta0, boxes, self.grid, stamp=t)
            if esc is not None and (new is None or esc.end_s > new.end_s + 0.2):
                new = esc
                self.plan_reasons['escape'] = self.plan_reasons.get('escape', 0) + 1
        self.plan_ms.add((self.wall() - w0) * 1000)
        self.edges.add(self.planner.last.get('edges', 0))
        self.last_plan_t = t
        self.last_plan_reason = reason
        self.path_version = self.tracker.version
        if new is None:
            self.plan_failures += 1
            last = self.planner.last
            self.last_fail = {'t': round(t, 2), 'reason': reason, 'start': [round(v, 3) for v in start],
                              'stations': [round(s_, 2) for s_ in last.get('stations', [])],
                              'nodes': [len(n) for _, n in last.get('layers', [])],
                              'edges': last.get('edges', 0), 'rejected': last.get('rejected', {}),
                              'ego_clear': [round(v, 3) for v in (self.planner.last_ego_clear or (0, 0))],
                              'boxes': [[round(v, 2) for v in (b.s0, b.s1, b.d0, b.d1)] for b in boxes][:8]}
            for k, v in last.get('rejected', {}).items():
                self.fail_checks[k] = self.fail_checks.get(k, 0) + v
            if hard:
                self.path = None
            return
        if self.path is not None:
            ov = (new.s >= s) & (new.s <= min(self.path.end_s, s + 1.5))
            if ov.any():
                diff = np.abs(new.d[ov] - self.path.d_at(new.s[ov]))
                if np.nanmax(diff) > 0.05:
                    self.route_changes += 1
        self.replans += 1
        self.plan_reasons[reason] = self.plan_reasons.get(reason, 0) + 1
        self.path = new

    def _update_clear_ahead(self, veh):
        p = self.p
        s, d = veh[0], veh[1]
        a = s + p.body_front
        b = min(a + p.done_clear_ahead, p.section_end)
        if b <= a + 0.05:
            self.clear_ahead = True
            return
        ss = np.arange(a, b, 0.05)
        wh = 0.5 * p.body_width
        ok = self.grid.is_seen(ss, np.full_like(ss, d))
        ok &= self.grid.is_seen(ss, np.full_like(ss, d + wh)) & self.grid.is_seen(ss, np.full_like(ss, d - wh))
        self.clear_ahead = bool(ok.all())

    # ------------------------------------------------------------ 제어 주기
    def control(self, t, dt):
        """-> None(발행 안 함) 또는 (speed, steering)."""
        p = self.p
        if self.state in (IDLE, DONE) or self.frame is None:
            return None
        to = self.latest_odom_t()
        with self.swap:
            veh = self.vehicle(t)
            path = self.path
        if self.rev is not None:
            if to is None or t - to > p.odom_timeout or veh is None:
                self._end_reverse(t, 'odom_stale')
            else:
                self.cmd = self._reverse_step(t, dt, veh)
                return self.cmd
        if veh is not None and self.rev_anchor_s is not None and veh[0] > self.rev_anchor_s + 0.5:
            self.rev_count, self.rev_anchor_s = 0, None  # 충분히 전진: 다음 막힘에서 다시 후진 허용
        reason = None
        if to is None or t - to > p.odom_timeout or veh is None:
            reason = 'odom_stale'
        elif self.last_scan_t is None or t - self.last_scan_t > p.scan_timeout:
            reason = 'scan_stale'
        elif t - self.emergency_t < 0.3:
            reason = 'emergency'
        elif self._lane_blocking(t):
            reason = 'first_plan'
        elif path is None:
            reason = 'no_path'
        if reason is None:
            s, d, psi, v = veh
            _, _, _, ey, _ = self.ctrl.errors(path, s, d, psi)
            if abs(ey) > p.deviation_stop and not p.observe_mode:
                reason = 'path_deviation'
        if reason is not None:
            if self.reason == 'first_plan' and reason in ('no_path', 'scan_stale') and self.replans == 0 \
                    and t - self.activated_t < 1.0 + p.lane_align_wait:
                reason = 'first_plan'
            self.set_state(STOP, reason, t)
            if self._maybe_start_reverse(t, veh):
                self.cmd = self._reverse_step(t, dt, veh)
                return self.cmd
            v0, d0 = self.ctrl.hold_stop(dt)
            self.cmd = (v0, d0 - p.steer_trim)
            return self.cmd
        if not p.observe_mode and self._check_done(veh, t):
            self.set_state(DONE, self.done_reason, t)
            self.cmd = (0.0, 0.0)
            return None
        v_cmd, delta, info = self.ctrl.step(path, s, d, psi, dt, p.v_max, self.verified_s)
        self._turn_diag(t)
        self.ey.add(abs(info['ey']))
        self.epsi.add(abs(info['epsi']))
        self.last_info = info
        if v_cmd > 0:
            self.set_state(RUN, 'tracking', t)
            self.stuck_since = None
        elif not path.complete:
            self.set_state(STOP, 'path_blocked_ahead', t)  # 안전 경로가 앞에서 끊김 -> 끝에서 정지
            if self._maybe_start_reverse(t, veh):
                self.cmd = self._reverse_step(t, dt, veh)
                return self.cmd
        else:
            self.set_state(STOP, 'unverified_ahead', t)
        self.cmd = (v_cmd, delta - p.steer_trim)  # 제어기 조향(의도한 실제 각) -> 차 쏠림만큼 빼서 명령
        return self.cmd

    # ------------------------------------------------------------ 후진 탈출
    def _maybe_start_reverse(self, t, veh):
        """앞으로 갈 경로가 없어 reverse_wait 넘게 서 있으면 후진 시작. 반환: 시작했는지."""
        p = self.p
        if self.reason not in STUCK:
            self.stuck_since = None
            return False
        if self.stuck_since is None:
            self.stuck_since = t
        if (not p.reverse_enable or p.observe_mode or veh is None or self.rev_count >= p.reverse_max
                or t - self.stuck_since < p.reverse_wait or abs(veh[3]) > 0.03):
            return False
        if self.last_scan_t is None or t - self.last_scan_t > p.scan_timeout:
            return False
        s, d = veh[0], veh[1]
        boxes = [tr.box for tr in self.tracker.confirmed()]
        half = 0.5 * p.body_width + p.margin_obstacle
        best = None
        for c in p.lane_centers:  # 앞쪽이 가장 멀리 비어 있는 차로(같으면 가까운 차로)
            ahead = [b.s0 for b in boxes if b.s1 > s - p.body_rear and b.d1 > c - half and b.d0 < c + half]
            free = (min(ahead) - s) if ahead else np.inf
            key = (min(free, 3.0), -abs(c - d))
            if best is None or key > best[0]:
                best = (key, c)
        target = best[1]
        sigma = 0 if abs(target - d) < 0.1 else (1 if target > d else -1)
        pose = self.pose_at(t)
        if pose is None:
            return False
        self.rev = {'t0': t, 'x0': pose[0], 'y0': pose[1], 'sigma': sigma, 'target': target}
        self.rev_count += 1
        self.reverses += 1
        self.rev_anchor_s = s if self.rev_anchor_s is None else min(self.rev_anchor_s, s)
        with self.swap:
            self.path = None
        side = {1: 'left', -1: 'right', 0: 'straight'}[sigma]
        self.set_state(REVERSE, f'reverse_{side}', t)
        return True

    def _reverse_clear(self, s, d, psi, boxes):
        """차체 네 모서리의 (추적 박스 여유, 노란 경계 여유)(m, 음수면 겹침·넘음)."""
        p = self.p
        c, sn = math.cos(psi), math.sin(psi)
        wh = 0.5 * p.body_width
        cs = np.array([s + ex * c - ey * sn for ex in (p.body_front, -p.body_rear) for ey in (wh, -wh)])
        cd = np.array([d + ex * sn + ey * c for ex in (p.body_front, -p.body_rear) for ey in (wh, -wh)])
        lo, hi = self.boundary.limits(cs)
        rg, lf = cd - np.asarray(lo), np.asarray(hi) - cd
        if hasattr(self.boundary, 'known'):  # 카메라로 본 노란선만(뒤쪽 미관측 구간의 사전 경계로 막지 않음)
            rg = np.where(self.boundary.known(cs, -1), rg, np.inf)
            lf = np.where(self.boundary.known(cs, 1), lf, np.inf)
        c_bound = float(min(rg.min(), lf.min()))
        c_box = np.inf
        for bx in boxes:
            gap = np.maximum(np.maximum(bx.s0 - cs, cs - bx.s1), np.maximum(bx.d0 - cd, cd - bx.d1))
            c_box = min(c_box, float(gap.min()))
        return c_box, c_bound

    def _reverse_step(self, t, dt, veh):
        p = self.p
        r = self.rev
        s, d, psi, v = veh
        if r['sigma'] != 0 and r['sigma'] * psi < p.reverse_psi:
            want = -r['sigma'] * p.max_steering  # 후진에서 이 조향이면 앞머리가 sigma 쪽으로 돈다
        elif r['sigma'] != 0:
            want = 0.0  # 앞머리가 이미 충분히 돌았음: 그 방향 그대로 물러나 앞 공간만 확보
        else:
            want = max(-p.max_steering, min(p.max_steering, 2.0 * psi))  # 똑바로 물러나며 방향 복원
        pose = self.pose_at(t)
        moved = math.hypot(pose[0] - r['x0'], pose[1] - r['y0']) if pose is not None else 0.0
        boxes = [tr.box for tr in self.tracker.confirmed()]
        c_now = self._reverse_clear(s, d, psi, boxes)
        target, end = None, None
        if moved >= p.reverse_dist:
            end = 'reversed'
        elif t - r['t0'] > p.reverse_timeout:
            end = 'reverse_timeout'
        else:
            # 조향 후보(원하는 쪽 -> 똑바로)마다 0.08 m 후진한 자세를 미리 검사: 여유가 reverse_margin 이상이거나
            # 지금보다 나빠지지 않는 첫 후보. 노란선·콘에 붙어 있으면 앞머리 돌리기를 포기하고 똑바로 물러난다.
            for cand in (want, 0.0):
                ds = 0.08
                psi_n = psi - ds * math.tan(cand) / p.wheelbase
                mid = 0.5 * (psi + psi_n)
                ob, bd = self._reverse_clear(s - ds * math.cos(mid), d - ds * math.sin(mid), psi_n, boxes)
                # 콘·차량은 reverse_margin, 노란선(테이프)은 넘지만 않으면 됨. 이미 붙어 있으면 나빠지지만 않게.
                if (ob >= p.reverse_margin or ob >= c_now[0] - 0.003) and (bd >= 0.0 or bd >= c_now[1] - 0.003):
                    target = cand
                    break
            if target is None:
                end = 'reverse_blocked'
        if end is not None:
            self._end_reverse(t, end)
            return (0.0, self.ctrl.delta - p.steer_trim)
        step = p.max_steering_rate * max(1e-3, min(dt, 0.2))
        self.ctrl.delta += max(-step, min(step, target - self.ctrl.delta))
        # 바퀴를 먼저 꺾고 움직인다(같은 후진 거리에서 방향 변화 최대)
        speed = -p.reverse_speed if abs(target - self.ctrl.delta) < 0.1 else 0.0
        return (speed, self.ctrl.delta - p.steer_trim)

    def _end_reverse(self, t, why):
        self.rev = None
        self.ctrl.v = 0.0
        self.blocked_count = 0
        self.stuck_since = t  # 다시 막히면 reverse_wait 뒤 다음 후진
        with self.swap:
            self.path = None
        self.last_plan_t = -1e9
        self.set_state(STOP, why, t)

    def _turn_diag(self, t):
        """서보 지연(약 0.25 s) 전 명령과 지금 실제 회전을 비교한 회전 비율(진단·steer_gain 튜닝용)."""
        p = self.p
        self.cmd_hist.append((t, self.ctrl.delta))
        pose = self.pose_at(t)
        old = [d_ for t_, d_ in self.cmd_hist if t - 0.3 <= t_ <= t - 0.2]
        if pose is None or not old or abs(pose[3]) < 0.08:
            return
        dc = float(np.mean(old))
        if abs(dc) < 0.12:
            return
        k_cmd = math.tan(dc) / p.wheelbase
        k_meas = pose[4] / pose[3]
        r = k_meas / k_cmd
        if 0.0 < r < 3.0:
            self.turn_ratio = r if self.turn_ratio is None else 0.95 * self.turn_ratio + 0.05 * r

    def _check_done(self, veh, t):
        p = self.p
        s, d, psi, _ = veh
        if self.tracker.passed < p.done_min_passed or self.tracker.section_tracks() or not self.clear_ahead:
            self.center_since = None
            return False
        if s + p.body_front >= p.section_end - 0.1:
            self.done_reason = 'section_end'
            return True
        if abs(d - p.home_d) <= p.done_center_tol and abs(psi) <= p.done_heading_tol:
            if self.center_since is None:
                self.center_since = t
            if t - self.center_since >= p.done_hold:
                self.done_reason = 'centered'
                return True
        else:
            self.center_since = None
        return False

    # ------------------------------------------------------------ 보고
    def status(self):
        tr = getattr(self, 'tracker', None)
        out = {'state': self.state, 'reason': self.reason, 'replans': self.replans, 'reverses': self.reverses,
               'route_changes': self.route_changes, 'plan_failures': self.plan_failures,
               'plan_reasons': self.plan_reasons, 'last_plan_reason': self.last_plan_reason,
               'stop_events': self.stop_events[-20:], 'n_stop_events': len(self.stop_events),
               'plan_ms': self.plan_ms.summary(), 'percep_ms': self.percep_ms.summary(),
               'scan_age_ms': self.scan_age_ms.summary(), 'scan_queue_ms': self.scan_queue_ms.summary(), 'edges': self.edges.summary(),
               'ey': self.ey.summary(), 'epsi': self.epsi.summary(),
               'cmd': [round(self.cmd[0], 3), round(self.cmd[1], 4)], 'done_reason': self.done_reason,
               'last_fail': self.last_fail, 'fail_checks': self.fail_checks,
               'turn_ratio': None if self.turn_ratio is None else round(self.turn_ratio, 3),
               'lane': {'frames': self.lane_frames, 'lidar_dropped': self.lane_lidar_dropped, 'ms': self.lane_ms.summary(), 'aligned': self.aligned,
                        'aligns': self.align_count, 'last_align': self.align_info, 'island_s': self.island,
                        'section_end': round(self.p.section_end, 3)}}
        if tr is not None:
            out['tracks'] = len(tr.confirmed())
            out['passed'] = tr.passed
        if self.last_info:
            out['ey_now'] = round(self.last_info['ey'], 4)
            out['epsi_now'] = round(self.last_info['epsi'], 4)
        return out
