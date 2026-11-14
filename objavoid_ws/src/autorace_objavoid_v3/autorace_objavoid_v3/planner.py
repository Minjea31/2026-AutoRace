"""Free-space/Gap 탐색 -> 통로 중심 Lattice(5차 다항식) -> 차체 충돌·운동학 검사 -> DP 선택.

층(station)은 전방 장애물 중심 s(차량이 그 옆을 지나는 위치)와 지평 끝에만 둔다.
각 층의 노드는 그 위치에서 차체가 들어갈 수 있는 빈 구간(Gap)의 중심·차로 중심·
구간 양 끝(좁은 통로)·기존 경로 위치다. 층 사이를 d(s) 5차 다항식으로 연결하고
(끝점 d'=d''=0, 시작점은 현재 경로 상태와 연속), 모든 샘플에서 회전 차체 직사각형으로
장애물·경계 충돌을 검사한다. 층 단위로 벡터화해 한 번의 numpy 연산으로 처리한다.
"""
import math
from dataclasses import dataclass, field

import numpy as np

from .params import side_radius


@dataclass
class Path:
    s: np.ndarray
    d: np.ndarray
    d1: np.ndarray
    d2: np.ndarray
    psi: np.ndarray
    kappa: np.ndarray
    clear: np.ndarray         # 샘플별 장애물 최소 간격(차체 기준)
    cost: float
    complete: bool            # 지평 끝까지 도달했는가(아니면 끝에서 정지)
    nodes: list = field(default_factory=list)
    stamp: float = 0.0
    dsteer: np.ndarray = None  # 샘플별 |dδ/ds| (rad/m): 제어기 속도 제한용
    bclear: np.ndarray = None  # 샘플별 계획 당시 노란 경계 여유

    @property
    def end_s(self):
        return float(self.s[-1])

    def state_at(self, s):
        s = min(max(s, self.s[0]), self.s[-1])
        return (float(np.interp(s, self.s, self.d)), float(np.interp(s, self.s, self.d1)),
                float(np.interp(s, self.s, self.d2)))

    def d_at(self, s):
        return np.interp(s, self.s, self.d, left=np.nan, right=np.nan)


def obstacle_array(boxes):
    if not boxes:
        return np.zeros((0, 4))
    return np.array([[(b.s0 + b.s1) / 2, (b.d0 + b.d1) / 2, (b.s1 - b.s0) / 2, (b.d1 - b.d0) / 2]
                     for b in boxes])


def body_clearance(s, d, psi, obs, p):
    """회전 차체 직사각형과 장애물 AABB 사이 분리축(SAT) 간격. 음수 = 겹침.

    분리축 최대값은 실제 거리의 하한(모서리에서 과소평가)이므로 안전 쪽이다.
    """
    if len(obs) == 0:
        return np.full(np.shape(s), np.inf)
    c, sn = np.cos(psi)[..., None], np.sin(psi)[..., None]
    lh = 0.5 * (p.body_front + p.body_rear)
    off = 0.5 * (p.body_front - p.body_rear)
    wh = 0.5 * p.body_width
    bx = s[..., None] + off * c
    by = d[..., None] + off * sn
    cs, cd, hs, hd = obs[:, 0], obs[:, 1], obs[:, 2], obs[:, 3]
    dx, dy = cs - bx, cd - by
    ac, asn = np.abs(c), np.abs(sn)
    sep = np.maximum.reduce([
        np.abs(dx) - (hs + lh * ac + wh * asn),
        np.abs(dy) - (hd + lh * asn + wh * ac),
        np.abs(dx * c + dy * sn) - (lh + hs * ac + hd * asn),
        np.abs(-dx * sn + dy * c) - (wh + hs * asn + hd * ac),
    ])
    return sep.min(axis=-1)


def boundary_clearance(s, d, psi, boundary, p):
    c, sn = np.cos(psi), np.sin(psi)
    wh = 0.5 * p.body_width
    out = None
    for ex in (p.body_front, -p.body_rear):
        for ey in (wh, -wh):
            cs = s + ex * c - ey * sn
            cd = d + ex * sn + ey * c
            right, left = boundary.limits(cs)
            m = np.minimum(cd - right, left - cd)
            out = m if out is None else np.minimum(out, m)
    return out


def quintic(d0, v0, a0, d1, T, tau):
    """d(s) 5차: 시작 (d0, d0', d0''), 끝 (d1, 0, 0). 입력은 (E,) 배열, tau는 (n,)."""
    T = T[:, None]
    d0, v0, a0, d1 = d0[:, None], v0[:, None], a0[:, None], d1[:, None]
    A = d1 - d0 - v0 * T - 0.5 * a0 * T * T
    B = -v0 * T - a0 * T * T
    C = -a0 * T * T
    c3 = 10 * A - 4 * B + 0.5 * C
    c4 = -15 * A + 7 * B - C
    c5 = 6 * A - 3 * B + 0.5 * C
    t = tau[None, :]
    d = d0 + v0 * T * t + 0.5 * a0 * T * T * t ** 2 + c3 * t ** 3 + c4 * t ** 4 + c5 * t ** 5
    dt = v0 * T + a0 * T * T * t + 3 * c3 * t ** 2 + 4 * c4 * t ** 3 + 5 * c5 * t ** 4
    ddt = a0 * T * T + 6 * c3 * t + 12 * c4 * t ** 2 + 20 * c5 * t ** 3
    return d, dt / T, ddt / (T * T)


class Planner:
    def __init__(self, p, boundary):
        self.p = p
        self.boundary = boundary
        self.last = {}
        self._boxes = ()
        self._ego_clear = (np.inf, np.inf)
        self.last_ego_clear = None

    def plan_limit(self, boxes=()):
        """경로 끝 s 상한: 구간 끝 + plan_beyond_end(갈림길 표지판 앞), 단 구간 장애물이 끝에 붙어 있으면
        차량 뒤끝이 그 장애물을 지날 때까지 연장(이때는 표지판도 장애물로 피함)."""
        p = self.p
        lim = p.section_end + p.plan_beyond_end - p.body_front  # 경로 끝에서 차 앞끝 위치 기준
        for b in boxes:
            if b.s0 < p.section_end:
                lim = max(lim, b.s1 + p.body_rear + p.pass_margin + p.stop_buffer + 0.1)
        return min(lim, self.boundary.s_max - p.body_front - 0.05)

    # ---------- Free-space / Gap ----------
    def free_intervals(self, sk, obs):
        """정거장 sk에서 차량 중심 d가 가질 수 있는 빈 구간 목록 [(lo, hi)]."""
        p = self.p
        wh = 0.5 * p.body_width
        ss = np.array([sk - p.body_rear, sk, sk + p.body_front])
        right, left = self.boundary.limits(ss)
        lo, hi = right.max() + wh + p.margin_boundary, left.min() - wh - p.margin_boundary
        if hi < lo:
            return [], []
        blocked = []
        m = p.margin_obstacle
        for cs, cd, hs, hd in obs:
            if cs + hs + m < sk - p.body_rear or cs - hs - m > sk + p.body_front:
                continue
            blocked.append((cd - hd - wh - m, cd + hd + wh + m))
        blocked.sort()
        free, cur = [], lo
        for a, b in blocked:
            if a > cur:
                free.append((cur, min(a, hi)))
            cur = max(cur, b)
            if cur >= hi:
                break
        if cur < hi:
            free.append((cur, hi))
        free = [(a, b) for a, b in free if b - a >= 0.0]
        gaps = [(a - wh - m, b + wh + m) for a, b in free]  # 시각화용 실제 통로 폭
        return free, gaps

    def nodes_for(self, sk, intervals, prev, keep=None):
        p = self.p
        cand = []
        for a, b in intervals:
            pts = [(0.5 * (a + b), 0)]
            if keep is not None and a <= keep <= b:
                pts.append((keep, -1))  # 현재 진행 방향 그대로 이어가는 위치
            if b - a > 0.04:
                pts += [(a + 0.01, 2), (b - 0.01, 2)]  # 좁은 통로용 가장자리
            comfort = p.clearance_soft - p.margin_obstacle
            if b - a > 2 * comfort:
                pts += [(a + comfort, 0), (b - comfort, 0)]  # 장애물에서 편안한 여유만큼 떨어진 최소 이탈 위치
            for lc in p.lane_centers:
                if a <= lc <= b:
                    pts.append((lc, 1))
            if a <= p.home_d <= b:
                pts.append((p.home_d, 1))
            if prev is not None:
                dp = prev.d_at(sk)
                if np.isfinite(dp) and a <= dp <= b:
                    pts.append((float(dp), -1))
            cand += pts
        cand.sort(key=lambda x: x[1])
        out = []
        for d, _ in cand:
            if all(abs(d - o) > p.node_merge for o in out):
                out.append(d)
        return out[:p.max_nodes_per_layer]

    def stations(self, s0, obs):
        p = self.p
        s_end = min(s0 + p.horizon, self.plan_limit(self._boxes))
        # 장애물 옆(중심)과 진입 지점(차 앞끝이 장애물 앞면에 닿는 곳): 장애물 앞에서 미리 통로로 들어가는 경로를
        # 만들 수 있게(중심 노드만 있으면 '지금 크게 꺾었다 옆에서 펴는' 경로가 없음)
        cand = [cs for cs, _, _, _ in obs]
        cand += [cs - hs - p.body_front - 0.02 for cs, _, hs, _ in obs]
        st = sorted(c for c in cand if s0 + p.station_min_ahead < c < s_end - 0.1)
        merged = []
        for s in st:
            if merged and s - merged[-1][-1] < p.station_merge:
                merged[-1].append(s)
            else:
                merged.append([s])
        st = [float(np.mean(g)) for g in merged]
        if st and s_end - st[-1] < 0.1:
            st[-1] = s_end  # 마지막 층은 항상 지평 끝(경로 길이가 들쭉날쭉해 연장 재계획이 반복되지 않도록)
        else:
            st.append(s_end)
        out, prev = [], s0
        for s in st:
            if s - prev > p.station_max_gap:
                n = int(math.ceil((s - prev) / p.station_max_gap))
                out += [prev + (s - prev) * k / n for k in range(1, n)]
            out.append(s)
            prev = s
        return out[:p.max_layers], s_end

    # ---------- Lattice + DP ----------
    def plan(self, start, boxes, grid=None, prev=None, w_cons=None, stamp=0.0):
        """start = (s0, d0, d0', d0''). 성공 시 Path, 실패 시 None."""
        p = self.p
        w_cons = p.w_cons if w_cons is None else w_cons
        s0, d0, v0, a0 = start
        self._boxes = boxes
        obs = obstacle_array([b for b in boxes if b.s1 > s0 - p.body_rear - 0.3])
        psi0 = math.atan(v0)
        a0_ = np.array([s0]), np.array([d0]), np.array([psi0])
        self._ego_clear = (float(body_clearance(*a0_, obs, p)[0]), float(boundary_clearance(*a0_, self.boundary, p)[0]))
        self.last_ego_clear = self._ego_clear
        sts, s_end = self.stations(s0, obs)
        layers, gaps_vis = [], []
        for sk in sts:
            free, gaps = self.free_intervals(sk, obs)
            gaps_vis.append((sk, gaps))
            keep = d0 + max(-0.3, min(0.3, v0 * (sk - s0)))
            nodes = self.nodes_for(sk, free, prev, keep)
            layers.append((sk, nodes))
        self.last = {'stations': sts, 'gaps': gaps_vis, 'layers': layers, 'edges': 0, 'feasible': 0}

        # 층 0 = 시작 상태. 모든 정거장은 선택 사항: 간선은 최대 max_skip 층을 건너뛸 수 있다.
        L = [(s0, np.array([d0]), np.array([v0]), np.array([a0]))]
        for sk, nodes in layers:
            n = np.asarray(nodes, float)
            L.append((sk, n, np.zeros(len(n)), np.zeros(len(n))))
        best = [np.zeros(1)] + [np.full(len(l[1]), np.inf) for l in L[1:]]
        parent = [[None]] + [[None] * len(l[1]) for l in L[1:]]
        seen_cost = [None] + [self._unknown_cost(grid, sk, l[1]) for sk, l in zip(sts, L[1:])]
        for j in range(1, len(L)):
            sj, nj_d = L[j][0], L[j][1]
            if len(nj_d) == 0:
                continue
            for i in range(max(0, j - p.max_skip), j):
                src = np.flatnonzero(np.isfinite(best[i]))
                if len(src) == 0:
                    continue
                si, di, vi, ai = L[i]
                res = self._edges(si, di[src], vi[src], ai[src], sj, nj_d, obs, s0, prev, w_cons)
                ok, ec, smp = res
                ec = ec + seen_cost[j][None, :]
                tot = np.where(ok, best[i][src][:, None] + ec, np.inf)
                arg = tot.argmin(0)
                val = tot[arg, np.arange(len(nj_d))]
                for n_ in np.flatnonzero(val < best[j]):
                    best[j][n_] = val[n_]
                    r = arg[n_]
                    parent[j][n_] = (i, int(src[r]), {k: (v if k == 's' else v[r, n_]) for k, v in smp.items()})
        # 종단: 지평 끝 층에 도달했으면 그 중 최소, 아니면 가장 멀리 간 층(부분 경로 -> 끝에서 정지)
        jl = None
        for j in range(len(L) - 1, 0, -1):
            if np.isfinite(best[j]).any():
                jl = j
                break
        if jl is None:
            return None
        complete = jl == len(L) - 1
        term = best[jl] + p.w_terminal * (L[jl][1] - p.home_d) ** 2
        if not complete and p.w_partial > 0:
            # 끝까지 못 가는 부분 경로: 끝점을 다음 층(막힌 곳 너머)의 통과 가능한 노드 쪽으로 당긴다.
            # (예: 왼쪽 차로로 피한 뒤 다음 무리가 오른쪽만 열려 있으면 왼쪽 차로 바깥이 아니라 오른쪽 가장자리에서 대기)
            nxt = next((L[j][1] for j in range(jl + 1, len(L)) if len(L[j][1])), None)
            if nxt is not None:
                term = term + p.w_partial * np.min((L[jl][1][:, None] - np.asarray(nxt)[None, :]) ** 2, axis=1)
        k = int(term.argmin())
        segs, nodes_out = [], []
        j = jl
        while j > 0:
            i, ki, seg = parent[j][k]
            segs.append(seg)
            nodes_out.append((float(L[j][0]), float(L[j][1][k])))
            j, k = i, ki
        segs.reverse()
        nodes_out.reverse()
        cat = lambda key: np.concatenate([segs[0][key]] + [g[key][1:] for g in segs[1:]])
        return Path(cat('s'), cat('d'), cat('d1'), cat('d2'), cat('psi'), cat('k'), cat('c'),
                    float(term.min()), bool(complete), nodes_out, stamp, cat('dd'), cat('b'))

    def _unknown_cost(self, grid, sk, nodes):
        if grid is None or len(nodes) == 0:
            return np.zeros(len(nodes))
        wh = 0.5 * self.p.body_width
        ss = np.full(len(nodes), sk)
        seen = grid.is_seen(ss, nodes) & grid.is_seen(ss, nodes + wh) & grid.is_seen(ss, nodes - wh)
        return self.p.w_unknown * (~seen)

    def _edges(self, si, d0, v0, a0, sj, dj, obs, s_start, prev, w_cons):
        """출발 노드 (m,) x 도착 노드 (n,) 간선을 한 번에 생성·검사. -> ok (m,n), cost (m,n), 샘플."""
        p = self.p
        m, nn = len(d0), len(dj)
        I, J = np.meshgrid(np.arange(m), np.arange(nn), indexing='ij')
        I, J = I.ravel(), J.ravel()
        T = sj - si
        n = max(4, int(math.ceil(T / p.sample_ds))) + 1
        tau = np.linspace(0.0, 1.0, n)
        d, d1, d2 = quintic(d0[I], v0[I], a0[I], dj[J], np.full(len(I), T), tau)
        s = si + T * tau
        S = np.broadcast_to(s, d.shape)
        psi = np.arctan(d1)
        kap = d2 / (1 + d1 * d1) ** 1.5
        dk = np.gradient(kap, T / (n - 1), axis=1)
        ddelta = p.wheelbase * dk / (1 + (p.wheelbase * kap) ** 2)
        clr = body_clearance(S, d, psi, obs, p)
        bcl = boundary_clearance(S, d, psi, self.boundary, p)
        ramp = np.clip((S - s_start) / 0.3, 0.2, 1.0)  # 출발 직후는 이미 가까울 수 있음
        # 출발 직후 start_relax 구간: 지금 차체 여유보다 더 가까워지지만 않으면 허용(수동 조종으로 이미
        # 장애물·노란선에 붙어 있어도 거기서 벗어나는 경로를 낼 수 있게). 그 뒤는 정상 여유.
        # 회전하며 벗어날 때 뒤 모서리가 바깥으로 약 1 cm 밀리는 것까지 허용(start_clear_tol). 절대 하한:
        # 노란선 -lane_measure_tol(투영 오차로 이미 걸쳐 보이는 경우), 장애물 start_obstacle_floor.
        eo, eb = self._ego_clear
        need_o, need_b = self._recovery_need(S - s_start, ramp, eo, eb)
        # 출발 직후 start_relax 구간은 운동학(헤딩·곡률·조향 변화율) 검사 면제: 지금 자세가 이미 한계 밖이어도
        # (수동 조종으로 크게 돈 경우) 거기서 풀어 나가는 경로를 허용. 충돌·경계 검사는 전 구간 유지.
        relax = (S - s_start) < p.start_relax
        rl, rr = side_radius(p, 'plan')  # 좌(+)·우(-) 회전 반경 한계가 다를 수 있음
        checks = {
            'curvature': (((kap <= 1.0 / rl) & (kap >= -1.0 / rr))
                          | (relax & (np.abs(kap) <= 1.0 / p.relax_min_radius))).all(1),
            'steer_rate': ((np.abs(ddelta) <= p.plan_steer_rate / max(p.plan_steer_speed, 0.05)) | relax).all(1),
            'heading': ((np.abs(psi) <= p.max_heading) | relax).all(1),
            'obstacle': (clr >= need_o).all(1),
            'boundary': (bcl >= need_b).all(1),
        }
        ok = np.logical_and.reduce(list(checks.values()))
        cand = self.last.setdefault('cand', [])
        for e in np.flatnonzero(ok)[:max(0, 40 - len(cand))]:  # 시각화: 통과한 후보 간선(최대 40)
            cand.append(np.column_stack((s, d[e])))
        rej = self.last.setdefault('rejected', {})
        for name, c in checks.items():  # 진단: 후보가 어느 검사에서 탈락했는지(중복 집계)
            rej[name] = rej.get(name, 0) + int((~c).sum())
        self.last['edges'] += len(I)
        self.last['feasible'] += int(ok.sum())
        w = np.full(n, T / (n - 1))
        w[0] = 0.0
        cost = (p.w_lane * (d - p.home_d) ** 2 + p.w_kappa * kap ** 2 + p.w_dkappa * dk ** 2
                + p.w_clear * np.maximum(0.0, p.clearance_soft - clr) ** 2)
        if prev is not None and w_cons > 0:
            dp = prev.d_at(s)
            has = np.isfinite(dp) & (s <= s_start + p.cons_length)  # 곧 실행할 구간만 유지 압력
            if has.any():
                cost = cost + w_cons * np.where(has[None, :], (d - np.nan_to_num(dp)) ** 2, 0.0)
        ec = (cost * w).sum(1)
        shp = (m, nn, n)
        smp = {'s': s, 'd': d.reshape(shp), 'd1': d1.reshape(shp), 'd2': d2.reshape(shp),
               'psi': psi.reshape(shp), 'k': kap.reshape(shp), 'c': clr.reshape(shp),
               'dd': np.abs(ddelta).reshape(shp), 'b': bcl.reshape(shp)}
        return ok.reshape(m, nn), ec.reshape(m, nn), smp

    def _recovery_need(self, travel, ramp, eo, eb):
        """출발 직후 요구 여유: 지금 여유(-start_clear_tol)에서 시작해 복귀 구간 끝에서 정상 여유가 되도록 선형 증가.
        이미 노란선·장애물 여유 안(또는 경계 밖)이면 복귀 구간을 start_recover_len으로 늘려 '돌아오는' 경로를 허용하고
        더 나빠지는 경로는 막는다. 장애물은 절대 하한 start_obstacle_floor."""
        p = self.p
        mo, mb = p.margin_obstacle * ramp, p.margin_boundary * ramp
        zo = p.start_clear_relax if eo >= p.margin_obstacle else p.start_recover_len
        zb = p.start_clear_relax if eb >= p.margin_boundary else p.start_recover_len
        uo = np.clip(travel / zo, 0.0, 1.0)
        ub = np.clip(travel / zb, 0.0, 1.0)
        lo_o = min(p.margin_obstacle, eo - p.start_clear_tol)
        lo_b = min(p.margin_boundary, eb - p.start_clear_tol)
        need_o = np.maximum(p.start_obstacle_floor, np.minimum(mo, lo_o + (p.margin_obstacle - lo_o) * uo))
        need_b = np.minimum(mb, lo_b + (p.margin_boundary - lo_b) * ub)
        return need_o, need_b

    # ---------- 조향 후보(사람처럼 꺾고-유지-풀기) ----------
    def escape(self, s0, d0, psi0, delta0, boxes, grid=None, stamp=0.0):
        """5차 노드 경로가 실패하거나 짧을 때: 실제 조향 궤적 후보를 차 운동 모델로 적분해 가장 멀리 갈 수 있는 것.

        조향각 escape_steers개(좌우 최대각까지) x 유지 길이 escape_holds -> 그 뒤 도로 방향(헤딩 0)으로 풀어
        지평 끝까지. 조향 변화율 한계 포함. 차체 전체로 장애물·경계 충돌 검사(출발 직후 완화 동일).
        """
        p = self.p
        L = p.wheelbase
        rl, rr = side_radius(p, 'escape')
        dl_max, dr_max = math.atan(L / rl), math.atan(L / rr)  # 좌(+)·우(-) 최대 조향
        dmax = max(dl_max, dr_max)
        rate = p.max_steering_rate / max(p.v_max, 0.1) * p.sample_ds  # 표본 간 최대 조향 변화
        n = int(p.horizon / p.sample_ds) + 1
        targets = np.linspace(-dr_max, dl_max, p.escape_steers)
        cands = [(dt, h, np.nan) for dt in targets for h in p.escape_holds]
        # 방향각 기준 후보: 끝까지 꺾어 헤딩이 th만큼 돌 때까지 유지 -> 반대로 펴기(사람이 좁은 슬라롬에서 하는 S자)
        cands += [(dt, np.inf, th) for dt in (-dr_max, dl_max) for th in p.escape_hold_psi]
        E = len(cands)
        S, D, PSI = np.zeros((E, n)), np.zeros((E, n)), np.zeros((E, n))
        K, DD = np.zeros((E, n)), np.zeros((E, n))
        S[:, 0], D[:, 0], PSI[:, 0] = s0, d0, psi0
        dl = np.full(E, float(np.clip(delta0, -dr_max, dl_max)))
        tgt = np.array([c[0] for c in cands])
        hold = np.array([c[1] for c in cands])
        hold_psi = np.array([c[2] for c in cands])
        turning = np.ones(E, bool)
        for i in range(1, n):
            travelled = (i - 1) * p.sample_ds
            turning &= (travelled < hold) & ~(np.abs(PSI[:, i - 1] - psi0) >= hold_psi)  # nan 비교는 False
            want = np.where(turning, tgt, np.clip(-p.escape_return_gain * PSI[:, i - 1], -dr_max, dl_max))
            step = np.clip(want - dl, -rate, rate)
            DD[:, i] = np.abs(step) / p.sample_ds
            dl = dl + step
            k = np.tan(dl) / L
            ps = PSI[:, i - 1] + k * p.sample_ds
            mid = 0.5 * (PSI[:, i - 1] + ps)
            S[:, i] = S[:, i - 1] + np.cos(mid) * p.sample_ds
            D[:, i] = D[:, i - 1] + np.sin(mid) * p.sample_ds
            PSI[:, i] = ps
            K[:, i] = k
        K[:, 0] = np.tan(dl * 0 + np.clip(delta0, -dr_max, dl_max)) / L
        obs = obstacle_array([b for b in boxes if b.s1 > s0 - p.body_rear - 0.3])
        clr = body_clearance(S, D, PSI, obs, p)
        bcl = boundary_clearance(S, D, PSI, self.boundary, p)
        travel = np.arange(n) * p.sample_ds
        ramp = np.clip(travel / 0.3, 0.2, 1.0)
        a0 = np.array([s0]), np.array([d0]), np.array([psi0])
        eo = float(body_clearance(*a0, obs, p)[0])
        eb = float(boundary_clearance(*a0, self.boundary, p)[0])
        need_o, need_b = self._recovery_need(np.broadcast_to(travel, S.shape), ramp, eo, eb)
        ok = (clr >= need_o) & (bcl >= need_b) & (np.abs(PSI) < max(1.35, p.max_heading + 0.1)) & (np.diff(np.c_[S[:, :1] - 1, S], axis=1) > 0)
        bad = ~ok
        first = np.where(bad.any(1), bad.argmax(1), n)          # 처음 막히는 표본
        free = travel[np.maximum(first - 1, 0)]
        # 점수: 갈 수 있는 길이 우선, 그다음 여유·도로 방향 정렬·가운데 근접(작은 가중치)
        cmin = np.array([clr[e, :max(first[e], 1)].min() for e in range(E)])
        score = (free + p.escape_w_clear * np.minimum(cmin, p.clearance_soft)
                 - p.escape_w_heading * np.abs(PSI[np.arange(E), np.maximum(first - 1, 0)])
                 - 0.05 * np.abs(D[np.arange(E), np.maximum(first - 1, 0)] - p.home_d))
        e = int(score.argmax())
        m = int(max(first[e] - 1, 0))
        self.last['escape'] = {'free': round(float(free[e]), 2), 'steer': round(float(tgt[e]), 3),
                               'hold': float(hold[e]), 'n': E}
        if free[e] < p.escape_min_len:
            return None
        sl = slice(0, m + 1)
        d1 = np.tan(PSI[e, sl])
        out = Path(S[e, sl].copy(), D[e, sl].copy(), d1, K[e, sl] * (1 + d1 * d1) ** 1.5, PSI[e, sl].copy(),
                   K[e, sl].copy(), clr[e, sl].copy(), -float(score[e]), bool(first[e] >= n),
                   [(float(S[e, m]), float(D[e, m]))], stamp, DD[e, sl] * L, bcl[e, sl].copy())
        cand = self.last.setdefault('cand', [])
        for j in np.argsort(-score)[:12]:  # 시각화: 상위 조향 후보
            cand.append(np.column_stack((S[j, :max(first[j], 2)], D[j, :max(first[j], 2)])))
        return out

    # ---------- 기존 경로 검사 ----------
    def path_clearance(self, path, s_from, boxes):
        """s_from 이후 경로 샘플의 (장애물 여유 부족, 경계 여유 부족) — 계획 당시보다 얼마나 나빠졌는지(양수=나빠짐).
        기준 = min(정상 여유, 계획 당시 여유): 복귀 경로처럼 일부러 여유가 작게 계획된 구간을 곧바로 막힘으로 보지 않음."""
        p = self.p
        i = max(0, int(np.searchsorted(path.s, s_from)) - 1)
        if i >= len(path.s) - 1:
            return -np.inf, -np.inf
        s, d, psi = path.s[i:], path.d[i:], path.psi[i:]
        obs = obstacle_array(boxes)
        c = body_clearance(s, d, psi, obs, p)
        b = boundary_clearance(s, d, psi, self.boundary, p)
        ref_c = np.minimum(p.margin_obstacle, path.clear[i:]) if path.clear is not None else p.margin_obstacle
        ref_b = np.minimum(p.margin_boundary, path.bclear[i:]) if path.bclear is not None else p.margin_boundary
        return float(np.max(ref_c - c)), float(np.max(ref_b - b))

    def verified_until(self, path, s_from, grid, lane=None):
        """s_from부터 차체 중심·양옆이 모두 관측된 빈 공간인 마지막 s.
        lane이 있으면: 노란 경계에 가까운(lane_unknown_band 이내) 곳은 그 쪽 노란선이 실제로 관측돼야 검증으로 본다."""
        i = max(0, int(np.searchsorted(path.s, s_from)) - 1)
        s, d = path.s[i:], path.d[i:]
        wh = 0.5 * self.p.body_width
        near = s <= s_from + 0.25
        ok = near | (grid.is_seen(s, d) & grid.is_seen(s, d + wh) & grid.is_seen(s, d - wh))
        if lane is not None:
            r, l = lane.limits(s)
            band = self.p.lane_unknown_band
            need_l = (l - (d + wh)) < band
            need_r = ((d - wh) - r) < band
            ok &= near | ((~need_l | lane.known(s, 1)) & (~need_r | lane.known(s, -1)))
        bad = np.flatnonzero(~ok)
        return float(s[bad[0]]) if len(bad) else float(s[-1])
