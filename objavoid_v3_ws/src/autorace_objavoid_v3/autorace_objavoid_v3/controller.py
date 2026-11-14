"""PD 경로 추종: steering = atan(L*k_ff) - K_y*e_y - K_psi*e_psi (- K_i*int e_y).

선형화(기준점 = base_link, 운동학 자전거 모델):
  e_y'' = -(v^2 K_y / L) e_y - (v K_psi / L) e_y'
공간 고유주파수 kn, 감쇠비 z로 K_y = L kn^2, K_psi = 2 z L kn (속도와 무관한 거리 응답).
기본값 K_y=1.3, K_psi=1.0 -> kn ~ 2.55/m, z ~ 0.98.
"""
import math

import numpy as np


class PathTracker:
    def __init__(self, p):
        self.p = p
        self.reset()

    def reset(self):
        self.delta = 0.0
        self.v = 0.0
        self.i_err = 0.0
        self.idx = 0
        self.path_id = None

    def project(self, path, s, d):
        """차량 위치에 대응하는 경로 기준점 인덱스(이전 인덱스 주변 탐색)."""
        if self.path_id != id(path):
            self.path_id = id(path)
            self.idx = int(np.clip(np.searchsorted(path.s, s), 0, len(path.s) - 1))
        lo = max(0, self.idx - 10)
        hi = min(len(path.s), self.idx + 30)
        dist = (path.s[lo:hi] - s) ** 2 + (path.d[lo:hi] - d) ** 2
        self.idx = lo + int(np.argmin(dist))
        return self.idx

    def errors(self, path, s, d, psi):
        i = self.project(path, s, d)
        # 기준점을 인접 선분에 수직 투영해 연속값으로
        j = min(i + 1, len(path.s) - 1)
        if j == i:
            i = max(0, i - 1)
            j = i + 1
        ps, pd = path.s[i], path.d[i]
        ts, td = path.s[j] - ps, path.d[j] - pd
        L2 = ts * ts + td * td
        u = 0.0 if L2 < 1e-12 else max(0.0, min(1.0, ((s - ps) * ts + (d - pd) * td) / L2))
        rs, rd = ps + u * ts, pd + u * td
        rpsi = path.psi[i] + u * (path.psi[j] - path.psi[i])
        ey = -(s - rs) * math.sin(rpsi) + (d - rd) * math.cos(rpsi)
        epsi = (psi - rpsi + math.pi) % (2 * math.pi) - math.pi
        return rs, rd, rpsi, ey, epsi

    def step(self, path, s, d, psi, dt, v_cap, verified_s=None):
        """-> (speed, steering, info). v_cap: 외부 속도 상한(정지 시 0)."""
        p = self.p
        dt = max(1e-3, min(dt, 0.2))
        rs, rd, rpsi, ey, epsi = self.errors(path, s, d, psi)
        v_now = max(self.v, 0.05)
        s_ff = rs + v_now * p.ff_preview_time
        k_ff = float(np.interp(s_ff, path.s, path.kappa))
        if p.k_i > 0:
            self.i_err = float(np.clip(self.i_err + ey * dt, -p.i_limit, p.i_limit))
        fb = -p.k_y * ey - p.k_psi * epsi - p.k_i * self.i_err
        fb = max(-p.fb_limit, min(p.fb_limit, fb))
        target = math.atan(p.wheelbase * k_ff) + fb
        if p.steer_gain != 1.0:  # 실차가 명령보다 덜/더 돌면 곡률 기준으로 보정
            target = math.atan(p.steer_gain * math.tan(target))
        target = max(-p.max_steering, min(p.max_steering, target))
        step = p.max_steering_rate * dt
        self.delta += max(-step, min(step, target - self.delta))

        # ---- 속도 ----
        kmax = 1.0 / p.plan_min_radius
        ahead = (path.s >= rs) & (path.s <= rs + 0.5)
        k_ahead = float(np.abs(path.kappa[ahead]).max()) if ahead.any() else 0.0
        v = p.v_max * max(0.5, 1.0 - p.curve_slow * k_ahead / kmax)
        if path.dsteer is not None and ahead.any():
            # 거리당 조향 변화가 큰 구간은 실제 조향 변화율 한계 안에 들도록 감속
            need = float(path.dsteer[ahead].max())
            if need > 1e-6:
                v = min(v, 0.8 * p.max_steering_rate / need)
        near = (path.s >= rs - 0.2) & (path.s <= rs + 0.6)
        c_min = float(path.clear[near].min()) if near.any() else np.inf
        if c_min < p.narrow_clearance:
            v = min(v, p.v_narrow)
        if abs(ey) > 0.08 or abs(epsi) > 0.3:
            v = min(v, p.v_narrow)
        stop_s = path.end_s if verified_s is None else min(path.end_s, verified_s)
        room = stop_s - rs - p.stop_buffer
        if not path.complete or verified_s is not None:
            v = min(v, math.sqrt(2 * p.decel * max(0.0, room)))
        v = min(v, v_cap)
        if v < p.v_min:
            v = 0.0 if room < 0.05 or v_cap <= 0 else p.v_min
        if v > self.v:
            v = min(v, self.v + p.accel * dt)
        else:
            v = max(v, self.v - p.decel * 2 * dt)
        self.v = max(0.0, v)
        info = {'ref_s': rs, 'ref_d': rd, 'ref_psi': rpsi, 'ey': ey, 'epsi': epsi,
                'k_ff': k_ff, 'c_min': c_min, 'room': room}
        return self.v, self.delta, info

    def hold_stop(self, dt):
        """정지 명령: 조향은 유지(급변 없음), 속도 0."""
        self.v = 0.0
        return 0.0, self.delta
