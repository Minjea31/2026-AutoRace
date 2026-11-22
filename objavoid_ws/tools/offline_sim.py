#!/usr/bin/env python3
"""개발용 빠른 폐루프: simul의 Simulation(운동·충돌·LiDAR 모델)을 그대로 import해 ROS 없이 돌린다.

최종 검증은 tools/run_sim_tests.py(ROS + headless Panda3D 시뮬레이터)로 한다.
정답 장애물 좌표는 평가(최소 간격 계산)에만 사용한다.

  PYTHONNOUSERSITE=1 bash -c 'source ../simul/env.sh >/dev/null && python tools/offline_sim.py scenarios/default7.yaml -v'
"""
import argparse
import json
import math
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
WS = os.path.dirname(HERE)
ROOT = os.path.dirname(WS)
sys.path.insert(0, os.path.join(ROOT, 'simul/src/limo_parking_sim'))
sys.path.insert(0, os.path.join(WS, 'src/autorace_objavoid_v3'))

from limo_parking_sim.config import load_config  # noqa: E402
from limo_parking_sim.model import Simulation  # noqa: E402
from limo_parking_sim.parking_adapter import drive_to_twist  # noqa: E402

from autorace_objavoid_v3.mission import ObjAvoidCore, DONE  # noqa: E402
from autorace_objavoid_v3.params import Params  # noqa: E402
sys.path.insert(0, HERE)
from metrics import rect_distance, section_obstacles  # noqa: E402


LANE_YAML = os.path.join(ROOT, 'lane_fusion_ws/src/autorace_lane_deep_lite/config/sim/mission_vision.yaml')


class VirtualLaneCamera:
    """개발용: 퓨전 lane.yaml BEV 시야에서 트랙 텍스처의 노란 페인트를 샘플링(딥 마스크 대역, 가림 무시)."""

    def __init__(self):
        import yaml
        from PIL import Image
        from autorace_objavoid_v3.lane_boundary import GroundProjection
        self.gp = GroundProjection(yaml.safe_load(open(LANE_YAML))['lane_lite'])
        im = np.asarray(Image.open(os.path.join(ROOT, 'simul/src/limo_parking_sim/assets/track.png')).convert('RGB'))
        im = im[::4, ::4].astype(int)  # 5 mm 해상도
        self.yellow = (im[..., 0] > 150) & (im[..., 1] > 130) & (im[..., 2] < 100)
        self.white = (im[..., 0] > 190) & (im[..., 1] > 190) & (im[..., 2] > 190)
        self.H, self.W = self.yellow.shape
        xs = np.arange(self.gp.near, self.gp.max_range, 0.01)
        ys = np.arange(-self.gp.max_lateral, self.gp.max_lateral, 0.01)
        X, Y = np.meshgrid(xs, ys)
        dense = np.column_stack((X.ravel(), Y.ravel()))
        u, v = self.gp.to_image(dense)
        ok = (u >= 0) & (u <= self.gp.w - 1) & (v >= self.gp.roi[2] * (self.gp.h - 1)) & (v <= self.gp.h - 1)
        self.dense = dense[ok]

    def capture(self, sim):
        c, s = math.cos(sim.yaw), math.sin(sim.yaw)
        mx = sim.x + c * self.dense[:, 0] - s * self.dense[:, 1]
        my = sim.y + s * self.dense[:, 0] + c * self.dense[:, 1]
        col = (mx / 10.0 * self.W).astype(int)
        row = (self.H - 1 - my / 6.0 * self.H).astype(int)
        ok = (col >= 0) & (col < self.W) & (row >= 0) & (row < self.H)
        hit = np.zeros(len(mx), bool)
        hit[ok] = self.yellow[row[ok], col[ok]]
        wh = np.zeros(len(mx), bool)
        wh[ok] = self.white[row[ok], col[ok]]
        self.last_white = self.dense[wh]
        return self.dense[hit], self.gp.view_points


def manual_driver(sim, route, v=0.3, L=0.2, ld=0.35):
    """수동 조종 대역: 정답 자세로 손 경로(waypoints, map 좌표)를 Pure Pursuit. V3 명령은 차에 적용하지 않음."""
    pts = np.asarray(route, float)
    d = np.hypot(pts[:, 0] - sim.x, pts[:, 1] - sim.y)
    i = int(np.argmin(d))
    arc = np.r_[0, np.cumsum(np.hypot(*np.diff(pts, axis=0).T))]
    j = min(len(pts) - 1, int(np.searchsorted(arc, arc[i] + ld)))
    if i >= len(pts) - 2:
        return 0.0, 0.0
    tx, ty = pts[j]
    a = math.atan2(ty - sim.y, tx - sim.x) - sim.yaw
    delta = math.atan2(2 * L * math.sin(a), max(math.hypot(tx - sim.x, ty - sim.y), 1e-3))
    return v, delta


def run(map_path, p, timeout=40.0, verbose=False, scan_drop=0.0, seed=0, odom_delay=0.0, trace=False,
        spawn=None, route=None):
    cfg = load_config(None, map_path)
    cfg['simulation']['spawn'] = list(spawn) if spawn else cfg['world']['spawns']['obstacle']
    sim = Simulation(cfg)
    rng = np.random.default_rng(seed)
    x0, y0, yaw0 = cfg['simulation']['spawn']
    obs = section_obstacles(cfg)
    core = ObjAvoidCore(p)
    dt = 1 / 120
    angle_min, inc = float(sim.angles[0]), float(sim.angles[1] - sim.angles[0])
    ctrl_dt = 1 / p.control_rate
    next_scan = next_ctrl = next_odom = 0.0
    activated = False
    min_gap, steer, steer_rate, last_delta = np.inf, [], [], 0.0
    odom_buf = []
    core_has_path, fails = [], []
    cam = VirtualLaneCamera() if p.use_lane else None
    next_lane = 0.0
    log = []
    while sim.time < timeout:
        t = sim.time
        if t + 1e-9 >= next_odom:
            next_odom += 0.02
            dx, dy = sim.x - x0, sim.y - y0
            ox = math.cos(yaw0) * dx + math.sin(yaw0) * dy
            oy = -math.sin(yaw0) * dx + math.cos(yaw0) * dy
            odom_buf.append((t, ox, oy, sim.yaw - yaw0, sim.speed, sim.yaw_rate))
            while odom_buf and odom_buf[0][0] <= t - odom_delay + 1e-9:
                core.on_odom(*odom_buf.pop(0))
        if not activated and t >= 0.5:
            activated = core.activate(t)
        if cam is not None and activated and t + 1e-9 >= next_lane:
            next_lane += 0.1  # 딥 마스크 10 Hz
            yb, vb = cam.capture(sim)
            core.on_lane(t, yb, vb, cam.last_white)
        if t + 1e-9 >= next_scan:
            next_scan += 0.1
            ranges = sim.scan()
            if activated and rng.random() >= scan_drop:
                core.process_scan(t, ranges, angle_min, inc, t)
        if t + 1e-9 >= next_ctrl:
            next_ctrl += ctrl_dt
            out = core.control(t, ctrl_dt) if activated else None
            if route is not None:  # 수동 조종 재현: V3는 관찰만
                core_has_path.append(core.path is not None)
                if core.path is None and core.last_fail:
                    fails.append(core.last_fail)
                v_m, d_m = manual_driver(sim, route)
                sim.command(*drive_to_twist(v_m, d_m, cfg['vehicle']['wheelbase'], 1.0, sim.max_steering))
                out = None
            if out is not None:
                v, delta = out
                sim.command(*drive_to_twist(v, delta, cfg['vehicle']['wheelbase'], 1.0, sim.max_steering))
                steer.append(delta)
                steer_rate.append(abs(delta - last_delta) / ctrl_dt)
                last_delta = delta
            if trace and activated:
                log.append((round(t, 2), round(sim.x, 3), round(sim.y, 3), core.state, core.reason))
        sim.step(dt)
        body = sim.body()
        for b in obs:
            min_gap = min(min_gap, rect_distance(body, b))
        if core.state == DONE and route is None:
            break
        if sim.collisions and route is None:
            break
    st = core.status()
    res = {'map': os.path.basename(map_path), 'state': core.state, 'reason': core.reason,
           'time': round(sim.time, 2), 'collisions': sim.collisions, 'collision_names': sim.collision_names,
           'min_gap': round(float(min_gap), 4), 'final': [round(sim.x, 3), round(sim.y, 3), round(sim.yaw, 3)],
           'passed': st.get('passed'), 'replans': st['replans'], 'route_changes': st['route_changes'],
           'plan_failures': st['plan_failures'], 'plan_reasons': st['plan_reasons'],
           'plan_ms': st['plan_ms'], 'percep_ms': st['percep_ms'], 'edges': st['edges'],
           'ey': st['ey'], 'epsi': st['epsi'], 'stops': st['stop_events'],
           'steer_max': round(float(np.max(np.abs(steer))), 3) if steer else 0,
           'steer_rate_max': round(float(np.max(steer_rate)), 3) if steer_rate else 0,
           'done_reason': st['done_reason'], 'lane': st.get('lane')}
    if trace:
        res['trace'] = log
    if route is not None:
        res['path_available'] = round(float(np.mean(core_has_path)), 3) if core_has_path else None
        res['fail_samples'] = fails[::max(1, len(fails) // 6)][:6]
        res['fail_checks'] = st.get('fail_checks')
    if verbose:
        print(json.dumps(res, ensure_ascii=False))
    return res, core


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('maps', nargs='+')
    ap.add_argument('-v', action='store_true')
    ap.add_argument('--timeout', type=float, default=40)
    ap.add_argument('--set', nargs='*', default=[], help='param=value')
    ap.add_argument('--scan-drop', type=float, default=0)
    ap.add_argument('--odom-delay', type=float, default=0)
    ap.add_argument('--spawn', nargs=3, type=float)
    a = ap.parse_args()
    p = Params()
    for kv in a.set:
        k, v = kv.split('=')
        cur = getattr(p, k)
        setattr(p, k, json.loads(v) if isinstance(cur, list) else (v.lower() == 'true') if isinstance(cur, bool)
                else type(cur)(float(v)))
    for m in a.maps:
        w = time.perf_counter()
        r, _ = run(m, p, a.timeout, False, a.scan_drop, 0, a.odom_delay, spawn=a.spawn)
        print(f"{r['map']:22s} {r['state']:5s} {r['reason']:16s} t={r['time']:5.1f} coll={r['collisions']} "
              f"gap={r['min_gap']:.3f} passed={r['passed']} replan={r['replans']} chg={r['route_changes']} "
              f"fail={r['plan_failures']} plan_ms={r['plan_ms'].get('mean')}/{r['plan_ms'].get('max')} "
              f"ey={r['ey'].get('max')} steer={r['steer_max']} rate={r['steer_rate_max']} stops={r['stops'][:4]} "
              f"wall={time.perf_counter() - w:.1f}s")
        if a.v:
            print(json.dumps(r, ensure_ascii=False))


if __name__ == '__main__':
    main()
