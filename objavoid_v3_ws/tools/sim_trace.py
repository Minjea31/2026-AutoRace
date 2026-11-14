#!/usr/bin/env python3
"""오프라인 폐루프 시뮬 1회 + 실차 조향 특성 모델(좌우 이득·최대각·지연) + 궤적 출력.

  PYTHONNOUSERSITE=1 bash -c 'source ../simul/env.sh && python tools/sim_trace.py scenarios/slalom_rlr.yaml \
      --act 1.0,0.8,0.3,0.44,0.34 --set plan_min_radius_right=0.6 reverse_enable=false -v'

--act 좌이득,우이득,지연s[,좌최대rad,우최대rad]: 명령 조향 δ에 이득을 곱하고 최대각으로 자른 뒤 지연해 시뮬 차량에 넣는다
(04번 실측: 서보 상한 0.48에서 좌 +0.44, 우 -0.34 rad, 명령->실제 지연 약 0.3 s).
--spawn x,y,yaw: 시작 자세(시뮬 좌표). -v: 0.5 s마다 (t, x, y, 상태) 출력.
"""
import argparse
import collections
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import offline_sim as osim  # noqa: E402
from autorace_objavoid_v3.params import Params  # noqa: E402


def actuator_model(gl, gr, lag, maxl=9.0, maxr=9.0, wheelbase=0.2):
    from limo_parking_sim import model as sm
    cls = next(v for v in sm.__dict__.values() if isinstance(v, type) and hasattr(v, 'command'))
    orig = cls.command
    q = collections.deque()

    def command(self, speed, yaw_rate):
        q.append((self.time, speed, yaw_rate))
        while len(q) > 1 and q[1][0] <= self.time - lag:
            q.popleft()
        _, v, w = q[0]
        if abs(v) < 1e-4:
            return orig(self, v, w)
        dl = math.atan(w * wheelbase / v)
        da = max(-maxr, min(maxl, dl * (gl if dl > 0 else gr)))
        return orig(self, v, v * math.tan(da) / wheelbase)
    cls.command = command


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('scenario')
    ap.add_argument('--set', nargs='*', default=[])
    ap.add_argument('--act')
    ap.add_argument('--spawn')
    ap.add_argument('--timeout', type=float, default=60.0)
    ap.add_argument('-v', action='store_true')
    a = ap.parse_args()
    p = Params()
    for kv in a.set:
        k, v = kv.split('=', 1)
        cur = getattr(p, k)
        setattr(p, k, json.loads(v) if isinstance(cur, list) else (v.lower() == 'true') if isinstance(cur, bool)
                else type(cur)(float(v)))
    if a.act:
        actuator_model(*map(float, a.act.split(',')))
    spawn = tuple(map(float, a.spawn.split(','))) if a.spawn else None
    r, core = osim.run(a.scenario, p, a.timeout, False, 0, 0, 0, trace=a.v, spawn=spawn)
    if a.v:
        last = -1.0
        for t, x, y, st, rs in r['trace']:
            if t - last >= 0.5:
                print(t, 'x=', round(x, 2), 'y=', round(y, 2), st, rs)
                last = t
    print(os.path.basename(a.scenario), {k: r[k] for k in ('state', 'reason', 'collisions', 'min_gap') if k in r},
          'reverses', core.reverses)


if __name__ == '__main__':
    main()
