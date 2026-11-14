#!/usr/bin/env python3
"""계산 시간 벤치마크(최악 조건 포함). 시뮬레이터 LiDAR 모델로 만든 실제 스캔을 사용.

  PYTHONNOUSERSITE=1 bash -c 'source ../simul/env.sh >/dev/null && python tools/bench.py'
"""
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
from autorace_objavoid_v3.params import Params  # noqa: E402
from autorace_objavoid_v3.perception import Box, Tracker, VisibilityGrid, detect  # noqa: E402
from autorace_objavoid_v3.planner import Planner  # noqa: E402
from autorace_objavoid_v3.road import PriorBoundary, RoadFrame  # noqa: E402


def stats(a):
    a = np.asarray(a)
    return {'mean': round(float(a.mean()), 3), 'p50': round(float(np.percentile(a, 50)), 3),
            'p99': round(float(np.percentile(a, 99)), 3), 'max': round(float(a.max()), 3)}


def main():
    p = Params()
    bnd = PriorBoundary(p.boundary_s, p.boundary_left, p.boundary_right)
    out = {}
    # 1) 인지: 실제 시뮬 스캔(2445빔), 7대 배치 출발 위치
    cfg = load_config(None, os.path.join(WS, 'scenarios/default7.yaml'))
    cfg['simulation']['spawn'] = cfg['world']['spawns']['obstacle']
    sim = Simulation(cfg)
    ranges = sim.scan()
    am, inc = float(sim.angles[0]), float(sim.angles[1] - sim.angles[0])
    fr = RoadFrame(0, 0, 0)
    tr, grid = Tracker(p), VisibilityGrid(p, bnd.s_min, bnd.s_max)
    td, tt, tg = [], [], []
    for k in range(300):
        w = time.perf_counter()
        boxes, _ = detect(ranges, am, inc, (p.lidar_x, 0, 0), fr, bnd, 0.0, p)
        td.append((time.perf_counter() - w) * 1e3)
        w = time.perf_counter()
        tr.update(boxes, k * 0.1, -0.16, ranges, {'frame': fr, 'lidar_pose': (p.lidar_x, 0, 0),
                                                  'angle_min': am, 'angle_inc': inc, 'dt': 0.1})
        tt.append((time.perf_counter() - w) * 1e3)
        w = time.perf_counter()
        grid.update(ranges, am, inc, (p.lidar_x, 0, 0), fr)
        tg.append((time.perf_counter() - w) * 1e3)
    out['beams'] = len(ranges)
    out['detect_ms'], out['track_ms'], out['grid_ms'] = stats(td), stats(tt), stats(tg)
    # 2) 계획: 무작위 7대(3차로 x 8열) + 무작위 출발 상태, 지평 안 장애물 최대
    rng = np.random.default_rng(0)
    pl = Planner(p, bnd)
    xs = [0.755 + 0.4703 * i for i in range(8)]
    tp, edges, ok = [], [], 0
    for k in range(1000):
        cells = rng.choice(24, 7, replace=False)
        boxes = []
        for c in cells:
            s, d = xs[c % 8], (-0.52, 0.0, 0.52)[c // 8]
            boxes.append(Box(s - 0.165, s + 0.165, d - 0.115, d + 0.115))
        s0 = rng.uniform(0, 1.2)
        start = (s0, rng.uniform(-0.3, 0.3), rng.uniform(-0.2, 0.2), rng.uniform(-0.5, 0.5))
        w = time.perf_counter()
        path = pl.plan(start, boxes, grid)
        tp.append((time.perf_counter() - w) * 1e3)
        edges.append(pl.last['edges'])
        ok += path is not None
    out['plan_ms'] = stats(tp)
    out['plan_edges'] = stats(edges)
    out['plan_layers_max'] = p.max_layers
    out['plan_returned_path_ratio'] = round(ok / 1000, 3)
    print(json.dumps(out, ensure_ascii=False, indent=1))
    json.dump(out, open(os.path.join(WS, 'results', 'bench.json'), 'w'), ensure_ascii=False, indent=1)


if __name__ == '__main__':
    main()
