"""ROS 없이 실행하는 단위 테스트: PYTHONNOUSERSITE=1 python3 -m pytest -q src/autorace_objavoid_v3/test"""
import math
import os

import numpy as np
import pytest
import yaml

from autorace_objavoid_v3.controller import PathTracker
from autorace_objavoid_v3.params import Params, param_items
from autorace_objavoid_v3.perception import Box, Tracker, VisibilityGrid, correct_extent, detect
from autorace_objavoid_v3.planner import Planner, body_clearance, obstacle_array, quintic
from autorace_objavoid_v3.road import PriorBoundary, RoadFrame

P = Params()
BND = PriorBoundary([-1, 6], [0.768, 0.768], [-0.768, -0.768])


def test_quintic_boundary_conditions():
    tau = np.linspace(0, 1, 101)
    d, d1, d2 = quintic(np.array([0.1]), np.array([0.2]), np.array([-0.5]), np.array([0.5]), np.array([1.3]), tau)
    assert d[0, 0] == pytest.approx(0.1) and d1[0, 0] == pytest.approx(0.2) and d2[0, 0] == pytest.approx(-0.5)
    assert d[0, -1] == pytest.approx(0.5) and d1[0, -1] == pytest.approx(0, abs=1e-9)
    assert d2[0, -1] == pytest.approx(0, abs=1e-9)
    # 수치 미분과 해석 미분 일치
    ds = 1.3 / 100
    assert np.allclose(np.gradient(d[0], ds)[5:-5], d1[0, 5:-5], atol=1e-3)


def test_body_clearance_axis_aligned():
    obs = obstacle_array([Box(1.0, 1.32, -0.11, 0.11)])
    # 차량 앞끝 0.161, 장애물 앞면 1.0 -> 간격 1.0 - 0.161 = 0.839
    c = body_clearance(np.array([0.0]), np.array([0.0]), np.array([0.0]), obs, P)
    assert c[0] == pytest.approx(0.839, abs=1e-9)
    # 옆으로 0.3 비켜 나란히: 간격 0.3 - 0.11 - 0.11 = 0.08
    c = body_clearance(np.array([1.16]), np.array([0.30]), np.array([0.0]), obs, P)
    assert c[0] == pytest.approx(0.08, abs=1e-9)
    # 겹치면 음수
    assert body_clearance(np.array([1.1]), np.array([0.1]), np.array([0.0]), obs, P)[0] < 0


def test_body_clearance_rotated_is_conservative():
    obs = obstacle_array([Box(1.0, 1.32, 0.3, 0.52)])
    psi = 0.4
    c = body_clearance(np.array([1.16]), np.array([0.0]), np.array([psi]), obs, P)[0]
    # 회전 차체 모서리 최대 d = 0.161 sin + 0.11 cos
    true_gap = 0.3 - (0.161 * math.sin(psi) + 0.11 * math.cos(psi))
    assert c <= true_gap + 1e-9


def test_free_intervals_and_gap_width():
    pl = Planner(P, BND)
    obs = obstacle_array([Box(1.0, 1.32, -0.11, 0.11)])
    free, gaps = pl.free_intervals(1.16, obs)
    wh, m = P.body_width / 2, P.margin_obstacle
    assert len(free) == 2
    assert free[0][1] == pytest.approx(-0.11 - wh - m)
    assert free[1][0] == pytest.approx(0.11 + wh + m)
    # 실제 통로 폭(시각화) >= 차폭 + 양쪽 여유
    for a, b in gaps:
        assert b - a >= P.body_width + 2 * m - 1e-9


def test_narrow_gap_excluded():
    pl = Planner(P, BND)
    # 두 장애물 사이 실제 틈 0.25 < 0.22 + 2*0.025
    obs = obstacle_array([Box(1.0, 1.32, -0.6, -0.125), Box(1.0, 1.32, 0.125, 0.6)])
    free, _ = pl.free_intervals(1.16, obs)
    assert all(not (a < 0 < b) for a, b in free)


def test_plan_avoids_and_respects_kinematics():
    pl = Planner(P, BND)
    path = pl.plan((0.0, 0.0, 0.0, 0.0), [Box(1.5, 1.82, -0.11, 0.11)])
    assert path is not None and path.complete
    assert np.all(np.abs(path.kappa) <= 1 / P.plan_min_radius + 1e-9)
    assert np.min(path.clear) >= P.margin_obstacle * 0.2  # 출발 직후 완화 구간 포함
    beside = (path.s > 1.5) & (path.s < 1.82)
    assert np.all(np.abs(path.d[beside]) >= 0.11 + 0.11 + P.margin_obstacle - 1e-6)


def test_plan_none_or_partial_when_blocked():
    pl = Planner(P, BND)
    wall = [Box(1.5, 1.82, -0.8, -0.2), Box(1.5, 1.82, -0.1, 0.8)]  # 틈 0.10 m
    path = pl.plan((0.0, 0.0, 0.0, 0.0), wall)
    assert path is None or (not path.complete and path.end_s < 1.5 - P.body_front)


def test_correct_extent_never_shrinks_and_extends_away():
    lo, hi = correct_extent(0.5, 0.6, 0.0, 0.23, False, False)  # 센서가 아래쪽 -> 위로 확장
    assert (lo, hi) == pytest.approx((0.5, 0.73))
    lo, hi = correct_extent(0.5, 0.6, 1.0, 0.23, False, False)
    assert (lo, hi) == pytest.approx((0.37, 0.6))
    assert correct_extent(0.0, 0.5, 0.2, 0.23, False, False) == (0.0, 0.5)


def test_detect_min_size_and_tracker_pass():
    fr = RoadFrame(0, 0, 0)
    # 정면 1.2 m 앞 폭 0.22 면을 보는 스캔 합성
    ang = np.linspace(-1.9, 1.9, 2000)
    r = np.full(len(ang), 0.0)
    for i, a in enumerate(ang):
        if abs(a) < 1.5:
            x = 1.2
            y = x * math.tan(a)
            if abs(y) <= 0.11:
                r[i] = math.hypot(x, y)
    boxes, _ = detect(r, ang[0], ang[1] - ang[0], (0.0, 0.0, 0.0), fr, BND, 0.0, P)
    assert len(boxes) == 1
    b = boxes[0]
    assert b.s1 - b.s0 >= P.obstacle_min_length - 1e-9 and b.d1 - b.d0 >= P.obstacle_min_width - 1e-9
    tr = Tracker(P)
    tr.update(boxes, 0.0, -0.16)
    tr.update(boxes, 0.1, -0.16)
    assert len(tr.confirmed()) == 1
    tr.update([], 5.0, b.s1 + P.pass_margin + 0.01)   # 차량 뒤끝이 장애물 뒤끝을 지남
    assert tr.passed == 1 and not tr.tracks


def test_tracker_keeps_separate_consecutive_cars():
    tr = Tracker(P)
    a, b = Box(1.0, 1.322, -0.11, 0.11), Box(1.47, 1.792, -0.11, 0.11)  # 0.47 m 간격 연속 차량
    for t in (0.0, 0.1):
        tr.update([a, b], t, -1.0)
    assert len(tr.confirmed()) == 2


def test_visibility_grid_marks_only_traversed():
    g = VisibilityGrid(P, -1, 5)
    fr = RoadFrame(0, 0, 0)
    ang = np.linspace(-0.3, 0.3, 200)
    r = np.full(len(ang), 1.0)  # 1 m 앞 벽
    g.update(r, ang[0], ang[1] - ang[0], (0.0, 0.0, 0.0), fr)
    assert g.is_seen(np.array([0.5]), np.array([0.0]))[0]
    assert not g.is_seen(np.array([1.5]), np.array([0.0]))[0]   # 벽 뒤는 미관측
    assert not g.is_seen(np.array([0.5]), np.array([0.6]))[0]   # 시야 밖


def test_pd_controller_converges_on_straight_path():
    pl = Planner(P, BND)
    path = pl.plan((0.0, 0.0, 0.0, 0.0), [])
    c = PathTracker(P)
    s, d, psi, dt = 0.0, 0.08, 0.1, 0.04
    for _ in range(150):
        v, delta, info = c.step(path, s, d, psi, dt, P.v_max, None)
        s += v * math.cos(psi) * dt
        d += v * math.sin(psi) * dt
        psi += v * math.tan(delta) / P.wheelbase * dt
        if s > path.end_s - 0.3:
            break
    assert abs(d) < 0.01 and abs(psi) < 0.03
    assert abs(delta) <= P.max_steering + 1e-9


def test_config_yaml_matches_params():
    cfg = os.path.join(os.path.dirname(__file__), '..', 'config', 'objavoid_v3.yaml')
    y = yaml.safe_load(open(cfg))['objavoid_v3_node']['ros__parameters']
    # Runtime YAML deliberately tunes these three values; C++ separately checks
    # the actual Python defaults. Keep all other parameters synchronized.
    expected = dict(param_items())
    expected.update(max_steering=0.5, margin_obstacle=0.03, start_obs_y=0.35)
    for k, v in expected.items():
        assert k in y, k
        assert y[k] == pytest.approx(v) if not isinstance(v, bool) else y[k] == v, k


def test_reverse_escape_when_blocked():
    """앞이 도로 폭 전체로 막히면 경로 끝에서 서고, reverse_wait 뒤 후진(음수 속도)해 reverse_dist 물러난 뒤 멈춘다."""
    from autorace_objavoid_v3.mission import REVERSE, STOP, ObjAvoidCore
    p = Params()
    p.use_lane = False
    p.reverse_enable = True
    p.reverse_max = 1
    core = ObjAvoidCore(p)
    ang = np.linspace(-1.9, 1.9, 400)
    wall_x = 0.9  # 라이다 기준 앞 벽(도로 폭 전체)

    def scan(x):
        return np.array([(wall_x - x) / math.cos(a) if abs((wall_x - x) * math.tan(a)) < 1.0 and abs(a) < 1.4
                         else 8.0 for a in ang])
    x, t, v = 0.0, 0.0, 0.0
    states, xs = [], []
    while t < 12.0:
        core.on_odom(t, x, 0.0, 0.0, v, 0.0)
        if t == 0.0:
            assert core.activate(t)
        if abs(t * 10 - round(t * 10)) < 1e-6:
            core.process_scan(t, scan(x), ang[0], ang[1] - ang[0], t)
        out = core.control(t, 0.04)
        v = out[0] if out is not None else 0.0
        x += v * 0.04
        states.append(core.state)
        xs.append(x)
        t = round(t + 0.04, 6)
    i = states.index(REVERSE)
    assert max(xs[:i]) + P.body_front < wall_x + p.lidar_x  # 벽에 닿기 전에 섬
    assert max(xs[:i]) - min(xs[i:]) == pytest.approx(p.reverse_dist, abs=0.03)  # 그 자리에서 reverse_dist 후진
    assert core.reverses == 1 and core.state == STOP  # reverse_max=1: 다시 막혀도 더 후진하지 않음
