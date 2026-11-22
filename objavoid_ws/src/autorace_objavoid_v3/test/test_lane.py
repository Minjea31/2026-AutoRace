"""차선(노란 실선) 경계·도로 정렬·시작 곡률 단위 테스트."""
import math

import numpy as np

from autorace_objavoid_v3.lane_boundary import LaneBoundary, fit_road, line_segments
from autorace_objavoid_v3.mission import RUN, ObjAvoidCore
from autorace_objavoid_v3.params import Params
from autorace_objavoid_v3.road import PriorBoundary

P = Params()
PRIOR = PriorBoundary(P.boundary_s, P.boundary_left, P.boundary_right)


def road_cells(rot=0.0, shift=0.0, funnel=True):
    """3차로 외곽선(±0.78) + 입구 깔때기 대각선, 도로 좌표를 rot/shift만큼 틀어서."""
    pts = []
    for s in np.arange(0.5, 2.5, 0.03):
        pts += [(s, 0.78), (s, -0.78)]
    if funnel:
        for s in np.arange(0.0, 0.5, 0.03):
            w = 0.27 + (0.78 - 0.27) * s / 0.5
            pts += [(s, w), (s, -w)]
    pts = np.array(pts)
    c, sn = math.cos(rot), math.sin(rot)
    return np.column_stack((c * pts[:, 0] - sn * pts[:, 1], sn * pts[:, 0] + c * pts[:, 1] + shift))


def test_ransac_splits_bent_line():
    segs = line_segments(road_cells(), 0.03)
    long = [g for g in segs if g[1] > 1.0]
    assert len(long) == 2 and all(abs(g[0]) < 0.01 for g in long)       # 외곽 직선 2개
    assert any(abs(abs(g[0]) - math.atan(0.51 / 0.5)) < 0.05 for g in segs)  # 깔때기 대각선은 따로


def test_fit_road_recovers_rotation_and_offset():
    a, center, _ = fit_road(road_cells(rot=0.2, shift=0.06), P, -0.5, 3.0, PRIOR)
    assert abs(a - 0.2) < 0.02
    assert center is not None and abs(center - 0.06) < 0.03


def test_fit_road_single_line_uses_prior_model():
    cells = road_cells(shift=0.05, funnel=False)
    cells = cells[cells[:, 1] < 0]                      # 오른쪽 선 하나만 보임
    a, center, _ = fit_road(cells, P, -0.5, 3.0, PRIOR)
    assert abs(a) < 0.01 and abs(center - 0.05) < 0.03


def test_boundary_uses_observed_line_and_marks_unknown():
    lb = LaneBoundary(PRIOR, P)
    cells = np.array([(s, 0.70) for s in np.arange(1.0, 2.0, 0.03)])  # 왼쪽 선이 사전보다 안쪽에 보임
    lb.rebuild(cells, P.section_end)
    r, l = lb.limits(np.array([1.5, 3.0]))
    assert abs(l[0] - (0.70 - P.lane_line_half)) < 1e-6          # 관측된 곳은 실제 선
    assert abs(l[1] - 0.768) < 1e-6                                # 안 본 곳은 사전 경계
    assert lb.known(np.array([1.5]), 1)[0] and not lb.known(np.array([1.5]), -1)[0]


def test_start_curvature_from_motion_and_clamped():
    core = ObjAvoidCore(Params())
    kmax = 1 / P.plan_min_radius
    core.ctrl.delta = 0.4                       # 자기 명령 조향이 커도
    st, _ = core._start_state((1.0, 0.0, 0.0, 0.3), w=0.0)
    assert abs(st[3]) < 1e-9                    # 실제로 직진 중이면 곡률 0
    st, _ = core._start_state((1.0, 0.0, 0.0, 0.3), w=3.0)   # 측정 곡률 10 1/m -> 한계로
    assert abs(st[3] - kmax) < 1e-9
    core.state = RUN
    st, _ = core._start_state((1.0, 0.0, 0.0, 0.0), w=0.0)   # 정지 중 + 직접 몰던 중: 명령 조향(한계 안)
    assert abs(st[3]) <= kmax + 1e-9



def test_deep_projection_needs_only_class_mask():
    from autorace_objavoid_v3.lane_boundary import GroundProjection
    cfg=dict(width=100,height=100,bev_top_y=.1,bev_bottom_y=.9,
             bev_top_left=.1,bev_top_right=.9,bev_bottom_left=.1,bev_bottom_right=.9,
             lane_width_m=.5,lane_width_px=50,bev_near_m=.2,bev_far_m=2.)
    gp=GroundProjection(cfg,max_range=2.,homography=[0,.02,.1,.02,0,-1,0,0,1])
    mask=np.ones((100,100),np.uint8);mask[:,50:]=2
    yellow=gp.yellow_points(mask);white=gp.white_points(mask)
    assert len(yellow)>0 and len(white)>0
    assert np.all(yellow[:,0]<=2.) and np.all(white[:,0]<=2.)
    mask[:]=0
    assert len(gp.yellow_points(mask))==0 and len(gp.white_points(mask))==0
