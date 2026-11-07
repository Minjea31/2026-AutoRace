from dataclasses import replace
import math

import pytest

from autorace_parking.core import Settings,Scan,Cluster,find_gaps,cluster_points,CollisionChecker
from autorace_parking.control import build_plan_report
from autorace_parking.obstacle_memory import MemoryScan,ObstacleMemory


def fragment(x):
    return [(x+dx,-.5+dy) for dx in [0.,.05,.1] for dy in [0.,.05,.1]]


def remembered(owners):
    raw=Scan([0.]*361,-math.pi,math.pi/180,.01,12.,(0.,0.,0.))
    raw.points=fragment(.5)+fragment(1.)
    result=MemoryScan.__new__(MemoryScan);result.__dict__=raw.__dict__.copy()
    result.point_owners={p:owner for points,owner in zip([fragment(.5),fragment(1.)],owners) for p in points}
    return result


def test_two_fragments_of_one_track_cannot_form_a_parking_pair():
    cfg=replace(Settings(clearance=.04),max_gap=.7,assume_unobserved_free=True)
    s=remembered([7,7])
    assert len(find_gaps(cluster_points(s.points),cfg))==1  # The previous false pair.
    report=build_plan_report(s,cfg)
    assert report['diagnostics']['candidates']==0 and not report['path']
    assert len(s.candidate_clusters(cfg))==1


def test_unowned_accumulated_points_are_not_candidate_objects_but_remain_obstacles():
    cfg=replace(Settings(clearance=.04),max_gap=.7,assume_unobserved_free=True)
    s=remembered([7,None])
    assert build_plan_report(s,cfg)['diagnostics']['candidates']==0
    assert not CollisionChecker(s,cfg).free((1.05,-.45,0.))


def test_distinct_tracks_form_pair_and_report_two_different_ids():
    cfg=replace(Settings(clearance=.04),max_gap=.7,assume_unobserved_free=True)
    report=build_plan_report(remembered([7,8]),cfg)
    pair=report['diagnostics']['selected_pair']
    assert len(pair)==2 and {p['track_id'] for p in pair}=={7,8}
    assert report['diagnostics']['target']['x']==pytest.approx(.8)


def test_find_gaps_rejects_same_known_id_even_with_separate_input_clusters():
    cfg=replace(Settings(clearance=.04),max_gap=.7)
    assert not find_gaps([Cluster(fragment(.5),7),Cluster(fragment(1.),7)],cfg)


def test_drawn_object_bounds_contain_its_points_even_when_kf_center_lags():
    cfg=Settings();s=remembered([7,8]);m=ObstacleMemory()
    m.update(s,s,(0.,0.,0.),1.,cfg)
    for track in m.tracks.values():track.center=(10.,10.)
    boxes=m.boxes((0.,0.,0.))
    for tid,_,corners in boxes:
        points=[cell[:2] for cell in m.cells.values() if cell[2]==tid]
        xs,ys=zip(*corners)
        assert all(min(xs)-1e-9<=x<=max(xs)+1e-9 and min(ys)-1e-9<=y<=max(ys)+1e-9 for x,y in points)
