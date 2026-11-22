"""Differential checks against the preserved Python reference implementation."""
import math
import random
import pytest
import cone_native as cpp
from cone_lane_follower.node import RoiNode as py, CENTER_DEFAULTS as CP, BOUNDARY_DEFAULTS as BP
from cone_lane_follower.path_memory import PathMemory as PyMemory
from cone_lane_follower.corner_offset import offset, curvature


def close(a,b):
    if a is None or b is None:
        assert a is None and b is None
    elif isinstance(a,(bool,str)):
        assert a == b
    elif isinstance(a,(int,float)):
        assert a == pytest.approx(b,rel=1e-8,abs=2e-9)
    else:
        assert len(a)==len(b)
        for x,y in zip(a,b):close(x,y)


def line(n,y=0,dx=.05):return [(i*dx,y,0.) for i in range(n)]


@pytest.mark.parametrize('seed',range(60))
def test_boundary_and_control(seed):
    rng=random.Random(seed)
    points=[(i*.2+rng.uniform(-.03,.03),side*.3+rng.uniform(-.03,.03),0.) for i in range(12) for side in (-1,1)]
    rng.shuffle(points)
    bp=dict(BP,boundary_connect_distance=.35,boundary_max_turn=100.)
    close(py.select_boundaries(points,bp),cpp.select_boundaries(points,bp))
    close(py.select_boundary_chain(points,bp),cpp.select_boundary_chain(points,bp))
    for ref in (.2,.25,.28,.3):
        path=[(i*.05,rng.uniform(-.1,.1),0) for i in range(20)]
        close(py.path_control_geometry(path,ref,.5),cpp.path_control_geometry(path,ref,.5))


@pytest.mark.parametrize('seed',range(50))
def test_paths(seed):
    rng=random.Random(seed)
    theta=rng.uniform(-.6,.6)
    left=[(i*.1,.3+theta*(i*.1)**2,0.) for i in range(rng.randrange(3,18))]
    right=[(i*.1,-.3+theta*(i*.1)**2,0.) for i in range(rng.randrange(3,18))]
    for path in (left,right):
        close(py.sample_boundary(path,.05),cpp.sample_boundary(path,.05))
        close(py.offset_boundary(path,.3,.05),cpp.offset_boundary(path,.3,.05))
    pairs=py.match_cross_sections(left,right,.05,.1,1.5)
    close(pairs,cpp.match_cross_sections(left,right,.05,.1,1.5))
    close(py.match_projected_sections(left,right,.05,.1,1.5),cpp.match_projected_sections(left,right,.05,.1,1.5))
    centers=[tuple((a[k]+b[k])/2 for k in range(3)) for a,b in pairs]
    measured=py.smooth_center_path(centers,left,right,.25,.3,75)
    close(measured,cpp.smooth_center_path(centers,left,right,.25,.3,75))
    for ll,rr,pp,mm in ((left,right,pairs,measured),(left,[],[],[]),([],right,[],[])):
        close(py.unified_path(ll,rr,pp,mm,.6,CP),cpp.unified_path(ll,rr,pp,mm,.6,CP))


@pytest.mark.parametrize('seed',range(20))
def test_memory(seed):
    rng=random.Random(seed)
    a,b=PyMemory(),cpp.PathMemory()
    valid=lambda p:py.valid_center_geometry(p,CP)
    for i in range(20):
        p=line(rng.randrange(3,40),rng.uniform(-.015,.015))
        close(a.refresh(p,valid),b.refresh(p,valid));close(a.points,b.points)
        close(a.update_reason,b.update_reason)
        local=[(x-i*.01,y,z) for x,y,z in a.points]
        close(a.remaining(local,.08,.5),b.remaining(local,.08,.5))
        close(a.index,b.index);close(a.finished,b.finished)


@pytest.mark.parametrize('mirror',[-1,1])
def test_corner(mirror):
    boundary=[(0.,0.,0.),(1.,0.,0.),(1.,mirror*1.,0.),(2.,mirror*1.,0.)]
    close(offset(boundary,mirror*.3,.05,.263,py,CP,{}),cpp.corner_offset(boundary,mirror*.3,.05,.263,CP))
    close(offset(boundary,mirror*.3,.05,10.,py,CP,{}),cpp.corner_offset(boundary,mirror*.3,.05,10.,CP))


def test_cluster_filters():
    rng=random.Random(43)
    for _ in range(100):
        points=[(rng.random(),rng.random(),0.) for i in range(30)]
        indices=sorted(rng.sample(range(60),30))
        # These methods use parameters only; supply the same minimal access shim.
        class Param:
            def __init__(self,x):self.value=x
        class Node:
            def get_parameter(self,k):return Param(dict(cluster_threshold=.1,min_cluster_points=2,max_cluster_width=.28)[k])
        c=py.cluster_points(Node(),points,indices)
        close(c,cpp.cluster_points(points,indices,.1))
        close(py.filter_clusters(Node(),c),cpp.filter_clusters(c,2,.28))
