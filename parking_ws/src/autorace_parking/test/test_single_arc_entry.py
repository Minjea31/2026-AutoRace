"""A bag-derived entry should finish on one safe turn rather than a straight tail."""
from dataclasses import fields,replace
import json
import math
from pathlib import Path

import pytest
import yaml

from autorace_parking.core import Settings,Scan,Gap,CollisionChecker,plan_path,wrap
from autorace_parking.obstacle_memory import MemoryScan
from autorace_parking.tracking import compose,inverse


def scene():
    package=Path(__file__).resolve().parents[1]
    params=yaml.safe_load((package/'config/parking_real.yaml').read_text())['parking_node']['ros__parameters']
    cfg=Settings(**{f.name:params[f.name] for f in fields(Settings) if f.name in params})
    f=json.loads((package/'test/data/parking_entry_margin_20261110.json').read_text())
    tr=inverse(tuple(f['pose']))
    source=MemoryScan.__new__(MemoryScan)
    source.__dict__=Scan([math.inf]*361,-math.pi,math.pi/180,.01,12.).__dict__.copy()
    source.points=[compose(tr,(*p[:2],0.))[:2] for p in f['memory_cells']]
    goal=Gap(*compose(tr,tuple(f['goal'])),.5,0,1,kind='enclosed_center',confirmed=True,
             heading_options=tuple(wrap(a-f['pose'][2]) for a in f['heading_options']))
    cfg=replace(cfg,require_goal_heading=True,goal_yaw_tolerance=math.pi,goal_tolerance=.015,
                allow_reverse=False,clearance=cfg.clearance+cfg.path_tracking_margin)
    known=[compose(tr,tuple(p)) for p in f['known_poses']]
    return source,goal,cfg,known,tr[:2]


def test_real_bag_entry_is_straight_then_one_arc_ending_at_the_center():
    scan,goal,cfg,known,center=scene()
    assert cfg.prefer_arc_finish_entry
    path=plan_path(scan,goal,cfg,known,center)
    assert path and math.dist(path[-1][:2],(goal.x,goal.y))<1e-8
    checker=CollisionChecker(scan,cfg,known,center)
    assert all(checker.free(p) for p in path)
    turned=False
    for a,b in zip(path,path[1:]):
        ds=math.dist(a[:2],b[:2]);yaw=wrap(b[2]-a[2])
        if abs(yaw)>1e-8:
            turned=True
            curvature=2*math.sin(yaw/2)/ds
            assert curvature==pytest.approx(-1/cfg.preferred_turn_radius,abs=1e-8)
        else:
            assert not turned,'No straight suffix after the turn'
        assert (b[0]-a[0])*math.cos(a[2])+(b[1]-a[1])*math.sin(a[2])>0
    assert turned


def test_arc_preference_cannot_override_a_required_terminal_heading():
    scan,goal,cfg,known,center=scene();scan.points=[]
    goal=replace(goal,x=1.4,y=-.4,yaw=0.,heading_options=(0.,))
    cfg=replace(cfg,goal_yaw_tolerance=.05)
    path=plan_path(scan,goal,cfg,known,center)
    assert path and abs(wrap(path[-1][2]))<=cfg.goal_yaw_tolerance


def test_arc_preference_respects_the_entry_length_budget():
    scan,goal,cfg,known,center=scene()
    assert not plan_path(scan,goal,replace(cfg,max_entry_path_length=.5,planning_timeout=.03),known,center)


def test_blocked_single_arc_does_not_bypass_the_obstacle_checker():
    scan,goal,cfg,known,center=scene()
    scan.points.extend((.7,-3.+i*.02) for i in range(301))
    assert not plan_path(scan,goal,replace(cfg,planning_timeout=.03),known,center)
