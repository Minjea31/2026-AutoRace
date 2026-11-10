"""A fixed scene from the 04:01 bag must retain hard safety and roomier entry."""
from dataclasses import fields,replace
import json
import math
from pathlib import Path

import yaml

from autorace_parking.core import Settings,Scan,Gap,CollisionChecker,plan_path,wrap
from autorace_parking.obstacle_memory import MemoryScan
from autorace_parking.tracking import compose,inverse


def test_recorded_entry_chooses_more_body_clearance_with_little_added_distance():
    package=Path(__file__).resolve().parents[1]
    params=yaml.safe_load((package/'config/parking_real.yaml').read_text())['parking_node']['ros__parameters']
    cfg=Settings(**{f.name:params[f.name] for f in fields(Settings) if f.name in params})
    cfg=replace(cfg,prefer_arc_finish_entry=False,preferred_turn_radius=0.,max_planning_turn_radius=.58)
    frame=json.loads((package/'test/data/parking_entry_margin_20261110.json').read_text())
    pose=tuple(frame['pose']);transform=inverse(pose)
    scan=MemoryScan.__new__(MemoryScan)
    scan.__dict__=Scan([math.inf]*361,-math.pi,math.pi/180,.01,12.,(.103,0.,0.)).__dict__.copy()
    scan.points=[compose(transform,(*cell[:2],0.))[:2] for cell in frame['memory_cells']]
    scan.point_owners={p:cell[2] for p,cell in zip(scan.points,frame['memory_cells'])}
    goal=Gap(*compose(transform,tuple(frame['goal'])),.5,0,1,kind='enclosed_center',confirmed=True,
             stop_tolerance=.03,heading_options=tuple(wrap(a-pose[2]) for a in frame['heading_options']))
    cfg=replace(cfg,require_goal_heading=True,allow_reverse=False,clearance=cfg.clearance+cfg.path_tracking_margin,
                goal_tolerance=.015,goal_yaw_tolerance=math.pi,resolution=.04,yaw_resolution=cfg.target_heading_step)
    known=[compose(transform,tuple(p)) for p in frame['known_poses']]
    paths=[plan_path(scan,goal,replace(cfg,path_clearance_weight=weight),known,transform[:2]) for weight in (0.,cfg.path_clearance_weight)]
    checker=CollisionChecker(scan,cfg,known,transform[:2])
    assert all(path and all(checker.free(p) for p in path) for path in paths)
    def gap(path):
        closest=math.inf
        for x,y,yaw in path:
            c,s=math.cos(yaw),math.sin(yaw)
            for px,py in scan.points:
                dx,dy=px-x,py-y
                distance=math.hypot(max(abs(c*dx+s*dy)-cfg.vehicle_length/2,0.),
                                    max(abs(-s*dx+c*dy)-cfg.vehicle_width/2,0.))
                closest=min(closest,distance)
        return closest
    lengths=[sum(math.dist(a[:2],b[:2]) for a,b in zip(path,path[1:])) for path in paths]
    assert gap(paths[1])>=gap(paths[0])+.05
    assert lengths[1]<=lengths[0]+.03
