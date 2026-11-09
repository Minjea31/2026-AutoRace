from pathlib import Path
from dataclasses import fields,replace
import json,math
import pytest,yaml
from autorace_parking.core import Settings,Scan,CollisionChecker
from autorace_parking.control import build_plan_report
from autorace_parking.obstacle_memory import MemoryScan
from autorace_parking.regions import with_occupied_slots
from autorace_parking.tracking import compose,inverse


def clear():return Scan([math.inf]*361,-math.pi,math.pi/180,.01,12.)


def profile():
    values=yaml.safe_load((Path(__file__).resolve().parents[1]/'config/parking_real.yaml').read_text())['parking_node']['ros__parameters']
    return Settings(**{f.name:values[f.name] for f in fields(Settings) if f.name in values})


def test_default_and_real_profile_use_obstacles_without_whole_cell_exclusions():
    assert not Settings().block_occupied_slots
    assert not profile().block_occupied_slots


@pytest.mark.parametrize('yaw',[0.,math.pi/2])
def test_stale_region_metadata_is_ignored_but_real_points_and_body_margin_still_block(yaw):
    cfg=Settings(assume_unobserved_free=True,clearance=.04)
    scan=clear();scan.occupied_slots=[{'id':3,'pose':(0.,0.,0.),'half_size':(.26,.31)}]
    assert CollisionChecker(scan,cfg).free((0.,0.,yaw))
    scan.points=[(.19*math.cos(yaw),.19*math.sin(yaw))]
    checker=CollisionChecker(scan,cfg)
    assert not checker.free((0.,0.,yaw)) and checker.last_block=='obstacle'
    assert CollisionChecker(scan,replace(cfg,clearance=0.)).free((0.,0.,yaw))


def test_manual_bag_start_now_generates_a_complete_path_around_observed_obstacles():
    cfg=profile();data=json.loads((Path(__file__).parent/'data/manual_start_20261108.json').read_text());pose=tuple(data['pose'])
    scan=MemoryScan.__new__(MemoryScan);scan.__dict__=clear().__dict__.copy();scan.points=[];scan.point_owners={}
    for x,y,owner in data['memory_cells']:
        p=compose(inverse(pose),(x,y,0.))[:2];scan.points.append(p);scan.point_owners[p]=owner
    old=build_plan_report(scan,replace(cfg,block_occupied_slots=True),origin=pose,recovery_allowed=False,layout_hint=data['slot_map'])
    assert old['diagnostics']['reason_code']=='start_blocked' and not old['path']
    result=build_plan_report(scan,cfg,origin=pose,recovery_allowed=False,layout_hint=data['slot_map'])
    assert result['final'] and result['diagnostics']['reason_code']=='path_ready'
    assert result['goal'].slot_id==4
    checker=CollisionChecker(scan,cfg)
    assert all(checker.free(p) for p in result['path'])
    scan.points.append(tuple(result['path'][len(result['path'])//2][:2]))
    assert not CollisionChecker(scan,cfg).free(result['path'][len(result['path'])//2])


def test_remembered_obstacle_behind_vehicle_remains_a_collision():
    cfg=profile();scan=clear();scan.points=[(-.28,0.)]
    scan.occupied_slots=[{'id':3,'pose':(0.,0.,0.),'half_size':(.26,.31)}]
    checker=CollisionChecker(scan,cfg)
    assert checker.free((0.,0.,0.))
    assert not checker.free((-.1,0.,0.)) and checker.last_block=='obstacle'
