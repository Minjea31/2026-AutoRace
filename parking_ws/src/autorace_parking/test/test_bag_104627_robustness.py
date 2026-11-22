"""10 Hz acquisition age, narrow real objects, and fixed front-middle bay."""
from dataclasses import replace
from types import SimpleNamespace
import math
import json
from pathlib import Path
from dataclasses import fields
import yaml
import numpy as np
import pytest
import rclpy
from rclpy.parameter import Parameter
from autorace_parking.node import ParkingNode
from autorace_parking.core import Settings,Scan,Gap,cluster_points
from autorace_parking.control import Follower
from autorace_parking.obstacle_memory import ObstacleMemory
from autorace_parking.slot_map import fit_slot_map
from autorace_parking.targets import automatic_goals
from test_slot_map import scene,snapshot,cfg


@pytest.fixture
def search_node():
    rclpy.init()
    n=ParkingNode(parameter_overrides=[Parameter('localization_source',value='scan'),
        Parameter('search_enabled',value=True)])
    n.planning_info={'reason_code':'no_gap'}
    n.scan_time=100.;n.last_scan_received_stamp=100.;n.scan_sequence=1
    scan=Scan([3.]*181,-math.pi/2,math.pi/180,.01,3.)
    try:yield n,scan
    finally:n.destroy_node();rclpy.shutdown()


def test_search_does_not_pulse_at_the_normal_10hz_acquisition_age(search_node):
    n,scan=search_node
    assert n.search_command(100.19,(0.,0.,0.),scan)==(0.,0.)
    # Repeated control ticks on one observation cannot satisfy confirmation.
    assert n.search_command(100.21,(0.,0.,0.),scan)==(0.,0.)
    n.scan_time=100.1;n.last_scan_received_stamp=100.1;n.scan_sequence+=1
    assert n.search_command(100.29,(0.,0.,0.),scan)==(.1,0.)
    assert n.search_command(100.31,(0.,0.,0.),scan)==(.1,0.)
    assert n.search_command(100.36,(0.,0.,0.),scan)==(.1,0.)


def test_search_stops_for_real_input_loss_and_requires_new_clear_scans(search_node):
    n,scan=search_node;n.search_resume_scans=1
    assert n.search_command(100.21,(0.,0.,0.),scan)==(.1,0.)
    assert n.search_command(100.31,(0.,0.,0.),scan)==(0.,0.)
    n.search_resume_scans=2;n.last_scan_received_stamp=100.2;n.scan_time=100.1;n.scan_sequence+=1
    assert n.search_command(100.32,(0.,0.,0.),scan)==(0.,0.)
    n.scan_time=100.2;n.scan_sequence+=1
    assert n.search_command(100.33,(0.,0.,0.),scan)==(.1,0.)
    scan.points=[(.35,0.)];n.scan_sequence+=1
    assert n.search_command(100.34,(0.,0.,0.),scan)==(0.,0.)


def test_finished_mission_stays_finished_when_rc_is_used_after_release(search_node):
    n,_=search_node;n.activation.active=True;n.follower.exited=True
    n.manual_mode_guard=True;n.control_mode=3
    n.tick()
    assert n.state=='EXITED' and not n.manual_interrupted


def test_cached_final_entry_arc_inside_grid_cannot_be_reused_as_full_exit(search_node):
    from concurrent.futures import Future
    n,scan=search_node
    n.cfg=replace(n.cfg,validate_reverse_entry=True,assume_unobserved_free=True)
    grid={'origin':[1.,-.5],'yaw':0.,'width':1.,'depth':1.}
    n.follower.goal=Gap(1.,-.5,0.,.5,0,1,parking_grid=grid)
    n.follower.entry_path=[(.8,-.5,0.),(1.,-.5,0.)]
    calls=[];pending=Future();n.worker.shutdown(wait=True)
    n.worker=SimpleNamespace(submit=lambda *a:(calls.append(a)or pending),shutdown=lambda **k:None)
    assert n.queue_exit_plan([(0.,0.,0.)],(1.,-.5,0.),[(0.,0.,0.)],scan,n.cfg) is None
    assert len(calls)==1 and n.exit_pending is not None


def test_short_alignment_arc_cannot_end_inside_grid():
    from autorace_parking.departure import single_reverse_route
    from autorace_parking.regions import outside_grid
    config=replace(Settings(),assume_unobserved_free=True,exit_curve_handoff=True,
        exit_path_tracking_margin=.04,planning_timeout=1.)
    grid={'origin':[1.,-.4],'yaw':0.,'width':1.2,'depth':1.}
    scan=Scan([math.inf]*361,-math.pi,math.pi/180,.01,5.)
    pose=(.9,-.05,-.12)
    route,reason=single_reverse_route([(0.,0.,0.)],pose,[(.4,0.,0.),(-.3,0.,0.)],scan,config,grid)
    assert route and outside_grid(route[-1],grid,config),reason


def test_reverse_arrival_uses_current_progress_before_endpoint_rejection():
    f=Follower(replace(Settings(),assume_unobserved_free=True))
    f.parked=f.exiting=f.exit_route_ready=True;f.entry_trace=[(0.,0.,0.)]
    f.goal=Gap(-.09,0.,0.,.6,0,1);f.path=[(-.018*i,0.,0.)for i in range(6)]
    f.set_segment(0);f.last_drive_direction=-1;f.index=0
    scan=Scan([math.inf]*361,-math.pi,math.pi/180,.01,5.)
    assert f.command(scan,(-.07,0.,0.),True,now=1.)[2]=='EXIT_CONFIRMING'
    assert f.exit_route_reason!='endpoint_alignment'


def test_entry_does_not_hold_just_outside_arrival_tolerance_when_path_ends_short():
    config=replace(Settings(),assume_unobserved_free=True,require_complete_entry_path=True,
        continuous_replanning=True)
    f=Follower(config,speed=.2,require_parking_heading=False)
    goal=Gap(.03,0.,0.,.5,0,1,kind='enclosed_center',heading_options=(0.,),stop_tolerance=.015)
    f.accept(goal,[(0.,0.,0.),(.022,0.,0.)],True,(0.,0.,0.))
    scan=Scan([math.inf]*361,-math.pi,math.pi/180,.01,5.)
    v,a,state=f.command(scan,(.0146,0.,0.),True,now=1.)
    assert v>0 and state=='PARKING'
    assert f.goal.stop_tolerance==.015
    assert f.command(scan,(.016,0.,0.),True,now=1.1)==(0.,0.,'CONFIRMING')


@pytest.mark.parametrize('angle',[0.,.23,-.4])
def test_fixed_front_middle_cell_matches_five_occupied_cells(angle):
    config=cfg(fixed_parking_slot_id=4,enclosure_confirm_objects=4)
    objects,truth=scene((0,1,2,3,5),origin=(2.,-1.2),yaw=angle)
    layout=fit_slot_map(objects,config)
    assert layout and layout['locked'] and layout['matched_cells']==5
    empty=[c for c in layout['cells']if c['eligible_empty']]
    assert len(empty)==1 and empty[0]['id']==4
    assert empty[0]['center']==pytest.approx(truth[4],abs=1e-7)


def test_fixed_target_handles_one_occluded_object_and_preserves_locked_registration():
    config=cfg(fixed_parking_slot_id=4,enclosure_confirm_objects=4)
    objects,truth=scene((0,1,3,5),origin=(2.,-1.2),yaw=.1)
    layout=fit_slot_map(objects,config)
    assert layout and layout['locked']
    scan=snapshot(objects);details={}
    goals=automatic_goals(scan,scan.candidate_clusters(config),config,(0.,0.,0.),details)
    assert len(goals)==1 and goals[0].slot_id==4
    locked=fit_slot_map(objects[:2],config,layout)
    assert locked['locked'] and locked['origin']==layout['origin']


@pytest.mark.parametrize('indices',[(0,2),(3,5),(0,1,3)])
def test_insufficient_objects_cannot_create_fixed_slot_or_pair(indices):
    config=cfg(fixed_parking_slot_id=4,enclosure_confirm_objects=4)
    objects,_=scene(indices,origin=(2.,-1.2),yaw=.1);scan=snapshot(objects);details={}
    assert fit_slot_map(objects,config) is None
    assert not automatic_goals(scan,scan.candidate_clusters(config),config,(0.,0.,0.),details)


def test_fixed_goal_never_switches_to_another_empty_cell():
    config=cfg(fixed_parking_slot_id=4,enclosure_confirm_objects=4)
    objects,truth=scene((0,1,2,3,5),origin=(2.,-1.2),yaw=.1)
    layout=fit_slot_map(objects,config)
    occupied_target,_=scene((0,1,2,3,4),origin=(2.,-1.2),yaw=.1)
    scan=snapshot(occupied_target);details={}
    assert not automatic_goals(scan,scan.candidate_clusters(config),config,(0.,0.,0.),details,layout)


@pytest.mark.parametrize('increment',[math.pi/180,-math.pi/180])
def test_batch_memory_visibility_preserves_unknown_occluded_and_clear_rays(increment):
    rng=np.random.default_rng(104627)
    ranges=rng.uniform(.05,4.,361);ranges[::11]=np.nan;ranges[::17]=0.;ranges[::23]=np.inf
    scan=Scan(ranges,-math.pi if increment>0 else math.pi,increment,.01,5.,(.103,.02,.13))
    points=rng.uniform(-4.,4.,(2000,2))
    expected=[ObstacleMemory.measured_clear(scan,p)for p in points]
    assert ObstacleMemory.measured_clear_many(scan,points).tolist()==expected


@pytest.mark.parametrize('frame',range(6))
def test_recorded_narrow_sign_is_an_obstacle_before_it_grows_to_twelve_cm(frame):
    package=Path(__file__).resolve().parents[1]
    params=yaml.safe_load((package/'config/parking_real.yaml').read_text())['parking_node']['ros__parameters']
    config=Settings(**{f.name:params[f.name]for f in fields(Settings)if f.name in params})
    assert config.obstacle_min_cluster_size==0.
    data=json.loads((package/'test/data/parking_104627_front_obstacle.json').read_text())
    scan=Scan(**data['scans'][frame]);old=scan.filtered_obstacles(replace(config,obstacle_min_cluster_size=.12))
    current=scan.filtered_obstacles(config);before=set(old.points);after=set(current.points)
    witnesses=[g for g in cluster_points(scan.points,config.cluster_distance,config.cluster_min_points)
               if len(g.points)>=10 and .05<=max(g.bounds[1]-g.bounds[0],g.bounds[3]-g.bounds[2])<.12
               and all(p not in before and p in after for p in g.points)]
    assert witnesses
    assert scan.ranges==current.ranges


def test_fixed_known_empty_bay_does_not_wait_for_occluded_center_rays_and_can_gain_entry_margin():
    from recorded_profiles import fixed_empty_recording
    from autorace_parking.control import build_plan_report
    from autorace_parking.obstacle_memory import MemoryScan
    from autorace_parking.tracking import compose,inverse
    from autorace_parking.core import CollisionChecker
    package=Path(__file__).resolve().parents[1]
    params=yaml.safe_load((package/'config/parking_real.yaml').read_text())['parking_node']['ros__parameters']
    config=fixed_empty_recording(Settings(**{f.name:params[f.name]for f in fields(Settings)if f.name in params}))
    data=json.loads((package/'test/data/parking_104627_conservative_map.json').read_text())['frames'][0]
    pose=data['pose'];pairs=[(compose(inverse(pose),(x,y,0.))[:2],owner)for x,y,owner in data['memory_cells']]
    scan=MemoryScan.__new__(MemoryScan);scan.__dict__=Scan([0.]*361,-math.pi,math.pi/180,.01,3.).__dict__.copy()
    scan.points=[p for p,_ in pairs];scan.point_owners=dict(pairs)
    # Preserve this recording's 10 cm margin and 3 cm departure reserve.
    # The live point/box profile now has a single 5 cm margin.
    config=replace(config,row_yaw=-pose[2],clearance=.1,path_tracking_margin=.02,
        entry_departure_reserve=.03,recovery_front_clearance=.1,recovery_margin_escape_enabled=True)
    first=build_plan_report(scan,config,origin=pose,failed_plans=0)
    assert first['diagnostics']['fixed_empty_prior']
    assert first['diagnostics']['entry_reserve_blocked'] and not first['path']
    retry=build_plan_report(scan,config,origin=pose,failed_plans=3)
    assert retry['diagnostics']['reason_code']=='recovery_ready'
    assert retry['diagnostics']['slot_id']==4
    assert all(CollisionChecker(scan,config).free(p,allow_unknown=True)for p in retry['path'])
    unknown=build_plan_report(scan,replace(config,fixed_slot_empty_known=False),origin=pose)
    assert not unknown['path'] and not unknown['diagnostics'].get('fixed_empty_prior')
