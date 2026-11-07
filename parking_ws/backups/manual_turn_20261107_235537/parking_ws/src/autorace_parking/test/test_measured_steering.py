"""Measured weak steering response must affect both planning and control."""
from dataclasses import fields,replace
import json
import math
from pathlib import Path

import pytest
import yaml

from autorace_parking.core import Settings,Gap,Scan,CollisionChecker,advance,wrap
from autorace_parking.control import Follower,build_plan_report,needs_reverse_for_goal
from autorace_parking.obstacle_memory import MemoryScan
from autorace_parking.enclosed import enclosed_goals
from autorace_parking.node import validate_settings


def profile():
    params=yaml.safe_load((Path(__file__).resolve().parents[1]/'config/parking_real.yaml').read_text())['parking_node']['ros__parameters']
    return Settings(**{f.name:params[f.name] for f in fields(Settings) if f.name in params})


def clear_scan():return Scan([math.inf]*361,-math.pi,math.pi/180,.01,12.,(0.,0.,0.))


def test_measured_radius_does_not_reduce_the_hardware_steering_command_twice():
    cfg=profile();ideal=replace(cfg,steering_response_scale=1.)
    assert 1.0<cfg.turn_radius<1.15
    assert cfg.steering_limit==pytest.approx(ideal.steering_limit)
    assert cfg.steering_response_scale*math.tan(cfg.steering_limit)/cfg.wheelbase==pytest.approx(1/cfg.turn_radius)


@pytest.mark.parametrize('value',[0.,-.1,math.nan,math.inf,2.1])
def test_invalid_response_cannot_be_applied(value):
    with pytest.raises(ValueError):validate_settings(replace(profile(),steering_response_scale=value))


def test_larger_steering_gain_is_immediate_and_stays_within_command_limit():
    cfg=profile();scan=clear_scan();path=[advance((0.,0.,0.),i*.02,-.3) for i in range(101)]
    angles=[]
    for gain in (1.,1.5):
        follower=Follower(replace(cfg,steering_gain=gain),speed=.25,max_steering_rate=0.)
        follower.accept(Gap(*path[-1],.6,0,1),path,True,(0.,0.,0.))
        v,a,state=follower.command(scan,(0.,.005,0.),now=0.)
        assert v>0 and abs(a)<=cfg.steering_limit
        angles.append(a)
    assert abs(angles[1])>1.1*abs(angles[0])
    follower=Follower(cfg,speed=.25,max_steering_rate=0.)
    tight=[advance((0.,0.,0.),i*.02,-1/cfg.turn_radius) for i in range(101)]
    follower.accept(Gap(*tight[-1],.6,0,1),tight,True,(0.,0.,0.))
    assert follower.command(scan,(0.,0.,0.),now=0.)[1]==pytest.approx(-cfg.steering_limit)


def recorded_scan(index):
    data=json.loads((Path(__file__).resolve().parent/'data/parking_steering_bag_20261107.json').read_text())
    scan=MemoryScan.__new__(MemoryScan);scan.__dict__=clear_scan().__dict__.copy()
    cells=data['frames'][index]['memory_cells'];scan.points=[(x,y) for x,y,_ in cells]
    scan.point_owners={(x,y):tid for x,y,tid in cells}
    return scan


@pytest.mark.parametrize('index',[0,1])
def test_reachable_center_beats_an_unreachable_partial_approach_in_recorded_bay(index):
    # Optional legacy policy remains available; the real profile now uses fixed geometric centers.
    cfg=replace(profile(),automatic_target_selection=False,slot_map_enabled=False);scan=recorded_scan(index)
    candidates=enclosed_goals(scan,scan.candidate_clusters(cfg),cfg,(0.,0.,0.),{})
    assert len(candidates)>=2
    report=build_plan_report(scan,cfg)
    assert report['final'] and report['diagnostics']['reason_code']=='path_ready'
    assert report['goal'].x>2.05
    assert report['goal'].support_ids==candidates[0].support_ids
    assert {p['track_id'] for p in report['diagnostics']['selected_pair']}==set(report['goal'].support_ids)
    assert all(CollisionChecker(scan,cfg).free(p) for p in report['path'])
    for a,b in zip(report['path'],report['path'][1:]):
        distance=math.dist(a[:2],b[:2])
        if distance>1e-6:assert 2*abs(math.sin(wrap(b[2]-a[2])/2))/distance<=1/cfg.turn_radius+1e-6


def test_same_bay_goal_can_relocate_when_the_old_center_is_unreachable():
    cfg=profile();scan=recorded_scan(0)
    old=enclosed_goals(scan,scan.candidate_clusters(cfg),cfg,(0.,0.,0.),{})[0]
    report=build_plan_report(scan,cfg,old)
    assert report['final'] and report['diagnostics']['goal_relocated_for_reachability']
    assert report['goal'].confirmed and report['goal'].support_ids==old.support_ids
    assert report['goal'].x>old.x+.3


def test_forward_goal_inside_measured_turning_circle_can_request_bounded_recovery():
    cfg=profile()
    assert needs_reverse_for_goal(Gap(.46,-.25,0.,.6,0,1),cfg)
    assert not needs_reverse_for_goal(Gap(2.,-.8,0.,.6,0,1),cfg)


@pytest.mark.parametrize('blind_budget',[0.,.3])
def test_stalled_confirmed_route_uses_only_the_operator_cleared_reverse_budget(blind_budget):
    from unittest.mock import patch
    cfg=replace(profile(),initial_clear_radius=0.,recovery_uses_memory_map=False)
    scan=Scan([math.inf]*221,math.radians(-110),math.pi/180,.01,12.,(0.,0.,0.))
    goal=Gap(1.2,-.4,0.,.6,0,1,support_ids=(1,2,3,4,5),confirmed=True)
    with patch('autorace_parking.control.plan_path',return_value=[]),patch('autorace_parking.control.parking_candidates',return_value=[]):
        report=build_plan_report(scan,cfg,goal,failed_plans=3,stalled_route=True,blind_reverse_remaining=blind_budget)
    assert bool(report['path'])==bool(blind_budget)
    if blind_budget:
        assert report['diagnostics']['recovery_trigger']=='path_failure'
        assert report['diagnostics']['blind_reverse_distance_m']<=blind_budget+1e-8
        assert report['path'][-1][0]==pytest.approx(-.3)
    scan.points=[(-.25,0.)]
    with patch('autorace_parking.control.plan_path',return_value=[]),patch('autorace_parking.control.parking_candidates',return_value=[]):
        assert not build_plan_report(scan,cfg,goal,failed_plans=3,stalled_route=True,blind_reverse_remaining=blind_budget)['path']


def test_final_validated_goal_is_stable_during_arrival():
    from unittest.mock import patch
    cfg=replace(Settings(),continuous_replanning=True,enclosed_space_detection=True,require_goal_heading=False)
    goal=Gap(.11,0.,0.,.6,0,1,support_ids=(1,2,3,4,5),confirmed=True)
    candidate=replace(goal,x=.14,y=.02)
    with patch('autorace_parking.control.enclosed_goals',return_value=[candidate]):
        report=build_plan_report(clear_scan(),cfg,goal)
    assert report['diagnostics']['target']['x']==pytest.approx(goal.x)
    assert report['diagnostics']['target']['y']==pytest.approx(goal.y)


def test_partial_prefix_cannot_take_control_when_complete_entry_is_required():
    from unittest.mock import patch
    cfg=replace(profile(),allow_reverse=False)
    target=Gap(1.,-.4,0.,.6,0,1)
    prefix=[(0.,0.,0.),(.2,0.,0.),(.4,0.,0.)]
    with patch('autorace_parking.control.parking_candidates',return_value=[(1.,[(1.,0.)])]),\
         patch('autorace_parking.control.sample_candidate',return_value=(prefix,False)),\
         patch('autorace_parking.control.plan_path',return_value=[]):
        report=build_plan_report(clear_scan(),cfg,target)
    assert not report['path'] and report['diagnostics']['partial_path_rejected']


@pytest.mark.parametrize('obstacle',[False,True])
def test_same_goal_retains_a_valid_route_but_new_obstacle_allows_replacement(obstacle):
    import rclpy
    from concurrent.futures import Future
    from rclpy.parameter import Parameter
    from autorace_parking.node import ParkingNode
    rclpy.init()
    n=ParkingNode(parameter_overrides=[Parameter('localization_source',value='scan'),
        Parameter('continuous_replanning',value=True),Parameter('require_complete_entry_path',value=True),
        Parameter('assume_unobserved_free',value=True)])
    try:
        n.activation.active=True;n.now_seconds=lambda:20.;n.scan=clear_scan();n.scan_time=20.;n.failed_plans=2
        goal=Gap(1.,0.,0.,.6,0,1,support_ids=(1,2,3,4,5),confirmed=True)
        n.follower.accept(goal,[(0.,0.,0.),(.5,0.,0.),(1.,0.,0.)],True,(0.,0.,0.));old=n.follower.path
        if obstacle:n.scan.points=[(.6,0.)]
        future=Future();future.set_result({'goal':goal,'path':[(0.,0.,0.),(.5,.4,.8),(1.,0.,0.)],'final':True,
            'diagnostics':{'target':{'x':1.,'y':0.,'yaw':0.},'goal_confirmed':True,'support_ids':goal.support_ids,'reason_code':'path_ready'}})
        n.pending=future,n.epoch,(0.,0.,0.),20.;n.poll_plan()
        assert (n.follower.path is old)==(not obstacle)
        assert bool(n.planning_info.get('retained_valid_path'))==(not obstacle)
        assert n.failed_plans==0
    finally:n.destroy_node();rclpy.shutdown()


def test_goal_refinement_considers_later_candidates_that_improve_clearance():
    from autorace_parking.control import refine_goal
    cfg=Settings();scan=clear_scan();scan.points=[(1.,.4)]
    goal=Gap(1.,0.,0.,.6,0,1,support_ids=(1,2,3,4,5),confirmed=True)
    updated,changed=refine_goal(goal,[replace(goal,y=.01),replace(goal,y=-.05)],cfg,CollisionChecker(scan,cfg))
    assert changed and updated.y<0 and math.dist((updated.x,updated.y),(goal.x,goal.y))<=cfg.goal_update_step


def test_actual_command_preview_catches_corner_cutting_outside_the_nominal_route():
    results=[]
    for enabled in (False,True):
        cfg=replace(profile(),require_complete_entry_path=enabled)
        follower=Follower(cfg,speed=.25,max_steering_rate=0.)
        path=[(i*.02,0.,0.) for i in range(51)]
        follower.accept(Gap(1.,0.,0.,.6,0,1),path,True,(0.,0.,0.))
        scan=clear_scan();scan.points=[(.23,-.172)]
        result=follower.command(scan,(0.,.02,0.),now=0.,obstacle_scan=scan)
        results.append(result)
        assert follower.path
    assert results[0][0]>0
    assert results[1]==(0.,0.,'ROUTE_BLOCKED')


def test_terminal_braking_reaches_arrival_without_previewing_past_the_stop():
    cfg=profile();scan=clear_scan();scan.points=[(.511,0.)]
    follower=Follower(cfg,speed=.25,lookahead=.25,parking_position_tolerance=.07,
                      require_parking_heading=False,min_turn_speed=.2,max_steering_rate=0.)
    follower.accept(Gap(.3,0.,0.,.6,0,1),[(i*.01,0.,0.) for i in range(31)],True,(0.,0.,0.))
    pose=(0.,0.,0.);velocity=0.;slow=False
    for i in range(300):
        # Keep the same world obstacle in the current body frame.
        scan.points=[(.511-pose[0],0.)]
        v,a,state=follower.command(scan,pose,new_scan=i%2==0,now=i*.05,obstacle_scan=scan)
        if 0<v<.2:slow=True
        velocity+=(v-velocity)*(.05/.12)
        pose=advance(pose,velocity*.05,0.)
        if follower.parked:break
    assert follower.parked and slow and 0<=.3-pose[0]<=.07


def test_steering_can_change_sign_at_full_command_in_one_control_cycle():
    cfg=profile();follower=Follower(cfg,speed=.25,lookahead=.25,max_steering_rate=0.)
    follower.accept(Gap(1.,0.,0.,.6,0,1),[(i*.01,0.,0.) for i in range(101)],True,(0.,0.,0.))
    left=follower.command(clear_scan(),(0.,.03,0.),now=0.)[1]
    right=follower.command(clear_scan(),(0.,-.03,0.),now=.05)[1]
    assert left==pytest.approx(-cfg.steering_limit)
    assert right==pytest.approx(cfg.steering_limit)
