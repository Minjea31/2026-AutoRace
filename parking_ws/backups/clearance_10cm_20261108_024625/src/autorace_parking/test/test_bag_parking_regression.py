"""Failures found in the November 7 platform bag and bounded recovery contracts."""
from concurrent.futures import Future
from dataclasses import fields,replace
import json
import math
from pathlib import Path
from unittest.mock import patch

import pytest
import yaml

from autorace_parking.core import Settings,Gap,Scan,CollisionChecker,plan_path
from autorace_parking.control import Follower,build_plan_report,refine_goal
from autorace_parking.enclosed import enclosed_goals
from autorace_parking.obstacle_memory import MemoryScan


def clear_scan(front_only=False):
    return Scan([math.inf]*(181 if front_only else 361),-math.pi/2 if front_only else -math.pi,
                math.pi/180,.01,12.,(0.,0.,0.))


@pytest.mark.parametrize('stamp',[12.,27.,36.,43.])
def test_recorded_objects_select_the_actual_parking_interior(stamp):
    package=Path(__file__).resolve().parents[1]
    params=yaml.safe_load((package/'config/parking_real.yaml').read_text())['parking_node']['ros__parameters']
    cfg=Settings(**{f.name:params[f.name] for f in fields(Settings) if f.name in params})
    frame=next(f for f in json.loads((package/'test/data/parking_enclosure_bag_20261107.json').read_text())['frames'] if f['t']==stamp)
    scan=MemoryScan.__new__(MemoryScan);scan.__dict__=clear_scan().__dict__.copy()
    scan.points=[(x,y) for x,y,_ in frame['memory_cells']]
    scan.point_owners={(x,y):owner for x,y,owner in frame['memory_cells']}
    goals=enclosed_goals(scan,scan.candidate_clusters(cfg),cfg,(0.,0.,0.),{})
    assert goals and goals[0].confirmed
    assert 1.7<goals[0].x<2.3 and -.85<goals[0].y<-.45
    assert {1,2,3,5,13}.issubset(goals[0].support_ids)
    assert goals[0].width<=cfg.max_gap


def test_confirmed_goal_does_not_jump_to_another_bay_and_keeps_heading():
    cfg=replace(Settings(),continuous_replanning=True)
    goal=Gap(1.,-.6,-math.pi/2,.5,0,1,support_ids=(1,2,3,4,5),confirmed=True)
    other=replace(goal,x=1.05,support_ids=(6,7,8,9,10))
    scan=clear_scan();checker=CollisionChecker(scan,cfg)
    assert refine_goal(goal,[other],cfg,checker)==(goal,False)
    candidate=replace(goal,x=1.2,y=-.64)
    updated,changed=refine_goal(goal,[candidate],cfg,checker)
    assert changed and updated.confirmed and updated.support_ids==goal.support_ids
    assert 0<math.dist((updated.x,updated.y),(goal.x,goal.y))<=cfg.goal_update_step


def test_new_observation_cannot_freeze_goal_inside_an_obstacle():
    cfg=replace(Settings(),continuous_replanning=True)
    goal=Gap(1.,0.,0.,.5,0,1,support_ids=(1,2,3,4,5),confirmed=True)
    candidate=replace(goal,x=1.06)
    scan=clear_scan();scan.points=[(.82,0.)];checker=CollisionChecker(scan,cfg)
    assert not checker.free((goal.x,goal.y,goal.yaw))
    updated,changed=refine_goal(goal,[candidate],cfg,checker)
    assert changed and checker.free((updated.x,updated.y,updated.yaw))
    assert cfg.goal_update_gain*.06<updated.x-goal.x<=cfg.goal_update_step


def recovery_report(scan=None,failed=3,goal=None,known=()):
    cfg=replace(Settings(),allow_reverse=True,recovery_on_front_obstacle_only=True,
                recovery_on_path_failure=True,recovery_failure_count=3,recovery_reverse_distance=.30,
                assume_unobserved_free=True,require_goal_heading=False,initial_clear_radius=0.)
    with patch('autorace_parking.control.parking_candidates',return_value=[]),patch('autorace_parking.control.plan_path',return_value=[]):
        return build_plan_report(scan or clear_scan(),cfg,goal or Gap(-.1,-.5,-math.pi/2,.6,0,1),
                                 failed_plans=failed,known_free_poses=known)


def test_repeated_failure_allows_only_verified_rear_and_retains_front_trigger():
    assert not recovery_report(failed=2)['path']
    assert not recovery_report(goal=Gap(1.,-.5,0.,.6,0,1))['path']
    report=recovery_report()
    assert report['diagnostics']['recovery_trigger']=='path_failure'
    assert report['path'][-1]==pytest.approx((-.3,0.,0.))
    assert not recovery_report(clear_scan(front_only=True))['path']
    known=[(-i*.02,0.,0.) for i in range(21)]
    assert recovery_report(clear_scan(front_only=True),known=known)['path']
    scan=clear_scan();scan.points=[(-.25,0.)]
    assert not recovery_report(scan)['path']


def test_short_ackermann_primitive_can_reach_a_goal_six_cm_away():
    cfg=replace(Settings(),assume_unobserved_free=True,require_goal_heading=False,goal_tolerance=.025)
    with patch('autorace_parking.core.parking_candidates',return_value=[]):
        path=plan_path(clear_scan(),Gap(.065,-.005,0.,.6,0,1),cfg)
    assert path and math.dist(path[-1][:2],(.065,-.005))<=cfg.goal_tolerance
    assert all(CollisionChecker(clear_scan(),cfg).free(p) for p in path)


@pytest.mark.parametrize('blocked',[False,True])
def test_completed_or_blocked_reverse_keeps_route_and_zero_control_until_replacement(blocked):
    cfg=replace(Settings(),continuous_replanning=True,allow_reverse=True,recovery_reverse_distance=.3)
    follower=Follower(cfg,require_parking_heading=False)
    follower.accept(Gap(.8,-.5,0.,.5,0,1),[(-i*.02,0.,0.) for i in range(16)],False,(0.,0.,0.),recovery=True)
    scan=clear_scan();pose=(0.,0.,0.) if blocked else (-.29,0.,0.)
    if blocked:scan.points=[(-.25,0.)]
    follower.last_drive_direction=-1
    v,a,state=follower.command(scan,pose,True,now=20.)
    assert (v,a)==(0.,0.) and state==('RECOVERY_BLOCKED' if blocked else 'PATH_HELD')
    assert follower.path and not follower.recovering and follower.recovery_finished
    assert follower.command(scan,pose,True,now=20.1)[0]==0.


@pytest.fixture
def node():
    import rclpy
    from rclpy.parameter import Parameter
    from autorace_parking.node import ParkingNode
    rclpy.init();n=ParkingNode(parameter_overrides=[Parameter('localization_source',value='scan'),
        Parameter('allow_reverse',value=True),Parameter('recovery_on_front_obstacle_only',value=True),
        Parameter('recovery_on_path_failure',value=True),Parameter('continuous_replanning',value=True),
        Parameter('max_recovery_attempts',value=2)])
    n.activation.active=True;n.now_seconds=lambda:20.;n.scan=clear_scan();n.scan_time=20.;n.failed_plans=3
    try:yield n
    finally:n.destroy_node();rclpy.shutdown()


def propose(node,origin=(0.,0.,0.)):
    future=Future();future.set_result({'goal':Gap(-.1,-.5,-math.pi/2,.6,0,1),
        'path':[(-i*.02,0.,0.) for i in range(16)],'final':False,
        'diagnostics':{'reason_code':'recovery_ready','recovery_trigger':'path_failure','recovery_distance_m':.3}})
    node.pending=future,node.epoch,origin,20.;node.poll_plan()


def test_reverse_reserves_mission_total_budget_and_waits_for_three_stopped_scans(node):
    propose(node)
    assert node.follower.recovering and node.recovery_attempts==1 and node.recovery_remaining==0.
    for i in range(2):
        assert node.follower.command(node.scan,(0.,0.,0.),True,20.+i*.1)[2]=='CHANGING_DIRECTION'
    v,a,state=node.follower.command(node.scan,(0.,0.,0.),True,20.2)
    assert -.2<=v<0 and a==0 and state=='RECOVERING'
    node.follower.path=[];node.failed_plans=3;propose(node)
    assert not node.follower.path and node.recovery_attempts==1
    node.reset_motion();assert node.recovery_remaining==node.cfg.recovery_total_distance


@pytest.mark.parametrize('obstacle,origin',[(True,(0.,0.,0.)),(False,(.2,0.,0.))])
def test_reverse_rechecks_new_obstacles_and_stale_pose_before_acceptance(node,obstacle,origin):
    if obstacle:node.scan.points=[(-.25,0.)]
    propose(node,origin)
    assert not node.follower.path and node.recovery_attempts==0 and node.recovery_remaining==.3


def test_target_updates_even_if_delayed_replacement_cannot_join(node):
    node.follower.accept(Gap(1.,-.5,0.,.5,0,1),[(0.,0.,0.),(.5,0.,0.)],True,(0.,0.,0.))
    old=node.follower.path.copy();future=Future()
    future.set_result({'goal':Gap(.2,-.5,-math.pi/2,.5,0,1),'path':[(0.,0.,0.),(.3,0.,0.)],'final':True,
        'diagnostics':{'target':{'x':.2,'y':-.5,'yaw':-math.pi/2},'support_ids':(1,2,3,4,5),'goal_confirmed':True}})
    node.pending=future,node.epoch,(1.,0.,0.),20.;node.poll_plan()
    assert node.follower.path==old and node.tracked_goal.x==pytest.approx(1.2) and node.tracked_goal.confirmed
