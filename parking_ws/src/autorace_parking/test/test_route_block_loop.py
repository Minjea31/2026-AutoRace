"""Obstacle stops persist between scans; stale worker routes cannot reopen them."""
from concurrent.futures import Future
from dataclasses import replace
import math
from unittest.mock import Mock

import pytest

from autorace_parking.core import Gap,Scan
from autorace_parking.control import Follower
from autorace_parking.regions import first_path_collision
from autorace_parking.core import CollisionChecker,Settings
from test_operator_ros import node,activate


def scan(points=()):
    result=Scan([math.inf]*361,-math.pi,math.pi/180,.01,12.)
    result.points=list(points)
    return result


def test_obstacle_stop_does_not_alternate_with_planning_between_scans():
    cfg=Settings(continuous_replanning=True,assume_unobserved_free=True,clearance=.10)
    follower=Follower(cfg)
    path=[(i*.02,0.,0.) for i in range(51)]
    follower.accept(Gap(1.,0.,0.,.6,0,1),path,True,(0.,0.,0.))
    original=follower.path
    # A target can become invalid while the full route is kept for replanning.
    follower.goal=None;follower.awaiting_target_path=True
    blocked=scan([(.39,0.)])
    for i in range(8):
        assert follower.command(blocked,(0.,0.,0.),new_scan=i%2==0,now=i*.05)==(0.,0.,'ROUTE_BLOCKED')
        assert follower.path is original
        assert follower.last_collision['kind']=='obstacle'
    # Only a genuinely fresh clear observation can release the collision stop.
    assert follower.command(scan(),(0.,0.,0.),new_scan=False,now=.45)[2]=='ROUTE_BLOCKED'
    assert follower.command(scan(),(0.,0.,0.),new_scan=True,now=.5)[2]=='PATH_HELD'
    assert follower.command(scan(),(0.,0.,0.),new_scan=False,now=.55)[2]=='PATH_HELD'


def test_validated_replacement_clears_obstacle_latch():
    cfg=Settings(continuous_replanning=True,assume_unobserved_free=True,clearance=.10)
    follower=Follower(cfg)
    path=[(i*.02,0.,0.) for i in range(51)];goal=Gap(1.,0.,0.,.6,0,1)
    follower.accept(goal,path,True,(0.,0.,0.));follower.stop('ROUTE_BLOCKED',0.)
    follower.accept(goal,path,True,(0.,0.,0.))
    assert not follower.route_blocked
    assert follower.command(scan(),(0.,0.,0.),new_scan=False,now=.05)[0]>0.


def prepare_node(node,points):
    activate(node)
    node.cfg=replace(node.cfg,continuous_replanning=True,assume_unobserved_free=True,clearance=.10)
    node.follower.cfg=node.cfg;node.memory_enabled=False
    node.scan=scan(points);node.scan_time=20.;node.tracker.valid=True
    path=[(i*.02,0.,0.) for i in range(51)];goal=Gap(1.,0.,0.,.6,0,1)
    node.follower.accept(goal,path,True,(0.,0.,0.))
    node.follower.stop('ROUTE_BLOCKED',20.);node.state='ROUTE_BLOCKED'
    return goal,path


def proposal(node,goal,path):
    result={'goal':goal,'path':path,'final':True,'diagnostics':{'reason_code':'path_ready'}}
    future=Future();future.set_result(result)
    node.pending=future,node.epoch,(0.,0.,0.),20.
    node.poll_plan()
    return result


def test_new_obstacle_in_latest_map_rejects_stale_candidate_without_resetting_failure_count(node):
    goal,path=prepare_node(node,[(.4,0.)]);original=node.follower.path
    assert not node.cfg.block_occupied_slots
    for i in range(3):
        result=proposal(node,goal,path)
        assert result['diagnostics']['candidate_path_rejected']['kind']=='obstacle'
        assert node.follower.path is original and node.follower.route_blocked
        assert node.failed_plans==i+1


def test_candidate_validation_skips_already_passed_prefix(node):
    goal,path=prepare_node(node,[(-.5,0.)])
    node.tracker.pose=(.5,0.,0.)
    result=proposal(node,goal,path)
    assert 'candidate_path_rejected' not in result['diagnostics']
    assert not node.follower.route_blocked and node.follower.index>0


def test_full_sweep_rejects_obstacle_between_sparse_path_samples():
    checker=CollisionChecker(scan([(.5,0.)]),Settings(clearance=.10,assume_unobserved_free=True))
    assert checker.free((0.,0.,0.)) and checker.free((1.,0.,0.))
    collision=first_path_collision(checker,[(0.,0.,0.),(1.,0.,0.)],.02)
    assert collision['kind']=='obstacle'


@pytest.mark.parametrize('blocked',[False,True])
def test_real_replan_policy_skips_safe_confirmed_entry_but_replans_on_new_obstacle(node,blocked):
    goal,path=prepare_node(node,[(.39,0.)] if blocked else [])
    node.cfg=replace(node.cfg,replan_while_following=False)
    node.follower.cfg=node.cfg
    node.follower.accept(replace(goal,confirmed=True),path,True,(0.,0.,0.))
    node.state='PARKING';node.pending=None;node.last_plan_wall=-math.inf
    node.localization_ready=lambda now:True
    node.steering_model_ready=lambda:True
    node.worker.shutdown(wait=True)
    future=Future();node.worker=Mock();node.worker.submit.return_value=future
    node.tick()
    assert node.worker.submit.call_count==int(blocked)
    assert node.state==('ROUTE_BLOCKED' if blocked else 'PARKING')
    future.cancel()
