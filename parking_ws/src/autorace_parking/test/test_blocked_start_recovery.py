from dataclasses import replace
from concurrent.futures import Future
import math
from unittest.mock import patch
import pytest
from autorace_parking.core import Scan,Gap,CollisionChecker,RecoveryCollisionChecker,advance
from autorace_parking.control import Follower,build_plan_report
from test_obstacle_collision_mode import profile as current_profile
from test_operator_ros import node,activate


def profile():
 # These recorded escape scenarios predate the strict 5 cm point-box policy.
 return replace(current_profile(),clearance=.1,path_tracking_margin=.02,
                recovery_front_clearance=.1,recovery_margin_escape_enabled=True)


def scan(points=()):
 s=Scan([math.inf]*361,-math.pi,math.pi/180,.01,12.);s.points=list(points);return s


def goal(confirmed=True):return Gap(.50,-.73,0.,.5,0,1,kind='enclosed_center' if confirmed else 'pair_midpoint',confirmed=confirmed,support_ids=(2,5),heading_options=(0.,))


@pytest.mark.parametrize('confirmed',[False,True])
def test_start_blocked_enters_bounded_recovery_instead_of_retrying_forever(confirmed):
 cfg=profile();source=scan([(.238,-.130)])
 assert not CollisionChecker(source,cfg).free((0.,0.,0.))
 assert CollisionChecker(source,replace(cfg,clearance=0.)).free((0.,0.,0.))
 result=build_plan_report(source,cfg,goal(confirmed),failed_plans=3,stalled_route=True)
 assert result['diagnostics']['reason_code']=='recovery_ready',result['diagnostics']
 assert result['diagnostics']['recovery_margin_escape_points']==1
 assert result['path'][-1]==pytest.approx((-.3,0.,0.))
 assert not CollisionChecker(source,cfg).free(result['path'][0])
 assert CollisionChecker(source,cfg).free(result['path'][-1])


@pytest.mark.parametrize('points',[[(-.238,0.)],[(.10,0.)],[(.238,-.13),(-.4,0.)]])
def test_real_body_or_rear_obstacles_never_use_front_margin_escape(points):
 result=build_plan_report(scan(points),profile(),goal(),failed_plans=3,stalled_route=True)
 assert not result['path']


def test_escape_never_permits_motion_toward_an_existing_close_point():
 cfg=profile();checker=RecoveryCollisionChecker(scan([(.238,-.13)]),cfg)
 assert checker.free((-.04,0.,0.))
 assert not checker.free((.02,0.,0.))
 assert not checker.free((-.001,-.02,.05))


def test_held_pair_recovery_requires_failure_threshold_or_front_trigger():
 cfg=profile();source=scan([(.05,.18)])
 assert not build_plan_report(source,cfg,goal(False),failed_plans=2,stalled_route=True,front_scan=scan())['path']
 assert build_plan_report(source,cfg,goal(False),failed_plans=3,stalled_route=True,front_scan=scan())['path']


def test_actual_twenty_cm_retreat_from_front_margin_overlap_is_followable():
 cfg=profile();world_points=[(.238,-.13)];source=scan(world_points)
 result=build_plan_report(source,cfg,goal(),failed_plans=300,stalled_route=True)
 f=Follower(cfg,speed=.25,min_turn_speed=.2,max_steering_rate=0.);f.accept(result['goal'],result['path'],False,(0.,0.,0.),recovery=True)
 f.last_drive_direction=-1;pose=(0.,0.,0.)
 for i in range(200):
  current=scan([(x-pose[0],y) for x,y in world_points])
  v,a,state=f.command(current,pose,new_scan=i%2==0,now=i*.05,obstacle_scan=current)
  assert state!='RECOVERY_BLOCKED',(i,pose,f.last_collision)
  assert -min(f.speed,cfg.recovery_speed)-1e-8<=v<=0.
  pose=advance(pose,v*.05,math.tan(a)/cfg.wheelbase)
  if f.recovery_finished:break
 assert f.recovery_finished and .2<=abs(pose[0])<=.3
 assert CollisionChecker(scan(world_points),cfg).free(pose)


def test_node_accepts_recovery_from_blocked_start_and_unconfirmed_pair(node):
 activate(node);cfg=profile();node.cfg=cfg;node.follower.cfg=cfg
 source=scan([(.238,-.13)]);node.scan=source;node.scan_time=20.
 node.memory_enabled=True;node.memory.stamp=20.;node.planning_scan=lambda:source
 node.tracker.valid=True;node.tracked_goal=goal(False);node.state='ROUTE_BLOCKED';node.failed_plans=300
 result=build_plan_report(source,cfg,goal(False),failed_plans=300,stalled_route=True)
 future=Future();future.set_result(result);node.pending=future,node.epoch,(0.,0.,0.),20.
 node.poll_plan()
 assert node.follower.recovering and node.recovery_attempts==1


def test_live_832_point_map_can_retreat_out_of_existing_ten_cm_margin_overlap():
 import json
 from pathlib import Path
 from autorace_parking.core import Settings
 from autorace_parking.obstacle_memory import MemoryScan
 data=json.loads((Path(__file__).parent/'data/blocked_recovery_live_20261108.json').read_text())
 cfg=Settings(**data['parameters']);source=MemoryScan.__new__(MemoryScan);source.__dict__=scan().__dict__.copy()
 source.points=[tuple(p[:2]) for p in data['points']];source.point_owners={}
 assert len(source.points)==832
 assert not CollisionChecker(source,cfg).free((0.,0.,0.))
 assert CollisionChecker(source,replace(cfg,clearance=0.)).free((0.,0.,0.))
 target=Gap(.5,-.73,0.,.5,0,1,confirmed=True)
 result=build_plan_report(source,cfg,target,failed_plans=300,stalled_route=True)
 assert result['diagnostics']['reason_code']=='recovery_ready'
 assert result['diagnostics']['recovery_margin_escape_points']==3
 assert result['path'][-1]==pytest.approx((-.3,0.,0.))
 assert CollisionChecker(source,cfg).free(result['path'][-1])
