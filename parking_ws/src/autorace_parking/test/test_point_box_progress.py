"""The local parking policy: one 5 cm margin, point collisions and five states."""
from dataclasses import fields,replace
import math
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml
from visualization_msgs.msg import Marker

from autorace_parking.core import Settings,Scan,Gap,Cluster,CollisionChecker,RecoveryCollisionChecker
from autorace_parking.control import entry_motion_settings,entry_plan_settings,safe_progress_step
from autorace_parking.diagnostics import STATES,operator_report
from autorace_parking.node import ParkingNode,validate_settings
from autorace_parking.regions import with_occupied_slots,first_path_collision
from autorace_parking.targets import available_headings
from autorace_parking.tracking import compose


def profile():
    params=yaml.safe_load((Path(__file__).parents[1]/'config/parking_real.yaml').read_text())['parking_node']['ros__parameters']
    names={f.name for f in fields(Settings)}
    return Settings(**{k:v for k,v in params.items() if k in names})


def scan():
    return Scan([math.inf]*361,-math.pi,math.pi/180,.01,10.)


def test_exactly_one_five_cm_margin_for_detection_and_entry():
    cfg=profile();validate_settings(cfg)
    goal=Gap(1.,-.6,0.,.5,0,1,confirmed=True,stop_tolerance=.03)
    motion,_=entry_motion_settings(cfg,goal)
    for c in (cfg,entry_plan_settings(motion,goal)):
        assert c.vehicle_length+2*c.clearance==pytest.approx(.422)
        assert c.vehicle_width+2*c.clearance==pytest.approx(.32)
    assert cfg.auto_exit_after_parking and cfg.recovery_on_path_failure
    assert not cfg.block_occupied_slots and cfg.obstacle_min_cluster_size==0.
    assert cfg.assume_unobserved_free and not cfg.yellow_boundary_enabled
    assert cfg.entry_via_short_edge and cfg.entry_from_left_only


@pytest.mark.parametrize('yaw',[0.,.7,-1.3])
def test_rotated_point_inside_box_blocks_and_a_point_outside_does_not(yaw):
    cfg=profile();pose=(.5,-.2,yaw);s=scan()
    for local,blocked in [((.210,.159,0.),True),((.212,.161,0.),False)]:
        s.points=[compose(pose,local)[:2]]
        s.occupied_slots=[{'id':1,'pose':pose,'half_size':(.5,.5)}]
        checker=CollisionChecker(s,cfg)
        assert bool(list(checker.points_inside(pose)))==blocked
        assert checker.free(pose)==(not blocked)


def test_object_bbox_overlap_without_lidar_points_does_not_block_goal_heading():
    cfg=profile();s=scan()
    s.points=[(-.4,-.4),(-.4,.4),(.4,-.4),(.4,.4)]
    objects=[((0,),Cluster(s.points,1))]
    assert available_headings((0.,0.),objects,cfg,(0.,0.,0.),CollisionChecker(s,cfg))
    solid=replace(cfg,block_occupied_slots=True)
    assert not available_headings((0.,0.),objects,solid,(0.,0.,0.),CollisionChecker(s,solid))


def test_recovery_uses_the_same_point_box_without_ignoring_margin_points():
    cfg=profile();s=scan();s.points=[(.20,0.)]
    checker=RecoveryCollisionChecker(s,cfg)
    assert not checker.escape_points and not checker.free((0.,0.,0.))
    assert checker.last_block=='obstacle'


def test_missing_full_path_can_make_a_safe_curved_step_toward_the_target():
    cfg=profile();s=scan();goal=Gap(1.,-.6,0.,.5,0,1,confirmed=True,stop_tolerance=.03)
    path,k,blocked=safe_progress_step(s,cfg,goal,.2)
    assert path and k<0 and blocked is None
    assert math.dist(path[-1][:2],(goal.x,goal.y))<math.hypot(goal.x,goal.y)
    assert not first_path_collision(CollisionChecker(s,cfg),path,cfg.collision_step)
    s.points=[(.20,0.)]
    path,_,blocked=safe_progress_step(s,cfg,goal,.2)
    assert not path and blocked['kind']=='obstacle'


def test_a_short_step_cannot_enter_the_bay_through_its_long_side():
    cfg=profile();grid={'origin':(0.,0.),'yaw':0.,'cell_width':.5,'cell_depth':.6,
        'cells':[{'id':3,'center':(.5,0.),'occupied':False,'eligible_empty':True}]}
    s=with_occupied_slots(scan(),grid,(0.,0.,0.),cfg,3)
    path,_,blocked=safe_progress_step(s,cfg,Gap(.5,0.,0.,.5,0,1,confirmed=True,slot_id=3),.4)
    assert not path and blocked['kind']=='slot_long_side_entry'


def test_vehicle_box_color_depends_only_on_lidar_points_inside_it():
    cfg=profile();s=scan();messages=[]
    node=SimpleNamespace(cfg=cfg,planning_scan=lambda:s,
        marker=lambda ns,ident,kind:Marker(ns=ns,id=ident,type=kind),
        roi_pub=SimpleNamespace(publish=messages.append))
    for point,hit in [((.212,0.),False),((.210,0.),True)]:
        s.points=[point];ParkingNode.publish_collision_box(node,Marker(ns='candidate_roi'))
        roi,box,points=messages[-1].markers
        assert roi.ns=='candidate_roi'
        assert node.collision_box_points==int(hit)
        assert box.color.r==1. if hit else box.color.g==1.
        assert max(p.x for p in box.points)-min(p.x for p in box.points)==pytest.approx(.422)
        assert max(p.y for p in box.points)-min(p.y for p in box.points)==pytest.approx(.32)
        assert points.action==Marker.ADD if hit else points.action==Marker.DELETE


def test_only_five_public_states_and_previous_plan_failure_does_not_mask_motion():
    assert set(STATES.values())=={'EKF 대기','path 계산','전진중','후진중','주차 완료'}
    for phase,speed,expected in [('WAITING_SCAN',0.,'WAITING_EKF'),('PLANNING',0.,'PLANNING'),
        ('SEARCHING',.1,'FORWARD'),('EXITING',-.1,'REVERSE'),('PARKED',0.,'PARKED')]:
        report=operator_report(phase,speed,planning={'reason_code':'budget_exceeded'})
        assert report['state']==expected
        if speed!=0.:assert not report['error_code']
    blocked=operator_report('SEARCH_BLOCKED',0.,planning={'search_block':{'kind':'obstacle','point':[.2,.1]}})
    assert blocked['state']=='PLANNING' and blocked['error_code']=='OBSTACLE'
    assert '라이다 점' in blocked['error_reason'] and '0.20' in blocked['error_reason']
