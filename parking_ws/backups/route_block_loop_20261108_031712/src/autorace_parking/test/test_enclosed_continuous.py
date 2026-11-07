from concurrent.futures import Future
from dataclasses import replace
import math
from unittest.mock import patch

import pytest

from autorace_parking.core import Settings,Gap,Scan,advance
from autorace_parking.control import Follower,build_plan_report
from autorace_parking.obstacle_memory import ObstacleMemory
from autorace_parking.enclosed import enclosed_goals


def config():
    return replace(Settings(clearance=.04),enclosed_space_detection=True,continuous_replanning=True,
                   assume_unobserved_free=True,require_goal_heading=False,min_cluster_size=.14,
                   max_gap=.8,min_turn_radius=.4,max_steering=math.atan(.2/.4))


def scene(count=3,shift=0.):
    positions=[(1.,-.4),(1.8,-.4),(1.4,-1.15),(1.,-1.),(1.8,-1.)][:count]
    raw=Scan([0.]*361,-math.pi,math.pi/180,.01,12.,(0.,0.,0.))
    raw.points=[(x+shift+dx,y+dy) for x,y in positions for dx in [-.08,0.,.08] for dy in [-.08,0.,.08]]
    m=ObstacleMemory();m.update(raw,raw,(0.,0.,0.),1.,config())
    return m.snapshot(raw,(0.,0.,0.),config())


@pytest.mark.parametrize('count,confirmed',[(2,False),(3,False),(4,False),(5,True)])
def test_three_objects_find_provisional_space_and_five_confirm(count,confirmed):
    scan=scene(count);goals=enclosed_goals(scan,scan.candidate_clusters(config()),config(),(0.,0.,0.),{})
    if count==2:assert not goals;return
    assert goals
    assert goals[0].confirmed==confirmed
    assert len(goals[0].support_ids)==count
    assert 1.1<goals[0].x<1.7 and -1.<goals[0].y<-.5


def test_same_side_objects_do_not_define_enclosed_space():
    s=scene(3)
    # Move all support centers onto one line.
    from autorace_parking.core import Cluster
    objects=[Cluster([(x+dx,-.5+dy) for dx in [-.08,.08] for dy in [-.08,.08]],i) for i,x in enumerate([1.,1.4,1.8])]
    assert not enclosed_goals(s,objects,config(),(0.,0.,0.),{})


def test_confirmed_bay_refines_center_as_observations_change():
    cfg=config();s=scene(5);g=enclosed_goals(s,s.candidate_clusters(cfg),cfg,(0.,0.,0.),{})[0]
    assert g.confirmed
    report=build_plan_report(scene(5,.04),cfg,g)
    assert report['diagnostics'].get('goal_updated',False)
    target=report['diagnostics']['target']
    assert report['diagnostics']['goal_confirmed']
    assert 0 < math.hypot(target['x']-g.x,target['y']-g.y) <= cfg.goal_update_step
    assert target['yaw']==pytest.approx(g.yaw)


def test_provisional_goal_update_is_bounded():
    cfg=config();s=scene(3);g=enclosed_goals(s,s.candidate_clusters(cfg),cfg,(0.,0.,0.),{})[0]
    report=build_plan_report(scene(3,.08),cfg,g)
    assert report['diagnostics'].get('goal_updated')
    target=report['diagnostics']['target']
    assert math.hypot(target['x']-g.x,target['y']-g.y)<=cfg.goal_update_step+1e-9


@pytest.fixture
def node():
    import rclpy
    from rclpy.parameter import Parameter
    from std_msgs.msg import Bool
    from autorace_parking.node import ParkingNode
    rclpy.init();n=ParkingNode(parameter_overrides=[Parameter('localization_source',value='scan'),Parameter('continuous_replanning',value=True)])
    n.now_seconds=lambda:20.
    for _ in range(3):n.on_sign(Bool(data=True))
    n.scan=scene(3);n.scan_time=20.
    n.follower.accept(Gap(1.,-.5,0.,.5,0,1),[(0.,0.,0.),(.5,0.,0.),(1.,0.,0.)],True,(0.,0.,0.))
    try:yield n
    finally:n.destroy_node();rclpy.shutdown()


def test_replan_failure_keeps_old_path_and_commands_and_repeated_sign_does_not_reset(node):
    from std_msgs.msg import Bool
    old=node.follower.path.copy();f=Future();f.set_result({'goal':None,'path':[],'final':False,'diagnostics':{}})
    node.pending=f,node.epoch,(0.,0.,0.),20.
    node.poll_plan();assert node.follower.path==old
    node.sign_ignore_until=0.;epoch=node.epoch;node.on_sign(Bool(data=True));assert node.epoch==epoch
    outputs=[];node.publish_command=lambda v,a:outputs.append((v,a))
    with patch.object(node.worker,'submit',return_value=Future()) as submit:
        node.tick();assert submit.called
    assert outputs[-1][0]>0 and node.follower.path==old


def test_valid_replacement_swaps_without_empty_path_and_delayed_disconnected_one_is_rejected(node):
    old=node.follower.path.copy()
    for origin,accepted in [((0.,0.,0.),True),((2.,0.,0.),False)]:
        new=[(0.,0.,0.),(.3,0.,0.),(.7,-.1,-.1)]
        f=Future();f.set_result({'goal':Gap(.7,-.1,-.1,.5,0,1),'path':new,'final':True,'diagnostics':{}})
        node.pending=f,node.epoch,origin,20.;node.poll_plan()
        if accepted:
            assert all(a==pytest.approx(b) for a,b in zip(node.follower.path,new));old=node.follower.path.copy()
        else:assert node.follower.path==old


def test_actual_four_object_layout_has_an_interior_goal_and_valid_path():
    cfg=config();raw=Scan([0.]*361,-math.pi,math.pi/180,.01,12.,(0.,0.,0.))
    positions=[(.886,-.400),(.995,-.975),(1.407,-.895),(1.836,-.315)]
    raw.points=[(x+dx,y+dy) for x,y in positions for dx in [-.08,0.,.08] for dy in [-.08,0.,.08]]
    m=ObstacleMemory();m.update(raw,raw,(0.,0.,0.),1.,cfg)
    report=build_plan_report(m.snapshot(raw,(0.,0.,0.),cfg),cfg)
    target=report['diagnostics']['target']
    assert 1.05<target['x']<1.7 and -.9<target['y']<-.4
    assert len(report['diagnostics']['support_ids'])>=3
    assert report['path']


def test_new_nearby_collision_stops_without_deleting_the_old_route(node):
    old=node.follower.path.copy();node.scan.points=[(.25,0.)]
    _,_,state=node.follower.command(node.scan,(0.,0.,0.),new_scan=True,now=20.)
    assert state=='ROUTE_BLOCKED' and node.follower.path==old


def test_goal_estimation_is_invariant_to_vehicle_rotation():
    from autorace_parking.tracking import compose,inverse
    from autorace_parking.obstacle_memory import MemoryScan
    cfg=config();base=scene(4)
    first=enclosed_goals(base,base.candidate_clusters(cfg),cfg,(0.,0.,0.),{})[0]
    pose=(.3,.1,.7);trans=inverse(pose)
    moved=MemoryScan.__new__(MemoryScan);moved.__dict__=base.__dict__.copy()
    moved.points=[compose(trans,(*p,0.))[:2] for p in base.points]
    moved.point_owners={compose(trans,(*p,0.))[:2]:base.point_owners[p] for p in base.points}
    second=enclosed_goals(moved,moved.candidate_clusters(cfg),replace(cfg,row_yaw=-pose[2]),pose,{})[0]
    global_goal=compose(pose,(second.x,second.y,second.yaw))
    assert global_goal[:2]==pytest.approx((first.x,first.y),abs=.051)


def test_five_object_confirmation_is_retained_even_if_replacement_path_fails(node):
    old=node.follower.path.copy()
    f=Future();f.set_result({'goal':None,'path':[],'final':False,'diagnostics':{
        'target':{'x':1.1,'y':-.5,'yaw':0.},'goal_confirmed':True,'support_ids':(0,1,2,3,4)}})
    node.pending=f,node.epoch,(0.,0.,0.),20.;node.poll_plan()
    assert node.follower.path==old and node.tracked_goal.confirmed
    with patch.object(node.worker,'submit',return_value=Future()) as submit:
        node.tick()
    assert submit.call_args.args[3].confirmed


def test_continuous_replanning_model_reaches_the_enclosed_goal():
    from autorace_parking.tracking import compose,inverse
    from autorace_parking.obstacle_memory import MemoryScan
    from autorace_parking.control import local_goal
    cfg=replace(config(),steering_gain=1.,planning_timeout=.35)
    base=scene(4);f=Follower(cfg,speed=.25,lookahead=.25,require_parking_heading=False,
                           parking_position_tolerance=.06,min_turn_speed=.2)
    pose=(0.,0.,0.);goal=None;swaps=0
    for i in range(400):
        trans=inverse(pose);scan=MemoryScan.__new__(MemoryScan);scan.__dict__=base.__dict__.copy()
        scan.points=[compose(trans,(*p,0.))[:2] for p in base.points]
        scan.point_owners={compose(trans,(*p,0.))[:2]:base.point_owners[p] for p in base.points}
        if i%8==0 and not f.parked:
            report=build_plan_report(scan,replace(cfg,row_yaw=-pose[2]),local_goal(goal,pose) if goal else None,pose)
            if report['path']:
                f.accept(report['goal'],report['path'],report['final'],pose);goal=f.goal;swaps+=1
                f.index=min(range(len(f.path)),key=lambda j:math.hypot(f.path[j][0]-pose[0],f.path[j][1]-pose[1]))
        v,a,state=f.command(scan,pose,new_scan=i%2==0,now=i*.05,obstacle_scan=scan)
        pose=advance(pose,v*.05,math.tan(a)/cfg.wheelbase)
        if f.parked:break
    assert f.parked and swaps>=3
