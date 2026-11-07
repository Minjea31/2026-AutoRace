"""Final target semantics, two-object fallback, promotion and stationary hold."""
from concurrent.futures import Future
from dataclasses import fields,replace
from pathlib import Path
import json
import math
from unittest.mock import patch
import pytest
import yaml
from autorace_parking.core import Settings,Gap,Scan,Cluster,plan_path,advance
from autorace_parking.control import Follower,build_plan_report,local_goal
from autorace_parking.targets import automatic_goals,empty_area_center,polygon_moments,update_geometry_goal
from autorace_parking.obstacle_memory import MemoryScan
from autorace_parking.tracking import compose,inverse


def config(**kw):
    return replace(Settings(),automatic_target_selection=True,enclosed_space_detection=True,
        continuous_replanning=True,auto_exit_after_parking=False,assume_unobserved_free=True,
        min_cluster_size=.14,max_gap=.8,min_gap=.3,require_goal_heading=False,**kw)


def scan_scene(positions):
    raw=Scan([math.inf]*361,-math.pi,math.pi/180,.01,12.,(0.,0.,0.))
    scan=MemoryScan.__new__(MemoryScan);scan.__dict__=raw.__dict__.copy()
    cells=[((x+dx,y+dy),i) for i,(x,y) in enumerate(positions)
           for dx in (-.08,0.,.08) for dy in (-.08,0.,.08)]
    scan.points=[p for p,_ in cells];scan.point_owners=dict(cells)
    return scan


def targets(scan,cfg=None,pose=(0.,0.,0.)):
    cfg=cfg or config();details={}
    result=automatic_goals(scan,scan.candidate_clusters(cfg),cfg,pose,details)
    return result,details


@pytest.mark.parametrize('count',[3,4,5])
def test_enclosed_target_uses_three_boundary_midplanes_independent_of_count(count):
    positions=[(1.,-.4),(1.8,-.4),(1.4,-1.15),(1.,-1.),(1.8,-1.)][:count]
    scan=scan_scene(positions);goals,info=targets(scan)
    assert goals and goals[0].kind=='enclosed_center'
    assert goals[0].confirmed
    # Mouth y=-0.32; front inner face y=-1.07; x inner faces 1.08/1.72.
    assert (goals[0].x,goals[0].y)==pytest.approx((1.4,-.695),abs=1e-10)
    assert set(goals[0].support_ids)<=set(range(count))
    assert info['enclosure_detected']


def test_two_objects_use_exact_gap_midpoint_and_no_fake_enclosure():
    scan=scan_scene([(1.,-.7),(1.8,-.7)]);goals,details=targets(scan)
    assert goals and details['target_mode']=='pair_midpoint'
    g=goals[0];assert (g.x,g.y)==pytest.approx((1.4,-.7))
    assert g.width==pytest.approx(.64) and not g.confirmed
    assert g.stop_tolerance==pytest.approx(.05)


def test_three_collinear_objects_still_use_pair_gap():
    goals,details=targets(scan_scene([(1.,-.7),(1.8,-.7),(2.6,-.7)]))
    assert goals and not details['enclosure_detected']
    assert all(g.kind=='pair_midpoint' for g in goals)


@pytest.mark.parametrize('x',[1.2,2.2])
def test_two_objects_still_require_sufficient_configured_gap(x):
    assert not targets(scan_scene([(1.,-.7),(x,-.7)]))[0]


def test_pair_fallback_can_be_disabled():
    assert not targets(scan_scene([(1.,-.7),(1.8,-.7)]),config(pair_fallback_enabled=False))[0]


def test_enclosure_center_is_not_replaced_by_a_pair_when_its_pose_is_blocked():
    scan=scan_scene([(1.,-.4),(1.8,-.4),(1.4,-1.15)])
    scan.points.append((1.4,-.695))
    goals,details=targets(scan)
    assert goals and goals[0].kind=='enclosed_center'
    assert not goals[0].heading_options
    report=build_plan_report(scan,config())
    assert not report['path'] and report['diagnostics']['reason_code']=='center_pose_blocked'
    assert report['diagnostics']['target_kind']=='enclosed_center'
    assert (report['goal'].x,report['goal'].y)==pytest.approx((goals[0].x,goals[0].y))


def test_unreachable_center_does_not_shift_to_an_easier_entry_pocket():
    scan=scan_scene([(1.,-.4),(1.8,-.4),(1.4,-1.15)])
    g=targets(scan)[0][0]
    with patch('autorace_parking.control.plan_path',return_value=[]):
        report=build_plan_report(scan,config(),recovery_allowed=False)
    assert not report['path']
    assert (report['diagnostics']['target']['x'],report['diagnostics']['target']['y'])==pytest.approx((g.x,g.y))
    assert not report['diagnostics'].get('goal_relocated_for_reachability')


def test_centroid_subtracts_occupied_area_instead_of_using_vehicle_distance():
    polygon=[(0.,0.),(2.,0.),(2.,2.),(0.,2.)]
    obstacle=Cluster([(1.5,0.),(2.,0.),(2.,2.),(1.5,2.)])
    area,center=empty_area_center(polygon,[((0,),obstacle)])
    assert area==pytest.approx(3.) and center==pytest.approx((.75,1.))


def test_pair_promotes_to_enclosed_center_and_never_downgrades_after_occlusion():
    pair=targets(scan_scene([(1.,-.4),(1.8,-.4)]))[0][0]
    enclosed=targets(scan_scene([(1.,-.4),(1.8,-.4),(1.4,-1.15)]))[0][0]
    promoted,changed=update_geometry_goal(pair,[enclosed]);assert changed and promoted.kind=='enclosed_center'
    retained,changed=update_geometry_goal(promoted,[pair]);assert not changed and retained==promoted


def test_center_and_heading_options_are_invariant_under_vehicle_rotation():
    cfg=config();base=scan_scene([(1.,-.4),(1.8,-.4),(1.4,-1.15)])
    before=targets(base,cfg)[0][0];pose=(.2,.1,.8);transform=inverse(pose)
    moved=MemoryScan.__new__(MemoryScan);moved.__dict__=base.__dict__.copy()
    moved.points=[compose(transform,(*p,0.))[:2] for p in base.points]
    moved.point_owners={compose(transform,(*p,0.))[:2]:o for p,o in base.point_owners.items()}
    after=targets(moved,replace(cfg,row_yaw=-pose[2]),pose)[0][0]
    assert compose(pose,(after.x,after.y,0.))[:2]==pytest.approx((before.x,before.y),abs=1e-10)
    world=replace(before,heading_options=(0.,.1,math.pi))
    local=local_goal(world,pose)
    f=Follower(cfg);f.accept(local,[(0.,0.,0.),(1.,0.,0.)],True,pose)
    assert f.goal.heading_options==pytest.approx(world.heading_options)


@pytest.mark.parametrize('kind,tolerance',[('pair_midpoint',.05),('enclosed_center',.03)])
def test_arrival_holds_zero_forever_with_no_automatic_departure(kind,tolerance):
    cfg=config();scan=scan_scene([]);g=Gap(1.,0.,0.,.6,0,1,kind=kind,heading_options=(0.,),stop_tolerance=tolerance)
    f=Follower(cfg,require_parking_heading=False);f.accept(g,[(0.,0.,0.),(1.,0.,0.)],True,(0.,0.,0.))
    for t in (0.,.1,.2,.3):f.command(scan,(1.,0.,0.),True,t)
    assert f.parked and f.goal==g
    for t in (4.,30.,300.):
        assert f.command(scan,(1.,0.,0.),True,t)==(0.,0.,'PARKED')
        assert not f.exiting and f.goal==g and f.path


def test_short_forward_goal_is_not_lost_in_start_hash_cell():
    cfg=replace(config(),goal_tolerance=.005,require_goal_heading=True)
    scan=scan_scene([]);g=Gap(.025,.001,0.,.6,0,1,kind='pair_midpoint',heading_options=(0.,),stop_tolerance=.01)
    path=plan_path(scan,g,cfg)
    assert path and math.dist(path[-1][:2],(g.x,g.y))<=cfg.goal_tolerance


def test_real_profile_enables_selection_rear_arrival_and_three_second_departure():
    params=yaml.safe_load((Path(__file__).resolve().parents[1]/'config/parking_real.yaml').read_text())['parking_node']['ros__parameters']
    assert params['automatic_target_selection'] and params['pair_fallback_enabled']
    assert params['auto_exit_after_parking'] and params['parking_hold_seconds']==3.
    assert params['arrival_use_rear_axle'] and params['rear_axle_offset_x']==-.1
    assert params['block_occupied_slots']
    assert params['steering_gain']==2.0 and params['curvature_feedforward'] and params['recovery_total_distance']==1.5 and params['max_recovery_attempts']==5


def test_parked_pair_can_promote_without_erasing_the_old_path_before_new_route():
    import rclpy
    from rclpy.parameter import Parameter
    from autorace_parking.node import ParkingNode
    rclpy.init();n=ParkingNode(parameter_overrides=[Parameter('localization_source',value='scan'),
        Parameter('automatic_target_selection',value=True),Parameter('continuous_replanning',value=True),
        Parameter('auto_exit_after_parking',value=False),Parameter('assume_unobserved_free',value=True)])
    try:
        n.activation.active=True;n.now_seconds=lambda:20.;n.scan=scan_scene([]);n.scan_time=20.
        pair=Gap(1.,0.,0.,.6,0,1,support_ids=(0,1),kind='pair_midpoint',heading_options=(0.,),stop_tolerance=.05)
        n.follower.accept(pair,[(0.,0.,0.),(1.,0.,0.)],True,(0.,0.,0.));old=n.follower.path
        n.follower.parked=True;n.follower.parked_since=10.
        new=replace(pair,x=1.3,y=-.3,kind='enclosed_center',support_ids=(0,1,2),stop_tolerance=.03)
        future=Future();future.set_result({'goal':None,'path':[],'final':False,'diagnostics':{
            'target':{'x':new.x,'y':new.y,'yaw':new.yaw},'target_kind':new.kind,'target_stop_tolerance':new.stop_tolerance,
            'target_heading_options':new.heading_options,'support_ids':new.support_ids,'reason_code':'no_observed_path'}})
        n.pending=future,n.epoch,(0.,0.,0.),20.;n.poll_plan()
        assert not n.follower.parked and n.follower.goal.kind=='enclosed_center'
        assert n.follower.path is old and n.follower.awaiting_target_path
        assert n.follower.command(n.scan,(1.,0.,0.),True,20.)==(0.,0.,'PATH_HELD')
        n.tracker.pose=(1.,0.,0.)
        future=Future();future.set_result({'goal':replace(new,x=.3),'path':[(0.,0.,0.),(.15,-.1,-.1),(.3,-.3,0.)],
            'final':True,'diagnostics':{'target':{'x':.3,'y':-.3,'yaw':0.},'target_kind':new.kind,
            'target_heading_options':(0.,),'target_stop_tolerance':.03,'support_ids':(0,1,2),'reason_code':'path_ready'}})
        n.pending=future,n.epoch,(1.,0.,0.),20.;n.poll_plan()
        assert n.follower.path and not n.follower.awaiting_target_path
        assert n.follower.goal.stop_tolerance==.03
    finally:n.destroy_node();rclpy.shutdown()


@pytest.mark.parametrize('count',[2,3,5])
def test_measured_response_and_servo_delay_reach_the_geometric_goal(count):
    params=yaml.safe_load((Path(__file__).resolve().parents[1]/'config/parking_real.yaml').read_text())['parking_node']['ros__parameters']
    cfg=replace(Settings(**{f.name:params[f.name] for f in fields(Settings) if f.name in params}),slot_map_enabled=False,auto_exit_after_parking=False)
    positions=[(2.2,-.4),(3.,-.4),(3.,-1.2),(2.2,-1.2),(3.,-.8)][:count]
    if count==2:positions=[(2.2,-.7),(3.,-.7)]
    if count==3:positions=[(2.2,-.4),(3.,-.4),(2.6,-1.2)]
    base=scan_scene(positions);report=build_plan_report(base,cfg,recovery_allowed=False)
    assert report['final'] and report['path']
    f=Follower(cfg,speed=.25,lookahead=.25,require_parking_heading=False,min_turn_speed=.2)
    f.accept(report['goal'],report['path'],True,(0.,0.,0.))
    pose=(0.,0.,0.);velocity=angle=0.
    for i in range(600):
        transform=inverse(pose);scan=MemoryScan.__new__(MemoryScan);scan.__dict__=base.__dict__.copy()
        scan.points=[compose(transform,(*q,0.))[:2] for q in base.points]
        scan.point_owners={compose(transform,(*q,0.))[:2]:o for q,o in base.point_owners.items()}
        speed,steering,state=f.command(scan,pose,new_scan=i%2==0,now=i*.05,obstacle_scan=scan)
        velocity+=(speed-velocity)*(.05/.12);angle+=(steering-angle)*(.05/.25)
        next_pose=advance(pose,velocity*.05,cfg.steering_response_scale*math.tan(angle)/cfg.wheelbase)
        from autorace_parking.core import CollisionChecker
        assert CollisionChecker(scan,cfg).free(compose(inverse(pose),next_pose),allow_unknown=True)
        pose=next_pose
        if f.parked:break
    assert f.parked and not f.exiting
    assert math.dist(pose[:2],(f.goal.x,f.goal.y))<=f.goal.stop_tolerance
    assert f.goal.kind==('pair_midpoint' if count==2 else 'enclosed_center')
    for t in (40.,100.):assert f.command(scan,pose,True,t)[0]==0.


def test_auto_exit_enabled_after_indefinite_hold_prepares_reverse_trace_first():
    cfg=config();scan=scan_scene([]);f=Follower(cfg,require_parking_heading=False)
    g=Gap(1.,0.,0.,.6,0,1,kind='pair_midpoint',heading_options=(0.,),stop_tolerance=.05)
    f.accept(g,[(0.,0.,0.),(1.,0.,0.)],True,(0.,0.,0.))
    for t in (0.,.1,.2,.3):f.command(scan,(1.,0.,0.),True,t)
    assert f.parked and not f.exit_prepared
    f.cfg=replace(cfg,auto_exit_after_parking=True)
    assert f.command(scan,(1.,0.,0.),True,100.)==(0.,0.,'PARKED')
    assert f.exit_prepared and not f.exiting
