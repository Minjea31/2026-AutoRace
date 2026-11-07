from concurrent.futures import Future
from dataclasses import replace
import math
from unittest.mock import patch
import pytest
from autorace_parking.core import Settings,Scan,Gap,CollisionChecker
from autorace_parking.control import Follower,local_goal
from autorace_parking.regions import with_occupied_slots,first_occupied_path_collision,outside_grid


def scan():return Scan([math.inf]*361,-math.pi,math.pi/180,.01,12.,(0.,0.,0.))


def layout(yaw=0.):
    return {'origin':[1.,0.],'yaw':yaw,'cell_width':.5,'cell_depth':.6,'border':.02,
            'locked':False,'cells':[{'id':2,'center':[1.,0.],'occupied':True}]}


@pytest.mark.parametrize('yaw',[0.,.63])
def test_whole_occupied_cell_blocks_vehicle_corner_even_when_it_has_no_points(yaw):
    cfg=Settings();source=with_occupied_slots(scan(),layout(yaw),(0.,0.,0.),cfg);checker=CollisionChecker(source,cfg)
    pose=(1.-.45*math.cos(yaw),-.45*math.sin(yaw),yaw)
    assert not checker.free(pose,allow_unknown=True)
    assert checker.last_block=='occupied_slot' and checker.last_block_detail['slot_id']==2
    assert CollisionChecker(scan(),cfg).free(pose,allow_unknown=True)
    assert checker.free((1.-.6*math.cos(yaw),-.6*math.sin(yaw),yaw),allow_unknown=True)


def test_sparse_path_cannot_jump_across_filled_cell_between_endpoints():
    checker=CollisionChecker(with_occupied_slots(scan(),layout(),(0.,0.,0.),Settings()),Settings())
    assert checker.free((0.,0.,0.)) and checker.free((2.,0.,0.))
    collision=first_occupied_path_collision(checker,[(0.,0.,0.),(2.,0.,0.)],.02)
    assert collision['kind']=='occupied_slot' and collision['slot_id']==2


def test_rear_axle_error_uses_desired_body_pose_in_current_base_coordinates():
    f=Follower(replace(Settings(),arrival_use_rear_axle=True,rear_axle_offset_x=-.1))
    goal=Gap(1.,1.,math.pi/2,.6,0,1)
    assert f.arrival_error(goal)==pytest.approx((1.1,.9))
    assert (goal.x,goal.y)==(1.,1.)
    assert f.arrival_error(Gap(0.,0.,0.,.6,0,1))==pytest.approx((0.,0.))


def test_small_rear_error_does_not_allow_body_center_to_stop_outside_tolerance():
    cfg=replace(Settings(),arrival_use_rear_axle=True,auto_exit_after_parking=False,assume_unobserved_free=True)
    f=Follower(cfg,require_parking_heading=True,parking_position_tolerance=.03)
    g=Gap(1.,0.,0.,.6,0,1);f.accept(g,[(0.,0.,0.),(1.,0.,0.)],True,(0.,0.,0.))
    pose=(1.004,.035,.3)
    assert math.hypot(*f.arrival_error(local_goal(g,pose)))<.03
    for t in (0.,.1,.2,.3):f.command(scan(),pose,True,t)
    assert not f.parked and f.arrival_count==0


def test_auto_departure_waits_three_seconds_and_exits_grid_even_if_parked_lane_aligned():
    cfg=replace(Settings(),assume_unobserved_free=True,auto_exit_after_parking=True,arrival_use_rear_axle=True)
    f=Follower(cfg,speed=.2,hold_seconds=3.)
    grid={'origin':[1.,-1.],'yaw':0.,'width':.5,'depth':.6}
    goal=Gap(1.,-1.,0.,.5,0,1,parking_grid=grid)
    f.accept(goal,[(0.,-1.,0.),(1.,-1.,0.)],True,(0.,0.,0.))
    f.entry_trace=[(i*.02,-1.,0.) for i in range(51)]
    for t in (0.,.1,.2):f.command(scan(),(1.,-1.,0.),True,t)
    assert f.parked and f.exit_route_ready and len(f.path)>2
    assert f.exit_path_length>.5 and outside_grid(f.path[-1],grid,cfg)
    stamp=f.parked_since
    assert f.command(scan(),(1.,-1.,0.),True,stamp+2.999)==(0.,0.,'PARKED')
    v,a,state=f.command(scan(),(1.,-1.,0.),True,stamp+3.01)
    assert v<0 and state=='EXITING'


def test_incomplete_exit_history_holds_instead_of_pretending_to_leave():
    cfg=replace(Settings(),assume_unobserved_free=True,auto_exit_after_parking=True)
    f=Follower(cfg,hold_seconds=3.);grid={'origin':[1.,-1.],'yaw':0.,'width':.5,'depth':.6}
    f.accept(Gap(1.,-1.,0.,.5,0,1,parking_grid=grid),[(.9,-1.,0.),(1.,-1.,0.)],True,(0.,0.,0.))
    f.entry_trace=[(.9,-1.,0.),(1.,-1.,0.)]
    for t in (0.,.1,.2):f.command(scan(),(1.,-1.,0.),True,t)
    assert not f.exit_route_ready
    assert f.command(scan(),(1.,-1.,0.),True,4.)==(0.,0.,'EXIT_PATH_ERROR')


@pytest.fixture
def node():
    import rclpy
    from rclpy.parameter import Parameter
    from autorace_parking.node import ParkingNode
    rclpy.init();n=ParkingNode(parameter_overrides=[Parameter('localization_source',value='scan'),Parameter('assume_unobserved_free',value=True)])
    n.activation.active=True;n.now_seconds=lambda:20.;n.scan=scan();n.scan_time=20.
    try:yield n
    finally:n.destroy_node();rclpy.shutdown()


def test_blocked_old_path_is_hidden_but_zero_command_ownership_is_retained(node):
    node.slot_map=layout();node.follower.accept(Gap(2.,0.,0.,.6,0,1),[(0.,0.,0.),(2.,0.,0.)],True,(0.,0.,0.))
    assert node.visible_path()==[] and node.path_hidden_by_slot and node.follower.path


def test_current_occupied_cells_reject_stale_worker_path_before_acceptance(node):
    node.slot_map=layout();f=Future()
    f.set_result({'goal':Gap(2.,0.,0.,.6,0,1),'path':[(0.,0.,0.),(2.,0.,0.)],'final':True,'diagnostics':{}})
    node.pending=f,node.epoch,(0.,0.,0.),20.;node.poll_plan()
    assert not node.follower.path
    assert node.planning_info['candidate_path_rejected_occupied_slot']['slot_id']==2


def test_recovery_reports_exact_remembered_collision_instead_of_calling_it_new():
    cfg=replace(Settings(),continuous_replanning=True,recovery_reverse_distance=.3)
    f=Follower(cfg);f.accept(Gap(1.,0.,0.,.6,0,1),[(0.,0.,0.),(-.3,0.,0.)],False,(0.,0.,0.),recovery=True)
    memory=scan();memory.points=[(-.22,0.)]
    v,a,state=f.command(scan(),(0.,0.,0.),True,0.,obstacle_scan=memory)
    assert v==a==0 and state=='RECOVERY_BLOCKED'
    assert f.last_collision['kind']=='obstacle' and f.last_collision['point']==[-.22,0.]


def recovery_sources(obstacles=()):
    from autorace_parking.obstacle_memory import MemoryScan
    memory=MemoryScan.__new__(MemoryScan);memory.__dict__=scan().__dict__.copy();memory.points=list(obstacles);memory.point_owners={}
    raw=scan();raw.points=[(.25,0.)]
    return memory,raw


def recovery_config():
    return replace(Settings(),allow_reverse=True,recovery_uses_memory_map=True,recovery_on_front_obstacle_only=True,
        recovery_min_distance=.2,recovery_reverse_distance=.3,recovery_total_distance=1.5,max_recovery_attempts=5,
        recovery_speed=.2,assume_unobserved_free=True,continuous_replanning=True)


def test_reverse_does_not_start_when_less_than_twenty_cm_plus_stop_room_is_clear():
    from autorace_parking.control import build_plan_report
    memory,raw=recovery_sources([(-.4,0.)])
    result=build_plan_report(memory,recovery_config(),Gap(.5,-.3,0.,.6,0,1),front_scan=raw)
    assert not result['path'] and result['diagnostics']['recovery_below_minimum_distance']
    assert result['diagnostics']['recovery_available_distance_m']<.2


@pytest.mark.parametrize('obstacles',[(),((-.50,0.),)])
def test_each_normal_reverse_leg_actually_travels_twenty_to_thirty_cm(obstacles):
    from autorace_parking.control import build_plan_report
    from autorace_parking.core import advance
    memory,raw=recovery_sources(obstacles);config=recovery_config()
    result=build_plan_report(memory,config,Gap(.5,-.3,0.,.6,0,1),front_scan=raw)
    assert result['diagnostics']['reason_code']=='recovery_ready'
    f=Follower(config,speed=.25,min_turn_speed=.2);f.accept(result['goal'],result['path'],False,(0.,0.,0.),recovery=True)
    pose=(0.,0.,0.)
    for i in range(200):
        # The static map must be re-expressed in the moving body frame.
        observed=scan();observed.points=[(x-pose[0],y-pose[1]) for x,y in obstacles]
        v,a,state=f.command(raw,pose,True,i*.05,obstacle_scan=observed)
        assert -.2-1e-9<=v<=0.
        pose=advance(pose,v*.05,0.)
        if f.recovery_finished:break
    assert f.recovery_finished and .2-1e-6<=abs(pose[0])<=.3+1e-6


def test_five_recovery_legs_are_accepted_and_sixth_is_rejected(node):
    from autorace_parking.control import build_plan_report
    memory,raw=recovery_sources();config=recovery_config();node.cfg=config;node.follower.cfg=config
    node.memory_enabled=True;node.memory.stamp=20.;node.scan=raw;node.planning_scan=lambda:memory
    node.recovery_remaining=1.5;node.blind_reverse_remaining=0.
    report=build_plan_report(memory,config,Gap(.5,-.3,0.,.6,0,1),front_scan=raw)
    for count in range(1,6):
        node.follower.recovering=False;future=Future();future.set_result(report)
        node.pending=future,node.epoch,(0.,0.,0.),20.;node.poll_plan()
        assert node.recovery_attempts==count and node.follower.recovering
    assert node.recovery_remaining==pytest.approx(0.)
    path=node.follower.path;future=Future();future.set_result(report)
    node.pending=future,node.epoch,(0.,0.,0.),20.;node.poll_plan()
    assert node.recovery_attempts==5 and node.follower.path is path
