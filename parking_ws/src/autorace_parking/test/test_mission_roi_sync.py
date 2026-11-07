from dataclasses import replace
from types import SimpleNamespace
from unittest.mock import patch
import math,json
import numpy as np
import pytest
import rclpy
from rclpy.parameter import Parameter
from std_msgs.msg import Bool,String
from autorace_parking.core import Settings,Scan,CollisionChecker,Gap
from autorace_parking.roi import candidate_view,contains
from autorace_parking.tracking import compose,inverse
from autorace_parking.node import ParkingNode
from test_slot_map import objects,snapshot
from test_fusion_ros import odometry,clear_scan


def cfg():return replace(Settings(),candidate_roi_enabled=True,assume_unobserved_free=True)


@pytest.mark.parametrize('pose',[(0.,0.,0.),(1.,-.4,1.4),(3.5,-1.,math.pi)])
def test_roi_is_fixed_at_start_while_vehicle_translates_and_rotates(pose):
    world=[(.5,-.8),(2.9,-1.5),(-.1,-.8),(3.1,-.8),(1.,-.1),(1.,-2.1)]
    source=Scan([],0.,1.,.01,12.);source.points=[compose(inverse(pose),(*p,0.))[:2] for p in world]
    original=source.points.copy();view=candidate_view(source,cfg(),pose)
    assert [compose(pose,(*p,0.))[:2] for p in view.points]==pytest.approx(np.array(world[:2]))
    assert source.points==original


def test_outside_roi_points_remain_collision_obstacles_and_owner_groups_are_clipped():
    source=snapshot(objects([(1.,-.8),(1.,0.)]));view=candidate_view(source,cfg(),(0.,0.,0.))
    assert len(view.candidate_clusters(cfg()))==1
    assert len(source.candidate_clusters(cfg()))==2
    assert not CollisionChecker(source,cfg()).free((1.,0.,0.),allow_unknown=True)


@pytest.fixture
def node():
    rclpy.init();n=ParkingNode(parameter_overrides=[Parameter('localization_source',value='fused'),Parameter('odom_frame',value='odom'),Parameter('continuous_replanning',value=True),Parameter('candidate_roi_enabled',value=True)])
    n.now_seconds=lambda:20.
    yield n
    n.destroy_node();rclpy.shutdown()


def start(node):
    for _ in range(3):node.on_sign(Bool(data=True))


def test_yolo_restarts_active_route_tracker_memory_and_time_pairing_after_five_seconds(node):
    with patch('autorace_parking.node.time.monotonic',return_value=100.):start(node)
    node.on_localization(odometry(20.));node.on_scan(clear_scan(20.))
    node.follower.accept(Gap(1.,-.8,0.,.5,0,1),[(0.,0.,0.),(1.,-.8,0.)],True,(0.,0.,0.))
    old_tracker=node.tracker;old_follower=node.follower
    node.memory.cells[(0,0)]=[0.,0.,3,0]
    commands=[];node.publish_command=lambda v,a:commands.append((v,a))
    with patch('autorace_parking.node.time.monotonic',return_value=104.9):node.on_sign(Bool(data=True))
    assert node.tracker is old_tracker
    node.now_seconds=lambda:21.
    with patch('autorace_parking.node.time.monotonic',return_value=105.):node.on_sign(Bool(data=True))
    assert commands==[(0.,0.)] and node.attempt==2
    assert node.tracker is not old_tracker and node.follower is not old_follower
    assert node.tracker.origin is None and not node.memory.cells and not node.follower.path
    assert not node.odometry.samples and not node.pending_scans and node.fusion_health is None
    node.on_localization(odometry(20.99));node.on_scan(clear_scan(20.99))
    assert not node.odometry.samples and node.scan is None
    node.on_localization(odometry(21.,x=2.));node.on_scan(clear_scan(21.))
    assert node.tracker.pose==pytest.approx((0.,0.,0.))
    assert node.tracker.origin==pytest.approx((2.,0.,0.))


def test_scan_queue_retains_only_latest_and_ignores_late_older_packet(node):
    start(node);node.now_seconds=lambda:20.3
    node.on_scan(clear_scan(20.1));node.on_scan(clear_scan(20.2));node.on_scan(clear_scan(20.15))
    assert node.pending_scans.maxlen==1 and len(node.pending_scans)==1
    assert node.pending_scans[0].header.stamp.nanosec==200000000
    node.on_localization(odometry(20.1))
    assert node.scan is None  # Do not revive the older now-matchable scan.
    node.on_localization(odometry(20.25))
    assert node.scan_time==pytest.approx(20.2) and not node.pending_scans
    assert node.odometry.samples.maxlen==16


def test_roi_marker_stays_in_mission_coordinates_and_clears_on_reset(node):
    start(node);node.tracker.valid=True;node.tracker.pose=(.7,-.3,.8)
    messages=[];node.roi_pub=SimpleNamespace(publish=messages.append)
    node.publish_candidate_roi()
    marker=messages[-1].markers[0]
    corners=[compose(node.tracker.pose,(p.x,p.y,0.))[:2] for p in marker.points]
    assert corners==pytest.approx(np.array([(0.,-.2),(3.,-.2),(3.,-2.),(0.,-2.),(0.,-.2)]))
    node.reset_state();assert messages[-1].markers[0].action==3


def test_wait_log_reports_individual_ages_instead_of_hiding_input_loss(node):
    start(node);node.on_localization(odometry(20.));node.on_scan(clear_scan(20.))
    node.on_fusion_health(String(data=json.dumps({'stamp':20.,'ready':True})))
    node.now_seconds=lambda:21.
    from unittest.mock import Mock
    log=Mock();node.get_logger=lambda:log
    node.tick()
    assert node.state=='WAITING_LOCALIZATION'
    assert 'health_age_s=1.00s' in log.info.call_args.args[0]
    assert node.synchronization_status(21.)['scan_queue_capacity']==1


def test_roi_bounds_are_live_and_invalid_ranges_are_rejected(node):
    old=node.cfg
    result=node.set_parameters_atomically([Parameter('candidate_roi_forward_min',value=4.)])
    assert not result.successful and node.cfg is old
    result=node.set_parameters_atomically([Parameter('candidate_roi_forward_max',value=2.5)])
    assert result.successful and node.cfg.candidate_roi_forward_max==2.5


def test_roi_map_flags_are_plain_booleans_serializable_to_ros_status():
    from autorace_parking.slot_map import fit_slot_map
    from test_slot_map import scene
    config=replace(cfg(),slot_map_enabled=True)
    layout=fit_slot_map(scene(origin=(1.5,-1.2))[0],config)
    json.dumps(layout,allow_nan=False)
    assert all(type(c['eligible_empty']) is bool for c in layout['cells'])


def test_arrival_uses_same_roi_candidate_boxes_but_still_checks_all_obstacle_points():
    from autorace_parking.obstacle_memory import MemoryScan
    from autorace_parking.control import Follower
    config=replace(cfg(),automatic_target_selection=True,slot_map_enabled=True,auto_exit_after_parking=False)
    source=MemoryScan.__new__(MemoryScan);source.__dict__=Scan([math.inf]*361,-math.pi,math.pi/180,.01,12.).__dict__.copy()
    # A remote wall's bounding box spans the goal; its actual points are far away.
    source.points=[(-1.5,-.2),(-1.5,.2),(2.5,-.2),(2.5,.2)]
    source.point_owners={p:99 for p in source.points}
    f=Follower(config,require_parking_heading=False)
    f.accept(Gap(1.,-.8,0.,.5,0,1,kind='enclosed_center',confirmed=True,heading_options=(0.,)),[(0.,0.,0.),(1.,-.8,0.)],True,(0.,0.,0.))
    for i in range(3):f.command(source,(1.,-.8,0.),new_scan=True,now=i*.1,obstacle_scan=source)
    assert f.parked
    source.points.append((0.,0.))
    assert not CollisionChecker(source,config).free((0.,0.,0.))


def test_point_collision_mode_arrival_does_not_add_a_solid_bbox_veto():
    from autorace_parking.obstacle_memory import MemoryScan
    from autorace_parking.control import Follower
    config=replace(cfg(),automatic_target_selection=True,slot_map_enabled=True,auto_exit_after_parking=False)
    source=MemoryScan.__new__(MemoryScan);source.__dict__=Scan([math.inf]*361,-math.pi,math.pi/180,.01,12.).__dict__.copy()
    source.points=[(x,y) for x in (-.3,.3) for y in (-.23,.23)]
    source.point_owners={p:99 for p in source.points}
    assert CollisionChecker(source,config).free((0.,0.,0.))
    f=Follower(config,require_parking_heading=False)
    f.accept(Gap(1.,-.8,0.,.5,0,1,kind='enclosed_center',confirmed=True,heading_options=(0.,)),[(0.,0.,0.),(1.,-.8,0.)],True,(0.,0.,0.))
    for i in range(3):f.command(source,(1.,-.8,0.),new_scan=True,now=i*.1,obstacle_scan=source)
    assert f.parked
