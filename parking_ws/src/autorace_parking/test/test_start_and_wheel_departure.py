"""Parking reset transactions and wheel-only departure, isolated from vehicle DDS."""
from concurrent.futures import Future
from types import SimpleNamespace
import json
import math
import time

import pytest
import rclpy
from rclpy.context import Context
from rclpy.parameter import Parameter
from rclpy.signals import SignalHandlerOptions
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

from autorace_parking.node import ParkingNode
from autorace_parking.core import Scan, Settings, CollisionChecker, advance
from autorace_parking.departure import arc_straight_departure
from test_localization_rematch import odom, scan, health


@pytest.fixture
def node():
    context=Context()
    rclpy.init(context=context,domain_id=227,signal_handler_options=SignalHandlerOptions.NO)
    n=ParkingNode(context=context,parameter_overrides=[
        Parameter('localization_source',value='fused'),
        Parameter('white_roi_enabled',value=True),
        Parameter('assume_unobserved_free',value=True)])
    n.control_timer.cancel()
    clock=[100.]
    n.now_seconds=lambda:clock[0]
    n.commands=[]
    n.publish_command=lambda v,a:n.commands.append((v,a))
    yield n,clock
    n.destroy_node();context.shutdown()


def wheel(n,clock,stamp,v=0.,w=0.):
    clock[0]=stamp
    msg=Odometry()
    msg.header.stamp.sec=int(stamp)
    msg.header.stamp.nanosec=round((stamp-int(stamp))*1e9)
    msg.child_frame_id='base_link'
    msg.twist.twist.linear.x=v
    msg.twist.twist.angular.z=w
    n.on_measured_wheel(msg)


class Client:
    def __init__(self,response=None):self.calls=[];self.response=response
    def service_is_ready(self):return True
    def call_async(self,request):
        self.calls.append(request)
        future=Future();future.set_result(self.response)
        return future


def test_start_resets_latched_failure_and_retries_until_fresh_ekf(node):
    n,clock=node
    n.manual_interrupted=n.motion_failed=True
    n.pose_reset_client=Client(SimpleNamespace())
    n.request_parking_start()
    assert not n.manual_interrupted and not n.motion_failed
    assert n.attempt==1 and n.start_phase=='STOP'
    for t in (100.,100.01,100.02):
        wheel(n,clock,t);assert n.tick_start_sequence()
    assert n.start_phase=='SETPOSE'
    n.tick_start_sequence()
    assert n.start_phase=='POSE_RESPONSE' and len(n.pose_reset_client.calls)==1
    n.tick_start_sequence()
    assert n.start_phase=='POSE_WAIT'
    assert all(v==0 for v,a in n.commands)
    n.start_deadline=0
    n.tick_start_sequence()
    assert n.start_phase=='RETRY' and n.start_retries==1
    n.start_retry_at=0
    for t in (100.03,100.04,100.05):
        wheel(n,clock,t);n.tick_start_sequence()
    n.tick_start_sequence();n.tick_start_sequence()
    assert len(n.pose_reset_client.calls)==2 and n.start_phase=='POSE_WAIT'
    clock[0]=100.06
    n.on_localization(odom(100.06));n.on_scan(scan(100.06))
    assert n.tracker.valid
    n.tick_start_sequence()
    assert n.start_phase=='IDLE' and n.ready_until>time.monotonic()


def test_duplicate_start_is_idempotent_then_manual_failure_can_restart(node):
    n,clock=node
    n.request_parking_start()
    n.request_parking_start()
    assert n.attempt==1
    n.sign_ignore_until=0
    n.start_phase='IDLE'
    n.manual_interrupted=True
    n.on_sign(Bool(data=True));n.on_sign(Bool(data=True))
    n.floor_gate.observe(n.floor_gate.armed_stamp+1,.09,0.,1.)
    assert n.attempt==2 and n.start_phase=='STOP' and not n.manual_interrupted


def test_departure_ignores_ekf_jump_but_measured_wheel_moves_pose(node):
    n,clock=node
    n.activation.active=True
    n.on_localization(odom(100.));n.on_scan(scan(100.))
    n.follower.parked=True
    n.lock_departure_localization()
    before=n.tracker.pose
    wheel(n,clock,100.05,-.1)
    bad=odom(100.05,99.)
    n.on_localization(bad)
    bad_health=health(100.05)
    value=json.loads(bad_health.data);value['ready']=False
    bad_health.data=json.dumps(value)
    n.on_fusion_health(bad_health)
    n.on_scan(scan(100.05))
    assert n.departure_wheel and n.localization_ready(100.05)
    assert n.tracker.pose[0]<before[0] and abs(n.tracker.pose[0])<.02
    assert not n.odometry.discontinuity
    assert not n.localization_ready(101.)  # wheel loss still stops


def test_floor_uses_ratio_only_and_yellow_is_optional(node):
    n,clock=node
    from autorace_parking.floor_gate import ParkingFloorGate
    gate=n.floor_gate
    n.floor_gate=gate
    starts=[];gate.start=lambda:starts.append(True)
    gate.on_sign(Bool(data=True));gate.on_sign(Bool(data=True))
    gate.observe(gate.armed_stamp+1,.09,0.,1.)
    assert starts==[True]  # no width/angle/two-image requirement
    source=Scan([math.inf]*361,-math.pi,2*math.pi/360,.01,5.)
    cfg=Settings(assume_unobserved_free=True,max_range=8.)
    assert CollisionChecker(source,cfg).free((1.,0.,0.))
    source.yellow_segments=[((0.,1.),(3.,1.),-1)]
    checker=CollisionChecker(source,cfg)
    assert checker.free((1.,0.,0.))
    assert not checker.free((1.,.95,0.))
    assert checker.last_block=='yellow_boundary'
    assert checker.free((4.,1.1,0.))  # observed segment is finite


def test_arc_then_straight_and_obstacle_still_blocks():
    cfg=Settings(assume_unobserved_free=True,max_range=8.,obstacle_min_cluster_size=0.)
    source=Scan([math.inf]*361,-math.pi,2*math.pi/360,.01,8.)
    pose=advance((1.2,0.,0.),1.,1.)
    route,reason=arc_straight_departure([(0.,0.,0.)],pose,source,cfg)
    assert route and not reason
    assert math.dist(route[-1][:2],(0.,0.))<1e-6 and abs(route[-1][2])<1e-6
    directions=[]
    for a,b in zip(route,route[1:]):
        assert (b[0]-a[0])*math.cos(a[2])+(b[1]-a[1])*math.sin(a[2])<0
        directions.append(abs(b[2]-a[2])>1e-6)
    assert any(directions) and not directions[-1]
    assert sum(a!=b for a,b in zip(directions,directions[1:]))==1
    source.points=[(0.,0.)]
    assert not arc_straight_departure([(0.,0.,0.)],pose,source,cfg)[0]


def test_gyro_completion_precedes_pose_reset_and_does_not_use_old_generation(node):
    n,clock=node
    n.parking_calibration_stop=True
    n.approach_calibration_client=Client(SimpleNamespace(success=True,message='{"generation":7}'))
    n.pose_reset_client=Client(SimpleNamespace())
    n.request_parking_start()
    for t in (100.,100.01,100.02):
        wheel(n,clock,t);n.tick_start_sequence()
    assert n.start_phase=='GYRO' and len(n.approach_calibration_client.calls)==1
    n.tick_start_sequence()
    assert n.start_phase=='GYRO_WAIT'
    h=health(100.02)
    value=json.loads(h.data)
    value.update(calibration_generation=6,calibrating=False,ready=True)
    h.data=json.dumps(value);n.on_fusion_health(h)
    n.tick_start_sequence()
    assert n.start_phase=='GYRO_WAIT' and not n.pose_reset_client.calls
    clock[0]=100.03
    value.update(stamp=100.03,calibration_generation=7)
    h.data=json.dumps(value);n.on_fusion_health(h)
    wheel(n,clock,100.03)
    n.tick_start_sequence();n.tick_start_sequence()
    assert len(n.pose_reset_client.calls)==1 and n.start_phase=='POSE_RESPONSE'


def test_yellow_is_remembered_until_next_attempt(node):
    n,clock=node
    n.activation.active=True
    n.on_localization(odom(100.));n.on_scan(scan(100.))
    msg=Path();msg.header.frame_id='base_link';msg.header.stamp.sec=100
    for x in (.5,.8,1.1):
        p=PoseStamped();p.pose.position.x=x;p.pose.position.y=1.
        msg.poses.append(p)
    n.on_yellow_boundary(msg)
    assert len(n.yellow_segments)==2
    msg.poses=[];n.on_yellow_boundary(msg)
    assert len(n.yellow_segments)==2
    n.request_parking_start()
    assert not n.yellow_segments


def test_vehicle_status_wait_does_not_count_as_ekf_failure(node):
    n,clock=node
    n.manual_mode_guard=True
    n.control_mode=1
    n.control_mode_received=time.monotonic()-1.1
    assert n.vehicle_automatic()  # one missed 0.5 s status must not cancel reset
    n.control_mode_received=time.monotonic()-2.1
    assert not n.vehicle_automatic()
    assert '차량 상태 지연' in n.vehicle_wait_reason()
    n.request_parking_start()
    n.tick_start_sequence()
    assert n.start_phase=='WAIT_AUTO' and n.start_retries==0
    n.control_mode=3;n.control_mode_received=time.monotonic()
    n.tick_start_sequence()
    assert n.start_phase=='WAIT_AUTO' and n.start_retries==0
    assert 'control_mode=3' in n.vehicle_wait_reason()
    n.control_mode=1;n.control_mode_received=time.monotonic()
    n.tick_start_sequence()
    assert n.start_phase=='STOP' and n.start_retries==0
    assert all(v==0. for v,a in n.commands)


def test_actual_manual_status_still_stops_immediately(node):
    n,clock=node
    n.manual_mode_guard=True
    n.control_mode=1;n.control_mode_received=time.monotonic()
    n.activation.active=True
    n.on_vehicle_status(SimpleNamespace(control_mode=3))
    assert not n.vehicle_automatic() and n.manual_interrupted
    assert n.commands[-1]==(0.,0.)


def test_set_pose_ack_discards_old_map_and_waits_for_new_coordinate_epoch(node):
    from sensor_msgs.msg import LaserScan
    from pathlib import Path
    n,clock=node
    n.memory_enabled=True;n.activation.active=True;n.start_phase='STOP'
    data=json.loads((Path(__file__).parent/'data/parking_three_object_20261115.json').read_text())
    def measured_scan(stamp):
        m=LaserScan();m.header.frame_id=data['frame']
        m.header.stamp.sec=int(stamp);m.header.stamp.nanosec=round((stamp-int(stamp))*1e9)
        for key in ('ranges','angle_min','angle_increment','range_min','range_max'):setattr(m,key,data[key])
        return m
    n.on_fusion_health(health(100.));n.on_localization(odom(100.));n.on_scan(measured_scan(100.))
    assert n.tracker.valid and not n.memory.cells  # do not map in the old frame
    n.memory.cells[(0,0)]=[9.,9.,None,0]  # old/failed attempt must also be erased
    n.slot_map={'old':True}
    n.start_phase='POSE_RESPONSE';n.start_deadline=time.monotonic()+10
    n.start_future=Future();n.start_future.set_result(SimpleNamespace())
    old_tracker=n.tracker
    old_scan_job=Future()
    n.scan_job=(old_scan_job,n.epoch,measured_scan(100.),time.monotonic())
    wheel(n,clock,100.)
    n.tick_start_sequence()
    assert n.start_phase=='POSE_WAIT' and not n.memory.cells and n.slot_map is None
    assert n.tracker is not old_tracker and not n.odometry.samples
    assert n.scan_job is None and old_scan_job.cancelled()
    n.on_localization(odom(99.9));n.on_scan(measured_scan(99.9))
    assert not n.odometry.samples and not n.memory.cells
    clock[0]=100.1
    n.on_localization(odom(100.1));n.on_scan(measured_scan(100.1))
    assert n.tracker.valid and n.memory.cells  # only the new frame can seed the map


def test_normal_progress_ignores_new_sign_but_failure_allows_restart(node,monkeypatch):
    import autorace_parking.node as parking_module
    wall=[100.]
    monkeypatch.setattr(parking_module.time,'monotonic',lambda:wall[0])
    n,clock=node
    n.prevent_active_retrigger=True
    n.request_parking_start()
    n.start_phase='IDLE';n.sign_ignore_until=0.
    n.memory.cells[(0,0)]=[1.,1.,None,0]
    n.on_sign(Bool(data=True));n.on_sign(Bool(data=True))
    n.request_parking_start()  # bypassing the camera callback must not cancel either
    assert n.attempt==1 and n.memory.cells and not n.floor_gate.armed
    n.follower.parked=True
    assert not n.accepts_parking_start()  # hold and reverse departure are normal progress
    n.manual_interrupted=True
    wall[0]+=1.01  # Set grouping uses monotonic receipt time, not ROS time.
    n.on_sign(Bool(data=True));n.on_sign(Bool(data=True))
    n.floor_gate.observe(n.floor_gate.armed_stamp+1,.09,0.,0.)
    assert n.attempt==2 and n.start_phase=='STOP'


def test_departure_prefers_calibrated_imu_and_keeps_delayed_scan_history(node):
    from sensor_msgs.msg import Imu
    n,clock=node
    n.activation.active=True
    n.on_localization(odom(100.));n.on_scan(scan(100.));n.follower.parked=True
    n.lock_departure_localization()
    assert n.odometry.samples.maxlen==64
    for i in range(1,41):
        stamp=100.+i*.02;clock[0]=stamp
        imu=Imu();imu.header.frame_id='base_link';imu.header.stamp.sec=int(stamp)
        imu.header.stamp.nanosec=round((stamp-int(stamp))*1e9);imu.angular_velocity.z=.2
        n.on_departure_imu(imu)
        wheel(n,clock,stamp,-.1,.4)  # legacy wheel yaw is different from the calibrated gyro
    assert n.odometry.samples[-1].yaw_rate==.2
    n.on_scan(scan(100.45))  # 350 ms scan delivery delay at a 50 Hz wheel input
    assert n.tracker.valid and n.localization_ready(100.8)
    count=len(n.odometry.samples)
    n.on_localization(odom(100.8,99.))
    assert len(n.odometry.samples)==count and not n.odometry.discontinuity
