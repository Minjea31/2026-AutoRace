"""Regression of real-bag start loss, late scan jobs and manual completion."""
from concurrent.futures import Future
from dataclasses import replace
from types import SimpleNamespace
import math,time
import pytest
import rclpy
from rclpy.parameter import Parameter
from std_msgs.msg import Bool,Float64MultiArray
from autorace_parking.node import ParkingNode
from autorace_parking.core import Settings,Gap,Scan
from autorace_parking.control import Follower

@pytest.fixture
def node():
    rclpy.init()
    n=ParkingNode(parameter_overrides=[Parameter('localization_source',value='scan'),
        Parameter('prevent_active_retrigger',value=True),Parameter('retain_parking_control',value=True)])
    try:yield n
    finally:n.destroy_node();rclpy.shutdown()


def test_active_sign_does_not_reset_route_after_cooldown(node):
    for _ in range(2):node.on_sign(Bool(data=True))
    f=node.follower;f.path=[(0.,0.,0.),(1.,0.,0.)];node.sign_ignore_until=0.
    for _ in range(5):node.on_sign(Bool(data=True))
    assert node.follower is f and len(f.path)==2 and node.attempt==1


def test_pending_plan_keeps_zero_lease(node):
    commands=[];node.publish_command=lambda v,a:commands.append((v,a))
    for _ in range(2):node.on_sign(Bool(data=True))
    node.tick();node.tick()
    assert commands[-2:]==[(0.,0.),(0.,0.)]


def test_manual_takeover_latches_and_cannot_complete_on_recovery(node):
    node.manual_mode_guard=True;node.control_mode=1;node.control_mode_received=time.monotonic()
    for _ in range(2):node.on_sign(Bool(data=True))
    node.follower.parked=True
    node.on_vehicle_status(SimpleNamespace(control_mode=3))
    assert node.manual_interrupted and not node.follower.parked
    node.on_vehicle_status(SimpleNamespace(control_mode=1));node.tick()
    assert node.state=='MANUAL_INTERRUPTED' and not node.follower.exited


def test_async_scan_results_from_old_epoch_are_not_committed(node):
    future=Future();future.set_result((node.tracker,None,None,[],True))
    msg=SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=int(node.now_seconds()),nanosec=0)))
    node.scan_job=(future,node.epoch-1,msg,time.monotonic())
    tracker=node.tracker;node.poll_scan_match()
    assert node.tracker is tracker and node.scan is None


def test_departure_failure_is_bounded():
    cfg=replace(Settings(),exit_max_plan_attempts=3,exit_retry_timeout=6.)
    f=Follower(cfg);f.exit_planner=lambda *a:([],'no_reverse_route')
    scan=Scan([math.inf]*361,-math.pi,math.pi/180,.01,5.)
    for i in range(12):f.rebuild_exit((0.,0.,0.),i*.5,scan)
    assert f.exit_retry_exhausted and f.exit_plan_attempts==3 and f.exit_route_reason=='retry_limit'


def test_incomplete_health_cannot_start_calibration_or_crash(node):
    node.parking_calibration_stop=True
    node.approach_calibration.start(node.now_seconds(),time.monotonic())
    node.fusion_health={'stamp':node.now_seconds(),'ready':False,'wheel_speed_m_s':None,'wheel_yaw_rate_rad_s':None}
    node.fusion_health_received=time.monotonic()
    node.poll_approach_calibration()
    assert not node.approach_calibration.requested


def test_pending_slow_scan_does_not_block_stop_publication(node):
    future=Future();msg=SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=int(node.now_seconds()),nanosec=0)))
    node.scan_job=(future,node.epoch,msg,time.monotonic())
    commands=[];node.publish_command=lambda v,a:commands.append((v,a))
    node.activation.active=True
    for _ in range(4):node.tick()
    assert len(commands)>=4 and all(v==0 for v,a in commands)


def test_live_tuning_cannot_allow_center_offset_outside_arrival_tolerance(node):
    result=node.on_tuning_parameters([Parameter('parking_center_adjustment',value=.05),
        Parameter('parking_position_tolerance',value=.04)])
    assert not result.successful
    assert node.cfg.parking_center_adjustment==0.


def test_successful_short_routes_do_not_replenish_departure_retry_budget():
    f=Follower(replace(Settings(),exit_max_plan_attempts=3));f.goal=Gap(0.,0.,0.,.5,0,1)
    calls=[]
    def plan(*args):
        calls.append(1)
        return [(0.,0.,0.),(-.2,0.,0.)],''
    f.exit_planner=plan
    scan=Scan([math.inf]*361,-math.pi,math.pi/180,.01,5.)
    for i in range(10):f.rebuild_exit((0.,0.,0.),i*.5,scan)
    assert len(calls)==3 and f.exit_retry_exhausted
