from dataclasses import replace
import math
from unittest.mock import patch
import pytest
from autorace_parking.core import Settings,Scan,Gap
from autorace_parking.obstacle_memory import MemoryScan
from autorace_parking.control import Follower,build_plan_report


def config():
    return replace(Settings(),allow_reverse=True,recovery_uses_memory_map=True,recovery_on_front_obstacle_only=True,
        recovery_reverse_distance=.3,recovery_total_distance=.3,recovery_speed=.2,max_recovery_attempts=1,
        blind_reverse_distance=0.,assume_unobserved_free=False,initial_clear_radius=0.,
        continuous_replanning=True,require_complete_entry_path=True)


def source(points=()):
    scan=MemoryScan.__new__(MemoryScan);scan.__dict__=Scan([0.]*181,-math.pi/2,math.pi/180,.01,12.,(0.,0.,0.)).__dict__.copy()
    scan.points=list(points);scan.point_owners={};return scan


def raw_front():
    scan=Scan([0.]*181,-math.pi/2,math.pi/180,.01,12.,(0.,0.,0.));scan.points=[(.25,0.)];return scan


def test_unobserved_rear_absent_from_memory_allows_thirty_cm_with_zero_blind_budget():
    cfg=config();result=build_plan_report(source(),cfg,Gap(.5,-.3,0.,.6,0,1),front_scan=raw_front())
    assert result['diagnostics']['reason_code']=='recovery_ready'
    assert result['diagnostics']['recovery_uses_memory_map']
    assert result['path'][-1]==pytest.approx((-.3,0.,0.))
    assert result['diagnostics']['blind_reverse_distance_m']==0.


def test_remembered_rear_obstacle_blocks_reverse_even_when_raw_scan_cannot_see_it():
    result=build_plan_report(source([(-.22,0.)]),config(),Gap(.5,-.3,0.,.6,0,1),front_scan=raw_front())
    assert not result['path'] and result['diagnostics']['recovery_blocked_by']=='obstacle'


def test_exhausted_mission_recovery_limit_still_blocks_map_clear_reverse():
    result=build_plan_report(source(),config(),Gap(.5,-.3,0.,.6,0,1),front_scan=raw_front(),recovery_allowed=False)
    assert not result['path'] and result['diagnostics']['recovery_limit_reached']


def test_raw_rear_return_is_not_used_but_new_memory_obstacle_stops_motion():
    cfg=config();result=build_plan_report(source(),cfg,Gap(.5,-.3,0.,.6,0,1),front_scan=raw_front())
    f=Follower(cfg,speed=.25,min_turn_speed=.2);f.accept(result['goal'],result['path'],False,(0.,0.,0.),recovery=True)
    raw=raw_front();raw.points=[(-.22,0.)]
    for t in (0.,.1,.2,.3):v,a,state=f.command(raw,(0.,0.,0.),True,t,obstacle_scan=source())
    assert -.2<=v<0 and a==0 and state=='RECOVERING'
    v,a,state=f.command(raw,(0.,0.,0.),True,.4,obstacle_scan=source([(-.22,0.)]))
    assert v==a==0 and state=='RECOVERY_BLOCKED'


def test_node_accepts_map_clear_reverse_and_rechecks_the_current_map():
    import rclpy
    from rclpy.parameter import Parameter
    from concurrent.futures import Future
    from autorace_parking.node import ParkingNode
    rclpy.init();n=ParkingNode(parameter_overrides=[Parameter('localization_source',value='scan'),
        Parameter('obstacle_memory_enabled',value=True),Parameter('allow_reverse',value=True),
        Parameter('recovery_uses_memory_map',value=True),Parameter('recovery_on_front_obstacle_only',value=True),
        Parameter('recovery_reverse_distance',value=.3),Parameter('recovery_total_distance',value=.3),
        Parameter('max_recovery_attempts',value=1),Parameter('blind_reverse_distance',value=0.),
        Parameter('assume_unobserved_free',value=False)])
    try:
        n.activation.active=True;n.now_seconds=lambda:20.;n.scan=raw_front();n.scan_time=20.;n.memory.stamp=20.
        goal=Gap(.5,-.3,0.,.6,0,1);report=build_plan_report(source(),config(),goal,front_scan=n.scan)
        for blocked in (True,False):
            n.planning_scan=lambda:source([(-.22,0.)] if blocked else [])
            f=Future();f.set_result(report);n.pending=f,n.epoch,(0.,0.,0.),20.;n.poll_plan()
            if blocked:assert n.recovery_attempts==0 and not n.follower.recovering
            else:
                assert n.follower.recovering and n.recovery_attempts==1
                assert n.recovery_remaining==pytest.approx(0.) and n.blind_reverse_remaining==0.
    finally:n.destroy_node();rclpy.shutdown()
