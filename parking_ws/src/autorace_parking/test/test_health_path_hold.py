"""Heartbeat delay keeps a checked path; pose/scan faults still hold motion."""
from dataclasses import replace
import json
import math
import time

import pytest
import rclpy
from rclpy.parameter import Parameter
from std_msgs.msg import String

from autorace_parking.core import Gap
from autorace_parking.node import ParkingNode
from test_localization_rematch import odom, scan, health


@pytest.fixture
def scenario():
    rclpy.init()
    n = ParkingNode(parameter_overrides=[
        Parameter('localization_source', value='fused'),
        Parameter('localization_preserve_timeout', value=1.),
        Parameter('health_path_hold_timeout', value=2.),
        Parameter('retain_parking_control', value=True),
        Parameter('parking_speed', value=.2),
        Parameter('assume_unobserved_free', value=True),
        Parameter('hold_entry_path', value=True),
    ])
    clock = [100.]
    n.now_seconds = lambda: clock[0]
    commands, guards = [], []
    n.publish_command = lambda v, a: commands.append((v, a))
    n.guard_pub.publish = lambda msg: guards.append(json.loads(msg.data))
    n.activation.active = True
    n.on_localization(odom(100.)); n.on_scan(scan(100.)); n.on_fusion_health(health(100.))
    n.follower.accept(Gap(1., 0., 0., .5, 0, 1),
        [(i*.02, 0., 0.) for i in range(51)], True, (0., 0., 0.))
    # The test follows an accepted route; it never starts an external planner.
    n.plan_period = math.inf
    try:
        yield n, clock, commands, guards
    finally:
        n.destroy_node(); rclpy.shutdown()


def advance_inputs(n, clock, t, *, ekf=True, lidar=True):
    clock[0] = t
    if ekf: n.on_localization(odom(t))
    if lidar: n.on_scan(scan(t))


def test_late_health_follows_same_path_and_mux_remains_ready(scenario):
    n, clock, commands, guards = scenario
    path, tracker = n.follower.path, n.tracker
    for i in range(1, 14):
        advance_inputs(n, clock, 100.+i*.1)
        n.tick()
        assert commands[-1][0] > 0
        assert guards[-1]['ready'] and guards[-1]['phase'] == 'DRIVE'
        assert n.follower.path is path and n.tracker is tracker
        assert not n.localization_recovering
    assert n.health_path_tracking
    status=n.synchronization_status(clock[0])
    assert status['health_path_tracking'] and status['health_path_hold_timeout_s']==2.
    # A delayed queued packet must not erase the path or refresh its timeout.
    received=n.fusion_health_received
    n.on_fusion_health(health(100.5))
    assert n.follower.path is path and n.fusion_health['stamp']==100.
    assert n.fusion_health_received==received
    n.on_fusion_health(health(clock[0]));n.tick()
    assert not n.health_path_tracking and n.follower.path is path
    assert commands[-1][0] > 0 and guards[-1]['ready']


def test_health_hold_expires_to_stop_without_discarding_map(scenario):
    n, clock, commands, guards = scenario
    path, tracker = n.follower.path, n.tracker
    for i in range(1, 23):
        advance_inputs(n, clock, 100.+i*.1);n.tick()
    assert n.state=='WAITING_LOCALIZATION' and commands[-1]==(0.,0.)
    assert not guards[-1]['ready'] and not n.health_path_tracking
    assert n.follower.path is path and n.tracker is tracker and n.odometry.samples
    assert not n.localization_recovering
    n.on_fusion_health(health(clock[0]));n.tick()
    assert n.follower.path is path and commands[-1][0]>0 and guards[-1]['ready']


@pytest.mark.parametrize('fault',['ekf','scan','unready','jump','covariance'])
def test_saved_path_never_bypasses_pose_scan_or_input_faults(scenario,fault):
    n, clock, commands, guards = scenario
    for i in range(1,5):advance_inputs(n,clock,100.+i*.1)
    assert n.can_follow_saved_path(clock[0])
    if fault=='ekf':
        for i in range(5,10):advance_inputs(n,clock,100.+i*.1,ekf=False)
    elif fault=='scan':
        for i in range(5,11):advance_inputs(n,clock,100.+i*.1,lidar=False)
    elif fault=='unready':
        n.on_fusion_health(String(data=json.dumps({'stamp':clock[0],'ready':False,'reason':'IMU unavailable'})))
    else:
        clock[0]+= .1;bad=odom(clock[0],5. if fault=='jump' else 0.)
        if fault=='covariance':bad.pose.covariance[0]=-1.
        n.on_localization(bad)
    assert not n.can_follow_saved_path(clock[0])
    n.tick()
    assert commands[-1]==(0.,0.) and not guards[-1]['ready']


def test_health_wait_keeps_partial_recovery_and_does_not_use_motion_timeout(scenario):
    n, clock, commands, guards = scenario
    n.follower.path=[];n.localization_recovering=True
    tracker=n.tracker;epoch=n.epoch
    n.fusion_health=n.fusion_health_received=None
    n.motion_stall_since=70.
    advance_inputs(n,clock,100.1);n.tick()
    assert n.state=='WAITING_LOCALIZATION' and not n.motion_failed
    assert n.motion_stall_since is None and n.tracker is tracker and n.odometry.samples
    assert n.epoch==epoch and commands[-1]==(0.,0.)
    # Old heartbeat packets do not wipe partially received new scan/odom data.
    n.on_fusion_health(health(99.7))
    assert n.tracker is tracker and n.odometry.samples and n.epoch==epoch
    n.on_fusion_health(health(clock[0]))
    assert n.localization_ready(clock[0]) and n.tracker is tracker
    assert not n.localization_recovering


def test_late_health_does_not_replace_path_with_pending_planner_result(scenario):
    from concurrent.futures import Future
    n, clock, _, _ = scenario
    for i in range(1,5):advance_inputs(n,clock,100.+i*.1)
    future=Future();future.set_result((None,[],False))
    pending=(future,n.epoch,n.tracker.pose,clock[0])
    n.pending=pending;path=n.follower.path
    n.poll_plan()
    assert n.pending is pending and n.follower.path is path



def test_new_obstacle_still_blocks_forward_motion_during_health_delay(scenario):
    n, clock, commands, _ = scenario
    for i in range(1,5):advance_inputs(n,clock,100.+i*.1)
    assert n.can_follow_saved_path(clock[0])
    clock[0]=100.5;n.on_localization(odom(100.5))
    blocked=scan(100.5);blocked.header.frame_id='base_link'
    for i in range(177,184):blocked.ranges[i]=.25
    n.on_scan(blocked);n.tick()
    assert commands[-1][0]<=0.
