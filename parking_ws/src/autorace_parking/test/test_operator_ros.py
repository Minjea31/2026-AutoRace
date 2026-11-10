"""Korean diagnostics and recovery bookkeeping at the ROS adapter boundary."""
from concurrent.futures import Future
import json
import math
import time
from types import SimpleNamespace
from unittest.mock import Mock, patch

import pytest
import rclpy
from rclpy.parameter import Parameter
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

from autorace_parking.control import build_plan_report
from autorace_parking.core import Scan
from autorace_parking.node import ParkingNode


@pytest.fixture
def node():
    rclpy.init()
    n = ParkingNode(parameter_overrides=[Parameter('localization_source', value='scan'),
                                        Parameter('clearance', value=.04),
                                        Parameter('allow_reverse', value=True)])
    n.now_seconds = lambda: 20.
    try:
        yield n
    finally:
        n.destroy_node()
        rclpy.shutdown()


def activate(node):
    for _ in range(3):
        node.on_sign(Bool(data=True))


def test_invalid_scan_keeps_specific_korean_explanation_in_status(node):
    messages = []
    node.status_pub = SimpleNamespace(publish=lambda m: messages.append(json.loads(m.data)))
    activate(node)
    msg = LaserScan(); msg.header.frame_id = 'wrong_frame'; msg.header.stamp.sec = 20
    msg.angle_increment = .1; msg.range_min = .01; msg.range_max = 12.; msg.ranges = [1.]*20
    node.on_scan(msg); node.tick()
    result = messages[-1]
    assert result['state'] == 'WAITING_SCAN'
    assert result['state_text'] == '라이다 수신 대기'
    assert '좌표계' in result['reason']
    assert 'Waiting' not in result['reason']


def test_persistent_stop_is_repeated_in_korean_after_five_seconds(node):
    logger = Mock(); node.get_logger = lambda: logger
    activate(node); node.tick()
    before = logger.info.call_count
    node.last_operator_log_wall = time.monotonic()-5.1
    node.tick()
    assert logger.info.call_count == before+1
    assert '정지 이유' in logger.info.call_args.args[0]
    assert '라이다' in logger.info.call_args.args[0]


def test_recovery_plan_without_a_target_is_accepted_and_counted(node):
    activate(node)
    scan = Scan([10.]*361, -math.pi, math.pi/180, .01, 12., (0., 0., 0.))
    node.scan=scan
    future = Future(); future.set_result(build_plan_report(scan, node.cfg))
    node.pending = future, node.epoch, (0., 0., 0.), 20.
    node.poll_plan()
    assert node.recovery_attempts == 1 and node.recovery_running
    assert node.follower.recovering and node.follower.path
    assert node.follower.goal is None and not node.follower.parked
    node.reset_state()
    assert node.recovery_attempts == 0 and not node.recovery_running


@pytest.mark.parametrize('speed,state',[(.2,'PARKING'),(-.2,'RECOVERING')])
def test_stationary_vehicle_keeps_parking_ownership_and_commands(node,speed,state):
    messages=[];commands=[]
    node.status_pub=SimpleNamespace(publish=lambda m:messages.append(json.loads(m.data)))
    node.publish_command=lambda speed,steering:commands.append((speed,steering))
    activate(node)
    node.scan=Scan([10.]*361,-math.pi,math.pi/180,.01,12.)
    node.scan_time=20.
    node.follower.path=[(0.,0.,0.),(.5,0.,0.)]
    node.follower.final_approach=True
    node.follower.measured_speed=0.
    node.follower.command=Mock(return_value=(speed,0.,state))
    wall_start=time.monotonic()
    for elapsed in (0.,3.,30.):
        with patch('autorace_parking.node.time.monotonic',return_value=wall_start+elapsed):
            node.tick()
    assert commands==[(speed,0.)]*3
    assert node.activation.active and node.state==state
    assert node.follower.command.call_count==3
    assert messages[-1]['speed']==speed and node.reason==''


def test_clock_reset_reason_survives_return_to_waiting_sign(node):
    messages = []
    node.status_pub = SimpleNamespace(publish=lambda m: messages.append(json.loads(m.data)))
    activate(node); node.last_now = 30.
    node.tick()
    assert messages[-1]['state'] == 'WAITING_SIGN'
    assert '시각이 뒤로' in messages[-1]['reason']
    assert not node.activation.active


def test_inactive_ticks_clear_cached_rviz_path(node):
    paths=[];boxes=[]
    node.path_pub=SimpleNamespace(publish=paths.append)
    node.obstacles_pub=SimpleNamespace(publish=boxes.append)
    node.tick();node.tick()
    assert len(paths)==2 and all(not msg.poses for msg in paths)
    assert not boxes  # Keep fresh scan boxes visible while waiting for a sign.
    node.clear_visuals()  # Explicit reset/invalid scan still clears everything.
    assert boxes[-1].markers[0].action==3


@pytest.mark.parametrize('clearance,goal_x,attempts,accepted',[
    (.08,1.,0,True),(.11,1.,0,False),(.08,.05,0,False),(.08,1.,1,False),
])
def test_recovery_acceptance_rechecks_current_front_arrival_and_attempt_limit(node,clearance,goal_x,attempts,accepted):
    from dataclasses import replace
    from autorace_parking.core import Gap
    activate(node)
    node.cfg=replace(node.cfg,recovery_on_front_obstacle_only=True,
                     recovery_front_clearance=.1,max_recovery_attempts=1)
    node.scan=Scan([math.inf]*361,-math.pi,math.pi/180,.01,12.,(0.,0.,0.))
    node.scan.points=[(node.cfg.vehicle_length/2+clearance,0.)]
    node.recovery_attempts=attempts
    future=Future()
    future.set_result({'goal':Gap(goal_x,0.,0.,.85,0,1),
                       'path':[(0.,0.,0.),(-.3,0.,0.)],'final':False,
                       'diagnostics':{'reason_code':'recovery_ready'}})
    node.pending=future,node.epoch,(0.,0.,0.),20.
    node.poll_plan()
    assert bool(node.follower.path)==accepted
    assert node.recovery_attempts==attempts+int(accepted)


def test_true_after_five_seconds_restarts_from_new_scan_and_replans(node):
    from autorace_parking.core import Gap
    commands=[];active=[]
    node.publish_command=lambda speed,steering:commands.append((speed,steering))
    node.active_pub=SimpleNamespace(publish=lambda msg:active.append(msg.data))
    with patch('autorace_parking.node.time.monotonic',return_value=100.):
        activate(node)
    first=node.follower;first.goal=Gap(1.,-.5,0.,.5,0,1)
    first.path=[(0.,0.,0.),(.5,0.,0.)]
    first.entry_trace=first.path.copy()
    node.recovery_attempts=1;node.blind_reverse_remaining=0.
    node.observed_corridor.append((20.,(.2,0.,0.)))
    old_epoch=node.epoch;future=Future()
    node.pending=future,old_epoch,(0.,0.,0.),20.
    with patch('autorace_parking.node.time.monotonic',return_value=104.999):
        node.on_sign(Bool(data=True))
    assert node.follower is first and node.attempt==1
    with patch('autorace_parking.node.time.monotonic',return_value=105.):
        node.on_sign(Bool(data=False))
        assert node.follower is first
        node.on_sign(Bool(data=True))
    assert commands==[(0.,0.)] and active==[True,True]
    assert node.activation.active and node.attempt==2 and node.epoch==old_epoch+1
    assert node.follower is not first and node.follower.goal is None
    assert not node.follower.path and not node.follower.entry_trace
    assert node.pending is None and future.cancelled()
    assert not node.observed_corridor and node.recovery_attempts==0
    assert node.blind_reverse_remaining==node.cfg.blind_reverse_distance
    assert node.scan is None and node.sign_ignore_until==110.
    msg=LaserScan();msg.header.frame_id='base_link';msg.header.stamp.sec=19;msg.header.stamp.nanosec=900000000
    msg.angle_min=-math.pi;msg.angle_increment=math.pi/180
    msg.range_min=.01;msg.range_max=12.;msg.ranges=[1.]*361
    node.on_scan(msg)
    assert node.scan is None  # A queued frame from before the restart is not reused.
    msg.header.stamp.sec=20;msg.header.stamp.nanosec=0
    node.on_scan(msg)
    assert node.scan is not None
    with patch.object(node.worker,'submit',return_value=Future()) as submit:
        node.tick()
        assert submit.call_args.args[1] is node.scan
        assert submit.call_args.args[3] is None  # Start with no old goal.
