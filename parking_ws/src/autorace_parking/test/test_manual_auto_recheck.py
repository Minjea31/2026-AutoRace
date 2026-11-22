"""A mode edge rechecks parking only when parking owned the mux session."""
from concurrent.futures import Future
import time
from types import SimpleNamespace

from std_msgs.msg import String
from autorace_parking.core import Gap
from test_operator_ros import node,activate


def automatic(node):
    node.manual_mode_guard=True
    node.control_mode=1
    node.control_mode_received=time.monotonic()
    activate(node)


def test_parking_manual_to_auto_discards_old_pose_map_and_path_once(node):
    automatic(node)
    node.on_mux_active_source(String(data='parking'))
    old_goal=Gap(1.,-.5,0.,.5,0,1,confirmed=True)
    node.tracked_goal=node.follower.goal=old_goal
    node.follower.path=[(0.,0.,0.),(1.,-.5,0.)]
    node.slot_map={'old':True}
    node.memory.cells[(0,0)]=[0.,0.,1,0]
    future=Future();node.pending=future,node.epoch,(0.,0.,0.),20.
    previous_attempt=node.attempt;previous_epoch=node.epoch;previous_tracker=node.tracker
    commands=[];node.publish_command=lambda v,a:commands.append((v,a))
    node.on_vehicle_status(SimpleNamespace(control_mode=3))
    assert node.manual_interrupted and node.manual_resume_owned
    assert not node.follower.path and commands[-1]==(0.,0.)
    node.on_mux_active_source(String(data='stop'))
    # The sign cooldown is intentionally still active during this mode edge.
    assert node.sign_ignore_until>time.monotonic()
    node.on_vehicle_status(SimpleNamespace(control_mode=1))
    assert node.attempt==previous_attempt+1 and node.start_phase=='STOP'
    assert not node.manual_interrupted and not node.manual_resume_owned
    assert node.slot_map is None and node.tracked_goal is None and not node.memory.cells
    assert node.scan is None and node.tracker is not previous_tracker
    assert not node.odometry.samples and node.scan_not_before==20.
    assert future.cancelled() and node.epoch>previous_epoch
    assert node.operation['state']=='WAITING_EKF' and all(v==0. for v,_ in commands)
    node.on_vehicle_status(SimpleNamespace(control_mode=1))
    assert node.attempt==previous_attempt+1


def test_recent_guard_stop_retains_the_preceding_parking_owner(node):
    automatic(node);node.on_mux_active_source(String(data='parking'))
    node.on_mux_active_source(String(data='stop'))
    node.on_vehicle_status(SimpleNamespace(control_mode=3))
    assert node.manual_resume_owned


def test_other_mux_source_cancels_ownership_even_if_it_later_stops(node):
    automatic(node);node.on_mux_active_source(String(data='lane'))
    attempt=node.attempt
    node.on_vehicle_status(SimpleNamespace(control_mode=3))
    node.on_vehicle_status(SimpleNamespace(control_mode=1))
    assert node.attempt==attempt and node.manual_interrupted
    node.manual_interrupted=False
    node.on_mux_active_source(String(data='parking'))
    node.on_mux_active_source(String(data='lane'))
    node.on_mux_active_source(String(data='stop'))
    node.on_vehicle_status(SimpleNamespace(control_mode=3))
    assert not node.manual_resume_owned


def test_failed_parking_repositioned_at_stop_line_restarts_ekf_after_a_long_manual_pause(node):
    from unittest.mock import patch
    automatic(node);node.on_mux_active_source(String(data='parking'))
    node.motion_failed=True;node.state='MISSION_FAILED'
    node.on_mux_active_source(String(data='stop'))
    attempt=node.attempt;wall=time.monotonic()
    with patch('autorace_parking.node.time.monotonic',return_value=wall+60.):
        node.on_vehicle_status(SimpleNamespace(control_mode=3))
    assert node.manual_interrupted and node.manual_resume_owned
    # The vehicle is carried to the original stop line during two minutes of
    # manual mode. Its old mission origin, map and route must not be reused.
    node.now_seconds=lambda:1000.
    with patch('autorace_parking.node.time.monotonic',return_value=wall+180.):
        node.on_vehicle_status(SimpleNamespace(control_mode=1))
    assert node.attempt==attempt+1 and node.start_phase=='STOP'
    assert not node.motion_failed and not node.manual_interrupted
    assert node.pose_reset_stamp<=node.scan_not_before==node.localization_not_before==1000.
    assert node.scan is None and node.tracked_goal is None and node.slot_map is None
    assert node.operation['state']=='WAITING_EKF'


def test_completed_parking_does_not_restart_on_a_mode_change(node):
    automatic(node);node.on_mux_active_source(String(data='parking'))
    node.follower.exited=True;attempt=node.attempt
    node.on_vehicle_status(SimpleNamespace(control_mode=3))
    node.on_vehicle_status(SimpleNamespace(control_mode=1))
    assert node.attempt==attempt and node.follower.exited


def test_another_mission_taking_the_mux_during_manual_control_cancels_resume(node):
    automatic(node);node.on_mux_active_source(String(data='parking'))
    node.on_vehicle_status(SimpleNamespace(control_mode=3));attempt=node.attempt
    node.on_mux_active_source(String(data='lane'))
    node.on_vehicle_status(SimpleNamespace(control_mode=1))
    assert node.attempt==attempt and node.manual_interrupted and not node.manual_resume_owned


def test_auto_return_uses_an_already_pending_ekf_reset(node):
    automatic(node);node.on_mux_active_source(String(data='parking'))
    node.start_phase='WAIT_AUTO';attempt=node.attempt
    node.on_vehicle_status(SimpleNamespace(control_mode=3))
    node.on_vehicle_status(SimpleNamespace(control_mode=1))
    assert node.attempt==attempt and node.start_phase=='WAIT_AUTO'
    assert node.tick_start_sequence() and node.start_phase=='STOP'


def test_fused_guard_subscribes_to_real_mode_and_mux_and_waits_for_new_ekf():
    import rclpy
    from rclpy.parameter import Parameter
    from autorace_parking.node import ParkingNode
    rclpy.init()
    n=ParkingNode(parameter_overrides=[Parameter('localization_source',value='fused'),
        Parameter('manual_mode_guard',value=True)])
    try:
        topics={s.topic_name for s in n.subscriptions}
        assert '/limo_status' in topics and '/mux/active_source' in topics
        n.now_seconds=lambda:20.;n.activation.active=True
        n.control_mode=1;n.control_mode_received=time.monotonic()
        n.on_mux_active_source(String(data='parking'))
        n.on_vehicle_status(SimpleNamespace(control_mode=3))
        n.on_vehicle_status(SimpleNamespace(control_mode=1))
        assert n.start_phase=='STOP' and not n.tracker.valid
        assert n.operation['state']=='WAITING_EKF' and not n.operation['error_code']
    finally:
        n.destroy_node();rclpy.shutdown()
