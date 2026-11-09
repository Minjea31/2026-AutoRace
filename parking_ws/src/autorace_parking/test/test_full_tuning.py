from concurrent.futures import Future
from dataclasses import replace

import pytest

from autorace_parking.core import Settings,Cluster,Gap,find_gaps
from autorace_parking.diagnostics import planning_text


@pytest.fixture
def node():
    import rclpy
    from rclpy.parameter import Parameter
    from autorace_parking.node import ParkingNode
    rclpy.init();n=ParkingNode(parameter_overrides=[Parameter('localization_source',value='scan')])
    try:yield n
    finally:n.destroy_node();rclpy.shutdown()


def set_values(node,**kwargs):
    from rclpy.parameter import Parameter
    return node.set_parameters_atomically([Parameter(k,value=v) for k,v in kwargs.items()])


def test_selection_tuning_updates_real_config_cancels_stale_worker_and_keeps_route_map_tracker(node):
    follower,memory,tracker=node.follower,node.memory,node.tracker
    path=[(0.,0.,0.),(1.,0.,0.)];follower.accept(Gap(1.,0.,0.,.5,0,1),path,True,(0.,0.,0.))
    f=Future();node.pending=f,node.epoch,(0.,0.,0.),20.;epoch=node.epoch
    result=set_values(node,min_gap=.35,max_gap=.8,row_tolerance=.2,min_cluster_size=.1,cluster_min_points=4)
    assert result.successful,result.reason
    assert (node.cfg.min_gap,node.cfg.max_gap,node.cfg.row_tolerance)==(.35,.8,.2)
    assert node.follower.cfg is node.cfg and node.cfg.cluster_min_points==4
    assert f.cancelled() and node.pending is None and node.epoch==epoch+1
    assert node.follower is follower and follower.path==path
    assert node.memory is memory and node.tracker is tracker


def test_invalid_cross_parameter_batch_is_rejected_without_partial_changes(node):
    before=node.cfg
    result=set_values(node,steering_gain=.8,min_gap=.9,max_gap=.7)
    assert not result.successful and node.cfg is before and node.follower.cfg is before
    assert node.get_parameter('steering_gain').value==1.
    assert node.get_parameter('min_gap').value==0.


def test_controller_and_mission_hyperparameters_are_live_and_timer_is_replaced(node):
    timer=node.control_timer
    result=set_values(node,parking_speed=.4,min_turn_speed=.15,lookahead=.3,
                      recognition_frames=4,control_hz=25.,max_steering_rate=.5,require_path_heading=False)
    assert result.successful,result.reason
    assert node.follower.speed==.4 and node.follower.min_turn_speed==.15
    assert node.follower.lookahead==.3 and node.follower.max_steering_rate==.5
    assert node.activation.required==4 and node.control_hz==25. and node.control_timer is not timer
    assert not node.cfg.require_goal_heading


def test_hold_toggle_is_live_without_changing_path(node):
    path=[(0.,0.,0.),(1.,0.,0.)];node.follower.accept(Gap(1.,0.,0.,.5,0,1),path,True,(0.,0.,0.))
    assert set_values(node,hold_entry_path=True).successful
    assert node.follower.entry_locked and node.follower.entry_path==path
    assert set_values(node,hold_entry_path=False).successful
    assert not node.follower.entry_locked and node.follower.path==path


def test_localization_settings_cannot_be_silently_changed_by_tuning(node):
    old=node.localization_timeout
    result=set_values(node,localization_timeout=.8)
    assert not result.successful and node.localization_timeout==old


def pair(offset=0.,width=.5):
    return [Cluster([(.5,-.6),(.6,-.6),(.6,-.4),(.5,-.4)]),
            Cluster([(.6+width,-.6-offset),(.7+width,-.6-offset),(.7+width,-.4-offset),(.6+width,-.4-offset)])]


@pytest.mark.parametrize('cfg,objects,reason',[
    (Settings(),pair(offset=.2),'not_aligned'),
    (replace(Settings(),min_gap=.6),pair(width=.5),'gap_too_small'),
    (replace(Settings(),max_gap=.4),pair(width=.5),'gap_too_large'),
])
def test_candidate_filter_diagnostics_explain_the_actual_rejection(cfg,objects,reason):
    d={};assert not find_gaps(objects,cfg,diagnostics=d)
    assert d['pair_filter_counts'][reason]==1
    d.update(reason_code='no_gap')
    assert '제외 사유' in planning_text(d)
