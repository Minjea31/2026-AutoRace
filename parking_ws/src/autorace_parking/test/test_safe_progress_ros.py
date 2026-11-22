from dataclasses import replace

from autorace_parking.core import Gap
from test_search import search


def configure(node):
    node.cfg=replace(node.cfg,clearance=.05,assume_unobserved_free=True,
        block_occupied_slots=False,obstacle_min_cluster_size=0.)
    node.follower.cfg=node.cfg
    node.tracked_goal=Gap(1.,.6,0.,.5,0,1,confirmed=True)
    node.planning_info={'reason_code':'budget_exceeded'}


def test_confirmed_goal_continues_with_a_checked_arc_after_search_budget(search):
    node,scan=search;configure(node)
    node.search_distance=node.search_max_distance+1.
    node.search_started=1.
    speed,steering=node.search_command(100.,(0.,0.,0.),scan)
    assert speed==node.search_speed and steering>0
    assert node.state=='SEARCHING' and node.search_path
    assert node.follower.goal is node.tracked_goal


def test_blocked_old_path_does_not_hide_a_clear_alternative_step(search):
    node,scan=search;configure(node)
    node.follower.path=[(0.,0.,0.),(1.,0.,0.)]
    node.state='ROUTE_BLOCKED'
    scan.points=[(.3,-.14)]
    speed,steering=node.search_command(100.,(0.,0.,0.),scan)
    assert speed>0 and steering>0 and node.search_path


def test_a_point_in_the_vehicle_box_or_stale_scan_still_stops_progress(search):
    node,scan=search;configure(node)
    scan.points=[(.20,0.)]
    assert node.search_command(100.,(0.,0.,0.),scan)==(0.,0.)
    assert node.search_block_detail['kind']=='obstacle'
    scan.points=[];node.scan_time=99.
    assert node.search_command(100.,(0.,0.,0.),scan)==(0.,0.)


def test_checked_progress_can_resume_parking_after_recovery_handoff(search):
    node,scan=search;configure(node)
    node.recovery_lane_released=True
    node.follower.recovery_finished=True
    assert node.search_command(100.,(0.,0.,0.),scan)[0]>0
    assert not node.recovery_lane_released and not node.follower.recovery_finished
