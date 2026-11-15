import math
import pytest
import rclpy
from rclpy.parameter import Parameter
from autorace_parking.core import Scan,Gap
from autorace_parking.node import ParkingNode


@pytest.fixture
def search():
    rclpy.init();n=ParkingNode(parameter_overrides=[Parameter('localization_source',value='scan'),Parameter('search_enabled',value=True),Parameter('search_resume_scans',value=1)])
    n.planning_info={'reason_code':'no_gap'};n.scan_time=100.;n.scan_sequence=1
    scan=Scan([3.]*181,-math.pi/2,math.pi/180,.01,3.)
    try:yield n,scan
    finally:n.destroy_node();rclpy.shutdown()


def test_clear_measured_corridor_moves_and_target_immediately_ends_search(search):
    n,s=search
    assert n.search_command(100.,(0.,0.,0.),s)==(.1,0.)
    assert n.state=='SEARCHING'
    n.follower.goal=Gap(1.,-.5,0.,.5,0,1)
    n.follower.path=[(0.,0.,0.),(.5,0.,0.)]
    assert n.search_command(100.1,(.01,0.,0.),s) is None


def test_detected_but_unreachable_candidate_does_not_wait_forever(search):
    n,s=search;n.follower.goal=Gap(1.,-.5,0.,.5,0,1)
    n.planning_info={'reason_code':'no_observed_path'}
    assert n.search_command(100.,(0.,0.,0.),s)[0]>.0


def test_search_stops_for_unknown_space_obstacles_and_stale_scans(search):
    n,s=search
    assert n.search_command(100.,(0.,0.,0.),s)[0]>.0
    s.points=[(.35,0.)];n.scan_sequence+=1
    assert n.search_command(100.1,(.01,0.,0.),s)==(0.,0.)
    s.points=[];s.ranges=[0.]*181;n.scan_sequence+=1
    assert n.search_command(100.1,(.01,0.,0.),s)==(0.,0.)
    assert n.search_command(100.3,(.01,0.,0.),s)==(0.,0.)


def test_distance_and_time_limits_latch_until_mission_reset(search):
    n,s=search;n.search_max_distance=.15
    assert n.search_command(100.,(0.,0.,0.),s)[0]>.0
    assert n.search_command(100.1,(.16,0.,0.),s)==(0.,0.)
    assert n.state=='SEARCH_LIMIT'
    n.scan_time=100.2;n.scan_sequence+=1
    assert n.search_command(100.2,(.16,0.,0.),s)==(0.,0.)
    n.reset_motion();n.planning_info={'reason_code':'no_gap'};n.scan_time=110.
    assert n.search_command(110.,(0.,0.,0.),s)[0]>.0
    n.search_timeout=.1;n.scan_time=110.2
    assert n.search_command(110.2,(.01,0.,0.),s)==(0.,0.)
