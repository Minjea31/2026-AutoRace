from dataclasses import replace
import math
from autorace_parking.core import Settings,Scan,advance
from autorace_parking.departure import reverse_entry_route,arc_then_straight,arc_straight_departure


def entry():
    straight=[(.8*i/40,0.,0.) for i in range(41)]
    return straight+[advance(straight[-1],.8*i/40,1.) for i in range(1,41)]


def test_recorded_reverse_keeps_entry_clearance_and_rejects_added_margin():
    path=entry();pose=path[-1]
    cfg=Settings(assume_unobserved_free=True,max_range=8.,obstacle_min_cluster_size=0.,exit_path_tracking_margin=0.)
    scan=Scan([math.inf]*361,-math.pi,math.pi/180,.01,8.)
    scan.points=[(.28,0.)]
    route=reverse_entry_route(path,pose,scan,cfg)
    assert route and arc_then_straight(route)
    assert not reverse_entry_route(path,pose,scan,replace(cfg,exit_path_tracking_margin=.04))
    scan.points=[(.12,0.)]  # actual body collision remains a stop
    assert not reverse_entry_route(path,pose,scan,cfg)


def test_more_than_one_curve_is_not_accepted_as_curve_straight():
    path=entry();pose=path[-1]
    extra=[advance(pose,.3*i/20,-1.) for i in range(1,21)]
    assert not arc_then_straight(list(reversed(path+extra)))


def test_small_stop_error_can_anchor_whole_recorded_curve_without_bending_join():
    from autorace_parking.departure import anchored_reverse_entry
    path=entry();nominal=path[-1]
    pose=(nominal[0]+.04,nominal[1]-.01,nominal[2]+.02)
    cfg=Settings(assume_unobserved_free=True,max_range=8.,exit_path_tracking_margin=0.,exit_lane_tolerance=.2)
    scan=Scan([math.inf]*361,-math.pi,math.pi/180,.01,8.)
    route=anchored_reverse_entry(path,pose,scan,cfg)
    assert route and route[0]==pose and arc_then_straight(route)
    bad=(nominal[0]+.5,nominal[1],nominal[2])
    assert not anchored_reverse_entry(path,bad,scan,cfg)
