from dataclasses import replace
import math

import pytest

from autorace_parking.core import Settings,Scan,Gap
from autorace_parking.control import Follower,build_plan_report
from autorace_parking.obstacle_memory import MemoryScan


def scene(extra=()):
    s=MemoryScan.__new__(MemoryScan)
    raw=Scan([0.]*361,-math.pi,math.pi/180,.01,12.,(0.,0.,0.))
    s.__dict__=raw.__dict__.copy();s.points=[];s.point_owners={}
    for tid,cx in enumerate([.8,1.6]):
        for dx in [-.1,0.,.1]:
            for dy in [-.1,0.,.1]:
                p=(cx+dx,-.5+dy);s.points.append(p);s.point_owners[p]=tid
    s.points.extend(extra)
    return s


def config():
    return replace(Settings(),validate_parking_space=True,max_gap=.7,assume_unobserved_free=True)


def test_empty_gap_accepts_distinct_objects_and_checks_vehicle_clearance():
    report=build_plan_report(scene(),config())
    check=report['diagnostics']['candidate_checks'][0]
    assert check['accepted'] and check['width_m']==pytest.approx(.6)
    assert check['required_width_m']==pytest.approx(.3)


def test_isolated_unowned_point_between_boxes_rejects_candidate():
    report=build_plan_report(scene([(1.2,-.5)]),config())
    assert report['diagnostics']['candidates']==0
    assert report['diagnostics']['candidate_checks'][0]['reason']=='points_between_objects'


def test_point_outside_box_depth_but_inside_vehicle_footprint_rejects_candidate():
    report=build_plan_report(scene([(1.2,-.66)]),config())
    assert report['diagnostics']['candidates']==0
    assert report['diagnostics']['candidate_checks'][0]['reason']=='goal_footprint_obstacle'


@pytest.mark.parametrize('gain',[.5,1.,1.5])
def test_pursuit_angle_scales_by_gain_and_remains_bounded(gain):
    cfg=replace(Settings(),steering_gain=gain,max_steering_on_curves=False)
    f=Follower(cfg,lookahead=.25)
    path=[(i*.01,i*.002,0.) for i in range(101)]
    f.accept(Gap(1.,.2,0.,.5,0,1),path,True,(0.,0.,0.))
    _,angle,_=f.command(Scan([math.inf]*361,-math.pi,math.pi/180,.01,12.,(0.,0.,0.)),(0.,0.,0.),now=0.)
    x,y,_=f.lookahead_point
    expected=math.atan(cfg.wheelbase*2*y/(x*x+y*y))*gain
    assert angle==pytest.approx(min(cfg.steering_limit,expected))


def test_live_parameters_update_controller_without_resetting_route_and_reject_invalid_batch():
    import rclpy
    from rclpy.parameter import Parameter
    from autorace_parking.node import ParkingNode
    rclpy.init();n=ParkingNode(parameter_overrides=[Parameter('localization_source',value='scan')])
    try:
        path=[(0.,0.,0.),(1.,0.,0.)]
        n.follower.accept(Gap(1.,0.,0.,.5,0,1),path,True,(0.,0.,0.))
        previous=n.follower;epoch=n.epoch
        result=n.set_parameters_atomically([Parameter('steering_gain',value=.8),Parameter('lookahead',value=.3)])
        assert result.successful
        assert n.cfg.steering_gain==n.follower.cfg.steering_gain==.8
        assert n.lookahead==n.follower.lookahead==.3
        assert n.follower is previous and n.follower.path==path and n.epoch==epoch
        result=n.set_parameters_atomically([Parameter('steering_gain',value=1.2),Parameter('lookahead',value=-.1)])
        assert not result.successful and n.follower.cfg.steering_gain==.8 and n.follower.lookahead==.3
    finally:n.destroy_node();rclpy.shutdown()
