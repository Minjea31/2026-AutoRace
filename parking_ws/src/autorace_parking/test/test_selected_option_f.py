"""Selected preview F: straight, a 40 cm radius quarter-circle, then entry."""
from dataclasses import fields, replace
import math
from pathlib import Path

import pytest
import yaml

from autorace_parking.core import Gap, Scan, Settings, advance, wrap
from autorace_parking.control import Follower, build_plan_report


def profile():
    p = Path(__file__).resolve().parents[1]/'config/parking_real.yaml'
    params = yaml.safe_load(p.read_text())['parking_node']['ros__parameters']
    # Archive the selected F geometry independently of the new live detector.
    params.update(min_turn_radius=.4,require_steering_calibration=False,prefer_straight_entry=False,max_planning_turn_radius=0.,entry_outward_penalty=0.,curvature_feedforward=False,reverse_steering_response_scale=1.,steering_response_scale=1.,steering_gain=1.,require_path_heading=True,parking_speed=.5,lookahead=.25,parking_position_tolerance=.1)
    cfg=Settings(**{f.name:params[f.name] for f in fields(Settings) if f.name in params})
    return replace(cfg,require_goal_heading=True,max_steering_paths_only=True,
                   enclosed_space_detection=False,continuous_replanning=False),params


def clear_scan():
    return Scan([math.inf]*361,-math.pi,math.pi/180,.01,12.,(0.,0.,0.))


def selected_target():
    return Gap(1.908263838795647,-.4494544893545864,-math.pi/2,.85,0,1)


def test_real_profile_matches_selected_f_geometry():
    cfg,p=profile()
    assert cfg.turn_radius == pytest.approx(.4)
    assert cfg.steering_limit == pytest.approx(math.atan(.2/.4))
    assert cfg.max_steering_paths_only and cfg.max_turn_arc_length >= math.pi*.4/2
    assert cfg.require_goal_heading and p['require_path_heading']
    assert not p['require_parking_heading']


def test_generated_f_path_matches_preview_target_radius_arc_and_turn_start():
    cfg,_=profile();goal=selected_target()
    report=build_plan_report(clear_scan(),cfg,goal)
    assert report['final']
    path=report['path']
    assert path[-1] == pytest.approx((goal.x,goal.y,-math.pi/2))
    curved=[]
    for a,b in zip(path,path[1:]):
        delta=wrap(b[2]-a[2])
        if abs(delta)>1e-8:
            chord=math.hypot(b[0]-a[0],b[1]-a[1])
            assert 2*abs(math.sin(delta/2))/chord == pytest.approx(2.5)
            curved.append((a,b))
    assert curved[0][0][0] == pytest.approx(goal.x-.4)
    assert sum(abs(wrap(b[2]-a[2]))*.4 for a,b in curved) == pytest.approx(.6283185307)


def test_selected_f_parks_and_exits_with_speed_floor_and_acceleration_limit():
    cfg,p=profile();cfg=replace(cfg,auto_exit_after_parking=True);scan=clear_scan();report=build_plan_report(scan,cfg,selected_target())
    f=Follower(cfg,speed=p['parking_speed'],lookahead=p['lookahead'],
               hold_seconds=p['parking_hold_seconds'],max_yaw_rate=p['max_yaw_rate'],
               require_parking_heading=p['require_parking_heading'],
               parking_position_tolerance=p['parking_position_tolerance'],
               max_steering_rate=p['max_steering_rate'],min_turn_speed=p['min_turn_speed'])
    f.accept(report['goal'],report['path'],True,(0.,0.,0.))
    pose=(0.,0.,0.);velocity=0.
    for i in range(1800):
        speed,angle,state=f.command(scan,pose,new_scan=i%2==0,now=i*.05)
        assert abs(angle)<=cfg.steering_limit+1e-9
        velocity=max(velocity-.05,min(velocity+.05,speed))
        pose=advance(pose,velocity*.05,math.tan(angle)/cfg.wheelbase)
        if f.exited:break
    assert f.parked and f.exited


def test_ros_path_heading_is_independent_of_the_arrival_heading_requirement():
    rclpy=pytest.importorskip('rclpy')
    from rclpy.parameter import Parameter
    from autorace_parking.node import ParkingNode
    rclpy.init()
    node=ParkingNode(parameter_overrides=[Parameter('localization_source',value='scan'),
                                         Parameter('require_path_heading',value=True),
                                         Parameter('require_parking_heading',value=False)])
    try:
        assert node.cfg.require_goal_heading
        assert not node.follower.require_parking_heading
    finally:
        node.destroy_node();rclpy.shutdown()
