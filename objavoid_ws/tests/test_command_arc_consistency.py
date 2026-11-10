import math
import pytest
from autorace_objavoid.node import *

LANE=(1.,0.,0.,1.,0.,1.,2.,2.,0.,0.,.49)

def scan(points=(),boxes=()):
    z=Sector(math.inf,20)
    return Sectors(z,z,z,1.,1.,points=points,boxes=boxes)


@pytest.mark.parametrize('side',[-1,1])
def test_held_40_degree_command_stays_circular_after_60_degrees(side):
    c=Config();a=side*c.max_avoid_steering;k=math.tan(a)/c.wheelbase
    poses=preview_poses(a,c)
    assert abs(poses[-1][2])>math.radians(60)
    for i,(x,y,yaw) in enumerate(poses):
        d=c.preview_distance*i/24
        assert x==pytest.approx(math.sin(k*d)/k)
        assert y==pytest.approx((1-math.cos(k*d))/k)
        assert yaw==pytest.approx(k*d)


def test_collision_after_60_degrees_is_not_hidden_by_fake_straight_tail():
    c=replace(Config(),preview_distance=1.)
    a=c.max_avoid_steering;k=math.tan(a)/c.wheelbase;d=.75
    point=(math.sin(k*d)/k,(1-math.cos(k*d))/k)
    assert path_clearance(scan(points=(point,)),a,c)==0.


def test_recovery_can_immediately_unwind_previous_maximum_turn():
    c=Config();window=recovery_window(-c.max_avoid_steering,.05,c.max_avoid_steering)
    assert window[1]==0.
    state,_,angle,route=plan_in_lane(scan(),c,LANE,previous_angle=-c.max_avoid_steering,
                                   held_angle=-c.max_avoid_steering,steering_window=window)
    assert path_is_valid(state) and angle==0.
    assert path_clearance(scan(),angle,route)>0


def test_planner_rejects_looping_maximum_angle_in_clear_space():
    c=Config();s=scan(boxes=((1.5,.05,.03,.03,0.),(1.8,-.18,.03,.03,0.)))
    state,_,angle,route=plan_in_lane(s,c,LANE,previous_angle=c.max_avoid_steering)
    assert path_is_valid(state)
    assert abs(angle)<=c.max_avoid_steering and 0<route.turn_distance<route.preview_distance
    assert preview_poses(angle,route)[-1][0]>0
    assert all(abs(yaw)<math.pi/2 for _,_,yaw in preview_poses(angle,route))
    assert path_clearance(s,angle,route)>0
