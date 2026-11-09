from autorace_objavoid.node import *


def test_previous_direction_does_not_override_new_clear_lane_hint():
    z=Sector(float('inf'),20);s=Sectors(z,z,z,1.,1.)
    state,_,angle,route=plan_avoidance(s,Config(),-.08,True,previous_angle=.08)
    assert angle<0
    assert path_clearance(s,angle,route)>0


def test_previous_turn_is_rejected_when_new_obstacle_blocks_it():
    z=Sector(float('inf'),20);c=Config()
    previous=.3;point=preview_points(previous,c)[18]
    s=Sectors(z,z,z,1.,1.,points=(point,))
    state,_,angle,route=plan_avoidance(s,c,0.,True,previous_angle=previous)
    if path_is_valid(state):
        assert path_clearance(s,angle,route)>0


def test_clear_straight_path_unwinds_previous_steering():
    z=Sector(float('inf'),20);s=Sectors(z,z,z,1.,1.)
    for degrees in (-35,-5,5,35):
        angle=math.radians(degrees)
        for _ in range(5):
            state,_,angle,route=plan_avoidance(s,Config(),0.,True,previous_angle=angle)
            assert path_clearance(s,angle,route)>0
        assert abs(angle)<1e-6


def test_previous_steering_does_not_override_clear_lane_correction():
    z=Sector(float('inf'),20);s=Sectors(z,z,z,1.,1.)
    for sign in (-1,1):
        _,_,angle,route=plan_avoidance(s,Config(),sign*.15,True,previous_angle=-sign*.3)
        assert angle*sign>0
        assert path_clearance(s,angle,route)>0
