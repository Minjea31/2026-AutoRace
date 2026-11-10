import math
from dataclasses import replace
from autorace_objavoid.node import *

LANE=(1.,0.,0.,1.,0.,1.,2.,2.,0.,0.,.49)

def scene(side=1,road=None):
    z=Sector(math.inf,20)
    return Sectors(z,z,z,1.,1.,boxes=((1.5,side*.05,.03,.03,0.),(1.8,-side*.18,.03,.03,0.)),road=road)


def test_avoidance_ignores_small_slew_window_and_old_small_angle():
    c=Config()
    for side in (-1,1):
        s=replace(scene(side),boxes=((.8,side*.05,.03,.03,0.),(1.2,-side*.18,.03,.03,0.)))
        state,_,angle,route=plan_in_lane(s,c,LANE,previous_angle=0.,held_angle=side*.05,
                                       steering_window=(-.05,.05))
        assert path_is_valid(state)
        assert angle*side>.05 and abs(angle)<=c.max_avoid_steering
        assert abs(preview_poses(angle,route)[-1][2])<math.pi/2
        assert path_clearance(s,angle,route)>0


def test_large_angle_is_rejected_if_vehicle_crosses_measured_boundary():
    c=Config();s=scene(1,RoadBand(0.,1.,0.,.3,2))
    state,_,angle,route=plan_in_lane(s,c,LANE,steering_window=(-.05,.05))
    assert path_is_valid(state) and 0<angle<c.max_avoid_steering
    assert path_clearance(s,angle,route)>0
    assert path_clearance(s,c.max_avoid_steering,route)==0
    assert abs(preview_poses(angle,route)[-1][2])<math.pi/2
