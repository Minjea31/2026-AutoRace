from dataclasses import replace
import math
from autorace_objavoid.node import AvoidanceHold, Config, RoadBand, Sector, Sectors, path_is_valid, preview_points, plan_in_lane, path_clearance


def scan():
    z=Sector(math.inf,20)
    return Sectors(z,z,z,1.,1.,road=RoadBand(0.,1.,0.,2.,2))


def test_hold_releases_at_priority_cone_and_keeps_object_in_collision_checks():
    from autorace_objavoid.node import nearest_objects, path_clearance
    c=Config()
    for sign in (-1,1):
        initial=(.4,sign*.25,.06,.06,0.)
        hold=AvoidanceHold(initial,sign*.15)
        assert hold.observe((initial,),c)
        assert hold.angle==sign*.15
        outside=(.3,sign*.25,.06,.06,0.)
        following=(.9,-sign*.15,.06,.06,0.)
        observed=replace(scan(),boxes=(outside,following))
        assert hold.observe(observed.boxes,c)
        assert hold.angle is None
        assert nearest_objects(observed,c,initial)==[following]
        # Exclusion from steering priority never removes a box from collision checks.
        overlapping=(.05,sign*.12,.06,.06,0.)
        assert path_clearance(replace(scan(),boxes=(overlapping,)),0.,c)==0.


def test_hold_priority_cone_threshold_matches_configured_angle():
    for limit in (30.,35.):
        c=replace(Config(),priority_half_angle_deg=limit)
        for degrees,retained in ((limit-.1,True),(limit,True),(limit+.1,False)):
            box=(.5*math.cos(math.radians(degrees)),.5*math.sin(math.radians(degrees)),.03,.03,0.)
            hold=AvoidanceHold(box,.15)
            assert hold.observe((box,),c)
            assert (hold.angle is not None)==retained


def test_hold_without_yellow_boundary_keeps_previous_speed_rules():
    c=Config();s=replace(scan(),road=None)
    data=(1.,0.,0.,1.,0.,1.,2.,2.,0.,0.,.49)
    state,speed,angle,route=plan_in_lane(s,c,data,held_angle=.15)
    assert path_is_valid(state) and speed==.3 and angle==0.
    assert path_clearance(s,angle,route)>0


def test_unsafe_held_turn_uses_existing_collision_checked_planner():
    c=Config();point=preview_points(.3,c)[3]
    s=replace(scan(),points=(point,))
    data=(1.,0.,0.,1.,0.,1.,2.,2.,0.,0.,.49)
    state,speed,angle,route=plan_in_lane(s,c,data,held_angle=.3)
    assert speed==.1 if state=='NO_CLEAR_PATH' else speed>=.2
    if path_is_valid(state):
        assert path_clearance(s,angle,route)>0
