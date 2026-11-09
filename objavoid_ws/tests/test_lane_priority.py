from autorace_objavoid.node import *


def lane(offset=0.,heading=0.):
    return (1.,offset,heading,1.,0.,1.,2.,2.,offset,heading,.3)


def scan(boxes=()):
    z=Sector(float('inf'),20)
    return Sectors(z,z,z,1.,1.,boxes=boxes)


def test_clear_lane_is_allowed_and_whole_body_stays_inside():
    c=Config();data=lane()
    state,speed,angle,route=plan_in_lane(scan(),c,data,previous_angle=.3)
    assert path_is_valid(state) and speed>0
    assert path_clearance(replace(scan(),road=lane_boundary(data,c)),angle,route)>0


def test_obstacle_cannot_force_escape_from_lane():
    c=Config();s=scan(((.35,0.,.08,.26,0.),));data=lane()
    state,speed,angle,route=plan_in_lane(s,c,data)
    assert state=='NO_CLEAR_PATH' and speed==.2 and angle==0
    assert not path_is_valid(state)


def test_missing_held_or_invalid_lane_keeps_low_speed():
    for data in (None,(0.,)*11):
        state,speed,angle,_=plan_in_lane(scan(),Config(),data)
        assert state=='LANE_UNAVAILABLE' and speed==.2 and angle==0


def test_lane_offset_does_not_create_a_fake_road_boundary():
    state,speed,angle,route=plan_in_lane(scan(),Config(),lane(.2))
    assert path_is_valid(state) and speed>0 and angle>0


def test_body_width_fallback_keeps_actual_supplied_boundaries():
    c=Config();data=lane(.12)
    s=replace(scan(),road=lane_boundary(data,c))
    state,speed,angle,route=plan_in_lane(s,c,data)
    assert path_is_valid(state) and '/BODY_WIDTH' in state
    assert speed==.2 and route.clearance_margin==0.
    assert path_clearance(s,angle,route)>0
    assert path_clearance(s,angle,replace(route,clearance_margin=.05))==0.


def test_real_lane_deep_equal_reference_distances_are_valid():
    data=(1.,.2051949203,.2451025844,1.0257213116,-2.5353081226,.71764326,
          2.,2.,.2051949203,.1709221154,1.0257213116)
    assert lane_reference(data)
    assert lane_boundary(data,Config()) is not None
    state,_,_,_=plan_in_lane(scan(),Config(),data)
    assert not state.startswith('LANE_UNAVAILABLE')


def test_equal_points_with_heading_can_generate_nonzero_lane_correction():
    data=(1.,.08,.1,.8,0.,1.,2.,2.,.08,.1,.8)
    state,speed,angle,route=plan_in_lane(scan(),Config(),data)
    assert path_is_valid(state) and angle>0
    assert path_clearance(scan(),angle,route)>0


def test_valid_lane_deep_target_is_used_even_if_one_boundary_is_held():
    data=(*lane()[:6],1.,2.,*lane()[8:])
    state,_,_,_=plan_in_lane(scan(),Config(),data)
    assert path_is_valid(state)


def test_white_divider_crossing_is_not_rejected_as_road_departure():
    data=(1.,.08,.1,.8,0.,1.,2.,2.,.08,.1,.8)
    state,_,angle,route=plan_in_lane(scan(),Config(),data)
    assert path_is_valid(state)
    assert path_clearance(scan(),angle,route)>0
    # This candidate crosses the old 0.52m corridor, which is not a road edge.
    assert path_clearance(replace(scan(),road=lane_boundary(data,Config())),angle,route)==0
