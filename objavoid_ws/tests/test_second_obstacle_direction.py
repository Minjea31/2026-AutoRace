import math
from dataclasses import replace

import pytest

from autorace_objavoid.node import (
    Config, MeasuredRoad, Sector, Sectors, avoidance_side, avoidance_target,
    nearest_objects, path_clearance, path_is_valid, plan_in_lane,
)


LANE = (1., 0., 0., 1., 0., 1., 2., 2., 0., 0., .49)


def scene(side, second_x=1.2):
    z = Sector(math.inf, 20)
    # The first obstacle is 6 cm off-centre: outside the old 3 cm exception.
    first = (.85, side*.06, .04, .04, 0.)
    second = (second_x, -side*.18, .05, .05, 0.)
    return Sectors(z, z, z, 1., 1., boxes=(first, second))


@pytest.mark.parametrize('side', [-1, 1])
@pytest.mark.parametrize('second_x', [.9, 1.2])
def test_second_obstacle_sets_side_even_with_off_centre_first_and_wrong_hold(side, second_x):
    c = Config()
    scan = scene(side, second_x)
    objects = nearest_objects(scan, c)
    assert objects == list(scan.boxes)
    assert avoidance_side(objects) == side
    assert avoidance_target(objects[0], c, avoidance_side(objects))[1]*side > 0
    for held in (None, -side*.35):
        state, _, angle, route = plan_in_lane(
            scan, c, LANE, previous_angle=-side*.35, held_angle=held)
        assert path_is_valid(state)
        assert angle*side > 0
        assert path_clearance(scan, angle, route) > 0


@pytest.mark.parametrize('side', [-1, 1])
def test_second_obstacle_preference_does_not_override_yellow_boundary(side):
    c = Config()
    boundary = MeasuredRoad(0., 1., 0., 0., 2, ((0., -side, .19),))
    scan = replace(scene(side), road=boundary)
    state, speed, angle, route = plan_in_lane(scan, c, LANE)
    if path_is_valid(state):
        assert angle*side < 0
        assert path_clearance(scan, angle, route) > 0
    else:
        assert state=='NO_CLEAR_PATH' and angle==0.
        assert speed==.1


def test_second_obstacle_is_next_nearest_not_input_order():
    scan = scene(1)
    first, second = scan.boxes
    third = (1.6, .22, .05, .05, 0.)
    assert nearest_objects(replace(scan, boxes=(third, second, first))) == [first, second, third]


def test_central_second_does_not_arbitrarily_force_a_side():
    first = scene(1).boxes[0]
    assert avoidance_side([first, (1.2, 0., .05, .05, 0.)]) == 0


def test_actual_distance_ranking_overrides_old_primary_and_longitudinal_order():
    z=Sector(math.inf,20)
    # Smaller x alone would rank the lateral object first; Euclidean distance does not.
    lateral=(.8,.55,.03,.03,0.)
    nearest=(.9,0.,.03,.03,0.)
    old=(1.4,-.1,.03,.03,0.)
    s=Sectors(z,z,z,1.,1.,boxes=(old,lateral,nearest))
    assert nearest_objects(s,primary=old)==[nearest,lateral,old]
    assert nearest_objects(replace(s,boxes=(nearest,old,lateral)),primary=old)==[nearest,lateral,old]


def test_inflated_front_extent_does_not_override_actual_distance():
    z=Sector(math.inf,20)
    wide=(1.1,.1,.4,.03,0.)
    close=(.8,.1,.03,.03,0.)
    s=Sectors(z,z,z,1.,1.,boxes=(wide,close))
    assert nearest_objects(s)==[close,wide]


def test_blocked_preferred_side_uses_long_open_alternative():
    c=Config()
    s=replace(scene(1),road=MeasuredRoad(0.,1.,0.,0.,2,((0.,-1.,.19),)))
    state,_,angle,route=plan_in_lane(s,c,LANE,held_angle=-.35)
    assert path_is_valid(state) and 'SHORT_PREVIEW' not in state
    assert angle<0 and route.preview_distance==c.max_preview_distance
    assert path_clearance(s,angle,route)>0
