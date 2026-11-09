import math
import random
from dataclasses import replace
import pytest
from autorace_objavoid.node import (Config, RoadBand, Sector, Sectors, current_road,
    nearest_objects, path_clearance, plan_avoidance, preview_poses)


def scan(boxes=(), points=(), road=None):
    s = Sector(math.inf, 20)
    return Sectors(s,s,s,1.,1.,boxes=boxes,points=points,road=road)


def test_runtime_side_objects_do_not_steal_ranks():
    side = ((.13,.51,.05,.03,0.),(.60,1.13,.10,.08,0.))
    assert not nearest_objects(scan(side))
    threat=(1.,.12,.06,.06,0.)
    assert nearest_objects(scan(side+(threat,)))[0] == threat
    # Excluded ranking is still included in collision geometry.
    assert path_clearance(scan(side),.6,Config(clearance_margin=.1)) == 0


def test_primary_is_retained_but_neighbour_is_not_reidentified():
    old=(.35,.18,.06,.06,0.)
    same=(.30,.20,.06,.06,0.)
    neighbour=(.30,.50,.04,.04,0.)
    following=(1.2,-.2,.06,.06,0.)
    assert nearest_objects(scan((neighbour,same,following)),primary=old)[0] == same
    assert nearest_objects(scan((neighbour,following)),primary=old)[0] == following


def test_retained_primary_does_not_force_extra_turn_when_clear_to_side():
    primary=(.1,.5,.04,.04,0.)
    state,_,angle,_=plan_avoidance(scan((primary,)),Config(),primary=primary)
    assert state == 'LANE'
    assert angle == 0.


def test_detected_pair_fallback_survives_missing_outer_edges():
    data=(1.,0.,0.,1.,0.,1.,2.,2.,0.,0.,.2)
    assert current_road(data,None,True,Config()) is None
    band=RoadBand(0.,1.,0.,.78,2)
    assert current_road(None,band,True,Config()) is band
    assert current_road(None,band,False,Config()) is None


def test_follow_turn_collision_is_checked():
    c=replace(Config(),preview_distance=1.6,turn_distance=.6,follow_steering=-.25)
    poses=preview_poses(.25,c)
    obstacle=poses[20][:2]
    assert path_clearance(scan(points=(obstacle,)),.25,c) == 0.
    assert poses[-1][2] < poses[10][2]
    # In the initial 0.6m the follow steering cannot change the path.
    a=preview_poses(.25,replace(c,follow_steering=.25))
    assert poses[:10] == a[:10]


def test_two_obstacle_route_checks_beyond_old_horizon():
    s=scan(((.8,.13,.06,.06,0.),(1.6,-.2,.06,.06,0.)))
    state,speed,angle,checked=plan_avoidance(s,Config(preview_distance=.8,max_preview_distance=2.5),lane_ok=True)
    assert state.startswith('AVOID')
    assert checked.preview_distance > .8
    assert path_clearance(s,angle,checked)>0
    assert 0 < checked.turn_distance < checked.preview_distance
    if state == 'AVOID':
        assert preview_poses(angle,checked)[-1][0] > 1.6+.06+.161+Config().clearance_margin


def test_bulk_boxes_match_individual_sat_for_two_turn_paths():
    rng=random.Random(22)
    boxes=tuple((rng.uniform(-1,3),rng.uniform(-2,2),rng.uniform(.02,.2),
                 rng.uniform(.02,.2),rng.uniform(-math.pi,math.pi)) for _ in range(16))
    for follow in (-.35,0.,.35):
        c=replace(Config(),preview_distance=1.7,turn_distance=.7,follow_steering=follow)
        bulk=path_clearance(scan(boxes),.2,c)
        single=min(path_clearance(scan((b,)),.2,c) for b in boxes)
        assert bulk == pytest.approx(single,abs=1e-12)
