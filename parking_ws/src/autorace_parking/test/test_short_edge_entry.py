from dataclasses import replace
import math

import pytest

from autorace_parking.core import Settings, Scan, CollisionChecker, Gap, plan_path, sample_candidate
from autorace_parking.control import Follower
from autorace_parking.regions import slot_entry_edges, with_occupied_slots, first_path_collision, invalid_slot_entry
from autorace_parking.tracking import compose, inverse
from autorace_parking.departure import transformed_scan


def clear():
    return Scan([math.inf]*361,-math.pi,math.pi/180,.01,10.)


def layout(center=(0.,0.),yaw=0.):
    return {'origin':(2.,2.),'yaw':yaw,'columns':3,'rows':2,
            'cell_width':.5,'cell_depth':.6,'border':.02,'locked':True,
            'cells':[{'id':3,'center':center,'occupied':False,'eligible_empty':True}]}


def config(**kwargs):
    return replace(Settings(),assume_unobserved_free=True,entry_via_short_edge=True,
                   yellow_boundary_enabled=False,**kwargs)


def test_yellow_disabled_keeps_actual_obstacle_margin():
    scan=clear();scan.yellow_segments=[((-.5,0.),(.5,0.),1)]
    cfg=config()
    assert not CollisionChecker(scan,replace(cfg,yellow_boundary_enabled=True)).free((0.,0.,0.))
    assert CollisionChecker(scan,cfg).free((0.,0.,0.))
    scan.points=[(.24,0.)]
    checker=CollisionChecker(scan,cfg)
    assert not checker.free((0.,0.,0.)) and checker.last_block=='obstacle'
    assert CollisionChecker(scan,replace(cfg,clearance=0.)).free((0.,0.,0.))


def test_entry_is_the_empty_cell_short_edge_not_the_whole_grid():
    edges=slot_entry_edges(layout(),config())
    assert {e['edge'] for e in edges if e['open']} == {'y_min','y_max'}
    assert [math.dist(e['a'],e['b']) for e in edges if e['open']] == pytest.approx([.5,.5])
    assert [math.dist(e['a'],e['b']) for e in edges if not e['open']] == pytest.approx([.6,.6])
    assert max(abs(v) for e in edges for p in (e['a'],e['b']) for v in p) <= .3


def test_only_the_selected_empty_slot_has_entry_markers():
    grid=layout()
    grid['cells'] += [{'id':4,'center':(1.,0.),'occupied':False,'eligible_empty':True},
                      {'id':5,'center':(2.,0.),'occupied':True,'eligible_empty':False}]
    assert not slot_entry_edges(grid,config())
    edges=slot_entry_edges(grid,config(),4)
    assert all(.75<=p[0]<=1.25 for e in edges for p in (e['a'],e['b']))
    assert not slot_entry_edges(grid,config(),5)


@pytest.mark.parametrize('yaw',[0.,.6,math.pi,-2.4])
def test_only_road_facing_short_edge_allows_entry_and_survives_body_rotation(yaw):
    cfg=config(entry_from_left_only=True)
    grid=layout((.8,-1.),yaw)
    edges=slot_entry_edges(grid,cfg,3)
    opened=[e for e in edges if e['open']]
    assert len(opened)==1
    edge=opened[0]
    mid=tuple((a+b)/2 for a,b in zip(edge['a'],edge['b']))
    normal=tuple((v-c)/.3 for v,c in zip(mid,grid['cells'][0]['center']))
    assert normal[1]>0  # +Y is road/left in the mission-start frame.
    outside=(*[c+.6*n for c,n in zip(grid['cells'][0]['center'],normal)],0.)
    opposite=(*[c-.6*n for c,n in zip(grid['cells'][0]['center'],normal)],0.)
    center=(*grid['cells'][0]['center'],0.)
    for body in [(0.,0.,0.),(.2,-.1,1.8)]:
        scan=with_occupied_slots(clear(),grid,body,cfg,3)
        checker=CollisionChecker(scan,cfg)
        local=lambda p:compose(inverse(body),p)
        assert checker.transition_free(local(outside),local(center))
        assert not checker.transition_free(local(opposite),local(center))
        assert checker.last_block=='slot_wrong_side_entry'
        assert checker.free(local(center))  # No body-containment fence.


@pytest.mark.parametrize('yaw',[0.,.6,-1.2])
def test_center_crosses_short_edge_after_frame_rotation(yaw):
    cfg=config();origin=(1.7,-.8,yaw);pose=(.3,.1,-.2)
    observed=with_occupied_slots(clear(),layout(origin[:2],yaw),pose,cfg)
    checker=CollisionChecker(observed,cfg)
    def local(p):return compose(inverse(pose),compose(origin,p))
    assert checker.transition_free(local((0.,-.6,0.)),local((0.,0.,0.)))
    assert not checker.transition_free(local((-.6,0.,0.)),local((0.,0.,0.)))
    assert checker.last_block=='slot_long_side_entry'


def test_overhanging_body_is_not_treated_as_crossing_a_wall():
    cfg=config();scan=with_occupied_slots(clear(),layout(),(0.,0.,0.),cfg)
    checker=CollisionChecker(scan,cfg)
    # At this offset the 52.2 x 42 cm safety box protrudes from the 50 x 60 cm bay.
    assert .20+checker.half_length > .25
    assert checker.free((.20,0.,0.))
    assert checker.transition_free((.20,-.6,0.),(.20,0.,0.))
    assert checker.free((.26,0.,0.))  # Neither side is a physical wall.


def test_exact_transition_check_cannot_jump_across_the_bay():
    cfg=config();scan=with_occupied_slots(clear(),layout(),(0.,0.,0.),cfg)
    checker=CollisionChecker(scan,cfg)
    assert not checker.transition_free((-1.,0.,0.),(1.,0.,0.))
    assert checker.last_block=='slot_long_side_entry'
    assert checker.transition_free((0.,-1.,0.),(0.,1.,0.))
    blocked=first_path_collision(CollisionChecker(scan,cfg),[(-1.,0.,0.),(0.,0.,0.)],.7)
    assert blocked['kind']=='slot_long_side_entry'


def test_inside_motion_and_outward_exit_do_not_recheck_entry():
    scan=with_occupied_slots(clear(),layout(),(0.,0.,0.),config());slot=scan.entry_slot
    assert invalid_slot_entry(slot,(0.,0.,0.),(.6,0.,0.)) is None
    assert invalid_slot_entry(slot,(.25,0.,0.),(.6,0.,0.)) is None
    assert invalid_slot_entry(slot,(.25,0.,0.),(0.,0.,0.))['kind']=='slot_long_side_entry'
    assert invalid_slot_entry(slot,(-1.,.3,0.),(1.,.3,0.)) is None  # tangent


def test_analytic_planner_checks_the_selected_slot_entry_side():
    cfg=config(clearance=.02,max_range=5.,planning_timeout=1.,require_goal_heading=False)
    scan=with_occupied_slots(clear(),layout((0.,1.)),(0.,0.,0.),cfg)
    checker=CollisionChecker(scan,cfg)
    path,valid=sample_candidate(checker,[(1.,0.)],cfg)  # parallel to the bay, no entry
    assert valid
    # A straight forward entry through the short side of a 90-degree bay.
    scan=with_occupied_slots(clear(),layout((1.,0.),math.pi/2),(0.,0.,0.),cfg)
    path=plan_path(scan,Gap(1.,0.,0.,.5,0,1,slot_id=3),cfg)
    assert path and math.dist(path[-1][:2],(1.,0.)) <= cfg.goal_tolerance
    assert not first_path_collision(CollisionChecker(scan,cfg),path,cfg.collision_step)
    # Same approach into an unrotated bay would enter its long side.
    scan=with_occupied_slots(clear(),layout((1.,0.)),(0.,0.,0.),cfg)
    _,valid=sample_candidate(CollisionChecker(scan,cfg),[(1.,0.)],cfg)
    assert not valid


def test_disabling_entry_policy_and_exit_transform_drop_slot_metadata():
    cfg=config();scan=with_occupied_slots(clear(),layout(),(0.,0.,0.),cfg)
    assert scan.entry_slot
    result=with_occupied_slots(scan,layout(),(0.,0.,0.),replace(cfg,entry_via_short_edge=False))
    assert result.entry_slot is None
    assert transformed_scan(scan,(.1,.2,.3)).entry_slot is None
    assert scan.entry_slot


def test_point_in_ego_safety_box_returns_a_zero_speed_command():
    cfg=config(continuous_replanning=True,require_complete_entry_path=True)
    follower=Follower(cfg,speed=.4)
    path=[(i*.02,0.,0.) for i in range(41)]
    follower.accept(Gap(.8,0.,0.,.5,0,1),path,True,(0.,0.,0.))
    scan=clear();scan.points=[(.24,0.)]
    speed,steering,state=follower.command(scan,(0.,0.,0.),new_scan=True,now=1.)
    assert speed==0. and state=='ROUTE_BLOCKED'
    assert follower.last_collision['kind']=='obstacle'


def test_follower_stops_a_newly_invalid_long_side_entry_without_blocking_overhang():
    cfg=config(continuous_replanning=True,require_complete_entry_path=True)
    follower=Follower(cfg,speed=.4)
    path=[(i*.02,0.,0.) for i in range(41)]
    follower.accept(Gap(.8,0.,0.,.5,0,1),path,True,(0.,0.,0.))
    scan=with_occupied_slots(clear(),layout((.3,0.)),(0.,0.,0.),cfg)
    assert CollisionChecker(scan,cfg).free((0.,0.,0.))
    speed,_,state=follower.command(scan,(0.,0.,0.),new_scan=True,now=1.)
    assert speed==0. and state=='ROUTE_BLOCKED'
    assert follower.last_collision['kind']=='slot_long_side_entry'
