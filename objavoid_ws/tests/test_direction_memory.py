from dataclasses import replace
import math
from autorace_objavoid.node import *

LANE=(1.,0.,0.,1.,0.,1.,2.,2.,0.,0.,.49)

def scan(boxes=(),points=()):
    z=Sector(math.inf,20)
    return Sectors(z,z,z,1.,1.,boxes=boxes,points=points)


def test_direction_updates_before_first_obstacle_passes_35_degrees():
    c=Config();h=AvoidanceHold()
    first=(.8,.05,.03,.03,0.);second=(1.2,-.2,.03,.03,0.)
    h.update(scan((first,second)),c,1.)
    assert h.side==1
    s=scan((first,(1.2,.2,.03,.03,0.)))
    targets=h.update(s,c,1.1)
    assert h.side==-1 and targets[0]==first
    assert abs(math.atan2(first[1],first[0]))<math.radians(35)
    state,_,angle,route=plan_in_lane(s,c,LANE,objects_override=targets,direction=h.side)
    assert path_is_valid(state) and angle<0 and path_clearance(s,angle,route)>0


def test_dropout_uses_same_target_only_for_bounded_time_and_keeps_ghost_collision():
    c=Config();h=AvoidanceHold();first=(.6,.1,.04,.04,0.)
    h.update(scan((first,)),c,1.)
    targets=h.update(scan(),c,1.2)
    assert targets[0]==first and h.reason=='dropout_hold'
    collision=h.collision_scan(scan(),c,1.2)
    assert collision.boxes and collision.boxes[0][2]>first[2]
    assert not h.update(scan(),c,1.401)
    assert h.side==0 and h.reason=='dropout_expired'


def test_passing_35_degrees_releases_direction_and_acquires_next_pair():
    c=Config();h=AvoidanceHold();first=(.4,.2,.03,.03,0.)
    h.update(scan((first,(1.,-.2,.03,.03,0.))),c,1.)
    assert h.side==1
    outside=(.2,.2,.03,.03,0.)
    nxt=(.8,.1,.03,.03,0.);second=(1.2,.2,.03,.03,0.)
    targets=h.update(scan((outside,nxt,second)),c,1.1)
    assert targets==[nxt,second] and h.side==-1
    assert outside in scan((outside,nxt,second)).boxes


def test_stale_scan_does_not_extend_dropout_memory():
    c=Config();h=AvoidanceHold();first=(.6,.1,.03,.03,0.)
    h.update(scan((first,)),c,1.,received=1.)
    h.update(scan((first,)),c,1.3,received=1.)
    assert not h.update(scan(),c,1.401)


def test_recovery_angle_is_selected_inside_window_then_collision_checked():
    c=Config();s=scan()
    # A left turn must not immediately become the opposite lane target.
    lane=(1.,-.3,-.2,1.,0.,1.,2.,2.,-.3,-.2,.49)
    state,_,angle,route=plan_in_lane(s,c,lane,previous_angle=.35,steering_window=(.30,.40))
    assert path_is_valid(state) and .30<=angle<=.40
    assert path_clearance(s,angle,route)>0
    blocked=replace(s,points=(preview_points(angle,route)[2],))
    state,speed,angle,route=plan_in_lane(blocked,c,lane,previous_angle=.35,steering_window=(.30,.40))
    if path_is_valid(state):
        assert .30<=angle<=.40 and path_clearance(blocked,angle,route)>0
    else:
        assert state=='NO_CLEAR_PATH' and speed==.1 and angle==0.


def test_avoidance_recomputes_safe_angle_as_obstacle_approaches():
    c=Config();h=AvoidanceHold();angles=[]
    for t,x in ((1.,1.2),(1.1,.9),(1.2,.6)):
        first=(x,.05,.03,.03,0.);second=(1.5,-.2,.03,.03,0.)
        s=scan((first,second));targets=h.update(s,c,t)
        state,_,angle,route=plan_in_lane(s,c,LANE,objects_override=targets,direction=h.side)
        assert path_is_valid(state) and abs(angle)<=c.max_avoid_steering and path_clearance(s,angle,route)>0
        assert abs(preview_poses(angle,route)[-1][2])<math.pi/2
        angles.append(angle)
    assert len(set(round(a,4) for a in angles))>1
    assert all(abs(a)<c.max_avoid_steering for a in angles)


def test_fresh_nearer_targets_replace_dropout_memory_immediately():
    c=Config();h=AvoidanceHold()
    h.update(scan(((1.5,.1,.03,.03,0.),(1.8,-.2,.03,.03,0.))),c,1.)
    first=(.5,-.1,.03,.03,0.);second=(1.,.2,.03,.03,0.)
    targets=h.update(scan((first,second)),c,1.1)
    assert targets==[first,second] and h.box==first and h.side==-1 and not h.missing


def test_third_obstacle_changes_side_when_it_occupies_more_of_opposite_side():
    c=Config();h=AvoidanceHold()
    first=(.7,.02,.03,.03,0.);second=(1.,-.03,.03,.03,0.);third=(1.3,.4,.03,.03,0.)
    h.update(scan((first,second)),c,1.)
    assert h.side==1
    targets=h.update(scan((third,first,second)),c,1.1)
    assert targets==[first,second,third] and h.side==-1
