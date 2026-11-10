from dataclasses import replace
import math
import pytest
from autorace_objavoid.node import Config, Sector, Sectors, preview_poses, path_clearance
from autorace_objavoid.route_execution import MotionDistance, RouteExecution


def setup():
    c=replace(Config(),preview_distance=1.,turn_distance=.2,follow_steering=0.)
    ex=RouteExecution();primary=(.8,.1,.04,.04,0.)
    ex.begin(('AVOID',.3,c.max_avoid_steering,c),(0.,0),[primary],1)
    return ex,c,primary


def test_turn_unwinds_at_measured_distance_even_with_repeated_updates():
    ex,c,primary=setup()
    for distance in (0.,.05,.10,.15,.18):
        assert ex.matches([primary],1)
        state,_,angle,route=ex.remaining((distance,0))
        assert state.endswith('/TURN') and angle==c.max_avoid_steering
        assert route.turn_distance==pytest.approx(.2-distance)
        assert ex.origin==0.  # New scans never restart the initial-turn budget.
    state,_,angle,route=ex.remaining((.19,0))
    assert state.endswith('/FOLLOW') and angle==0.
    state,_,angle,route=ex.remaining((.2,0))
    assert state.endswith('/FOLLOW') and angle==0.
    assert route.preview_distance==pytest.approx(.8)
    assert all(abs(y)<1e-9 for x,y,yaw in preview_poses(angle,route))


def test_executed_segments_match_checked_curve_at_switch():
    ex,c,_=setup();a=c.max_avoid_steering
    junction=preview_poses(a,replace(c,preview_distance=.2,turn_distance=-1.))[-1]
    _,_,follow,remaining=ex.remaining((.2,0))
    tail=preview_poses(follow,remaining)[-1]
    co,si=math.cos(junction[2]),math.sin(junction[2])
    executed=(junction[0]+co*tail[0]-si*tail[1],junction[1]+si*tail[0]+co*tail[1],junction[2]+tail[2])
    assert executed==pytest.approx(preview_poses(a,c)[-1])


def test_new_obstacle_in_follow_segment_invalidates_execution():
    ex,c,_=setup();z=Sector(math.inf,20)
    # Current frame at the switch: an obstacle blocks the straight remaining segment.
    s=Sectors(z,z,z,1.,1.,points=((.3,0.),))
    assert ex.checked_remaining((.2,0),lambda a,r:path_clearance(s,a,r)>0) is None
    assert ex.result is None


def test_stale_or_reset_odometry_cannot_advance_old_plan():
    motion=MotionDistance()
    assert motion.snapshot(1.,.5) is None
    motion.update(0.,0.,1.,1.)
    motion.update(.1,0.,1.1,1.1)
    assert motion.snapshot(1.2,.5)==pytest.approx((.1,0))
    assert motion.snapshot(1.7,.5) is None
    ex,c,primary=setup()
    motion.update(10.,0.,1.2,1.2) # Pose jump/reset.
    assert ex.remaining(motion.snapshot(1.2,.5)) is None
    assert ex.result is None


def test_switch_to_new_target_is_not_delayed_until_end_of_old_route():
    ex,c,primary=setup()
    assert not ex.matches([(.3,-.2,.04,.04,0.)],-1)
    assert not ex.matches([],0)


def test_full_metre_plan_can_start_above_17_degrees_and_actually_unwind():
    from autorace_objavoid.node import plan_in_lane, path_is_valid
    c=Config();z=Sector(math.inf,20)
    targets=[(.8,.05,.03,.03,0.),(1.2,-.2,.03,.03,0.)]
    s=Sectors(z,z,z,1.,1.,boxes=tuple(targets))
    lane=(1.,0.,0.,1.,0.,1.,2.,2.,0.,0.,.49)
    result=plan_in_lane(s,c,lane)
    state,speed,initial,route=result
    assert path_is_valid(state) and initial>math.radians(17.5)
    assert route.preview_distance==1. and 0<route.turn_distance<route.preview_distance
    assert route.follow_steering<initial
    assert path_clearance(s,initial,route)>0
    ex=RouteExecution();ex.begin(result,(0.,0),targets,1)
    assert ex.remaining((0.,0))[2]==initial
    tail=ex.remaining((route.turn_distance,0))
    assert tail[0].endswith('/FOLLOW') and tail[2]==route.follow_steering


def test_duplicate_odometry_does_not_reset_ongoing_route():
    motion=MotionDistance();motion.update(0.,0.,1.,1.)
    motion.update(0.,0.,1.,1.1)
    assert motion.snapshot(1.1,.5)==(0.,0)
    assert motion.sample[2]==1. # A duplicate must not extend freshness.


def test_valid_lane_received_during_tick_is_not_rejected_as_future(monkeypatch):
    import autorace_objavoid.node as node
    data=(1.,0.,0.,1.,0.,1.,2.,2.,0.,0.,.49)
    # Tick began at 10.0; another callback received this packet at 10.001.
    monkeypatch.setattr(node.time,'monotonic',lambda:10.002)
    assert node.fresh_lane((data,10.001),.5)==data
    monkeypatch.setattr(node.time,'monotonic',lambda:10.6)
    assert node.fresh_lane((data,10.001),.5) is None
    assert node.fresh_lane(None,.5) is None
