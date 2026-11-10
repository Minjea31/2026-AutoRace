from dataclasses import fields,replace
import json,math
from pathlib import Path
from concurrent.futures import Future
import pytest,yaml
from autorace_parking.core import Settings,Scan,Gap,CollisionChecker,wrap
from autorace_parking.control import Follower
from autorace_parking.departure import lane_targets,single_reverse_route
from autorace_parking.tracking import compose,inverse
from autorace_parking.regions import with_occupied_slots
from autorace_parking.diagnostics import operator_reason


def scene():
    root=Path(__file__).resolve().parents[1]
    params=yaml.safe_load((root/'config/parking_real.yaml').read_text())['parking_node']['ros__parameters']
    cfg=Settings(**{f.name:params[f.name] for f in fields(Settings) if f.name in params})
    data=json.loads((root/'test/data/departure_recovery_trace.json').read_text());trace=[tuple(p) for p in data['trace']]
    pose=trace[-1];scan=Scan([math.inf]*361,-math.pi,math.pi/180,.01,12.)
    scan.points=[compose(inverse(pose),(x,y,0.))[:2] for x,y,_ in data['memory_cells']]
    scan=with_occupied_slots(scan,data['slot_map'],pose,cfg)
    return cfg,trace,pose,scan


def test_recovery_detours_become_one_clear_reverse_only_route():
    cfg,trace,pose,scan=scene()
    signs=[(b[0]-a[0])*math.cos(a[2])+(b[1]-a[1])*math.sin(a[2]) for a,b in zip(trace,trace[1:])]
    assert any(v<-.001 for v in signs) and any(v>.001 for v in signs)
    targets=lane_targets(trace,pose,cfg,None,.05)
    route,reason=single_reverse_route(trace,pose,targets,scan,cfg)
    assert route and not reason and math.dist(route[0][:2],pose[:2])<1e-9
    checker=CollisionChecker(scan,cfg)
    assert all(checker.free(compose(inverse(pose),p)) for p in route)
    for a,b in zip(route,route[1:]):
        distance=math.dist(a[:2],b[:2]);yaw=a[2]+wrap(b[2]-a[2])/2
        assert (b[0]-a[0])*math.cos(yaw)+(b[1]-a[1])*math.sin(yaw)<0
        assert 2*abs(math.sin(wrap(b[2]-a[2])/2))/distance<=1/cfg.turn_radius+1e-6
    assert abs(wrap(route[-1][2]-trace[0][2]))<=cfg.exit_arrival_yaw_tolerance


def test_full_departure_display_does_not_shrink_with_progress():
    f=Follower(Settings());f.path=[(1.,0.,0.),(.5,0.,0.),(0.,0.,0.)]
    f.exit_path=f.path.copy();f.exit_prepared=f.exiting=True
    initial=f.local_path((0.,0.,0.));f.index=2
    assert f.local_path((0.,0.,0.))==initial
    f.exited=True
    assert f.local_path((0.,0.,0.))==[]


def test_wrong_way_exit_target_stops_then_reconnects_instead_of_latching_error():
    f=Follower(replace(Settings(),assume_unobserved_free=True),speed=.2)
    f.goal=Gap(1.,0.,0.,.6,0,1);f.path=[(0.,0.,0.),(1.,0.,0.)]
    f.parked=f.exiting=True;f.motion_direction=f.last_drive_direction=-1;f.segment_end=1
    scan=Scan([math.inf]*361,-math.pi,math.pi/180,.01,12.)
    assert f.command(scan,(0.,0.,0.),False,0.)[2]=='EXIT_PATH_ERROR'
    assert f.exit_route_reason=='lookahead_wrong_side' and not f.exit_route_ready
    f.exit_planner=lambda *args: ([(0.,0.,0.),(-.5,0.,0.)],'')
    v,a,state=f.command(scan,(0.,0.,0.),False,1.)
    assert v<0 and state=='EXITING' and f.exit_route_ready


def test_exit_failure_explains_the_condition_and_retries():
    text=operator_reason('EXIT_PATH_ERROR','stale older planning reason',{'exit_route_reason':'lookahead_wrong_side'})
    assert '후진 방향' in text and '재계산' in text and '초기화' not in text


def test_no_route_through_a_newly_occupied_exit_corridor():
    cfg,trace,pose,scan=scene();scan.points.append((0.,0.))
    route,reason=single_reverse_route(trace,pose,lane_targets(trace,pose,cfg,None,.05),scan,cfg)
    assert not route and reason=='no_reverse_route'


@pytest.mark.parametrize('new_obstacle',[False,True])
def test_node_plans_departure_off_the_control_loop_and_checks_latest_map(new_obstacle):
    import rclpy
    from rclpy.parameter import Parameter
    from autorace_parking.node import ParkingNode
    rclpy.init();node=ParkingNode(parameter_overrides=[Parameter('localization_source',value='scan'),
        Parameter('assume_unobserved_free',value=True)])
    try:
        node.worker.shutdown(wait=True);pending=Future();submitted=[]
        class Worker:
            def submit(self,*args):submitted.append(args);return pending
            def shutdown(self,**kwargs):pass
        node.worker=Worker();node.activation.active=True;node.tracker.valid=True
        node.follower.goal=Gap(-1.,0.,0.,.6,0,1);node.follower.parked=True
        scan=Scan([math.inf]*361,-math.pi,math.pi/180,.01,12.)
        node.planning_scan=lambda:scan
        assert node.queue_exit_plan([(0.,0.,0.)],(0.,0.,0.),[(-1.,0.,0.)],scan,node.cfg) is None
        node.queue_exit_plan([], (0.,0.,0.),[],scan,node.cfg)
        assert len(submitted)==1 and not pending.done()
        route=[(-i*.02,0.,0.) for i in range(51)]
        if new_obstacle:scan.points=[(-.5,0.)]
        pending.set_result((route,''));node.poll_exit_plan()
        assert node.exit_pending is None
        assert node.follower.exit_route_ready is (not new_obstacle)
        assert node.follower.exit_route_reason==('map_changed' if new_obstacle else '')
    finally:node.destroy_node();rclpy.shutdown()


def test_real_process_worker_can_build_the_departure_without_blocking_control():
    import time
    import rclpy
    from rclpy.parameter import Parameter
    from autorace_parking.node import ParkingNode
    from autorace_parking.core import advance
    rclpy.init();node=ParkingNode(parameter_overrides=[Parameter('localization_source',value='scan'),
        Parameter('assume_unobserved_free',value=True)])
    try:
        trace=[(i*.02,0.,0.) for i in range(51)]
        trace += [advance(trace[-1],i*.02,-1/.65) for i in range(1,51)]
        pose=trace[-1];scan=Scan([math.inf]*361,-math.pi,math.pi/180,.01,12.)
        node.activation.active=True;node.tracker.valid=True;node.tracker.pose=pose
        node.planning_scan=lambda:scan;node.follower.goal=Gap(*pose,.6,0,1)
        node.follower.entry_trace=trace;node.follower.parked=True
        node.follower.exit_targets=lane_targets(trace,pose,node.cfg,None,.05)
        before=time.monotonic();node.follower.rebuild_exit(pose,0.,scan)
        assert time.monotonic()-before<1. and node.exit_pending is not None
        end=time.monotonic()+10.
        while node.exit_pending is not None and time.monotonic()<end:
            node.poll_exit_plan();time.sleep(.01)
        assert node.exit_pending is None and node.follower.exit_route_ready
        assert node.follower.motion_direction==-1
    finally:node.destroy_node();rclpy.shutdown()
