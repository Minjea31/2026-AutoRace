"""Explicit real-car policy: bypass collision checks only during final departure."""
import math
from dataclasses import replace
from concurrent.futures import Future
from unittest.mock import Mock
import pytest
from autorace_parking.core import Settings,Scan,Gap,advance
from autorace_parking.control import Follower
from autorace_parking.departure import reverse_entry_route,arc_straight_departure,single_reverse_route,arc_then_straight
from test_start_and_wheel_departure import node


def source():
    scan=Scan([0.]*361,-math.pi,math.pi/180,.01,8.)
    scan.points=[(0.,0.),(-.1,0.),(.1,0.)]
    scan.yellow_segments=[((-.5,0.),(.5,0.),1)]
    return scan


def config():
    return replace(Settings(),exit_collision_check_enabled=False,assume_unobserved_free=False,
                   continuous_replanning=True,require_complete_entry_path=True,obstacle_min_cluster_size=0.)


def test_departure_generators_do_not_construct_collision_checker(monkeypatch):
    import autorace_parking.departure as module
    monkeypatch.setattr(module,'CollisionChecker',Mock(side_effect=AssertionError('departure collision check')))
    cfg=config();scan=source()
    straight=[(.8*i/40,0.,0.) for i in range(41)]
    entry=straight+[advance(straight[-1],.8*i/40,1.) for i in range(1,41)]
    pose=entry[-1]
    route=reverse_entry_route(entry,pose,scan,cfg)
    assert route and arc_then_straight(route)
    route,reason=arc_straight_departure(entry,pose,scan,cfg)
    assert route and not reason and arc_then_straight(route)
    route,reason=single_reverse_route(entry,pose,[],scan,cfg)
    assert route and not reason


def test_reverse_control_does_not_run_braking_collision_check(monkeypatch):
    import autorace_parking.control as module
    cfg=config();f=Follower(cfg,speed=.2)
    f.goal=Gap(-.8,0.,0.,.6,0,1)
    f.path=[(0.,0.,0.),(-.8,0.,0.)];f.exit_path=f.path.copy()
    f.entry_trace=[(-.8,0.,0.),(0.,0.,0.)]
    f.parked=f.exiting=f.exit_prepared=f.exit_route_ready=True
    f.set_segment(0);f.last_drive_direction=-1
    monkeypatch.setattr(module,'CollisionChecker',Mock(side_effect=AssertionError('departure braking check')))
    speed,_,state=f.command(source(),(0.,0.,0.),True,now=0.)
    assert speed<0 and state=='EXITING'


def test_entry_collision_still_stops_with_departure_checks_disabled():
    f=Follower(config(),speed=.2)
    f.accept(Gap(1.,0.,0.,.6,0,1),[(0.,0.,0.),(1.,0.,0.)],True,(0.,0.,0.))
    speed,_,state=f.command(source(),(0.,0.,0.),True,now=0.)
    assert speed==0 and state in ('ROUTE_BLOCKED','FRONT_BLOCKED')


def test_async_departure_acceptance_skips_map_collision_recheck(node,monkeypatch):
    n,_=node
    n.cfg=replace(n.cfg,exit_collision_check_enabled=False)
    f=n.follower;f.cfg=n.cfg;f.goal=Gap(-.5,0.,0.,.6,0,1);f.parked=True
    n.activation.active=True;n.tracker.valid=True;n.tracker.pose=(0.,0.,0.)
    future=Future();future.set_result(([(0.,0.,0.),(-.5,0.,0.)],''))
    n.exit_pending=(future,n.epoch,f,(0.,0.,0.))
    n.planning_scan=Mock(side_effect=AssertionError('departure map inspection'))
    n.poll_exit_plan()
    assert f.exit_route_ready and f.path[-1][0]==-.5
    n.planning_scan.assert_not_called()
