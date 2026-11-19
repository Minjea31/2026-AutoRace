"""Curve departure stops once and permanently releases the parking input."""
from concurrent.futures import Future
import json
import math
from types import SimpleNamespace
from unittest.mock import Mock

from autorace_parking.control import Follower
from autorace_parking.core import Gap,Settings,Scan,advance
from autorace_parking.departure import curve_only_route
from autorace_parking.node import ParkingNode
from test_recorded_exit_margin import entry
from test_start_and_wheel_departure import node


def test_curve_endpoint_keeps_entire_turn_and_omits_straight():
    recorded=entry()
    route=list(reversed(recorded))
    trimmed=curve_only_route(route)
    assert trimmed==list(reversed(recorded[40:]))
    assert trimmed[-1]==(.8,0.,0.)
    assert curve_only_route(trimmed)==trimmed


def test_no_curve_never_executes_straight_reverse():
    assert curve_only_route([(.8,0.,0.),(.4,0.,0.),(0.,0.,0.)])==[(.8,0.,0.)]
    assert curve_only_route([])==[]


def test_more_than_one_turn_cannot_be_used_for_curve_departure():
    recorded=entry()
    extra=[advance(recorded[-1],.3*i/20,-1.) for i in range(1,21)]
    assert curve_only_route(list(reversed(recorded+extra)))==[]


def test_follower_releases_after_stationary_curve_endpoint_inside_grid():
    cfg=Settings(exit_curve_only=True,exit_collision_check_enabled=False,continuous_replanning=True)
    f=Follower(cfg)
    f.parked=f.exiting=True
    f.entry_trace=[(0.,0.,0.)]
    f.goal=Gap(.8,0.,0.,1.,0,1,parking_grid={'origin':[.5,-.5],'yaw':0.,'width':1.,'depth':1.})
    assert f.accept_exit_route(list(reversed(entry())),'',now=0.)
    assert f.path[-1]==f.exit_path[-1]==(.8,0.,0.)
    assert (f.goal.x,f.goal.y)==(.8,0.)
    f.index=len(f.path)-1
    source=Scan([math.inf]*361,-math.pi,math.pi/180,.01,8.)
    for i in range(3):
        speed,_,state=f.command(source,(.8,0.,0.),True,now=1.+i*.1)
        assert speed==0.
        if i<2:assert state=='EXIT_CONFIRMING' and not f.exited
    assert f.exited and state=='EXITED'


def test_release_sends_one_stop_then_no_drive_guard_or_replanning(node):
    n,clock=node
    n.activation.active=True
    n.follower.exited=n.follower.parked=True
    n.follower.path=n.follower.exit_path=[(.8,0.,0.)]
    n.had_control_path=True
    commands=[];guards=[];handoffs=[];active=[]
    n.command_pub=SimpleNamespace(publish=commands.append)
    n.publish_command=lambda v,a:ParkingNode.publish_command(n,v,a)
    n.guard_pub=SimpleNamespace(publish=lambda m:guards.append(json.loads(m.data)))
    n.lane_handoff_pub=SimpleNamespace(publish=handoffs.append)
    n.active_pub=SimpleNamespace(publish=lambda m:active.append(m.data))
    pending=Future();exit_pending=Future()
    n.pending=(pending,n.epoch,(0.,0.,0.),clock[0])
    n.exit_pending=(exit_pending,n.epoch,(0.,0.,0.),clock[0])
    n.release_parking_commands()
    assert pending.cancelled() and exit_pending.cancelled()
    assert len(commands)==len(handoffs)==len(guards)==1 and active==[False]
    assert commands[0].drive.speed==0.
    assert guards[0]['phase']=='RELEASE' and guards[0]['ready'] is True
    assert n.parking_commands_released and not n.had_control_path
    assert not n.follower.path and not n.follower.exit_path
    n.motion_failed=n.manual_interrupted=True
    n.vehicle_automatic=Mock(return_value=False)
    n.worker.submit=Mock()
    n.localization_ready=Mock(return_value=False)
    for _ in range(3):
        n.release_parking_commands()
        n.tick()
        n.publish_command(0.,0.)
        n.publish_command(-.2,.1)
        n.publish_guard()
        assert n.state=='EXITED'
    assert len(commands)==len(handoffs)==len(guards)==1
    n.worker.submit.assert_not_called()
    n.localization_ready.assert_not_called()
    n.reset_motion()
    n.publish_command(0.,0.)
    assert not n.parking_commands_released and len(commands)==2


def test_release_ignores_signs_for_exactly_ten_seconds(node,monkeypatch):
    import autorace_parking.node as module
    from std_msgs.msg import Bool
    n,_=node
    wall=[100.]
    monkeypatch.setattr(module.time,'monotonic',lambda:wall[0])
    n.sign_ignore_until=0.
    n.activation.active=True
    n.follower.exited=n.follower.parked=True
    n.release_parking_commands()
    assert n.sign_ignore_until==110.
    n.floor_gate=Mock(armed=False,failure='')
    n.accepts_parking_start=Mock(return_value=True)
    wall[0]=109.999
    n.on_sign(Bool(data=True))
    n.floor_gate.on_confirmed_sign.assert_not_called()
    wall[0]=110.
    n.on_sign(Bool(data=True))
    n.floor_gate.on_confirmed_sign.assert_called_once()
