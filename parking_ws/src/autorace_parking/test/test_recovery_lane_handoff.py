from concurrent.futures import Future
from dataclasses import replace
from unittest.mock import patch
import math
import pytest
from autorace_parking.core import Gap,Scan
from test_operator_ros import node,activate


def clear():return Scan([math.inf]*361,-math.pi,math.pi/180,.01,12.)


def finished_recovery(node):
    activate(node);node.scan=clear();node.scan_time=20.
    node.cfg=replace(node.cfg,continuous_replanning=True,assume_unobserved_free=True)
    node.follower.cfg=node.cfg
    goal=Gap(1.,-.7,0.,.5,0,1)
    node.follower.accept(goal,[(-i*.02,0.,0.) for i in range(16)],False,(0.,0.,0.),recovery=True)
    node.follower.recovering=False;node.follower.recovery_finished=True
    node.follower.last_drive_direction=-1
    node.had_control_path=True;node.tracker.valid=True
    return goal


def tick_new_scan(node):
    node.scan_sequence+=1;node.tick()


def release(node):
    with patch.object(node.worker,'submit',return_value=Future()):
        for _ in range(3):tick_new_scan(node)
    assert node.recovery_lane_released and not node.follower.path


def result(node,goal,path,recovery=False,origin=(0.,0.,0.)):
    future=Future();future.set_result({'goal':goal,'path':path,'final':not recovery,'diagnostics':{'reason_code':'recovery_ready' if recovery else 'path_ready'}})
    node.pending=future,node.epoch,origin,20.;node.poll_plan()


def test_reverse_stop_confirmation_then_silence_persists_while_lane_moves(node):
    finished_recovery(node);commands=[];node.publish_command=lambda v,a:commands.append((v,a))
    # A moving reverse vehicle must continue receiving stop, not hand off yet.
    node.follower.measured_speed=.12;node.update_recovery_handoff(True)
    assert node.recovery_stop_confirmations==0 and not node.recovery_lane_released
    node.follower.measured_speed=0.
    release(node);count=len(commands)
    assert count==3 and commands==[(0.,0.)]*3
    assert node.state=='LANE_REPLAN'
    node.follower.measured_speed=.2
    with patch.object(node.worker,'submit',return_value=Future()):
        for _ in range(5):tick_new_scan(node)
    assert len(commands)==count  # No repeated zero commands while lane owns motion.
    assert node.recovery_lane_released and not node.had_control_path


def test_only_fresh_scans_count_toward_reverse_stop_confirmation(node):
    finished_recovery(node)
    for _ in range(10):node.update_recovery_handoff(False)
    assert node.recovery_stop_confirmations==0 and not node.recovery_lane_released


def test_released_observer_replans_without_issuing_another_reverse(node):
    goal=finished_recovery(node);release(node)
    result(node,goal,[(-i*.02,0.,0.) for i in range(16)],recovery=True)
    assert node.recovery_lane_released and not node.follower.path
    node.last_plan_wall=-math.inf;node.pending=None
    with patch.object(node.worker,'submit',return_value=Future()) as submit:
        tick_new_scan(node)
    assert submit.called and submit.call_args.args[6] is False
    assert node.state=='LANE_REPLAN'


def test_safe_forward_route_reacquires_parking_but_delayed_disconnected_one_does_not(node):
    goal=finished_recovery(node);release(node)
    result(node,goal,[(i*.02,0.,0.) for i in range(51)],origin=(1.,0.,0.))
    assert node.recovery_lane_released and not node.follower.path
    result(node,goal,[(i*.02,0.,0.) for i in range(51)])
    assert not node.recovery_lane_released and node.follower.path
    assert node.follower.last_drive_direction==1 and not node.follower.recovery_finished
    commands=[];node.publish_command=lambda v,a:commands.append((v,a))
    with patch.object(node.worker,'submit',return_value=Future()):tick_new_scan(node)
    assert commands[-1][0]>0.


def test_mux_returns_to_lane_after_parking_command_expires(node):
    from pathlib import Path
    from conftest import ROOT
    import sys
    sys.path.insert(0,str(ROOT/'mux_ws/src/autorace_mux'))
    mux=pytest.importorskip('autorace_mux.core');arb=mux.Arbitrator(mux.Config());clock=[0.]
    def receive(source,v,a):
        ns=20000000000+round(clock[0]*1e9)
        assert arb.receive(source,v,a,ns,clock[0],ns)[0]
    def chosen():
        receive('lane',.2,0.)
        return arb.select(clock[0],20000000000+round(clock[0]*1e9)).source
    finished_recovery(node);node.publish_command=lambda v,a:receive('parking',v,a)
    release(node);assert chosen()=='parking'
    clock[0]=.3
    with patch.object(node.worker,'submit',return_value=Future()):tick_new_scan(node)
    assert chosen()=='lane'
    result(node,Gap(1.,0.,0.,.5,0,1),[(i*.02,0.,0.) for i in range(51)])
    with patch.object(node.worker,'submit',return_value=Future()):tick_new_scan(node)
    assert chosen()=='parking'
