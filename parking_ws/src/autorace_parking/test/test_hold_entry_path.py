from concurrent.futures import Future
from dataclasses import replace
import math
from unittest.mock import patch

import pytest

from autorace_parking.core import Settings,Scan,Gap,advance
from autorace_parking.control import Follower
from autorace_parking.tracking import compose


def scan():
    return Scan([math.inf]*361,-math.pi,math.pi/180,.01,12.,(0.,0.,0.))


def test_keeps_full_straight_and_curve_geometry_and_rejects_replacement():
    cfg=replace(Settings(),hold_entry_path=True)
    f=Follower(cfg)
    path=[(0.,0.,0.),(.5,0.,0.)]+[advance((.5,0.,0.),i*.02,-1/cfg.turn_radius) for i in range(1,20)]
    goal=Gap(1.,-.5,-math.pi/2,.5,0,1)
    origin=(.1,.2,.3);f.accept(goal,path,False,origin)
    fixed=f.path.copy();f.index=10
    local=f.local_path(origin)
    for a,b in zip(local,path):assert a==pytest.approx(b)
    f.accept(Gap(2.,-1.,0.,.5,0,1),[(0.,0.,0.),(2.,0.,0.)],True,(0.,0.,0.))
    assert f.path==fixed and f.entry_locked and len(f.local_path(origin))==len(path)


def test_partial_endpoint_stops_without_discarding_or_replanning():
    f=Follower(replace(Settings(),hold_entry_path=True),require_parking_heading=False)
    path=[(0.,0.,0.),(.5,0.,0.)]
    f.accept(Gap(1.,-.5,0.,.5,0,1),path,False,(0.,0.,0.))
    assert f.command(scan(),(.49,0.,0.),now=0.)==(0.,0.,'PATH_HELD')
    assert f.path==path and f.entry_path==path


def test_front_stop_preserves_entry_until_recovery_is_accepted():
    cfg=replace(Settings(),hold_entry_path=True,recovery_on_front_obstacle_only=True)
    f=Follower(cfg);path=[(0.,0.,0.),(.5,0.,0.)]
    goal=Gap(1.,-.5,0.,.5,0,1);f.accept(goal,path,True,(0.,0.,0.))
    s=scan();s.points=[(.24,0.)]
    assert f.command(s,(0.,0.,0.),now=0.)==(0.,0.,'FRONT_BLOCKED')
    assert f.path==path and f.entry_locked
    reverse=[(0.,0.,0.),(-.3,0.,0.)]
    f.accept(goal,reverse,False,(0.,0.,0.),recovery=True)
    assert not f.entry_locked and not f.entry_path and f.path==reverse


def test_parked_hold_displays_entry_then_switches_to_exit_route():
    f=Follower(replace(Settings(),hold_entry_path=True),hold_seconds=3.)
    path=[(0.,0.,0.),(.5,0.,0.)]
    f.accept(Gap(.5,0.,0.,.5,0,1),path,True,(0.,0.,0.))
    f.entry_trace=path.copy();f.prepare_exit((.5,0.,0.),0.)
    assert len(f.local_path((0.,0.,0.)))==len(path)
    f.exiting=True
    assert f.local_path((0.,0.,0.))==f.path


@pytest.fixture
def node():
    import rclpy
    from rclpy.parameter import Parameter
    from std_msgs.msg import Bool
    from autorace_parking.node import ParkingNode
    rclpy.init();n=ParkingNode(parameter_overrides=[Parameter('localization_source',value='scan'),Parameter('hold_entry_path',value=True),Parameter('clearance',value=.04)])
    n.now_seconds=lambda:20.
    for _ in range(3):n.on_sign(Bool(data=True))
    n.scan=scan();n.scan_time=20.
    try:yield n
    finally:n.destroy_node();rclpy.shutdown()


def test_node_does_not_submit_new_forward_plans_while_locked_and_keeps_route_on_scan_loss(node):
    node.follower.accept(Gap(1.,-.5,0.,.5,0,1),[(0.,0.,0.),(.5,0.,0.)],False,(0.,0.,0.))
    with patch.object(node.worker,'submit') as submit:
        node.tick();submit.assert_not_called()
        node.scan_time=19.
        node.tick();submit.assert_not_called()
    assert node.state=='WAITING_SCAN' and node.follower.path and node.follower.entry_locked


def test_node_accepts_reverse_recovery_even_when_final_entry_is_locked(node):
    node.cfg=replace(node.cfg,recovery_on_front_obstacle_only=True,max_recovery_attempts=1)
    goal=Gap(1.,-.5,0.,.5,0,1)
    node.follower.accept(goal,[(0.,0.,0.),(.5,0.,0.)],True,(0.,0.,0.))
    node.state='FRONT_BLOCKED';node.scan.points=[(.24,0.)]
    future=Future();future.set_result({'goal':goal,'path':[(0.,0.,0.),(-.3,0.,0.)],
        'final':False,'diagnostics':{'reason_code':'recovery_ready'}})
    node.pending=future,node.epoch,(0.,0.,0.),20.
    node.poll_plan()
    assert node.follower.recovering and not node.follower.entry_locked
    assert node.recovery_attempts==1
