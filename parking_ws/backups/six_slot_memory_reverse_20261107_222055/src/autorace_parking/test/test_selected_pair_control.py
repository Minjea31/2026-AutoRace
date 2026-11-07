from concurrent.futures import Future
from dataclasses import fields,replace
import math
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import pytest
import yaml

from autorace_parking.core import Settings,Scan,find_gaps,cluster_points
from autorace_parking.control import build_plan_report
from autorace_parking.obstacle_memory import ObstacleMemory


def profile():
    params=yaml.safe_load((Path(__file__).resolve().parents[1]/'config/parking_real.yaml').read_text())['parking_node']['ros__parameters']
    return replace(Settings(**{f.name:params[f.name] for f in fields(Settings) if f.name in params}),
                   enclosed_space_detection=False,continuous_replanning=False)


def scene():
    s=Scan([0.]*361,-math.pi,math.pi/180,.01,12.,(0.,0.,0.))
    s.points=[(cx+dx,y+dy) for cx,y in [(.8,-.5),(1.6,-.5),(2.,.5)]
              for dx in [-.1,0.,.1] for dy in [-.1,0.,.1]]
    return s


def test_goal_is_measured_gap_center_without_depth_extension_and_tracks_are_reported():
    cfg=profile();raw=scene();m=ObstacleMemory();m.update(raw,raw,(0.,0.,0.),20.,cfg)
    snap=m.snapshot(raw,(0.,0.,0.),cfg);report=build_plan_report(snap,cfg)
    assert not cfg.goal_from_observed_front
    assert report['diagnostics']['target']['x']==pytest.approx(1.2)
    assert report['diagnostics']['target']['y']==pytest.approx(-.5)
    pair=report['diagnostics']['selected_pair']
    assert len(pair)==2 and {p['track_id'] for p in pair}=={0,1}


@pytest.fixture
def node():
    import rclpy
    from rclpy.parameter import Parameter
    from std_msgs.msg import Bool
    from autorace_parking.node import ParkingNode
    rclpy.init();n=ParkingNode(parameter_overrides=[Parameter('localization_source',value='scan'),Parameter('obstacle_memory_enabled',value=True)])
    n.now_seconds=lambda:20.
    for _ in range(3):n.on_sign(Bool(data=True))
    n.scan=scene();n.scan_time=20.
    try:yield n
    finally:n.destroy_node();rclpy.shutdown()


def test_selected_memory_boxes_are_magenta_and_goal_visible_without_accepted_route(node):
    cfg=profile();node.memory.update(node.scan,node.scan,(0.,0.,0.),20.,cfg)
    report=build_plan_report(node.memory.snapshot(node.scan,(0.,0.,0.),cfg),cfg)
    report['path']=[];report['final']=False
    f=Future();f.set_result(report);node.pending=f,node.epoch,(0.,0.,0.),20.
    node.poll_plan()
    batches=[];targets=[]
    node.obstacles_pub=SimpleNamespace(publish=batches.append)
    node.target_pub=SimpleNamespace(publish=targets.append)
    node.publish_memory();node.publish_target()
    boxes={m.id:m for m in batches[-1].markers if m.action==0}
    assert all(boxes[i].color.r==boxes[i].color.b==1. and boxes[i].color.g==0. for i in [0,1])
    assert boxes[2].color.r==0.
    arrow=next(m for m in targets[-1].markers if m.ns=='parking_goal')
    assert arrow.pose.position.x==pytest.approx(1.2)
    assert arrow.pose.position.y==pytest.approx(-.5)


def test_no_path_sends_no_parking_commands_and_path_loss_stops_only_once(node):
    from autorace_parking.core import Gap
    outputs=[];node.publish_command=lambda v,a:outputs.append((v,a))
    with patch.object(node.worker,'submit',return_value=Future()):
        node.tick();node.tick()
    assert outputs==[]
    node.follower.accept(Gap(1.,0.,0.,.5,0,1),[(0.,0.,0.),(1.,0.,0.)],True,(0.,0.,0.))
    node.tick();assert outputs[-1][0]>0.
    node.follower.path=[];node.follower.goal=None
    with patch.object(node.worker,'submit',return_value=Future()):
        node.tick();node.tick()
    assert outputs[-1]==(0.,0.) and len(outputs)==2
    assert node.activation.active


def test_mux_selects_lane_without_path_and_parking_when_path_arrives(node):
    from autorace_parking.core import Gap
    from std_msgs.msg import Bool
    mux=pytest.importorskip('autorace_mux.core')
    arb=mux.Arbitrator(mux.Config());clock=[0.]
    def receive(source,v,a):
        now_ns=20000000000+int(clock[0]*1e9)
        assert arb.receive(source,v,a,now_ns,clock[0],now_ns)[0]
    node.publish_command=lambda v,a:receive('parking',v,a)
    def select():
        receive('lane',.2,0.)
        return arb.select(clock[0],20000000000+int(clock[0]*1e9)).source
    with patch.object(node.worker,'submit',return_value=Future()):
        node.tick()
        assert select()=='lane'
        node.sign_ignore_until=0.;node.on_sign(Bool(data=True))
        assert select()=='lane'  # Repeated signs alone must not seize control.
        node.scan=scene();node.scan_time=20.
        goal=Gap(1.,0.,0.,.5,0,1)
        node.follower.accept(goal,[(0.,0.,0.),(1.,0.,0.)],True,(0.,0.,0.))
        node.tick();assert select()=='parking'
        node.follower.path=[];node.follower.goal=None
        node.tick();assert select()=='parking'  # One final stop expires after 0.25 s.
        clock[0]=.26
        node.tick();assert select()=='lane'
        node.follower.accept(goal,[(0.,0.,0.),(1.,0.,0.)],True,(0.,0.,0.))
        node.tick();assert select()=='parking'
