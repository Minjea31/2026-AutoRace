from dataclasses import replace
from types import SimpleNamespace
import math
import pickle

import pytest

from autorace_parking.core import Scan,Settings,cluster_points,find_gaps,CollisionChecker
from autorace_parking.obstacle_memory import ObstacleMemory
from autorace_parking.tracking import compose,inverse


def scan(points=(),ranges=None):
    s=Scan([0.]*361 if ranges is None else ranges,-math.pi,math.pi/180,.01,12.,(0.,0.,0.))
    s.points=list(points)
    return s


def cars():
    return [(x,y) for center in (.6,1.3) for x in (center-.05,center,center+.05)
            for y in (-.65,-.6,-.55)]


def test_fixed_map_survives_translation_rotation_and_preserves_gap_and_collision():
    cfg=replace(Settings(),max_gap=.7,assume_unobserved_free=True)
    m=ObstacleMemory();s=scan(cars());m.update(s,s,(0.,0.,0.),1.,cfg)
    assert len(find_gaps(cluster_points(s.points),cfg))==1
    pose=(1.,.3,math.pi/2)
    hidden=scan();m.update(hidden,hidden,pose,2.,cfg)
    snap=m.snapshot(hidden,pose,cfg)
    original=[compose(pose,(*p,0.))[:2] for p in snap.points]
    assert len(original)==len(s.points)
    assert all(min(math.dist(p,q) for q in original)<1e-9 for p in s.points)
    assert len(find_gaps(cluster_points(m.local_points((0.,0.,0.))),cfg))==1
    assert not CollisionChecker(m.snapshot(hidden,(0.,0.,0.),cfg),cfg).free((.6,-.6,0.))
    assert all(not t.observed for t in m.tracks.values())
    restored=pickle.loads(pickle.dumps(snap))
    assert restored.filtered_obstacles(cfg) is restored
    assert snap.points is not m.cells


def test_kf_preserves_ids_reduces_variance_and_smooths_measurements():
    m=ObstacleMemory();cfg=Settings();s=scan(cars());m.update(s,s,(0.,0.,0.),1.,cfg)
    ids=set(m.tracks)
    for i in range(2,12):
        shifted=scan([(x+.01,y) for x,y in cars()])
        m.update(shifted,shifted,(0.,0.,0.),i,cfg)
    assert set(m.tracks)==ids
    assert all(t.variance<.001 for t in m.tracks.values())
    assert m.tracks[0].center[0]==pytest.approx(.61,abs=.002)


@pytest.mark.parametrize('ranges',[[0.]*361,[math.nan]*361,[.3]*361])
def test_unknown_or_occluded_rays_never_erase_memory(ranges):
    cfg=Settings();m=ObstacleMemory();s=scan(cars());m.update(s,s,(0.,0.,0.),1.,cfg)
    count=len(m.cells)
    for i in range(2,7):
        empty=scan(ranges=ranges);m.update(empty,empty,(0.,0.,0.),i,cfg)
    assert len(m.cells)==count


def test_clear_rays_need_three_distinct_frames_and_remove_tracks():
    cfg=Settings();m=ObstacleMemory();s=scan(cars());m.update(s,s,(0.,0.,0.),1.,cfg)
    empty=scan(ranges=[math.inf]*361)
    for stamp in (2.,2.,3.):m.update(empty,empty,(0.,0.,0.),stamp,cfg)
    assert m.cells
    m.update(empty,empty,(0.,0.,0.),4.,cfg)
    assert not m.cells and not m.tracks


def test_outside_fov_does_not_clear_and_limit_prunes_farthest():
    cfg=Settings();m=ObstacleMemory(limit=3);s=scan([(-.5,0.),(-.7,0.),(-.9,0.),(-1.1,0.)])
    m.update(s,s,(0.,0.,0.),1.,cfg)
    assert len(m.cells)==3
    assert min(x for x,y in m.local_points((0.,0.,0.)))==-.9
    empty=Scan([math.inf]*181,-math.pi/2,math.pi/180,.01,12.,(0.,0.,0.))
    for i in range(2,6):m.update(empty,empty,(0.,0.,0.),i,cfg)
    assert len(m.cells)==3
    m.reset();assert not m.cells and m.stamp is None


def test_ros_memory_cloud_boxes_snapshot_and_reset():
    import rclpy
    from rclpy.parameter import Parameter
    from std_msgs.msg import Bool
    from sensor_msgs.msg import LaserScan
    from autorace_parking.node import ParkingNode
    rclpy.init();n=ParkingNode(parameter_overrides=[Parameter('localization_source',value='scan'),
                                                 Parameter('obstacle_memory_enabled',value=True)])
    try:
        n.now_seconds=lambda:20.
        for _ in range(3):n.on_sign(Bool(data=True))
        clouds=[];boxes=[]
        n.memory_pub=SimpleNamespace(publish=clouds.append)
        n.obstacles_pub=SimpleNamespace(publish=boxes.append)
        msg=LaserScan();msg.header.frame_id='base_link';msg.header.stamp.sec=20
        msg.angle_min=-math.pi;msg.angle_increment=math.pi/180
        msg.range_min=.01;msg.range_max=12.;msg.ranges=[1.]*361
        n.on_scan(msg);n.publish_memory()
        assert n.memory.cells and n.planning_scan() is not n.scan
        cloud=clouds[-1]
        assert cloud.header.frame_id=='base_link' and cloud.width==len(n.memory.cells)
        assert len(cloud.data)==12*cloud.width
        assert all(m.color.b==1. for m in boxes[-1].markers if m.action==0)
        n.memory.update(scan(),scan(),n.tracker.pose,21.,n.cfg)
        n.last_memory_wall=-math.inf;n.publish_memory()
        assert all(m.color.r==1. for m in boxes[-1].markers if m.action==0)
        n.sign_ignore_until=0.;n.on_sign(Bool(data=True))
        assert not n.memory.cells and n.memory.next_id==0 and clouds[-1].width==0
    finally:
        n.destroy_node();rclpy.shutdown()


def test_remembered_front_points_cannot_trigger_live_front_recovery():
    from autorace_parking.core import Gap
    from autorace_parking.control import build_plan_report
    cfg=replace(Settings(),allow_reverse=True,recovery_on_front_obstacle_only=True,
                recovery_reverse_distance=.3,blind_reverse_distance=.3,assume_unobserved_free=True)
    raw=scan([(.24,0.)]);m=ObstacleMemory();m.update(raw,raw,(0.,0.,0.),1.,cfg)
    live=scan()
    report=build_plan_report(m.snapshot(live,(0.,0.,0.),cfg),cfg,
                            Gap(1.,-.5,-math.pi/2,.5,0,1),front_scan=live)
    assert report['diagnostics']['reason_code']!='recovery_ready'


@pytest.mark.parametrize('group',[0,3])
def test_recorded_stationary_scans_keep_hidden_points_and_reject_unaligned_candidates(group):
    import json,yaml
    from dataclasses import fields
    from pathlib import Path
    package=Path(__file__).resolve().parents[1]
    params=yaml.safe_load((package/'config/parking_real.yaml').read_text())['parking_node']['ros__parameters']
    cfg=Settings(**{f.name:params[f.name] for f in fields(Settings) if f.name in params})
    frames=json.loads((package/'test/data/limo_pro_stationary_scans.json').read_text())['scans']
    m=ObstacleMemory()
    for i in range(group,group+3):
        raw=Scan(**frames[i]);m.update(raw,raw.filtered_obstacles(cfg),(0.,0.,0.),float(i+1),cfg)
    points=m.local_points((0.,0.,0.));goals=find_gaps(cluster_points(points),cfg)
    assert not goals and m.tracks and points
    empty=scan();m.update(empty,empty,(0.,0.,0.),10.,cfg)
    remaining=find_gaps(cluster_points(m.snapshot(empty,(0.,0.,0.),cfg).points),cfg)
    assert len(remaining)==len(goals)
    assert len(m.cells)==len(points)
