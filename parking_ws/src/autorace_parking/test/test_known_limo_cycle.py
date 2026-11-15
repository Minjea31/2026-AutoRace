"""Closed-loop nominal bicycle simulation: surface registration through exit."""
import math
from copy import copy
from pathlib import Path
from dataclasses import fields,replace
import pytest
import yaml
from autorace_parking.core import Settings,Scan,advance
from autorace_parking.slot_map import reference_centers
from autorace_parking.control import build_plan_report,Follower
from autorace_parking.departure import reverse_entry_route,anchored_reverse_entry,arc_then_straight,arc_straight_departure
from autorace_parking.tracking import compose,inverse


@pytest.mark.parametrize('curve_only',[False,True])
def test_known_body_map_entry_hold_reverse_arc_straight_and_exit(curve_only):
    root=Path(__file__).resolve().parents[4]
    p=yaml.safe_load((root/'parking_ws/src/autorace_parking/config/parking_real.yaml').read_text())['parking_node']['ros__parameters']
    cfg=Settings(**{f.name:p[f.name] for f in fields(Settings) if f.name in p})
    assert cfg.exit_curve_only
    cfg=replace(cfg,exit_curve_only=curve_only)
    assert cfg.slot_object_box_enabled and cfg.slot_object_length==.322 and cfg.slot_object_width==.22
    source=Scan([math.inf]*361,-math.pi,math.pi/180,.01,8.);source.points=[]
    for i,(x,y) in enumerate(reference_centers(cfg)+(1.6,-.8)):
        if i==4:continue
        source.points.extend((float(x+dx),float(y+cfg.slot_object_length/2))
                             for dx in (-.11,-.08,-.04,0.,.04,.08,.11))
    points=tuple(source.points)
    result=build_plan_report(source,cfg,None,(0.,0.,0.),[(0.,0.,0.)],True,cfg.blind_reverse_distance,True,source,0,False,None)
    assert result['path'] and result['final'] and tuple(source.points)==points
    f=Follower(cfg,p['parking_speed'],p['lookahead'],p['parking_hold_seconds'],p['max_deceleration'],
               p['max_yaw_rate'],p['exit_heading_tolerance'],p['require_parking_heading'],
               p['parking_position_tolerance'],p['max_steering_rate'],p['min_turn_speed'])
    f.accept(result['goal'],result['path'],True,(0.,0.,0.))
    def departure(trace,pose,targets,scan,cfg):
        route=reverse_entry_route(f.entry_path,pose,scan,cfg) or anchored_reverse_entry(f.entry_path,pose,scan,cfg)
        if route and arc_then_straight(route):return route,''
        return arc_straight_departure(trace,pose,scan,cfg)
    f.exit_planner=departure
    pose=(0.,0.,0.);states=set();parked_at=exiting_at=None
    for i in range(2400):
        now=i*.05;local=copy(source);transform=inverse(pose)
        local.points=[compose(transform,(*q,0.))[:2] for q in source.points]
        local.sensor_pose=(.103,0.,0.)
        speed,steering,state=f.command(local,pose,i%2==0,now=now)
        states.add(state)
        if state=='PARKED' and parked_at is None:parked_at=now
        if state=='EXITING' and exiting_at is None:exiting_at=now
        scale=cfg.steering_response_scale*(cfg.reverse_steering_response_scale if speed<0 else 1.)
        pose=advance(pose,speed*.05,scale*math.tan(steering)/cfg.wheelbase)
        if f.exited:break
    assert f.exited and {'PARKING','PARKED','EXITING','EXITED'}<=states
    assert exiting_at-parked_at>=p['parking_hold_seconds']-.051
    assert arc_then_straight(f.exit_path)
    if curve_only:
        assert math.dist(pose[:2],f.exit_path[-1][:2])<.035
        assert f.exit_path[-1][0]>.4  # release at arc end, before returning along the straight
        assert all(abs(b[2]-a[2])>.0001 for a,b in zip(f.exit_path,f.exit_path[1:]))
    else:
        assert math.hypot(*pose[:2])<.12
