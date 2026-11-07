"""Operator-selected gap, visibility and fixed curve steering policies."""
from dataclasses import fields,replace
import math
from pathlib import Path

import pytest
import yaml

from autorace_parking.core import Cluster,CollisionChecker,Gap,Scan,Settings,advance,find_gaps
from autorace_parking.control import Follower,build_plan_report


def real_config():
    p=Path(__file__).resolve().parents[1]/'config/parking_real.yaml'
    values=yaml.safe_load(p.read_text())['parking_node']['ros__parameters']
    return Settings(**{f.name:values[f.name] for f in fields(Settings) if f.name in values})


def scan():
    return Scan([0.]*221,math.radians(-110),math.pi/180,.01,12.,(0.,0.,0.))


@pytest.mark.parametrize('width,accepted',[(.299,False),(.30,True),(.50,True),(.80,True),(.801,False)])
@pytest.mark.parametrize('offset',[0.,.16,.16001,.5])
def test_real_gap_limits_require_lateral_alignment(width,accepted,offset):
    cfg=real_config()
    first=Cluster([(.5,-.8),(.6,-.8),(.6,-.6),(.5,-.6)])
    second=Cluster([(.6+width,-.8-offset),(.7+width,-.8-offset),
                    (.7+width,-.6-offset),(.6+width,-.6-offset)])
    gaps=find_gaps([first,second],cfg)
    assert bool(gaps)==(accepted and offset <= cfg.row_tolerance)
    if gaps:assert gaps[0].width==pytest.approx(width)


def test_unobserved_sectors_and_zero_ranges_are_free_but_measured_obstacles_are_not():
    cfg=real_config();raw=scan();checker=CollisionChecker(raw,cfg)
    assert cfg.assume_unobserved_free
    assert checker.free((.8,0.,0.)) and checker.free((-.6,0.,0.))
    assert not CollisionChecker(raw,replace(cfg,assume_unobserved_free=False)).free((.8,0.,0.))
    raw.points=[(.8,0.)]
    assert not CollisionChecker(raw,cfg).free((.8,0.,0.))
    assert raw.ranges==[0.]*221


def test_zero_range_visibility_does_not_prevent_generating_an_entry_path():
    cfg=real_config()
    report=build_plan_report(scan(),cfg,Gap(1.1,-.5,-math.pi/2,.5,0,1))
    assert report['final'] and report['path']
    assert report['diagnostics']['assume_unobserved_free']


@pytest.mark.parametrize('side',[-1,1])
@pytest.mark.parametrize('reverse',[False,True])
def test_curved_segment_holds_exact_maximum_angle_despite_pursuit_error(side,reverse):
    cfg=replace(real_config(),max_steering_on_curves=True);f=Follower(cfg,lookahead=.25)
    points=[advance((0.,0.,0.),i*.01,side/cfg.turn_radius) for i in range(61)]
    if reverse:points=list(reversed(points))
    f.accept(Gap(2.,-2.,0.,.5,0,1),points,False,(0.,0.,0.))
    f.last_drive_direction=f.motion_direction
    for i in [0,5,10]:
        p=points[i]
        pose=(p[0],p[1]+.015,p[2]+.025)
        _,angle,state=f.command(scan(),pose,now=i*.1)
        assert angle==pytest.approx(side*cfg.steering_limit)
        assert state in ('APPROACHING','REVERSING')


def test_straight_segment_stays_zero_until_curve_entry_then_releases_after_curve():
    cfg=replace(real_config(),max_steering_on_curves=True);f=Follower(cfg,lookahead=.25)
    straight=[(i*.01,0.,0.) for i in range(11)]
    curve=[advance(straight[-1],i*.01,-1/cfg.turn_radius) for i in range(1,41)]
    after=[advance(curve[-1],i*.01,0.) for i in range(1,31)]
    points=straight+curve+after
    f.accept(Gap(2.,-2.,0.,.5,0,1),points,False,(0.,0.,0.))
    # The lookahead already lies in the curve; the current segment is still straight.
    assert f.command(scan(),(0.,0.,0.),now=0.)[1]==0.
    assert f.command(scan(),curve[5],now=1.)[1]==pytest.approx(-cfg.steering_limit)
    f.command(scan(),curve[25],now=1.5)  # Stay within the controller's local waypoint window.
    assert f.command(scan(),after[5],now=2.)[1]==0.
