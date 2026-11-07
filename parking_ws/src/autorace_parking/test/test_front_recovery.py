"""Recovery requires an unreached goal and a close obstacle in front of the bumper."""
from dataclasses import replace
import math
from unittest.mock import patch

import pytest

from autorace_parking.core import Gap, Scan, Settings, front_clearance
from autorace_parking.control import Follower, build_plan_report


def config():
    return Settings(clearance=.04,allow_reverse=True,recovery_on_front_obstacle_only=True,
                    recovery_front_clearance=.10,recovery_reverse_distance=.30,
                    blind_reverse_distance=.30,recovery_speed=.2,max_recovery_attempts=1)


def scan_with_clearance(clearance=None):
    scan=Scan([math.inf]*221,math.radians(-110),math.pi/180,.01,12.,(0.,0.,0.))
    if clearance is not None:
        scan.points=[(config().vehicle_length/2+clearance,0.)]
    return scan


def target():
    return Gap(1.,-.5,-math.pi/2,.85,0,1)


def test_clearance_is_bumper_relative_and_ignores_sides_and_rear():
    cfg=config();scan=scan_with_clearance(.08)
    scan.points.extend([(-.1,0.),(.05,cfg.vehicle_width/2+cfg.clearance+.001)])
    assert front_clearance(scan,cfg)==pytest.approx(.08)
    scan.points=scan.points[1:]
    assert front_clearance(scan,cfg) is None


@pytest.mark.parametrize('distance',[.08,.10])
def test_close_front_obstacle_generates_only_thirty_cm_straight_recovery(distance):
    cfg=config();report=build_plan_report(scan_with_clearance(distance),cfg,target())
    assert report['diagnostics']['reason_code']=='recovery_ready'
    assert report['path'][-1]==pytest.approx((-.30,0.,0.))
    assert all(y==yaw==0. for _,y,yaw in report['path'])


@pytest.mark.parametrize('distance',[None,.10001,.20])
def test_no_path_without_close_front_obstacle_does_not_trigger_reverse(distance):
    cfg=config()
    with patch('autorace_parking.control.parking_candidates',return_value=[]), \
         patch('autorace_parking.control.plan_path',return_value=[]) as planner:
        report=build_plan_report(scan_with_clearance(distance),cfg,target())
    assert not report['path']
    assert report['diagnostics']['recovery_waiting_for_front_obstacle']
    assert planner.call_args.args[2].allow_reverse is False


def test_no_target_does_not_reverse_even_with_close_front_obstacle():
    report=build_plan_report(scan_with_clearance(.08),config())
    assert not report['path'] and report['diagnostics']['recovery_no_target']


def test_exhausted_attempts_stop_despite_close_front_obstacle():
    report=build_plan_report(scan_with_clearance(.08),config(),target(),recovery_allowed=False)
    assert not report['path'] and report['diagnostics']['recovery_limit_reached']


def test_front_proximity_stops_a_running_forward_path_before_arrival():
    cfg=config();f=Follower(cfg,require_parking_heading=False)
    f.accept(target(),[(0.,0.,0.),(.5,0.,0.)],False,(0.,0.,0.))
    assert f.command(scan_with_clearance(.08),(0.,0.,0.),now=0.)==(0.,0.,'FRONT_BLOCKED')
    assert not f.path and not f.recovering


def test_arrival_confirmation_precedes_front_obstacle_recovery():
    cfg=config();f=Follower(cfg,require_parking_heading=False,parking_position_tolerance=.1)
    goal=Gap(.08,0.,0.,.85,0,1)
    f.accept(goal,[(0.,0.,0.),(.08,0.,0.)],True,(0.,0.,0.))
    assert f.command(scan_with_clearance(.08),(0.,0.,0.),new_scan=True,now=0.)==(0.,0.,'CONFIRMING')
    assert not f.recovering


def test_recovery_travel_cap_uses_actual_start_pose_not_old_plan_origin():
    cfg=config();f=Follower(cfg)
    # A delayed/misanchored path must not extend the actual reverse allowance.
    f.accept(target(),[(0.,0.,0.),(-.6,0.,0.)],False,(0.,0.,0.),recovery=True)
    f.command(scan_with_clearance(),(.1,0.,0.),now=0.)
    assert f.command(scan_with_clearance(),(-.2,0.,0.),now=2.)==(0.,0.,'REOBSERVE')
    assert not f.path and not f.recovering
