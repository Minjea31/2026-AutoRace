import math
from dataclasses import replace

import pytest

from autorace_parking.core import Settings, Gap, Scan
from autorace_parking.control import Follower


def follower():
    result = Follower(Settings())
    result.parked = result.exiting = True
    result.entry_trace = [(0., 0., 0.)]
    result.path = [(0., 0., .6), (-.2, -.1, .3), (-.4, -.1, 0.), (.2, -.1, 0.)]
    return result


def test_lane_alignment_does_not_skip_a_later_reverse_curve():
    f = follower()
    assert not f.exit_curve_complete((0., 0., 0.))
    f.index = 1
    assert not f.exit_curve_complete((-.2, -.1, .3))


def test_turn_complete_can_skip_remaining_forward_tail():
    f = follower(); f.index = 2
    assert f.exit_curve_complete((-.4, -.1, 0.))
    assert not f.exit_curve_complete((-.4, -.1, .06))
    f.index = 3
    assert f.exit_curve_complete((-.4, -.1, .01))


def test_remaining_straight_reverse_and_s_turn_are_not_skipped():
    f = follower()
    f.path = [(0., 0., 0.), (-.2, 0., 0.), (-.6, 0., 0.)]
    assert not f.exit_curve_complete((0., 0., 0.))
    f.index = 2
    assert f.exit_curve_complete((-.6, 0., 0.))
    f.index = 0
    f.path = [(0., 0., 0.), (-.2, 0., .3), (-.6, 0., 0.)]
    assert not f.exit_curve_complete((0., 0., 0.))


def test_many_small_reverse_turns_cannot_evade_curve_check():
    f = follower()
    f.path = [(-.05*i, 0., .03*(i%2)) for i in range(8)]
    assert not f.exit_curve_complete((0., 0., 0.))


@pytest.mark.parametrize('exiting,recovering,exited', [(False, False, False), (True, True, False), (True, False, True)])
def test_other_mission_phases_cannot_handoff(exiting, recovering, exited):
    f = follower(); f.index = 2
    f.exiting, f.recovering, f.exited = exiting, recovering, exited
    assert not f.exit_curve_complete((-.4, -.1, 0.))


def test_remaining_forward_segment_is_held_without_positive_command():
    f = follower(); f.index = 2; f.motion_direction = 1; f.exit_route_ready = True
    f.goal = Gap(.2, -.1, 0., .8, 0, 1)
    scan = Scan([10.]*361, -math.pi, math.pi/180, .01, 12.)
    speed, steering, state = f.command(scan, (-.4, -.1, 0.), True, now=1.)
    assert speed == 0 and state == 'EXIT_CONFIRMING'
    assert not f.exited


def test_zero_length_aligned_departure_can_still_finish():
    f = follower()
    f.path = [(0., 0., 0.)]
    assert f.exit_curve_complete((0., 0., 0.))
    assert not f.exit_curve_complete((0., 0., .2))


def test_real_policy_can_skip_straight_reverse_after_the_last_curve_only():
    f=follower();f.cfg=replace(f.cfg,exit_curve_handoff=True)
    f.path=[(0.,0.,.6),(-.2,-.1,.3),(-.4,-.1,0.),(-1.,-.1,0.)]
    assert not f.exit_curve_complete((0.,0.,0.))
    f.index=2
    assert f.exit_curve_complete((-.4,-.1,0.))
    assert not f.exit_curve_complete((-.4,-.1,0.),final_arrival=True)
    assert not f.exit_curve_complete((-.4,-.1,.06))
    assert not f.exit_curve_complete((-.4,-.3,0.))
    f.goal=Gap(-1.,-.1,0.,.6,0,1,parking_grid={'origin':[-.4,-.1],'yaw':0.,'width':1.,'depth':1.})
    assert not f.exit_curve_complete((-.4,-.1,0.))


def test_early_handoff_keeps_a_later_s_turn_and_accumulated_small_curves():
    f=follower();f.cfg=replace(f.cfg,exit_curve_handoff=True)
    f.path=[(0.,0.,0.),(-.2,0.,0.),(-.4,-.1,.3),(-.6,-.1,0.)]
    assert not f.exit_curve_complete((0.,0.,0.))
    f.path=[(-.05*i,0.,.03*(i%2)) for i in range(8)]
    assert not f.exit_curve_complete((0.,0.,0.))
