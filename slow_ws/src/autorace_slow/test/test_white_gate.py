import cv2
import numpy as np
import pytest

from autorace_slow.core import SlowState
from autorace_slow.white_roi import WhiteROI, validate_white


@pytest.mark.parametrize('kind', ['slow', 'bump'])
def test_yolo_quorum_only_arms_and_one_later_white_observation_starts(kind):
    state = SlowState()
    assert state.receive_white(True, 0.) == ()
    assert not state.receive(kind, True, .1)
    assert not state.missions[kind].armed
    assert state.receive_white(True, .15) == ()
    assert not state.receive(kind, True, .2)
    assert state.missions[kind].armed
    assert state.selected(.4) is None
    assert state.receive_white(False, .4) == ()
    assert state.receive_white(True, .5) == (kind,)
    assert state.selected(.5) == kind
    assert state.missions[kind].deadline == (15.5 if kind == 'slow' else 8.5)
    assert state.receive_white(True, .6) == ()
    assert state.missions[kind].deadline == (15.5 if kind == 'slow' else 8.5)


def test_confirmed_sign_stays_armed_after_sign_disappears_and_each_kind_is_independent():
    state = SlowState()
    for i in range(2):
        state.receive('slow', True, i * .1)
    state.receive('slow', False, .3)
    state.receive('bump', True, .3)
    assert state.receive_white(True, 2.) == ('slow',)
    assert state.selected(2.) == 'slow'
    assert state.missions['bump'].deadline is None


def test_old_floor_frame_cannot_start_a_more_recently_armed_mission():
    state = SlowState()
    for i in range(2):
        state.receive('slow', True, i * .1)
        state.receive('bump', True, i * .1)
    assert state.receive_white(True, .3, eligible_kinds=['bump']) == ('bump',)
    assert state.missions['slow'].armed
    assert state.selected(.3) == 'bump'


def test_after_expiry_both_yolo_quorum_and_new_white_are_required_again():
    state = SlowState(hold_sec=1.)
    for i in range(2):
        state.receive('bump', True, i * .1)
    state.receive_white(True, .3)
    assert state.selected(1.3) is None
    assert state.receive_white(True, 1.4) == ()
    for i in range(2):
        state.receive('bump', True, 1.5 + i * .1)
    assert state.selected(1.8) is None
    assert state.receive_white(True, 1.9) == ('bump',)


def test_white_ratio_uses_whole_roi_and_ignores_white_outside_it():
    roi = WhiteROI()
    frame = np.full((200, 400, 3), 255, dtype=np.uint8)
    frame[192:200, 20:380] = 0
    ratio, bounds, mask = roi.measure(frame, 70, 120)
    assert bounds == (20, 192, 380, 200)
    assert ratio == 0 and mask.size == 360 * 8
    frame[192:200, 20:200] = 255
    ratio, _, _ = roi.measure(frame, 70, 120)
    assert ratio == pytest.approx(.5, abs=.02)


def test_hsv_rejects_yellow_and_dark_floor_with_the_lane_limits():
    roi = WhiteROI()
    for colour, expected in [((0, 255, 255), 0), ((119, 119, 119), 0), ((120, 120, 120), 1)]:
        frame = np.empty((200, 400, 3), np.uint8)
        frame[:] = colour
        ratio, _, _ = roi.measure(frame, 70, 120)
        assert ratio == expected
    with pytest.raises(ValueError):
        validate_white(float('nan'), 120)
    with pytest.raises(ValueError):
        WhiteROI(top=.95, bottom=.9)
