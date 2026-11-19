import pytest
from autorace_slow.core import SlowState


def arm(state, now):
    state.receive('slow', True, now)
    state.receive('slow', True, now+.1)


def test_first_actual_start_runs_six_seconds_and_repeated_publications_cannot_renew_it():
    s = SlowState(first_slow_hold_sec=6., start_delay_sec=.5)
    arm(s, 0.)
    assert s.slow_missions_started == 0
    assert s.receive_white(True, .2) == ('slow',)
    assert s.slow_missions_started == 0 and s.selected(.699) is None
    assert s.selected(.7) == 'slow' and s.slow_missions_started == 1
    for i in range(58):
        now = .8+i*.1
        s.receive('slow', True, now)
        s.receive_white(True, now)
        assert s.selected(now) == 'slow'
        assert s.slow_missions_started == 1
        assert s.missions['slow'].deadline == pytest.approx(6.7)
    assert s.selected(6.7) is None
    for i in range(30):
        s.receive('slow', True, 6.8+i*.1)
        s.receive_white(True, 6.8+i*.1)
        assert s.selected(6.8+i*.1) is None
        assert s.slow_missions_started == 1


def test_second_and_third_missions_use_existing_fifteen_second_hold_and_renewal():
    s = SlowState(first_slow_hold_sec=6., start_delay_sec=.5)
    arm(s, 0.); s.receive_white(True, .2); s.selected(.7); s.selected(6.7)
    s.receive('slow', False, 6.8)
    arm(s, 7.); s.receive_white(True, 7.2)
    assert s.selected(7.7) == 'slow' and s.slow_missions_started == 2
    assert s.missions['slow'].deadline == pytest.approx(22.7)
    arm(s, 10.); s.receive_white(True, 10.2)
    assert s.missions['slow'].deadline == pytest.approx(25.7)
    assert s.slow_missions_started == 2
    assert s.selected(25.7) is None
    arm(s, 26.); s.receive_white(True, 26.2)
    assert s.selected(26.7) == 'slow' and s.slow_missions_started == 3
    assert s.missions['slow'].deadline == pytest.approx(41.7)


def test_canceled_confirmation_or_scheduled_start_does_not_consume_first_mission():
    s = SlowState(first_slow_hold_sec=6., start_delay_sec=.5)
    arm(s, 0.); s.stop_slow()
    arm(s, 1.); s.receive_white(True, 1.2); s.stop_slow()
    assert s.slow_missions_started == 0
    arm(s, 2.); s.receive_white(True, 2.2)
    assert s.selected(2.7) == 'slow'
    assert s.missions['slow'].ordinal == 1
    assert s.missions['slow'].deadline == pytest.approx(8.7)
    s.stop_slow()
    s.receive('slow', False, 2.8)
    arm(s, 3.); s.receive_white(True, 3.2)
    assert s.selected(3.7) == 'slow'
    assert s.missions['slow'].ordinal == 2
    assert s.missions['slow'].deadline == pytest.approx(18.7)


def test_bump_missions_do_not_change_the_first_slow_count():
    s = SlowState(first_slow_hold_sec=6., require_white_mark=False)
    s.receive('bump', True, 0.); s.receive('bump', True, .1)
    assert s.selected(.1) == 'bump' and s.slow_missions_started == 0
    arm(s, 1.)
    assert s.missions['slow'].ordinal == 1
    assert s.missions['slow'].deadline == pytest.approx(7.1)
