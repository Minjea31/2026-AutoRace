from autorace_slow.core import SlowState


def activate(state, kind, start=0.):
    for i in range(3):
        state.receive(kind, True, start+i*.1)


def test_confirmation_false_gap_and_independent_inputs():
    s = SlowState(confirm_frames=3, require_white_mark=False)
    s.receive('slow', True, 0.)
    s.receive('bump', True, .1)
    s.receive('slow', True, .2)
    assert s.selected(.2) is None
    s.receive('slow', False, .3)
    s.receive('slow', True, .4)
    s.receive('slow', True, 1.)
    assert s.missions['slow'].count == 1
    s.receive('slow', True, 1.1)
    assert s.receive('slow', True, 1.2)
    assert s.selected(1.2) == 'slow'
    s.receive('slow', False, 1.3)
    assert s.selected(1.3) == 'slow'


def test_bump_single_true_after_five_seconds_renews_ten_second_hold():
    s = SlowState(confirm_frames=3, require_white_mark=False)
    activate(s, 'bump')
    assert s.missions['bump'].deadline == 10.2
    assert not s.receive('bump', True, 5.2)
    assert s.missions['bump'].deadline == 15.2
    assert s.selected(10.2) == 'bump'
    assert s.selected(15.199) == 'bump'
    assert s.selected(15.2) is None
    s.receive('bump', True, 15.3)
    s.receive('bump', True, 15.4)
    assert s.selected(15.4) is None
    assert s.receive('bump', True, 15.5)


def test_bump_priority_independent_deadlines_and_fallback():
    s = SlowState(confirm_frames=3, require_white_mark=False)
    activate(s, 'bump')
    activate(s, 'slow', 1.)
    assert s.selected(2.) == 'bump'
    s.receive('slow', True, 7.)
    assert s.selected(10.2) == 'slow'
    assert s.selected(16.2) is None


def test_false_does_not_extend_and_expiry_in_receive_needs_new_confirmation():
    s = SlowState(confirm_frames=3, require_white_mark=False)
    activate(s, 'bump')
    s.receive('bump', False, 5.)
    assert not s.receive('bump', True, 10.2)
    assert s.selected(10.2) is None
    assert s.missions['bump'].count == 1


def test_white_delay_preserves_full_hold_and_repeated_signs_do_not_postpone():
    for kind, hold in [('slow', 15.), ('bump', 10.)]:
        s = SlowState(start_delay_sec=.5)
        s.receive(kind, True, 0.)
        s.receive(kind, True, .1)
        assert s.receive_white(True, .25) == (kind,)
        assert s.selected(.749) is None
        s.receive(kind, True, .4)
        s.receive(kind, True, .5)
        assert s.receive_white(True, .6) == ()
        assert s.selected(.75) == kind
        assert s.missions[kind].deadline == .75 + hold
        assert s.selected(.75 + hold) is None


def test_stop_event_cancels_a_scheduled_slow_start():
    s = SlowState(start_delay_sec=.5)
    s.receive('slow', True, 0.)
    s.receive('slow', True, .1)
    s.receive_white(True, .25)
    s.stop_slow()
    assert s.selected(2.) is None
    assert s.receive_white(True, 2.) == ()
