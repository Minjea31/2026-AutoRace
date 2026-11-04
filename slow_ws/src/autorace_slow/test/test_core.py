from autorace_slow.core import SlowState


def activate(state, kind, start=0.):
    for i in range(3):
        state.receive(kind, True, start+i*.1)


def test_confirmation_false_gap_and_independent_inputs():
    s = SlowState()
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


def test_single_true_after_five_seconds_renews_eight_second_hold():
    s = SlowState()
    activate(s, 'slow')
    assert s.missions['slow'].deadline == 8.2
    assert not s.receive('slow', True, 5.2)
    assert s.missions['slow'].deadline == 13.2
    assert s.selected(8.2) == 'slow'
    assert s.selected(13.199) == 'slow'
    assert s.selected(13.2) is None
    s.receive('slow', True, 13.3)
    s.receive('slow', True, 13.4)
    assert s.selected(13.4) is None
    assert s.receive('slow', True, 13.5)


def test_bump_priority_independent_deadlines_and_fallback():
    s = SlowState()
    activate(s, 'bump')
    activate(s, 'slow', 1.)
    assert s.selected(2.) == 'bump'
    s.receive('slow', True, 7.)
    assert s.selected(8.2) == 'slow'
    assert s.selected(15.) is None


def test_false_does_not_extend_and_expiry_in_receive_needs_new_confirmation():
    s = SlowState()
    activate(s, 'bump')
    s.receive('bump', False, 5.)
    assert not s.receive('bump', True, 8.2)
    assert s.selected(8.2) is None
    assert s.missions['bump'].count == 1
