from autorace_slow.core import SlowState


def arm(s, t):
    s.receive('slow', True, t)
    s.receive('slow', True, t + .1)


def test_slow_renewal_requires_new_quorum_and_roi():
    s = SlowState()
    arm(s, 0.)
    s.receive_white(True, .2)
    assert s.missions['slow'].deadline == 15.2
    s.receive('slow', True, 1.)
    assert s.receive_white(True, 1.05) == ()
    s.receive('slow', True, 1.1)
    assert s.missions['slow'].deadline == 15.2
    assert s.receive_white(True, 1.2, eligible_kinds=[]) == ()
    assert s.receive_white(True, 1.3) == ('slow',)
    assert s.missions['slow'].deadline == 16.3
    assert s.receive_white(True, 1.4) == ()
    arm(s, 2.)
    assert s.selected(16.3) is None
    assert not s.missions['slow'].armed


def test_crosswalk_clears_only_slow_and_allows_new_start():
    s = SlowState()
    arm(s, 0.)
    s.receive_white(True, .2)
    s.receive('bump', True, .3)
    s.receive('bump', True, .4)
    s.receive_white(True, .5)
    s.stop_slow()
    assert s.missions['slow'].deadline is None
    assert s.selected(.6) == 'bump'
    assert s.receive_white(True, .7) == ()
    arm(s, 1.)
    assert s.receive_white(True, 1.2) == ('slow',)
