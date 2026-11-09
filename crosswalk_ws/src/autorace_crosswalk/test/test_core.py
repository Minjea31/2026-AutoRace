from autorace_crosswalk.core import CrosswalkState


def test_confirmation_reset_and_no_gap_timeout():
    state = CrosswalkState()
    state.receive_white(True)
    state.receive(True)
    assert not state.tick(0., True)
    state.receive(False)
    state.receive(True)
    assert not state.tick(100., True)
    state.receive(True)
    assert not state.tick(200., True)
    assert state.phase == 'waiting_white'
    state.receive_white(False)
    assert not state.tick(200., True)
    state.receive_white(True)
    assert state.tick(200., True)


def test_full_stop_despite_false_and_cooldown_ignores_detections():
    state = CrosswalkState()
    state.receive(True)
    state.receive(True)
    state.receive(False)
    state.receive_white(True)
    assert state.tick(10., True)
    state.receive(False)
    assert state.tick(12.999, True)
    assert not state.tick(13., True)
    for value in (False, True, True, True):
        state.receive(value)
        state.receive_white(True)
        assert not state.tick(20., True)
    assert state.phase == 'cooldown'
    assert not state.tick(22.999, True)
    assert state.phase == 'cooldown'
    assert not state.tick(23., True)
    assert state.phase == 'waiting'
    assert state.confirmations == 0
    # Cooldown detections cannot supply either condition for the next stop.
    state.receive_white(True)
    state.receive(True)
    assert not state.tick(23.1, True)
    state.receive(True)
    assert not state.tick(23.2, True)
    assert state.phase == 'waiting_white'
    state.receive_white(True)
    assert state.tick(23.3, True)
    assert not state.tick(26.3, True)
    assert state.phase == 'cooldown'


def test_cooldown_starts_when_stop_publishing_completes():
    state = CrosswalkState()
    state.receive(True)
    state.receive(True)
    state.receive_white(True)
    assert state.tick(0., True)
    # A delayed timer callback must not shorten the subsequent cooldown.
    assert not state.tick(4., True)
    assert not state.tick(13.999, True)
    assert state.phase == 'cooldown'
    assert not state.tick(14., True)
    assert state.phase == 'waiting'


def test_waits_for_first_lane_before_starting_timer():
    state = CrosswalkState()
    state.receive(True)
    state.receive(True)
    state.receive_white(True)
    assert not state.tick(0., False)
    assert not state.tick(10., False)
    assert state.tick(20., True)
    assert state.tick(22.999, True)
    assert not state.tick(23., True)
