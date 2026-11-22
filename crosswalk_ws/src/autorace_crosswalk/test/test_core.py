from autorace_crosswalk.core import CrosswalkState


def test_confirmation_reset_and_no_gap_timeout():
    state = CrosswalkState(stop_hold_sec=3.)
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
    assert not state.tick(26.301, True)
    assert state.phase == 'cooldown'


def test_cooldown_starts_when_stop_publishing_completes():
    state = CrosswalkState(stop_hold_sec=3.)
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
    state = CrosswalkState(stop_hold_sec=3.)
    state.receive(True)
    state.receive(True)
    state.receive_white(True)
    assert not state.tick(0., False)
    assert not state.tick(10., False)
    assert state.tick(20., True)
    assert state.tick(22.999, True)
    assert not state.tick(23., True)


def test_white_detection_delay_is_not_postponed_and_full_stop_starts_after_delay():
    state=CrosswalkState(stop_delay_sec=.5,stop_hold_sec=3.)
    state.receive(True)
    state.receive(True)
    assert not state.receive_white(False,10.)
    assert state.receive_white(True,10.)
    assert state.ready_at==10.5
    assert not state.receive_white(True,10.2)
    state.receive(True)
    assert state.ready_at==10.5
    assert not state.tick(10.499,True)
    assert state.started_at is None
    assert state.tick(10.5,True)
    assert state.tick(13.499,True)
    assert not state.tick(13.5,True)
    assert state.phase=='cooldown'
    assert not state.tick(23.5,True)
    assert state.phase=='waiting' and state.ready_at is None
    state.receive(True)
    state.receive(True)
    state.receive_white(True,24.)
    assert not state.tick(24.499,True)
    assert state.tick(24.5,True)


def test_delay_expires_without_lane_but_stop_duration_waits_for_lane():
    state=CrosswalkState(stop_delay_sec=.5,stop_hold_sec=3.)
    state.receive(True)
    state.receive(True)
    state.receive_white(True,10.)
    assert not state.tick(10.5,False)
    assert not state.tick(11.,False)
    assert state.tick(12.,True)
    assert state.started_at==12.
    assert state.tick(14.999,True)
    assert not state.tick(15.,True)
