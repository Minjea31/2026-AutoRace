import pytest
from autorace_parking.sign_motion import SignMotionGate


def feed(gate, speed, start, duration, dt=.05):
    for i in range(round(duration/dt)+1):
        wall=start+i*dt
        gate.update(wall, speed, wall)
    return wall


def test_startup_stationary_and_ordinary_stops_do_not_block_signs():
    gate=SignMotionGate()
    assert gate.allows(0.)
    feed(gate,0.,10.,20.);assert gate.allows(30.)
    feed(gate,.2,30.05,.2);assert gate.allows(30.25)
    feed(gate,0.,30.3,10.);assert gate.allows(40.3)
    feed(gate,-.2,40.35,.2);assert gate.allows(40.55)


def test_only_explicit_parking_stop_starts_five_seconds_from_stop():
    gate=SignMotionGate();feed(gate,.2,10.,.2)
    gate.on_parking_stop(11.)
    feed(gate,0.,11.,4.9);assert not gate.allows(15.999)
    assert gate.allows(16.)  # No departure is needed to finish this cooldown.
    feed(gate,0.,16.05,5.);assert gate.allows(21.05)
    feed(gate,.2,21.1,.2);assert gate.allows(21.3)


def test_feedback_stops_gaps_and_duplicates_do_not_extend_stop_timer():
    gate=SignMotionGate();gate.on_parking_stop(10.)
    gate.update(10.,0.,10.);gate.update(10.,0.,13.)
    gate.update(14.,0.,14.)
    assert not gate.allows(14.999)
    assert gate.allows(15.)
    assert gate.allows(30.)


def test_a_new_parking_stop_restarts_cooldown():
    gate=SignMotionGate();gate.on_parking_stop(10.);gate.on_parking_stop(12.)
    assert not gate.allows(16.999)
    assert gate.allows(17.)


def test_failed_attempt_requires_stop_then_departure_without_another_five_seconds():
    gate=SignMotionGate();feed(gate,.2,10.,.2);gate.require_departure()
    feed(gate,.2,10.25,6.);assert not gate.allows(16.25)
    feed(gate,0.,16.3,.2);assert not gate.allows(16.5)
    feed(gate,.2,16.55,.15);assert gate.allows(16.7)


@pytest.mark.parametrize('delay',[-1.,float('nan'),float('inf')])
def test_invalid_delay_is_rejected(delay):
    with pytest.raises(ValueError):SignMotionGate(delay)
