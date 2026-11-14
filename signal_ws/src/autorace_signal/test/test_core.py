import pytest
from autorace_signal.core import SignalState


def send(state, green, count, start):
    for i in range(count):
        state.receive(green, start+i*.1)


def test_five_red_then_five_green_and_noise():
    s = SignalState(5, 10., .5)
    send(s, False, 4, 0.)
    assert not s.stopping
    s.receive(True, .4)
    send(s, False, 4, .5)
    assert not s.stopping
    s.receive(False, .9)
    assert s.stopping and s.started_at == .9
    send(s, True, 4, 1.)
    assert s.stopping
    s.receive(True, 1.4)
    assert not s.stopping


def test_gap_resets_counter_but_preserves_stop():
    s = SignalState(5, 10., .5)
    send(s, False, 4, 0.)
    s.receive(False, 1.)
    assert not s.stopping and s.count == 1
    send(s, False, 4, 1.1)
    assert s.stopping
    send(s, True, 4, 2.)
    s.receive(True, 3.)
    assert s.stopping and s.count == 1


@pytest.mark.parametrize('continuous_red', [False, True])
def test_exact_ten_second_timeout_and_fresh_five_to_restart(continuous_red):
    s = SignalState(5, 10., .5)
    send(s, False, 5, 0.)
    if continuous_red:
        send(s, False, 90, .5)
    last_red = 9.4 if continuous_red else .4
    deadline = last_red+10.
    s.tick(deadline-.001)
    assert s.stopping and s.started_at == pytest.approx(last_red)
    s.tick(deadline)
    assert not s.stopping and s.count == 0
    send(s, False, 4, deadline+1.)
    assert not s.stopping
    s.receive(False, deadline+1.4)
    assert s.stopping and s.started_at == pytest.approx(deadline+1.4)


def test_single_red_after_five_seconds_restarts_ten_second_wait():
    s = SignalState(5, 10., .5)
    send(s, False, 5, 0.)
    s.receive(False, 5.4)
    assert s.stopping and s.started_at == 5.4
    # The confirmation gap has elapsed, but one red still extends an active stop.
    assert s.count == 1
    s.tick(10.4)
    assert s.stopping
    s.tick(15.399)
    assert s.stopping
    s.tick(15.4)
    assert not s.stopping


def test_red_interrupts_green_confirmation_and_green_does_not_extend_wait():
    s = SignalState(5, 10., .5)
    send(s, False, 5, 0.)
    send(s, True, 4, 5.)
    assert s.stopping and s.started_at == .4
    s.receive(False, 5.4)
    assert s.stopping and s.started_at == 5.4
    send(s, True, 4, 5.5)
    assert s.stopping and s.started_at == 5.4
    s.receive(True, 5.9)
    assert not s.stopping and s.started_at is None


def test_red_just_before_timeout_extends_but_after_timeout_needs_five():
    s = SignalState(5, 10., .5)
    send(s, False, 5, 0.)
    s.receive(False, 10.399)
    assert s.stopping
    s.tick(10.4)
    assert s.stopping
    # Without a timer tick first, receive must still expire the previous stop.
    s.receive(False, 20.4)
    assert not s.stopping and s.count == 1


def test_initial_green_and_tunable_threshold():
    s = SignalState(2, 3., .2)
    send(s, True, 10, 0.)
    assert not s.stopping
    send(s, False, 2, 1.)
    assert s.stopping
    s.tick(4.100001)
    assert not s.stopping


def test_two_confirmations_and_five_seconds_since_last_red():
    s = SignalState(2, 5., .5)
    s.tick(100.)
    assert not s.stopping
    s.receive(False, 100.)
    assert not s.stopping
    s.receive(False, 100.1)
    assert s.stopping
    # Even after a confirmation gap, one red renews the active stop.
    s.receive(False, 103.)
    s.receive(True, 103.1)
    assert s.stopping and s.started_at == 103.
    s.tick(107.999)
    assert s.stopping
    s.tick(108.)
    assert not s.stopping
    s.receive(False, 109.)
    assert not s.stopping
    s.receive(False, 109.1)
    assert s.stopping
    s.receive(True, 109.2)
    assert s.stopping
    s.receive(True, 109.3)
    assert not s.stopping


def test_two_confirmations_require_same_colour_without_long_gap():
    s = SignalState(2, 5., .5)
    s.receive(False, 0.)
    s.receive(True, .1)
    s.receive(False, .2)
    assert not s.stopping
    s.receive(False, .8)
    assert not s.stopping
    s.receive(False, .9)
    assert s.stopping
    s.receive(True, 1.)
    s.receive(True, 1.6)
    assert s.stopping
    s.receive(False, 1.7)
    s.receive(True, 1.8)
    assert s.stopping
    s.receive(True, 1.9)
    assert not s.stopping


def test_stop_generation_changes_only_for_a_new_stop_including_timeout_restart():
    s = SignalState(1, .5, .5)
    assert s.stop_generation == 0
    s.receive(False, 0.)
    assert s.stopping and s.stop_generation == 1
    s.receive(False, .1)
    assert s.stop_generation == 1
    s.receive(False, .7)  # Expire and immediately enter a new stop with quorum=1.
    assert s.stopping and s.stop_generation == 2
    s.receive(True, .8)
    assert not s.stopping and s.stop_generation == 2
    s.receive(False, .9)
    assert s.stopping and s.stop_generation == 3
