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
    s.tick(10.399)
    assert s.stopping and s.started_at == .4
    s.tick(10.4)
    assert not s.stopping and s.count == 0
    send(s, False, 4, 11.)
    assert not s.stopping
    s.receive(False, 11.4)
    assert s.stopping and s.started_at == 11.4


def test_initial_green_and_tunable_threshold():
    s = SignalState(2, 3., .2)
    send(s, True, 10, 0.)
    assert not s.stopping
    send(s, False, 2, 1.)
    assert s.stopping
    s.tick(4.100001)
    assert not s.stopping
