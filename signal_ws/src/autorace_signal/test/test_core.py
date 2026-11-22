import pytest
from autorace_signal.core import SignalState


@pytest.mark.parametrize('white_detected', [False, True])
def test_fixed_red_deadline_cancels_white_wait_and_pending_stop(white_detected):
    state = SignalState(2, 10., .5, require_white_mark=True, stop_delay_sec=20.)
    send(state, False, 2, 0.)
    assert state.armed
    if white_detected:
        state.receive_white(True, .2)
    state.tick(10.1)
    assert state.timed_out
    assert not state.armed and not state.stopping and state.stop_at is None
    assert state.stop_generation == 0
    state.tick(30.)
    assert not state.receive_white(True, 30.)
    assert not state.stopping


def send(state, green, count, start):
    for i in range(count):
        state.receive(green, start+i*.1)


def test_five_red_then_five_green_and_noise():
    s=SignalState(5,10.,.5)
    send(s,False,4,0.)
    assert not s.stopping
    s.receive(True,.4)
    send(s,False,4,.5)
    assert not s.stopping
    s.receive(False,.9)
    assert s.stopping and s.red_started_at==.9
    send(s,True,4,1.)
    assert s.stopping
    s.receive(True,1.4)
    assert not s.stopping


def test_gap_resets_counter_but_preserves_stop():
    s=SignalState(5,10.,.5)
    send(s,False,4,0.)
    s.receive(False,1.)
    assert not s.stopping and s.count==1
    send(s,False,4,1.1)
    assert s.stopping
    send(s,True,4,2.)
    s.receive(True,3.)
    assert s.stopping and s.count==1


@pytest.mark.parametrize('continuous_red',[False,True])
def test_fixed_ten_seconds_from_confirmed_red_and_green_rearms(continuous_red):
    s=SignalState(5,10.,.5)
    send(s,False,5,0.)
    if continuous_red:
        for i in range(1,100):s.receive(False,.4+i*.1)
    s.tick(10.399)
    assert s.stopping and s.started_at==.4 and s.red_started_at==.4
    s.tick(10.4)
    assert not s.stopping and s.timed_out
    for i in range(4):s.receive(False,10.41+i*.1)
    assert not s.stopping and s.stop_generation==1
    send(s,True,5,11.)
    assert not s.timed_out
    send(s,False,4,11.5)
    assert not s.stopping
    s.receive(False,11.9)
    assert s.stopping and s.stop_generation==2


def test_single_red_does_not_restart_fixed_wait():
    s=SignalState(5,10.,.5)
    send(s,False,5,0.)
    s.receive(False,5.4)
    assert s.stopping and s.started_at==.4 and s.red_started_at==.4
    s.tick(10.399)
    assert s.stopping
    s.tick(10.4)
    assert not s.stopping


def test_red_interrupts_green_confirmation_without_extending_deadline():
    s=SignalState(5,10.,.5)
    send(s,False,5,0.)
    send(s,True,4,5.)
    assert s.stopping
    s.receive(False,5.4)
    assert s.red_started_at==.4
    send(s,True,4,5.5)
    assert s.stopping
    s.receive(True,5.9)
    assert not s.stopping and s.red_started_at is None


def test_red_just_before_timeout_does_not_extend_and_gap_allows_new_mission():
    s=SignalState(5,10.,.5)
    send(s,False,5,0.)
    s.receive(False,10.399)
    s.tick(10.4)
    assert not s.stopping
    send(s,False,5,10.41)
    assert not s.stopping
    send(s,False,4,12.)
    assert not s.stopping
    s.receive(False,12.4)
    assert s.stopping and s.red_started_at==12.4


def test_initial_green_and_tunable_threshold():
    s=SignalState(2,3.,.2)
    send(s,True,10,0.)
    assert not s.stopping
    send(s,False,2,1.)
    assert s.stopping
    s.tick(4.100001)
    assert not s.stopping


def test_two_confirmations_and_fixed_timeout_from_red_confirmation():
    s=SignalState(2,10.,.5)
    s.receive(False,100.)
    assert not s.stopping
    s.receive(False,100.1)
    assert s.stopping
    s.receive(False,103.)
    s.receive(True,103.1)
    assert s.stopping and s.red_started_at==100.1
    s.tick(110.099)
    assert s.stopping
    s.tick(110.1)
    assert not s.stopping
    s.receive(False,111.)
    assert not s.stopping
    s.receive(False,111.1)
    assert s.stopping
    s.receive(True,111.2)
    s.receive(True,111.3)
    assert not s.stopping


def test_two_confirmations_require_same_colour_without_long_gap():
    s=SignalState(2,5.,.5)
    s.receive(False,0.)
    s.receive(True,.1)
    s.receive(False,.2)
    assert not s.stopping
    s.receive(False,.8)
    assert not s.stopping
    s.receive(False,.9)
    assert s.stopping
    s.receive(True,1.)
    s.receive(True,1.6)
    assert s.stopping
    s.receive(False,1.7)
    s.receive(True,1.8)
    assert s.stopping
    s.receive(True,1.9)
    assert not s.stopping


def test_timeout_cannot_create_new_stop_generation_from_same_continuous_red():
    s=SignalState(1,.5,.5)
    s.receive(False,0.)
    assert s.stop_generation==1
    s.receive(False,.1)
    s.receive(False,.5)
    assert not s.stopping and s.stop_generation==1
    s.receive(True,.6)
    s.receive(False,.7)
    assert s.stopping and s.stop_generation==2
    s.receive(True,.8)
    s.receive(False,.9)
    assert s.stopping and s.stop_generation==3
