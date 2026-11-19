import pytest

from autorace_slow.zigzag import ZigzagSlow


def feed(z, levels, t0=0., dt=.05):
    out = []
    for i, level in enumerate(levels):
        out.append(z.receive(level, t0 + i * dt))
    return out


def test_both_sides_twice_starts():
    z = ZigzagSlow()
    assert feed(z, [1, 1, 2]) == [None, None, None]
    assert z.receive(2, .15) == 'start'
    assert z.active


def test_one_side_never_starts():
    z = ZigzagSlow()
    feed(z, [1] * 40)
    assert not z.active


def test_interrupted_both_sides_restart_count():
    z = ZigzagSlow()
    assert feed(z, [2, 1, 2, 0, 2]) == [None] * 5
    assert not z.active


def test_message_gap_resets_count():
    z = ZigzagSlow(gap_sec=.5)
    z.receive(2, 0.)
    assert z.receive(2, 1.) is None  # stale first frame does not count
    assert z.receive(2, 1.05) == 'start'


def test_exit_after_no_zigzag():
    z = ZigzagSlow(exit_sec=2.)
    feed(z, [2, 2])
    z.receive(1, .5)               # one side still counts as inside the zone
    z.receive(0, .6)
    assert z.tick(2.4) is None
    assert z.tick(2.5) == 'exit'
    assert not z.active and not z.disabled
    feed(z, [2, 2], t0=10.)        # next pass may start again
    assert z.active


def test_exit_when_messages_stop():
    z = ZigzagSlow(exit_sec=2.)
    feed(z, [2, 2])
    assert z.tick(1.) is None
    assert z.tick(2.1) == 'exit'


def test_crosswalk_sign_disables_but_keeps_active_slow():
    z = ZigzagSlow()
    feed(z, [2, 2])
    assert z.disable(1.) is True
    assert z.disable(1.1) is False
    assert z.active
    assert z.tick(10.) is None           # no zigzag for 10 s: still slow until the crosswalk
    assert z.active
    assert z.stop(disable=True) is True  # /start_crosswalk
    feed(z, [2] * 10, t0=5.)
    assert not z.active


def test_crosswalk_sign_before_zigzag_blocks_start():
    z = ZigzagSlow()
    z.disable()
    assert feed(z, [2] * 10) == [None] * 10
    assert not z.active


def test_start_crosswalk_disables_for_good():
    z = ZigzagSlow()
    assert z.stop(disable=True) is False
    feed(z, [2] * 10)
    assert not z.active


def test_red_light_stop_keeps_armed():
    z = ZigzagSlow()
    feed(z, [2, 2])
    assert z.stop() is True
    assert not z.disabled
    feed(z, [2, 2], t0=3.)
    assert z.active


@pytest.mark.parametrize('args', [(0, 2., .5), (2, 0., .5), (2, 2., 0.), (2, 2., .5, 0.)])
def test_invalid_settings(args):
    with pytest.raises(ValueError):
        ZigzagSlow(*args)


def test_after_sign_timeout():
    z = ZigzagSlow(max_after_sign_sec=15.)
    feed(z, [2, 2])
    z.disable(1.)
    assert z.tick(15.9) is None
    assert z.tick(16.) == 'timeout'
    assert not z.active and z.disabled


def test_sign_while_armed_has_no_timer():
    z = ZigzagSlow()
    z.disable(0.)
    assert z.sign_time is None
