import math

import pytest

from cone_lane_follower.path_memory import PathMemory


def test_memory_trims_passed_segments_and_stops_at_goal():
    memory = PathMemory()
    original = [(x, 0., 0.) for x in (.1, .3, .5, .7, .9)]
    assert memory.store(original)
    local = [(x-.4, y, z) for x, y, z in original]
    path, reason = memory.remaining(local, .08, .5)
    assert reason is None and path[0][0] == pytest.approx(0.)
    assert memory.index == 1
    assert path[-1][0] == pytest.approx(.5)
    path, reason = memory.remaining([(x-.85, y, z) for x, y, z in original], .08, .5)
    assert not path and reason == 'memory_goal_reached'
    assert memory.finished
    assert memory.remaining(local, .08, .5)[1] == 'memory_goal_reached'


def test_memory_does_not_jump_to_nearby_hairpin_tail():
    memory = PathMemory()
    points = [(0., 0., 0.), (.3, 0., 0.), (.6, 0., 0.), (.6, .1, 0.),
              (.3, .1, 0.), (0., .1, 0.)]
    memory.store(points)
    path, reason = memory.remaining([(x, y-.1, z) for x, y, z in points], .05, .4)
    # Being near the final point alone must not finish a long remaining hairpin.
    assert reason is None and path and memory.index == 0


def test_memory_keeps_compatible_tail_and_rejects_invalid_input():
    memory = PathMemory()
    memory.store([(.1, 0., 0.), (.4, 0., 0.), (.7, 0., 0.), (1., 0., 0.)])
    memory.store([(.2, 0., 0.), (.5, 0., 0.)])
    assert memory.points[-1] == (1., 0., 0.)
    assert not memory.store([(math.nan, 0., 0.), (.5, 0., 0.)])
    memory.store([(.1, 1., 0.), (.5, 1., 0.)])
    assert memory.points[-1] == (.5, 1., 0.)
    memory.clear()
    assert not memory.points and not memory.finished


def test_memory_stops_if_vehicle_leaves_path():
    memory = PathMemory()
    memory.store([(.1, 0., 0.), (.5, 0., 0.)])
    assert memory.remaining([(.1, 1., 0.), (.5, 1., 0.)], .08, .5)[1] == 'memory_path_too_far'
