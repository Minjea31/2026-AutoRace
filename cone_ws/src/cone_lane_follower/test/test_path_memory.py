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
    before = memory.points[:]
    assert not memory.store([(.1, 1., 0.), (.5, 1., 0.)])
    assert memory.points == before
    memory.clear()
    assert not memory.points and not memory.finished


@pytest.mark.parametrize('angle,accepted', [(60., True), (90., True), (100., False)])
def test_memory_extension_allows_up_to_ninety_degree_join(angle, accepted):
    memory = PathMemory()
    original = [(.1, 0., 0.), (.5, 0., 0.), (1., 0., 0.)]
    memory.store(original)
    theta = math.radians(angle)
    end = (1.+.3*math.cos(theta), .3*math.sin(theta), 0.)
    assert memory.store([(.8, 0., 0.), (1., 0., 0.), end]) == accepted
    assert memory.points[-1] == (end if accepted else original[-1])


def test_memory_stops_if_vehicle_leaves_path():
    memory = PathMemory()
    memory.store([(.1, 0., 0.), (.5, 0., 0.)])
    assert memory.remaining([(.1, 1., 0.), (.5, 1., 0.)], .08, .5)[1] == 'memory_path_too_far'


def test_only_aligned_forward_extension_changes_goal():
    memory = PathMemory()
    original = [(.1, 0., 0.), (.5, 0., 0.), (1., 0., 0.)]
    memory.store(original)
    assert not memory.store([(.1, 0., 0.), (.9, 0., 0.)])
    assert memory.points == original
    assert memory.store([(.5, 0., 0.), (1., 0., 0.), (1.3, 0., 0.)])
    assert memory.points == original+[(1.3, 0., 0.)]
    assert not memory.store([(1.5, 0., 0.), (1.3, 0., 0.), (.9, 0., 0.)])
    assert memory.points[-1] == (1.3, 0., 0.)


def test_extension_around_curve_can_end_closer_to_vehicle():
    memory = PathMemory()
    original = [(0., 0., 0.), (.5, 0., 0.), (1., 0., 0.)]
    memory.store(original)
    extension = [(.5, 0., 0.), (1., 0., 0.), (1.2, .1, 0.), (1.3, .3, 0.),
                 (1.2, .5, 0.), (1., .6, 0.), (.8, .6, 0.), (.6, .6, 0.)]
    assert memory.store(extension)
    assert memory.points[-1] == (.6, .6, 0.)
    assert math.hypot(*memory.points[-1][:2]) < math.hypot(*original[-1][:2])
    assert memory.points[:3] == original


def test_finished_path_is_not_rearmed_by_same_or_shorter_observation():
    memory = PathMemory()
    original = [(.1, 0., 0.), (.5, 0., 0.)]
    memory.store(original)
    memory.remaining([(-.35, 0., 0.), (.05, 0., 0.)], .08, .5)
    assert memory.finished
    assert not memory.store(original)
    assert memory.finished
    assert memory.store([(.3, 0., 0.), (.5, 0., 0.), (.8, 0., 0.)])
    assert not memory.finished and memory.points[-1] == (.8, 0., 0.)
