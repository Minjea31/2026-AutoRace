import math
import pytest
from cone_lane_follower.node import RoiNode, CENTER_DEFAULTS
from cone_lane_follower.path_memory import PathMemory


def line(end, y):
    return [(i*.05, y, 0.) for i in range(round(end/.05)+1)]


@pytest.mark.parametrize('side', [-1, 1])
def test_straight_extension(side):
    left, right = line(2 if side == -1 else 1, .3), line(2 if side == 1 else 1, -.3)
    pairs = RoiNode.match_cross_sections(left, right, .05, .1, 1.5)
    measured = [tuple((a[k]+b[k])/2 for k in range(3)) for a, b in pairs]
    path = RoiNode.unified_path(left, right, pairs, measured, .6, CENTER_DEFAULTS)
    assert path[-1][0] > 1.9
    assert max(abs(q[1]) for q in path) < 1e-8


def test_curve_inward_and_crossing_rejection():
    # Left-turn arc continues beyond a quarter turn (decreasing X).
    left = [( .7*math.sin(t), 1.-.7*math.cos(t), 0.) for t in [i*.025 for i in range(81)]]
    path = RoiNode.unified_path(left, [], [], [], .6, CENTER_DEFAULTS)
    assert path and RoiNode.offset_safe(path, [(left, -1.)])
    assert max(abs(math.hypot(x, y-1)-1.) for x,y,z in path) < .02
    assert not RoiNode.offset_safe([(0., -.1, 0.), (1., .5, 0.)], [(line(1, .3), -1.)])


def test_latest_backup_prefix_and_compatible_tail():
    memory = PathMemory()
    valid = lambda path: RoiNode.valid_center_geometry(path, CENTER_DEFAULTS)
    memory.refresh(line(2, 0), valid)
    latest = line(1, .01)
    memory.refresh(latest, valid)
    assert memory.points[:len(latest)] == latest
    assert memory.points[-1][0] == 2
    latest = line(1, .5)
    memory.refresh(latest, valid)
    assert memory.points == latest


def generate(left, right):
    pairs = RoiNode.match_cross_sections(left, right, .05, .1, 1.5) if len(left) >= 3 and len(right) >= 3 else []
    centers = [tuple((a[k]+b[k])/2 for k in range(3)) for a, b in pairs]
    measured = RoiNode.smooth_center_path(centers, left, right, .25, .3, 75) if pairs else []
    return RoiNode.unified_path(left, right, pairs, measured, .6, CENTER_DEFAULTS)


def test_equal_boundaries_match_analytic_center():
    path = generate(line(2, .3), line(2, -.3))
    assert len(path) > 30
    assert all(abs(y) < 1e-9 for x,y,z in path)
    assert path[-1][0] > 1.9


@pytest.mark.parametrize('side', [-1, 1])
def test_single_boundary_matches_half_width(side):
    boundary = line(2, -.3*side)
    path = generate(boundary if side == -1 else [], boundary if side == 1 else [])
    assert path[-1][0] == pytest.approx(2.)
    assert all(abs(y) < 1e-9 for x,y,z in path)


def test_disjoint_boundaries_choose_nearest_without_bridging_gap():
    left = line(1, .3)
    right = [(x+3,y,z) for x,y,z in line(1,-.3)]
    assert not RoiNode.match_cross_sections(left, right, .05, .1, 1.5)
    path = generate(left, right)
    assert path and max(x for x,y,z in path) <= 1.
    assert all(abs(y) < 1e-9 for x,y,z in path)


def test_quintic_join_has_no_heading_or_curvature_spike():
    first = line(1, 0)
    second = [(x+1.05, .03, z) for x,y,z in line(1, 0)]
    bridge = RoiNode.quintic_bridge(first, len(first)-4, second, 3, .01)
    path = first[:-4]+bridge+second[4:]
    assert RoiNode.valid_center_geometry(path, CENTER_DEFAULTS)
    headings = [math.atan2(b[1]-a[1], b[0]-a[0]) for a,b in zip(path,path[1:])]
    assert max(abs(b-a) for a,b in zip(headings,headings[1:])) < .1
    assert bridge[0] == pytest.approx(first[-4])
    assert bridge[-1] == pytest.approx(second[3])


@pytest.mark.parametrize('long_side', [-1, 1])
def test_sharp_curve_extends_to_observed_end(long_side):
    def arc(radius, end):
        return [(radius*math.sin(t), 1.-radius*math.cos(t), 0.)
                for t in [i*.025 for i in range(round(end/.025)+1)]]
    left = arc(.7, 2. if long_side == -1 else 1.)
    right = arc(1.3, 2. if long_side == 1 else 1.)
    path = generate(left, right)
    assert path
    assert math.atan2(path[-1][0], 1-path[-1][1]) > 1.9
    assert max(abs(math.hypot(x,y-1)-1) for x,y,z in path) < .03
    assert RoiNode.offset_safe(path, [(left,-1.), (right,1.)])
    assert RoiNode.valid_center_geometry(path, CENTER_DEFAULTS)


def test_backup_rejects_reversed_or_crossing_tail():
    valid = lambda path: RoiNode.valid_center_geometry(path, CENTER_DEFAULTS)
    memory = PathMemory()
    old = [(1.-i*.05, 0., 0.) for i in range(21)]
    assert memory.refresh(old, valid)
    latest = line(.5, 0.)
    assert memory.refresh(latest, valid)
    assert memory.points == latest


def test_geometry_rejects_gap_sharp_turn_and_self_crossing():
    assert not RoiNode.valid_center_geometry([(0.,0.,0.), (.31,0.,0.)], CENTER_DEFAULTS)
    assert not RoiNode.valid_center_geometry([(0.,0.,0.), (.1,0.,0.), (.1,.1,0.)], CENTER_DEFAULTS)
    assert not RoiNode.valid_center_geometry([(0.,0.,0.), (.1,.1,0.), (0.,.1,0.), (.1,0.,0.)], CENTER_DEFAULTS)


def test_geometry_diagnostics_include_measurement_without_changing_rejection():
    diagnostics = {}
    path = [(0.,0.,0.), (.1,0.,0.), (.1,.1,0.)]
    assert not RoiNode.valid_center_geometry(path, CENTER_DEFAULTS, diagnostics)
    assert 'point=1' in diagnostics['turn']
    assert 'angle=90.0deg' in diagnostics['turn']
    assert 'limit=75.0deg' in diagnostics['turn']
    assert not RoiNode.valid_center_geometry(path, CENTER_DEFAULTS)


def test_single_boundary_corner_is_smoothed_with_vehicle_radius():
    from cone_lane_follower.corner_offset import offset, curvature
    boundary = [(0.,0.,0.), (1.,0.,0.), (1.,1.,0.), (2.,1.,0.)]
    diagnostics = {}
    path = offset(boundary, .3, .05, .2630871384, RoiNode, CENTER_DEFAULTS, diagnostics)
    assert path, diagnostics
    assert max(curvature(path)) <= 1/.2630871384
    assert RoiNode.offset_safe(path, [(boundary, 1.)])
    assert RoiNode.valid_center_geometry(path, CENTER_DEFAULTS)
    assert diagnostics['corner_smoothing'] == 'success'


def test_impossible_boundary_reversal_is_not_smoothed_into_route():
    from cone_lane_follower.corner_offset import offset
    diagnostics = {}
    assert not offset([(0.,0.,0.),(1.,0.,0.),(.1,.01,0.)], .3,.05,.263,
                      RoiNode,CENTER_DEFAULTS,diagnostics)
    assert 'reversal' in diagnostics['corner_failure']


def test_corner_offset_mirrors_and_rejects_impossible_radius():
    from cone_lane_follower.corner_offset import offset, curvature
    boundary = [(0.,0.,0.), (1.,0.,0.), (1.,1.,0.), (2.,1.,0.)]
    mirrored = [(x,-y,z) for x,y,z in boundary]
    left = offset(boundary,.3,.05,.263,RoiNode,CENTER_DEFAULTS,{})
    right = offset(mirrored,-.3,.05,.263,RoiNode,CENTER_DEFAULTS,{})
    assert left and right
    assert len(left) == len(right)
    for a,b in zip(left,right):
        assert a == pytest.approx((b[0],-b[1],b[2]))
    diagnostics = {}
    assert not offset(boundary,.3,.05,10.,RoiNode,CENTER_DEFAULTS,diagnostics)
    assert 'corner_failure' in diagnostics
