import math
from dataclasses import replace

import numpy as np
import pytest

from autorace_parking.core import Activation, Cluster, CollisionChecker, Gap, Scan, Settings, advance, cluster_points, find_gaps, plan_path
from autorace_parking.control import Follower, build_plan, local_goal
from autorace_parking.tracking import ScanTracker, compose, inverse


def test_three_true_frames_retrigger_in_groups_false_and_timeout_reset_counts():
    trigger = Activation()
    for value, now in [(True, 0), (True, .1), (False, .2), (True, .3), (True, .4)]:
        assert not trigger.update(value, now)
    assert not trigger.update(True, 1.1)
    assert not trigger.update(True, 1.2)
    assert trigger.update(True, 1.3)
    assert trigger.generation == 1 and trigger.count == 0
    assert trigger.update(False, 2.0)  # sign leaving the image cannot cancel parking
    for value, now in [(True, 2.1), (True, 2.2), (True, 2.3)]:
        trigger.update(value, now)
    assert trigger.generation == 2 and trigger.count == 0
    trigger.update(True, 2.4)
    trigger.update(True, 2.5)
    trigger.expire(3.1)
    assert trigger.count == 0 and trigger.active
    for now in (3.2, 3.3, 3.4, 3.5, 3.6, 3.7):
        trigger.update(True, now)
    assert trigger.generation == 4  # even a continuous stream is grouped in threes
    trigger.reset()
    assert not trigger.active and trigger.count == 0
    trigger.update(True, 3.0)
    trigger.expire(4.0)
    assert trigger.count == 0


def test_scan_filters_invalid_and_applies_extrinsics():
    scan = Scan([0, math.nan, -1, math.inf, .5, 5], -.4, .1, .01, 64, (1., 2., .3), 3.)
    assert len(scan.points) == 1
    assert scan.points[0] == pytest.approx((1 + .5 * math.cos(.3), 2 + .5 * math.sin(.3)))
    assert not scan.visible(-1, -1)


def test_euclidean_clusters_and_noise_remains_collision_obstacle():
    points = [(1., -.1), (1.05, -.1), (1.1, -.1), (2., -.1), (2.05, -.1), (2.1, -.1), (.12, 0.)]
    groups = cluster_points(points)
    assert [len(c.points) for c in groups] == [3, 3]
    scan = Scan([10.] * 361, -math.pi, math.pi / 180, .01, 12., (0., 0., 0.))
    scan.points = points
    assert not CollisionChecker(scan, Settings()).free((0., 0., 0.))


def rectangle(x, y, width=.22, length=.32):
    return Cluster([(x - width / 2, y - length / 2), (x + width / 2, y - length / 2),
                    (x + width / 2, y + length / 2), (x - width / 2, y + length / 2)])


def test_gap_width_neighbour_filter_and_walls():
    cfg = Settings()
    assert find_gaps([rectangle(.8, -.6), rectangle(1.8, -.6)], cfg)[0].x == pytest.approx(1.3)
    assert not find_gaps([rectangle(.8, -.6), rectangle(1.3, -.6)], cfg)  # 28cm < body+margin
    groups = [rectangle(.8, -.6), rectangle(1.3, -.6), rectangle(1.8, -.6)]
    assert not find_gaps(groups, cfg)  # cannot plan through the middle vehicle
    assert not find_gaps([rectangle(1., 1., width=3.), rectangle(1., -.6)], cfg)


def test_arc_path_footprint_curvature_and_unknown_space():
    cfg = Settings()
    scan = Scan([10.] * 361, -math.pi, math.pi / 180, .01, 12., (0., 0., 0.))
    goal = Gap(1.3, -.5, -math.pi / 2, .8, 0, 1)
    path = plan_path(scan, goal, cfg)
    assert len(path) > 20
    assert path[-1] == pytest.approx((goal.x, goal.y, goal.yaw), abs=.001)
    checker = CollisionChecker(scan, cfg)
    for a, b in zip(path, path[1:]):
        assert checker.free(b)
        distance = math.hypot(b[0] - a[0], b[1] - a[1])
        if distance > 1e-6:
            assert abs(math.atan2(math.sin(b[2]-a[2]), math.cos(b[2]-a[2]))) / distance <= math.tan(cfg.max_steering) / cfg.wheelbase + .001
    unknown = Scan([0.] * 361, -math.pi, math.pi / 180, .01, 12., (0., 0., 0.))
    assert plan_path(unknown, goal, cfg) == []


def test_follower_stops_for_new_obstacle_and_requires_three_arrival_scans():
    cfg = Settings()
    scan = Scan([10.] * 361, -math.pi, math.pi / 180, .01, 12., (0., 0., 0.))
    goal = Gap(.8, 0., 0., .8, 0, 1)
    follower = Follower(cfg)
    follower.accept(goal, [(i / 100, 0., 0.) for i in range(81)], True, (0., 0., 0.))
    assert follower.command(scan, (0., 0., 0.))[0] > 0
    scan.points = [(.28, 0.)]  # single return still blocks the swept body
    assert follower.command(scan, (0., 0., 0.))[:2] == (0., 0.)
    assert not follower.path
    scan.points = []
    follower.accept(goal, [(.8, 0., 0.)], True, (0., 0., 0.))
    for _ in range(2):
        assert follower.command(scan, (.8, 0., 0.), True)[2] == 'CONFIRMING'
    assert follower.command(scan, (.8, 0., 0.), True)[2] == 'PARKED'
    assert follower.command(scan, (.8, 0., 0.))[:2] == (0., 0.)


def test_scan_registration_and_degenerate_failure():
    points = [(x, 1.) for x in np.linspace(-.5, 2., 80)] + [(1., y) for y in np.linspace(-1., .5, 50)]
    tracker = ScanTracker()
    assert tracker.update(points)
    motion = (.015, -.005, .02)
    observed = [compose(inverse(motion), (x, y, 0.))[:2] for x, y in points]
    assert tracker.update(observed)
    assert tracker.pose == pytest.approx(motion, abs=.003)
    wall = ScanTracker()
    assert wall.update(points[:80])
    assert not wall.update(points[:80])
    assert not wall.valid


@pytest.mark.parametrize('direction', [1, -1])
def test_scan_registration_scales_prediction_and_limits_for_missed_frames(direction):
    points = [(x, 1.) for x in np.linspace(-1., 3., 100)] + [(2., y) for y in np.linspace(-2., .8, 80)]
    tracker = ScanTracker()
    assert tracker.update(points, stamp=0.)
    pose, stamp = (0., 0., 0.), 0.
    # The 300 ms interval covers three scan periods at 1 m/s; this used to
    # exceed the fixed 12 cm / 0.25 rad registration limits and latch failure.
    intervals = [(.1, i / 10) for i in range(1, 11)] + [(.3, 1.), (.1, 1.), (.2, 1.), (.1, 1.)]
    for dt, speed in intervals:
        pose = compose(pose, advance((0., 0., 0.), direction * speed * dt, .9))
        stamp += dt
        observed = [compose(inverse(pose), (x, y, 0.))[:2] for x, y in points]
        assert tracker.update(observed, stamp=stamp), tracker.reason
        assert tracker.pose == pytest.approx(pose, abs=.004)
    assert not tracker.update(observed, stamp=stamp + 1.)
    assert 'interval' in tracker.reason


def test_reverse_departure_retraces_all_stages_after_three_seconds():
    cfg = Settings()
    scan = Scan([10.] * 361, -math.pi, math.pi / 180, .01, 12., (0., 0., 0.))
    follower = Follower(cfg)
    goal = Gap(1., 0., 0., .8, 0, 1)
    follower.accept(goal, [(i / 100, 0., 0.) for i in range(101)], False, (0., 0., 0.))
    assert follower.command(scan, (0., 0., 0.), True, now=0.)[0] == 1.
    follower.command(scan, (.4, 0., 0.), True, now=1.)
    follower.accept(goal, [(i / 100, 0., 0.) for i in range(40, 101)], True, (0., 0., 0.))
    follower.command(scan, (1., 0., 0.), True, now=2.)
    for now in (2.1, 2.2, 2.3):
        result = follower.command(scan, (1., 0., 0.), True, now=now)
    assert result == (0., 0., 'PARKED')
    route = follower.path.copy()
    assert route[-1] == (0., 0., 0.)  # includes movement before the latest plan
    assert all(a[0] >= b[0] and a[2] == b[2] == 0. for a, b in zip(route, route[1:]))
    assert follower.local_path((1., 0., 0.))[-1][0] == -1.
    follower.accept(goal, [(9., 9., 0.)], True, (0., 0., 0.))  # a late planner result
    assert follower.path == route
    assert follower.command(scan, (1., 0., 0.), True, now=5.29) == (0., 0., 'PARKED')
    speed, steering, state = follower.command(scan, (1., 0., 0.), True, now=5.3)
    assert speed == -1. and steering == 0. and state == 'EXITING'
    scan.points = [(-.28, 0.)]
    assert follower.command(scan, (1., 0., 0.), True, now=5.4) == (0., 0., 'EXIT_BLOCKED')
    assert follower.path == route
    scan.points = []
    assert follower.command(scan, (1., 0., 0.), True, now=5.5)[0] < 0.
    follower.command(scan, (0., 0., 0.), True, now=7.)
    for now in (7.1, 7.2, 7.3):
        result = follower.command(scan, (0., 0., 0.), True, now=now)
    assert result == (0., 0., 'EXITED') and follower.exited and not follower.path
    assert follower.command(scan, (0., 0., 0.), now=20.) == (0., 0., 'EXITED')


def test_departure_corridor_covers_only_the_travelled_area_and_live_obstacles_win():
    cfg = Settings()
    unknown = Scan([0.] * 361, -math.pi, math.pi / 180, .01, 12., (0., 0., 0.))
    route = [(-i / 100, 0., 0.) for i in range(101)]
    assert not CollisionChecker(unknown, cfg).free((-.5, 0., 0.))
    assert CollisionChecker(unknown, cfg, route).free((-.5, 0., 0.))
    assert not CollisionChecker(unknown, cfg, route).free((-.5, .5, 0.))
    unknown.points = [(-.5, 0.)]
    assert not CollisionChecker(unknown, cfg, route).free((-.5, 0., 0.))


def test_simulator_parks_holds_and_reverses_to_start_without_collision():
    from limo_parking_sim.config import load_config
    from limo_parking_sim.model import Simulation
    sim = Simulation(load_config())
    cfg = Settings()
    tracker, follower = ScanTracker(), Follower(cfg)
    start = sim.x, sim.y, sim.yaw
    parked_at = exiting_at = None
    previous_parked = False
    xmin, xmax, ymin, ymax = sim.cfg['world']['parking']['goal']
    for tick in range(600):
        now, new_scan = tick / 20, tick % 2 == 0
        if new_scan:
            scan = Scan(sim.scan(), sim.angles[0], sim.angles[1] - sim.angles[0], .01, 64.)
            assert tracker.update(scan.points), f'tracking lost at tick {tick}'
        if not follower.parked and (not follower.path or tick % 30 == 0):
            goal = local_goal(follower.goal, tracker.pose) if follower.goal else None
            result = build_plan(scan, replace(cfg, row_yaw=-tracker.pose[2]), goal)
            if result[1]:
                follower.accept(*result, tracker.pose)
        speed, steering, state = follower.command(scan, tracker.pose, new_scan=new_scan, now=now)
        assert abs(speed) <= 1.
        if follower.parked and not previous_parked:
            parked_at = now
            assert abs(sim.speed) < .03
            assert all(xmin <= x <= xmax and ymin <= y <= ymax for x, y in sim.body().corners())
        previous_parked = follower.parked
        if state == 'PARKED':
            assert speed == 0. and abs(sim.speed) < .03
        if speed < 0. and exiting_at is None:
            exiting_at = now
            assert exiting_at - parked_at >= 3. - 1e-9
            assert follower.local_path(tracker.pose)[-1][0] != 0.
        if follower.exited:
            break
        for _ in range(6):
            sim.command(speed, speed * math.tan(steering) / cfg.wheelbase)
            sim.step(1 / 120)
            assert not sim.collision, sim.collision_names
    assert follower.exited and exiting_at is not None
    assert math.hypot(sim.x - start[0], sim.y - start[1]) < .09
    assert abs(sim.yaw - start[2]) < .20
    assert abs(sim.speed) < .03
