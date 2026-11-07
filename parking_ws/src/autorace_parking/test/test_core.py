import math
from dataclasses import replace

import numpy as np
import pytest

from autorace_parking.core import Activation, Cluster, CollisionChecker, Gap, Scan, Settings, advance, cluster_points, find_gaps, plan_path, wrap
from autorace_parking.control import Follower, build_plan, local_goal
from autorace_parking.tracking import ScanTracker, compose, inverse


def test_three_true_frames_start_once_until_reset_false_and_timeout_reset_counts():
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
    assert trigger.generation == 1 and trigger.count == 0
    trigger.update(True, 2.4)
    trigger.update(True, 2.5)
    trigger.expire(3.1)
    assert trigger.count == 0 and trigger.active
    for now in (3.2, 3.3, 3.4, 3.5, 3.6, 3.7):
        trigger.update(True, now)
    assert trigger.generation == 1  # a continuous stream cannot restart a mission
    for now in (10., 10.1, 10.2):
        trigger.update(True, now)
    assert trigger.generation == 1 and trigger.count == 0
    trigger.reset()
    assert not trigger.active and trigger.count == 0
    trigger.update(True, 3.0)
    trigger.expire(4.0)
    assert trigger.count == 0
    for now in (4.1, 4.2, 4.3):
        trigger.update(True, now)
    assert trigger.active and trigger.generation == 1


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


def test_parking_candidates_ignore_closer_left_row():
    left = [rectangle(.4, .4), rectangle(.95, .4)]
    right = [rectangle(.8, -.8), rectangle(1.8, -.8)]
    assert not find_gaps(left, Settings())
    gaps = find_gaps(left + right, Settings())
    assert len(gaps) == 1
    assert (gaps[0].x, gaps[0].y, gaps[0].yaw) == pytest.approx((1.3, -.8, -math.pi / 2))


@pytest.mark.parametrize('origin', [(0., 0., 0.), (.4, -1.2, 0.),
                                        (.4, -.2, -math.pi / 2), (.4, -.7, math.pi - .01)])
def test_right_row_remains_relative_to_start_after_translation_and_turn(origin):
    groups = [rectangle(.8, -.6), rectangle(1.8, -.6),
              rectangle(.8, .4), rectangle(1.8, .4)]
    local = [Cluster([compose(inverse(origin), (x, y, 0.))[:2] for x, y in group.points])
             for group in groups]
    gaps = find_gaps(local, replace(Settings(), row_yaw=-origin[2]), origin)
    assert len(gaps) == 1
    goal = compose(origin, (gaps[0].x, gaps[0].y, gaps[0].yaw))
    assert goal == pytest.approx((1.3, -.6, -math.pi / 2))


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


def test_follower_keeps_path_with_obstacle_returns_and_requires_three_arrival_scans():
    cfg = Settings()
    scan = Scan([10.] * 361, -math.pi, math.pi / 180, .01, 12., (0., 0., 0.))
    goal = Gap(.8, 0., 0., .8, 0, 1)
    follower = Follower(cfg)
    follower.accept(goal, [(i / 100, 0., 0.) for i in range(81)], True, (0., 0., 0.))
    assert follower.command(scan, (0., 0., 0.))[0] > 0
    route = follower.path.copy()
    scan.points = [(.28, 0.), (0., 0.)]
    assert follower.command(scan, (0., 0., 0.))[0] > 0.
    assert follower.path == route
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


def entry_turn_trace(direction=-1):
    straight = [(i/100, 0., 0.) for i in range(101)]
    turn = [advance(straight[-1], i/100, direction*2.) for i in range(1, 81)]
    finish = [advance(turn[-1], i/100, 0.) for i in range(1, 11)]
    return straight+turn+finish


def test_reverse_departure_replays_entry_turn_despite_obstacle_returns():
    cfg = Settings()
    scan = Scan([10.]*361, -math.pi, math.pi/180, .01, 12., (0., 0., 0.))
    follower = Follower(cfg)
    trace = entry_turn_trace()
    parked_pose = trace[-1]
    goal = Gap(*parked_pose, .8, 0, 1)
    follower.accept(goal, trace, True, (0., 0., 0.))
    for i, pose in enumerate(trace):
        follower.observe_pose(pose, i*.01, True)
    now = len(trace)*.01
    for i in range(4):
        result = follower.command(scan, parked_pose, True, now=now+i*.1)
    assert result == (0., 0., 'PARKED')
    route = follower.path.copy()
    exit_goal = route[-1]
    assert exit_goal[0] == pytest.approx(1., abs=.03)
    assert abs(exit_goal[1]) < .01
    assert abs(exit_goal[2]) <= follower.exit_heading_tolerance
    follower.accept(goal, [(9., 9., 0.)], True, (0., 0., 0.))
    assert follower.path == route
    start_exit = follower.parked_since+follower.hold_seconds
    assert follower.command(scan, parked_pose, True, now=start_exit-.01) == (0., 0., 'PARKED')
    speed, steering, state = follower.command(scan, parked_pose, True, now=start_exit)
    assert -1. <= speed < 0. and abs(steering) <= cfg.max_steering and state == 'EXITING'
    scan.points = [(-.28, 0.)]
    assert follower.command(scan, parked_pose, True, now=start_exit+.1)[0] < 0.
    assert follower.path == route
    scan.points = []
    assert follower.command(scan, parked_pose, True, now=start_exit+.2)[0] < 0.
    for i in range(4):
        result = follower.command(scan, exit_goal, True, now=start_exit+1+i*.1)
    assert result == (0., 0., 'EXITED') and follower.exited and not follower.path
    assert follower.command(scan, exit_goal, now=20.) == (0., 0., 'EXITED')


def test_departure_corridor_covers_only_the_travelled_area_and_live_obstacles_win():
    cfg = Settings()
    unknown = Scan([0.] * 361, -math.pi, math.pi / 180, .01, 12., (0., 0., 0.))
    route = [(-i / 100, 0., 0.) for i in range(101)]
    assert not CollisionChecker(unknown, cfg).free((-.5, 0., 0.))
    assert CollisionChecker(unknown, cfg, route).free((-.5, 0., 0.))
    assert not CollisionChecker(unknown, cfg, route).free((-.5, .5, 0.))
    unknown.points = [(-.5, 0.)]
    assert not CollisionChecker(unknown, cfg, route).free((-.5, 0., 0.))


def test_occluded_bay_approach_starts_straight_then_turns_right():
    from limo_parking_sim.config import load_config
    from limo_parking_sim.model import Simulation
    sim = Simulation(load_config())
    cfg = Settings()
    scan = Scan(sim.scan(), sim.angles[0], sim.angles[1]-sim.angles[0], .01, 64.)
    goal, path, final = build_plan(scan, cfg)
    assert goal.y < 0 and path and not final
    first_turn = next(p for p in path if abs(p[2]) > .01)
    assert first_turn[0] > .4
    assert max(p[1] for p in path) <= 1e-9
    assert all(p[2] <= 1e-9 for p in path)
    checker = CollisionChecker(scan, cfg)
    assert all(checker.free(p) for p in path)
    assert not checker.free((goal.x, goal.y, goal.yaw))


@pytest.mark.parametrize('direction', [-1, 1])
@pytest.mark.parametrize('origin_yaw', [0., math.pi-.01])
def test_reverse_turn_ends_on_recorded_lane_pose_and_omits_initial_straight(direction, origin_yaw):
    cfg = Settings()
    route = [compose((0., 0., origin_yaw), p) for p in entry_turn_trace(direction)]
    follower = Follower(cfg)
    follower.goal = Gap(*route[-1], .8, 0, 1)
    follower.entry_trace = route.copy()
    follower.prepare_exit(route[-1], 0.)
    assert follower.path[-1] in route
    endpoint = compose(inverse((0., 0., origin_yaw)), follower.path[-1])
    assert endpoint[0] == pytest.approx(1., abs=.03)
    assert abs(endpoint[2]) <= follower.exit_heading_tolerance
    assert follower.exit_path_length > .8  # no former fixed 0.5 m cutoff
    transform = inverse(route[-1])
    known = [compose(transform, p) for p in route]
    unknown = Scan([0.]*361, -math.pi, math.pi/180, .01, 12., (0., 0., 0.))
    checker = CollisionChecker(unknown, cfg, known)
    assert all(checker.free(p) for p in follower.local_path(route[-1]))


def test_no_turn_does_not_reverse_the_straight_approach():
    follower = Follower(Settings())
    follower.entry_trace = [(i/100, 0., 0.) for i in range(101)]
    follower.goal = Gap(1., 0., 0., .8, 0, 1)
    follower.prepare_exit((1., 0., 0.), 0.)
    assert follower.path == [(1., 0., 0.)]
    assert follower.exit_path_length == 0.


def test_planner_accepted_terminal_pose_can_complete_and_reverse():
    cfg = Settings()
    scan = Scan([10.]*361, -math.pi, math.pi/180, .01, 12., (0., 0., 0.))
    trace = entry_turn_trace()
    endpoint = trace[-1]
    goal = Gap(endpoint[0]+.08, endpoint[1], wrap(endpoint[2]+.25), .8, 0, 1)
    follower = Follower(cfg)
    follower.accept(goal, trace, True, (0., 0., 0.))
    follower.entry_trace = trace.copy()
    for i in range(3):
        result = follower.command(scan, endpoint, True, now=i*.1)
    assert result == (0., 0., 'PARKED')
    assert follower.command(scan, endpoint, True, now=3.2)[0] < 0.


def test_partial_path_endpoint_is_not_parking_completion():
    cfg = Settings()
    scan = Scan([10.]*361, -math.pi, math.pi/180, .01, 12., (0., 0., 0.))
    follower = Follower(cfg)
    follower.accept(Gap(1., 0., 0., .8, 0, 1), [(0., 0., 0.), (1., 0., 0.)], False, (0., 0., 0.))
    for i in range(4):
        follower.command(scan, (1., 0., 0.), True, now=i*.1)
    assert not follower.parked and follower.parked_since is None


def test_obstacle_returns_during_confirmation_do_not_interrupt_reverse():
    cfg = Settings()
    scan = Scan([10.]*361, -math.pi, math.pi/180, .01, 12., (0., 0., 0.))
    route = entry_turn_trace()
    pose = route[-1]
    follower = Follower(cfg)
    follower.accept(Gap(*pose, .8, 0, 1), route, True, (0., 0., 0.))
    follower.entry_trace = route.copy()
    assert follower.command(scan, pose, True, now=0.)[2] == 'CONFIRMING'
    scan.points = [(0., 0.), (-.15, 0.), (.1, .1)]
    assert follower.command(scan, pose, True, now=.1)[2] == 'CONFIRMING'
    assert follower.command(scan, pose, True, now=.2)[2] == 'PARKED'
    reverse = follower.path.copy()
    assert follower.command(scan, pose, True, now=3.2)[0] < 0.
    assert follower.path == reverse


def test_simulator_straight_right_entry_and_turn_reverse_without_collision():
    from limo_parking_sim.config import load_config
    from limo_parking_sim.model import Simulation
    sim = Simulation(load_config())
    cfg = Settings()
    tracker, follower = ScanTracker(), Follower(cfg)
    start = sim.x, sim.y, sim.yaw
    parked_at = exiting_at = None
    previous_parked = False
    entry_left_excursion = 0.
    exit_goal = None
    xmin, xmax, ymin, ymax = sim.cfg['world']['parking']['goal']
    for tick in range(600):
        now, new_scan = tick / 20, tick % 2 == 0
        if new_scan:
            scan = Scan(sim.scan(), sim.angles[0], sim.angles[1] - sim.angles[0], .01, 64.)
            assert tracker.update(scan.points), f'tracking lost at tick {tick}'
        if not follower.parked and (not follower.path or tick % 30 == 0):
            goal = local_goal(follower.goal, tracker.pose) if follower.goal else None
            result = build_plan(scan, replace(cfg, row_yaw=-tracker.pose[2]), goal, tracker.pose)
            if result[1]:
                follower.accept(*result, tracker.pose)
        speed, steering, state = follower.command(scan, tracker.pose, new_scan=new_scan, now=now)
        assert abs(speed) <= 1.
        if follower.parked and not previous_parked:
            parked_at = now
            assert abs(sim.speed) < .03
            assert all(xmin <= x <= xmax and ymin <= y <= ymax for x, y in sim.body().corners())
            exit_goal = follower.path[-1]
            reverse_length = sum(math.hypot(b[0]-a[0], b[1]-a[1]) for a,b in zip(follower.path, follower.path[1:]))
            assert reverse_length > .5
            assert abs(exit_goal[2]) <= follower.exit_heading_tolerance
            assert exit_goal[0] > .4
        if not follower.parked:
            entry_left_excursion = max(entry_left_excursion, sim.y-start[1])
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
    assert math.hypot(sim.x-start[0]-exit_goal[0], sim.y-start[1]-exit_goal[1]) < .09
    assert math.hypot(sim.x-start[0], sim.y-start[1]) > .5
    assert abs(math.atan2(math.sin(sim.yaw-exit_goal[2]), math.cos(sim.yaw-exit_goal[2]))) < .20
    assert entry_left_excursion < .05
    assert abs(sim.speed) < .03


def test_precise_low_speed_arrival_does_not_brake_to_zero_six_cm_short():
    cfg = replace(Settings(), goal_tolerance=.035, goal_yaw_tolerance=.12)
    follower = Follower(cfg, speed=.15)
    scan = Scan([10.]*361, -math.pi, math.pi/180, .01, 12., (0.,0.,0.))
    follower.accept(Gap(.8,0.,0.,.8,0,1), [(i/100,0.,0.) for i in range(74,81)], True, (0.,0.,0.))
    speed, _, state = follower.command(scan, (.74,0.,0.), now=0.)
    assert speed > 0. and state == 'PARKING'
    for i in range(3):
        result = follower.command(scan, (.775,0.,0.), new_scan=True, now=.1*(i+1))
    assert result[2] == 'PARKED'


def test_planner_and_controller_use_the_same_physical_turning_radius():
    cfg = replace(Settings(), max_steering=.65, min_turn_radius=.5)
    assert cfg.turn_radius == pytest.approx(.5)
    assert cfg.steering_limit == pytest.approx(math.atan(.2/.5))
    scan = Scan([10.]*361, -math.pi, math.pi/180, .01,12.,(0.,0.,0.))
    goal = Gap(1.5,-.7,-math.pi/2,.8,0,1)
    path = plan_path(scan,goal,cfg)
    assert path
    for a,b in zip(path,path[1:]):
        distance=math.hypot(b[0]-a[0],b[1]-a[1])
        if distance>1e-6:
            assert abs(wrap(b[2]-a[2]))/distance <= 1/cfg.turn_radius+1e-5
    follower=Follower(cfg,max_steering_rate=.8)
    follower.accept(goal,path,True,(0.,0.,0.))
    previous=0.
    for i in range(10):
        _,steering,_=follower.command(scan,(0.,0.,0.),now=i*.05)
        assert abs(steering)<=cfg.steering_limit+1e-9
        assert abs(steering-previous)<=.8*.05+1e-9
        previous=steering


def test_position_only_arrival_with_discarded_path_starts_recorded_reverse():
    cfg=Settings()
    follower=Follower(cfg,speed=.5,require_parking_heading=False,parking_position_tolerance=.10)
    scan=Scan([10.]*361,-math.pi,math.pi/180,.01,12.,(0.,0.,0.))
    trace=entry_turn_trace()
    pose=trace[-1]
    # Same failure as the vehicle: ~7 cm from the accepted goal, with a path
    # already discarded by REPLAN. Heading is deliberately far from the goal.
    goal=Gap(pose[0]+.07,pose[1],pose[2]+.6,.8,0,1)
    follower.accept(goal,trace,True,(0.,0.,0.))
    follower.entry_trace=trace.copy()
    follower.path=[]
    for i in range(3):
        result=follower.command(scan,pose,True,now=i*.1)
    assert result[2]=='PARKED' and follower.parked
    assert len(follower.path)>10
    assert follower.command(scan,pose,True,now=3.19)[2]=='PARKED'
    speed,_,state=follower.command(scan,pose,True,now=3.21)
    assert speed<0 and state=='EXITING'


def test_position_only_arrival_still_requires_position_and_measured_stop():
    follower=Follower(Settings(),require_parking_heading=False,parking_position_tolerance=.10)
    scan=Scan([10.]*361,-math.pi,math.pi/180,.01,12.,(0.,0.,0.))
    follower.goal=Gap(.2,0.,1.,.8,0,1)
    follower.final_approach=True
    for i in range(4):
        assert follower.command(scan,(0.,0.,0.),True,now=i*.1)[2]=='PLANNING'
    assert not follower.parked
    follower.goal=Gap(0.,0.,1.,.8,0,1)
    follower.measured_speed=.1
    assert follower.command(scan,(0.,0.,0.),False,now=1.)[2]=='CONFIRMING'
    assert not follower.parked


def test_position_only_planner_does_not_force_an_unnecessary_right_angle():
    cfg=replace(Settings(),max_steering=.65,min_turn_radius=.5,require_goal_heading=False)
    scan=Scan([10.]*361,-math.pi,math.pi/180,.01,12.,(0.,0.,0.))
    goal=Gap(.7,-.35,-math.pi/2,.8,0,1)
    path=plan_path(scan,goal,cfg)
    assert path
    assert path[-1][:2]==pytest.approx((goal.x,goal.y),abs=cfg.goal_tolerance)
    assert abs(wrap(path[-1][2]-goal.yaw))>.1
    for a,b in zip(path,path[1:]):
        length=math.hypot(b[0]-a[0],b[1]-a[1])
        if length>1e-6:assert abs(wrap(b[2]-a[2]))/length<=2.+1e-5
