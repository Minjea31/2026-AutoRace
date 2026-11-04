"""Local parking plans and a bounded pure-pursuit Ackermann controller."""
from dataclasses import replace
import math
import time

from .core import (CollisionChecker, Gap, cluster_points, find_gaps, plan_path,
                   sample_candidate, straight_turn_candidates, wrap)
from .tracking import compose, inverse


def local_goal(goal, pose):
    x, y, yaw = compose(inverse(pose), (goal.x, goal.y, goal.yaw))
    return replace(goal, x=x, y=y, yaw=yaw)


def build_plan(scan, cfg, goal=None, origin=(0., 0., 0.)):
    """Return (final goal, path, final approach).

    Prefer driving straight along the row before turning into the bay. If
    occluded, advance only through the observed prefix and then observe again.
    New bays must lie to the right of the mission-start pose, not necessarily
    to the right of the current vehicle after it has turned.
    """
    goals = [goal] if goal is not None else find_gaps(
        cluster_points(scan.points, cfg.cluster_distance, cfg.cluster_min_points), cfg, origin)
    checker = CollisionChecker(scan, cfg)
    for target in goals:
        partial, progress = [], 0.
        for _, parts in straight_turn_candidates(target, cfg):
            path, complete = sample_candidate(checker, parts, cfg)
            if complete:
                return target, path, True
            length = sum(math.hypot(b[0]-a[0], b[1]-a[1]) for a, b in zip(path, path[1:]))
            if length > max(.15, progress) and math.hypot(*path[-1][:2]) > .13:
                partial, progress = path, length
        if partial:
            return target, partial, False
        path = plan_path(scan, target, cfg)
        if path:
            return target, path, True
    return None, [], False


class Follower:
    def __init__(self, cfg, speed=1.0, lookahead=0.18, hold_seconds=3.0, deceleration=1.0,
                 max_yaw_rate=1.0, exit_heading_tolerance=0.05):
        self.cfg, self.speed, self.lookahead = cfg, speed, lookahead
        self.hold_seconds, self.deceleration = hold_seconds, deceleration
        self.max_yaw_rate = max_yaw_rate
        if not math.isfinite(exit_heading_tolerance) or not 0 < exit_heading_tolerance < math.pi/2:
            raise ValueError('exit_heading_tolerance must be between zero and pi/2')
        self.exit_heading_tolerance = exit_heading_tolerance
        self.exit_path_length = 0.
        self.goal = None
        self.path = []
        self.index = 0
        self.final_approach = False
        self.arrival_count = 0
        self.parked = False
        self.parked_since = None
        self.exiting = self.exited = False
        self.entry_trace = []
        self.previous_sample = None
        self.measured_speed = 0.0

    def accept(self, goal, path, final_approach, origin):
        if not path or self.parked:
            return
        gx, gy, yaw = compose(origin, (goal.x, goal.y, goal.yaw))
        self.goal = replace(goal, x=gx, y=gy, yaw=yaw)
        self.path = [compose(origin, point) for point in path]
        self.final_approach, self.index = final_approach, 0

    def local_path(self, pose):
        transform = inverse(pose)
        return [compose(transform, p) for p in self.path[self.index:]]

    def observe_pose(self, pose, now, new_scan):
        if new_scan:
            if self.previous_sample is not None:
                previous, stamp = self.previous_sample
                if now - stamp > .001:
                    self.measured_speed = math.hypot(pose[0] - previous[0], pose[1] - previous[1]) / (now - stamp)
            self.previous_sample = pose, now
        if self.parked:
            return
        if not self.entry_trace:
            self.entry_trace.append(pose)
            return
        a = self.entry_trace[-1]
        distance = math.hypot(pose[0] - a[0], pose[1] - a[1])
        if distance < .005:
            return
        count = max(1, math.ceil(distance / self.cfg.collision_step))
        for i in range(1, count + 1):
            t = i / count
            self.entry_trace.append((a[0] + t * (pose[0] - a[0]), a[1] + t * (pose[1] - a[1]),
                                     wrap(a[2] + t * wrap(pose[2] - a[2]))))

    def prepare_exit(self, pose, now):
        # Keep vehicle headings unchanged: reverse waypoint order and velocity,
        # rather than turning the vehicle by pi. Stop at the last recorded
        # lane-aligned pose: reverse the entry turn, not the earlier straight.
        self.parked, self.parked_since = True, now
        self.path = [pose]
        lane_yaw = self.entry_trace[0][2] if self.entry_trace else pose[2]
        if abs(wrap(pose[2]-lane_yaw)) > self.exit_heading_tolerance:
            for point in reversed(self.entry_trace):
                previous = self.path[-1]
                if math.hypot(point[0]-previous[0], point[1]-previous[1]) <= 1e-9:
                    continue
                # Keep an actual recorded footprint, including at the endpoint.
                self.path.append(point)
                if abs(wrap(point[2]-lane_yaw)) <= self.exit_heading_tolerance:
                    break
        self.exit_path_length = sum(math.hypot(b[0]-a[0], b[1]-a[1])
                                    for a,b in zip(self.path, self.path[1:]))
        self.index = self.arrival_count = 0
        x, y, yaw = self.path[-1]
        self.goal = replace(self.goal, x=x, y=y, yaw=yaw)

    def command(self, scan, pose, new_scan=False, now=None):
        now = time.monotonic() if now is None else now
        self.observe_pose(pose, now, new_scan)
        if self.exited:
            return 0.0, 0.0, 'EXITED'
        if self.parked and not self.exiting:
            if now - self.parked_since < self.hold_seconds:
                return 0.0, 0.0, 'PARKED'
            self.exiting = True
        if not self.path or self.goal is None:
            return 0.0, 0.0, 'EXIT_PATH_ERROR' if self.exiting else 'PLANNING'
        goal = local_goal(self.goal, pose)
        # The fallback planner may legally end within its configured goal
        # tolerances. A stricter controller used to reject that endpoint and
        # discard the path, leaving a forward-only replan with no solution.
        position_tolerance = .07 if self.exiting else max(.07, self.cfg.goal_tolerance)
        yaw_tolerance = .20 if self.exiting else max(.20, self.cfg.goal_yaw_tolerance)
        arrived = ((self.exiting or self.final_approach)
                   and math.hypot(goal.x, goal.y) <= position_tolerance and abs(goal.yaw) <= yaw_tolerance)
        if new_scan:
            self.arrival_count = self.arrival_count + 1 if arrived and self.measured_speed < .03 else 0
        if arrived:
            if self.arrival_count >= 3:
                if self.exiting:
                    self.exited = True
                    self.path = []
                    return 0.0, 0.0, 'EXITED'
                self.prepare_exit(pose, now)
                return 0.0, 0.0, 'PARKED'
            return 0.0, 0.0, 'EXIT_CONFIRMING' if self.exiting else 'CONFIRMING'
        # Monotonic closest waypoint avoids jumping to another loop/segment.
        upper = min(len(self.path), self.index + 30)
        self.index = min(range(self.index, upper),
                         key=lambda i: math.hypot(self.path[i][0] - pose[0], self.path[i][1] - pose[1]))
        path = self.local_path(pose)
        if not path or math.hypot(path[-1][0], path[-1][1]) < 0.065:
            if self.exiting:
                return 0.0, 0.0, 'EXIT_PATH_ERROR'
            self.path = []
            return 0.0, 0.0, 'REOBSERVE'
        # Follow the accepted path. Obstacle returns do not stop motion or
        # discard this route; the operator decides when to reset a failed run.
        aim = path[-1]
        lookahead = self.lookahead + min(.15, self.measured_speed * .2)
        for point in path:
            if math.hypot(point[0], point[1]) >= lookahead:
                aim = point
                break
        direction = -1 if self.exiting else 1
        if direction * aim[0] <= 0.005:
            if self.exiting:
                return 0.0, 0.0, 'EXIT_PATH_ERROR'
            self.path = []
            return 0.0, 0.0, 'REPLAN'
        distance2 = aim[0] ** 2 + aim[1] ** 2
        steering = math.atan(self.cfg.wheelbase * 2 * aim[1] / max(distance2, 1e-6))
        steering = max(-self.cfg.max_steering, min(self.cfg.max_steering, steering))
        remaining = math.hypot(path[0][0], path[0][1]) + sum(
            math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(path, path[1:]))
        # Reserve a scan/control interval for actuator response before braking.
        braking = math.sqrt((self.deceleration * .2) ** 2 +
                            2 * self.deceleration * max(0., remaining - .06)) - self.deceleration * .2
        speed = min(self.speed, braking, self.max_yaw_rate * self.cfg.wheelbase /
                    max(abs(math.tan(steering)), 1e-6))
        distance = 0.0
        for a, b in zip(path, path[1:]):
            segment = math.hypot(b[0] - a[0], b[1] - a[1])
            if segment < 1e-6:
                continue
            curvature = abs(wrap(b[2] - a[2])) / segment
            turn_speed = self.max_yaw_rate / max(curvature, 1e-6)
            speed = min(speed, math.sqrt(turn_speed ** 2 + 2 * self.deceleration *
                                        max(0., distance - self.measured_speed * .2)))
            distance += segment
        state = 'EXITING' if self.exiting else ('PARKING' if self.final_approach else 'APPROACHING')
        return direction * speed, steering, state
