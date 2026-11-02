"""Local parking plans and a bounded pure-pursuit Ackermann controller."""
from dataclasses import replace
import math
import time

from .core import CollisionChecker, Gap, cluster_points, find_gaps, plan_path, wrap
from .tracking import compose, inverse


def local_goal(goal, pose):
    x, y, yaw = compose(inverse(pose), (goal.x, goal.y, goal.yaw))
    return replace(goal, x=x, y=y, yaw=yaw)


def build_plan(scan, cfg, goal=None):
    """Return (final goal, path, final approach).

    If occluded, stop before the row and observe again. Never mark a staging
    endpoint as parking completion or extend the path into unobserved space.
    """
    goals = [goal] if goal is not None else find_gaps(
        cluster_points(scan.points, cfg.cluster_distance, cfg.cluster_min_points), cfg)
    for target in goals:
        path = plan_path(scan, target, cfg)
        if path:
            return target, path, True
        for offset in (0.28, 0.38, 0.48):
            staging = replace(target, x=target.x - offset * math.cos(target.yaw),
                              y=target.y - offset * math.sin(target.yaw))
            # No repeated approach to a staging pose already reached.
            if math.hypot(staging.x, staging.y) < 0.13:
                continue
            path = plan_path(scan, staging, cfg)
            if path:
                return target, path, False
    return None, [], False


class Follower:
    def __init__(self, cfg, speed=1.0, lookahead=0.18, hold_seconds=3.0, deceleration=1.0,
                 max_yaw_rate=1.0):
        self.cfg, self.speed, self.lookahead = cfg, speed, lookahead
        self.hold_seconds, self.deceleration = hold_seconds, deceleration
        self.max_yaw_rate = max_yaw_rate
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
        # rather than turning the vehicle by pi. Preserve all staging segments.
        self.parked, self.parked_since = True, now
        self.path = [pose] + list(reversed(self.entry_trace))
        self.index = self.arrival_count = 0
        x, y, yaw = self.entry_trace[0]
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
            return 0.0, 0.0, 'EXIT_BLOCKED' if self.exiting else 'PLANNING'
        transform = inverse(pose)
        known = [compose(transform, p) for p in self.entry_trace] if self.exiting else ()
        checker = CollisionChecker(scan, self.cfg, known)
        goal = local_goal(self.goal, pose)
        arrived = (math.hypot(goal.x, goal.y) <= 0.07 and abs(goal.yaw) <= 0.20
                   and checker.free((0.0, 0.0, 0.0)))
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
                return 0.0, 0.0, 'EXIT_BLOCKED'
            self.path = []
            return 0.0, 0.0, 'REOBSERVE'
        # Cover braking distance and scan/control latency at the configured speed.
        horizon = max(.35, self.measured_speed ** 2 / (2 * self.deceleration) + self.measured_speed * .2 + .1)
        for point in path:
            if not checker.free(point):
                if self.exiting:
                    return 0.0, 0.0, 'EXIT_BLOCKED'
                self.path = []
                return 0.0, 0.0, 'BLOCKED'
            if math.hypot(point[0], point[1]) > horizon:
                break
        aim = path[-1]
        lookahead = self.lookahead + min(.15, self.measured_speed * .2)
        for point in path:
            if math.hypot(point[0], point[1]) >= lookahead:
                aim = point
                break
        direction = -1 if self.exiting else 1
        if direction * aim[0] <= 0.005:
            if self.exiting:
                return 0.0, 0.0, 'EXIT_BLOCKED'
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
