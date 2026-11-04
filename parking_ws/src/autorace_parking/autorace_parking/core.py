"""ROS-independent geometry. Coordinates are metres in the current base frame.

This is a local path preview, not a controller or a persistent obstacle tracker.
All valid scan returns participate in collision checks, even small/noisy groups.
"""
from dataclasses import dataclass
import heapq
import math
import time
from collections import defaultdict


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass
class Settings:
    cluster_distance: float = 0.12
    cluster_min_points: int = 3
    max_range: float = 3.0
    max_cluster_size: float = 0.70
    min_cluster_size: float = 0.08
    row_yaw: float = 0.0
    row_tolerance: float = 0.16
    max_gap: float = 1.10
    vehicle_length: float = 0.322
    vehicle_width: float = 0.220
    wheelbase: float = 0.200
    max_steering: float = 0.408
    clearance: float = 0.04
    initial_clear_radius: float = 0.30
    resolution: float = 0.06
    yaw_resolution: float = math.pi / 18
    step: float = 0.12
    collision_step: float = 0.02
    goal_tolerance: float = 0.09
    goal_yaw_tolerance: float = math.pi / 9
    max_expansions: int = 10000
    allow_reverse: bool = False
    planning_timeout: float = 1.0


class Activation:
    def __init__(self, required=3, timeout=0.5):
        self.required, self.timeout = required, timeout
        self.reset()

    def reset(self):
        self.active, self.count, self.last = False, 0, None
        self.generation = 0

    def update(self, detected, now):
        if self.last is None or not 0 <= now - self.last <= self.timeout:
            self.count = 0
        self.last = now
        self.count = self.count + 1 if detected else 0
        if self.count >= self.required:
            self.active = True
            self.count = 0
            self.generation += 1
        return self.active

    def expire(self, now):
        if self.last is not None and not 0 <= now - self.last <= self.timeout:
            self.count = 0


@dataclass
class Cluster:
    points: list

    @property
    def bounds(self):
        xs, ys = zip(*self.points)
        return min(xs), max(xs), min(ys), max(ys)

    @property
    def center(self):
        a, b, c, d = self.bounds
        return (a + b) / 2, (c + d) / 2


def cluster_points(points, distance=0.12, min_points=3):
    """Euclidean connected components with a spatial hash (not scan index IDs)."""
    cells = defaultdict(list)
    for i, (x, y) in enumerate(points):
        cells[math.floor(x / distance), math.floor(y / distance)].append(i)
    remaining = set(range(len(points)))
    clusters = []
    while remaining:
        root = min(remaining)
        remaining.remove(root)
        queue, component = [root], []
        while queue:
            i = queue.pop()
            x, y = points[i]
            component.append(points[i])
            cx, cy = math.floor(x / distance), math.floor(y / distance)
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    for j in cells[cx + dx, cy + dy]:
                        if j in remaining and math.hypot(points[j][0] - x, points[j][1] - y) <= distance:
                            remaining.remove(j)
                            queue.append(j)
        if len(component) >= min_points:
            clusters.append(Cluster(component))
    return sorted(clusters, key=lambda c: c.center)


class Scan:
    def __init__(self, ranges, angle_min, angle_increment, range_min, range_max,
                 sensor_pose=(0.103, 0.0, 0.0), max_range=3.0):
        self.ranges = list(ranges)
        self.angle_min, self.angle_increment = angle_min, angle_increment
        self.range_min, self.range_max = range_min, range_max
        self.sensor_pose, self.max_range = sensor_pose, max_range
        self.points = []
        sx, sy, yaw = sensor_pose
        for i, r in enumerate(ranges):
            if math.isfinite(r) and max(0.001, range_min) <= r <= min(range_max, max_range):
                angle = angle_min + i * angle_increment + yaw
                self.points.append((sx + r * math.cos(angle), sy + r * math.sin(angle)))

    def visible(self, x, y):
        """Only accept measured free rays; zero/NaN/missing sectors are unknown.

        Check both bracketing beams to avoid declaring space behind an occluding
        obstacle free. Positive infinity is a clear ray up to range_max.
        """
        sx, sy, yaw = self.sensor_pose
        dx, dy = x - sx, y - sy
        angle = wrap(math.atan2(dy, dx) - yaw)
        if not self.ranges or not math.isfinite(self.angle_increment) or self.angle_increment == 0:
            return False
        index = (angle - self.angle_min) / self.angle_increment
        if index < 0 or index > len(self.ranges) - 1:
            return False
        distance = math.hypot(dx, dy)
        for i in {math.floor(index), math.ceil(index)}:
            r = self.ranges[i]
            if math.isnan(r) or r < max(0.001, self.range_min):
                return False
            if distance + 0.015 >= min(r, self.range_max, self.max_range):
                return False
        return True


@dataclass
class Gap:
    x: float
    y: float
    yaw: float
    width: float
    left: int
    right: int


def find_gaps(clusters, cfg, origin=(0., 0., 0.)):
    """Adjacent car-sized groups on the mission's initial right side.

    A gap is a local LiDAR candidate, not recognition of a painted parking bay.
    Long walls remain collision obstacles but cannot form parking candidates.
    origin is the current base pose in the mission-start frame. Keep the side
    fixed in that frame even after translating or turning into the parking row.
    """
    c, s = math.cos(cfg.row_yaw), math.sin(cfg.row_yaw)
    origin_c, origin_s = math.cos(origin[2]), math.sin(origin[2])
    projected = []
    for i, cluster in enumerate(clusters):
        x, y = cluster.center
        if origin[1] + origin_s * x + origin_c * y >= 0:
            continue
        pts = [(c * x + s * y, -s * x + c * y) for x, y in cluster.points]
        xs, ys = zip(*pts)
        bounds = min(xs), max(xs), min(ys), max(ys)
        size = max(bounds[1] - bounds[0], bounds[3] - bounds[2])
        if cfg.min_cluster_size <= size <= cfg.max_cluster_size:
            projected.append((i, bounds))
    projected.sort(key=lambda item: item[1][0])
    gaps = []
    for k, (i, a) in enumerate(projected):
        for j, b in projected[k + 1:]:
            width = b[0] - a[1]
            if not cfg.vehicle_width + 2 * cfg.clearance <= width <= cfg.max_gap:
                continue
            va, vb = (a[2] + a[3]) / 2, (b[2] + b[3]) / 2
            if abs(va - vb) > cfg.row_tolerance:
                continue
            if any(a[1] < (d[0] + d[1]) / 2 < b[0] and
                   abs((d[2] + d[3]) / 2 - (va + vb) / 2) <= cfg.row_tolerance
                   for n, d in projected if n not in (i, j)):
                continue
            u, v = (a[1] + b[0]) / 2, (va + vb) / 2
            yaw = cfg.row_yaw - math.pi / 2
            gaps.append(Gap(c * u - s * v, s * u + c * v, wrap(yaw), width, i, j))
    return sorted(gaps, key=lambda g: math.hypot(g.x, g.y))


class CollisionChecker:
    def __init__(self, scan, cfg, known_free_poses=()):
        self.scan, self.cfg = scan, cfg
        self.half_length = cfg.vehicle_length / 2 + cfg.clearance
        self.half_width = cfg.vehicle_width / 2 + cfg.clearance
        self.radius = math.hypot(self.half_length, self.half_width)
        self.cells = defaultdict(list)
        # During reverse departure only, the occupied inbound corridor supplies
        # coverage for the LiDAR's masked rear sector. Live returns take priority.
        self.known_cells = defaultdict(list)
        for x, y, yaw in known_free_poses:
            self.known_cells[math.floor(x / .2), math.floor(y / .2)].append(
                (x, y, math.cos(yaw), math.sin(yaw)))
        for point in scan.points:
            self.cells[math.floor(point[0] / 0.2), math.floor(point[1] / 0.2)].append(point)
        # Sample the whole footprint, including interior: a hidden hole/occlusion
        # inside the body must not be treated as free merely because corners fit.
        nx, ny = math.ceil(2 * self.half_length / 0.06), math.ceil(2 * self.half_width / 0.06)
        self.samples = [(self.half_length * (2 * i / nx - 1),
                         self.half_width * (2 * j / ny - 1))
                        for i in range(nx + 1) for j in range(ny + 1)]

    def previously_free(self, x, y):
        for ix in range(math.floor((x - self.radius) / .2), math.floor((x + self.radius) / .2) + 1):
            for iy in range(math.floor((y - self.radius) / .2), math.floor((y + self.radius) / .2) + 1):
                for px, py, c, s in self.known_cells[ix, iy]:
                    dx, dy = x - px, y - py
                    if (abs(c * dx + s * dy) <= self.half_length + 1e-8 and
                            abs(-s * dx + c * dy) <= self.half_width + 1e-8):
                        return True
        return False

    def free(self, pose):
        x, y, yaw = pose
        if math.hypot(x, y) > self.cfg.max_range:
            return False
        c, s = math.cos(yaw), math.sin(yaw)
        for ix in range(math.floor((x - self.radius) / 0.2), math.floor((x + self.radius) / 0.2) + 1):
            for iy in range(math.floor((y - self.radius) / 0.2), math.floor((y + self.radius) / 0.2) + 1):
                for px, py in self.cells[ix, iy]:
                    dx, dy = px - x, py - y
                    if abs(c * dx + s * dy) <= self.half_length and abs(-s * dx + c * dy) <= self.half_width:
                        return False
        for bx, by in self.samples:
            px, py = x + c * bx - s * by, y + s * bx + c * by
            # A configurable clear startup bubble covers the close rear sector
            # masked by the sensor. Measured obstacles are still checked above.
            if math.hypot(px, py) <= self.cfg.initial_clear_radius:
                continue
            if not self.scan.visible(px, py) and not self.previously_free(px, py):
                return False
        return True


def advance(pose, distance, curvature):
    x, y, yaw = pose
    if abs(curvature) < 1e-10:
        return x + distance * math.cos(yaw), y + distance * math.sin(yaw), yaw
    new_yaw = yaw + distance * curvature
    return (x + (math.sin(new_yaw) - math.sin(yaw)) / curvature,
            y + (math.cos(yaw) - math.cos(new_yaw)) / curvature, wrap(new_yaw))


def straight_turn_candidates(goal, cfg):
    """Straight, one turn toward the bay, then straight along its heading."""
    theta = wrap(goal.yaw)
    if abs(theta) < 1e-4:
        return [(goal.x, [(goal.x, 0.)])] if goal.x > 0 and abs(goal.y) < 1e-4 else []
    if abs(theta) >= math.pi - .05:
        return []
    direction = 1 if theta > 0 else -1
    minimum_radius = cfg.wheelbase / math.tan(cfg.max_steering)
    candidates = []
    for factor in (1., 1.05, 1.1, 1.25, 1.5):
        radius = minimum_radius * factor
        turn_x = direction * radius * math.sin(theta)
        turn_y = direction * radius * (1 - math.cos(theta))
        suffix = (goal.y - turn_y) / math.sin(theta)
        prefix = goal.x - turn_x - suffix * math.cos(theta)
        if prefix < -1e-6 or suffix < -1e-6:
            continue
        parts = [(max(0., prefix), 0.), (radius * abs(theta), direction / radius),
                 (max(0., suffix), 0.)]
        candidates.append((sum(length for length, _ in parts), parts))
    return sorted(candidates, key=lambda item: item[0])


def sample_candidate(checker, parts, cfg):
    """Return only the collision-checked, observed prefix of an arc sequence."""
    pose = (0., 0., 0.)
    if not checker.free(pose):
        return [], False
    path = [pose]
    for length, curvature in parts:
        if length > cfg.max_range * 3:
            return path, False
        if length <= 1e-9:
            continue
        count = max(1, math.ceil(length / cfg.collision_step))
        for i in range(1, count + 1):
            point = advance(pose, length * i / count, curvature)
            if not checker.free(point):
                return path, False
            path.append(point)
        pose = path[-1]
    return path, True


def arc_candidates(goal, cfg):
    """Forward CSC Dubins candidates from common signed-circle tangents.

    Four turn combinations, several radii and short initial straight sections
    give a fast feasible solution before the discretized fallback is needed.
    """
    minimum_radius = cfg.wheelbase / math.tan(cfg.max_steering)
    candidates = []
    for radius_factor in (1.0, 1.25, 1.5):
        radius = minimum_radius * radius_factor
        for prefix in (0.0, 0.12, 0.24, 0.36, 0.48):
            for t1 in (-1, 1):
                for t2 in (-1, 1):
                    c1 = (prefix, t1 * radius)
                    c2 = (goal.x - t2 * radius * math.sin(goal.yaw),
                          goal.y + t2 * radius * math.cos(goal.yaw))
                    dx, dy = c2[0] - c1[0], c2[1] - c1[1]
                    distance = math.hypot(dx, dy)
                    offset = (t2 - t1) * radius
                    if distance <= abs(offset) + 1e-9:
                        continue
                    alpha = math.atan2(dy, dx) - math.asin(offset / distance)
                    a1 = (t1 * alpha) % (2 * math.pi)
                    a2 = (t2 * (goal.yaw - alpha)) % (2 * math.pi)
                    straight = math.sqrt(distance * distance - offset * offset)
                    parts = [(prefix, 0.0), (radius * a1, t1 / radius),
                             (straight, 0.0), (radius * a2, t2 / radius)]
                    candidates.append((sum(length for length, _ in parts), parts))
    return sorted(candidates, key=lambda item: item[0])


def plan_path(scan, goal, cfg):
    """Bounded hybrid A*: Ackermann arcs with swept rectangular footprint.

    Returns densely sampled (x,y,heading) poses, or [] if no observed path fits.
    Forward-only by default; never connects the endpoint by a straight shortcut.
    """
    checker = CollisionChecker(scan, cfg)
    start = (0.0, 0.0, 0.0)
    if not checker.free(start) or not checker.free((goal.x, goal.y, goal.yaw)):
        return []
    deadline = time.monotonic() + cfg.planning_timeout
    for _, parts in straight_turn_candidates(goal, cfg) + arc_candidates(goal, cfg):
        path, valid = sample_candidate(checker, parts, cfg)
        if valid:
            return path
        if time.monotonic() > deadline:
            return []
    def key(pose):
        x, y, yaw = pose
        return round(x / cfg.resolution), round(y / cfg.resolution), round(wrap(yaw) / cfg.yaw_resolution)
    def heuristic(pose):
        return math.hypot(pose[0] - goal.x, pose[1] - goal.y) + 0.15 * abs(wrap(pose[2] - goal.yaw))
    nodes = [(start, -1, [])]
    costs = {key(start): 0.0}
    queue = [(heuristic(start), 0.0, 0)]
    radius = cfg.wheelbase / math.tan(cfg.max_steering)
    curvatures = (-1 / radius, -0.5 / radius, 0.0, 0.5 / radius, 1 / radius)
    directions = (1, -1) if cfg.allow_reverse else (1,)
    samples = max(1, math.ceil(cfg.step / cfg.collision_step))
    expanded = 0
    while queue and expanded < cfg.max_expansions and time.monotonic() < deadline:
        _, cost, index = heapq.heappop(queue)
        pose, _, _ = nodes[index]
        if cost > costs.get(key(pose), math.inf) + 1e-9:
            continue
        expanded += 1
        if math.hypot(pose[0] - goal.x, pose[1] - goal.y) <= cfg.goal_tolerance and abs(wrap(pose[2] - goal.yaw)) <= cfg.goal_yaw_tolerance:
            segments = []
            while index > 0:
                _, parent, segment = nodes[index]
                segments.append(segment)
                index = parent
            return [start] + [p for seg in reversed(segments) for p in seg]
        for direction in directions:
            for curvature in curvatures:
                segment = []
                for i in range(1, samples + 1):
                    new_pose = advance(pose, direction * cfg.step * i / samples, curvature)
                    if not checker.free(new_pose):
                        break
                    segment.append(new_pose)
                if len(segment) != samples:
                    continue
                new_pose = segment[-1]
                new_cost = cost + cfg.step * (1.0 if direction > 0 else 1.8) + 0.01 * abs(curvature)
                cell = key(new_pose)
                if new_cost >= costs.get(cell, math.inf):
                    continue
                costs[cell] = new_cost
                nodes.append((new_pose, index, segment))
                heapq.heappush(queue, (new_cost + heuristic(new_pose), new_cost, len(nodes) - 1))
    return []
