"""Reactive LiDAR avoidance with an entry-anchored three-lane road guard."""
from dataclasses import dataclass, replace
import math
import time
import numpy as np

DISTANCE_EPS = 1e-6  # LaserScan float32 rounding at inclusive distance thresholds.


@dataclass(frozen=True)
class Config:
    front_half_angle_deg: float = 25.0
    side_limit_angle_deg: float = 90.0
    far_distance: float = 1.0
    near_distance: float = 0.5
    stop_distance: float = 0.05
    cruise_speed: float = 0.4
    avoid_speed: float = 0.25
    near_speed: float = 0.2
    far_steering: float = 0.30
    near_steering: float = 0.32
    side_steering: float = 0.12
    switch_margin: float = 0.15
    lane_gain: float = 0.8
    max_lane_steering: float = 0.15
    scan_timeout: float = 0.25
    lane_timeout: float = 0.5
    future_tolerance: float = 0.05
    publish_rate: float = 20.0
    min_valid_beams: int = 3
    lidar_yaw_deg: float = 0.0
    inf_is_clear: bool = False
    wheelbase: float = 0.2
    preview_distance: float = 0.7
    min_preview_distance: float = 0.15
    cluster_jump_distance: float = 0.12
    cluster_min_points: int = 3
    cluster_max_distance: float = 3.0
    object_max_extent: float = 1.0
    vehicle_length: float = 0.322
    vehicle_width: float = 0.220
    lidar_x: float = 0.103
    lidar_y: float = 0.0
    clearance_margin: float = 0.02
    visual_road_distance: float = 2.0
    road_guard_enabled: bool = True
    lane_width: float = 0.52
    initial_lane: int = 2
    lane_change_hysteresis: float = 0.03
    odom_timeout: float = 0.5

    def __post_init__(self):
        if not all(math.isfinite(v) for v in vars(self).values()):
            raise ValueError('all settings must be finite')
        if not 0 < self.front_half_angle_deg < self.side_limit_angle_deg <= 110:
            raise ValueError('invalid sector angles')
        if not 0 < self.stop_distance < self.near_distance < self.far_distance:
            raise ValueError('require stop_distance < near_distance < far_distance')
        if not 0 < self.near_speed <= self.avoid_speed <= self.cruise_speed <= .5:
            raise ValueError('invalid speed limits')
        if not 0 < self.far_steering <= self.near_steering <= .4886921906:
            raise ValueError('invalid steering limits')
        if not 0 < self.side_steering <= self.far_steering:
            raise ValueError('side_steering must be positive and <= far_steering')
        if not 0 < self.min_preview_distance <= self.preview_distance:
            raise ValueError('require 0 < min_preview_distance <= preview_distance')
        if self.visual_road_distance <= 0:
            raise ValueError('visual_road_distance must be positive')
        if (type(self.road_guard_enabled) is not bool or type(self.initial_lane) is not int
                or self.initial_lane not in (1, 2, 3)
                or self.lane_width <= self.vehicle_width + 2 * self.clearance_margin
                or not 0 <= self.lane_change_hysteresis < self.lane_width / 2
                or self.odom_timeout <= 0):
            raise ValueError('invalid road guard settings')
        if (min(self.scan_timeout, self.lane_timeout, self.publish_rate,
                self.wheelbase, self.preview_distance, self.cluster_jump_distance,
                self.cluster_max_distance, self.object_max_extent,
                self.vehicle_length, self.vehicle_width) <= 0
                or min(self.switch_margin, self.future_tolerance, self.lane_gain,
                       self.max_lane_steering, self.clearance_margin) < 0
                or self.max_lane_steering > self.near_steering
                or type(self.min_valid_beams) is not int or self.min_valid_beams < 1
                or type(self.cluster_min_points) is not int or self.cluster_min_points < 1):
            raise ValueError('invalid timing, beam count, or lane settings')


@dataclass(frozen=True)
class Sector:
    distance: float
    valid_beams: int

    def known(self, c):
        return self.valid_beams >= c.min_valid_beams


@dataclass(frozen=True)
class RoadBand:
    nx: float
    ny: float
    offset: float
    half_width: float
    lane: int

    def body_gap(self, x, y, yaw, half_length, half_width):
        co, si = math.cos(yaw), math.sin(yaw)
        radius = (half_length * abs(self.nx * co + self.ny * si)
                  + half_width * abs(-self.nx * si + self.ny * co))
        return self.half_width - abs(self.nx * x + self.ny * y + self.offset) - radius

    def point(self, distance, lateral):
        return (self.ny * distance + self.nx * (lateral - self.offset),
                -self.nx * distance + self.ny * (lateral - self.offset))

    def lane_steering(self, c):
        target = (2 - self.lane) * c.lane_width
        angle = c.lane_gain * (math.atan2(target - self.offset, c.preview_distance)
                               + math.atan2(-self.nx, self.ny))
        return max(-c.max_lane_steering, min(c.max_lane_steering, angle))


@dataclass(frozen=True)
class OdomPose:
    x: float
    y: float
    yaw: float
    stamp: float
    received: float
    frame: str


def read_odometry(msg, received):
    p, q = msg.pose.pose.position, msg.pose.pose.orientation
    stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    if not all(math.isfinite(v) for v in (p.x, p.y, q.x, q.y, q.z, q.w, stamp, received)):
        raise ValueError('non-finite odometry')
    norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
    if not math.isfinite(norm) or norm < 1e-8 or not msg.header.frame_id:
        raise ValueError('invalid odometry orientation/frame')
    x, y, z, w = q.x / norm, q.y / norm, q.z / norm, q.w / norm
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return OdomPose(p.x, p.y, yaw, stamp, received, msg.header.frame_id)


def odom_fresh(pose, c, ros_now, steady_now):
    return (pose is not None and pose.stamp > 0
            and -c.future_tolerance <= ros_now - pose.stamp < c.odom_timeout
            and 0 <= steady_now - pose.received < c.odom_timeout)


@dataclass
class RoadTracker:
    origin: tuple | None = None
    yaw: float = 0.
    frame: str = ''
    lane: int = 2

    def reset(self):
        self.origin, self.frame = None, ''

    def initialize(self, pose, data, c):
        if self.origin is not None:
            return pose.frame == self.frame
        if not lane_reference(data):
            return False
        near_offset = data[10] * math.sin(data[9]) - data[8] * math.cos(data[9])
        if abs(near_offset) > c.lane_width / 2:
            return False  # The initial detection must agree with the stated entry lane.
        self.yaw = pose.yaw + data[9]
        co, si = math.cos(pose.yaw), math.sin(pose.yaw)
        x = pose.x + co * data[10] - si * data[8]
        y = pose.y + si * data[10] + co * data[8]
        lateral = (2 - c.initial_lane) * c.lane_width
        self.origin = (x + math.sin(self.yaw) * lateral, y - math.cos(self.yaw) * lateral)
        self.frame, self.lane = pose.frame, c.initial_lane
        return True

    def band(self, pose, c):
        if self.origin is None or pose.frame != self.frame:
            return None
        nx, ny = -math.sin(self.yaw), math.cos(self.yaw)
        lateral = nx * (pose.x - self.origin[0]) + ny * (pose.y - self.origin[1])
        threshold = c.lane_width / 2
        margin = c.lane_change_hysteresis
        left_threshold = threshold + (-margin if self.lane == 1 else margin)
        right_threshold = -threshold + (margin if self.lane == 3 else -margin)
        if lateral > left_threshold:
            self.lane = 1
        elif lateral < right_threshold:
            self.lane = 3
        else:
            self.lane = 2
        co, si = math.cos(pose.yaw), math.sin(pose.yaw)
        return RoadBand(nx * co + ny * si, -nx * si + ny * co,
                        lateral, 1.5 * c.lane_width, self.lane)


@dataclass(frozen=True)
class Sectors:
    left: Sector
    front: Sector
    right: Sector
    stamp: float
    received: float
    frame_id: str = 'laser_link'
    points: tuple = ()
    boxes: tuple = ()  # (centre x/y, half length/width, yaw), matching displayed boxes.
    road: RoadBand | None = None


def read_scan(msg, received, c):
    if (not all(math.isfinite(v) for v in (
            msg.angle_min, msg.angle_increment, msg.range_min, msg.range_max))
            or msg.angle_increment == 0 or not 0 <= msg.range_min < msg.range_max
            or len(msg.ranges) < 2
            or (len(msg.ranges) - 1) * abs(msg.angle_increment) > 2 * math.pi + .01):
        raise ValueError('invalid LaserScan geometry')
    values = {'left': [], 'front': [], 'right': []}
    points = []
    for i, value in enumerate(msg.ranges):
        angle = math.degrees(msg.angle_min + i * msg.angle_increment) + c.lidar_yaw_deg
        angle = (angle + 180) % 360 - 180
        a = math.radians(angle)
        co, si = math.cos(a), math.sin(a)
        valid = math.isfinite(value) and max(msg.range_min, 1e-6) <= value <= msg.range_max
        if valid:
            points.append((c.lidar_x + value * co, c.lidar_y + value * si))
        if abs(angle) <= c.front_half_angle_deg:
            name = 'front'
        elif c.front_half_angle_deg < angle <= c.side_limit_angle_deg:
            name = 'left'
        elif -c.side_limit_angle_deg <= angle < -c.front_half_angle_deg:
            name = 'right'
        else:
            continue
        if valid:
            values[name].append(float(value))
        elif c.inf_is_clear and value == math.inf:
            values[name].append(float(msg.range_max))
    sectors = {name: Sector(min(points) if points else math.inf, len(points))
               for name, points in values.items()}
    stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    if not math.isfinite(stamp) or not math.isfinite(received):
        raise ValueError('invalid scan time')
    return Sectors(sectors['left'], sectors['front'], sectors['right'], stamp, received,
                   getattr(msg.header, 'frame_id', '') or 'laser_link', tuple(points))


def cluster_scan(msg, c):
    """Group consecutive finite laser returns by Euclidean distance, in scan frame."""
    groups, current = [], []
    first_valid = last_valid = False
    limit = min(msg.range_max, c.cluster_max_distance)
    for index, value in enumerate(msg.ranges):
        valid = math.isfinite(value) and max(msg.range_min, 1e-6) <= value <= limit
        if index == 0:
            first_valid = valid
        last_valid = valid
        if not valid:
            if current:
                groups.append(current)
                current = []
            continue
        angle = msg.angle_min + index * msg.angle_increment
        point = (value * math.cos(angle), value * math.sin(angle))
        if current and math.dist(current[-1], point) > c.cluster_jump_distance:
            groups.append(current)
            current = []
        current.append(point)
    if current:
        groups.append(current)
    full_circle = (len(msg.ranges) - 1) * abs(msg.angle_increment) >= (
        2 * math.pi - 2 * abs(msg.angle_increment))
    if (full_circle and first_valid and last_valid and len(groups) > 1
            and math.dist(groups[-1][-1], groups[0][0]) <= c.cluster_jump_distance):
        tail = groups[-1]
        if math.dist(tail[-1], groups[0][0]) < 1e-8:
            tail = tail[:-1]  # A 360-degree scan can repeat the same endpoint.
        groups = [tail + groups[0]] + groups[1:-1]
    return tuple(tuple(points) for points in groups if len(points) >= c.cluster_min_points)


def scan_fresh(scan, c, ros_now, steady_now):
    return (scan is not None and scan.stamp > 0
            and -c.future_tolerance <= ros_now - scan.stamp < c.scan_timeout
            and 0 <= steady_now - scan.received < c.scan_timeout)


def compact_clusters(groups, c):
    """Long wall-like groups are separate from compact obstacle-side preference."""
    return tuple(p for p in groups if math.hypot(
        max(x for x, y in p) - min(x for x, y in p),
        max(y for x, y in p) - min(y for x, y in p)) <= c.object_max_extent)


def cluster_bounds(points):
    xs, ys = zip(*points)
    return min(xs) - .02, max(xs) + .02, min(ys) - .02, max(ys) + .02


def object_sectors(scan, groups, c):
    distances = {1: math.inf, -1: math.inf}
    counts = {1: scan.left.valid_beams, -1: scan.right.valid_beams}
    a = math.radians(c.lidar_yaw_deg)
    co, si = math.cos(a), math.sin(a)
    boxes = []
    for group in compact_clusters(groups, c):
        xmin, xmax, ymin, ymax = cluster_bounds(group)
        x, y = (xmin + xmax) / 2, (ymin + ymax) / 2
        boxes.append((c.lidar_x + co * x - si * y, c.lidar_y + si * x + co * y,
                      (xmax - xmin) / 2, (ymax - ymin) / 2, a))
        for x, y in group:
            bx, by = c.lidar_x + co * x - si * y, c.lidar_y + si * x + co * y
            # Include adjacent/rear corners outside the old +/-90 degree sectors.
            if (-c.vehicle_length / 2 - .1 <= bx <= c.far_distance + c.vehicle_length / 2
                    and abs(by) > c.vehicle_width / 2 + c.clearance_margin):
                side = 1 if by > 0 else -1
                distances[side] = min(distances[side], math.hypot(x, y))
                counts[side] += 1
    return Sectors(Sector(distances[1], counts[1]), scan.front,
                   Sector(distances[-1], counts[-1]), scan.stamp, scan.received,
                   scan.frame_id, scan.points, tuple(boxes), scan.road)


def path_clearance(scan, steering, c):
    """Swept-body clearance to raw returns and filled RViz obstacle boxes."""
    curvature = math.tan(steering) / c.wheelbase
    half_length = c.vehicle_length / 2 + c.clearance_margin
    half_width = c.vehicle_width / 2 + c.clearance_margin
    # Cover motion and rotation between samples, not only the sampled rectangles.
    step = c.preview_distance / 24
    padding = step / 2 + 2 * math.hypot(half_length, half_width) * math.sin(
        min(math.pi / 2, abs(curvature) * step / 4))
    half_length += padding
    half_width += padding
    clearance = c.far_distance
    # More distant returns cannot reduce the capped gap anywhere on this arc.
    limit = c.preview_distance + math.hypot(half_length, half_width) + clearance
    points = tuple((x, y) for x, y in scan.points if math.hypot(x, y) <= limit)
    boxes = tuple((x, y, hx, hy, math.cos(a), math.sin(a))
                  for x, y, hx, hy, a in scan.boxes
                  if math.hypot(x, y) <= limit + math.hypot(hx, hy))
    path = preview_points(steering, c)
    if len(points) > 32:
        # Evaluate the exact same rectangles/points in bulk, without dropping
        # returns or changing the swept-body padding and clearance thresholds.
        positions = np.asarray(path, dtype=float)
        cloud = np.asarray(points, dtype=float)
        yaw = np.arange(25) * curvature * c.preview_distance / 24
        co, si = np.cos(yaw)[:, None], np.sin(yaw)[:, None]
        dx = cloud[None, :, 0] - positions[:, None, 0]
        dy = cloud[None, :, 1] - positions[:, None, 1]
        gap_x = np.abs(co*dx + si*dy) - half_length
        gap_y = np.abs(-si*dx + co*dy) - half_width
        if np.any((gap_x <= 0.) & (gap_y <= 0.)):
            return 0.
        clearance = min(clearance, float(np.min(np.hypot(np.maximum(0., gap_x),
                                                        np.maximum(0., gap_y)))))
        points = ()
    for index, (x, y) in enumerate(path):
        yaw = curvature * c.preview_distance * index / 24
        co, si = math.cos(yaw), math.sin(yaw)
        if scan.road is not None:
            gap = scan.road.body_gap(x, y, yaw, half_length, half_width)
            if gap <= 0:
                return 0.
            clearance = min(clearance, gap)
        for px, py in points:
            dx, dy = px - x, py - y
            bx, by = co * dx + si * dy, -si * dx + co * dy
            gap_x, gap_y = abs(bx) - half_length, abs(by) - half_width
            if gap_x <= 0 and gap_y <= 0:
                return 0.
            clearance = min(clearance, math.hypot(max(0., gap_x), max(0., gap_y)))
        for ox, oy, hx, hy, oc, os in boxes:
            dx, dy = ox - x, oy - y
            along, across = abs(co * oc + si * os), abs(si * oc - co * os)
            # Separating-axis test for two oriented, filled rectangles.
            gap = max(abs(co * dx + si * dy) - half_length - hx * along - hy * across,
                      abs(-si * dx + co * dy) - half_width - hx * across - hy * along,
                      abs(oc * dx + os * dy) - hx - half_length * along - half_width * across,
                      abs(-os * dx + oc * dy) - hy - half_length * across - half_width * along)
            if gap <= 0:
                return 0.
            clearance = min(clearance, gap)
    return clearance


def path_clear(scan, steering, c):
    return path_clearance(scan, steering, c) > 0.


def cluster_direction_scores(scan, c):
    """Avoid the nearest observed cluster behind the immediate front obstacle."""
    width = c.vehicle_width / 2 + c.clearance_margin
    front = c.vehicle_length / 2
    primary = []
    for index, (x, y, hx, hy, yaw) in enumerate(scan.boxes):
        ex = abs(math.cos(yaw))*hx + abs(math.sin(yaw))*hy
        ey = abs(math.sin(yaw))*hx + abs(math.cos(yaw))*hy
        if x + ex > front and abs(y) <= width + ey:
            primary.append((x-ex, index, x))
    if primary:
        _, first_index, first_x = min(primary)
    else:
        points = [x for x, y in scan.points if x > front and abs(y) <= width]
        if not points:
            return {1: None, -1: None}
        first_index, first_x = -1, min(points)
    behind = []
    for index, (x, y, hx, hy, yaw) in enumerate(scan.boxes):
        if index == first_index or x <= first_x + DISTANCE_EPS:
            continue
        ex = abs(math.cos(yaw))*hx + abs(math.sin(yaw))*hy
        ey = abs(math.sin(yaw))*hx + abs(math.cos(yaw))*hy
        distance = math.hypot(max(0., abs(x)-ex), max(0., abs(y)-ey))
        behind.append((distance, y))
    if not behind:
        return {1: None, -1: None}
    distance, lateral = min(behind)
    if abs(lateral) <= c.clearance_margin:
        return {1: None, -1: None}  # A centred car provides no left/right preference.
    if any(abs(other_distance-distance) <= DISTANCE_EPS and other_y*lateral < 0
           for other_distance, other_y in behind):
        return {1: None, -1: None}
    preferred = -1 if lateral > 0 else 1
    return {preferred: 1., -preferred: 0.}


@dataclass
class AvoidanceMemory:
    side: int = 0
    phase: str = 'STRAIGHT'
    entry_yaw: float | None = None
    frame: str = ''
    obstacle: tuple | None = None  # world x/y, half length/width, yaw

    def reset(self):
        self.side, self.phase = 0, 'STRAIGHT'
        self.entry_yaw, self.frame = None, ''
        self.obstacle = None


def forward_entry_clear(scan, c):
    """The longer straight corridor triggers avoidance before the command arc."""
    return path_clear(scan, 0., replace(c, preview_distance=max(c.preview_distance, c.far_distance)))


def adaptive_prefix(evaluate, c):
    """Find the longest checked prefix; never command a token sliver of space."""
    # Cover one input-timeout window plus one control period at crawl speed.
    minimum = max(c.min_preview_distance,
                  c.near_speed * (c.scan_timeout + 1 / c.publish_rate))
    if minimum >= c.preview_distance:
        return None
    low, high = minimum, c.preview_distance
    checked = replace(c, preview_distance=low)
    result = evaluate(checked)
    if result[1] <= 0:
        return None
    best = result, checked
    for _ in range(8):
        distance = (low + high) / 2
        checked = replace(c, preview_distance=distance)
        result = evaluate(checked)
        if result[1] > 0:
            low, best = distance, (result, checked)
        else:
            high = distance
    result, checked = best
    state, speed, steering, side = result
    return (state, min(speed, c.near_speed), steering, side), checked


def checked_command(scan, c, angles, state, speed, side=0):
    """Prefer long safe paths, using the same candidate order at every length."""
    def evaluate(checked):
        for angle in dict.fromkeys(angles):
            if path_clear(scan, angle, checked):
                return state, speed, angle, side
        return 'STOP_PATH_BLOCKED', 0., 0., side
    result = evaluate(c)
    return (result, c) if result[1] > 0 else adaptive_prefix(evaluate, c) or (result, c)


def turn_path(scan, c, side=0, scores=None):
    """Avoid the front car; use the nearest following cluster for side preference."""
    scores = cluster_direction_scores(scan, c) if scores is None else scores
    limit = c.near_steering if scan.front.distance <= c.near_distance else c.far_steering
    speed = c.near_speed if scan.front.distance <= c.near_distance else c.avoid_speed

    def evaluate(checked):
        candidates = []
        for direction in (1, -1):
            sector = scan.left if direction > 0 else scan.right
            if not sector.known(c):
                continue
            for magnitude in dict.fromkeys((c.side_steering, (limit + c.side_steering) / 2, limit)):
                angle = direction * magnitude
                gap = path_clearance(scan, angle, checked)
                if gap > 0:
                    candidates.append((gap, angle, direction))
        if not candidates:
            return 'STOP_PATH_BLOCKED', 0., 0., side
        preferred = 0
        left, right = scores.get(1), scores.get(-1)
        if left is not None or right is not None:
            if left != right:
                preferred = 1 if (left or 0.) > (right or 0.) else -1
        pool = ([p for p in candidates if p[2] == side]
                or [p for p in candidates if p[2] == preferred] or candidates)
        best = max(p[0] for p in pool)
        tolerance = min(c.switch_margin, best * .25)
        comparable = [p for p in pool if p[0] >= best - tolerance]
        gap, angle, direction = min(comparable, key=lambda p: (abs(p[1]), -p[0]))
        return ('AVOID_LEFT' if direction > 0 else 'AVOID_RIGHT'), speed, angle, direction

    result = evaluate(c)
    return (result, c) if result[1] > 0 else adaptive_prefix(evaluate, c) or (result, c)


def remember_obstacle(scan, c, memory, pose):
    """Remember the triggering observed footprint in odom, not a sector minimum."""
    memory.obstacle = None
    if pose is None:
        return
    candidates = list(scan.boxes) or [(x, y, .02, .02, 0.) for x, y in scan.points]
    front = c.vehicle_length / 2
    width = c.vehicle_width / 2 + c.clearance_margin
    ahead = []
    for x, y, hx, hy, yaw in candidates:
        ex = abs(math.cos(yaw)) * hx + abs(math.sin(yaw)) * hy
        ey = abs(math.sin(yaw)) * hx + abs(math.cos(yaw)) * hy
        if x + ex > front and abs(y) <= width + ey:
            ahead.append((x - ex, x, y, hx, hy, yaw))
    if not ahead:
        return
    _, x, y, hx, hy, yaw = min(ahead)
    co, si = math.cos(pose.yaw), math.sin(pose.yaw)
    memory.obstacle = (pose.x + co*x - si*y, pose.y + si*x + co*y,
                       hx, hy, pose.yaw + yaw)
    memory.frame = pose.frame


def obstacle_passed(scan, c, memory, pose):
    if memory.obstacle is None:
        # Without an entry reference, require observed side passage rather
        # than assuming a briefly clear forward preview means the car passed.
        front = c.vehicle_length / 2 + c.clearance_margin
        width = c.vehicle_width / 2 + c.clearance_margin
        for x, y, hx, hy, yaw in scan.boxes:
            ex = abs(math.cos(yaw))*hx + abs(math.sin(yaw))*hy
            ey = abs(math.sin(yaw))*hx + abs(math.cos(yaw))*hy
            if x - ex <= front and -memory.side*y - ey > width:
                return True
        return not (scan.points or scan.boxes) and scan.front.distance >= c.far_distance
    if pose is None or pose.frame != memory.frame:
        return False
    ox, oy, hx, hy, yaw = memory.obstacle
    co, si = math.cos(pose.yaw), math.sin(pose.yaw)
    dx, dy = ox - pose.x, oy - pose.y
    x, y = co*dx + si*dy, -si*dx + co*dy
    relative = yaw - pose.yaw
    ex = abs(math.cos(relative))*hx + abs(math.sin(relative))*hy
    ey = abs(math.sin(relative))*hx + abs(math.cos(relative))*hy
    return (x - ex <= c.vehicle_length / 2 + c.clearance_margin
            and -memory.side*y - ey > c.vehicle_width / 2 + c.clearance_margin)


def control_command(scan, c, memory, lane_steering=None, heading_error=None, scores=None, pose=None):
    """Straight -> fixed-side avoidance -> heading alignment -> lane following."""
    if not scan.front.known(c):
        return ('STOP_FRONT_UNKNOWN', 0., 0., memory.side), c
    if scan.front.distance <= c.stop_distance + DISTANCE_EPS:
        return ('STOP_TOO_CLOSE', 0., 0., memory.side), c
    forward = forward_entry_clear(scan, c)
    obstacle_forward = forward_entry_clear(replace(scan, road=None), c)
    if memory.phase == 'TURN':
        if obstacle_passed(scan, c, memory, pose):
            memory.phase = 'ALIGN'
            memory.side, memory.obstacle = 0, None
        else:
            result, checked = turn_path(scan, c, memory.side, scores)
            if result[1] > 0 and result[3] != memory.side:
                memory.side = result[3]
                remember_obstacle(scan, c, memory, pose)
            return result, checked
    if memory.phase == 'ALIGN':
        if not obstacle_forward:
            result, checked = turn_path(scan, c, 0, scores)
            if result[1] > 0:
                memory.side, memory.phase = result[3], 'TURN'
                remember_obstacle(scan, c, memory, pose)
            return result, checked
        if heading_error is None:
            return ('STOP_ALIGNMENT_REFERENCE', 0., 0., memory.side), c
        if abs(heading_error) > math.radians(5):
            angle = max(-c.max_lane_steering, min(c.max_lane_steering, c.lane_gain * heading_error))
            if abs(angle) < 1e-6:
                return ('STOP_ALIGNMENT_REFERENCE', 0., 0., memory.side), c
            return checked_command(scan, c, (angle, angle / 2), 'ALIGN', c.avoid_speed, memory.side)
        memory.phase, memory.side = 'LANE', 0
    if not forward:
        result, checked = turn_path(scan, c, memory.side, scores)
        if result[1] > 0:
            memory.side, memory.phase = result[3], 'TURN'
            remember_obstacle(scan, c, memory, pose)
        return result, checked
    if memory.phase == 'LANE' and lane_steering is not None:
        angle = max(-c.max_lane_steering, min(c.max_lane_steering, lane_steering))
        speed = c.avoid_speed if abs(angle) > 1e-6 else c.cruise_speed
        return checked_command(scan, c, (angle, angle / 2, 0.), 'LANE_FOLLOW', speed)
    return ('PATH_FORWARD', c.cruise_speed, 0., 0), c


def path_rejections(s, c):
    limit = c.near_steering if s.front.distance <= c.near_distance else c.far_steering
    angles = (0., c.side_steering, (limit + c.side_steering) / 2, limit)
    angles += tuple(-a for a in angles[1:])
    obstacles = replace(s, road=None)
    road = replace(s, points=(), boxes=())
    return (sum(not path_clear(obstacles, a, c) for a in angles),
            sum(not path_clear(road, a, c) for a in angles), len(angles))


def lane_hint(data, c):
    if len(data) != 11 or not all(math.isfinite(v) for v in data) or data[0] != 1:
        return None
    if data[3] <= 0:
        return None
    angle = c.lane_gain * (math.atan2(data[1], data[3]) + data[2])
    return max(-c.max_lane_steering, min(c.max_lane_steering, angle))


def lane_guidance(lane, road, c, now):
    """Prefer fresh camera measurements; odometry bridges missing lane input."""
    if lane is not None and 0 <= now - lane[1] < c.lane_timeout:
        return lane[0]
    if road is not None:
        return road.lane_steering(c)
    return None


def lane_reference(data):
    """Connect the two reported centre points; these are not measured road edges."""
    if len(data) != 11 or data[0] != 1 or not all(math.isfinite(v) for v in data):
        return ()
    if not 0 < data[10] < data[3]:
        return ()
    return ((float(data[10]), float(data[8])), (float(data[3]), float(data[1])))


def preview_points(steering, c):
    """Constant-steering bicycle arc used for preview and known-point screening."""
    curvature = math.tan(steering) / c.wheelbase
    points = []
    for i in range(25):
        distance = c.preview_distance * i / 24
        if abs(curvature) < 1e-6:
            points.append((distance, 0.))
        else:
            points.append((math.sin(curvature * distance) / curvature,
                           (1 - math.cos(curvature * distance)) / curvature))
    return points


def main(args=None):
    import rclpy
    from rclpy.clock import Clock, ClockType
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, qos_profile_sensor_data
    from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import Float32MultiArray
    from ackermann_msgs.msg import AckermannDriveStamped
    from nav_msgs.msg import Odometry

    class ObjavoidNode(Node):
        def __init__(self):
            super().__init__('objavoid_node')
            ro = ParameterDescriptor(read_only=True)
            self.c = Config(**{key: self.declare_parameter(key, value, ro).value
                               for key, value in vars(Config()).items()})
            scan_topic = self.declare_parameter('scan_topic', '/scan', ro).value
            lane_topic = self.declare_parameter('lane_target_topic', '/lane_lite/target', ro).value
            odom_topic = self.declare_parameter('odom_topic', '/odom', ro).value
            command_topic = self.declare_parameter('command_topic', '/mission/objavoid/command_05', ro).value
            if not scan_topic or not lane_topic or not odom_topic or not command_topic or command_topic == '/cmd_vel':
                raise ValueError('invalid topic setting')
            self.declare_parameter('manual_active', True)
            self.scan = None
            self.scan_error = 'not received'
            self.clusters = ()
            self.cluster_frame = 'laser_link'
            self.lane = None
            self.odom = None
            self.odom_error = 'not received'
            self.road_tracker = RoadTracker()
            self.current_road = None
            self.memory = AvoidanceMemory()
            self.direction_scores = {1: None, -1: None}
            self.last_ros = None
            self.last_state = None
            self.last_lane = None
            self.last_log = -math.inf
            self.path_config = self.c
            self.pub = self.create_publisher(AckermannDriveStamped, command_topic, QoSProfile(depth=1))
            from .visualization import Visualizer
            self.visualizer = Visualizer(self, preview_points, cluster_bounds)
            self.scan_sub = self.create_subscription(LaserScan, scan_topic, self.on_scan,
                                                     QoSProfile(depth=1, reliability=qos_profile_sensor_data.reliability))
            self.lane_sub = self.create_subscription(Float32MultiArray, lane_topic,
                                                     self.on_lane, QoSProfile(depth=1))
            self.odom_sub = self.create_subscription(Odometry, odom_topic, self.on_odom,
                                                     qos_profile_sensor_data)
            self.add_on_set_parameters_callback(self.on_parameters)
            self.steady = Clock(clock_type=ClockType.STEADY_TIME)
            self.timer = self.create_timer(1 / self.c.publish_rate, self.tick, clock=self.steady)
            self.get_logger().info('LiDAR avoidance; road guard requires odometry and an initial lane reference')
            self.get_logger().info('RViz MarkerArray: /objavoid/markers (fixed frame: base_link)')
            self.get_logger().info('RViz Path: /objavoid/path (command projection)')
            self.get_logger().info('Compact obstacle clusters: side selection + /objavoid/clusters')
            self.get_logger().info(f'Applied distances: far={self.c.far_distance:.2f} '
                                   f'near={self.c.near_distance:.2f} stop={self.c.stop_distance:.2f}m')
            self.get_logger().info('Direction choice: opposite the nearest cluster behind the front obstacle')
            self.get_logger().info(f'Road guard={self.c.road_guard_enabled}: '
                                   f'initial lane={self.c.initial_lane}, lane width={self.c.lane_width:.2f}m, '
                                   f'odometry={odom_topic}; waiting for entry lane reference')

        def publish_visual(self, state, speed, steering, now):
            if not self.visualizer.due(state, now):
                return
            lane_points = (lane_reference(self.lane[2]) if self.lane and
                           0 <= now - self.lane[1] < self.c.lane_timeout else ())
            fresh = scan_fresh(self.scan, self.c, self.get_clock().now().nanoseconds * 1e-9, now)
            groups = compact_clusters(self.clusters, self.c) if fresh else ()
            self.visualizer.publish(state, speed, steering, now, self.path_config,
                                    lane_points, self.current_road, groups, self.cluster_frame)

        def on_parameters(self, parameters):
            for p in parameters:
                if p.name == 'manual_active' and type(p.value) is not bool:
                    return SetParametersResult(successful=False, reason='manual_active must be bool')
            return SetParametersResult(successful=True)

        def on_scan(self, msg):
            try:
                scan = read_scan(msg, time.monotonic(), self.c)
                if self.scan and scan.stamp <= self.scan.stamp:
                    return
                self.clusters = cluster_scan(msg, self.c)
                self.direction_scores = {1: None, -1: None}
                self.cluster_frame = scan.frame_id
                self.scan = scan
                self.scan_error = None
            except (ValueError, TypeError, OverflowError) as error:
                self.scan = None
                self.clusters = ()
                self.direction_scores = {1: None, -1: None}
                self.memory.reset()
                self.scan_error = str(error)

        def on_lane(self, msg):
            hint = lane_hint(msg.data, self.c)
            self.lane = (hint, time.monotonic(), tuple(msg.data)) if hint is not None else None

        def on_odom(self, msg):
            try:
                pose = read_odometry(msg, time.monotonic())
                if self.odom is not None and pose.stamp <= self.odom.stamp:
                    return
                self.odom, self.odom_error = pose, None
            except (ValueError, TypeError, OverflowError) as error:
                self.odom, self.odom_error = None, str(error)

        def road_context(self, ros_now, now):
            if not self.c.road_guard_enabled:
                return None, None
            if self.odom is None:
                return None, f'WAIT_ODOM ({self.odom_error})'
            if not odom_fresh(self.odom, self.c, ros_now, now):
                return None, 'STOP_ODOM_EXPIRED'
            if self.road_tracker.origin is None:
                if (not self.lane or not 0 <= now - self.lane[1] < self.c.lane_timeout
                        or not self.road_tracker.initialize(self.odom, self.lane[2], self.c)):
                    return None, 'WAIT_ROAD_REFERENCE'
                self.get_logger().info('Road reference initialized from entry lane + odometry; '
                                       'later lane detections do not shift the road band')
            road = self.road_tracker.band(self.odom, self.c)
            return (road, None) if road is not None else (None, 'STOP_ODOM_FRAME_CHANGED')

        def tick(self):
            now = time.monotonic()
            ros_now = self.get_clock().now().nanoseconds * 1e-9
            if self.last_ros is not None and ros_now < self.last_ros:
                self.scan = self.lane = None
                self.odom = None
                self.road_tracker.reset()
                self.scan_error = 'clock reset; waiting for new scan'
                self.memory.reset()
                self.direction_scores = {1: None, -1: None}
            self.last_ros = ros_now
            self.current_road, road_error = self.road_context(ros_now, now)
            speed = steering = 0.
            self.path_config = self.c
            control_scan = self.scan
            if not self.get_parameter('manual_active').value:
                self.memory.reset()
                self.last_state = 'IDLE'
                self.publish_visual('IDLE', 0., 0., now)
                return
            if self.scan is None:
                self.memory.reset()
                state = f'WAIT_SCAN ({self.scan_error})'
            elif not scan_fresh(self.scan, self.c, ros_now, now):
                self.memory.reset()
                state = 'STOP_SCAN_EXPIRED'
            elif road_error is not None:
                self.memory.reset()
                state = road_error
            else:
                hint = lane_guidance(self.lane, self.current_road, self.c, now)
                control_scan = object_sectors(self.scan, self.clusters, self.c)
                control_scan = replace(control_scan, road=self.current_road)
                self.direction_scores = cluster_direction_scores(control_scan, self.c)
                pose = self.odom if odom_fresh(self.odom, self.c, ros_now, now) else None
                if self.memory.side == 0 and pose is not None:
                    self.memory.entry_yaw, self.memory.frame = pose.yaw, pose.frame
                heading_error = None
                if self.current_road is not None:
                    heading_error = math.atan2(-self.current_road.nx, self.current_road.ny)
                elif pose is not None and self.memory.entry_yaw is not None and pose.frame == self.memory.frame:
                    delta = self.memory.entry_yaw - pose.yaw
                    heading_error = math.atan2(math.sin(delta), math.cos(delta))
                elif self.lane is not None and 0 <= now - self.lane[1] < self.c.lane_timeout:
                    data = self.lane[2]
                    heading_error = data[9] if data[10] > 0 else data[2]
                result, self.path_config = control_command(
                    control_scan, self.c, self.memory, hint, heading_error, self.direction_scores, pose)
                base_state, speed, steering, _ = result
                state = ('CREEP/' + base_state if speed > 0
                         and self.path_config.preview_distance < self.c.preview_distance else base_state)
                # Final gate: the published command and the blue RViz arc use
                # this same steering and must pass the same swept-body check.
                if speed > 0 and not path_clear(control_scan, steering, self.path_config):
                    state, speed, steering = 'STOP_PATH_BLOCKED', 0., 0.
                if self.current_road is not None and self.current_road.body_gap(
                        0., 0., 0., self.c.vehicle_length / 2 + self.c.clearance_margin,
                        self.c.vehicle_width / 2 + self.c.clearance_margin) <= 0:
                    state, speed, steering = 'STOP_ROAD_BOUNDARY', 0., 0.
                if state == 'STOP_FRONT_UNKNOWN':
                    self.memory.reset()
                # A scan can expire during an expensive search, too. Never
                # publish motion based only on the freshness check at entry.
                finished = time.monotonic()
                if not scan_fresh(self.scan, self.c,
                                  self.get_clock().now().nanoseconds * 1e-9, finished):
                    state, speed, steering = 'STOP_SCAN_EXPIRED', 0., 0.
                    self.memory.reset()
            current_lane = self.current_road.lane if self.current_road is not None else None
            if state != self.last_state or current_lane != self.last_lane or now - self.last_log >= 2.:
                distances = ''
                if self.scan is not None:
                    def value(sector):
                        return ('clear' if math.isinf(sector.distance) else f'{sector.distance:.2f}m') if sector.known(self.c) else 'unknown'
                    distances = (f' left={value(control_scan.left)} front={value(control_scan.front)}'
                                 f' right={value(control_scan.right)}')
                    def score(side):
                        v = self.direction_scores[side]
                        return 'unknown' if v is None or state.startswith(('WAIT_', 'STOP_SCAN_')) else f'{v:.2f}'
                    hold = {1: 'LEFT', -1: 'RIGHT', 0: 'NONE'}[self.memory.side]
                    phase = self.memory.phase
                    distances += f' prefer_left={score(1)} prefer_right={score(-1)} hold={hold} phase={phase}'
                    if self.current_road is not None:
                        distances += f' lane={self.current_road.lane} road_offset={self.current_road.offset:+.2f}m'
                    distances += f' path={self.path_config.preview_distance:.2f}m'
                    if state == 'STOP_SCAN_EXPIRED':
                        distances += (f' scan_age={ros_now-self.scan.stamp:.3f}s'
                                      f' receive_age={time.monotonic()-self.scan.received:.3f}s')
                    if state == 'STOP_PATH_BLOCKED' or state.startswith('CREEP/'):
                        obstacles, road, total = path_rejections(control_scan, self.c)
                        distances += f' long_rejected_obstacles={obstacles}/{total} long_rejected_road={road}/{total}'
                self.get_logger().info(f'{state}:{distances} speed={speed:.2f} steer={steering:.2f}')
                self.last_state, self.last_log = state, now
                self.last_lane = current_lane
            msg = AckermannDriveStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'
            msg.drive.speed, msg.drive.steering_angle = float(speed), float(steering)
            self.pub.publish(msg)
            self.publish_visual(state, speed, steering, now)

    rclpy.init(args=args)
    node = None
    try:
        node = ObjavoidNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            if rclpy.ok():
                msg = AckermannDriveStamped()
                msg.header.stamp = node.get_clock().now().to_msg()
                msg.header.frame_id = 'base_link'
                node.pub.publish(msg)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
