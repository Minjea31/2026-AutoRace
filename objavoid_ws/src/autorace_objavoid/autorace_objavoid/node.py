"""Reactive LiDAR avoidance with an entry-anchored three-lane road guard."""
from dataclasses import dataclass, replace
import colorsys
import math
import time
from .tracking import AvoidanceTarget

DISTANCE_EPS = 1e-6  # LaserScan float32 rounding at inclusive distance thresholds.


@dataclass(frozen=True)
class Config:
    front_half_angle_deg: float = 25.0
    side_limit_angle_deg: float = 90.0
    far_distance: float = 1.0
    near_distance: float = 0.5
    stop_distance: float = 0.05
    cruise_speed: float = 0.25
    avoid_speed: float = 0.20
    near_speed: float = 0.08
    far_steering: float = 0.38
    near_steering: float = 0.40
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
    preview_distance: float = 0.8
    creep_preview_distance: float = 0.25
    cluster_jump_distance: float = 0.12
    cluster_min_points: int = 3
    cluster_max_distance: float = 3.0
    object_max_extent: float = 1.0
    vehicle_length: float = 0.322
    vehicle_width: float = 0.220
    lidar_x: float = 0.103
    lidar_y: float = 0.0
    clearance_margin: float = 0.02
    direction_near_distance: float = 1.5
    direction_far_distance: float = 2.0
    direction_offset: float = 0.5
    direction_score_margin: float = 0.10
    direction_release_time: float = 1.5
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
        if (not 0 < self.direction_near_distance < self.direction_far_distance
                or self.direction_near_distance <= self.lidar_x + self.vehicle_length / 2 + self.clearance_margin
                or self.direction_offset <= self.vehicle_width / 2 + self.clearance_margin
                or not 0 <= self.direction_score_margin <= 1
                or self.direction_release_time <= 0):
            raise ValueError('invalid long-range direction settings')
        if (type(self.road_guard_enabled) is not bool or type(self.initial_lane) is not int
                or self.initial_lane not in (1, 2, 3)
                or self.lane_width <= self.vehicle_width + 2 * self.clearance_margin
                or not 0 <= self.lane_change_hysteresis < self.lane_width / 2
                or self.odom_timeout <= 0):
            raise ValueError('invalid road guard settings')
        if (min(self.scan_timeout, self.lane_timeout, self.publish_rate,
                self.wheelbase, self.preview_distance, self.creep_preview_distance, self.cluster_jump_distance,
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

    def heading_steering(self, c):
        angle = c.lane_gain * math.atan2(-self.nx, self.ny)
        return max(-c.max_lane_steering, min(c.max_lane_steering, angle))

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
    rays: tuple = ()
    objects: tuple = ()
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
    rays = []
    for i, value in enumerate(msg.ranges):
        angle = math.degrees(msg.angle_min + i * msg.angle_increment) + c.lidar_yaw_deg
        angle = (angle + 180) % 360 - 180
        a = math.radians(angle)
        co, si = math.cos(a), math.sin(a)
        valid = math.isfinite(value) and max(msg.range_min, 1e-6) <= value <= msg.range_max
        observed_range = (float(value) if valid else float(msg.range_max)
                          if c.inf_is_clear and value == math.inf else math.nan)
        rays.append((co, si, observed_range))
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
                   getattr(msg.header, 'frame_id', '') or 'laser_link', tuple(points), tuple(rays))


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
    objects = []
    boxes = []
    for group in compact_clusters(groups, c):
        xmin, xmax, ymin, ymax = cluster_bounds(group)
        x, y = (xmin + xmax) / 2, (ymin + ymax) / 2
        boxes.append((c.lidar_x + co * x - si * y, c.lidar_y + si * x + co * y,
                      (xmax - xmin) / 2, (ymax - ymin) / 2, a))
        for x, y in group:
            bx, by = c.lidar_x + co * x - si * y, c.lidar_y + si * x + co * y
            objects.append((bx, by))
            # Include adjacent/rear corners outside the old +/-90 degree sectors.
            if (-c.vehicle_length / 2 - .1 <= bx <= c.far_distance + c.vehicle_length / 2
                    and abs(by) > c.vehicle_width / 2 + c.clearance_margin):
                side = 1 if by > 0 else -1
                distances[side] = min(distances[side], math.hypot(x, y))
                counts[side] += 1
    return Sectors(Sector(distances[1], counts[1]), scan.front,
                   Sector(distances[-1], counts[-1]), scan.stamp, scan.received,
                   scan.frame_id, scan.points, scan.rays, tuple(objects), tuple(boxes), scan.road)


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
    for index, (x, y) in enumerate(preview_points(steering, c)):
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


def direction_free_space(scan, c):
    """Observed free-ray fraction through distant left/right rectangular corridors."""
    scores = {1: None, -1: None}
    if len(scan.rays) < 2:
        return scores
    ax, ay, _ = scan.rays[0]
    bx, by, _ = scan.rays[1]
    step = abs(math.atan2(ax * by - ay * bx, ax * bx + ay * by))
    if step < 1e-9:
        return scores
    half_length = c.vehicle_length / 2 + c.clearance_margin
    half_width = c.vehicle_width / 2 + c.clearance_margin
    xmin = c.direction_near_distance - half_length
    xmax = c.direction_far_distance + half_length
    origin_x, origin_y = c.lidar_x, c.lidar_y
    rays = scan.rays
    lane_centre = 0.
    if scan.road is not None:
        road = scan.road
        origin_x = road.ny * c.lidar_x - road.nx * c.lidar_y
        origin_y = road.nx * c.lidar_x + road.ny * c.lidar_y + road.offset
        rays = tuple((road.ny * dx - road.nx * dy, road.nx * dx + road.ny * dy, r)
                     for dx, dy, r in rays)
        lane_centre = (2 - road.lane) * c.lane_width
    for side in (1, -1):
        centre = lane_centre + side * c.direction_offset
        ymin, ymax = centre - half_width, centre + half_width
        if scan.road is not None and (ymin < -scan.road.half_width or ymax > scan.road.half_width):
            scores[side] = 0.
            continue
        angles = [math.atan2(y - origin_y, x - origin_x)
                  for x in (xmin, xmax) for y in (ymin, ymax)]
        expected = max(1, math.ceil((max(angles) - min(angles)) / step))
        total = valid = 0
        free = 0.
        for dx, dy, distance in rays:
            enter, leave = 0., math.inf
            for origin, unit, low, high in (
                    (origin_x, dx, xmin, xmax), (origin_y, dy, ymin, ymax)):
                if abs(unit) < 1e-9:
                    if not low <= origin <= high:
                        leave = -1.
                        break
                else:
                    a, b = (low - origin) / unit, (high - origin) / unit
                    enter, leave = max(enter, min(a, b)), min(leave, max(a, b))
            if leave <= enter:
                continue
            total += 1
            if math.isfinite(distance):
                valid += 1
                # Require visibility through the corridor: a hit inside it blocks
                # that ray, and a hit before it cannot clear the occluded space.
                free += float(distance >= leave)
        if valid >= c.min_valid_beams:
            scores[side] = free / max(expected, total)
    return scores


def avoidance_front_clear(scan, c):
    """Leave the turning phase beyond the entry distance, with distance hysteresis."""
    return (scan.front.known(c)
            and scan.front.distance >= c.far_distance + c.switch_margin - DISTANCE_EPS)


@dataclass
class AvoidanceMemory:
    side: int = 0
    clear_since: float | None = None
    turning: bool = False
    target: AvoidanceTarget | None = None

    def reset(self):
        self.side, self.clear_since = 0, None
        self.turning = False
        self.target = None

    def select(self, side, pose, c, road=None):
        if self.target is None or side != self.side or not self.turning:
            self.target = AvoidanceTarget.start(pose, side, c, road)
        self.side, self.clear_since = side, None
        self.turning = side in (1, -1)

    def observe(self, scan, c, now):
        if not self.side:
            return
        opposite = scan.right if self.side > 0 else scan.left
        rear = -c.vehicle_length / 2 - c.clearance_margin
        width = c.vehicle_width / 2 + c.clearance_margin
        alongside = any(rear <= x <= c.far_distance
                        and -c.direction_offset - width <= self.side * y <= width
                        for x, y in scan.objects)
        forward_clear = avoidance_front_clear(scan, c) and path_clear(scan, 0., c)
        passed = not self.turning and forward_clear and opposite.known(c) and not alongside
        if not passed:
            self.clear_since = None
        elif self.clear_since is None:
            self.clear_since = now
        elif now - self.clear_since >= c.direction_release_time:
            self.reset()


def cluster_markers(groups, frame, previous_count=0):
    from geometry_msgs.msg import Point
    from visualization_msgs.msg import Marker, MarkerArray

    result = MarkerArray()
    for index, points in enumerate(groups):
        color = colorsys.hsv_to_rgb((.53 + index * .61803398875) % 1., .65, 1.)
        def marker(namespace, kind):
            m = Marker()
            m.header.frame_id = frame
            m.ns, m.id, m.type = namespace, index, kind
            m.pose.orientation.w = 1.
            m.color.r, m.color.g, m.color.b = color
            m.color.a = 1.
            result.markers.append(m)
            return m
        def point(x, y, z=.06):
            return Point(x=float(x), y=float(y), z=float(z))
        cloud = marker('cluster_points', Marker.POINTS)
        cloud.scale.x = cloud.scale.y = .025
        cloud.points = [point(x, y) for x, y in points]
        xmin, xmax, ymin, ymax = cluster_bounds(points)
        box = marker('cluster_boxes', Marker.LINE_STRIP)
        box.scale.x = .012
        box.points = [point(x, y, .07) for x, y in (
            (xmin, ymin), (xmax, ymin), (xmax, ymax), (xmin, ymax), (xmin, ymin))]
        label = marker('cluster_labels', Marker.TEXT_VIEW_FACING)
        label.scale.z = .065
        label.pose.position = point(xmax + .09, (ymin + ymax) / 2, .18)
        nearest = min(math.hypot(x, y) for x, y in points)
        label.text = f'C{index + 1:02d}\n{nearest:.2f}m'
    # Remove disappeared clusters without clearing and redrawing every frame.
    for index in range(len(groups), previous_count):
        for namespace in ('cluster_points', 'cluster_boxes', 'cluster_labels'):
            m = Marker()
            m.header.frame_id = frame
            m.ns, m.id, m.action = namespace, index, Marker.DELETE
            result.markers.append(m)
    return result


def pick_side(s, c, previous):
    """Rank measured sides; with actual points the swept body decides clearance."""
    choices = {side: sector.distance for side, sector in ((1, s.left), (-1, s.right))
               if sector.known(c) and (s.points or s.boxes or s.road or sector.distance > c.near_distance + DISTANCE_EPS)}
    if not choices:
        return 0
    best = max(choices, key=choices.get)
    if previous in choices and choices[best] <= choices[previous] + c.switch_margin:
        return previous
    return best


def propose(s, c, previous=0, lane_steering=0.):
    """Positive steering is left. Invalid/unseen sectors are not empty space."""
    front, left, right = s.front, s.left, s.right
    if not front.known(c):
        return 'STOP_FRONT_UNKNOWN', 0., 0., 0
    if front.distance <= c.stop_distance + DISTANCE_EPS:
        return 'STOP_TOO_CLOSE', 0., 0., previous
    # A car alongside must not trigger a turn while the forward sector is clear.
    if front.distance >= c.far_distance - DISTANCE_EPS:
        steering = max(-c.max_lane_steering, min(c.max_lane_steering, lane_steering))
        return 'CLEAR', c.cruise_speed, steering, 0
    if front.distance <= c.near_distance + DISTANCE_EPS:
        side = pick_side(s, c, previous)
        if not side:
            return 'STOP_SURROUNDED', 0., 0., previous
        return ('NEAR_LEFT' if side > 0 else 'NEAR_RIGHT',
                c.near_speed, side * c.near_steering, side)
    if front.distance < c.far_distance:
        side = pick_side(s, c, previous)
        # A nearby car alongside can be behind the turning path. Its radial
        # distance must not be compared with the forward obstacle's distance.
        # Rank the available sides here; decide() checks the swept body next.
        if side:
            return ('FAR_LEFT' if side > 0 else 'FAR_RIGHT',
                    c.avoid_speed, side * c.far_steering, side)
        return 'APPROACH_FORWARD', c.avoid_speed, 0., previous
    steering = max(-c.max_lane_steering, min(c.max_lane_steering, lane_steering))
    return 'CLEAR', c.cruise_speed, steering, 0


def passing_command(s, c, side, heading_steering=0.):
    """Pass parallel to the lane without pulling back toward its centre."""
    heading_steering = max(-c.max_lane_steering, min(c.max_lane_steering, heading_steering))
    for angle in dict.fromkeys((heading_steering, heading_steering / 2, 0.)):
        if angle and not (s.left if angle > 0 else s.right).known(c):
            continue
        if not (s.points or s.boxes or s.road) or path_clear(s, angle, c):
            if angle != 0:
                return 'PASS_ALIGN', c.avoid_speed, angle, side
            return 'PASS_FORWARD', c.cruise_speed, 0., side
    return 'STOP_PATH_BLOCKED', 0., 0., side


def decide(s, c, previous=0, lane_steering=0., direction_scores=None,
           pass_steering=0.):
    """Prefer straight initially; finish an active avoidance before straightening."""
    state, speed, steering, side = propose(s, c, previous, lane_steering)
    if state in ('STOP_FRONT_UNKNOWN', 'STOP_TOO_CLOSE'):
        return state, speed, steering, side
    geometry = bool(s.points or s.boxes or s.road)
    if state == 'CLEAR' and previous in (1, -1):
        result = passing_command(s, c, previous, pass_steering)
        if result[1] > 0:
            return result
        state, speed, steering, side = 'PASS_PATH_BLOCKED', c.avoid_speed, 0., previous
    if state == 'CLEAR':
        if not geometry or path_clear(s, steering, c):
            return state, speed, steering, side
        state, speed, steering, side = 'LANE_PATH_BLOCKED', c.avoid_speed, 0., 0
    # Before avoidance starts, a clear straight corridor remains the priority.
    if geometry and path_clear(s, 0., c):
        if previous in (1, -1):
            result = passing_command(s, c, previous, pass_steering)
            if result[1] > 0:
                return result
        return 'PATH_FORWARD', c.cruise_speed, 0., 0
    if speed <= 0:
        return state, speed, steering, side
    if geometry:
        # Evaluate both sides against the front, adjacent cars and walls before
        # choosing. A side's radial minimum alone cannot describe turning room.
        candidates = []
        for direction in (1, -1):
            sector = s.left if direction > 0 else s.right
            if sector.known(c):
                limit = (c.near_steering if s.front.distance <= c.near_distance + DISTANCE_EPS
                         else c.far_steering)
                for magnitude in dict.fromkeys((limit, (limit + c.side_steering) / 2, c.side_steering)):
                    candidate = direction * magnitude
                    clearance = path_clearance(s, candidate, c)
                    if clearance > 0:
                        candidates.append((clearance, candidate, direction))
        if not candidates:
            return 'STOP_PATH_BLOCKED', 0., 0., previous
        # Keep the committed direction while at least one of its near paths is clear.
        pool = [p for p in candidates if p[2] == previous]
        preferred = pick_side(s, c, 0)
        if not pool:
            pool = candidates
            scores = direction_free_space(s, c) if direction_scores is None else direction_scores
            left = scores[1] if scores[1] is not None else 0.
            right = scores[-1] if scores[-1] is not None else 0.
            if abs(left - right) > c.direction_score_margin:
                distant_side = 1 if left > right else -1
                distant_pool = [p for p in candidates if p[2] == distant_side]
                if distant_pool:
                    pool = distant_pool
        best_gap = max(gap for gap, angle, direction in pool)
        # Among comparable gaps, keep a decisive turn during the active manoeuvre.
        tolerance = min(c.switch_margin, best_gap * .25)
        comparable = [p for p in pool if p[0] >= best_gap - tolerance]
        gap, candidate, direction = max(comparable, key=lambda p: (
            p[2] == preferred, abs(p[1]), p[0]))
        if direction == previous:
            return ('HOLD_LEFT' if direction > 0 else 'HOLD_RIGHT', speed, candidate, direction)
        if candidate == steering:
            return state, speed, steering, side
        return ('PATH_LEFT' if direction > 0 else 'PATH_RIGHT', speed, candidate, direction)
    if steering == 0:
        return state, speed, steering, side
    target, opposite = (s.left, s.right) if steering > 0 else (s.right, s.left)
    target_free = target.known(c) and target.distance > c.near_distance + DISTANCE_EPS
    if target_free:
        return state, speed, steering, side
    opposite_free = opposite.known(c) and opposite.distance > c.near_distance + DISTANCE_EPS
    if opposite_free:
        side = -1 if steering > 0 else 1
        # Correct an unsafe direction gently; do not restart a full-lock turn
        # simply because a neighbouring vehicle remains alongside.
        steering = side * min(abs(steering), c.side_steering)
        suffix = 'SIDE_LEFT' if side > 0 else 'SIDE_RIGHT'
        return f'{state}/{suffix}', min(speed, c.avoid_speed), steering, side
    if s.front.known(c) and s.front.distance > c.near_distance + DISTANCE_EPS:
        return f'{state}/SIDES_BLOCKED_FORWARD', min(speed, c.avoid_speed), 0., 0
    return 'STOP_NO_SIDE_SPACE', 0., 0., 0


def target_path(s, c, memory, pose):
    """Follow one fixed lateral goal; blocked alignment must not restart a turn."""
    if not s.front.known(c):
        return ('STOP_FRONT_UNKNOWN', 0., 0., memory.side), c
    if s.front.distance <= c.stop_distance + DISTANCE_EPS:
        return ('STOP_TOO_CLOSE', 0., 0., memory.side), c
    angle = memory.target.command(pose, c)
    speed = c.near_speed if s.front.distance <= c.near_distance else c.avoid_speed
    for distance in dict.fromkeys((c.preview_distance, min(c.preview_distance, c.creep_preview_distance))):
        checked = replace(c, preview_distance=distance)
        for steering in dict.fromkeys((angle, angle / 2)):
            if path_clear(s, steering, checked):
                state = 'TARGET_ALIGN' if steering * memory.side <= 0 else 'TARGET_SHIFT'
                return (state, min(speed, c.near_speed) if distance < c.preview_distance else speed,
                        steering, memory.side), checked
    return ('STOP_TARGET_BLOCKED', 0., 0., memory.side), c


def choose_path(s, c, previous=0, lane_steering=0., direction_scores=None,
                pass_steering=0.):
    """Try the full arc first, then a slower checked prefix without relaxing bounds."""
    result = decide(s, c, previous, lane_steering, direction_scores, pass_steering)
    if (result[0] != 'STOP_PATH_BLOCKED'
            or c.creep_preview_distance >= c.preview_distance):
        return result, c
    short = replace(c, preview_distance=c.creep_preview_distance)
    candidate = decide(s, short, previous, lane_steering, direction_scores, pass_steering)
    if candidate[1] > 0:
        state, speed, steering, side = candidate
        return (state, min(speed, c.near_speed), steering, side), short
    return result, c


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


def lane_pass_hint(data, c):
    """Heading only: lateral error must not trigger early return to lane centre."""
    if len(data) != 11 or data[0] != 1 or not all(math.isfinite(v) for v in data):
        return 0.
    heading = data[9] if data[10] > 0 else data[2]
    return max(-c.max_lane_steering, min(c.max_lane_steering, c.lane_gain * heading))


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


def command_path(steering, c, active=True):
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Path

    path = Path()
    path.header.frame_id = 'base_link'
    if not active:
        return path
    # Visualization uses latest TF (stamp=0), independently of command stamps.
    curvature = math.tan(steering) / c.wheelbase
    for index, (x, y) in enumerate(preview_points(steering, c)):
        pose = PoseStamped()
        pose.header = path.header
        pose.pose.position.x, pose.pose.position.y = x, y
        pose.pose.position.z = .04
        yaw = curvature * c.preview_distance * index / 24
        pose.pose.orientation.z = math.sin(yaw / 2)
        pose.pose.orientation.w = math.cos(yaw / 2)
        path.poses.append(pose)
    return path


def command_markers(scan, c, state, speed, steering, preview_steering=None, lane_points=(), road=None):
    from geometry_msgs.msg import Point
    from visualization_msgs.msg import Marker, MarkerArray

    result = MarkerArray()

    def marker(index, kind, color, frame='base_link', namespace='objavoid'):
        m = Marker()
        m.header.frame_id = frame
        # No short lifetime: updates replace the same IDs without expiring.
        m.frame_locked = kind != Marker.TEXT_VIEW_FACING
        m.ns, m.id, m.type = namespace, index, kind
        m.pose.orientation.w = 1.
        m.color.r, m.color.g, m.color.b, m.color.a = color
        result.markers.append(m)
        return m

    def point(x, y, z=.04):
        return Point(x=float(x), y=float(y), z=float(z))

    display_steering = steering if speed > 0 or preview_steering is None else preview_steering
    arrow = marker(0, Marker.ARROW, (.15, 1., .2, 1.) if speed > 0 else (.5, .5, .5, .8),
                   namespace='direction')
    arrow.scale.x, arrow.scale.y, arrow.scale.z = .04, .09, .12
    arrow.points = [point(0., 0., .12),
                    point(.65 * math.cos(display_steering), .65 * math.sin(display_steering), .12)]
    path = marker(1, Marker.LINE_STRIP, (0., .85, 1., 1.) if speed > 0 else (.45, .45, .45, .8),
                  namespace='command_path')
    path.scale.x = .025
    if speed > 0:
        path.points = [point(x, y) for x, y in preview_points(display_steering, c)]
    else:
        path.action = Marker.DELETE
    lane = marker(11, Marker.LINE_STRIP, (1., .85, .15, 1.), namespace='lane_center')
    lane.scale.x = .025
    lane.points = [point(x, y, .05) for x, y in lane_points]
    if not lane_points:
        lane.action = Marker.DELETE

    road = road if road is not None else scan.road if scan is not None else None
    limits = marker(12, Marker.LINE_LIST, (.9, .9, 1., .75), namespace='road_limits')
    limits.scale.x = .015
    lane_id = marker(13, Marker.TEXT_VIEW_FACING, (.9, .9, 1., 1.), namespace='lane_id')
    lane_id.pose.position = point(-.25, 0., .12)
    lane_id.scale.z = .065
    if road is None:
        limits.action = lane_id.action = Marker.DELETE
    else:
        for lateral in (-road.half_width, road.half_width):
            distance = -.4
            while distance < c.direction_far_distance:
                limits.points += [point(*road.point(distance, lateral), z=.03),
                                  point(*road.point(min(distance + .15, c.direction_far_distance), lateral), z=.03)]
                distance += .25
        lane_id.text = f'LANE{road.lane}\nEST'

    text = marker(2, Marker.TEXT_VIEW_FACING,
                  (.85, .9, .95, 1.) if speed > 0 else (1., .45, .3, 1.), namespace='status')
    text.pose.position = point(-.58, 0., .18)
    text.scale.z = .065
    def distance(sector):
        return ('clear' if math.isinf(sector.distance) else f'{sector.distance:.2f}m') if sector and sector.known(c) else '?'
    title = ('WAIT SCAN' if state.startswith('WAIT_') else 'STOP' if state.startswith('STOP_') else
             {'CLEAR': 'LANE', 'APPROACH_FORWARD': 'FORWARD', 'PASS_FORWARD': 'PASSING',
              'PASS_ALIGN': 'PASS ALIGN',
              'FAR_LEFT': 'AVOID LEFT', 'FAR_RIGHT': 'AVOID RIGHT',
              'NEAR_LEFT': 'NEAR LEFT', 'NEAR_RIGHT': 'NEAR RIGHT'}.get(state, state))
    text.text = f'{title}\n{speed:.2f}m/s  {math.degrees(steering):+.1f}deg'
    if speed <= 0:
        text.text += '\nNO MOTION PATH'
    details = marker(10, Marker.TEXT_VIEW_FACING, (.65, .7, .75, 1.), namespace='details')
    details.pose.position = point(-1.05, 0., .18)
    details.scale.z = .06
    details.text = (f'{state}\nlimits far={c.far_distance:.2f} '
                    f'near={c.near_distance:.2f} stop={c.stop_distance:.2f}m')

    car = marker(3, Marker.LINE_STRIP, (.9, .95, 1., 1.), namespace='vehicle')
    car.scale.x = .015
    car.points = [point(x, y) for x, y in (
        (-.161, -.11), (.161, -.11), (.161, .11), (-.161, .11), (-.161, -.11))]
    sectors = ((c.front_half_angle_deg, c.side_limit_angle_deg, scan.left if scan else None),
               (-c.front_half_angle_deg, c.front_half_angle_deg, scan.front if scan else None),
               (-c.side_limit_angle_deg, -c.front_half_angle_deg, scan.right if scan else None))
    for index, (start, end, sector) in enumerate(sectors, 4):
        known = sector is not None and sector.known(c)
        color = (.5, .5, .5, .6)
        radius = c.far_distance
        if known:
            radius = min(c.far_distance, sector.distance)
            color = ((1., .2, .1, .85) if sector.distance <= c.near_distance else
                     (1., .8, .1, .85) if sector.distance < c.far_distance else
                     (.2, .8, .3, .65))
        sector_marker = marker(index, Marker.LINE_STRIP, color,
                               scan.frame_id if scan else 'laser_link', namespace='sectors')
        sector_marker.scale.x = .012
        sector_marker.points = [point(0., 0.)]
        for i in range(21):
            angle = math.radians(start + (end - start) * i / 20 - c.lidar_yaw_deg)
            sector_marker.points.append(point(radius * math.cos(angle), radius * math.sin(angle)))
        sector_marker.points.append(point(0., 0.))
    labels = ((f'FRONT\n{distance(scan.front if scan else None)}', .38, 0.),
              (f'LEFT\n{distance(scan.left if scan else None)}', 0., .46),
              (f'RIGHT\n{distance(scan.right if scan else None)}', 0., -.46))
    for index, (label, x, y) in enumerate(labels, 7):
        axis_label = marker(index, Marker.TEXT_VIEW_FACING, (.55, .65, .75, 1.),
                            namespace='orientation')
        axis_label.pose.position = point(x, y, .2)
        axis_label.scale.z = .055
        axis_label.text = label
    return result


def main(args=None):
    import rclpy
    from rclpy.clock import Clock, ClockType
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
    from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import Float32MultiArray
    from ackermann_msgs.msg import AckermannDriveStamped
    from visualization_msgs.msg import MarkerArray
    from nav_msgs.msg import Path, Odometry

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
            self.previous_cluster_count = 0
            self.reset_clusters = True
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
            self.last_visual = -math.inf
            self.last_visual_state = None
            self.preview_steering = 0.
            self.path_config = self.c
            self.reset_visual = True
            self.pub = self.create_publisher(AckermannDriveStamped, command_topic, QoSProfile(depth=1))
            visual_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.visual_pub = self.create_publisher(MarkerArray, '/objavoid/markers', visual_qos)
            self.path_pub = self.create_publisher(Path, '/objavoid/path', visual_qos)
            self.cluster_pub = self.create_publisher(MarkerArray, '/objavoid/clusters', visual_qos)
            self.scan_sub = self.create_subscription(LaserScan, scan_topic, self.on_scan,
                                                     qos_profile_sensor_data)
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
            self.get_logger().info(f'Direction lookahead: {self.c.direction_near_distance:.2f}'
                                   f'~{self.c.direction_far_distance:.2f}m, '
                                   f'offset={self.c.direction_offset:.2f}m')
            self.get_logger().info(f'Road guard={self.c.road_guard_enabled}: '
                                   f'initial lane={self.c.initial_lane}, lane width={self.c.lane_width:.2f}m, '
                                   f'odometry={odom_topic}; waiting for entry lane reference')

        def publish_visual(self, state, speed, steering, now, scan):
            if state != self.last_visual_state or now - self.last_visual >= .1:
                if speed > 0:
                    self.preview_steering = steering
                lane_points = (lane_reference(self.lane[2]) if self.lane and
                               0 <= now - self.lane[1] < self.c.lane_timeout else ())
                markers = command_markers(scan, self.path_config, state, speed, steering,
                                          self.preview_steering, lane_points, self.current_road)
                if self.reset_visual:
                    from visualization_msgs.msg import Marker
                    reset = Marker()
                    reset.header.frame_id = 'base_link'
                    reset.pose.orientation.w = 1.
                    reset.action = Marker.DELETEALL
                    markers.markers.insert(0, reset)
                    self.reset_visual = False
                self.visual_pub.publish(markers)
                self.path_pub.publish(command_path(steering, self.path_config, active=speed > 0))
                fresh = scan_fresh(self.scan, self.c,
                                   self.get_clock().now().nanoseconds * 1e-9, now)
                groups = compact_clusters(self.clusters, self.c) if fresh else ()
                clusters = cluster_markers(groups, self.cluster_frame, self.previous_cluster_count)
                if self.reset_clusters:
                    from visualization_msgs.msg import Marker
                    reset = Marker()
                    reset.header.frame_id = self.cluster_frame
                    reset.pose.orientation.w = 1.
                    reset.action = Marker.DELETEALL
                    clusters.markers.insert(0, reset)
                    self.reset_clusters = False
                self.cluster_pub.publish(clusters)
                self.previous_cluster_count = len(groups)
                self.last_visual, self.last_visual_state = now, state

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
                self.preview_steering = 0.
            self.last_ros = ros_now
            self.current_road, road_error = self.road_context(ros_now, now)
            speed = steering = 0.
            self.path_config = self.c
            control_scan = self.scan
            if not self.get_parameter('manual_active').value:
                self.memory.reset()
                self.last_state = 'IDLE'
                self.publish_visual('IDLE', 0., 0., now, None)
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
                hint = (self.lane[0] if self.lane and
                        0 <= now - self.lane[1] < self.c.lane_timeout else 0.)
                pass_hint = (lane_pass_hint(self.lane[2], self.c) if self.lane and
                             0 <= now - self.lane[1] < self.c.lane_timeout else 0.)
                if self.current_road is not None:
                    hint = self.current_road.lane_steering(self.c)
                    pass_hint = self.current_road.heading_steering(self.c)
                control_scan = object_sectors(self.scan, self.clusters, self.c)
                control_scan = replace(control_scan, road=self.current_road)
                self.direction_scores = direction_free_space(control_scan, self.c)
                self.memory.observe(control_scan, self.c, now)
                tracking = self.memory.turning and self.memory.target is not None
                if tracking and (not odom_fresh(self.odom, self.c, ros_now, now)
                                 or self.odom.frame != self.memory.target.frame):
                    result = ('STOP_TARGET_ODOM', 0., 0., self.memory.side)
                else:
                    if tracking and self.memory.target.reached(self.odom, self.c):
                        self.memory.turning = False
                        tracking = False
                    if tracking:
                        result, self.path_config = target_path(control_scan, self.c, self.memory, self.odom)
                    else:
                        result, self.path_config = choose_path(
                            control_scan, self.c, self.memory.side, hint, self.direction_scores,
                            pass_steering=pass_hint)
                base_state, speed, steering, selected = result
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
                if state in ('STOP_SURROUNDED', 'STOP_FRONT_UNKNOWN'):
                    self.memory.reset()
                elif (speed > 0 and steering != 0
                      and base_state not in ('CLEAR', 'PASS_ALIGN', 'TARGET_SHIFT', 'TARGET_ALIGN')):
                    pose = self.odom if odom_fresh(self.odom, self.c, ros_now, now) else None
                    if pose is None:
                        state, speed, steering = 'STOP_TARGET_ODOM', 0., 0.
                    else:
                        self.memory.select(selected, pose, self.c, self.current_road)
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
                    phase = 'TURN' if self.memory.turning else 'PASS' if self.memory.side else 'NONE'
                    distances += f' far_left={score(1)} far_right={score(-1)} hold={hold} phase={phase}'
                    if self.current_road is not None:
                        distances += f' lane={self.current_road.lane} road_offset={self.current_road.offset:+.2f}m'
                    distances += f' path={self.path_config.preview_distance:.2f}m'
                    if self.memory.target is not None and self.odom is not None:
                        error, heading = self.memory.target.errors(self.odom)
                        distances += f' target_error={error:+.2f}m heading={math.degrees(heading):+.1f}deg'
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
            visual_scan = control_scan if not state.startswith(('WAIT_', 'STOP_SCAN_')) else None
            self.publish_visual(state, speed, steering, now, visual_scan)

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
