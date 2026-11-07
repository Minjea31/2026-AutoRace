"""ROS-independent geometry. Coordinates are metres in the current base frame.

This is a local path preview, not a controller or a persistent obstacle tracker.
Optional compact-cluster filtering affects obstacles, not raw visibility rays.
All retained returns participate in collision checks, even isolated/noisy points.
"""
from dataclasses import dataclass, replace
from copy import copy
import heapq
import math
import time
from collections import defaultdict
from .regions import rectangles_intersect


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass
class Settings:
    candidate_roi_enabled: bool = False
    candidate_roi_forward_min: float = 0.0
    candidate_roi_forward_max: float = 3.0
    candidate_roi_right_min: float = 0.0
    candidate_roi_right_max: float = 2.0
    cluster_distance: float = 0.12
    cluster_min_points: int = 3
    max_range: float = 3.0
    max_cluster_size: float = 0.70
    min_cluster_size: float = 0.08
    obstacle_min_cluster_size: float = 0.0  # compact sign filtering; 0 disables
    small_cluster_filter_max_distance: float = 0.0  # 0: no range restriction
    row_yaw: float = 0.0
    row_tolerance: float = 0.16
    max_gap: float = 1.10
    min_gap: float = 0.0
    vehicle_length: float = 0.322
    vehicle_width: float = 0.220
    wheelbase: float = 0.200
    max_steering: float = 0.408
    min_turn_radius: float = 0.0
    preferred_turn_radius: float = 0.0  # gentle-curve preference; tight turns remain possible
    max_steering_paths_only: bool = False
    max_steering_on_curves: bool = False
    steering_gain: float = 1.0
    # Actual body curvature / commanded bicycle curvature, measured from IMU
    # and scan motion. One preserves ideal Ackermann geometry.
    steering_response_scale: float = 1.0
    reverse_steering_response_scale: float = 1.0
    hold_entry_path: bool = False
    validate_parking_space: bool = False
    require_steering_calibration: bool = False
    steering_command_gain_expected: float = 1.92
    steering_command_deadband_expected: float = 0.23
    steering_command_limit_expected: float = 0.35
    prefer_straight_entry: bool = False
    max_planning_turn_radius: float = 0.0
    entry_outward_penalty: float = 0.0
    curvature_feedforward: bool = False
    steering_preview_time: float = 0.0
    visualization_period: float = 0.10
    block_occupied_slots: bool = False
    exit_to_grid_edge: bool = True
    exit_grid_margin: float = 0.10
    exit_lane_tolerance: float = 0.10
    exit_arrival_yaw_tolerance: float = 0.20
    exit_planning_timeout: float = 0.0
    exit_start_speed: float = 0.0
    exit_steering_settle_time: float = 0.0
    arrival_use_rear_axle: bool = False
    rear_axle_offset_x: float = -0.10
    rear_axle_offset_y: float = 0.0
    slot_map_enabled: bool = False
    slot_initial_right_only: bool = True
    slot_columns: int = 3
    slot_rows: int = 2
    slot_width: float = 0.50
    slot_depth: float = 0.60
    slot_border: float = 0.02
    slot_match_tolerance: float = 0.18
    slot_fit_max_error: float = 0.10
    slot_ambiguity_margin: float = 0.02
    automatic_target_selection: bool = False
    pair_fallback_enabled: bool = True
    path_tracking_margin: float = 0.0
    max_entry_path_length: float = 0.0
    boundary_tolerance: float = 0.12
    target_heading_step: float = math.pi / 36
    center_stop_tolerance: float = 0.03
    pair_stop_tolerance: float = 0.05
    auto_exit_after_parking: bool = True
    enclosed_space_detection: bool = False
    continuous_replanning: bool = False
    require_complete_entry_path: bool = False
    control_preview_time: float = 0.4
    enclosure_min_objects: int = 3
    enclosure_confirm_objects: int = 5
    enclosure_grid_resolution: float = 0.05
    enclosure_support_radius: float = 1.0
    goal_update_gain: float = 0.25
    goal_update_step: float = 0.08
    max_turn_arc_length: float = 0.4
    # max_steering is the equivalent bicycle angle; the inner wheel turns more.
    # Zero disables the optional inner-wheel bound for other vehicle profiles.
    max_inner_steering: float = 0.0
    steering_track: float = 0.172
    goal_from_observed_front: bool = False
    clearance: float = 0.10
    initial_clear_radius: float = 0.30
    assume_unobserved_free: bool = False
    resolution: float = 0.06
    yaw_resolution: float = math.pi / 18
    step: float = 0.12
    collision_step: float = 0.02
    goal_tolerance: float = 0.09
    goal_yaw_tolerance: float = math.pi / 9
    require_goal_heading: bool = True
    max_expansions: int = 10000
    allow_reverse: bool = False
    recovery_uses_memory_map: bool = False
    recovery_min_distance: float = 0.0
    recovery_reverse_distance: float = 0.25
    recovery_on_front_obstacle_only: bool = False
    recovery_on_path_failure: bool = False
    recovery_failure_count: int = 3
    recovery_total_distance: float = 0.30
    recovery_front_clearance: float = 0.10
    recovery_left_when_goal_right: bool = False
    recovery_straight_distance: float = 0.10
    blind_reverse_distance: float = 0.0  # explicitly cleared, mission-total reverse budget
    recovery_speed: float = 0.12
    max_recovery_attempts: int = 3
    planning_timeout: float = 1.0

    @property
    def turn_radius(self):
        radius = self.wheelbase / math.tan(self.max_steering)
        if self.max_inner_steering > 0:
            radius = max(radius, self.wheelbase / math.tan(self.max_inner_steering)
                         + self.steering_track / 2)
        return max(self.min_turn_radius,radius/self.steering_response_scale)

    @property
    def steering_limit(self):
        # A larger measured turning radius must not reduce the actuator command
        # cap as well: doing so would count the weak response twice.
        if self.require_steering_calibration:
            inner=self.steering_command_gain_expected*self.steering_command_limit_expected-self.steering_command_deadband_expected
            physical_radius=self.wheelbase/math.tan(inner)+self.steering_track/2
            return min(self.max_steering,math.atan(self.wheelbase/physical_radius))
        return math.atan(self.wheelbase / (self.turn_radius*self.steering_response_scale))


class Activation:
    def __init__(self, required=3, timeout=0.5):
        self.required, self.timeout = required, timeout
        self.reset()

    def reset(self):
        self.active, self.count, self.last = False, 0, None
        self.generation = 0

    def restart(self, now):
        self.active, self.count, self.last = True, 0, now
        self.generation += 1

    def update(self, detected, now):
        # Active missions stay latched; the ROS caller controls restart timing.
        if self.active:
            return True
        if self.last is None or not 0 <= now - self.last <= self.timeout:
            self.count = 0
        self.last = now
        self.count = self.count + 1 if detected else 0
        if self.count >= self.required:
            self.restart(now)
        return self.active

    def expire(self, now):
        if self.last is not None and not 0 <= now - self.last <= self.timeout:
            self.count = 0


@dataclass
class Cluster:
    points: list
    track_id: object = None

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
        self.ignored_small_clusters = self.ignored_small_points = 0
        sx, sy, yaw = sensor_pose
        for i, r in enumerate(ranges):
            if math.isfinite(r) and max(0.001, range_min) <= r <= min(range_max, max_range):
                angle = angle_min + i * angle_increment + yaw
                self.points.append((sx + r * math.cos(angle), sy + r * math.sin(angle)))

    def candidate_clusters(self, cfg):
        return cluster_points(self.points,cfg.cluster_distance,cfg.cluster_min_points)

    def filtered_obstacles(self, cfg):
        """Exclude compact sign-like clusters without inventing visibility.

        Raw ranges remain unchanged: filtering a sign does not measure the
        space hidden behind it. Localization can still use the original points.
        Isolated returns below cluster_min_points remain collision obstacles.
        """
        if cfg.obstacle_min_cluster_size <= 0:
            return self
        groups = cluster_points(self.points, cfg.cluster_distance, cfg.cluster_min_points)
        ignored = [g for g in groups if max(g.bounds[1]-g.bounds[0], g.bounds[3]-g.bounds[2])
                   < cfg.obstacle_min_cluster_size
                   and (cfg.small_cluster_filter_max_distance <= 0
                        or math.hypot(*g.center) <= cfg.small_cluster_filter_max_distance)]
        if not ignored:
            return self
        excluded = {tuple(point) for group in ignored for point in group.points}
        filtered = copy(self)
        filtered.points = [point for point in self.points if tuple(point) not in excluded]
        filtered.ignored_small_clusters += len(ignored)
        filtered.ignored_small_points += len(self.points)-len(filtered.points)
        return filtered

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
    support_indices: tuple = ()
    support_ids: tuple = ()
    confirmed: bool = False
    # Fixed mission-frame boundary orientation and (entrance,front,right,left) planes.
    parking_grid: object = None
    slot_id: int = -1
    bay_yaw: object = None
    boundary_bounds: tuple = ()
    kind: str = 'legacy'
    heading_options: tuple = ()
    stop_tolerance: float = 0.0


def find_gaps(clusters, cfg, origin=(0., 0., 0.), diagnostics=None):
    """Adjacent car-sized groups on the mission's initial right side.

    A gap is a local LiDAR candidate, not recognition of a painted parking bay.
    Long walls remain collision obstacles but cannot form parking candidates.
    origin is the current base pose in the mission-start frame. Keep the side
    fixed in that frame even after translating or turning into the parking row.
    """
    c, s = math.cos(cfg.row_yaw), math.sin(cfg.row_yaw)
    origin_c, origin_s = math.cos(origin[2]), math.sin(origin[2])
    rejected = defaultdict(int)
    projected = []
    for i, cluster in enumerate(clusters):
        x, y = cluster.center
        if origin[1] + origin_s * x + origin_c * y >= 0:
            rejected['not_right_side'] += 1
            continue
        pts = [(c * x + s * y, -s * x + c * y) for x, y in cluster.points]
        xs, ys = zip(*pts)
        bounds = min(xs), max(xs), min(ys), max(ys)
        size = max(bounds[1] - bounds[0], bounds[3] - bounds[2])
        if cfg.min_cluster_size <= size <= cfg.max_cluster_size:
            projected.append((i, bounds))
        else:
            rejected['object_too_small' if size < cfg.min_cluster_size else 'object_too_large'] += 1
    projected.sort(key=lambda item: item[1][0])
    gaps = []
    for k, (i, a) in enumerate(projected):
        for j, b in projected[k + 1:]:
            if clusters[i].track_id is not None and clusters[i].track_id == clusters[j].track_id:
                rejected['same_object'] += 1
                continue
            width = b[0] - a[1]
            minimum = max(cfg.min_gap,cfg.vehicle_width + 2*cfg.clearance)
            if width < minimum-1e-9 or width > cfg.max_gap+1e-9:
                rejected['gap_too_small' if width < minimum-1e-9 else 'gap_too_large'] += 1
                continue
            va, vb = (a[2] + a[3]) / 2, (b[2] + b[3]) / 2
            if abs(va - vb) > cfg.row_tolerance + 1e-9:
                rejected['not_aligned'] += 1
                continue
            if any(a[1] < (d[0] + d[1]) / 2 < b[0] and
                   abs((d[2] + d[3]) / 2 - (va + vb) / 2) <= cfg.row_tolerance
                   for n, d in projected if n not in (i, j)):
                rejected['intervening_object'] += 1
                continue
            u, v = (a[1] + b[0]) / 2, (va + vb) / 2
            if cfg.goal_from_observed_front:
                # From the aisle, LiDAR often sees only the cars' front faces.
                # Their measured centre is not the bay centre. Put the rear of
                # our perpendicular footprint behind both observed front edges.
                # This is a target hypothesis: every driven prefix still has to
                # pass the measured-obstacle AND observed-space checks below.
                v = min(v, min(a[3], b[3]) - cfg.vehicle_length / 2 - cfg.clearance)
            yaw = cfg.row_yaw - math.pi / 2
            gaps.append(Gap(c * u - s * v, s * u + c * v, wrap(yaw), width, i, j))
    if diagnostics is not None:
        diagnostics['pair_filter_counts'] = dict(rejected)
        diagnostics['effective_min_gap_m'] = max(cfg.min_gap,cfg.vehicle_width+2*cfg.clearance)
    return sorted(gaps, key=lambda g: math.hypot(g.x, g.y))


class CollisionChecker:
    def __init__(self, scan, cfg, known_free_poses=(), initial_clear_center=(0., 0.)):
        scan = scan.filtered_obstacles(cfg)
        self.scan, self.cfg = scan, cfg
        self.last_block = ''
        self.last_block_detail = None
        self.occupied_slots=getattr(scan,'occupied_slots',()) if cfg.block_occupied_slots else ()
        self.initial_clear_center = initial_clear_center
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

    def occupied_free(self,pose):
        for region in self.occupied_slots:
            if rectangles_intersect(pose,(self.half_length,self.half_width),region['pose'],region['half_size']):
                self.last_block='occupied_slot'
                self.last_block_detail={'kind':'occupied_slot','slot_id':region['id'],
                    'checked_pose':list(pose),'cell_center':list(region['pose'][:2])}
                return False
        return True

    def free(self, pose, allow_unknown=False):
        self.last_block = ''
        self.last_block_detail = None
        if not self.occupied_free(pose):return False
        x, y, yaw = pose
        if math.hypot(x, y) > self.cfg.max_range:
            self.last_block = 'outside_range'
            return False
        c, s = math.cos(yaw), math.sin(yaw)
        for ix in range(math.floor((x - self.radius) / 0.2), math.floor((x + self.radius) / 0.2) + 1):
            for iy in range(math.floor((y - self.radius) / 0.2), math.floor((y + self.radius) / 0.2) + 1):
                for px, py in self.cells[ix, iy]:
                    dx, dy = px - x, py - y
                    if abs(c * dx + s * dy) <= self.half_length and abs(-s * dx + c * dy) <= self.half_width:
                        self.last_block = 'obstacle'
                        self.last_block_detail={'kind':'obstacle','point':[px,py],'checked_pose':list(pose)}
                        return False
        if allow_unknown or self.cfg.assume_unobserved_free:
            return True  # measured obstacle checks above always take priority
        for bx, by in self.samples:
            px, py = x + c * bx - s * by, y + s * bx + c * by
            # A configurable clear startup bubble covers the close rear sector
            # masked by the sensor. Measured obstacles are still checked above.
            if math.hypot(px-self.initial_clear_center[0], py-self.initial_clear_center[1]) <= self.cfg.initial_clear_radius:
                continue
            if not self.scan.visible(px, py) and not self.previously_free(px, py):
                self.last_block = 'unknown_space'
                return False
        return True


class RecoveryCollisionChecker(CollisionChecker):
    """Restore an already violated margin by straight retreat, away from points.

    Only initially close points in the front half may use this exception.
    All other points retain the full margin; actual body collision is forbidden.
    """
    def __init__(self,scan,cfg,*args):
        scan=scan.filtered_obstacles(cfg)
        cfg=replace(cfg,obstacle_min_cluster_size=0.)  # Do not re-filter fragments after masking the escape points.
        self.original=CollisionChecker(scan,cfg,*args)
        self.physical=CollisionChecker(scan,replace(cfg,clearance=0.),*args)
        self.escape_points=[]
        if cfg.recovery_uses_memory_map and not cfg.recovery_left_when_goal_right and self.physical.free((0.,0.,0.),allow_unknown=True):
            self.escape_points=[p for p in scan.points if p[0]>=0 and
                abs(p[0])<=cfg.vehicle_length/2+cfg.clearance and abs(p[1])<=cfg.vehicle_width/2+cfg.clearance]
        source=copy(scan);ignored=set(self.escape_points)
        source.points=[p for p in scan.points if p not in ignored]
        super().__init__(source,cfg,*args)

    def free(self,pose,allow_unknown=False):
        if not self.escape_points:return super().free(pose,allow_unknown)
        x,y,yaw=pose
        if x>1e-8 or abs(y)>.02 or abs(yaw)>.05:
            result=self.original.free(pose,allow_unknown)
            self.last_block,self.last_block_detail=self.original.last_block,self.original.last_block_detail
            return result
        if not self.physical.free(pose,allow_unknown):
            self.last_block,self.last_block_detail=self.physical.last_block,self.physical.last_block_detail
            return False
        c,s=math.cos(yaw),math.sin(yaw)
        def distance(px,py):
            return math.hypot(max(abs(px)-self.cfg.vehicle_length/2,0.),max(abs(py)-self.cfg.vehicle_width/2,0.))
        for px,py in self.escape_points:
            dx,dy=px-x,py-y
            if distance(c*dx+s*dy,-s*dx+c*dy)+1e-8<distance(px,py):
                self.last_block='obstacle'
                self.last_block_detail={'kind':'margin_escape_toward_obstacle','point':[px,py],'checked_pose':list(pose)}
                return False
        return super().free(pose,allow_unknown)


def front_clearance(scan, cfg):
    """Nearest retained return ahead of the front bumper in the vehicle corridor."""
    scan = scan.filtered_obstacles(cfg)
    half_width = cfg.vehicle_width/2+cfg.clearance
    return min((max(0., x-cfg.vehicle_length/2) for x, y in scan.points
                if x > 0 and abs(y) <= half_width), default=None)


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
    minimum_radius = cfg.turn_radius
    candidates = []
    radii = {minimum_radius * factor for factor in (1., 1.05, 1.1, 1.25, 1.5)}
    if cfg.preferred_turn_radius > 0:
        preferred = max(minimum_radius, cfg.preferred_turn_radius)
        radii.update(preferred * factor for factor in (1., 1.25, 1.5))
    if cfg.max_steering_paths_only:
        radii = {minimum_radius}
    for radius in sorted(radii):
        if cfg.max_planning_turn_radius>0 and radius>cfg.max_planning_turn_radius+1e-9:continue
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


def candidate_cost(parts, cfg):
    length = sum(distance for distance, _ in parts)
    effort = sum(distance * (cfg.preferred_turn_radius*curvature)**2
                 for distance, curvature in parts)
    return length + .2*effort


def outward_excursion(parts,side):
    pose=(0.,0.,0.);excursion=0.
    for length,curvature in parts:
        if abs(curvature)>1e-10:
            end_yaw=pose[2]+length*curvature
            for n in range(math.floor(min(pose[2],end_yaw)/math.pi),math.ceil(max(pose[2],end_yaw)/math.pi)+1):
                distance=(n*math.pi-pose[2])/curvature
                if 0.<distance<length:excursion=max(excursion,-side*advance(pose,distance,curvature)[1])
        pose=advance(pose,length,curvature)
        excursion=max(excursion,-side*pose[1])
    return excursion


def position_arc_candidates(goal,cfg):
    """Stop on the arc within the requested center tolerance, at any safe yaw.

    Requiring an exact tangent/heading used to replace an 8mm center residual
    with a full CSC detour. The sampled body must still be collision-free.
    """
    candidates=[]
    radius=cfg.turn_radius
    side=1 if goal.y>=0 else -1
    bend=abs(goal.y)
    prefixes={0.}
    if bend<=2*radius:
        theta=math.acos(max(-1.,min(1.,1-bend/radius)))
        prefixes.add(max(0.,goal.x-radius*math.sin(theta)))
    for prefix in prefixes:
        x=goal.x-prefix
        if x<0:continue
        theta=math.atan2(x,radius-bend)
        if not 0<theta<math.pi:continue
        parts=[(prefix,0.),(radius*theta,side/radius)]
        pose=advance((prefix,0.,0.),radius*theta,side/radius)
        if math.hypot(pose[0]-goal.x,pose[1]-goal.y)<=cfg.goal_tolerance:
            candidates.append((prefix+radius*theta,parts))
    return candidates


def parking_candidates(goal, cfg, include_arcs=False):
    """Allow oblique arrival before trying a tight perpendicular parking turn."""
    headings = list(goal.heading_options) if cfg.require_goal_heading and goal.heading_options else [goal.yaw]
    if not cfg.require_goal_heading:
        bearing = math.atan2(goal.y, goal.x)
        headings = [wrap(2*bearing), bearing, goal.yaw]
    candidates, seen = [], set()
    if goal.kind in ('enclosed_center','pair_midpoint') and (not cfg.require_goal_heading or cfg.goal_yaw_tolerance>=math.pi-1e-6):
        candidates.extend(position_arc_candidates(goal,cfg))
    for heading in headings:
        key = round(wrap(heading), 8)
        if key in seen:
            continue
        seen.add(key)
        target = replace(goal, yaw=heading)
        candidates.extend(straight_turn_candidates(target, cfg))
        if include_arcs:
            candidates.extend(arc_candidates(target, cfg))
    if cfg.entry_outward_penalty>0:
        side=1 if goal.y>=0 else -1
        candidates.sort(key=lambda item:candidate_cost(item[1],cfg)+cfg.entry_outward_penalty*outward_excursion(item[1],side))
    elif cfg.preferred_turn_radius > 0 or goal.kind in ('enclosed_center','pair_midpoint'):
        candidates.sort(key=lambda item: ((sum(length>1e-8 and abs(k)>1e-8 for length,k in item[1])>1) if cfg.prefer_straight_entry else False, candidate_cost(item[1], cfg)))
    return candidates


def sample_candidate(checker, parts, cfg):
    """Return only the collision-checked, observed prefix of an arc sequence."""
    if cfg.max_steering_paths_only:
        run, previous = 0., 0.
        for length, curvature in parts:
            if length <= 1e-9:
                continue
            if abs(curvature) < 1e-9:
                run, previous = 0., 0.
                continue
            if abs(abs(curvature)-1/cfg.turn_radius) > 1e-7:
                return [], False
            run = run+length if abs(curvature-previous) < 1e-8 else length
            previous = curvature
            if run > cfg.max_turn_arc_length+1e-8:
                return [], False
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
    minimum_radius = cfg.turn_radius
    candidates = []
    radii = {minimum_radius * factor for factor in (1., 1.25, 1.5)}
    if cfg.preferred_turn_radius > 0:
        preferred = max(minimum_radius, cfg.preferred_turn_radius)
        radii.update(preferred * factor for factor in (1., 1.25, 1.5))
    if cfg.max_steering_paths_only:
        radii = {minimum_radius}
    semantic=goal.kind in ('enclosed_center','pair_midpoint')
    if semantic:
        # All safe terminal headings already supply broad alternatives. Avoid
        # multiplying each by 15 radius/prefix combinations on the robot CPU.
        radii={minimum_radius}
        if cfg.preferred_turn_radius>0:radii.add(max(minimum_radius,cfg.preferred_turn_radius))
    for radius in sorted(radii):
        if cfg.max_planning_turn_radius>0 and radius>cfg.max_planning_turn_radius+1e-9:continue
        prefixes=((0.0,) if semantic else (0.0,0.12,0.24,0.36,0.48))
        for prefix in prefixes:
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


def plan_path(scan, goal, cfg, known_free_poses=(), initial_clear_center=(0., 0.)):
    """Bounded hybrid A*: Ackermann arcs with swept rectangular footprint.

    Returns densely sampled (x,y,heading) poses, or [] if no observed path fits.
    Forward-only by default; never connects the endpoint by a straight shortcut.
    """
    checker = CollisionChecker(scan, cfg, known_free_poses, initial_clear_center)
    start = (0.0, 0.0, 0.0)
    headings = goal.heading_options or (goal.yaw,)
    def heading_error(yaw):
        return min(abs(wrap(yaw-a)) for a in headings)
    def at_goal(pose):
        return (math.hypot(pose[0]-goal.x,pose[1]-goal.y)<=cfg.goal_tolerance
                and (not cfg.require_goal_heading or heading_error(pose[2])<=cfg.goal_yaw_tolerance))
    semantic = goal.kind in ('enclosed_center','pair_midpoint')
    path_limit=cfg.max_entry_path_length if semantic and cfg.max_entry_path_length>0 else math.inf
    if not checker.free(start) or (cfg.require_goal_heading and not any(checker.free((goal.x,goal.y,a)) for a in headings)):
        return []
    deadline = time.monotonic() + cfg.planning_timeout
    analytic_deadline = (time.monotonic()+cfg.planning_timeout*.45
                         if goal.kind in ('enclosed_center','pair_midpoint') else deadline)
    candidates=parking_candidates(goal,cfg,include_arcs=True)
    blocked_straight=None
    if semantic and cfg.prefer_straight_entry:
        maximum=min(cfg.max_range,max((parts[0][0] for _,parts in candidates if parts and abs(parts[0][1])<1e-10),default=0.))
        for i in range(1,math.ceil(maximum/cfg.collision_step)+1):
            distance=min(maximum,i*cfg.collision_step)
            if not checker.free((distance,0.,0.)):
                blocked_straight=distance
                break
    for length, parts in candidates:
        if length>path_limit:continue
        # Reject only at a proven colliding pose on the shared initial straight.
        # This avoids repeatedly sampling the same blocked prefix for dozens of
        # one-turn headings before a feasible two-arc approach gets any time.
        if (blocked_straight is not None and parts and abs(parts[0][1])<1e-10
                and parts[0][0]>=blocked_straight-1e-10):continue
        path, valid = sample_candidate(checker, parts, cfg)
        if valid:
            return path
        if time.monotonic() > analytic_deadline:
            break
    def key(pose, curvature=0., run=0.):
        x, y, yaw = pose
        position = (round(x/cfg.resolution), round(y/cfg.resolution), round(wrap(yaw)/cfg.yaw_resolution))
        if cfg.max_steering_paths_only:
            return position + (round(curvature*cfg.turn_radius), round(run/cfg.step))
        return position
    def heuristic(pose):
        return math.hypot(pose[0] - goal.x, pose[1] - goal.y) + (0.15 * heading_error(pose[2]) if cfg.require_goal_heading else 0.)
    nodes = [(start, -1, [])]
    turn_runs = [(0., 0.)]
    lengths=[0.]
    costs = {key(start): 0.0}
    queue = [(heuristic(start), 0.0, 0)]
    radius = cfg.turn_radius
    curvatures = (-1/radius, 0., 1/radius) if cfg.max_steering_paths_only else (-1/radius, -.5/radius, 0., .5/radius, 1/radius)
    if cfg.max_planning_turn_radius>0:
        curvatures=tuple(k for k in curvatures if abs(k)<1e-10 or 1/abs(k)<=cfg.max_planning_turn_radius+1e-9)
    directions = (1, -1) if cfg.allow_reverse else (1,)
    expanded = 0
    while queue and expanded < cfg.max_expansions and time.monotonic() < deadline:
        _, cost, index = heapq.heappop(queue)
        pose, _, _ = nodes[index]
        previous_curvature, arc_run = turn_runs[index]
        if cost > costs.get(key(pose, previous_curvature, arc_run), math.inf) + 1e-9:
            continue
        expanded += 1
        if at_goal(pose):
            segments = []
            while index > 0:
                _, parent, segment = nodes[index]
                segments.append(segment)
                index = parent
            return [start] + [p for seg in reversed(segments) for p in seg]
        # A fixed 12 cm primitive can skip over a goal only 6 cm away. Short
        # Ackermann primitives retain swept collision checks near the endpoint.
        step = min(cfg.step,max(cfg.collision_step,math.hypot(pose[0]-goal.x,pose[1]-goal.y)))
        samples = max(1,math.ceil(step/cfg.collision_step))
        if lengths[index]+step>path_limit:continue
        for direction in directions:
            for curvature in curvatures:
                run = 0. if abs(curvature) < 1e-9 else (arc_run+step if abs(curvature-previous_curvature) < 1e-8 else step)
                if cfg.max_steering_paths_only and run > cfg.max_turn_arc_length+1e-8:
                    continue
                segment = []
                for i in range(1, samples + 1):
                    new_pose = advance(pose, direction * step * i / samples, curvature)
                    if not checker.free(new_pose):
                        break
                    segment.append(new_pose)
                if len(segment) != samples:
                    continue
                new_pose = segment[-1]
                effort = .2*step*(cfg.preferred_turn_radius*curvature)**2
                new_cost = cost + step * (1.0 if direction > 0 else 1.8) + 0.01 * abs(curvature) + effort
                # A short final primitive can share the start hash cell. Test
                # its swept, valid endpoint before discretization discards it.
                if at_goal(new_pose):
                    segments=[segment]
                    parent=index
                    while parent>0:
                        _,ancestor,previous_segment=nodes[parent]
                        segments.append(previous_segment);parent=ancestor
                    return [start]+[p for seg in reversed(segments) for p in seg]
                cell = key(new_pose, curvature, run)
                if new_cost >= costs.get(cell, math.inf):
                    continue
                costs[cell] = new_cost
                nodes.append((new_pose, index, segment))
                turn_runs.append((curvature, run))
                lengths.append(lengths[index]+step)
                heapq.heappush(queue, (new_cost + heuristic(new_pose), new_cost, len(nodes) - 1))
    return []
