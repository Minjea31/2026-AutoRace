"""Visualize a base_link LiDAR ROI, scan clusters, and unconfirmed cone candidates."""
import colorsys
import math
import time

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Point
from rcl_interfaces.msg import SetParametersResult, ParameterDescriptor
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray
from cone_lane_follower.async_planner import AsyncPlanner, check_budget
from cone_lane_follower.path_memory import PathMemory
from cone_lane_follower.corner_offset import offset as corner_offset, curvature as path_curvature


ROI_DEFAULTS = dict(roi_x_min=-0.3, roi_x_max=1.0, roi_y_min=-0.8, roi_y_max=0.8)
CLUSTER_DEFAULTS = dict(cluster_threshold=0.10, min_cluster_points=2,
                        max_cluster_width=0.28, cluster_point_size=0.05,
                        cone_center_size=0.08)


CENTER_DEFAULTS = dict(center_min_turn_radius=0.2630871384, center_sample_step=0.05, center_min_width=0.10,
                       center_max_width=1.50, center_max_gap=0.30,
                       center_max_turn=75.0, center_smoothing_weight=0.25,
                       center_width_relative_tolerance=0.25)


BOUNDARY_DEFAULTS = dict(boundary_connect_distance=0.30, boundary_max_turn=80.0,
                         boundary_initial_max_turn=110.0, boundary_direction_window=3,
                         boundary_distance_weight=1.0, boundary_direction_weight=2.0)


CONTROL_DEFAULTS = dict(control_speed=0.15, control_reference_x=0.25,
                        control_lateral_gain=2.0, control_heading_gain=1.0,
                        control_max_steering=math.radians(80.), control_max_steering_rate=1.0,
                        control_speed_reference_steering=0.48,
                        control_timeout=0.50, control_memory_tf_timeout=0.20,
                        control_max_path_distance=0.50,
                        control_memory_speed=0.15, control_memory_goal_tolerance=0.08,
                        control_memory_recovery_distance=0.15,
                        control_exit_timeout=1.50)


class RoiNode(Node):
    def __init__(self, **kwargs):
        super().__init__('cone_lane_roi', **kwargs)
        for name, default in {**ROI_DEFAULTS, **CLUSTER_DEFAULTS, **CENTER_DEFAULTS, **BOUNDARY_DEFAULTS, **CONTROL_DEFAULTS}.items():
            self.declare_parameter(name, default)
        startup_only = ParameterDescriptor(read_only=True)
        self.declare_parameter('planner_async', False, startup_only)
        self.declare_parameter('planner_budget_sec', .06, startup_only)
        budget = self.get_parameter('planner_budget_sec').value
        if not math.isfinite(budget) or budget <= 0:
            raise ValueError('planner_budget_sec must be finite and > 0')
        self.async_planner = AsyncPlanner() if self.get_parameter('planner_async').value else None
        self.measured_center_path = []
        self.planned_snapshot = None
        reason = self.validate_roi(self.roi_bounds())
        if reason:
            raise ValueError(reason)
        reason = self.validate_cluster_parameters(
            {name: self.get_parameter(name).value for name in (*CLUSTER_DEFAULTS, *CENTER_DEFAULTS, *BOUNDARY_DEFAULTS, *CONTROL_DEFAULTS)})
        if reason:
            raise ValueError(reason)
        self.add_on_set_parameters_callback(self.validate_parameters)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # Retain the outline for RViz instances that connect after startup.
        outline_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.roi_pub = self.create_publisher(Marker, '/cone_lane/roi_marker', outline_qos)
        self.status_pub = self.create_publisher(Marker, '/cone_lane/control_status', outline_qos)
        self.points_pub = self.create_publisher(Marker, '/cone_lane/roi_points', 1)
        self.clusters_pub = self.create_publisher(MarkerArray, '/cone_lane/clusters', 1)
        self.centers_pub = self.create_publisher(MarkerArray, '/cone_lane/cone_centers', 1)
        self.boundaries_pub = self.create_publisher(MarkerArray, '/cone_lane/boundaries', 1)
        self.control_points_pub = self.create_publisher(MarkerArray, '/cone_lane/control_points', 1)
        self.center_pub = self.create_publisher(MarkerArray, '/cone_lane/center_path', 1)
        self.center_spot_pub = self.create_publisher(MarkerArray, '/cone_lane/center_spot', 1)
        self.backup_pub = self.create_publisher(MarkerArray, '/cone_lane/backup_path', 1)
        self.center_pairs, self.center_points, self.center_path = [], [], []
        self.center_path_mode = 'Invalid'
        self.projection_status = 'not_attempted'
        self.left_boundary, self.right_boundary = [], []
        self.command_pub = self.create_publisher(
            AckermannDriveStamped, '/mission/cone/command_04', 1)
        self.control_engaged = False
        self.control_fault = False
        self.control_scan_stamp = None
        self.control_received = None
        self.last_control_tick = None
        self.last_steering = 0.
        self.control_mode = 'Idle'
        self.control_stop_reason = None
        self.path_diagnostics = {}
        self.last_logged_state = None
        self.command_speed = self.command_steering = 0.
        self.path_memory = PathMemory()
        self.memory_scan_stamp = None
        self.memory_local_path = []
        self.memory_status = 'empty'
        self.memory_update_reason = 'not_attempted'
        self.memory_following = False
        self.control_lateral_error = self.control_heading_error = 0.
        self.mission_cone = False
        self.mission_exit_since = None
        self.lane_width_mean = None
        self.lane_width_samples = 0
        self.lane_width_updated = None
        self.width_mission_active = False
        self.all_clusters, self.candidate_clusters = [], []
        self.boundary_candidate_clusters = []
        # Keep only the newest scan while path generation is busy.
        scan_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, scan_qos)
        self.last_warning = -math.inf
        self.outline_timer = self.create_timer(0.2, self.publish_roi_marker)
        self.control_timer = self.create_timer(.05, self.control_tick)
        self.status_timer = self.create_timer(.2, self.publish_control_status)
        self.status_log_timer = self.create_timer(1., self.publish_control_log)
        self.publish_roi_marker()
        self.publish_control_status()

    def roi_bounds(self):
        return {name: self.get_parameter(name).value for name in ROI_DEFAULTS}

    @staticmethod
    def validate_roi(bounds):
        if not all(isinstance(v, (int, float)) and not isinstance(v, bool)
                   and math.isfinite(v) for v in bounds.values()):
            return 'ROI bounds must be finite numbers'
        if (bounds['roi_x_min'] >= bounds['roi_x_max']
                or bounds['roi_y_min'] >= bounds['roi_y_max']):
            return 'ROI minima must be smaller than maxima'
        return ''

    def validate_parameters(self, parameters):
        bounds = self.roi_bounds()
        cluster_parameters = {name: self.get_parameter(name).value for name in (*CLUSTER_DEFAULTS, *CENTER_DEFAULTS, *BOUNDARY_DEFAULTS, *CONTROL_DEFAULTS)}
        for parameter in parameters:
            if parameter.name in bounds:
                bounds[parameter.name] = parameter.value
            if parameter.name in cluster_parameters:
                cluster_parameters[parameter.name] = parameter.value
        reason = self.validate_roi(bounds) or self.validate_cluster_parameters(cluster_parameters)
        return SetParametersResult(successful=not reason, reason=reason)

    @staticmethod
    def validate_cluster_parameters(parameters):
        for name, value in parameters.items():
            if name in ('min_cluster_points', 'boundary_direction_window'):
                if isinstance(value, bool) or not isinstance(value, int) or value < 1:
                    return f'{name} must be an integer >= 1'
            elif (isinstance(value, bool) or not isinstance(value, (int, float))
                  or not math.isfinite(value) or value <= 0):
                return f'{name} must be finite and > 0'
        if parameters['center_min_width'] >= parameters['center_max_width']:
            return 'center_min_width must be smaller than center_max_width'
        if parameters['center_smoothing_weight'] > .5 or parameters['center_max_turn'] > 180:
            return 'smoothing weight must be <= 0.5 and center_max_turn <= 180'
        if parameters['boundary_max_turn'] >= 180 or parameters['boundary_initial_max_turn'] > 180:
            return 'boundary_max_turn must be < 180 and initial turn <= 180 degrees'
        if parameters['control_max_steering'] >= math.pi/2:
            return 'control_max_steering must be < pi/2 radians'
        return ''

    def scan_callback(self, msg):
        stamp_ns = Time.from_msg(msg.header.stamp).nanoseconds
        age = (self.get_clock().now().nanoseconds-stamp_ns)/1e9
        # A stale queued scan must not refresh the watchdog or start expensive work.
        if stamp_ns > 0 and age > self.get_parameter('control_timeout').value:
            return
        if (self.control_received is None or time.monotonic()-self.control_received
                > self.get_parameter('control_timeout').value):
            self.mission_exit_since = None
        self.control_received = time.monotonic()
        self.control_scan_stamp = Time.from_msg(msg.header.stamp).nanoseconds
        self.control_fault = False
        points, indices = self.scan_to_points(msg, with_indices=True)
        try:
            if not msg.header.frame_id:
                raise ValueError('LaserScan frame_id is empty')
            # No blocking wait inside the callback: the listener uses this executor.
            # Time zero has TF2's standard latest-transform semantics.
            transform = self.tf_buffer.lookup_transform(
                'base_link', msg.header.frame_id, Time.from_msg(msg.header.stamp))
            points_base = self.transform_points(points, transform)
        except (TransformException, ValueError) as exc:
            if time.monotonic() - self.last_warning >= 1.0:
                self.get_logger().debug(f'Skipping scan: cannot transform to base_link: {exc}')
                self.last_warning = time.monotonic()
            self.control_fault = True
            self.mission_exit_since = None
            self.all_clusters, self.candidate_clusters = [], []
            self.boundary_candidate_clusters = []
            self.publish_roi_points([], msg.header.stamp)
            self.update_boundary_markers([], msg.header.stamp)
            self.publish_cluster_markers([], msg.header.stamp)
            self.publish_cone_center_markers([], msg.header.stamp)
            return
        roi_points, roi_indices = self.filter_roi(points_base, indices)
        # ROI candidates are exclusively for mission entry/exit decisions.
        self.candidate_clusters = self.filter_clusters(self.cluster_points(roi_points, roi_indices))
        self.all_clusters = self.cluster_points(points_base, indices)
        self.boundary_candidate_clusters = self.filter_clusters(self.all_clusters)
        # Candidate count gates entry only; after entry, path validity gates driving.
        if len(self.candidate_clusters) > 5:
            self.mission_cone = True
        centers = [self.calculate_cluster_center(cluster) for cluster in self.boundary_candidate_clusters]
        self.update_boundary_markers(centers, msg.header.stamp)
        self.publish_roi_points(roi_points, msg.header.stamp)
        self.publish_roi_marker()
        self.publish_cluster_markers(self.all_clusters, msg.header.stamp)
        self.publish_cone_center_markers(centers, msg.header.stamp)

    @staticmethod
    def scan_to_points(msg, with_indices=False):
        points, indices = [], []
        if (not math.isfinite(msg.angle_min) or not math.isfinite(msg.angle_increment)
                or not math.isfinite(msg.range_min) or math.isnan(msg.range_max)
                or msg.range_min < 0 or msg.range_max < msg.range_min):
            return (points, indices) if with_indices else points
        for index, distance in enumerate(msg.ranges):
            if not math.isfinite(distance) or not msg.range_min <= distance <= msg.range_max:
                continue
            angle = msg.angle_min + index * msg.angle_increment
            if math.isfinite(angle):
                points.append((distance * math.cos(angle), distance * math.sin(angle), 0.0))
                indices.append(index)
        return (points, indices) if with_indices else points

    @staticmethod
    def transform_points(points, transform):
        """Apply the complete TF rotation and translation once, retaining real Z."""
        t, q = transform.transform.translation, transform.transform.rotation
        norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
        if not math.isfinite(norm) or norm < 1e-12 or not all(
                math.isfinite(v) for v in (t.x, t.y, t.z)):
            raise ValueError('Invalid TF translation or quaternion')
        qx, qy, qz, qw = q.x/norm, q.y/norm, q.z/norm, q.w/norm
        result = []
        for x, y, z in points:
            # Quaternion vector rotation: v' = v + w*(2*q cross v) + q cross (2*q cross v).
            ux, uy, uz = 2*(qy*z-qz*y), 2*(qz*x-qx*z), 2*(qx*y-qy*x)
            result.append((x + qw*ux + qy*uz - qz*uy + t.x,
                           y + qw*uy + qz*ux - qx*uz + t.y,
                           z + qw*uz + qx*uy - qy*ux + t.z))
        return result

    def filter_roi(self, points, indices=None):
        b = self.roi_bounds()
        selected = [i for i, (x, y, z) in enumerate(points)
                    if b['roi_x_min'] < x < b['roi_x_max']
                    and b['roi_y_min'] < y < b['roi_y_max']]
        filtered = [points[i] for i in selected]
        return filtered if indices is None else (filtered, [indices[i] for i in selected])

    def cluster_points(self, points, indices):
        """Connect only successive original scan indices with XY distance < threshold.

        Input order is preserved; the first and last rays are never joined.
        """
        if len(points) != len(indices):
            raise ValueError('Point and scan index counts differ')
        threshold = self.get_parameter('cluster_threshold').value
        clusters = []
        for i, point in enumerate(points):
            if (i == 0 or indices[i] != indices[i-1] + 1
                    or math.hypot(point[0]-points[i-1][0], point[1]-points[i-1][1]) >= threshold):
                clusters.append([])
            clusters[-1].append(point)
        return clusters

    def filter_clusters(self, clusters):
        """Return cone candidates, not confirmed cones, using point count and XY spans."""
        minimum = self.get_parameter('min_cluster_points').value
        maximum_width = self.get_parameter('max_cluster_width').value
        return [cluster for cluster in clusters if len(cluster) >= minimum
                and max(p[0] for p in cluster)-min(p[0] for p in cluster) <= maximum_width
                and max(p[1] for p in cluster)-min(p[1] for p in cluster) <= maximum_width]

    @staticmethod
    def calculate_cluster_center(cluster):
        """Mean of observed points, without geometric cone center correction."""
        if not cluster:
            raise ValueError('Cannot calculate an empty cluster center')
        return tuple(sum(p[axis] for p in cluster)/len(cluster) for axis in range(3))

    @staticmethod
    def select_boundary_chain(cones, parameters=None, opposite=()):
        """Prefer seeds within +/-90 degrees, then choose by origin distance.

        Average recent unit segment directions to reduce jitter; also gate against
        the last segment so averaging cannot hide a sharp reversal. No X-order gate.
        Opposite is an already ordered boundary whose segments must not be crossed.
        """
        p = dict(BOUNDARY_DEFAULTS, **(parameters or {}))
        remaining = sorted(set(cones), key=lambda q: (math.hypot(q[0], q[1]), q))
        if not remaining:
            return []
        preferred = [q for q in remaining if abs(math.atan2(q[1], q[0])) <= math.radians(90.)]
        seed = (preferred or remaining)[0]
        remaining.remove(seed)
        chain, directions = [seed], []
        while remaining:
            options = RoiNode.boundary_options(chain, directions, remaining, p, opposite)
            if not options:
                break
            _, q, direction = min(options)
            chain.append(q)
            directions.append(direction)
            remaining.remove(q)
        return chain

    @staticmethod
    def boundary_options(chain, directions, remaining, p, opposite):
        def turn(a, b):
            return math.degrees(math.acos(max(-1., min(1., a[0]*b[0]+a[1]*b[1]))))
        recent = directions[-p['boundary_direction_window']:]
        hx, hy = (sum(v[0] for v in recent), sum(v[1] for v in recent)) if recent else (1., 0.)
        length = math.hypot(hx, hy)
        heading = (hx/length, hy/length) if length > 1e-9 else directions[-1]
        limit = p['boundary_max_turn'] if directions else p['boundary_initial_max_turn']
        options = []
        for q in remaining:
            dx, dy = q[0]-chain[-1][0], q[1]-chain[-1][1]
            gap = math.hypot(dx, dy)
            if not 1e-9 < gap <= p['boundary_connect_distance']:
                continue
            direction = (dx/gap, dy/gap)
            deviation = turn(heading, direction)
            if deviation > limit or (directions and turn(directions[-1], direction) > p['boundary_max_turn']):
                continue
            if any(RoiNode.segments_cross(chain[-1], q, a, b)
                   for a, b in list(zip(chain, chain[1:]))[:-1]+list(zip(opposite, opposite[1:]))):
                continue
            score = (p['boundary_distance_weight']*gap/p['boundary_connect_distance']
                     + p['boundary_direction_weight']*deviation/limit)
            options.append((score, q, direction))
        return options

    @staticmethod
    def boundary_width_matches(q, opposite, side, width, tolerance):
        # Only compare overlapping segments; do not extrapolate a shorter boundary.
        comparisons = []
        for a, b in zip(opposite, opposite[1:]):
            dx, dy = b[0]-a[0], b[1]-a[1]
            norm = dx*dx+dy*dy
            if norm < 1e-12:
                continue
            t = ((q[0]-a[0])*dx+(q[1]-a[1])*dy)/norm
            if 0. <= t <= 1.:
                signed = (dx*(q[1]-a[1])-dy*(q[0]-a[0]))/math.sqrt(norm)
                comparisons.append((abs(signed), side*signed))
        if not comparisons:
            return True
        _, inward_width = min(comparisons)
        return inward_width > 0. and abs(inward_width-width) <= width*tolerance

    @staticmethod
    def select_boundaries(cones, parameters, width=None, width_tolerance=.25):
        remaining = sorted(set(cones), key=lambda q: (math.hypot(q[0], q[1]), q))
        chains, directions = [[], []], [[], []]
        # Y sign is used only to initialize the two boundary identities.
        for i, side in enumerate((1., -1.)):
            candidates = [q for q in remaining if side*q[1] > 0.]
            preferred = [q for q in candidates if abs(math.atan2(q[1], q[0])) <= math.pi/2]
            if candidates:
                seed = (preferred or candidates)[0]
                chains[i].append(seed)
                remaining.remove(seed)
        while remaining:
            proposals = []
            for i, side in enumerate((1., -1.)):
                if not chains[i]:
                    continue
                for score, q, direction in RoiNode.boundary_options(
                        chains[i], directions[i], remaining, parameters, chains[1-i]):
                    if width is not None and not RoiNode.boundary_width_matches(
                            q, chains[1-i], side, width, width_tolerance):
                        continue
                    proposals.append((score, i, q, direction))
            if not proposals:
                break
            _, i, q, direction = min(proposals)
            chains[i].append(q)
            directions[i].append(direction)
            remaining.remove(q)
        return chains[0], chains[1]

    def update_boundary_markers(self, centers, stamp):
        parameters = {name: self.get_parameter(name).value for name in BOUNDARY_DEFAULTS}
        center_parameters = {name: self.get_parameter(name).value for name in CENTER_DEFAULTS}
        width = self.lane_width_mean if self.lane_width_is_fresh(center_parameters) else None
        self.left_boundary, self.right_boundary = self.select_boundaries(
            centers, parameters, width, center_parameters['center_width_relative_tolerance']) if self.mission_cone else ([], [])
        markers = self.marker_array(stamp)
        for name, boundary, color in (
                ('left_boundary', self.left_boundary, (1., 0., 0.)),
                ('right_boundary', self.right_boundary, (0., 0., 1.))):
            if not boundary:
                continue
            marker = self.new_marker(name, Marker.LINE_STRIP if len(boundary) >= 2 else Marker.POINTS, stamp)
            marker.scale.x = .03 if len(boundary) >= 2 else .10
            marker.scale.y = .10
            marker.color.r, marker.color.g, marker.color.b = color
            marker.points = [Point(x=x, y=y, z=z) for x, y, z in boundary]
            marker.lifetime = Duration(seconds=.5).to_msg()
            markers.markers.append(marker)
        self.boundaries_pub.publish(markers)
        self.update_center_path(stamp)
        self.update_control_activation()

    @staticmethod
    def sample_boundary(line, step):
        """Return points at cumulative XY arc lengths; preserve order, no extrapolation."""
        if len(line) < 2:
            return []
        result, target, total = [line[0]], step, 0.
        for a, b in zip(line, line[1:]):
            length = math.hypot(b[0]-a[0], b[1]-a[1])
            while length > 1e-9 and target < total+length:
                t = (target-total)/length
                result.append(tuple(a[k]+t*(b[k]-a[k]) for k in range(3)))
                target += step
            total += length
        if math.dist(result[-1][:2], line[-1][:2]) > 1e-8:
            result.append(line[-1])
        return result

    @staticmethod
    def segments_cross(a, b, c, d):
        def cross(p, q, r):
            return (q[0]-p[0])*(r[1]-p[1])-(q[1]-p[1])*(r[0]-p[0])
        if (max(a[0], b[0]) < min(c[0], d[0]) or max(c[0], d[0]) < min(a[0], b[0])
                or max(a[1], b[1]) < min(c[1], d[1]) or max(c[1], d[1]) < min(a[1], b[1])):
            return False
        return cross(a, b, c)*cross(a, b, d) <= 0 and cross(c, d, a)*cross(c, d, b) <= 0

    @staticmethod
    def match_cross_sections(left, right, step, min_width, max_width):
        """Intersect left local normals with right segments in increasing right arc order.

        With ROS positive-Y left, the inward normal is the right-hand normal
        of the forward tangent, directed from the left boundary into the lane. Ambiguous intersections are skipped.
        """
        refs = RoiNode.sample_boundary(left, step)
        if len(refs) < 2 or len(right) < 2:
            return []
        pairs, previous = [], -1.
        for i, q in enumerate(refs):
            a, b = refs[max(0, i-1)], refs[min(len(refs)-1, i+1)]
            dx, dy = b[0]-a[0], b[1]-a[1]
            length = math.hypot(dx, dy)
            if length < 1e-9:
                continue
            nx, ny = dy/length, -dx/length
            hits, arc = [], 0.
            for a, b in zip(right, right[1:]):
                vx, vy = b[0]-a[0], b[1]-a[1]
                segment_length = math.hypot(vx, vy)
                denominator = nx*vy-ny*vx
                if abs(denominator) > 1e-9 and dx*vx+dy*vy > 0:
                    ax, ay = a[0]-q[0], a[1]-q[1]
                    width = (ax*vy-ay*vx)/denominator
                    t = (ax*ny-ay*nx)/denominator
                    progress = arc+max(0., min(1., t))*segment_length
                    if min_width <= width <= max_width and -1e-8 <= t <= 1.+1e-8 and progress > previous+1e-8:
                        p = tuple(a[k]+max(0., min(1., t))*(b[k]-a[k]) for k in range(3))
                        if not any(math.dist(p[:2], h[1][:2]) < 1e-8 for h in hits):
                            hits.append((progress, p))
                arc += segment_length
            if len(hits) == 1:
                previous, opposite = hits[0]
                pairs.append((q, opposite))
        return pairs

    @staticmethod
    def match_projected_sections(left, right, step, min_width, max_width, diagnostics=None):
        """Return ordered left samples paired with closest valid right-segment projections.

        Prefer interior projections; out-of-range projections use the endpoint
        without an overrun-distance gate. Repeated endpoints are allowed. Both
        tangents must agree and the connector must point inward on both sides.
        Right arc progress must not decrease; ambiguous equal-distance hits are skipped.
        """
        def reject(reason):
            if diagnostics is not None:
                diagnostics[reason] = diagnostics.get(reason, 0)+1
        if len(left) < 2 or len(right) < 2:
            reject('insufficient_boundary_points')
            return []
        if not all(math.isfinite(v) for q in left+right for v in q):
            reject('nonfinite_boundary')
            return []
        refs = RoiNode.sample_boundary(left, step)
        if len(refs) < 2:
            reject('insufficient_boundary_samples')
        pairs, previous = [], -1.
        for i, q in enumerate(refs):
            a, b = refs[max(0, i-1)], refs[min(len(refs)-1, i+1)]
            dx, dy = b[0]-a[0], b[1]-a[1]
            norm = math.hypot(dx, dy)
            if norm < 1e-9:
                reject('degenerate_left_tangent')
                continue
            options, arc = [], 0.
            for a, b in zip(right, right[1:]):
                vx, vy = b[0]-a[0], b[1]-a[1]
                length = math.hypot(vx, vy)
                if length < 1e-9:
                    reject('degenerate_right_segment')
                    continue
                raw_t = ((q[0]-a[0])*vx+(q[1]-a[1])*vy)/(length*length)
                t = max(0., min(1., raw_t))
                point = tuple(a[k]+t*(b[k]-a[k]) for k in range(3))
                rx, ry = point[0]-q[0], point[1]-q[1]
                width = math.hypot(rx, ry)
                progress = arc+t*length
                arc += length
                interior = 1e-8 < raw_t < 1.-1e-8
                if not min_width <= width <= max_width:
                    reject('width_out_of_range')
                    continue
                if progress < previous-1e-8:
                    reject('backward_correspondence')
                    continue
                if dx*vx+dy*vy <= 0:
                    reject('opposing_tangents')
                    continue
                if (dy*rx-dx*ry)/norm <= 1e-8 or (vy*rx-vx*ry)/length <= 1e-8:
                    reject('connector_not_inward')
                    continue
                if (pairs and math.dist(pairs[-1][1][:2], point[:2]) > 1e-8
                        and RoiNode.segments_cross(pairs[-1][0], pairs[-1][1], q, point)):
                    reject('crossing_sections')
                    continue
                candidate = (not interior, width, progress, point)
                if not any(math.dist(point[:2], h[3][:2]) < 1e-8 for h in options):
                    options.append(candidate)
            if not options:
                continue
            options.sort()
            if (len(options) > 1 and options[0][0] == options[1][0]
                    and abs(options[0][1]-options[1][1]) < 1e-8):
                reject('ambiguous_projection')
                continue
            _, _, previous, point = options[0]
            pairs.append((q, point))
        return pairs

    @staticmethod
    def smooth_center_path(centers, left, right, weight, max_gap, max_turn, diagnostics=None):
        """Keep correspondence order; average neighbours only inside a simple lane polygon."""
        def fail(reason):
            if diagnostics is not None:
                diagnostics[reason] = diagnostics.get(reason, 0)+1
            return []
        if len(centers) < 2:
            return fail('insufficient_center_points')
        polygon = left+list(reversed(right))
        edges = list(zip(polygon, polygon[1:]+polygon[:1]))
        if any(RoiNode.segments_cross(a, b, c, d) for i, (a, b) in enumerate(edges)
               for j, (c, d) in enumerate(edges) if j > i+1 and not (i == 0 and j == len(edges)-1)):
            return fail('lane_polygon_crosses')
        def inside(p):
            count = 0
            for a, b in edges:
                if (a[1] > p[1]) != (b[1] > p[1]):
                    x = a[0]+(p[1]-a[1])*(b[0]-a[0])/(b[1]-a[1])
                    count += x > p[0]
            return count % 2 == 1
        def safe_segment(a, b):
            return inside(a) and inside(b) and not any(RoiNode.segments_cross(a, b, c, d) for c, d in edges)
        # Separate invalid gaps; do not reconnect them by nearest-neighbour sorting.
        runs, run = [], []
        for p in centers:
            if not inside(p):
                runs.append(run)
                run = []
                continue
            if run and (math.dist(run[-1][:2], p[:2]) > max_gap or not safe_segment(run[-1], p)):
                runs.append(run)
                run = []
            run.append(p)
        runs.append(run)
        path = list(max(runs, key=len))
        if len(path) < 2:
            return fail('no_continuous_inside_path')
        for i in range(1, len(path)-1):
            p = tuple((1.-weight)*path[i][k]+weight*(path[i-1][k]+path[i+1][k])/2 for k in range(3))
            if safe_segment(path[i-1], p) and safe_segment(p, path[i+1]):
                path[i] = p
        for i, (a, b, c) in enumerate(zip(path, path[1:], path[2:])):
            u, v = (b[0]-a[0], b[1]-a[1]), (c[0]-b[0], c[1]-b[1])
            cosine = (u[0]*v[0]+u[1]*v[1])/max(1e-12, math.hypot(*u)*math.hypot(*v))
            if math.degrees(math.acos(max(-1., min(1., cosine)))) > max_turn:
                return fail('path_turn_limit')
        segments = list(zip(path, path[1:]))
        if any(RoiNode.segments_cross(a, b, c, d) for i, (a, b) in enumerate(segments) for c, d in segments[i+2:]):
            return fail('path_self_intersection')
        return path

    @staticmethod
    def offset_boundary(boundary, distance, step):
        """Offset along local left normals; negative distance moves to the right."""
        if not all(math.isfinite(v) for q in boundary for v in q):
            return []
        refs = RoiNode.sample_boundary(boundary, step)
        shifted = []
        for i, q in enumerate(refs):
            a, b = refs[max(0, i-1)], refs[min(len(refs)-1, i+1)]
            dx, dy = b[0]-a[0], b[1]-a[1]
            length = math.hypot(dx, dy)
            if length < 1e-9:
                return []
            shifted.append((q[0]-distance*dy/length, q[1]+distance*dx/length, q[2]))
        return shifted

    def reset_lane_width(self):
        self.lane_width_mean = None
        self.lane_width_samples = 0
        self.lane_width_updated = None
        self.width_mission_active = False

    def lane_width_is_fresh(self, p):
        if self.lane_width_mean is None or self.lane_width_updated is None:
            return False
        return p['center_min_width'] <= self.lane_width_mean <= p['center_max_width']

    @staticmethod
    def point_near_boundary(point, line, tolerance):
        for a, b in zip(line, line[1:]):
            dx, dy = b[0]-a[0], b[1]-a[1]
            norm = dx*dx+dy*dy
            if norm < 1e-12:
                continue
            t = max(0., min(1., ((point[0]-a[0])*dx+(point[1]-a[1])*dy)/norm))
            if math.hypot(point[0]-a[0]-t*dx, point[1]-a[1]-t*dy) <= tolerance:
                return True
        return False

    def update_center_path(self, stamp):
        self.path_diagnostics = {}
        self.projection_status = 'not_attempted'
        if not self.mission_cone or not self.width_mission_active:
            self.reset_lane_width()
        self.width_mission_active = self.mission_cone
        p = {name: self.get_parameter(name).value for name in CENTER_DEFAULTS}
        both_boundaries_valid = len(self.left_boundary) >= 3 and len(self.right_boundary) >= 3
        self.center_pairs = self.match_cross_sections(
            self.left_boundary, self.right_boundary, p['center_sample_step'],
            p['center_min_width'], p['center_max_width']) if both_boundaries_valid else []
        self.center_points = [tuple((a[k]+b[k])/2 for k in range(3)) for a, b in self.center_pairs]
        self.center_path = self.smooth_center_path(
            self.center_points, self.left_boundary, self.right_boundary,
            p['center_smoothing_weight'], p['center_max_gap'], p['center_max_turn'],
            self.path_diagnostics)
        self.center_path_mode = 'Normal Sections' if self.center_path else 'Invalid'
        # Learn perpendicular widths only from valid measured cross sections.
        # Endpoint projections can overestimate width and are excluded from learning.
        if (self.mission_cone and self.center_path and self.path_errors(
                self.center_path, self.get_parameter('control_reference_x').value,
                self.get_parameter('control_max_path_distance').value) is not None):
            widths = [math.dist(a[:2], b[:2]) for a, b in self.center_pairs]
            mean = sum(widths)/len(widths)
            if max(widths)-min(widths) <= mean*p['center_width_relative_tolerance']:
                # Reset unavailable or out-of-range measurements before learning.
                if not self.lane_width_is_fresh(p):
                    self.lane_width_mean, self.lane_width_samples = None, 0
                total = (self.lane_width_mean or 0.)*self.lane_width_samples + sum(widths)
                self.lane_width_samples += len(widths)
                self.lane_width_mean = total/self.lane_width_samples
                self.lane_width_updated = (time.monotonic(), self.get_clock().now().nanoseconds)
            # Reject noisy measurements while preserving the previously trusted width.
        self.measured_center_path = list(self.center_path)
        if self.async_planner is None:
            self.center_path = self.unified_path(
                self.left_boundary, self.right_boundary, self.center_pairs,
                self.center_path, self.lane_width_mean, p, self.path_diagnostics)
            self.center_path_mode = 'Unified' if self.center_path else 'Invalid'
        elif not self.mission_cone or self.lane_width_mean is None:
            self.async_planner.reset()
            self.planned_snapshot = None
        else:
            try:
                tf = self.memory_transform('odom', 'base_link', Time.from_msg(stamp))
            except (TransformException, ValueError):
                tf = None
            payload = (list(self.left_boundary), list(self.right_boundary),
                       list(self.center_pairs), list(self.center_path), self.lane_width_mean,
                       p, self.get_parameter('planner_budget_sec').value)
            self.async_planner.request(payload, (
                Time.from_msg(stamp).nanoseconds, tf, p, time.monotonic()))
            self.use_async_path()
        if (not self.center_path and self.lane_width_mean is not None
                and (len(self.left_boundary) >= 3) != (len(self.right_boundary) >= 3)
                and self.left_boundary and self.right_boundary):
            if 'width_conflict' in self.path_diagnostics:
                self.center_path_mode = 'Boundary Width Conflict'
        self.publish_center_path_markers(stamp)

    @staticmethod
    def unified_path(left, right, pairs, measured, width, p, diagnostics=None):
        diagnostics = diagnostics if diagnostics is not None else {}
        valid = [(line, side) for line, side in ((left, -1.), (right, 1.))
                 if len(line) >= 3]
        if not valid or width is None:
            diagnostics['offset_unavailable'] = 'requires boundary >=3 points and learned width'
            return measured
        def distance(line):
            return min(RoiNode.segment_projection((0., 0., 0.), a, b)[0]
                       for a, b in zip(line, line[1:]))
        if not pairs:
            line, side = min(valid, key=lambda item: distance(item[0]))
            opposite = right if side == -1. else left
            for point in opposite if len(opposite) < 3 else ():
                candidates = []
                for a, b in zip(line, line[1:]):
                    dist, t, q = RoiNode.segment_projection(point, a, b)
                    dx, dy = b[0]-a[0], b[1]-a[1]
                    length = math.hypot(dx, dy)
                    if length > 1e-9:
                        candidates.append((dist, side*(dx*(point[1]-q[1])-dy*(point[0]-q[0]))/length))
                if candidates:
                    dist, inward = min(candidates)
                    if inward <= 0 or abs(dist-width) > width*p['center_width_relative_tolerance']:
                        diagnostics['width_conflict'] = (f'point={point} width={width:.3f}m '
                            f'distance={dist:.3f}m inward={inward:.3f}m '
                            f'allowed=[{width*(1-p["center_width_relative_tolerance"]):.3f}, '
                            f'{width*(1+p["center_width_relative_tolerance"]):.3f}]m')
                        return []
            diagnostics['offset_side'] = 'left boundary -> right normal' if side == -1 else 'right boundary -> left normal'
            path = corner_offset(line, side*width/2, p['center_sample_step'],
                                 p['center_min_turn_radius'], RoiNode, p, diagnostics)
            diagnostics['offset_length_m'] = sum(math.dist(a,b) for a,b in zip(path,path[1:]))
            return path if RoiNode.offset_safe(path, valid, diagnostics) and RoiNode.valid_center_geometry(path, p, diagnostics) else []
        if not measured:
            return []  # Ambiguous/invalid observed overlap is not a missing boundary.
        # Arc positions of the final measured section on both ordered boundaries.
        tails = []
        for index, (line, side) in enumerate(((left, -1.), (right, 1.))):
            if len(line) < 3:
                continue
            refs = RoiNode.sample_boundary(line, p['center_sample_step'])
            point = pairs[-1][index]
            hits = [(RoiNode.segment_projection(point, a, b), i)
                    for i, (a, b) in enumerate(zip(refs, refs[1:]))]
            (dist, t, q), i = min(hits, key=lambda hit: hit[0][0])
            remaining = math.dist(q, refs[i+1])+sum(math.dist(a, b)
                        for a, b in zip(refs[i+1:], refs[i+2:]))
            tails.append((remaining, line, side, i, t))
        if len(tails) != 2:
            return measured
        short, long = sorted(tails, key=lambda item: item[0])
        # Sample discretization is not evidence of an unobserved tail.
        if long[0]-short[0] <= p['center_sample_step']:
            return measured
        _, line, side, i, t = long
        offset = corner_offset(line, side*width/2, p['center_sample_step'],
                               p['center_min_turn_radius'], RoiNode, p, diagnostics)
        if not offset:
            return measured
        target = RoiNode.offset_boundary(line, side*width/2, p['center_sample_step'])[i]
        cut = min(range(len(offset)), key=lambda j: math.dist(offset[j], target))
        tail = offset[cut+1:]
        diagnostics['offset_side'] = 'left boundary -> right normal' if side == -1 else 'right boundary -> left normal'
        diagnostics['offset_length_m'] = sum(math.dist(a,b) for a,b in zip(tail,tail[1:]))
        if len(tail) < 2:
            return measured
        for count in range(3, max(len(measured), len(tail))+1):
            check_budget(p)
            start = max(0, len(measured)-count)
            end = min(len(tail)-1, count-1)
            bridge = RoiNode.quintic_bridge(measured, start, tail, end, p['center_sample_step'])
            combined = measured[:start]+bridge+tail[end+1:]
            if (bridge and RoiNode.offset_safe(combined, valid, diagnostics)
                    and RoiNode.valid_center_geometry(combined, p, diagnostics)
                    and max(path_curvature(combined), default=0.) <= 1/p['center_min_turn_radius']):
                diagnostics['join'] = f'success window_samples={count}'
                return combined
        diagnostics['join'] = 'failed; retained measured sections'
        return measured

    @staticmethod
    def segment_projection(point, a, b):
        dx, dy = b[0]-a[0], b[1]-a[1]
        norm = dx*dx+dy*dy
        t = max(0., min(1., ((point[0]-a[0])*dx+(point[1]-a[1])*dy)/norm)) if norm else 0.
        q = tuple(a[k]+t*(b[k]-a[k]) for k in range(3))
        return math.dist(point[:2], q[:2]), t, q

    @staticmethod
    def offset_safe(path, boundaries, diagnostics=None):
        for line, side in boundaries:
            segments = list(zip(line, line[1:]))
            for point in path:
                (distance, t, q), a, b = min(
                    [(RoiNode.segment_projection(point, a, b), a, b) for a, b in segments],
                    key=lambda item: item[0][0])
                # Only enforce side over the observed finite span.
                dx, dy = b[0]-a[0], b[1]-a[1]
                raw = ((point[0]-a[0])*dx+(point[1]-a[1])*dy)/(dx*dx+dy*dy)
                if 0 <= raw <= 1 and side*(dx*(point[1]-q[1])-dy*(point[0]-q[0])) <= 0:
                    if diagnostics is not None:
                        diagnostics['outside_boundary'] = f'point={point} boundary_side={side:+.0f}'
                    return False
            if any(RoiNode.segments_cross(a, b, c, d)
                   for a, b in zip(path, path[1:]) for c, d in segments):
                if diagnostics is not None:
                    diagnostics['boundary_crossing'] = f'boundary_side={side:+.0f}'
                return False
        return True

    @staticmethod
    def quintic_bridge(first, i, second, j, step):
        def derivatives(path, index):
            lo, hi = max(0, index-1), min(len(path)-1, index+1)
            length = math.dist(path[lo], path[hi])
            if length < 1e-9:
                return None
            tangent = [(path[hi][k]-path[lo][k])/length for k in range(3)]
            curvature = [0., 0., 0.]
            if 0 < index < len(path)-1:
                a, b, c = path[index-1:index+2]
                u, v = math.dist(a, b), math.dist(b, c)
                if min(u, v) > 1e-9:
                    curvature = [2*((c[k]-b[k])/v-(b[k]-a[k])/u)/(u+v) for k in range(3)]
            return tangent, curvature
        left, right = derivatives(first, i), derivatives(second, j)
        length = math.dist(first[i], second[j])
        if left is None or right is None or length < 1e-9:
            return []
        coefficients = []
        for k in range(3):
            a, b, c = first[i][k], left[0][k]*length, left[1][k]*length*length/2
            d = second[j][k]-a-b-c
            e = right[0][k]*length-b-2*c
            f = right[1][k]*length*length-2*c
            coefficients.append((a, b, c, 10*d-4*e+f/2, -15*d+7*e-f, 6*d-3*e+f/2))
        samples = max(2, math.ceil(length/step)*4)
        curve = [tuple(sum(coef[n]*(m/samples)**n for n in range(6))
                       for coef in coefficients) for m in range(samples+1)]
        return RoiNode.sample_boundary(curve, step)

    @staticmethod
    def valid_center_geometry(path, p, diagnostics=None):
        if len(path) < 2 or not all(math.isfinite(v) for q in path for v in q):
            return False
        segments = list(zip(path, path[1:]))
        for i, (a,b) in enumerate(segments):
            gap = math.dist(a[:2], b[:2])
            if gap > p['center_max_gap']:
                if diagnostics is not None:
                    diagnostics['gap'] = f'segment={i}->{i+1} distance={gap:.4f}m limit={p["center_max_gap"]:.4f}m'
                return False
        for i, (a, b, c) in enumerate(zip(path, path[1:], path[2:])):
            u, v = (b[0]-a[0], b[1]-a[1]), (c[0]-b[0], c[1]-b[1])
            norm = math.hypot(*u)*math.hypot(*v)
            if norm < 1e-12 or math.degrees(math.acos(max(-1., min(1.,
                    (u[0]*v[0]+u[1]*v[1])/norm)))) > p['center_max_turn']:
                if diagnostics is not None:
                    angle = math.degrees(math.acos(max(-1., min(1., (u[0]*v[0]+u[1]*v[1])/norm)))) if norm >= 1e-12 else None
                    diagnostics['turn'] = f'point={i+1} xy={b[:2]} angle={angle}deg limit={p["center_max_turn"]}deg'
                return False
        for i, (a, b) in enumerate(segments):
            for j in range(i+2, len(segments)):
                c, d = segments[j]
                if RoiNode.segments_cross(a, b, c, d):
                    if diagnostics is not None:
                        diagnostics['self_crossing'] = f'segments={i}->{i+1}, {j}->{j+1}'
                    return False
        return True

    def publish_center_path_markers(self, stamp):
        markers = self.marker_array(stamp)
        backup = self.marker_array(stamp)
        spots = self.marker_array(stamp)
        for name, points, kind, color, size in (
                ('cross_sections', [p for pair in self.center_pairs for p in pair], Marker.LINE_LIST, (1., 1., 0.), .01),
                ('center_points', self.center_points, Marker.POINTS, (1., 0., 1.), .05),
                ('center_path', self.center_path, Marker.LINE_STRIP,
                 (0., 1., 0.), .03),
                ('remembered_path', self.memory_local_path, Marker.LINE_STRIP,
                 (1., .6, 0.), .05)):
            if not points:
                continue
            marker = self.new_marker(name, kind, stamp)
            marker.points = [Point(x=x, y=y, z=z) for x, y, z in points]
            marker.scale.x = marker.scale.y = size
            marker.color.r, marker.color.g, marker.color.b = color
            marker.lifetime = Duration(seconds=.5).to_msg()
            target = (backup if name == 'remembered_path' else
                      spots if name in ('cross_sections', 'center_points') else markers)
            target.markers.append(marker)
        self.center_pub.publish(markers)
        self.backup_pub.publish(backup)
        self.center_spot_pub.publish(spots)

    @staticmethod
    def path_errors(path, reference_x, max_distance=0.50):
        geometry = RoiNode.path_control_geometry(path, reference_x, max_distance)
        return geometry[:2] if geometry is not None else None

    @staticmethod
    def path_control_geometry(path, reference_x, max_distance=0.50):
        """Return signed lateral metres and heading radians at a forward reference point.

        Project (reference_x, 0) onto the closest forward-directed finite segment.
        Positive lateral error means the path lies to the left of the reference.
        No extrapolation or reverse-driving segment is accepted.
        """
        if len(path) < 2 or not all(math.isfinite(v) for q in path for v in q):
            return None
        choices = []
        for a, b in zip(path, path[1:]):
            dx, dy = b[0]-a[0], b[1]-a[1]
            length = math.hypot(dx, dy)
            if length < 1e-8 or dx/length <= .1:
                continue
            t = max(0., min(1., ((reference_x-a[0])*dx-a[1]*dy)/(length*length)))
            x, y = a[0]+t*dx, a[1]+t*dy
            if x <= 0:
                continue
            lateral = (dx*y-dy*(x-reference_x))/length
            choices.append((math.hypot(x-reference_x, y), lateral, math.atan2(dy, dx), x, y, a, b))
        if not choices:
            return None
        selected = min(choices, key=lambda choice: choice[:3])
        return selected[1:] if selected[0] <= max_distance else None

    def publish_drive(self, speed, steering):
        self.command_speed, self.command_steering = speed, steering
        message = AckermannDriveStamped()
        message.header.frame_id = 'base_link'
        message.header.stamp = self.get_clock().now().to_msg()
        message.drive.speed, message.drive.steering_angle = float(speed), float(steering)
        self.command_pub.publish(message)

    def clear_path_memory(self):
        if self.async_planner is not None:
            self.async_planner.reset()
            self.planned_snapshot = None
        self.path_memory.clear()
        self.memory_scan_stamp = None
        self.memory_local_path = []
        self.memory_status = 'empty'
        self.memory_update_reason = 'not_attempted'
        self.memory_following = False

    def memory_transform(self, target, source, stamp):
        tf = self.tf_buffer.lookup_transform(target, source, stamp)
        age = (self.get_clock().now().nanoseconds-Time.from_msg(tf.header.stamp).nanoseconds)/1e9
        if (tf.header.frame_id != target or tf.child_frame_id != source
                or not 0 <= age <= self.get_parameter('control_memory_tf_timeout').value):
            raise ValueError('Missing or stale odom transform')
        return tf

    def remaining_memory_path(self, p):
        if not self.path_memory.points:
            return [], 'memory_path_unavailable'
        if self.path_memory.finished:
            return [], 'memory_goal_reached'
        try:
            tf = self.memory_transform('base_link', 'odom', Time())
            local = self.transform_points(self.path_memory.points, tf)
        except (TransformException, ValueError):
            return [], 'memory_tf_unavailable_or_stale'
        return self.path_memory.remaining(local, p['control_memory_goal_tolerance'],
                                          p['control_max_path_distance'])

    def remember_live_path(self, replace=False):
        if self.memory_scan_stamp == self.control_scan_stamp and not replace:
            return
        try:
            tf = self.memory_transform('odom', 'base_link',
                                       Time(nanoseconds=self.control_scan_stamp))
            points = self.transform_points(self.center_path, tf)
            if replace:
                tf = self.memory_transform('base_link', 'odom', Time())
                local = self.transform_points(points, tf)
        except (TransformException, ValueError):
            self.memory_status = 'save_tf_unavailable_or_stale'
            self.memory_update_reason = 'save_tf_unavailable_or_stale'
            return
        if replace:
            self.path_memory.replace_from_vehicle(
                points, local, self.get_parameter('control_memory_recovery_distance').value)
        else:
            self.path_memory.refresh(points, lambda path: self.valid_center_geometry(
                path, {name: self.get_parameter(name).value for name in CENTER_DEFAULTS}))
        self.memory_scan_stamp = self.control_scan_stamp
        self.memory_status = self.path_memory.update_reason
        self.memory_update_reason = self.path_memory.update_reason

    @staticmethod
    def path_stop_reason(path, reference_x):
        if not path:
            return 'empty_path'
        if len(path) < 2:
            return 'path_too_few_points'
        if not all(math.isfinite(v) for q in path for v in q):
            return 'path_nonfinite_coordinates'
        if RoiNode.path_errors(path, reference_x, math.inf) is None:
            return 'path_no_usable_forward_segment'
        return 'path_too_far_from_reference'

    def stop_control(self, reason='unspecified'):
        """Stop immediately and reset steering."""
        if reason in ('tf_error', 'mission_inactive'):
            self.clear_path_memory()
        self.publish_control_points()
        self.publish_drive(0., 0.)
        self.last_steering = 0.
        self.control_mode = 'Stopped'
        if reason != self.control_stop_reason:
            self.get_logger().warning(f'Control stopped: reason={reason}')
        self.control_stop_reason = reason

    def update_control_activation(self):
        if (not self.mission_cone or self.control_fault
                or len(self.candidate_clusters) > 3
                or self.path_errors(self.center_path,
                                    self.get_parameter('control_reference_x').value,
                                    self.get_parameter('control_max_path_distance').value) is not None):
            self.mission_exit_since = None
        # A zero-speed command is active in the mux: idle must eventually be silent.
        if self.mission_cone:
            self.control_engaged = True
        elif self.control_engaged and not self.control_fault:
            self.stop_control('mission_inactive')
            self.control_engaged = False
        if self.control_engaged and self.control_fault:
            self.stop_control('tf_error')
        elif self.control_engaged and not self.center_path:
            self.control_tick()

    def control_tick(self):
        """20 Hz feedback; stop whenever no usable path is available."""
        steady, now = time.monotonic(), self.get_clock().now().nanoseconds
        dt = .05 if self.last_control_tick is None else max(0., min(.1, steady-self.last_control_tick))
        self.last_control_tick = steady
        if not self.control_engaged:
            self.publish_control_points()
            self.clear_path_memory()
            self.mission_exit_since = None
            self.last_steering = 0.
            self.control_mode = 'Idle'
            return
        p = {name: self.get_parameter(name).value for name in CONTROL_DEFAULTS}
        fresh = (self.control_received is not None and self.control_scan_stamp is not None
                 and self.control_scan_stamp > 0
                 and 0 <= steady-self.control_received <= p['control_timeout']
                 and 0 <= (now-self.control_scan_stamp)/1e9 <= p['control_timeout'])
        if not fresh or self.control_fault or not self.mission_cone:
            self.mission_exit_since = None
            if self.control_fault:
                reason = 'tf_error'
            elif not self.mission_cone:
                reason = 'mission_inactive'
            elif self.control_received is None or self.control_scan_stamp is None:
                reason = 'scan_not_received'
            elif self.control_scan_stamp <= 0:
                reason = 'scan_stamp_zero_or_negative'
            elif steady < self.control_received:
                reason = 'scan_receive_clock_reversed'
            elif steady-self.control_received > p['control_timeout']:
                reason = 'scan_receive_timeout'
            elif now < self.control_scan_stamp:
                reason = 'scan_stamp_in_future'
            else:
                reason = 'scan_stamp_timeout'
            if reason in ('scan_receive_timeout', 'scan_stamp_timeout'):
                # Stop on stale sensor data, but keep the odom path for recovery.
                self.memory_following = False
                self.memory_local_path = []
                self.memory_status = 'paused_scan_timeout' if self.path_memory.points else 'empty'
                self.publish_center_path_markers(self.get_clock().now().to_msg())
            else:
                self.clear_path_memory()
            self.stop_control(reason)
            return
        if self.async_planner is not None:
            self.use_async_path()
        errors = self.path_errors(self.center_path, p['control_reference_x'], p['control_max_path_distance'])
        live_errors = errors
        memory_reason = None
        self.memory_local_path = []
        # Advance old path progress before considering an extension; past points
        # cannot make a new short observation appear longer than the stored path.
        if self.path_memory.points:
            previous, memory_reason = self.remaining_memory_path(p)
            if memory_reason is None and self.path_errors(
                    previous, p['control_reference_x'], p['control_max_path_distance']) is None:
                memory_reason = 'memory_'+self.path_stop_reason(previous, p['control_reference_x'])
        if live_errors is not None:
            self.remember_live_path()
            if self.path_memory.finished:
                self.remember_live_path(replace=True)
        elif live_errors is not None and memory_reason != 'memory_tf_unavailable_or_stale':
            self.remember_live_path(replace=True)
        elif live_errors is None:
            self.memory_update_reason = 'not_attempted: '+self.path_stop_reason(
                self.center_path, p['control_reference_x'])
        else:
            self.memory_update_reason = 'not_attempted: '+memory_reason
        self.memory_following = False
        if self.path_memory.points:
            remaining, memory_reason = self.remaining_memory_path(p)
            errors = self.path_errors(remaining, p['control_reference_x'], p['control_max_path_distance'])
            if errors is not None:
                self.memory_local_path = remaining
                self.memory_following = True
                if live_errors is None:
                    self.memory_status = 'following'
            else:
                memory_reason = memory_reason or 'memory_'+self.path_stop_reason(
                    remaining, p['control_reference_x'])
                self.memory_status = memory_reason
        if live_errors is not None:
            errors = live_errors
            self.memory_following = False
        self.publish_center_path_markers(self.get_clock().now().to_msg())
        if errors is None and len(self.candidate_clusters) <= 3:
            if self.mission_exit_since is None:
                self.mission_exit_since = (steady, now)
            saved_steady, saved_ros = self.mission_exit_since
            if steady < saved_steady or now < saved_ros:
                self.mission_exit_since = (steady, now)
            elif (steady-saved_steady >= p['control_exit_timeout']
                    and (now-saved_ros)/1e9 >= p['control_exit_timeout']):
                self.mission_cone = False
                self.clear_path_memory()
                self.reset_lane_width()
                self.mission_exit_since = None
                self.stop_control('mission_exit_detection_loss')
                self.control_engaged = False
                self.get_logger().info('Cone mission ended: insufficient candidates and no valid path')
                return
        else:
            self.mission_exit_since = None
        if errors is None:
            reason = {'Width Unavailable or Stale': 'lane_width_unavailable_or_stale',
                      'Boundary Width Conflict': 'boundary_width_conflict'}.get(self.center_path_mode)
            self.stop_control(memory_reason or reason or self.path_stop_reason(self.center_path, p['control_reference_x']))
            return
        self.control_lateral_error, self.control_heading_error = errors
        target = p['control_lateral_gain']*errors[0]+p['control_heading_gain']*errors[1]
        limit = p['control_max_steering']
        target = max(-limit, min(limit, target))
        step = p['control_max_steering_rate']*dt
        self.last_steering += max(-step, min(step, target-self.last_steering))
        self.last_steering = max(-limit, min(limit, self.last_steering))
        # Saturate slowdown independently of the permitted steering range.
        slowdown = min(1., abs(target)/p['control_speed_reference_steering'])
        speed = p['control_speed']*(1.-.25*slowdown)
        active_path = self.memory_local_path if self.memory_following else self.center_path
        self.publish_control_points(active_path)
        peak_curvature = max(path_curvature(active_path), default=0.)
        if peak_curvature > 1e-9:
            corner_ratio = min(1., peak_curvature*self.get_parameter('center_min_turn_radius').value)
            speed = min(speed, p['control_speed']*(1.-.25*corner_ratio))
        if self.memory_following:
            speed = min(speed, p['control_memory_speed'])
        self.control_mode = 'Memory Path Feedback' if self.memory_following else 'Path Feedback'
        self.control_stop_reason = None
        self.publish_drive(speed, self.last_steering)

    def use_async_path(self):
        # Measured sections are immediately available; never wait for the child.
        self.center_path = list(self.measured_center_path)
        self.center_path_mode = 'Measured (planner pending)' if self.center_path else 'Invalid'
        result = self.async_planner.poll()
        if result is not None:
            metadata, (path, diagnostics) = result
            stamp_ns, to_odom, parameters, received = metadata
            self.path_diagnostics.update(diagnostics)
            if path and to_odom is not None:
                self.planned_snapshot = (
                    self.transform_points(path, to_odom), stamp_ns, parameters, received)
            else:
                self.planned_snapshot = None
        if self.planned_snapshot is None:
            return
        odom_path, stamp_ns, parameters, received = self.planned_snapshot
        timeout = self.get_parameter('control_timeout').value
        age = (self.get_clock().now().nanoseconds-stamp_ns)/1e9
        current_parameters = {name: self.get_parameter(name).value for name in CENTER_DEFAULTS}
        if (not 0 <= age <= timeout or not 0 <= time.monotonic()-received <= timeout
                or parameters != current_parameters):
            self.planned_snapshot = None
            return
        try:
            # Keep the same scan-time base frame used by measured paths and
            # remember_live_path; do not label a current-pose path as an older scan.
            scan_time = Time(nanoseconds=self.control_scan_stamp)
            to_base = self.memory_transform('base_link', 'odom', scan_time)
            self.center_path = self.transform_points(odom_path, to_base)
            self.center_path_mode = 'Unified (worker)'
        except (TransformException, ValueError):
            self.path_diagnostics['planner_tf'] = 'unavailable; retained measured path'

    def destroy_node(self):
        if self.async_planner is not None:
            self.async_planner.close()
        return super().destroy_node()

    def publish_control_points(self, path=None):
        markers = self.marker_array()
        geometry = self.path_control_geometry(
            path or [], self.get_parameter('control_reference_x').value,
            self.get_parameter('control_max_path_distance').value)
        if geometry is not None:
            _, heading, x, y, a, b = geometry
            reference_x = self.get_parameter('control_reference_x').value
            for namespace, position, color in (
                    ('reference', (reference_x, 0.), (1., 1., 1.)),
                    ('projection', (x, y), (0., 1., 1.))):
                marker = self.new_marker(namespace, Marker.SPHERE)
                marker.pose.position = Point(x=float(position[0]), y=float(position[1]), z=.06)
                marker.scale.x = marker.scale.y = marker.scale.z = .08
                marker.color.r, marker.color.g, marker.color.b = color
                marker.lifetime = Duration(seconds=.2).to_msg()
                markers.markers.append(marker)
            for namespace, points, color in (
                    ('selected_segment', [a, b], (1., .5, 0.)),
                    ('projection_error', [(reference_x, 0.), (x, y)], (1., 1., 1.)),
                    ('path_heading', [(x, y), (x+.2*math.cos(heading), y+.2*math.sin(heading))],
                     (0., 1., 1.))):
                marker = self.new_marker(namespace, Marker.LINE_STRIP)
                marker.points = [Point(x=float(q[0]), y=float(q[1]), z=.06) for q in points]
                marker.scale.x = .025
                marker.color.r, marker.color.g, marker.color.b = color
                marker.lifetime = Duration(seconds=.2).to_msg()
                markers.markers.append(marker)
        self.control_points_pub.publish(markers)

    def marker_array(self, stamp=None):
        # Each topic owns its MarkerArray; DELETEALL cannot clear the ROI Marker topics.
        clear = self.new_marker('clear', Marker.POINTS, stamp)
        clear.action = Marker.DELETEALL
        return MarkerArray(markers=[clear])

    def publish_cluster_markers(self, clusters, stamp=None):
        markers = self.marker_array(stamp)
        for i, cluster in enumerate(clusters):
            marker = self.new_marker('scan_clusters', Marker.POINTS, stamp)
            marker.id = i
            marker.scale.x = marker.scale.y = self.get_parameter('cluster_point_size').value
            marker.color.r, marker.color.g, marker.color.b = colorsys.hsv_to_rgb((i*0.618034) % 1, .8, 1.)
            marker.points = [Point(x=x, y=y, z=z) for x, y, z in cluster]
            marker.lifetime = Duration(seconds=0.5).to_msg()
            markers.markers.append(marker)
        self.clusters_pub.publish(markers)

    def publish_cone_center_markers(self, centers, stamp=None):
        markers = self.marker_array(stamp)
        for i, (x, y, z) in enumerate(centers):
            marker = self.new_marker('cone_candidate_centers', Marker.SPHERE, stamp)
            marker.id = 2*i
            size = self.get_parameter('cone_center_size').value
            marker.scale.x = marker.scale.y = marker.scale.z = size
            marker.color.r = 1.0
            marker.pose.position = Point(x=x, y=y, z=z)
            marker.lifetime = Duration(seconds=0.5).to_msg()
            markers.markers.append(marker)
            label = self.new_marker('cone_candidate_labels', Marker.TEXT_VIEW_FACING, stamp)
            label.id = 2*i+1
            label.text = f'Cone Candidate {i}'  # Temporary per-frame number, not a tracking ID.
            label.pose.position = Point(x=x, y=y, z=z+0.12)
            label.scale.z = 0.10
            label.color.r = label.color.g = label.color.b = 1.0
            label.lifetime = Duration(seconds=0.5).to_msg()
            markers.markers.append(label)
        self.centers_pub.publish(markers)

    def new_marker(self, namespace, marker_type, stamp=None):
        marker = Marker()
        marker.header.frame_id = 'base_link'
        marker.header.stamp = stamp if stamp is not None else self.get_clock().now().to_msg()
        marker.ns, marker.id = namespace, 0
        marker.type, marker.action = marker_type, Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.color.a = 1.0
        marker.frame_locked = True
        return marker

    def control_log_summary(self):
        return (f'[Cone] mission={self.mission_cone} | roi_candidates={len(self.candidate_clusters)}\n'
                f'  Path       : {self.center_path_mode} | memory={self.memory_status}\n'
                f'  Path update: {self.memory_update_reason}\n'
                f'  Control    : active={self.control_engaged} | state={self.control_mode}\n'
                f'  Stop reason: {self.control_stop_reason or "None"}\n'
                f'  Projection : {self.projection_status}\n'
                f'  Boundaries : left={len(self.left_boundary)} right={len(self.right_boundary)}\n'
                f'  Geometry   : sections={len(self.center_pairs)} width={self.lane_width_mean}m '
                f'live_length={self.path_length(self.center_path):.2f}m\n'
                f'  Diagnostics: {self.diagnostic_summary()}\n'
                f'  Backup     : stored_points={len(self.path_memory.points)} '
                f'remaining={self.path_length(self.memory_local_path):.2f}m '
                f'following={self.memory_following}\n'
                f'  Command    : speed={self.command_speed:.3f}m/s '
                f'steering={math.degrees(self.command_steering):+.1f}deg')

    def diagnostic_summary(self):
        if not self.path_diagnostics:
            return 'none'
        return '\n               '.join(f'{key}: {value}' for key, value in self.path_diagnostics.items())

    @staticmethod
    def path_length(path):
        return sum(math.dist(a[:2], b[:2]) for a, b in zip(path, path[1:]))

    def publish_control_log(self):
        self.get_logger().info(self.control_log_summary())

    def publish_control_status(self):
        state = (self.mission_cone, self.control_mode, self.control_stop_reason,
                 tuple(k for k in self.path_diagnostics if k in
                       ('width_conflict', 'turn', 'gap', 'boundary_crossing', 'outside_boundary')))
        if state != self.last_logged_state:
            self.get_logger().info(f'[CONE transition] {self.last_logged_state} -> {state}')
            self.last_logged_state = state
            self.publish_control_log()
        marker = self.new_marker('control_status', Marker.TEXT_VIEW_FACING)
        b = self.roi_bounds()
        marker.pose.position = Point(x=(b['roi_x_min']+b['roi_x_max'])/2,
                                     y=b['roi_y_max']+.4, z=.3)
        marker.scale.z = .12
        marker.text = (f'Control: {self.control_mode} | mission: {self.mission_cone}\n'
                       f'stop_reason: {self.control_stop_reason or "None"}')
        if self.control_stop_reason:
            marker.color.r, marker.color.g, marker.color.b = 1., .25, .25
        elif self.control_mode in ('Path Feedback', 'Memory Path Feedback'):
            marker.color.r, marker.color.g, marker.color.b = .25, 1., .25
        else:
            marker.color.r = marker.color.g = marker.color.b = .8
        marker.lifetime = Duration(seconds=.6).to_msg()
        self.status_pub.publish(marker)

    def publish_roi_marker(self):
        b = self.roi_bounds()
        marker = self.new_marker('roi_boundary', Marker.LINE_STRIP)
        marker.scale.x = 0.03
        marker.color.r = marker.color.g = 1.0
        x0, x1, y0, y1 = (b[k] for k in ROI_DEFAULTS)
        marker.points = [Point(x=x, y=y, z=0.0) for x, y in
                         ((x0, y0), (x1, y0), (x1, y1), (x0, y1), (x0, y0))]
        self.roi_pub.publish(marker)

    def publish_roi_points(self, points, stamp=None):
        marker = self.new_marker('roi_points', Marker.POINTS, stamp)
        marker.scale.x = marker.scale.y = 0.04
        marker.color.g = 1.0
        marker.points = [Point(x=x, y=y, z=z) for x, y, z in points]
        # Same namespace/id replaces previous points; DELETE also clears empty scans.
        if not points:
            marker.action = Marker.DELETE
        marker.lifetime = Duration(seconds=0.5).to_msg()
        self.points_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RoiNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except rclpy._rclpy_pybind11.RCLError:
        if rclpy.ok():
            raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
