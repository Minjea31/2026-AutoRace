"""Visualize a base_link LiDAR ROI, scan clusters, and unconfirmed cone candidates."""
import colorsys
import math
import time

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Point
from rcl_interfaces.msg import SetParametersResult
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray
from cone_lane_follower.path_memory import PathMemory


ROI_DEFAULTS = dict(roi_x_min=-0.3, roi_x_max=1.5, roi_y_min=-0.8, roi_y_max=0.8)
CLUSTER_DEFAULTS = dict(cluster_threshold=0.10, min_cluster_points=2,
                        max_cluster_width=0.28, cluster_point_size=0.05,
                        cone_center_size=0.08)


CENTER_DEFAULTS = dict(center_sample_step=0.05, center_min_width=0.10,
                       center_max_width=1.50, center_max_gap=0.30,
                       center_max_turn=75.0, center_smoothing_weight=0.25,
                       center_width_timeout=3.0, center_width_relative_tolerance=0.25)


BOUNDARY_DEFAULTS = dict(boundary_connect_distance=0.30, boundary_max_turn=80.0,
                         boundary_initial_max_turn=110.0, boundary_direction_window=3,
                         boundary_distance_weight=1.0, boundary_direction_weight=2.0)


CONTROL_DEFAULTS = dict(control_speed=0.15, control_reference_x=0.25,
                        control_lateral_gain=2.0, control_heading_gain=1.0,
                        control_max_steering=math.radians(80.), control_max_steering_rate=1.0,
                        control_speed_reference_steering=0.48,
                        control_timeout=0.20, control_max_path_distance=0.50,
                        control_memory_speed=0.15, control_memory_goal_tolerance=0.08,
                        control_exit_timeout=1.50)


class RoiNode(Node):
    def __init__(self, **kwargs):
        super().__init__('cone_lane_roi', **kwargs)
        for name, default in {**ROI_DEFAULTS, **CLUSTER_DEFAULTS, **CENTER_DEFAULTS, **BOUNDARY_DEFAULTS, **CONTROL_DEFAULTS}.items():
            self.declare_parameter(name, default)
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
        self.center_pub = self.create_publisher(MarkerArray, '/cone_lane/center_path', 1)
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
        self.path_memory = PathMemory()
        self.memory_scan_stamp = None
        self.memory_local_path = []
        self.memory_status = 'empty'
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
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
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
                if len(result) >= 1000:
                    return []  # Reject excessive sampling rather than silently bridge a gap.
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
        for a, b, c in zip(path, path[1:], path[2:]):
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
        steady, ros = self.lane_width_updated
        return (p['center_min_width'] <= self.lane_width_mean <= p['center_max_width']
                and 0 <= time.monotonic()-steady <= p['center_width_timeout']
                and 0 <= (self.get_clock().now().nanoseconds-ros)/1e9 <= p['center_width_timeout'])

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
        self.projection_status = 'not_attempted'
        if not self.mission_cone or not self.width_mission_active:
            self.reset_lane_width()
        self.width_mission_active = self.mission_cone
        p = {name: self.get_parameter(name).value for name in CENTER_DEFAULTS}
        self.center_pairs = self.match_cross_sections(
            self.left_boundary, self.right_boundary, p['center_sample_step'],
            p['center_min_width'], p['center_max_width'])
        self.center_points = [tuple((a[k]+b[k])/2 for k in range(3)) for a, b in self.center_pairs]
        self.center_path = self.smooth_center_path(
            self.center_points, self.left_boundary, self.right_boundary,
            p['center_smoothing_weight'], p['center_max_gap'], p['center_max_turn'])
        self.center_path_mode = 'Normal Sections' if self.center_path else 'Invalid'
        # Learn perpendicular widths only from valid measured cross sections.
        # Endpoint projections can overestimate width and are excluded from learning.
        if (self.mission_cone and self.center_path and self.path_errors(
                self.center_path, self.get_parameter('control_reference_x').value,
                self.get_parameter('control_max_path_distance').value) is not None):
            widths = [math.dist(a[:2], b[:2]) for a, b in self.center_pairs]
            mean = sum(widths)/len(widths)
            if max(widths)-min(widths) <= mean*p['center_width_relative_tolerance']:
                # Expired measurements do not bias a newly observed lane width.
                if not self.lane_width_is_fresh(p):
                    self.lane_width_mean, self.lane_width_samples = None, 0
                total = (self.lane_width_mean or 0.)*self.lane_width_samples + sum(widths)
                self.lane_width_samples += len(widths)
                self.lane_width_mean = total/self.lane_width_samples
                self.lane_width_updated = (time.monotonic(), self.get_clock().now().nanoseconds)
            else:
                self.lane_width_updated = None
        if not self.center_path:
            projection_diagnostics = {}
            pairs = self.match_projected_sections(
                self.left_boundary, self.right_boundary, p['center_sample_step'],
                p['center_min_width'], p['center_max_width'], diagnostics=projection_diagnostics)
            centers = [tuple((a[k]+b[k])/2 for k in range(3)) for a, b in pairs]
            path = self.smooth_center_path(
                centers, self.left_boundary, self.right_boundary,
                p['center_smoothing_weight'], p['center_max_gap'], p['center_max_turn'],
                diagnostics=projection_diagnostics)
            if path:
                self.center_pairs, self.center_points, self.center_path = pairs, centers, path
                self.center_path_mode = 'Nearest Projection'
                self.projection_status = 'success'
            else:
                self.projection_status = 'failed: '+', '.join(
                    f'{reason}={count}' for reason, count in projection_diagnostics.items())
        if (self.mission_cone and not self.control_fault and not self.center_path
                and (len(self.left_boundary) >= 2) != (len(self.right_boundary) >= 2)):
            if not self.lane_width_is_fresh(p):
                self.center_path_mode = 'Width Unavailable or Stale'
                self.publish_center_path_markers(stamp)
                return
            left_visible = len(self.left_boundary) >= 2
            boundary = self.left_boundary if left_visible else self.right_boundary
            signed_width = -self.lane_width_mean if left_visible else self.lane_width_mean
            refs = self.sample_boundary(boundary, p['center_sample_step'])
            opposite = self.offset_boundary(boundary, signed_width, p['center_sample_step'])
            observed_opposite = self.right_boundary if left_visible else self.left_boundary
            if observed_opposite and not self.point_near_boundary(
                    observed_opposite[0], opposite,
                    self.lane_width_mean*p['center_width_relative_tolerance']):
                self.center_path_mode = 'Boundary Width Conflict'
                self.publish_center_path_markers(stamp)
                return
            pairs = list(zip(refs, opposite)) if left_visible else list(zip(opposite, refs))
            centers = [tuple((a[k]+b[k])/2 for k in range(3)) for a, b in pairs]
            path = self.smooth_center_path(
                centers, refs if left_visible else opposite, opposite if left_visible else refs,
                p['center_smoothing_weight'], p['center_max_gap'], p['center_max_turn'])
            if path:
                self.center_pairs, self.center_points, self.center_path = pairs, centers, path
                self.center_path_mode = 'Left Boundary Offset' if left_visible else 'Right Boundary Offset'
        self.publish_center_path_markers(stamp)

    def publish_center_path_markers(self, stamp):
        markers = self.marker_array(stamp)
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
            markers.markers.append(marker)
        self.center_pub.publish(markers)

    @staticmethod
    def path_errors(path, reference_x, max_distance=0.50):
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
            choices.append((math.hypot(x-reference_x, y), lateral, math.atan2(dy, dx)))
        if not choices:
            return None
        distance, lateral, heading = min(choices)
        return (lateral, heading) if distance <= max_distance else None

    def publish_drive(self, speed, steering):
        message = AckermannDriveStamped()
        message.header.frame_id = 'base_link'
        message.header.stamp = self.get_clock().now().to_msg()
        message.drive.speed, message.drive.steering_angle = float(speed), float(steering)
        self.command_pub.publish(message)

    def clear_path_memory(self):
        self.path_memory.clear()
        self.memory_scan_stamp = None
        self.memory_local_path = []
        self.memory_status = 'empty'
        self.memory_following = False

    def memory_transform(self, target, source, stamp):
        tf = self.tf_buffer.lookup_transform(target, source, stamp)
        age = (self.get_clock().now().nanoseconds-Time.from_msg(tf.header.stamp).nanoseconds)/1e9
        if (tf.header.frame_id != target or tf.child_frame_id != source
                or not 0 <= age <= self.get_parameter('control_timeout').value):
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

    def remember_live_path(self):
        if self.memory_scan_stamp == self.control_scan_stamp:
            return
        try:
            tf = self.memory_transform('odom', 'base_link',
                                       Time(nanoseconds=self.control_scan_stamp))
            points = self.transform_points(self.center_path, tf)
        except (TransformException, ValueError):
            self.memory_status = 'save_tf_unavailable_or_stale'
            return
        if self.path_memory.store(points):
            self.memory_scan_stamp = self.control_scan_stamp
            self.memory_status = 'stored'

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
            self.clear_path_memory()
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
            self.stop_control(reason)
            return
        errors = self.path_errors(self.center_path, p['control_reference_x'], p['control_max_path_distance'])
        memory_errors, memory_reason = None, None
        use_memory = not self.center_path
        if self.memory_following and errors is not None:
            remaining, memory_reason = self.remaining_memory_path(p)
            previous = self.path_errors(remaining, p['control_reference_x'], p['control_max_path_distance'])
            if previous is not None:
                heading_jump = math.atan2(math.sin(errors[1]-previous[1]), math.cos(errors[1]-previous[1]))
                if abs(errors[0]-previous[0]) > .25 or abs(heading_jump) > math.pi/4:
                    use_memory = True
                    self.memory_status = 'rejoin_rejected'
        self.memory_local_path = []
        if use_memory and self.path_memory.points:
            remaining, memory_reason = self.remaining_memory_path(p)
            memory_errors = self.path_errors(remaining, p['control_reference_x'], p['control_max_path_distance'])
            if memory_errors is not None:
                self.memory_local_path = remaining
                errors = memory_errors
                if self.memory_status != 'rejoin_rejected':
                    self.memory_status = 'following'
            else:
                errors = None
                self.memory_status = memory_reason or 'memory_path_unusable'
        self.memory_following = memory_errors is not None
        if errors is not None and not self.memory_following:
            self.remember_live_path()
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
        speed = p['control_speed']*(1.-.7*slowdown)
        if self.memory_following:
            speed = min(speed, p['control_memory_speed'])
        self.control_mode = 'Memory Path Feedback' if self.memory_following else 'Path Feedback'
        self.control_stop_reason = None
        self.publish_drive(speed, self.last_steering)

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
                f'  Control    : active={self.control_engaged} | state={self.control_mode}\n'
                f'  Stop reason: {self.control_stop_reason or "None"}\n'
                f'  Projection : {self.projection_status}')

    def publish_control_log(self):
        self.get_logger().info(self.control_log_summary())

    def publish_control_status(self):
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
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
