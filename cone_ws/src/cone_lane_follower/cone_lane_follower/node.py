"""Visualize a base_link LiDAR ROI, scan clusters, and unconfirmed cone candidates."""
import colorsys
import math
import time

import rclpy
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


ROI_DEFAULTS = dict(roi_x_min=0.1, roi_x_max=0.8, roi_y_min=-0.5, roi_y_max=0.5)
CLUSTER_DEFAULTS = dict(cluster_threshold=0.10, min_cluster_points=2,
                        max_cluster_width=0.28, cluster_point_size=0.05,
                        cone_center_size=0.08)


class RoiNode(Node):
    def __init__(self, **kwargs):
        super().__init__('cone_lane_roi', **kwargs)
        for name, default in {**ROI_DEFAULTS, **CLUSTER_DEFAULTS}.items():
            self.declare_parameter(name, default)
        reason = self.validate_roi(self.roi_bounds())
        if reason:
            raise ValueError(reason)
        reason = self.validate_cluster_parameters(
            {name: self.get_parameter(name).value for name in CLUSTER_DEFAULTS})
        if reason:
            raise ValueError(reason)
        self.add_on_set_parameters_callback(self.validate_parameters)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # Retain the outline for RViz instances that connect after startup.
        outline_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.roi_pub = self.create_publisher(Marker, '/cone_lane/roi_marker', outline_qos)
        self.points_pub = self.create_publisher(Marker, '/cone_lane/roi_points', 1)
        self.clusters_pub = self.create_publisher(MarkerArray, '/cone_lane/clusters', 1)
        self.centers_pub = self.create_publisher(MarkerArray, '/cone_lane/cone_centers', 1)
        self.all_clusters, self.candidate_clusters = [], []
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        self.last_stats = self.last_warning = -math.inf
        self.outline_timer = self.create_timer(0.2, self.publish_roi_marker)
        self.publish_roi_marker()

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
        cluster_parameters = {name: self.get_parameter(name).value for name in CLUSTER_DEFAULTS}
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
            if name == 'min_cluster_points':
                if isinstance(value, bool) or not isinstance(value, int) or value < 1:
                    return 'min_cluster_points must be an integer >= 1'
            elif (isinstance(value, bool) or not isinstance(value, (int, float))
                  or not math.isfinite(value) or value <= 0):
                return f'{name} must be finite and > 0'
        return ''

    def scan_callback(self, msg):
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
                self.get_logger().warning(f'Skipping scan: cannot transform to base_link: {exc}')
                self.last_warning = time.monotonic()
            self.publish_roi_points([], msg.header.stamp)
            self.all_clusters, self.candidate_clusters = [], []
            self.publish_cluster_markers([], msg.header.stamp)
            self.publish_cone_center_markers([], msg.header.stamp)
            return
        roi_points, roi_indices = self.filter_roi(points_base, indices)
        self.all_clusters = self.cluster_points(roi_points, roi_indices)
        self.candidate_clusters = self.filter_clusters(self.all_clusters)
        centers = [self.calculate_cluster_center(cluster) for cluster in self.candidate_clusters]
        self.publish_roi_points(roi_points, msg.header.stamp)
        self.publish_roi_marker()
        self.publish_cluster_markers(self.all_clusters, msg.header.stamp)
        self.publish_cone_center_markers(centers, msg.header.stamp)
        now = time.monotonic()
        if now - self.last_stats >= 1.0:
            summary = (
                f'Valid LiDAR Points: {len(points)}\n'
                f'ROI Points: {len(roi_points)}\n'
                f'Outside ROI Points: {len(points) - len(roi_points)}\n'
                f'Total Clusters: {len(self.all_clusters)}\n'
                f'Cone Candidates: {len(centers)}')
            details = [f'Cluster {i}: points={len(cluster)}'
                       for i, cluster in enumerate(self.all_clusters)]
            details += [f'Cone Candidate {i}: x={center[0]:.3f}, y={center[1]:.3f}, '
                        f'points={len(cluster)}'
                        for i, (center, cluster) in enumerate(zip(centers, self.candidate_clusters))]
            self.get_logger().info('\n'.join([summary, *details]))
            self.last_stats = now

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
