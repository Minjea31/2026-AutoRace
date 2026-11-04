import math
import os
import time

import pytest
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import StaticTransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from cone_lane_follower.node import RoiNode


def scan(ranges):
    msg = LaserScan()
    msg.header.frame_id = 'laser_link'
    msg.angle_min = 0.0
    msg.angle_increment = math.pi / 2
    msg.angle_max = msg.angle_min + (len(ranges)-1)*msg.angle_increment
    msg.range_min, msg.range_max = 0.01, 64.0
    msg.ranges = ranges
    return msg


def transform():
    tf = TransformStamped()
    tf.header.frame_id, tf.child_frame_id = 'base_link', 'laser_link'
    tf.transform.translation.x, tf.transform.translation.z = 0.103, -0.034
    tf.transform.rotation.w = 1.0
    return tf


def test_scan_metadata_invalid_ranges_and_sensor_limits():
    msg = scan([1., float('nan'), float('inf'), -1., 0.001, 65.])
    assert RoiNode.scan_to_points(msg) == [(1., 0., 0.)]
    msg = scan([0.01, 64.])
    msg.range_min = msg.ranges[0]  # LaserScan ranges use float32 storage.
    assert len(RoiNode.scan_to_points(msg)) == 2
    msg = scan([2., 3.])
    msg.angle_min, msg.angle_increment = -0.5, 0.25
    assert RoiNode.scan_to_points(msg)[1] == pytest.approx(
        (3*math.cos(-0.25), 3*math.sin(-0.25), 0))


def test_complete_transform_and_no_double_offset():
    tf = transform()
    assert RoiNode.transform_points([(1., 0., 0.)], tf)[0] == pytest.approx((1.103, 0., -.034))
    # A future pitch change must rotate into Z as well as translate X.
    tf.transform.rotation.y = math.sin(math.pi/4)
    tf.transform.rotation.w = math.cos(math.pi/4)
    assert RoiNode.transform_points([(1., 0., 0.)], tf)[0] == pytest.approx((.103, 0., -1.034))


@pytest.fixture
def harness():
    context = Context()
    rclpy.init(context=context, domain_id=180+os.getpid()%20)
    roi = RoiNode(context=context, parameter_overrides=[Parameter(name, value=value)
                  for name, value in dict(roi_x_min=.1, roi_x_max=3.,
                                          roi_y_min=-1.5, roi_y_max=1.5).items()])
    probe = Node('roi_test_probe', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(roi)
    executor.add_node(probe)
    points, outlines = [], []
    outline_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    subscriptions = [
        probe.create_subscription(Marker, '/cone_lane/roi_points', points.append, 1),
        probe.create_subscription(Marker, '/cone_lane/roi_marker', outlines.append, outline_qos),
    ]
    publisher = probe.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)

    def until(condition, timeout=4.):
        deadline = time.monotonic()+timeout
        while not condition():
            assert time.monotonic() < deadline, 'DDS/TF condition timed out'
            executor.spin_once(timeout_sec=.01)

    until(lambda: publisher.get_subscription_count() == 1 and roi.points_pub.get_subscription_count() == 1)
    yield roi, probe, publisher, points, outlines, until
    executor.shutdown()
    roi.destroy_node()
    probe.destroy_node()
    context.shutdown()


def test_strict_roi_and_parameter_validation(harness):
    roi, _, _, _, _, _ = harness
    values = [(x, y, -.034) for x, y in
              ((.1, 0), (3., 0), (1., -1.5), (1., 1.5), (1., 0))]
    assert roi.filter_roi(values) == [(1., 0., -.034)]
    result = roi.set_parameters_atomically([Parameter('roi_x_min', value=4.)])
    assert not result.successful
    assert roi.get_parameter('roi_x_min').value == .1
    result = roi.set_parameters_atomically([Parameter('roi_y_min', value=float('nan'))])
    assert not result.successful
    result = roi.set_parameters_atomically([
        Parameter('roi_x_min', value=4.), Parameter('roi_x_max', value=5.)])
    assert result.successful
    assert roi.filter_roi([(4.5, 0, 0), (1., 0, 0)]) == [(4.5, 0, 0)]


def test_dds_tf_markers_empty_scan_failure_and_dynamic_outline(harness):
    roi, probe, publisher, points, outlines, until = harness
    broadcaster = StaticTransformBroadcaster(probe)
    tf = transform()
    tf.header.stamp = probe.get_clock().now().to_msg()
    broadcaster.sendTransform(tf)
    until(lambda: roi.tf_buffer.can_transform('base_link', 'laser_link', rclpy.time.Time()))
    msg = scan([1., 1., 1., float('nan')])
    msg.header.stamp = probe.get_clock().now().to_msg()
    publisher.publish(msg)
    until(lambda: bool(points) and bool(outlines))
    marker = points[-1]
    assert marker.type == Marker.POINTS and marker.action == Marker.ADD
    assert marker.header.frame_id == 'base_link' and marker.header.stamp == msg.header.stamp
    assert len(marker.points) == 2  # second ray at x=.103 is also strictly inside
    assert marker.points[0].x == pytest.approx(1.103)
    assert marker.points[0].z == pytest.approx(-.034)
    assert marker.scale.x == .04 and marker.color.g == 1.
    assert marker.lifetime.nanosec == 500_000_000
    outline = outlines[-1]
    assert outline.type == Marker.LINE_STRIP and len(outline.points) == 5
    assert outline.points[0] == outline.points[-1]
    assert outline.scale.x == .03 and outline.color.r == outline.color.g == 1.
    assert roi.set_parameters_atomically([Parameter('roi_x_max', value=2.)]).successful
    until(lambda: outlines[-1].points[1].x == 2.)
    count = len(points)
    publisher.publish(scan([float('inf')]))
    until(lambda: len(points) > count)
    assert points[-1].action == Marker.DELETE
    count = len(points)
    publisher.publish(scan([10.]))  # Valid distance, but zero points in ROI.
    until(lambda: len(points) > count)
    assert points[-1].action == Marker.DELETE
    count = len(points)
    msg.header.frame_id = 'missing_laser_frame'
    publisher.publish(msg)
    until(lambda: len(points) > count)
    assert points[-1].action == Marker.DELETE
    topics = {name for name, _ in roi.get_publisher_names_and_types_by_node(roi.get_name(), '/')}
    assert not any('command' in name or 'cmd_vel' in name for name in topics)


def test_missing_tf_does_not_publish_untransformed_points(harness):
    _, _, publisher, points, _, until = harness
    publisher.publish(scan([1.]))
    until(lambda: bool(points))
    assert points[-1].action == Marker.DELETE and not points[-1].points


def test_cluster_single_multiple_and_original_index_gaps(harness):
    roi, *_ = harness
    points = [(1., .48, -.034), (1.02, .50, -.034), (1.04, .52, -.034)]
    assert roi.cluster_points(points, [10, 11, 12]) == [points]
    assert roi.calculate_cluster_center(points) == pytest.approx((1.02, .50, -.034))
    # All pairs are close, but a discarded scan ray must create a break.
    assert roi.cluster_points(points, [10, 11, 20]) == [points[:2], points[2:]]
    separate = points + [(2., -.5, -.034), (2.02, -.5, -.034)]
    assert [len(c) for c in roi.cluster_points(separate, [0, 1, 2, 3, 4])] == [3, 2]
    assert roi.cluster_points([], []) == []
    with pytest.raises(ValueError):
        roi.cluster_points(points, [1])


@pytest.mark.parametrize('distance,expected', [(.099, 1), (.1, 2), (.101, 2)])
def test_cluster_threshold_strict(harness, distance, expected):
    roi, *_ = harness
    # Start at zero so threshold equality is exactly represented by the same float.
    points = [(0., 0., 0.), (distance, 0., 0.)]
    assert len(roi.cluster_points(points, [0, 1])) == expected


def test_indices_survive_invalid_ranges_transform_and_roi(harness):
    roi, *_ = harness
    msg = scan([1., float('nan'), 1., 10., 1., float('inf'), 1.])
    msg.angle_increment = .001
    points, indices = roi.scan_to_points(msg, with_indices=True)
    assert indices == [0, 2, 3, 4, 6]
    points = roi.transform_points(points, transform())
    points, indices = roi.filter_roi(points, indices)
    assert indices == [0, 2, 4, 6]
    assert len(roi.cluster_points(points, indices)) == 4
    # Preserve measurement order even when X goes backwards; don't wrap the end to start.
    points = [(1., 0., 0.), (1.02, 0., 0.), (.99, 0., 0.)]
    assert roi.cluster_points(points, [0, 1, 99]) == [points[:2], points[2:]]


def test_cluster_count_width_filters_and_single_point_override(harness):
    roi, *_ = harness
    small = [(1., 0., 0.), (1.02, .02, 0.)]
    too_wide_x = [(1., 0., 0.), (1.29, 0., 0.)]
    too_wide_y = [(1., 0., 0.), (1., .29, 0.)]
    exact_width = [(0., 0., 0.), (.28, .28, 0.)]
    singleton = [(1., 0., 0.)]
    clusters = [small, too_wide_x, too_wide_y, exact_width, singleton]
    assert roi.filter_clusters(clusters) == [small, exact_width]
    assert len(clusters) == 5  # Raw clusters remain available for visualization.
    assert roi.set_parameters_atomically([Parameter('min_cluster_points', value=1)]).successful
    assert roi.filter_clusters([singleton]) == [singleton]
    assert roi.calculate_cluster_center(singleton) == singleton[0]
    with pytest.raises(ValueError):
        roi.calculate_cluster_center([])


@pytest.mark.parametrize('name,value', [
    ('cluster_threshold', 0.), ('cluster_threshold', float('nan')),
    ('min_cluster_points', 0), ('max_cluster_width', -1.),
    ('cluster_point_size', float('inf')), ('cone_center_size', 0.),
])
def test_invalid_cluster_parameters_are_rejected(harness, name, value):
    roi, *_ = harness
    previous = roi.get_parameter(name).value
    assert not roi.set_parameters_atomically([Parameter(name, value=value)]).successful
    assert roi.get_parameter(name).value == previous


def test_cluster_center_dds_marker_replacement_and_tf_failure(harness):
    roi, probe, publisher, _, _, until = harness
    clusters, centers = [], []
    subscriptions = [
        probe.create_subscription(MarkerArray, '/cone_lane/clusters', clusters.append, 1),
        probe.create_subscription(MarkerArray, '/cone_lane/cone_centers', centers.append, 1),
    ]
    broadcaster = StaticTransformBroadcaster(probe)
    broadcaster.sendTransform(transform())
    until(lambda: roi.tf_buffer.can_transform('base_link', 'laser_link', rclpy.time.Time())
          and roi.clusters_pub.get_subscription_count() == 1
          and roi.centers_pub.get_subscription_count() == 1)

    def send(ranges, frame='laser_link'):
        msg = scan(ranges)
        msg.angle_increment = .01
        msg.header.frame_id = frame
        msg.header.stamp = probe.get_clock().now().to_msg()
        before = (len(clusters), len(centers))
        publisher.publish(msg)
        until(lambda: len(clusters) > before[0] and len(centers) > before[1])
        return msg

    msg = send([1., 1., float('nan'), 1.2, 1.2, float('inf'), 2., 2.])
    assert len(roi.all_clusters) == len(roi.candidate_clusters) == 3
    cluster_markers = clusters[-1].markers
    center_markers = centers[-1].markers
    assert cluster_markers[0].action == center_markers[0].action == Marker.DELETEALL
    assert len(cluster_markers) == 4 and len(center_markers) == 7
    assert len({(m.ns, m.id) for m in center_markers}) == len(center_markers)
    colors = {(m.color.r, m.color.g, m.color.b) for m in cluster_markers[1:]}
    assert len(colors) == 3
    for marker in cluster_markers[1:]:
        assert marker.type == Marker.POINTS and marker.scale.x == .05
        assert len(marker.points) == 2
        assert marker.header.stamp == msg.header.stamp and marker.header.frame_id == 'base_link'
    spheres = [m for m in center_markers if m.type == Marker.SPHERE]
    assert len(spheres) == 3
    assert spheres[0].pose.position.x == pytest.approx(.103+(1+math.cos(.01))/2, abs=1e-6)
    assert spheres[0].pose.position.z == pytest.approx(-.034)
    assert all(m.scale.x == m.scale.y == m.scale.z == .08 and m.color.r == 1. for m in spheres)
    assert len([m for m in center_markers if m.type == Marker.TEXT_VIEW_FACING]) == 3

    # Model RViz replacement, including removal of IDs after 3 -> 1 -> 0 clusters.
    active = {}

    def apply(array):
        for marker in array.markers:
            if marker.action == Marker.DELETEALL:
                active.clear()
            elif marker.action == Marker.ADD:
                active[(marker.ns, marker.id)] = marker

    apply(clusters[-1])
    assert len(active) == 3
    send([1., 1.])
    apply(clusters[-1])
    assert len(active) == 1 and len(centers[-1].markers) == 3
    send([float('inf')])
    apply(clusters[-1])
    assert active == {} and len(centers[-1].markers) == 1
    send([1., 1.])
    send([1., 1.], frame='missing_tf')
    assert len(clusters[-1].markers) == len(centers[-1].markers) == 1
    assert roi.all_clusters == roi.candidate_clusters == []
    # Rejected singleton stays in all-cluster visualization, but produces no center.
    send([1.])
    assert len(clusters[-1].markers) == 2 and len(centers[-1].markers) == 1
