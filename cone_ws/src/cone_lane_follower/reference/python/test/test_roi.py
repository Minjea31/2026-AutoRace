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

from cone_lane_follower.node import BOUNDARY_DEFAULTS, RoiNode


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
    assert {name for name in topics if 'command' in name or 'cmd_vel' in name} == {'/mission/cone/command_06'}


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


@pytest.mark.parametrize('count', [0, 5, 6, 7])
def test_mission_cone_candidate_count_entry_and_fault_stop(harness, monkeypatch, count):
    roi, *_ = harness
    assert roi.mission_cone is False
    class IdentityTF:
        def lookup_transform(self, *args):
            return transform()
    monkeypatch.setattr(roi, 'tf_buffer', IdentityTF())
    candidates = [[(1., .2, 0.), (1.01, .2, 0.)] for _ in range(count)]
    monkeypatch.setattr(roi, 'filter_clusters', lambda clusters: candidates)
    roi.scan_callback(scan([1.]))
    assert roi.mission_cone is (count > 5)
    candidates[:] = candidates[:6]
    roi.scan_callback(scan([1.]))
    assert roi.mission_cone is (count > 5)
    roi.mission_cone = True
    msg = scan([1.])
    msg.header.frame_id = ''
    roi.scan_callback(msg)
    assert roi.mission_cone and roi.control_fault


def test_mission_keeps_driving_with_fewer_candidates_and_stops_on_path_loss(harness, monkeypatch):
    roi, *_ = harness
    messages = []

    class IdentityTF:
        def lookup_transform(self, *args):
            return transform()

    class Capture:
        def publish(self, msg):
            messages.append(msg)

    monkeypatch.setattr(roi, 'tf_buffer', IdentityTF())
    roi.command_pub = Capture()
    centers = [(.2, .2), (.4, .2), (.6, .2), (.8, .2),
               (.2, -.2), (.4, -.2), (.6, -.2)]
    candidates = [[(x, y, 0.), (x+.01, y, 0.)] for x, y in centers]
    monkeypatch.setattr(roi, 'filter_clusters', lambda clusters: candidates)

    def send():
        msg = scan([1.])
        msg.header.stamp = roi.get_clock().now().to_msg()
        roi.scan_callback(msg)
        roi.control_tick()

    send()
    assert roi.mission_cone and roi.control_mode == 'Path Feedback'
    candidates.pop(3)
    candidates.pop(2)
    send()
    assert len(candidates) == 5
    assert roi.left_boundary and roi.right_boundary and roi.center_path
    assert roi.mission_cone and roi.control_mode == 'Path Feedback'
    assert messages[-1].drive.speed > 0.

    saved_candidates = candidates[:]
    candidates.clear()
    send()
    assert roi.mission_cone and not roi.center_path
    assert roi.control_mode == 'Stopped'
    assert messages[-1].drive.speed == messages[-1].drive.steering_angle == 0.
    assert roi.control_stop_reason == 'empty_path'
    assert roi.mission_cone

    candidates.extend(saved_candidates)
    send()
    assert roi.control_mode == 'Path Feedback'
    assert messages[-1].drive.speed > 0.


def test_roi_limits_mission_count_but_not_boundary_generation(harness, monkeypatch):
    roi, *_ = harness

    class IdentityTF:
        def lookup_transform(self, *args):
            tf = transform()
            tf.transform.translation.x = 0.
            return tf

    monkeypatch.setattr(roi, 'tf_buffer', IdentityTF())
    points, indices = [], []
    for i, (x, y) in enumerate((x, y) for y in [.3, -.3] for x in [.2, .45, .7, .95]):
        points.extend([(x, y, 0.), (x+.01, y, 0.)])
        indices.extend([4*i, 4*i+1])
    monkeypatch.setattr(roi, 'scan_to_points', lambda msg, with_indices: (points, indices))

    def send():
        msg = scan([1.])
        msg.header.stamp = roi.get_clock().now().to_msg()
        roi.scan_callback(msg)

    roi.set_parameters_atomically([Parameter('roi_x_max', value=.65)])
    send()
    assert len(roi.candidate_clusters) == 4
    assert len(roi.boundary_candidate_clusters) == 8
    assert not roi.mission_cone  # Out-of-ROI cones cannot trigger entry.
    roi.set_parameters_atomically([Parameter('roi_x_max', value=1.1)])
    send()
    assert roi.mission_cone and roi.center_path
    left, right = roi.left_boundary[:], roi.right_boundary[:]
    roi.set_parameters_atomically([Parameter('roi_x_max', value=.3)])
    send()
    assert len(roi.candidate_clusters) == 2
    assert len(roi.boundary_candidate_clusters) == 8
    assert roi.left_boundary == left and roi.right_boundary == right
    assert max(p[0] for p in roi.left_boundary) > .9
    roi.control_tick()
    assert roi.mission_cone and roi.control_mode == 'Path Feedback'
    msg = scan([1.])
    msg.header.frame_id = ''
    roi.scan_callback(msg)
    assert not roi.boundary_candidate_clusters and not roi.candidate_clusters
    assert not roi.left_boundary and not roi.right_boundary and not roi.center_path


@pytest.mark.parametrize('side', [-1., 1.])
def test_boundary_seed_prefers_front_sector_then_distance(side):
    behind = (-.1, side*.02, 0.)
    closest_preferred = (.2, side*.1, 0.)
    farther_preferred = (.4, side*.2, 0.)
    rear_side = (-.05, side*.1, 0.)  # Inside the old +/-135 sector, outside +/-90.
    chain = RoiNode.select_boundary_chain([farther_preferred, behind, rear_side, closest_preferred])
    assert chain[0] == closest_preferred
    # With no preferred candidate, the closest rear candidate is still selected.
    assert RoiNode.select_boundary_chain([(-.4, side*.05, 0.), behind])[0] == behind
    # The angular boundary is inclusive on either side.
    edge = (0., side*.2, 0.)
    assert RoiNode.select_boundary_chain([behind, edge])[0] == edge


def test_boundary_seed_sector_does_not_filter_later_connections():
    seed, behind = (0., .2, 0.), (-.1, .19, 0.)
    parameters = dict(boundary_initial_max_turn=180.)
    assert RoiNode.select_boundary_chain([behind, seed], parameters) == [seed, behind]


@pytest.mark.parametrize('mirror', [False, True])
def test_joint_boundaries_keep_identity_across_y_zero(harness, mirror):
    roi, *_ = harness
    left = [(.1, .25, 0.), (.3, .25, 0.), (.5, .2, 0.), (.7, .05, 0.), (.85, -.15, 0.)]
    right = [(.1, -.35, 0.), (.3, -.35, 0.), (.5, -.4, 0.), (.7, -.55, 0.), (.85, -.75, 0.)]
    if mirror:
        left, right = ([(x, -y, z) for x, y, z in right],
                       [(x, -y, z) for x, y, z in left])
    roi.mission_cone = True
    roi.update_boundary_markers(list(reversed(left+right)), roi.get_clock().now().to_msg())
    assert roi.left_boundary == left
    assert roi.right_boundary == right
    assert not set(roi.left_boundary) & set(roi.right_boundary)
    assert any(q[1] > 0 for q in roi.right_boundary) if mirror else any(q[1] < 0 for q in roi.left_boundary)


def test_joint_boundary_seeds_keep_front_preference_and_fallback():
    p = dict(BOUNDARY_DEFAULTS)
    left, right = RoiNode.select_boundaries([(-.1, .01, 0.), (.2, .2, 0.), (-.1, -.01, 0.)], p)
    assert left[0] == (.2, .2, 0.)
    assert right[0] == (-.1, -.01, 0.)


def test_joint_boundaries_reject_width_conflict_and_duplicate_assignment():
    p = dict(BOUNDARY_DEFAULTS, boundary_connect_distance=.5)
    left = [(.1, .3, 0.), (.3, .3, 0.), (.5, .3, 0.)]
    right = [(.1, -.3, 0.), (.3, -.3, 0.), (.5, -.3, 0.)]
    a, b = RoiNode.select_boundaries(left+right+[(.4, 0., 0.)], p, .6, .25)
    assert a == left and b == right
    assert not set(a) & set(b)
    assert not any(RoiNode.segments_cross(x, y, u, v)
                   for x, y in zip(a, a[1:]) for u, v in zip(b, b[1:]))
    assert RoiNode.boundary_width_matches((.2, .3, 0.), right, 1., .6, .25)
    assert not RoiNode.boundary_width_matches((.2, 0., 0.), right, 1., .6, .25)
    assert not RoiNode.boundary_width_matches((.2, -.5, 0.), right, 1., .6, .25)


def test_boundary_chain_direction_priority_and_distance_limit():
    seed, near, origin = (.2, -.2, 0.), (.3, -.2, 0.), (.1, -.3, 0.)
    assert RoiNode.select_boundary_chain([near, origin, seed]) == [seed, near]
    assert RoiNode.select_boundary_chain([]) == []
    assert RoiNode.select_boundary_chain([(0., 0., 0.), (.3, 0., 0.)]) == [(0., 0., 0.), (.3, 0., 0.)]
    assert RoiNode.select_boundary_chain([seed, (1., -.2, 0.), seed]) == [seed]


def test_mission_boundary_sides_colors_and_clearing(harness):
    roi, *_ = harness
    class Capture:
        def publish(self, array):
            self.array = array
    capture = Capture()
    roi.boundaries_pub = capture
    centers = [(.2, -.2, 0.), (.3, -.2, 0.), (.2, .2, 0.), (.3, .2, 0.), (.1, 0., 0.)]
    stamp = roi.get_clock().now().to_msg()
    roi.update_boundary_markers(centers, stamp)
    assert len(capture.array.markers) == 1
    roi.mission_cone = True
    roi.update_boundary_markers(centers, stamp)
    assert roi.left_boundary == centers[2:4] and roi.right_boundary == centers[:2]
    clear, left, right = capture.array.markers
    assert clear.action == Marker.DELETEALL
    assert left.type == right.type == Marker.LINE_STRIP
    assert left.color.r == right.color.b == 1. and left.color.b == right.color.r == 0.
    roi.update_boundary_markers([centers[0]], stamp)
    assert capture.array.markers[1].type == Marker.POINTS
    roi.mission_cone = False
    roi.update_boundary_markers(centers, stamp)
    assert not roi.left_boundary and len(capture.array.markers) == 1
    roi.mission_cone = True
    msg = scan([1.])
    msg.header.frame_id = ''
    roi.scan_callback(msg)
    assert roi.mission_cone and roi.control_fault and len(capture.array.markers) == 1


def test_boundary_chain_extends_from_last_point_and_stops_at_gap():
    points = [(x, -.2, 0.) for x in (.1, .3, .5, .7, 1.2)]
    assert RoiNode.select_boundary_chain(list(reversed(points))+[points[1]]) == points[:4]


def test_long_boundary_is_line_strip(harness):
    roi, *_ = harness
    class Capture:
        def publish(self, array):
            self.array = array
    capture = Capture()
    roi.boundaries_pub = capture
    roi.mission_cone = True
    left = [(x, .2, 0.) for x in (.1, .3, .5, .7)]
    right = [(x, -.2, 0.) for x in (.1, .3, .5, .7)]
    roi.update_boundary_markers(left+right, roi.get_clock().now().to_msg())
    assert roi.left_boundary == left and roi.right_boundary == right
    assert all(m.type == Marker.LINE_STRIP and len(m.points) == 4 and m.scale.x == .03
               for m in capture.array.markers[1:])


def test_cross_sections_staggered_spacing_and_order():
    left = [(x, .4, 0.) for x in (.2, .8, 1.4)]
    right = [(x, -.4, 0.) for x in (.3, 1., 1.6)]
    pairs = RoiNode.match_cross_sections(left, right, .05, .1, 1.5)
    assert len(pairs) > 10
    assert all(abs(a[0]-b[0]) < 1e-8 for a, b in pairs)
    centers = [tuple((a[k]+b[k])/2 for k in range(3)) for a, b in pairs]
    path = RoiNode.smooth_center_path(centers, left, right, .25, .3, 75.)
    assert len(path) > 10 and all(abs(p[1]) < 1e-8 for p in path)
    assert all(a[0] < b[0] for a, b in zip(path, path[1:]))


def test_cross_sections_curve_and_invalid_inputs():
    # ROS naming: left is the inner side of this left turn.
    left = [(1.6*math.sin(t), 2.-1.6*math.cos(t), 0.) for t in [.1*i for i in range(12)]]
    right = [(2.4*math.sin(t), 2.-2.4*math.cos(t), 0.) for t in [.1*i for i in range(12)]]
    pairs = RoiNode.match_cross_sections(left, right, .05, .1, 1.5)
    centers = [tuple((a[k]+b[k])/2 for k in range(3)) for a, b in pairs]
    path = RoiNode.smooth_center_path(centers, left, right, .25, .3, 75.)
    assert len(path) > 10
    assert max(abs(math.hypot(p[0], p[1]-2.)-2.) for p in path) < .04
    assert RoiNode.match_cross_sections(left, [], .05, .1, 1.5) == []
    assert RoiNode.match_cross_sections(left, right, .05, 2., 3.) == []
    assert RoiNode.smooth_center_path([(0., 0., 0.), (1., 0., 0.)],
        [(0., -.5, 0.), (1., .5, 0.)], [(0., .5, 0.), (1., -.5, 0.)], .25, .3, 75.) == []


def test_center_output_clears_when_mission_stops(harness):
    roi, *_ = harness
    class Capture:
        def publish(self, message):
            self.message = message
    output = Capture()
    roi.center_pub = output
    roi.mission_cone = True
    cones = [(x, y, 0.) for y in (-.3, .3) for x in (.2, .4, .6, .8)]
    roi.update_boundary_markers(cones, roi.get_clock().now().to_msg())
    assert roi.center_path
    assert any(m.ns == 'center_path' and m.color.g == 1. for m in output.message.markers)
    roi.mission_cone = False
    roi.update_boundary_markers(cones, roi.get_clock().now().to_msg())
    assert not roi.center_path and not roi.center_pairs
    assert len(output.message.markers) == 1 and output.message.markers[0].action == Marker.DELETEALL


def test_direction_cost_prefers_forward_over_nearer_sharp_candidate():
    seed, bad, forward = (.1, -.2, 0.), (.12, -.26, 0.), (.28, -.2, 0.)
    assert RoiNode.select_boundary_chain([bad, forward, seed]) == [seed, forward]
    # A separate first-link limit permits/rejects a lateral startup.
    lateral = [(0., .2, 0.), (0., .4, 0.)]
    assert len(RoiNode.select_boundary_chain(lateral)) == 2
    assert len(RoiNode.select_boundary_chain(lateral, dict(boundary_initial_max_turn=60.))) == 1


def test_direction_follows_curve_with_decreasing_x():
    chain = [(.1, -.3, 0.)]
    for angle in (0, 30, 60, 90, 120):
        a = math.radians(angle)
        x, y, z = chain[-1]
        chain.append((x+.2*math.cos(a), y+.2*math.sin(a), z))
    assert RoiNode.select_boundary_chain(list(reversed(chain))) == chain
    assert chain[-1][0] < chain[-2][0]


def test_boundary_crossing_candidate_is_rejected():
    chain = RoiNode.select_boundary_chain([(.1, 0., 0.), (.3, 0., 0.)],
        opposite=[(.2, -.2, 0.), (.2, .2, 0.)])
    assert chain == [(.1, 0., 0.)]


@pytest.mark.parametrize('name,value', [('boundary_direction_window', 0),
    ('boundary_connect_distance', 0.), ('boundary_max_turn', 180.),
    ('boundary_initial_max_turn', 181.), ('boundary_direction_weight', -1.)])
def test_invalid_direction_parameters(harness, name, value):
    roi, *_ = harness
    assert not roi.set_parameters_atomically([Parameter(name, value=value)]).successful


@pytest.mark.parametrize('offset,sign', [(0., 0), (.1, 1), (-.1, -1)])
def test_control_signed_errors(offset, sign):
    errors = RoiNode.path_errors([(.1, offset, 0.), (.8, offset, 0.)], .25)
    assert errors == pytest.approx((offset, 0.))
    if sign:
        assert errors[0]*sign > 0


@pytest.mark.parametrize('path,reason', [
    ([], 'empty_path'),
    ([(.1, 0., 0.)], 'path_too_few_points'),
    ([(.1, 0., 0.), (float('nan'), 0., 0.)], 'path_nonfinite_coordinates'),
    ([(.1, 0., 0.), (.1, .2, 0.)], 'path_no_usable_forward_segment'),
    ([(-.8, 0., 0.), (-.1, 0., 0.)], 'path_no_usable_forward_segment'),
    ([(.1, .8, 0.), (.8, .8, 0.)], 'path_too_far_from_reference'),
])
def test_path_stop_reasons(path, reason):
    assert RoiNode.path_errors(path, .25) is None
    assert RoiNode.path_stop_reason(path, .25) == reason


def test_stop_reason_logs_only_on_change_and_resets_on_recovery(harness, monkeypatch):
    roi, *_ = harness
    warnings = []

    class Logger:
        def warning(self, message):
            warnings.append(message)

    monkeypatch.setattr(roi, 'get_logger', lambda: Logger())
    roi.stop_control('scan_receive_timeout')
    roi.stop_control('scan_receive_timeout')
    assert len(warnings) == 1
    roi.stop_control('tf_error')
    assert len(warnings) == 2 and 'reason=tf_error' in warnings[-1]
    roi.mission_cone = roi.control_engaged = True
    roi.control_received = time.monotonic()
    roi.control_scan_stamp = roi.get_clock().now().nanoseconds
    roi.center_path = [(.1, 0., 0.), (.8, 0., 0.)]
    roi.control_tick()
    assert roi.control_stop_reason is None
    roi.stop_control('tf_error')
    assert len(warnings) == 3


@pytest.mark.parametrize('side', ['left', 'right'])
def test_learn_lane_width_and_drive_with_one_boundary(harness, side):
    roi, *_ = harness
    roi.mission_cone = True
    roi.left_boundary = [(.1, .3, 0.), (.45, .3, 0.), (.8, .3, 0.)]
    roi.right_boundary = [(.1, -.3, 0.), (.45, -.3, 0.), (.8, -.3, 0.)]
    stamp = roi.get_clock().now().to_msg()
    roi.update_center_path(stamp)
    assert roi.lane_width_mean == pytest.approx(.6)
    samples = roi.lane_width_samples
    assert samples > 0
    # Learn a second measured width; each perpendicular sample has equal weight.
    roi.left_boundary = [(.1, .4, 0.), (.45, .4, 0.), (.8, .4, 0.)]
    roi.right_boundary = [(.1, -.4, 0.), (.45, -.4, 0.), (.8, -.4, 0.)]
    roi.update_center_path(stamp)
    assert roi.lane_width_mean == pytest.approx(.7)
    assert roi.lane_width_samples == 2*samples
    if side == 'left':
        roi.left_boundary = [(.1, .35, 0.), (.45, .35, 0.), (.8, .35, 0.)]
        roi.right_boundary = []
    else:
        roi.left_boundary = []
        roi.right_boundary = [(.1, -.35, 0.), (.45, -.35, 0.), (.8, -.35, 0.)]
    roi.update_center_path(stamp)
    assert roi.center_path_mode == 'Unified'
    assert roi.center_path and all(abs(q[1]) < 1e-8 for q in roi.center_path)
    assert roi.lane_width_mean == pytest.approx(.7)
    assert roi.lane_width_samples == 2*samples
    roi.control_received = time.monotonic()
    roi.control_scan_stamp = roi.get_clock().now().nanoseconds
    roi.update_control_activation()
    roi.control_tick()
    assert roi.control_mode == 'Path Feedback'
    roi.left_boundary = roi.right_boundary = []
    roi.update_center_path(stamp)
    assert not roi.center_path
    roi.mission_cone = False
    roi.update_center_path(stamp)
    assert roi.lane_width_mean is None and roi.lane_width_samples == 0
    roi.mission_cone = True
    roi.left_boundary = [(.1, .35, 0.), (.45, .35, 0.), (.8, .35, 0.)]
    roi.update_center_path(stamp)
    assert not roi.center_path  # A new mission cannot reuse the old learned width.


@pytest.mark.parametrize('visible_side', ['left', 'right'])
@pytest.mark.parametrize('failure', [None, 'stale', 'clock_reverse', 'conflict', 'missing'])
def test_single_opposite_point_checks_width_without_expiry(harness, visible_side, failure):
    roi, *_ = harness
    roi.mission_cone = True
    stamp = roi.get_clock().now().to_msg()
    roi.left_boundary = [(.1, .3, 0.), (.45, .3, 0.), (.8, .3, 0.)]
    roi.right_boundary = [(.1, -.3, 0.), (.45, -.3, 0.), (.8, -.3, 0.)]
    roi.update_center_path(stamp)
    assert roi.lane_width_mean == pytest.approx(.6)
    if visible_side == 'left':
        roi.right_boundary = [(.4, -.3, 0.)]
    else:
        roi.left_boundary = [(.4, .3, 0.)]
    if failure == 'stale':
        steady, ros = roi.lane_width_updated
        roi.lane_width_updated = (steady-30.1, ros-30_100_000_000)
    elif failure == 'clock_reverse':
        steady, ros = roi.lane_width_updated
        roi.lane_width_updated = (steady, ros+1_000_000_000)
    elif failure == 'missing':
        roi.lane_width_mean = None
    elif failure == 'conflict':
        if visible_side == 'left':
            roi.right_boundary = [(.4, -.8, 0.)]
        else:
            roi.left_boundary = [(.4, .8, 0.)]
    samples = roi.lane_width_samples
    roi.update_center_path(stamp)
    assert roi.lane_width_samples == samples  # Inference must never train itself.
    roi.control_received = time.monotonic()
    roi.control_scan_stamp = roi.get_clock().now().nanoseconds
    roi.update_control_activation()
    roi.control_tick()
    if failure in ('conflict', 'missing'):
        assert not roi.center_path and roi.control_mode == 'Stopped'
        expected = 'boundary_width_conflict' if failure == 'conflict' else 'empty_path'
        assert roi.control_stop_reason == expected
    else:
        assert roi.center_path_mode == 'Unified'
        assert all(abs(q[1]) < 1e-8 for q in roi.center_path)
        assert roi.control_mode == 'Path Feedback'


def test_inconsistent_cross_sections_are_not_learned(harness, monkeypatch):
    roi, *_ = harness
    roi.mission_cone = True
    roi.left_boundary = [(.1, .3, 0.), (.3, .3, 0.), (.5, .3, 0.)]
    roi.right_boundary = [(.1, -.3, 0.), (.3, -.3, 0.), (.5, -.8, 0.)]
    pairs = [((.1, .3, 0.), (.1, -.3, 0.)), ((.5, .3, 0.), (.5, -.8, 0.))]
    monkeypatch.setattr(roi, 'match_cross_sections', lambda *args: pairs)
    monkeypatch.setattr(roi, 'smooth_center_path', lambda *args: [(.1, 0., 0.), (.5, 0., 0.)])
    roi.update_center_path(roi.get_clock().now().to_msg())
    assert roi.lane_width_mean is None and roi.lane_width_updated is None




def test_control_log_summary_contains_only_requested_status(harness):
    roi, *_ = harness
    roi.mission_cone = True
    roi.candidate_clusters = [[], []]
    roi.left_boundary = [(.1, .3, 0.)]
    roi.right_boundary = [(.1, -.3, 0.), (.45, -.3, 0.), (.8, -.3, 0.)]
    roi.update_center_path(roi.get_clock().now().to_msg())
    summary = roi.control_log_summary()
    assert 'mission=True | roi_candidates=2' in summary
    assert 'Path       : Invalid' in summary
    assert 'Control    : active=' in summary and 'state=' in summary
    assert 'Stop reason:' in summary
    assert 'Projection : not_attempted' in summary
    assert 'LiDAR Points' not in summary and 'Cone Candidate' not in summary
    roi.left_boundary = [(.1, .3, 0.), (.45, .3, 0.), (.8, .3, 0.)]
    roi.update_center_path(roi.get_clock().now().to_msg())
    assert roi.projection_status == 'not_attempted'  # No stale failure after recovery.


@pytest.mark.parametrize('left_count,right_count', [(3, 3), (4, 3), (3, 2), (2, 3),
    (3, 1), (1, 3), (3, 0), (0, 3), (2, 2), (2, 1), (0, 0)])
def test_three_point_boundary_selection_rules(harness, left_count, right_count):
    roi, *_ = harness
    roi.mission_cone = True
    stamp = roi.get_clock().now().to_msg()
    left = [(x, .3, 0.) for x in (.1, .3, .5, .7)]
    right = [(x, -.3, 0.) for x in (.1, .3, .5, .7)]
    roi.left_boundary, roi.right_boundary = left, right
    roi.update_center_path(stamp)
    assert roi.lane_width_mean == pytest.approx(.6)
    roi.left_boundary, roi.right_boundary = left[:left_count], right[:right_count]
    roi.update_center_path(stamp)
    if left_count >= 3 and right_count >= 3:
        assert roi.center_path_mode == 'Unified' and roi.center_path
    elif left_count >= 3 or right_count >= 3:
        expected = 'Unified'
        assert roi.center_path_mode == expected and roi.center_path
        assert roi.projection_status == 'not_attempted'
    else:
        assert not roi.center_path
        assert roi.projection_status == 'not_attempted'


@pytest.mark.parametrize('visible_side', ['left', 'right'])
def test_both_remaining_opposite_points_must_match_width(harness, visible_side):
    roi, *_ = harness
    roi.mission_cone = True
    stamp = roi.get_clock().now().to_msg()
    roi.left_boundary = [(.1, .3, 0.), (.4, .3, 0.), (.8, .3, 0.)]
    roi.right_boundary = [(.1, -.3, 0.), (.4, -.3, 0.), (.8, -.3, 0.)]
    roi.update_center_path(stamp)
    if visible_side == 'left':
        roi.right_boundary = [(.2, -.3, 0.), (.6, -.8, 0.)]
    else:
        roi.left_boundary = [(.2, .3, 0.), (.6, .8, 0.)]
    roi.update_center_path(stamp)
    assert not roi.center_path and roi.center_path_mode == 'Boundary Width Conflict'


@pytest.mark.parametrize('age,accepted', [(10., True), (31., True), (300., True)])
def test_trusted_lane_width_survives_longer_single_boundary_loss(harness, age, accepted):
    roi, *_ = harness
    roi.mission_cone = True
    stamp = roi.get_clock().now().to_msg()
    roi.left_boundary = [(x, .3, 0.) for x in (.1, .4, .8)]
    roi.right_boundary = [(x, -.3, 0.) for x in (.1, .4, .8)]
    roi.update_center_path(stamp)
    steady, ros = roi.lane_width_updated
    aged = (steady-age, ros-int(age*1e9))
    roi.lane_width_updated = aged
    roi.right_boundary = []
    roi.update_center_path(stamp)
    assert bool(roi.center_path) == accepted
    assert roi.lane_width_mean == pytest.approx(.6)
    assert roi.lane_width_updated == aged  # Using an old width never refreshes its age.


def test_noisy_measurement_preserves_last_trusted_width_and_timestamp(harness, monkeypatch):
    roi, *_ = harness
    roi.mission_cone = True
    stamp = roi.get_clock().now().to_msg()
    roi.left_boundary = [(x, .3, 0.) for x in (.1, .4, .8)]
    roi.right_boundary = [(x, -.3, 0.) for x in (.1, .4, .8)]
    roi.update_center_path(stamp)
    trusted = roi.lane_width_mean, roi.lane_width_samples, roi.lane_width_updated
    pairs = [((.1, .3, 0.), (.1, -.3, 0.)), ((.5, .3, 0.), (.5, -.8, 0.))]
    monkeypatch.setattr(roi, 'match_cross_sections', lambda *args: pairs)
    monkeypatch.setattr(roi, 'smooth_center_path', lambda *args: [(.1, 0., 0.), (.5, 0., 0.)])
    roi.update_center_path(stamp)
    assert (roi.lane_width_mean, roi.lane_width_samples, roi.lane_width_updated) == trusted
    roi.mission_cone = False
    roi.update_center_path(stamp)
    assert roi.lane_width_mean is None and roi.lane_width_updated is None


def test_boundary_offset_follows_local_normal():
    boundary = [(.1, .3, 0.), (.3, .5, 0.), (.6, .6, 0.)]
    refs = RoiNode.sample_boundary(boundary, .05)
    shifted = RoiNode.offset_boundary(boundary, -.3, .05)
    assert len(refs) == len(shifted)
    for i, (q, p) in enumerate(zip(refs, shifted)):
        a, b = refs[max(0, i-1)], refs[min(len(refs)-1, i+1)]
        assert math.dist(q[:2], p[:2]) == pytest.approx(.3)
        assert (p[0]-q[0])*(b[0]-a[0])+(p[1]-q[1])*(b[1]-a[1]) == pytest.approx(0.)
        assert p[0] > q[0] and p[1] < q[1]


def test_rviz_control_status_updates_reason_and_color(harness):
    roi, *_ = harness
    messages = []

    class Capture:
        def publish(self, msg):
            messages.append(msg)

    roi.status_pub = Capture()
    roi.publish_control_status()
    marker = messages[-1]
    assert marker.type == Marker.TEXT_VIEW_FACING
    assert marker.header.frame_id == 'base_link' and marker.frame_locked
    assert 'Control: Idle' in marker.text and 'stop_reason: None' in marker.text
    roi.stop_control('path_no_usable_forward_segment')
    roi.publish_control_status()
    marker = messages[-1]
    assert 'Control: Stopped' in marker.text
    assert 'stop_reason: path_no_usable_forward_segment' in marker.text
    assert marker.color.r > marker.color.g
    roi.control_mode = 'Path Feedback'
    roi.control_stop_reason = None
    roi.publish_control_status()
    marker = messages[-1]
    assert 'stop_reason: None' in marker.text
    assert marker.color.g > marker.color.r
    assert marker.ns == messages[0].ns and marker.id == messages[0].id


def test_control_heading_and_invalid_path():
    assert RoiNode.path_errors([(.1, -.1, 0.), (.4, .1, 0.)], .25)[1] > 0
    assert RoiNode.path_errors([], .25) is None
    assert RoiNode.path_errors([(.8, 0., 0.), (.1, 0., 0.)], .25) is None
    assert RoiNode.path_errors([(2., 0., 0.), (3., 0., 0.)], .25) is None
    assert RoiNode.path_errors([(.1, float('nan'), 0.), (.5, 0., 0.)], .25) is None


def test_control_message_limits_stop_release_and_watchdog(harness):
    import time
    from ackermann_msgs.msg import AckermannDriveStamped
    roi, *_ = harness
    class Capture:
        def __init__(self):
            self.messages = []
        def publish(self, msg):
            self.messages.append(msg)
    capture = Capture()
    roi.command_pub = capture
    roi.control_tick()
    assert not capture.messages  # Idle cannot mask other mux missions.
    roi.mission_cone = True
    roi.center_path = [(.1, .1, 0.), (.8, .1, 0.)]
    roi.control_scan_stamp = roi.get_clock().now().nanoseconds
    roi.control_received = time.monotonic()
    roi.update_control_activation()
    roi.last_control_tick = time.monotonic()-.05
    roi.control_tick()
    msg = capture.messages[-1]
    assert isinstance(msg, AckermannDriveStamped)
    assert msg.header.frame_id == 'base_link' and msg.header.stamp.sec > 0
    assert 0 < msg.drive.speed <= .15 and 0 < msg.drive.steering_angle <= .051
    roi.center_path = [(.1, .49, 0.), (.8, .49, 0.)]
    roi.last_steering = math.radians(80.)
    roi.control_tick()
    assert abs(capture.messages[-1].drive.steering_angle) <= math.radians(80.)
    roi.control_received = time.monotonic()-1.
    roi.control_tick()
    assert capture.messages[-1].drive.speed == capture.messages[-1].drive.steering_angle == 0.
    roi.control_received = time.monotonic()
    roi.control_scan_stamp = roi.get_clock().now().nanoseconds+1_000_000_000
    roi.control_tick()
    assert capture.messages[-1].drive.speed == 0.
    roi.control_fault = True
    roi.mission_cone = False
    roi.update_control_activation()
    assert roi.control_engaged and capture.messages[-1].drive.speed == 0.
    roi.control_fault = False
    roi.update_control_activation()
    assert not roi.control_engaged
    count = len(capture.messages)
    roi.control_tick()
    assert len(capture.messages) == count


def test_control_empty_path_and_zero_stamp_stop(harness):
    roi, *_ = harness
    messages = []
    class Capture:
        def publish(self, msg):
            messages.append(msg)
    roi.command_pub = Capture()
    roi.mission_cone = True
    roi.update_control_activation()
    assert messages[-1].drive.speed == 0.
    roi.center_path = [(.1, 0., 0.), (.8, 0., 0.)]
    roi.control_scan_stamp = 0
    roi.control_received = time.monotonic()
    roi.control_tick()
    assert messages[-1].drive.speed == 0.


def test_ackermann_command_dds_delivery(harness):
    from ackermann_msgs.msg import AckermannDriveStamped
    roi, probe, _, _, _, until = harness
    received = []
    subscription = probe.create_subscription(AckermannDriveStamped,
        '/mission/cone/command_06', received.append, 1)
    until(lambda: roi.command_pub.get_subscription_count() == 1)
    roi.mission_cone = True
    roi.center_path = [(.1, -.1, 0.), (.8, -.1, 0.)]
    roi.control_scan_stamp = roi.get_clock().now().nanoseconds
    roi.control_received = time.monotonic()
    roi.update_control_activation()
    roi.last_control_tick = time.monotonic()-.05
    roi.control_tick()
    until(lambda: bool(received))
    assert 0 < received[0].drive.speed <= .15
    assert received[0].drive.steering_angle < 0
    assert received[0].header.frame_id == 'base_link'


@pytest.mark.parametrize('name,value', [('control_speed', -1.),
    ('control_max_steering', 1.6), ('control_timeout', 0.), ('control_lateral_gain', float('nan')),
    ('control_speed_reference_steering', 0.), ('control_speed_reference_steering', float('nan'))])
def test_invalid_control_parameters(harness, name, value):
    roi, *_ = harness
    assert not roi.set_parameters_atomically([Parameter(name, value=value)]).successful


@pytest.mark.parametrize('missing_side', ['left', 'right', 'both'])
def test_missing_boundaries_use_learned_width_only_when_one_remains(harness, missing_side):
    """A single observed boundary supports an offset; losing both clears the path."""
    roi, *_ = harness
    roi.mission_cone = True
    stamp = roi.get_clock().now().to_msg()
    roi.left_boundary = [(x, .3, 0.) for x in (.1, .3, .5, .7)]
    roi.right_boundary = [(x, -.3, 0.) for x in (.1, .3, .5, .7)]
    roi.update_center_path(stamp)
    assert roi.center_path
    if missing_side in ('left', 'both'):
        roi.left_boundary = []
    if missing_side in ('right', 'both'):
        roi.right_boundary = []
    roi.update_center_path(stamp)
    if missing_side == 'both':
        assert not roi.center_pairs and not roi.center_points and not roi.center_path
    else:
        assert roi.center_path
        assert roi.center_path_mode == 'Unified'














@pytest.mark.parametrize('speed', [.1, .4])
def test_path_loss_stops_immediately_and_recovers(harness, speed):
    roi, *_ = harness
    messages = []

    class Capture:
        def publish(self, msg):
            messages.append(msg)

    roi.command_pub = Capture()
    roi.set_parameters_atomically([Parameter('control_speed', value=speed)])
    roi.mission_cone = True
    roi.center_path = [(.1, .1, 0.), (.8, .1, 0.)]
    roi.control_scan_stamp = roi.get_clock().now().nanoseconds
    roi.control_received = time.monotonic()
    roi.update_control_activation()
    roi.control_tick()
    assert messages[-1].drive.speed > 0.
    assert messages[-1].drive.steering_angle > 0.
    roi.center_path = []
    roi.update_control_activation()
    for _ in range(3):
        roi.control_tick()
        assert roi.control_mode == 'Stopped'
        assert roi.control_stop_reason == 'empty_path'
        assert messages[-1].drive.speed == messages[-1].drive.steering_angle == 0.
    assert roi.mission_cone
    roi.center_path = [(.1, -.1, 0.), (.8, -.1, 0.)]
    roi.control_tick()
    assert roi.control_mode == 'Path Feedback'
    assert messages[-1].drive.speed > 0.
    assert roi.control_stop_reason is None


@pytest.mark.parametrize('target,expected_speed', [(0., .4), (.24, .35),
    (-.24, .35), (.48, .3), (-.48, .3), (1.3, .3), (-1.3, .3)])
def test_corner_slowdown_independent_of_maximum_steering(harness, monkeypatch, target, expected_speed):
    roi, *_ = harness
    messages = []

    class Capture:
        def publish(self, msg):
            messages.append(msg)

    roi.command_pub = Capture()
    roi.set_parameters_atomically([Parameter('control_speed', value=.4)])
    roi.mission_cone = roi.control_engaged = True
    roi.control_scan_stamp = roi.get_clock().now().nanoseconds
    roi.control_received = time.monotonic()
    monkeypatch.setattr(roi, 'path_errors', lambda *args: (0., target))
    for limit in [.48, math.radians(80.)]:
        roi.set_parameters_atomically([Parameter('control_max_steering', value=limit)])
        roi.control_tick()
        assert messages[-1].drive.speed == pytest.approx(expected_speed)
        assert abs(messages[-1].drive.steering_angle) <= limit


@pytest.mark.parametrize('recovery', ['path', 'candidates', 'stale', 'tf'])
def test_mission_exit_requires_continuous_healthy_detection_loss(harness, monkeypatch, recovery):
    roi, *_ = harness
    clock = [100.]
    monkeypatch.setattr('cone_lane_follower.node.time.monotonic', lambda: clock[0])
    from rclpy.time import Time

    class Clock:
        def now(self):
            return Time(nanoseconds=int(clock[0]*1e9))

    monkeypatch.setattr(roi, 'get_clock', lambda: Clock())
    messages = []

    class Capture:
        def publish(self, msg):
            messages.append(msg)

    roi.command_pub = Capture()
    roi.mission_cone = roi.control_engaged = True
    roi.candidate_clusters = [[], [], []]

    def tick(t, fresh=True):
        clock[0] = t
        if fresh:
            roi.control_received = t
            roi.control_scan_stamp = int(t*1e9)
        roi.control_tick()

    tick(100.)
    tick(101.49)
    assert roi.mission_cone and roi.mission_exit_since is not None
    if recovery == 'path':
        roi.center_path = [(.1, 0., 0.), (.8, 0., 0.)]
    elif recovery == 'candidates':
        roi.candidate_clusters.append([])
    elif recovery == 'tf':
        roi.control_fault = True
    elif recovery == 'stale':
        roi.control_received -= 1.
    tick(101.8, fresh=recovery != 'stale')
    assert roi.mission_cone and roi.mission_exit_since is None
    roi.center_path = []
    roi.candidate_clusters = [[], [], []]
    roi.control_fault = False
    tick(102.)
    tick(103.49)
    assert roi.mission_cone
    tick(103.5)
    assert not roi.mission_cone and not roi.control_engaged
    assert messages[-1].drive.speed == messages[-1].drive.steering_angle == 0.
    count = len(messages)
    tick(103.55)
    assert len(messages) == count  # Release the mux after the final stop command.


@pytest.mark.parametrize('failure', ['tf', 'stale', 'mission', 'clock_reverse', 'unusable_path'])
def test_control_stops_on_fault_or_unusable_path(harness, failure):
    roi, *_ = harness
    messages = []
    class Capture:
        def publish(self, msg):
            messages.append(msg)
    roi.command_pub = Capture()
    roi.mission_cone = roi.control_engaged = True
    roi.center_path = []
    now = roi.get_clock().now().nanoseconds
    roi.control_scan_stamp = now
    roi.control_received = time.monotonic()
    if failure == 'tf':
        roi.control_fault = True
    elif failure == 'stale':
        roi.control_received -= 1.
    elif failure == 'mission':
        roi.mission_cone = False
    elif failure == 'clock_reverse':
        roi.control_scan_stamp = now+1_000_000_000
    else:
        roi.center_path = [(-.8, .1, 0.), (-.1, .1, 0.)]
    roi.control_tick()
    assert messages[-1].drive.speed == messages[-1].drive.steering_angle == 0.


@pytest.mark.parametrize('scenario', ['equal', 'left_long', 'right_long', 'disjoint', 'left_only', 'right_only'])
def test_unified_node_geometry_matches_straight_lane(harness, scenario):
    roi, *_ = harness
    roi.mission_cone = True
    stamp = roi.get_clock().now().to_msg()
    roi.left_boundary = [(x, .3, 0.) for x in (.1, .4, .7, 1.)]
    roi.right_boundary = [(x, -.3, 0.) for x in (.1, .4, .7, 1.)]
    roi.update_center_path(stamp)
    assert roi.lane_width_mean == pytest.approx(.6)
    samples = roi.lane_width_samples
    if scenario == 'left_long':
        roi.left_boundary += [(1.3, .3, 0.), (1.6, .3, 0.)]
    elif scenario == 'right_long':
        roi.right_boundary += [(1.3, -.3, 0.), (1.6, -.3, 0.)]
    elif scenario == 'disjoint':
        roi.right_boundary = [(x+3, y, z) for x,y,z in roi.right_boundary]
    elif scenario == 'left_only':
        roi.right_boundary = []
    elif scenario == 'right_only':
        roi.left_boundary = []
    roi.update_center_path(stamp)
    assert roi.center_path_mode == 'Unified'
    assert all(abs(y) < 1e-8 for x,y,z in roi.center_path)
    expected_end = 1.6 if scenario.endswith('long') else 1.
    assert roi.center_path[-1][0] == pytest.approx(expected_end, abs=.051)
    assert roi.path_errors(roi.center_path, .25) is not None
    if scenario in ('disjoint', 'left_only', 'right_only'):
        assert not roi.center_pairs
        assert roi.lane_width_samples == samples


def test_control_points_match_error_projection_and_clear(harness):
    roi, *_ = harness
    messages = []
    class Capture:
        def publish(self, msg):
            messages.append(msg)
    roi.control_points_pub = Capture()
    path = [(.1, .1, 0.), (.8, .3, 0.)]
    geometry = roi.path_control_geometry(path, .25)
    assert geometry[:2] == roi.path_errors(path, .25)
    roi.publish_control_points(path)
    markers = {m.ns: m for m in messages[-1].markers if m.action == Marker.ADD}
    projection = markers['projection'].pose.position
    assert (projection.x, projection.y) == pytest.approx(geometry[2:4])
    assert markers['reference'].pose.position.x == .25
    segment = markers['selected_segment'].points
    assert [(p.x, p.y) for p in segment] == [(p[0], p[1]) for p in path]
    assert all(m.header.frame_id == 'base_link' for m in markers.values())
    for path in (None, [], [(-.5, 0., 0.), (-.1, 0., 0.)]):
        roi.publish_control_points(path)
        assert len(messages[-1].markers) == 1
        assert messages[-1].markers[0].action == Marker.DELETEALL


def test_scan_queue_keeps_latest_only(harness):
    roi, *_ = harness
    assert roi.scan_sub.qos_profile.depth == 1
    assert roi.scan_sub.qos_profile.history == qos_profile_sensor_data.history
    assert roi.scan_sub.qos_profile.reliability == qos_profile_sensor_data.reliability


def test_stale_scan_skips_geometry_without_refreshing_watchdog(harness, monkeypatch):
    roi, *_ = harness
    previous_received = roi.control_received
    previous_stamp = roi.control_scan_stamp
    msg = scan([1., 1., 1.])
    msg.header.stamp = (roi.get_clock().now()-rclpy.duration.Duration(seconds=.6)).to_msg()
    monkeypatch.setattr(roi, 'scan_to_points',
                        lambda *a, **k: pytest.fail('Stale scan started geometry processing'))
    roi.scan_callback(msg)
    assert roi.control_received == previous_received
    assert roi.control_scan_stamp == previous_stamp
