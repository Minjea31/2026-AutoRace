import json
import runpy
import time
from pathlib import Path
from types import SimpleNamespace

import pytest
import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import PointCloud2


@pytest.fixture
def viewer():
    module = runpy.run_path(str(Path(__file__).resolve().parents[1] / 'scripts/mission_visualizer.py'))
    rclpy.init(domain_id=191)
    node = module['MissionVisualizer']()
    outputs = {}
    original = node.parking_publishers
    for topic, (kind, _) in original.items():
        outputs[topic] = []
        node.parking_publishers[topic] = (kind, SimpleNamespace(publish=outputs[topic].append))
    paths = []
    node.path_pub = SimpleNamespace(publish=paths.append)
    try:
        yield node, outputs, paths
    finally:
        node.destroy_node(); rclpy.shutdown()


def test_parking_activation_filters_text_and_does_not_mutate_original(viewer):
    node, outputs, _ = viewer
    sphere = Marker(); sphere.type = Marker.SPHERE
    text = Marker(); text.type = Marker.TEXT_VIEW_FACING; text.text = 'Do not show'
    source = MarkerArray(markers=[sphere, text])
    node.forward_parking('/parking/target', source)
    assert not outputs['/parking/target']
    node.parking_status(String(data=json.dumps({'active': True, 'state': 'PARKING'})))
    node.forward_parking('/parking/target', source)
    assert len(outputs['/parking/target'][-1].markers) == 1
    assert outputs['/parking/target'][-1].markers[0].type == Marker.SPHERE
    assert len(source.markers) == 2
    node.parking_status(String(data=json.dumps({'active': True, 'state': 'EXITED'})))
    assert not node.parking_active
    assert outputs['/parking/target'][-1].markers[0].action == Marker.DELETEALL


def test_status_timeout_clears_every_visual_and_empty_cloud_has_xyz(viewer):
    node, outputs, _ = viewer
    node.set_parking(True)
    node.parking_received = time.monotonic()-3.
    node.tick()
    assert not node.parking_active
    for topic, messages in outputs.items():
        assert messages
    cloud = outputs['/parking/obstacle_memory'][-1]
    assert isinstance(cloud, PointCloud2)
    assert [f.name for f in cloud.fields] == ['x', 'y', 'z']
    assert cloud.width == 0 and cloud.point_step == 12


def test_motion_curve_is_read_only_has_no_text_and_expires(viewer):
    node, _, paths = viewer
    message = Twist(); message.linear.x = .8; message.angular.z = .2
    node.velocity(message); node.tick()
    marker = paths[-1].markers[0]
    assert marker.type == Marker.LINE_STRIP and marker.action == Marker.ADD
    assert len(marker.points) == 31 and marker.points[-1].y > 0
    assert marker.text == '' and marker.lifetime.nanosec > 0
    node.command = (time.monotonic()-1., .8, .2); node.tick()
    assert paths[-1].markers[0].action == Marker.DELETE
    for topic, _ in node.get_publisher_names_and_types_by_node(node.get_name(), node.get_namespace()):
        assert topic.startswith('/missions_viz/') or topic in ('/rosout', '/parameter_events')
