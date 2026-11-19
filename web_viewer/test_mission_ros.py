"""ROS integration test for the lightweight web viewer on private domain 197."""
import os
import time
import unittest

from mission_ros import MAP_IDLE_TIMEOUT, YOLO_TTL, MonitorState, start_ros

try:
    import rclpy
except ImportError:
    rclpy = None


class DetectionHistoryTest(unittest.TestCase):
    def test_history_records_transitions_and_keeps_only_five(self):
        now = 0.0
        state = MonitorState(clock=lambda: now)
        signal = '/yolo/signal'
        sign = '/yolo/slow_sign'
        state.set_detection(sign, False)
        state.set_detection(sign, True)
        state.set_detection(sign, True)
        self.assertEqual(len(state.status_snapshot()['yolo_history']), 1)
        state.set_detection(sign, False)
        state.set_detection(sign, True)
        now = YOLO_TTL
        self.assertEqual(state.status_snapshot()['yolo'], [])
        state.set_detection(sign, True)
        for detected in (False, True, False, True):
            state.set_detection(signal, detected)
        history = state.status_snapshot()['yolo_history']
        self.assertEqual([item['detail'] for item in history], ['초록', '빨강', '초록', '빨강', ''])
        self.assertTrue(all(item['timestamp'] > 0 for item in history))


@unittest.skipUnless(
    rclpy and os.environ.get('ROS_LOCALHOST_ONLY') == '1' and os.environ.get('ROS_DOMAIN_ID') == '197',
    'isolated ROS test environment required',
)
class RosIntegration(unittest.TestCase):
    def test_activation_and_obstacle_map(self):
        from geometry_msgs.msg import Point
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, QoSProfile
        from sensor_msgs.msg import LaserScan
        from std_msgs.msg import Bool, String
        from visualization_msgs.msg import Marker, MarkerArray

        state = MonitorState()
        ros, bridge = start_ros(state)
        probe = Node('mission_web_test_fixture')
        latch = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        owner = probe.create_publisher(String, '/mux/active_source', latch)
        active = probe.create_publisher(Bool, '/objavoid_v3/active', latch)
        markers = probe.create_publisher(MarkerArray, '/objavoid_v3/markers', 1)
        lidar = probe.create_publisher(LaserScan, '/scan', 1)
        clear = Marker(action=Marker.DELETEALL)
        path = Marker(ns='path', type=Marker.LINE_STRIP)
        path.header.frame_id = 'base_link'
        path.points = [Point(x=0.5), Point(x=1.0)]
        path.scale.x = 0.02
        path.color.r = path.color.a = 1.0

        def spin_until(condition, publish=lambda: None, timeout=4.0):
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                if condition():
                    return
                publish()
                ros.spin_once(bridge, timeout_sec=0.02)
            self.fail('Timed out waiting for the viewer subscription or data')

        def publish():
            owner.publish(String(data='objavoid'))
            active.publish(Bool(data=True))
            markers.publish(MarkerArray(markers=[clear, path]))
            lidar.publish(LaserScan(range_min=0.1, range_max=3.0, ranges=[1.0]))

        def received():
            status = state.status_snapshot()
            geometry = state.map_snapshot()
            return status['status'].get('objavoid') and geometry['markers'] and geometry['scan']

        try:
            self.assertFalse(bridge.map_subscriptions)
            spin_until(received, publish)
            status = state.status_snapshot()
            self.assertEqual(status['status']['owner'], 'objavoid')
            geometry = state.map_snapshot()
            self.assertEqual(geometry['markers'][0]['points'], [[0.5, 0.0], [1.0, 0.0]])
            self.assertEqual(geometry['scan'], [[1.103, 0.0]])
            spin_until(lambda: not state.map_snapshot()['markers'],
                       lambda: markers.publish(MarkerArray(markers=[clear])))
            spin_until(lambda: not bridge.map_subscriptions, timeout=MAP_IDLE_TIMEOUT + 1.5)
            self.assertEqual(state.map_snapshot(), {'markers': [], 'scan': []})
        finally:
            probe.destroy_node()
            bridge.destroy_node()
            ros.try_shutdown()


if __name__ == '__main__':
    unittest.main()
