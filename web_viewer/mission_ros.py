"""Latest mission status and the vehicle-local obstacle map."""
from collections import deque
from functools import partial
import math
import threading
import time


# key, topic, transient-local
ACTIVATION = (
    ('slow', '/slow/mission_active', True),
    ('objavoid', '/objavoid_v3/active', True),
    ('parking', '/parking/active', False),
)
YOLO_TOPICS = {
    '/yolo/slow_sign': None,
    '/yolo/crosswalk_sign': None,
    '/yolo/parking_sign': None,
    '/yolo/signal': ('빨강', '초록'),
    '/yolo/LR_sign': ('왼쪽', '오른쪽'),
}
YOLO_TTL = 0.8
GEOMETRY_TTL = 2.0
MAP_INTERVAL = 0.25
MAP_IDLE_TIMEOUT = 3.0


class MonitorState:
    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.lock = threading.Lock()
        self.values = {}
        self.detections = {}
        self.detection_history = deque(maxlen=5)
        self.markers = (float('-inf'), [])
        self.scan = (float('-inf'), [])
        self.last_map_request = float('-inf')

    def set_status(self, key, value, latched):
        with self.lock:
            self.values[key] = (value, self.clock(), latched)

    def status_snapshot(self):
        with self.lock:
            now = self.clock()
            values = {
                key: value for key, (value, received, latched) in self.values.items()
                if latched or now - received < GEOMETRY_TTL
            }
            detections = [
                {'topic': topic, 'detail': detail}
                for topic, (detail, received) in self.detections.items()
                if now - received < YOLO_TTL
            ]
            return {'status': values, 'yolo': detections, 'yolo_history': list(self.detection_history)}

    def set_detection(self, topic, detected):
        labels = YOLO_TOPICS[topic]
        with self.lock:
            if labels is None and not detected:
                self.detections.pop(topic, None)
            else:
                detail = labels[detected] if labels is not None else ''
                now = self.clock()
                previous = self.detections.get(topic)
                if previous is None or previous[0] != detail or now - previous[1] >= YOLO_TTL:
                    self.detection_history.appendleft({
                        'topic': topic, 'detail': detail, 'timestamp': time.time(),
                    })
                self.detections[topic] = (detail, now)

    def map_snapshot(self):
        with self.lock:
            now = self.clock()
            self.last_map_request = now
            return {
                'markers': self.markers[1] if now - self.markers[0] < GEOMETRY_TTL else [],
                'scan': self.scan[1] if now - self.scan[0] < GEOMETRY_TTL else [],
            }


def start_ros(state):
    import rclpy
    from rclpy.node import Node
    from rclpy.signals import SignalHandlerOptions
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import Bool, String
    from visualization_msgs.msg import Marker, MarkerArray

    sensor = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
    latch = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    latched_topics = {'owner': '/mux/active_source'}
    latched_topics.update({key: topic for key, topic, latched in ACTIVATION if latched})

    class Bridge(Node):
        def __init__(self):
            super().__init__('mission_web_monitor', enable_rosout=False, start_parameter_services=False)
            self.create_subscription(String, latched_topics['owner'], partial(self.status, 'owner', True), latch)
            for key, topic, latched in ACTIVATION:
                self.create_subscription(Bool, topic, partial(self.status, key, latched), latch if latched else sensor)
            for topic in YOLO_TOPICS:
                self.create_subscription(Bool, topic, partial(self.detection, topic), sensor)
            self.map_subscriptions = []
            self.create_timer(0.5, self.update_map_subscriptions)
            self.create_timer(1.0, self.expire_disconnected_status)

        def update_map_subscriptions(self):
            with state.lock:
                wanted = state.clock() - state.last_map_request < MAP_IDLE_TIMEOUT
            if wanted and not self.map_subscriptions:
                self.map_subscriptions = [
                    self.create_subscription(MarkerArray, '/objavoid_v3/markers', self.obstacle_markers, sensor),
                    self.create_subscription(LaserScan, '/scan', self.lidar, sensor),
                ]
            elif not wanted and self.map_subscriptions:
                for subscription in self.map_subscriptions:
                    self.destroy_subscription(subscription)
                self.map_subscriptions.clear()
                with state.lock:
                    state.markers = (float('-inf'), [])
                    state.scan = (float('-inf'), [])

        def status(self, key, latched, msg):
            state.set_status(key, msg.data, latched)

        def detection(self, topic, msg):
            state.set_detection(topic, msg.data)

        def expire_disconnected_status(self):
            for key, topic in latched_topics.items():
                if not self.count_publishers(topic):
                    with state.lock:
                        state.values.pop(key, None)

        def obstacle_markers(self, msg):
            now = state.clock()
            # Each planner message is a complete snapshot. Clear immediately on reset.
            clear = len(msg.markers) == 1 and msg.markers[0].action == Marker.DELETEALL
            if not clear and now - state.markers[0] < MAP_INTERVAL:
                return
            result = []
            for marker in msg.markers:
                if marker.action == Marker.DELETEALL:
                    result.clear()
                if marker.action != Marker.ADD or marker.ns in ('ego', 'state'):
                    continue
                if marker.header.frame_id.strip('/') != 'base_link':
                    continue
                if marker.type not in (Marker.POINTS, Marker.LINE_STRIP, Marker.LINE_LIST,
                                       Marker.SPHERE, Marker.TEXT_VIEW_FACING):
                    continue
                position = marker.pose.position
                if not all(math.isfinite(value) for value in (position.x, position.y, marker.scale.x)):
                    continue
                result.append({
                    'ns': marker.ns,
                    'type': marker.type,
                    'points': [[round(point.x, 3), round(point.y, 3)] for point in marker.points
                               if math.isfinite(point.x) and math.isfinite(point.y)],
                    'position': [position.x, position.y],
                    'scale': marker.scale.x,
                    'color': [marker.color.r, marker.color.g, marker.color.b, marker.color.a],
                    'text': marker.text,
                })
            with state.lock:
                state.markers = (now, result)

        def lidar(self, msg):
            now = state.clock()
            if now - state.scan[0] < MAP_INTERVAL:
                return
            # Use the same vehicle-local LiDAR offset as the objavoid viewer.
            step = max(1, math.ceil(len(msg.ranges) / 1500))
            points = []
            for index in range(0, len(msg.ranges), step):
                distance = msg.ranges[index]
                if math.isfinite(distance) and msg.range_min <= distance <= msg.range_max:
                    angle = msg.angle_min + index * msg.angle_increment
                    points.append([round(0.103 + distance * math.cos(angle), 3),
                                   round(distance * math.sin(angle), 3)])
            with state.lock:
                state.scan = (now, points)

    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    try:
        return rclpy, Bridge()
    except Exception:
        rclpy.try_shutdown()
        raise
        
