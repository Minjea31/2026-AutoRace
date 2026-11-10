"""Read-only, bounded ROS geometry snapshots for the parking web view."""
import json
import math
import struct
import threading
import time

import rclpy
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import Buffer, TransformListener, TransformException


def finite_json(value):
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {k: finite_json(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [finite_json(v) for v in value]
    return value


def transform_point(point, transform):
    x, y, z = point
    if transform is None:
        return [x, y]
    t, q = transform.translation, transform.rotation
    # Full mounting rotation, including an upside-down LiDAR.
    cx, cy, cz = q.y*z-q.z*y, q.z*x-q.x*z, q.x*y-q.y*x
    return [x+2*(q.w*cx+q.y*cz-q.z*cy)+t.x,
            y+2*(q.w*cy+q.z*cx-q.x*cz)+t.y]


def cloud_points(message, limit=1000):
    fields = {f.name: f for f in message.fields}
    if not all(k in fields and fields[k].datatype in (7, 8) for k in ('x', 'y')):
        return []
    endian = '>' if message.is_bigendian else '<'
    size = message.width*message.height
    stride = max(1, math.ceil(size/limit))
    points = []
    for i in range(0, size, stride):
        row, col = divmod(i, message.width)
        offset = row*message.row_step+col*message.point_step
        values = []
        for name in ('x', 'y'):
            f = fields[name]
            fmt = endian+('f' if f.datatype == 7 else 'd')
            if offset+f.offset+struct.calcsize(fmt) > len(message.data):
                return points
            values.append(struct.unpack_from(fmt, message.data, offset+f.offset)[0])
        if all(math.isfinite(v) for v in values):
            points.append(values)
    return points


class ParkingState:
    def __init__(self):
        self.lock = threading.RLock()
        self.status = None
        self.status_time = 0.
        self.layers = {}
        self.errors = {}
        self.received = {}
        self.generation = None
        self.vehicle = {'length': .322, 'width': .22, 'rear_axle_x': -.10}

    def attach(self, node):
        self.node = node
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, node)
        topics = [('/parking/status', String, self.on_status),
                  ('/scan', LaserScan, self.on_scan),
                  ('/parking/path', Path, self.on_path),
                  ('/parking/obstacle_memory', PointCloud2, self.on_cloud),
                  ('/parking/localization/odometry', Odometry, self.on_pose)]
        self.subscriptions = [node.create_subscription(kind, topic, cb, qos_profile_sensor_data)
                              for topic, kind, cb in topics]
        for topic, key in [('/parking/obstacles', 'obstacles'), ('/parking/target', 'target'),
                           ('/parking/lookahead', 'lookahead'), ('/parking/candidate_roi', 'roi')]:
            self.subscriptions.append(node.create_subscription(
                MarkerArray, topic, lambda m, k=key: self.on_markers(k, m), qos_profile_sensor_data))

    def due(self, key, interval=.18):
        now = time.monotonic()
        if now-self.received.get(key, -math.inf) < interval:
            return False
        self.received[key] = now
        return True

    def transform(self, frame):
        if frame == 'base_link':
            return None
        if not frame:
            raise ValueError('좌표계가 없습니다.')
        try:
            return self.tf.lookup_transform('base_link', frame, rclpy.time.Time()).transform
        except TransformException as exc:
            raise ValueError(f'{frame} → base_link 변환 대기') from exc

    def put(self, key, value, error=''):
        with self.lock:
            self.layers[key] = (time.monotonic(), value)
            self.errors[key] = error

    def fail(self, key, error):
        with self.lock:
            self.layers.pop(key, None)
            self.errors[key] = str(error)

    def on_status(self, message):
        try:
            value = json.loads(message.data)
            if not isinstance(value, dict):
                return
            generation = (value.get('mission_attempt'),
                          value.get('synchronization', {}).get('localization_restart_count'))
            with self.lock:
                if self.generation != generation:
                    self.layers.clear()
                    self.errors.clear()
                    self.generation = generation
                self.status, self.status_time = value, time.monotonic()
        except (ValueError, TypeError):
            self.fail('status', '주차 상태 메시지를 읽을 수 없습니다.')

    def on_scan(self, message):
        if not self.due('scan'):
            return
        try:
            tf = self.transform(message.header.frame_id)
            points = []
            for i in range(0, len(message.ranges), max(1, len(message.ranges)//720)):
                distance = message.ranges[i]
                if not math.isfinite(distance) or not message.range_min <= distance <= min(6., message.range_max):
                    continue
                angle = message.angle_min+i*message.angle_increment
                points.append(transform_point((distance*math.cos(angle), distance*math.sin(angle), 0.), tf))
            self.put('scan', points)
        except ValueError as exc:
            self.fail('scan', exc)

    def on_path(self, message):
        if not self.due('path'):
            return
        try:
            tf = self.transform(message.header.frame_id)
            points = [transform_point((p.pose.position.x, p.pose.position.y, p.pose.position.z), tf)
                      for p in message.poses[::max(1, len(message.poses)//400)]]
            self.put('path', points)
        except ValueError as exc:
            self.fail('path', exc)

    def on_cloud(self, message):
        if not self.due('memory', .3):
            return
        try:
            tf = self.transform(message.header.frame_id)
            self.put('memory', [transform_point((*p, 0.), tf) for p in cloud_points(message)])
        except (ValueError, struct.error) as exc:
            self.fail('memory', exc)

    def on_pose(self, message):
        if not self.due('pose'):
            return
        p, q = message.pose.pose.position, message.pose.pose.orientation
        self.put('pose', [p.x, p.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))])

    def on_markers(self, key, message):
        # Process deletion immediately, including between throttled geometry updates.
        if any(m.action == 3 for m in message.markers) and not any(m.action == 0 for m in message.markers):
            self.put(key, [])
            return
        if not self.due(key):
            return
        result = []
        try:
            for marker in message.markers[:160]:
                if marker.action != 0:
                    continue
                tf = self.transform(marker.header.frame_id)
                p, q = marker.pose.position, marker.pose.orientation
                pose_tf = type('PoseTransform', (), {'translation': p, 'rotation': q})()
                points = []
                for point in marker.points[:500]:
                    local = transform_point((point.x, point.y, point.z), pose_tf)
                    points.append(transform_point((*local, p.z+point.z), tf))
                center = transform_point((p.x, p.y, p.z), tf)
                result.append(dict(type=marker.type, ns=marker.ns, id=marker.id,
                                   points=points, center=center, text=marker.text,
                                   scale=[marker.scale.x, marker.scale.y],
                                   color=[marker.color.r, marker.color.g, marker.color.b, marker.color.a]))
            self.put(key, result)
        except ValueError as exc:
            self.fail(key, exc)

    def snapshot(self):
        now = time.monotonic()
        with self.lock:
            age = now-self.status_time if self.status is not None else None
            fresh = age is not None and age < 2.
            active = fresh and self.status.get('active', False)
            # Clear mission overlays immediately on input failure; raw LiDAR remains useful.
            valid = fresh and self.status.get('localization', {}).get('valid', False)
            layers = {key: value for key, (updated, value) in self.layers.items()
                      if now-updated < 1.5 and (key == 'scan' or (active and valid))}
            ages = {key: round((now-updated)*1000) for key, (updated, _) in self.layers.items()}
            status = self.status if fresh else None
            return finite_json(dict(connected=fresh, active=bool(active),
                                    age_ms=None if age is None else round(age*1000),
                                    status=status, layers=layers, layer_age_ms=ages,
                                    errors=dict(self.errors), vehicle=self.vehicle,
                                    frame='base_link', generated_at=time.time()))
