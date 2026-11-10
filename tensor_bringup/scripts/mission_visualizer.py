#!/usr/bin/env python3
"""Read-only mission visualization. Never publishes vehicle commands or text."""
from copy import deepcopy
import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Path
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker, MarkerArray


PARKING_TOPICS = (
    (MarkerArray, '/parking/candidate_roi'),
    (MarkerArray, '/parking/obstacles'),
    (MarkerArray, '/parking/target'),
    (MarkerArray, '/parking/slot_map'),
    (MarkerArray, '/parking/lookahead'),
    (Path, '/parking/path'),
    (PointCloud2, '/parking/obstacle_memory'),
)


def without_text(message):
    output = deepcopy(message)
    # Retain DELETE/DELETEALL even when an old marker's type was text.
    output.markers = [m for m in output.markers
                      if m.type != Marker.TEXT_VIEW_FACING or m.action != Marker.ADD]
    return output


class MissionVisualizer(Node):
    def __init__(self):
        super().__init__('mission_visualizer')
        self.parking_active = False
        self.parking_received = None
        self.command = None
        self.subscriptions_ = []
        self.parking_publishers = {}
        self.input_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        for kind, topic in PARKING_TOPICS:
            self.parking_publishers[topic] = (kind, self.create_publisher(kind, '/missions_viz'+topic, 1))
            self.subscriptions_.append(self.create_subscription(
                kind, topic, lambda msg, topic=topic: self.forward_parking(topic, msg), self.input_qos))
        self.subscriptions_.extend([
            self.create_subscription(Bool, '/parking/active', lambda msg: self.set_parking(msg.data), self.input_qos),
            self.create_subscription(String, '/parking/status', self.parking_status, self.input_qos),
            self.create_subscription(Twist, '/cmd_vel', self.velocity, self.input_qos),
        ])
        self.path_pub = self.create_publisher(MarkerArray, '/missions_viz/command_path', 1)
        self.timer = self.create_timer(.1, self.tick)
        self.clear_parking()

    def parking_status(self, message):
        try:
            status = json.loads(message.data)
            active = status['active']
            if isinstance(active, bool):
                self.set_parking(active and status.get('state') != 'EXITED')
        except (ValueError, KeyError, TypeError):
            pass

    def set_parking(self, active):
        self.parking_received = time.monotonic()
        if active != self.parking_active:
            self.clear_parking()
        self.parking_active = active

    def forward_parking(self, topic, message):
        if (not self.parking_active or self.parking_received is None
                or time.monotonic()-self.parking_received > 2.):
            return
        if isinstance(message, MarkerArray):
            message = without_text(message)
        self.parking_publishers[topic][1].publish(message)

    def clear_parking(self):
        for kind, pub in self.parking_publishers.values():
            message = kind()
            if kind is MarkerArray:
                marker = Marker(); marker.action = Marker.DELETEALL
                message.markers = [marker]
            else:
                message.header.frame_id = 'base_link'
                message.header.stamp = self.get_clock().now().to_msg()
                if kind is PointCloud2:
                    message.height = 1
                    message.point_step = 12
                    message.is_dense = True
                    message.fields = [PointField(name=name, offset=i*4, datatype=PointField.FLOAT32, count=1)
                                      for i, name in enumerate(('x', 'y', 'z'))]
            pub.publish(message)

    def velocity(self, message):
        if math.isfinite(message.linear.x) and math.isfinite(message.angular.z):
            self.command = (time.monotonic(), message.linear.x, message.angular.z)

    def tick(self):
        if self.parking_active and self.parking_received is not None and time.monotonic()-self.parking_received > 2.:
            self.parking_active = False
            self.clear_parking()
        marker = Marker()
        marker.header.frame_id = 'base_link'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns, marker.id = 'actual_command', 0
        marker.pose.orientation.w = 1.
        marker.type = Marker.LINE_STRIP
        marker.lifetime.nanosec = 400_000_000
        marker.scale.x = .018
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = .1, .9, 1., .9
        marker.action = Marker.DELETE
        if self.command is not None and time.monotonic()-self.command[0] < .3:
            _, v, w = self.command
            if abs(v) > .001:
                marker.action = Marker.ADD
                # Requested path over the next 1.5 s; not an obstacle-free plan.
                for i in range(31):
                    t = i*.05
                    x = v*t if abs(w)<1e-6 else v*math.sin(w*t)/w
                    y = 0. if abs(w)<1e-6 else v*(1-math.cos(w*t))/w
                    marker.points.append(Point(x=x, y=y, z=.025))
        self.path_pub.publish(MarkerArray(markers=[marker]))


def main():
    rclpy.init()
    node = MissionVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
