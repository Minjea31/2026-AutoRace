"""RViz publishers only; control decisions and safety checks stay in node.py."""
import colorsys
import math

from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from rclpy.qos import DurabilityPolicy, QoSProfile
from visualization_msgs.msg import Marker, MarkerArray


class Visualizer:
    def __init__(self, node, preview_points, cluster_bounds):
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.markers = node.create_publisher(MarkerArray, '/objavoid/markers', qos)
        self.paths = node.create_publisher(Path, '/objavoid/path', qos)
        self.clusters = node.create_publisher(MarkerArray, '/objavoid/clusters', qos)
        self.preview_points, self.cluster_bounds = preview_points, cluster_bounds
        self.last_time, self.last_state = -math.inf, None
        self.previous_count, self.reset = 0, True

    def due(self, state, now):
        return state != self.last_state or now - self.last_time >= .1

    @staticmethod
    def marker(namespace, index, kind, color, frame='base_link', width=.025):
        marker = Marker()
        marker.header.frame_id = frame
        marker.ns, marker.id, marker.type = namespace, index, kind
        marker.frame_locked = kind != Marker.TEXT_VIEW_FACING
        marker.pose.orientation.w = 1.
        marker.scale.x = width
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        return marker

    @staticmethod
    def point(x, y, z=.04):
        return Point(x=float(x), y=float(y), z=float(z))

    def publish(self, state, speed, steering, now, c, lane_points, road, groups, frame):
        markers, clusters = MarkerArray(), MarkerArray()
        path = Path()
        path.header.frame_id = 'base_link'
        from .node import path_is_valid
        points = self.preview_points(steering, c) if speed > 0 and path_is_valid(state) else ()
        color = (1.,.65,.1,1.) if state.startswith('SCAN_UNAVAILABLE') else (0.,.85,1.,1.)
        curve = self.marker('command_path', 1, Marker.LINE_STRIP, color)
        curve.points = [self.point(x, y) for x, y in points]
        if not points:
            curve.action = Marker.DELETE
        markers.markers.append(curve)
        from .node import preview_poses
        poses = preview_poses(steering,c)
        for index, (x, y) in enumerate(points):
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position = self.point(x, y)
            yaw = poses[index][2]
            pose.pose.orientation.z, pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            path.poses.append(pose)
        lane = self.marker('lane_center', 11, Marker.LINE_STRIP, (1., .85, .15, 1.))
        lane.points = [self.point(x, y, .05) for x, y in lane_points]
        if not lane_points:
            lane.action = Marker.DELETE
        markers.markers.append(lane)
        limits = self.marker('road_limits', 12, Marker.LINE_LIST, (.9, .9, 1., .75), width=.015)
        lane_id = self.marker('lane_id', 13, Marker.TEXT_VIEW_FACING, (.9, .9, 1., 1.))
        lane_id.pose.position = self.point(-.25, 0., .12)
        lane_id.scale.z = .065
        if road is None:
            limits.action = lane_id.action = Marker.DELETE
        else:
            if hasattr(road,'edges'):
                for nx,ny,offset in road.edges:
                    for x in (i*.25 for i in range(math.ceil(c.visual_road_distance/.25))):
                        for xx in (x,min(x+.15,c.visual_road_distance)):
                            limits.points.append(self.point(xx,-(nx*xx+offset)/ny,z=.03))
            else:
                for lateral in (-road.half_width,road.half_width):
                    limits.points.extend((self.point(*road.point(0.,lateral)),
                                          self.point(*road.point(c.visual_road_distance,lateral))))
            lane_id.text = 'YELLOW BOUNDARIES' if hasattr(road,'edges') else 'CAMERA ROAD'
        markers.markers.extend((limits, lane_id))
        for index in range(max(len(groups), self.previous_count)):
            color = (*colorsys.hsv_to_rgb((.53 + index*.61803398875) % 1., .65, 1.), 1.)
            box = self.marker('cluster_boxes', index, Marker.LINE_STRIP, color, frame, .012)
            box.frame_locked = False
            if index < len(groups):
                xmin, xmax, ymin, ymax = self.cluster_bounds(groups[index])
                box.points = [self.point(x, y, .07) for x, y in
                              ((xmin, ymin), (xmax, ymin), (xmax, ymax), (xmin, ymax), (xmin, ymin))]
            else:
                box.action = Marker.DELETE
            clusters.markers.append(box)
        if self.reset:
            for result, reference in ((markers, 'base_link'), (clusters, frame)):
                clear = Marker()
                clear.header.frame_id = reference
                clear.pose.orientation.w = 1.
                clear.action = Marker.DELETEALL
                result.markers.insert(0, clear)
            self.reset = False
        self.markers.publish(markers)
        self.paths.publish(path)
        self.clusters.publish(clusters)
        self.previous_count = len(groups)
        self.last_time, self.last_state = now, state
