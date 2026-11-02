"""Parking ROS adapter. Motion is emitted only through the documented mux input."""
from concurrent.futures import ThreadPoolExecutor
from dataclasses import fields, replace
import colorsys
import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from rclpy.executors import ExternalShutdownException
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from .core import Activation, Scan, Settings, cluster_points
from .control import Follower, build_plan, local_goal
from .tracking import ScanTracker


class ParkingNode(Node):
    def __init__(self, **kwargs):
        super().__init__('parking_node', **kwargs)
        def parameter(name, default):
            return self.declare_parameter(name, default).value
        self.cfg = Settings(**{f.name: parameter(f.name, getattr(Settings(), f.name)) for f in fields(Settings)})
        for f in fields(Settings):
            value = getattr(self.cfg, f.name)
            if isinstance(value, (int, float)) and not isinstance(value, bool) and not math.isfinite(value):
                raise ValueError(f'{f.name} must be finite')
        for name in ('cluster_distance', 'max_range', 'vehicle_length', 'vehicle_width', 'wheelbase',
                     'max_steering', 'resolution', 'yaw_resolution', 'step', 'collision_step', 'planning_timeout'):
            if getattr(self.cfg, name) <= 0:
                raise ValueError(f'{name} must be positive')
        if self.cfg.allow_reverse:
            raise ValueError('Inbound planning must remain forward-only; departure reverses the recorded path')
        if not 0 < self.cfg.max_steering < math.pi / 2 or self.cfg.clearance < 0:
            raise ValueError('Invalid steering or clearance')
        self.base_frame = parameter('base_frame', 'base_link')
        self.lidar_frame = parameter('lidar_frame', 'laser_link')
        self.sensor_pose = (parameter('lidar_x', 0.103), parameter('lidar_y', 0.0), parameter('lidar_yaw', 0.0))
        required = parameter('recognition_frames', 3)
        self.sign_timeout = parameter('sign_timeout', 0.5)
        self.scan_timeout = parameter('scan_timeout', 0.5)
        self.plan_period = parameter('plan_period', 1.5)
        self.speed = parameter('parking_speed', 1.0)
        self.hold_seconds = parameter('parking_hold_seconds', 3.0)
        self.deceleration = parameter('max_deceleration', 1.0)
        self.max_yaw_rate = parameter('max_yaw_rate', 1.0)
        self.lookahead = parameter('lookahead', 0.18)
        self.control_hz = parameter('control_hz', 20.0)
        if (required < 1 or not all(math.isfinite(v) for v in
                (self.sign_timeout, self.scan_timeout, self.plan_period, self.speed,
                 self.lookahead, self.control_hz, self.hold_seconds, self.deceleration, self.max_yaw_rate)) or
                min(self.sign_timeout, self.scan_timeout, self.plan_period, self.speed,
                    self.lookahead, self.control_hz, self.deceleration, self.max_yaw_rate) <= 0 or self.hold_seconds < 0):
            raise ValueError('Frame count, timeouts, speed and control rate must be positive')
        self.activation = Activation(required, self.sign_timeout)
        self.follower = self.new_follower()
        self.tracker = self.new_tracker()
        self.scan, self.scan_header, self.scan_time = None, None, None
        self.last_scan_stamp = None
        self.scan_sequence = 0
        self.controlled_sequence = -1
        self.last_now = None
        self.state = 'WAITING_SIGN'
        self.attempt = 0
        self.reason = ''
        self.epoch = 0
        self.last_plan_wall = -math.inf
        self.last_status_wall = -math.inf
        self.worker = ThreadPoolExecutor(max_workers=1, thread_name_prefix='parking_planner')
        self.pending = None
        self.obstacles_pub = self.create_publisher(MarkerArray, '/parking/obstacles', 1)
        self.target_pub = self.create_publisher(MarkerArray, '/parking/target', 1)
        self.path_pub = self.create_publisher(Path, '/parking/path', 1)
        self.active_pub = self.create_publisher(Bool, '/parking/active', 1)
        self.status_pub = self.create_publisher(String, '/parking/status', 1)
        self.command_pub = self.create_publisher(AckermannDriveStamped,
                                               '/mission/parking/command_03', QoSProfile(depth=1))
        self.create_subscription(Bool, '/yolo/parking_sign', self.on_sign, QoSProfile(depth=1))
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_service(Trigger, '/parking/reset', self.on_reset)
        self.create_timer(1.0 / self.control_hz, self.tick)
        self.get_logger().info('Parking ready: 3 consecutive detections by default; /parking/path and '
                               '/parking/obstacles; control -> /mission/parking/command_03')

    def now_seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def new_follower(self):
        return Follower(self.cfg, self.speed, self.lookahead, self.hold_seconds, self.deceleration, self.max_yaw_rate)

    def new_tracker(self):
        return ScanTracker(self.speed, self.speed * math.tan(self.cfg.max_steering) / self.cfg.wheelbase,
                           self.scan_timeout)

    def clear_visuals(self):
        self.path_pub.publish(self.path_message([]))
        for publisher in (self.obstacles_pub, self.target_pub):
            clear = Marker()
            clear.action = Marker.DELETEALL
            publisher.publish(MarkerArray(markers=[clear]))

    def reset_motion(self):
        self.epoch += 1
        if self.pending is not None:
            self.pending[0].cancel()
            self.pending = None
        self.follower = self.new_follower()
        self.tracker = self.new_tracker()
        self.scan, self.scan_header, self.scan_time = None, None, None
        self.last_scan_stamp = None
        self.controlled_sequence = -1
        self.reason = ''
        self.last_plan_wall = -math.inf
        self.clear_visuals()

    def reset_state(self):
        self.reset_motion()
        self.activation.reset()
        self.state = 'WAITING_SIGN'
        self.active_pub.publish(Bool(data=False))

    def on_reset(self, request, response):
        # Explicit reset is also allowed to release the mux after a final stop.
        if self.activation.active:
            self.publish_command(0.0, 0.0)
        self.reset_state()
        response.success = True
        response.message = 'Parking reset; waiting for consecutive true detections'
        return response

    def on_sign(self, msg):
        was_active = self.activation.active
        generation = self.activation.generation
        self.activation.update(msg.data, self.now_seconds())
        if self.activation.generation != generation:
            if was_active:
                self.publish_command(0.0, 0.0)
            self.reset_motion()
            self.attempt += 1
            # Registration origin is the first scan acquired after activation.
            self.state, self.reason = 'WAITING_SCAN', ''
            self.active_pub.publish(Bool(data=True))
            self.get_logger().info(f'Parking mission activated (attempt {self.attempt})')

    def on_scan(self, msg):
        now = self.now_seconds()
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        valid_geometry = (msg.header.frame_id in (self.base_frame, self.lidar_frame)
                          and len(msg.ranges) >= 3 and math.isfinite(msg.angle_min)
                          and math.isfinite(msg.angle_increment) and msg.angle_increment != 0
                          and math.isfinite(msg.range_min) and math.isfinite(msg.range_max)
                          and 0 <= msg.range_min < msg.range_max)
        if not valid_geometry or not -0.1 <= now - stamp <= self.scan_timeout:
            self.scan = None
            self.reason = 'Invalid scan geometry/frame or stale timestamp'
            self.clear_visuals()
            return
        if self.last_scan_stamp is not None and stamp <= self.last_scan_stamp:
            return
        self.last_scan_stamp = stamp
        sensor_pose = (0.0, 0.0, 0.0) if msg.header.frame_id == self.base_frame else self.sensor_pose
        self.scan = Scan(msg.ranges, msg.angle_min, msg.angle_increment, msg.range_min,
                         msg.range_max, sensor_pose, self.cfg.max_range)
        self.scan_time, self.scan_header = stamp, msg.header
        self.scan_sequence += 1
        clusters = cluster_points(self.scan.points, self.cfg.cluster_distance, self.cfg.cluster_min_points)
        self.publish_obstacles(clusters)
        was_tracking = self.tracker.valid
        if self.activation.active and not self.tracker.update(self.scan.points, stamp=stamp):
            self.state, self.reason = 'TRACKING_LOST', f'Scan matching failed: {self.tracker.reason}; retry required'
            if was_tracking:
                self.get_logger().warning(self.reason)
            self.follower.path = []
            self.path_pub.publish(self.path_message([]))

    def path_message(self, points):
        msg = Path()
        msg.header.frame_id = self.base_frame
        msg.header.stamp = self.scan_header.stamp if self.scan_header else self.get_clock().now().to_msg()
        for x, y, yaw in points:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x, pose.pose.position.y = float(x), float(y)
            pose.pose.orientation.z, pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            msg.poses.append(pose)
        return msg

    def marker(self, namespace, index, kind):
        m = Marker()
        m.header.frame_id = self.base_frame
        m.header.stamp = self.scan_header.stamp if self.scan_header else self.get_clock().now().to_msg()
        m.ns, m.id, m.type = namespace, index, kind
        m.pose.orientation.w = 1.0
        m.lifetime.sec, m.lifetime.nanosec = 0, 600000000
        m.color.a = 1.0
        return m

    def publish_obstacles(self, clusters):
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers = [clear]
        for index, cluster in enumerate(clusters):
            rgb = colorsys.hsv_to_rgb((index * 0.618) % 1.0, 0.8, 1.0)
            points = self.marker('cluster_points', index, Marker.POINTS)
            points.scale.x = points.scale.y = .035
            points.color.r, points.color.g, points.color.b = rgb
            points.points = [Point(x=float(x), y=float(y), z=0.02) for x, y in cluster.points]
            markers.append(points)
            xmin, xmax, ymin, ymax = cluster.bounds
            box = self.marker('cluster_box', index, Marker.LINE_STRIP)
            box.scale.x = .015
            box.color.r, box.color.g, box.color.b = rgb
            box.points = [Point(x=float(x), y=float(y), z=.04) for x, y in
                          ((xmin, ymin), (xmax, ymin), (xmax, ymax), (xmin, ymax), (xmin, ymin))]
            markers.append(box)
            label = self.marker('cluster_label', index, Marker.TEXT_VIEW_FACING)
            label.pose.position.x, label.pose.position.y = map(float, cluster.center)
            label.pose.position.z = .18
            label.scale.z = .09
            label.color.r = label.color.g = label.color.b = 1.0
            label.text = f'obstacle {index + 1} ({len(cluster.points)} pts)'
            markers.append(label)
        ego = self.marker('ego', 0, Marker.CUBE)
        ego.scale.x, ego.scale.y, ego.scale.z = self.cfg.vehicle_length, self.cfg.vehicle_width, .10
        ego.pose.position.z = .05
        ego.color.r, ego.color.g, ego.color.b = .1, 1.0, .8
        markers.append(ego)
        self.obstacles_pub.publish(MarkerArray(markers=markers))

    def publish_target(self):
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers = [clear]
        if self.follower.goal is not None and self.tracker.valid:
            goal = local_goal(self.follower.goal, self.tracker.pose)
            m = self.marker('parking_goal', 0, Marker.ARROW)
            m.pose.position.x, m.pose.position.y = goal.x, goal.y
            m.pose.position.z = .05
            m.pose.orientation.z, m.pose.orientation.w = math.sin(goal.yaw / 2), math.cos(goal.yaw / 2)
            m.scale.x, m.scale.y, m.scale.z = .30, .06, .06
            m.color.g, m.color.b = 1.0, .2
            markers.append(m)
        self.target_pub.publish(MarkerArray(markers=markers))

    def publish_command(self, speed, steering):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base_frame
        msg.drive.speed, msg.drive.steering_angle = float(speed), float(steering)
        self.command_pub.publish(msg)

    def poll_plan(self):
        if self.pending is None or not self.pending[0].done():
            return
        future, epoch, origin, started = self.pending
        self.pending = None
        try:
            goal, path, final = future.result()
        except Exception as exc:
            self.get_logger().error(f'Parking planner failed: {exc}')
            self.reason = 'Planner failure'
            return
        if epoch != self.epoch or not self.activation.active or not self.tracker.valid or self.follower.parked:
            return
        # Plans are transformed from their captured scan frame to the mission
        # origin; the latest scan still checks the immediate swept path.
        if self.now_seconds() - started > self.scan_timeout + self.cfg.planning_timeout:
            return
        if path:
            self.follower.accept(goal, path, final, origin)
            self.reason = ''
        elif not self.follower.path:
            self.reason = 'No visible reachable gap/path'

    def tick(self):
        now = self.now_seconds()
        if self.last_now is not None and now < self.last_now - .01:
            if self.activation.active:
                self.publish_command(0.0, 0.0)
            self.reset_state()
        self.last_now = now
        self.activation.expire(now)
        self.poll_plan()
        speed, steering = 0.0, 0.0
        if not self.activation.active:
            self.state = 'WAITING_SIGN'
        elif self.scan is None or self.scan_time is None or not 0 <= now - self.scan_time <= self.scan_timeout:
            self.state = 'WAITING_SCAN'
            # Preserve the recorded departure route across a sensor interruption.
            if not self.follower.parked:
                self.follower.path = []
            self.clear_visuals()
        elif not self.tracker.valid:
            self.state = 'TRACKING_LOST'
        else:
            speed, steering, self.state = self.follower.command(
                self.scan, self.tracker.pose, self.controlled_sequence != self.scan_sequence, now=now)
            self.controlled_sequence = self.scan_sequence
            wall_now = time.monotonic()
            if not self.follower.parked and self.pending is None and wall_now - self.last_plan_wall >= self.plan_period:
                cfg = replace(self.cfg, row_yaw=self.cfg.row_yaw - self.tracker.pose[2])
                goal = local_goal(self.follower.goal, self.tracker.pose) if self.follower.goal else None
                self.pending = (self.worker.submit(build_plan, self.scan, cfg, goal),
                                self.epoch, self.tracker.pose, now)
                self.last_plan_wall = wall_now
            self.path_pub.publish(self.path_message(self.follower.local_path(self.tracker.pose)))
            self.publish_target()
        if self.activation.active:
            self.publish_command(speed, steering)
        if time.monotonic() - self.last_status_wall >= .2:
            self.active_pub.publish(Bool(data=self.activation.active))
            self.status_pub.publish(String(data=json.dumps({
                'state': self.state, 'active': self.activation.active,
                'mission_attempt': self.attempt,
                'consecutive_detections': self.activation.count, 'reason': self.reason,
                'path_poses': len(self.follower.path), 'scan_match_error_m': self.tracker.error,
                'speed': speed, 'steering_angle': steering,
                'parking_completed': self.follower.parked, 'exit_completed': self.follower.exited,
                'hold_remaining_seconds': (max(0., self.hold_seconds - (now - self.follower.parked_since))
                                           if self.follower.parked_since is not None else 0.),
            }, allow_nan=False)))
            self.last_status_wall = time.monotonic()

    def destroy_node(self):
        if self.activation.active and rclpy.ok(context=self.context):
            self.publish_command(0.0, 0.0)
        self.worker.shutdown(wait=True, cancel_futures=True)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ParkingNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
