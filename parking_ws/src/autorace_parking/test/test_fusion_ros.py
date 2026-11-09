import json
import math
import time

import pytest
import rclpy
from rclpy.parameter import Parameter
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Bool, String

from autorace_parking.core import Gap
from autorace_parking.node import ParkingNode
from sensor_fusion import SensorFusion


def odometry(stamp, x=0., speed=0.):
    msg = Odometry()
    ns = round(stamp*1e9)
    msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(ns, 10**9)
    msg.header.frame_id, msg.child_frame_id = 'odom', 'base_link'
    msg.pose.pose.position.x = x
    msg.pose.pose.orientation.w = 1.
    for i in (0, 7, 35): msg.pose.covariance[i] = .001
    msg.twist.twist.linear.x = speed
    return msg


def clear_scan(stamp):
    msg = LaserScan()
    ns = round(stamp*1e9)
    msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(ns, 10**9)
    msg.header.frame_id = 'laser_link'
    msg.angle_min, msg.angle_increment = -math.pi, math.pi/180
    msg.range_min, msg.range_max = .01, 12.
    msg.ranges = [10.]*361
    return msg


def test_fused_parking_waits_for_sensor_health_and_stops_on_raw_input_loss():
    rclpy.init()
    node = ParkingNode(parameter_overrides=[Parameter('localization_source', value='fused'), Parameter('odom_frame', value='odom')])
    commands = []
    node.publish_command = lambda speed, steering: commands.append((speed, steering))
    node.now_seconds = lambda: 20.2
    try:
        for _ in range(3): node.on_sign(Bool(data=True))
        node.on_localization(odometry(20.0))
        node.on_localization(odometry(20.1))
        node.on_scan(clear_scan(20.1))
        node.tick()
        assert node.state == 'WAITING_LOCALIZATION' and not commands
        node.on_fusion_health(String(data=json.dumps({'stamp':20.2,'ready':True})))
        node.follower.accept(Gap(.8,0.,0.,.8,0,1), [(0.,0.,0.),(.8,0.,0.)], True, (0.,0.,0.))
        node.tick()
        assert node.state == 'PARKING' and commands[-1][0] > 0.
        # An EKF may keep publishing predictions after raw inputs disappear.
        # A fresh filter timestamp must never bypass the independent watchdog.
        node.now_seconds = lambda: 21.
        node.on_localization(odometry(21.))
        node.on_scan(clear_scan(21.))
        node.tick()
        assert node.state == 'WAITING_LOCALIZATION' and commands[-1] == (0.,0.)
        node.on_fusion_health(String(data=json.dumps({'stamp':21.,'ready':False})))
        node.tick()
        assert commands[-1] == (0.,0.)
    finally:
        node.destroy_node(); rclpy.shutdown()


def test_sensor_conditioning_publishes_measured_gyro_and_realistic_noise():
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    rclpy.init()
    node = SensorFusion(parameter_overrides=[Parameter('imu_frame', value='base_link')])
    node.now = lambda: 20.
    probe = Node('fusion_conditioning_probe')
    executor = SingleThreadedExecutor()
    executor.add_node(node); executor.add_node(probe)
    received = []
    sub = probe.create_subscription(Imu, '/localization/imu', received.append, 10)
    def spin(duration):
        deadline = time.monotonic()+duration
        while time.monotonic()<deadline: executor.spin_once(timeout_sec=.005)
    try:
        spin(.2)
        for i in range(24):
            now = 20.+i*.1
            node.now = lambda value=now: value
            wheel = odometry(now)
            node.on_wheel(wheel)
            imu = Imu()
            imu.header = wheel.header
            imu.header.frame_id = 'base_link'
            imu.angular_velocity.z = .015
            node.on_imu(imu)
        spin(.15)
        assert node.calibration.bias == pytest.approx(.015)
        assert received
        assert received[-1].angular_velocity.z == 0.
        assert received[-1].angular_velocity_covariance[8] == pytest.approx(.015**2)
        assert received[-1].orientation_covariance[0] == -1.
        assert imu.angular_velocity.z == .015  # input was not mutated
    finally:
        executor.remove_node(node); executor.remove_node(probe)
        node.destroy_node(); probe.destroy_node(); executor.shutdown(); rclpy.shutdown()


def test_delayed_ekf_sample_replays_scan_without_erasing_accepted_path():
    rclpy.init()
    node=ParkingNode(parameter_overrides=[Parameter('localization_source',value='fused'),Parameter('odom_frame',value='odom')])
    node.now_seconds=lambda:20.2
    try:
        for _ in range(3):node.on_sign(Bool(data=True))
        node.on_localization(odometry(20.0))
        node.on_localization(odometry(20.1))
        node.on_scan(clear_scan(20.1))
        node.follower.accept(Gap(.8,0.,0.,.8,0,1),[(0.,0.,0.),(.8,0.,0.)],True,(0.,0.,0.))
        route=node.follower.path.copy()
        node.on_scan(clear_scan(20.16))
        assert node.last_scan_stamp==pytest.approx(20.1)
        assert len(node.pending_scans)==1 and node.follower.path==route
        assert node.tracker.valid
        node.on_localization(odometry(20.2))
        assert node.last_scan_stamp==pytest.approx(20.16)
        assert not node.pending_scans and node.follower.path==route
        assert node.tracker.valid
    finally:
        node.destroy_node();rclpy.shutdown()


def test_scan_waiting_for_alignment_expires_instead_of_being_replayed_late():
    rclpy.init()
    node=ParkingNode(parameter_overrides=[Parameter('localization_source',value='fused'),Parameter('odom_frame',value='odom')])
    commands=[]
    node.publish_command=lambda speed,steering:commands.append((speed,steering))
    node.now_seconds=lambda:20.2
    try:
        for _ in range(3):node.on_sign(Bool(data=True))
        node.on_localization(odometry(20.0))
        node.on_localization(odometry(20.1))
        node.on_scan(clear_scan(20.1))
        node.on_scan(clear_scan(20.16))
        assert len(node.pending_scans)==1
        node.now_seconds=lambda:21.
        node.on_localization(odometry(21.))
        assert not node.pending_scans
        # A real EKF outage now invalidates the old matching frame entirely.
        assert node.last_scan_stamp is None
        assert node.localization_recovering and node.tracker.matcher is None
        node.tick()
        assert commands and commands[-1] == (0., 0.)
        assert node.state=='WAITING_LOCALIZATION'
    finally:
        node.destroy_node();rclpy.shutdown()
