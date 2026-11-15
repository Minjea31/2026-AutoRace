"""Exercise the real lane event and parking subscriber on an isolated DDS domain."""
from pathlib import Path
import signal
import subprocess
import time

import pytest


def test_lane_slowdown_event_reaches_parking_without_resetting_ekf(monkeypatch, tmp_path):
    import rclpy
    from ament_index_python.packages import get_package_prefix, PackageNotFoundError
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.executors import SingleThreadedExecutor
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Bool, Header, Float32MultiArray
    from ackermann_msgs.msg import AckermannDriveStamped
    from autorace_parking.node import ParkingNode

    try:
        binary = Path(get_package_prefix('autorace_lane_fusion_pid')) / 'lib/autorace_lane_fusion_pid/lane_fusion_pid_controller'
    except PackageNotFoundError:
        pytest.skip('Build and source lane_fusion_ws for the cross-workspace integration test')
    monkeypatch.setenv('ROS_DOMAIN_ID', '191')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    log = (tmp_path / 'lane.log').open('w')
    process = subprocess.Popen([str(binary), '--ros-args', '-p', 'active:=1.0', '-p', 'speed:=0.6'], stdout=log, stderr=subprocess.STDOUT)
    rclpy.init(domain_id=191)
    parking = ParkingNode(parameter_overrides=[Parameter('localization_source', value='fused'),
                                             Parameter('white_roi_enabled', value=True)])
    probe = Node('parking_approach_integration_probe')
    executor = SingleThreadedExecutor()
    executor.add_node(parking); executor.add_node(probe)
    signs = probe.create_publisher(Bool, '/yolo/parking_sign', 10)
    targets = probe.create_publisher(Float32MultiArray, '/lane_fusion/target', 10)
    odometry = probe.create_publisher(Odometry, '/odometry/filtered', 10)
    events, relative, commands = [], [], []
    subscriptions = [
        probe.create_subscription(Header, '/parking/approach_start', events.append, 10),
        probe.create_subscription(Odometry, '/parking/approach_odometry', relative.append, 10),
        probe.create_subscription(AckermannDriveStamped, '/lane_fusion_pid/command', commands.append, 10),
    ]
    start = time.monotonic()
    def spin(seconds):
        end = time.monotonic() + seconds
        last = 0.
        while time.monotonic() < end:
            assert process.poll() is None, (tmp_path / 'lane.log').read_text()
            if time.monotonic() - last > .02:
                msg = Odometry(); msg.header.stamp = probe.get_clock().now().to_msg()
                msg.header.frame_id, msg.child_frame_id = 'parking_odom', 'base_link'
                msg.pose.pose.position.x = .2*(time.monotonic()-start)
                msg.pose.pose.orientation.w = 1.
                msg.twist.twist.linear.x = .2
                for i in (0, 7, 35): msg.pose.covariance[i] = .001
                odometry.publish(msg)
                targets.publish(Float32MultiArray(data=[1., 0., 0., .5, 0., 1.]))
                last = time.monotonic()
            executor.spin_once(timeout_sec=.005)
    try:
        spin(1.5)
        assert commands and not relative and not events
        signs.publish(Bool(data=True)); spin(.12)
        assert not events
        signs.publish(Bool(data=False)); spin(.1)
        signs.publish(Bool(data=True)); spin(.1)
        assert not events
        signs.publish(Bool(data=True)); spin(.3)
        assert len(events) == 1
        assert relative and relative[-1].header.frame_id == 'parking_approach'
        assert relative[-1].pose.pose.position.x > 0.
        assert commands[-1].drive.speed <= .30001
        origin = parking.approach_reference.origin
        signs.publish(Bool(data=True)); spin(.15)
        assert len(events) == 1 and parking.approach_reference.origin == origin
        assert parking.odometry.samples[-1].pose[0] > relative[-1].pose.pose.position.x
        assert parking.tracker.origin is None  # No scans/map changes are needed for approach odometry.
    finally:
        process.send_signal(signal.SIGINT)
        try: process.wait(timeout=5)
        except subprocess.TimeoutExpired: process.kill(); process.wait()
        log.close(); executor.shutdown(); parking.destroy_node(); probe.destroy_node(); rclpy.shutdown()
