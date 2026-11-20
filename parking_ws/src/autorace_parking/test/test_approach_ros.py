"""Exercise the current parking start/ready handshake with the real lane PID."""
from pathlib import Path
import signal
import subprocess
import time

import pytest


def test_parking_start_stops_lane_until_a_fresh_ready_event(monkeypatch, tmp_path):
    import rclpy
    from ament_index_python.packages import get_package_prefix, PackageNotFoundError
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.executors import SingleThreadedExecutor
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
    parking = ParkingNode(parameter_overrides=[Parameter('localization_source', value='fused')])
    probe = Node('parking_approach_integration_probe')
    executor = SingleThreadedExecutor()
    executor.add_node(parking); executor.add_node(probe)
    targets = probe.create_publisher(Float32MultiArray, '/lane_fusion/target', 10)
    events, commands = [], []
    subscriptions = [
        probe.create_subscription(Bool, '/parking_start', events.append, 10),
        probe.create_subscription(AckermannDriveStamped, '/lane_fusion_pid/command', commands.append, 10),
    ]
    def spin(seconds):
        end = time.monotonic() + seconds
        last = 0.
        while time.monotonic() < end:
            assert process.poll() is None, (tmp_path / 'lane.log').read_text()
            if time.monotonic() - last > .02:
                targets.publish(Float32MultiArray(data=[1., 0., 0., .5, 0., 1.]))
                last = time.monotonic()
            executor.spin_once(timeout_sec=.005)
    try:
        spin(1.5)
        assert commands and commands[-1].drive.speed > 0. and not events
        parking.request_parking_start()
        spin(.3)
        assert len(events) == 1 and events[0].data
        assert commands[-1].drive.speed == 0.

        # A ready message from an earlier initialization cannot release this stop.
        stale = Header()
        ns = probe.get_clock().now().nanoseconds - 1_000_000_000
        stale.stamp.sec, stale.stamp.nanosec = divmod(ns, 1_000_000_000)
        parking.ready_pub.publish(stale)
        spin(.2)
        assert commands[-1].drive.speed == 0.

        # The parking publisher's fresh completion event restores lane output.
        parking.ready_pub.publish(Header(stamp=parking.get_clock().now().to_msg()))
        spin(.3)
        assert commands[-1].drive.speed > 0.
        assert len(events) == 1  # Control ticks never retrigger initialization.
    finally:
        process.send_signal(signal.SIGINT)
        try: process.wait(timeout=5)
        except subprocess.TimeoutExpired: process.kill(); process.wait()
        log.close(); executor.shutdown(); parking.destroy_node(); probe.destroy_node(); rclpy.shutdown()
