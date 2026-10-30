#!/usr/bin/env python3
"""Verify both TF profiles without starting any sensor driver or EKF."""
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, QoSProfile
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener


def check_profile(profile):
    root = Path(os.environ['ROS_LOG_DIR'])
    root.mkdir(parents=True, exist_ok=True)
    with (root / f'tf_{profile}.log').open('w') as output:
        proc = subprocess.Popen([
            'ros2', 'launch', 'autorace_sensor_bringup', 'sensors.launch.py',
            'enable_base:=false', 'enable_lidar:=false', 'enable_camera:=false',
            'enable_ekf:=false', f'tf_profile:={profile}',
            'camera_name:=test_camera', 'degree:=0.2',
        ], stdout=output, stderr=subprocess.STDOUT, start_new_session=True)
        node = rclpy.create_node('tf_profile_verification')
        buffer = Buffer()
        listener = TransformListener(buffer, node)
        owners = {}

        def inspect(msg):
            for transform in msg.transforms:
                owners.setdefault(transform.child_frame_id, set()).add(transform.header.frame_id)

        subscription = node.create_subscription(
            TFMessage, '/tf_static', inspect,
            QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        try:
            frames = ('base_footprint', 'laser_link', 'imu_link', 'test_camera_link')
            deadline = time.monotonic() + 15
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
                if all(buffer.can_transform('base_link', frame, rclpy.time.Time()) for frame in frames):
                    break
            else:
                raise AssertionError(f'TF missing: {profile}; see {root}')
            # Allow all durable publishers to deliver for the duplicate-owner check.
            until = time.monotonic() + 1
            while time.monotonic() < until:
                rclpy.spin_once(node, timeout_sec=0.1)
            if profile == 'wego_limo_pro':
                expected = {'base_footprint': (0, 0, -0.15),
                            'laser_link': (0.103, 0, -0.034), 'imu_link': (0, 0, -0.1)}
                mount = (0.103, 0, 0.06)
            else:
                expected = {'base_footprint': (0.1, 0, 0.1),
                            'laser_link': (0.203, 0, 0.066), 'imu_link': (0.1, 0, 0)}
                mount = (0.2, 0.1, 0.06)
            expected['test_camera_link'] = (
                mount[0] + 0.03 * math.cos(0.2), mount[1] - 0.05,
                mount[2] - 0.03 * math.sin(0.2))
            for frame, xyz in expected.items():
                transform = buffer.lookup_transform('base_link', frame, rclpy.time.Time(), Duration())
                actual = transform.transform.translation
                assert all(math.isclose(a, b, abs_tol=1e-8) for a, b in zip((actual.x, actual.y, actual.z), xyz)), (profile, frame, actual, xyz)
            camera = buffer.lookup_transform('base_link', 'test_camera_link', rclpy.time.Time())
            assert math.isclose(camera.transform.rotation.y, math.sin(0.1), abs_tol=1e-8)
            assert all(len(parents) == 1 for parents in owners.values()), owners
            publishers = node.get_publishers_info_by_topic('/tf_static')
            expected_publishers = {'robot_state_publisher', 'base_link_to_camera_mount',
                                   'camera_mount_to_rotate', 'rotate_to_camera_link'}
            assert len(publishers) == 4, publishers
            assert {p.node_name for p in publishers} == expected_publishers, publishers
            assert {'laser_link', 'imu_link', 'test_camera_link'}.issubset(owners), owners
            print(f'PASS {profile}: sensor transforms, camera pitch/name and static TF publisher count/parent consistency')
        finally:
            listener.unregister()
            node.destroy_subscription(subscription)
            node.destroy_node()
            if proc.poll() is None:
                # Signal launch only; launch forwards it to each child once.
                proc.send_signal(signal.SIGINT)
            proc.wait(timeout=15)
        assert proc.returncode == 0, proc.returncode


def main():
    rclpy.init()
    try:
        check_profile('wego_limo_pro')
        check_profile('existing')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
