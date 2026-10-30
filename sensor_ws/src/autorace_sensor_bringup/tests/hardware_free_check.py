#!/usr/bin/env python3
"""Run with sourced ROS environment, isolated ROS_DOMAIN_ID and ROS_LOG_DIR."""
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from nav_msgs.msg import Odometry


def main():
    from ament_index_python.packages import get_package_prefix
    workspace = Path(__file__).resolve().parents[3]
    for package in ('autorace_sensor_bringup', 'limo_base', 'limo_msgs',
                    'limo_description', 'ydlidar_ros2_driver', 'orbbec_camera',
                    'orbbec_camera_msgs', 'robot_localization'):
        prefix = Path(get_package_prefix(package))
        assert prefix.parent == workspace / 'install', (package, prefix)
    for variable in ('AMENT_PREFIX_PATH', 'CMAKE_PREFIX_PATH', 'COLCON_PREFIX_PATH',
                     'PYTHONPATH', 'LD_LIBRARY_PATH'):
        assert '/home/wego/wego_ws' not in os.environ.get(variable, ''), variable
    root = Path(os.environ['ROS_LOG_DIR'])
    root.mkdir(parents=True, exist_ok=True)
    previous = set(root.glob('*/session.json'))
    output = root / 'verification_console.log'
    with output.open('w') as console:
        proc = subprocess.Popen([
            'ros2', 'launch', 'autorace_sensor_bringup', 'sensors.launch.py',
            'enable_base:=false', 'enable_lidar:=false', 'enable_camera:=false',
            'camera_name:=test_camera', 'startup_timeout:=2.0', 'stale_timeout:=1.0',
        ], stdout=console, stderr=subprocess.STDOUT, start_new_session=True)
        node = None
        try:
            deadline = time.monotonic() + 15
            while time.monotonic() < deadline:
                new = set(root.glob('*/session.json')) - previous
                if new:
                    folder = next(iter(new)).parent
                    break
                time.sleep(0.1)
            else:
                raise AssertionError('No session log')

            def events():
                path = folder / 'sensor_events.jsonl'
                return [json.loads(line)['event'] for line in path.read_text().splitlines()] if path.exists() else []

            def wait_for(event):
                deadline = time.monotonic() + 12
                while time.monotonic() < deadline:
                    if event in events():
                        return
                    time.sleep(0.1)
                raise AssertionError(f'Missing {event}: {events()}')

            wait_for('NO_DATA')
            rclpy.init()
            node = rclpy.create_node('synthetic_sensor_verification')
            publisher = node.create_publisher(Odometry, '/odometry/filtered', 10)
            deadline = time.monotonic() + 8
            while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
                time.sleep(0.1)
            assert publisher.get_subscription_count() > 0
            publisher.publish(Odometry())
            wait_for('READY')
            wait_for('STALE')
            publisher.publish(Odometry())
            wait_for('RECOVERED')
        finally:
            if node is not None:
                node.destroy_node()
                rclpy.shutdown()
            if proc.poll() is None:
                os.killpg(proc.pid, signal.SIGINT)
            proc.wait(timeout=15)
        assert proc.returncode == 0, (proc.returncode, output)
        process = [json.loads(line) for line in (folder / 'process_events.jsonl').read_text().splitlines()]
        starts = [e for e in process if e['event'] == 'PROCESS_START']
        exits = [e for e in process if e['event'] == 'PROCESS_EXIT']
        assert len(starts) == len(exits) == 7, process
        assert {e['pid'] for e in starts} == {e['pid'] for e in exits}
        assert all('returncode' in e for e in exits)
        assert all('limo_base' not in ' '.join(e['cmd']) for e in starts)
        assert any('test_camera_link' in e['cmd'] for e in starts)
        assert any(e['event'] == 'LAUNCH_SHUTDOWN' for e in process)
        assert len(list((folder / 'process_output').glob('*.log'))) == 14
        print(f'PASS: process lifecycle, stdout/stderr, NO_DATA → READY → STALE → RECOVERED\n{folder}')


if __name__ == '__main__':
    main()
