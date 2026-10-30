#!/usr/bin/env python3
"""Explicit hardware test: run only LiDAR, capture SDK bytes and scan quality."""
import argparse
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=240)
    parser.add_argument('--domain', type=int, default=187)
    parser.add_argument('--sdk-debug', action='store_true')
    args = parser.parse_args()
    if args.sdk_debug:
        parser.error('Restored wego_ws driver does not support sdk_debug; omit --sdk-debug')
    if args.duration <= 0:
        parser.error('duration must be positive')
    workspace = Path(__file__).resolve().parents[3]
    params = workspace / 'src/autorace_sensor_bringup/config/ydlidar.yaml'
    import yaml
    port = Path(yaml.safe_load(params.read_text())['ydlidar_ros2_driver_node']['ros__parameters']['port']).resolve()
    for process in Path('/proc').iterdir():
        if not process.name.isdigit():
            continue
        try:
            for fd in (process / 'fd').iterdir():
                try:
                    if fd.resolve() == port:
                        raise RuntimeError(f'{port} already open by PID {process.name}; refusing duplicate driver')
                except FileNotFoundError:
                    pass
        except (FileNotFoundError, PermissionError):
            pass
    os.environ['ROS_DOMAIN_ID'] = str(args.domain)
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan
    from ament_index_python.packages import get_package_prefix
    root = workspace / 'log/lidar_diagnostics' / datetime.now().strftime('%Y%m%d_%H%M%S_isolated')
    root.mkdir(parents=True, exist_ok=False)
    events = root / 'events.jsonl'

    def record(kind, **details):
        row = {'utc': datetime.now(timezone.utc).isoformat(), 'event': kind, **details}
        with events.open('a') as output:
            output.write(json.dumps(row) + '\n')
        print(json.dumps(row), flush=True)

    command = [str(Path(get_package_prefix('ydlidar_ros2_driver')) / 'lib/ydlidar_ros2_driver/ydlidar_ros2_driver_node'),
               '--ros-args', '--params-file', str(params)]
    rclpy.init()
    node = rclpy.create_node('lidar_isolated_check')
    counts = {'messages': 0, 'valid_messages': 0, 'empty_messages': 0}
    last = {}
    def receive(msg):
        valid = sum(math.isfinite(r) and msg.range_min <= r <= msg.range_max for r in msg.ranges)
        counts['messages'] += 1
        counts['valid_messages' if valid else 'empty_messages'] += 1
        last.update(points=len(msg.ranges), valid=valid,
                    nonzero_intensity=sum(i != 0 for i in msg.intensities))
    subscription = node.create_subscription(LaserScan, '/scan', receive, qos_profile_sensor_data)
    with (root / 'stdout.log').open('w') as stdout, (root / 'stderr.log').open('w') as stderr:
        process = subprocess.Popen(command, stdout=stdout, stderr=stderr, env=os.environ.copy(), start_new_session=True)
        record('PROCESS_START', pid=process.pid, command=command, duration=args.duration, domain=args.domain)
        started = time.monotonic()
        reported = started
        try:
            while time.monotonic() - started < args.duration:
                rclpy.spin_once(node, timeout_sec=0.1)
                if process.poll() is not None:
                    raise RuntimeError(f'LiDAR exited early: {process.returncode}')
                if time.monotonic() - reported >= 10:
                    record('SCAN_SUMMARY', elapsed=round(time.monotonic()-started, 1), **counts, latest=last)
                    reported = time.monotonic()
        finally:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                process.terminate()
                process.wait(timeout=5)
            record('PROCESS_EXIT', pid=process.pid, returncode=process.returncode, **counts, latest=last)
            node.destroy_subscription(subscription)
            node.destroy_node()
            rclpy.shutdown()
    print('REPORT', root)


if __name__ == '__main__':
    main()
