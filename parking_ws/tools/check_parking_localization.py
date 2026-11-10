#!/usr/bin/env python3
"""Observe real sensors through a shadow parking node; all control/sign topics
are remapped to /parking/check. No start signs or vehicle commands are sent.
Run with --start-localization only when the new parking EKF is not running.
"""
import argparse
import json
import math
import os
import signal
from pathlib import Path
import subprocess
import sys
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from ackermann_msgs.msg import AckermannDriveStamped
import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'parking_ws/src/autorace_parking'))
from autorace_parking.node import ParkingNode


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=30.)
    parser.add_argument('--start-localization', action='store_true')
    args = parser.parse_args()
    rclpy.init()
    probe = Node('parking_localization_probe')
    children = []
    executor = SingleThreadedExecutor()
    parking = None
    commands = []
    poses, ekf_poses, stationary_poses = [], [], []
    try:
        executor.add_node(probe)
        warm = time.monotonic()+1.
        while time.monotonic()<warm: executor.spin_once(timeout_sec=.05)
        if args.start_localization:
            if probe.count_publishers('/odometry/filtered') or probe.count_publishers('/localization/health'):
                raise RuntimeError('Parking localization already running; omit --start-localization')
            children.append(subprocess.Popen([sys.executable, str(ROOT/'sensor_ws/src/autorace_sensor_bringup/scripts/sensor_fusion.py')], start_new_session=True))
            children.append(subprocess.Popen([
                '/opt/ros/humble/bin/ros2', 'run', 'robot_localization', 'ekf_node', '--ros-args',
                '--params-file', str(ROOT/'sensor_ws/src/autorace_sensor_bringup/config/limo_ekf.yaml'),
                '-r', '__node:=ekf_filter_node_odom', '-r', 'odometry/filtered:=/odometry/filtered'], start_new_session=True))
        params = yaml.safe_load((ROOT/'parking_ws/src/autorace_parking/config/parking_real.yaml').read_text())['parking_node']['ros__parameters']
        remaps = ['--ros-args', '-r', '__node:=parking_localization_check']
        for topic in ('/mission/parking/command_03', '/yolo/parking_sign', '/parking/reset',
                      '/parking/path', '/parking/obstacles', '/parking/target', '/parking/status',
                      '/parking/active', '/parking/localization/odometry'):
            remaps += ['-r', topic+':=/parking/check/'+topic.rsplit('/', 1)[-1]]
        parking = ParkingNode(parameter_overrides=[Parameter(k, value=v) for k,v in params.items()],
                              cli_args=remaps, use_global_arguments=False)
        executor.add_node(parking)
        sub = probe.create_subscription(AckermannDriveStamped, '/parking/check/command_03', commands.append, 10)
        start = time.monotonic()
        next_output = 0.
        while time.monotonic()-start < args.seconds:
            executor.spin_once(timeout_sec=.02)
            for child in children:
                if child.poll() is not None:
                    raise RuntimeError(f'Localization child exited with code {child.returncode}')
            if parking.tracker.valid and parking.localization_ready(parking.now_seconds()):
                poses.append(parking.tracker.global_pose)
                ekf_poses.append(parking.tracker.sample.pose)
                if parking.tracker.stationary:
                    stationary_poses.append(parking.tracker.global_pose)
                else:
                    stationary_poses.clear()
            elapsed = time.monotonic()-start
            if elapsed >= next_output:
                print(json.dumps({'seconds':round(elapsed,1), 'localization':parking.tracker.diagnostics(parking.now_seconds()),
                                  'sensor_health':parking.fusion_health}),flush=True)
                next_output = elapsed+5.
        if commands:
            raise AssertionError('Shadow node emitted unexpected control commands')
        if len(poses)<20:
            raise AssertionError('Insufficient valid fused localization samples')
        def spread(points):
            if not points: return None
            first=points[0]
            return {'position_m':max(math.hypot(p[0]-first[0],p[1]-first[1]) for p in points),
                    'yaw_rad':max(abs(math.atan2(math.sin(p[2]-first[2]),math.cos(p[2]-first[2]))) for p in points)}
        print(json.dumps({'valid_samples':len(poses), 'shadow_commands':len(commands),
                          'fused_spread':spread(poses), 'ekf_spread':spread(ekf_poses),
                          'last_stationary_run_spread':spread(stationary_poses)},indent=2))
    finally:
        if parking is not None:
            executor.remove_node(parking); parking.destroy_node()
        executor.remove_node(probe); probe.destroy_node(); executor.shutdown(); rclpy.shutdown()
        for child in reversed(children):
            try: os.killpg(child.pid, signal.SIGINT)
            except ProcessLookupError: pass
            try: child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGKILL); child.wait()


if __name__ == '__main__':
    main()
