#!/usr/bin/env python3
"""Exercise real ROS conditioning + C++ EKF + parking against simulated sensors.
Requires robot_localization and a private ROS_DOMAIN_ID in 160..199. Control and
signs are additionally remapped to /parking/test; no hardware node is launched.
"""
import json
import argparse
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from collections import deque
from bisect import bisect_left

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Bool, String
from rosgraph_msgs.msg import Clock

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT/'parking_ws/src/autorace_parking'), str(ROOT/'simul/src/limo_parking_sim')]
from limo_parking_sim.config import load_config
from limo_parking_sim.model import Simulation


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--speed',type=float,default=.5)
    parser.add_argument('--real-time-factor',type=float,default=1.)
    args=parser.parse_args()
    if not .1 <= args.real_time_factor <= 1.: raise ValueError('real-time-factor must be in .1..1')
    if not 160 <= int(os.environ.get('ROS_DOMAIN_ID', '-1')) <= 199:
        raise RuntimeError('Set a private ROS_DOMAIN_ID in 160..199, e.g. 178')
    rclpy.init()
    probe = Node('fused_pipeline_probe')
    executor = SingleThreadedExecutor()
    executor.add_node(probe)
    children = []
    report = None
    try:
        deadline=time.monotonic()+.5
        while time.monotonic()<deadline: executor.spin_once(timeout_sec=.02)
        if any(name != probe.get_name() for name in probe.get_node_names()):
            raise RuntimeError('Private test domain contains other nodes')
        def launch(arguments):
            children.append(subprocess.Popen(arguments+['-p','use_sim_time:=true'], start_new_session=True))
        launch([sys.executable, str(ROOT/'sensor_ws/src/autorace_sensor_bringup/scripts/sensor_fusion.py'), '--ros-args',
                '-p', 'wheel_topic:=/parking/test/wheel', '-p', 'imu_topic:=/parking/test/imu',
                '-p', 'imu_frame:=base_link'])
        launch(['/opt/ros/humble/bin/ros2','run','robot_localization','ekf_node','--ros-args',
                '--params-file',str(ROOT/'sensor_ws/src/autorace_sensor_bringup/config/limo_ekf.yaml'),
                '-r','__node:=ekf_filter_node_odom','-r','odometry/filtered:=/odometry/filtered'])
        launch([sys.executable, '-m', 'autorace_parking.node', '--ros-args',
                '--params-file', str(ROOT/'parking_ws/src/autorace_parking/config/parking_real.yaml'),
                '-p',f'parking_speed:={args.speed}','-p','max_steering:=0.408','-p','plan_period:=0.5',
                '-r','/mission/parking/command_03:=/parking/test/command',
                '-r','/yolo/parking_sign:=/parking/test/sign',
                '-r','/parking/status:=/parking/test/status'])
        status = {}; health = {}
        def on_status(msg):
            status.clear(); status.update(json.loads(msg.data))
        def on_health(msg):
            health.clear(); health.update(json.loads(msg.data))
        status_sub = probe.create_subscription(String, '/parking/test/status', on_status, 10)
        health_sub = probe.create_subscription(String, '/localization/health', on_health, 10)
        wheel_pub=probe.create_publisher(Odometry,'/parking/test/wheel',10)
        imu_pub=probe.create_publisher(Imu,'/parking/test/imu',10)
        scan_pub=probe.create_publisher(LaserScan,'/scan',10)
        sign_pub=probe.create_publisher(Bool,'/parking/test/sign',10)
        clock_pub=probe.create_publisher(Clock,'/clock',10)
        latest={'speed':0.,'steering':0.,'at':0.}
        def command(msg):
            latest.update(speed=msg.drive.speed,steering=msg.drive.steering_angle,at=time.monotonic())
        sub=probe.create_subscription(AckermannDriveStamped,'/parking/test/command',command,10)
        sim=Simulation(load_config()); origin=(sim.x,sim.y,sim.yaw)
        queued=deque(); next_scan=0.; signs=0; next_sign=0.; previous=time.monotonic()
        started=previous; last_output=-5.; parked_inside=False; max_error=0.
        simulated_elapsed=0.; clock_origin=time.time(); truth_history=deque(maxlen=3000)
        while simulated_elapsed < 70. and time.monotonic()-started < 180.:
            loop=time.monotonic(); dt=(loop-previous)*args.real_time_factor; previous=loop
            simulated_elapsed+=dt
            if any(child.poll() is not None for child in children): raise RuntimeError('A pipeline process exited unexpectedly')
            speed,steering=(latest['speed'],latest['steering']) if loop-latest['at']<.25/args.real_time_factor else (0.,0.)
            sim.command(speed,speed*math.tan(steering)/.2)
            for _ in range(max(1,math.ceil(dt/.01))):
                sim.step(dt/max(1,math.ceil(dt/.01)))
            if sim.collision: raise AssertionError(f'Collision: {sim.collision_names}')
            clock=Clock(); ns=round((clock_origin+simulated_elapsed)*1e9)
            clock.clock.sec,clock.clock.nanosec=divmod(ns,10**9)
            clock_pub.publish(clock); stamp=clock.clock
            truth_history.append((clock_origin+simulated_elapsed,sim.x-origin[0],sim.y-origin[1]))
            wheel=Odometry(); wheel.header.stamp=stamp
            wheel.header.frame_id='odom'; wheel.child_frame_id='base_link'; wheel.pose.pose.orientation.w=1.
            wheel.twist.twist.linear.x=float(sim.speed*1.02)
            wheel.twist.twist.angular.z=float(sim.yaw_rate)
            wheel_pub.publish(wheel)
            imu=Imu(); imu.header.stamp=stamp; imu.header.frame_id='base_link'
            imu.angular_velocity.z=float(sim.yaw_rate+.015); imu.orientation.w=1.
            imu_pub.publish(imu)
            if simulated_elapsed>=next_scan:
                scan=LaserScan(); scan.header.stamp=stamp; scan.header.frame_id='laser_link'
                scan.angle_min=float(sim.angles[0]); scan.angle_increment=float(sim.angles[1]-sim.angles[0])
                scan.range_min=.01; scan.range_max=64.; scan.ranges=sim.scan().tolist()
                queued.append((simulated_elapsed+.10,scan)); next_scan=simulated_elapsed+.1
            while queued and queued[0][0]<=simulated_elapsed: scan_pub.publish(queued.popleft()[1])
            if health.get('ready') and status.get('localization', {}).get('valid') and signs<3 and simulated_elapsed>=next_sign:
                sign_pub.publish(Bool(data=True)); signs+=1; next_sign=simulated_elapsed+.1
            spin_until=time.monotonic()+.015
            while time.monotonic()<spin_until: executor.spin_once(timeout_sec=.001)
            elapsed=time.monotonic()-started
            if status.get('active') and status.get('localization', {}).get('valid'):
                estimate=status['localization']['pose']
                stamp=status['localization']['pose_stamp']
                truth=list(truth_history); index=bisect_left([p[0] for p in truth],stamp)
                if 0<index<len(truth):
                    a,b=truth[index-1],truth[index]; fraction=(stamp-a[0])/(b[0]-a[0])
                    tx,ty=a[1]+fraction*(b[1]-a[1]),a[2]+fraction*(b[2]-a[2])
                    max_error=max(max_error,math.hypot(tx-estimate[0],ty-estimate[1]))
            if status.get('parking_completed') and not parked_inside:
                xmin,xmax,ymin,ymax=sim.cfg['world']['parking']['goal']
                parked_inside=all(xmin<=x<=xmax and ymin<=y<=ymax for x,y in sim.body().corners())
                if status.get('require_parking_heading',True) and not parked_inside:
                    raise AssertionError(f'PARKED outside bay: truth={(sim.x,sim.y,sim.yaw)}, status={status}')
            if elapsed-last_output>=5.:
                print(json.dumps({'seconds':round(elapsed,1),'state':status.get('state'),'reason':status.get('reason'),
                                  'command':latest['speed'], 'localization':status.get('localization')}),flush=True)
                last_output=elapsed
            if status.get('exit_completed'):
                report={'parked':status.get('parking_completed',False),'whole_vehicle_inside_bay':parked_inside,'exited':True,'collision_count':sim.collisions,
                        'duration_seconds':round(simulated_elapsed,2),'wall_seconds':round(elapsed,2),
                        'max_tracking_position_error_m':max_error,
                        'lidar_corrections':status['localization']['lidar_corrections'],
                        'gyro_bias_rad_s':health.get('gyro_bias_rad_s'),'control_topic':'/parking/test/command'}
                break
            delay=.02/args.real_time_factor-(time.monotonic()-loop)
            if delay>0:time.sleep(delay)
        if report is None: raise AssertionError(f'Pipeline did not finish: {status}')
        print(json.dumps(report,indent=2))
    finally:
        executor.remove_node(probe); probe.destroy_node()
        executor.shutdown(); rclpy.shutdown()
        for child in reversed(children):
            try: os.killpg(child.pid, signal.SIGINT)
            except ProcessLookupError: pass
            try: child.wait(timeout=5)
            except subprocess.TimeoutExpired: os.killpg(child.pid, signal.SIGKILL); child.wait()


if __name__=='__main__': main()
