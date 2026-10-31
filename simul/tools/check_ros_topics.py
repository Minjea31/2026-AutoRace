#!/usr/bin/env python3
"""Launch an offscreen sim and verify real DDS sensor/control/service exchange.

Run after `conda activate autorace-simul; source simul/env.sh`. Uses the current
ROS_DOMAIN_ID; keep it separate from hardware. Does not install anything.
"""
import argparse
import io
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import time

import numpy as np
from PIL import Image as PILImage
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, Imu, LaserScan
from std_msgs.msg import String
from std_srvs.srv import Empty, SetBool
from tf2_msgs.msg import TFMessage

from limo_parking_sim.config import load_config


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cli-command', action='store_true',
                        help='Verify motion with simul/ros2.sh topic pub instead of a Python publisher')
    options = parser.parse_args()
    cfg = load_config()
    rclpy.init()
    node = Node('parking_sim_interface_check')
    latest,counts,tf = {},{},{}
    subscriptions = []

    def receive(key,msg):
        latest[key] = msg
        counts[key] = counts.get(key,0)+1

    specs = {'compressed':CompressedImage,'color':Image,'depth':Image,'color_info':CameraInfo,
             'depth_info':CameraInfo,'scan':LaserScan,'imu':Imu,'odom':Odometry,
             'clock':Clock,'ground_truth':Odometry,'status':String}
    for key,kind in specs.items():
        subscriptions.append(node.create_subscription(kind,cfg['topics'][key],
            lambda msg,key=key:receive(key,msg),qos_profile_sensor_data))

    def on_tf(msg):
        for t in msg.transforms:
            if t.child_frame_id in tf:
                assert tf[t.child_frame_id].header.frame_id == t.header.frame_id
            tf[t.child_frame_id] = t

    subscriptions.append(node.create_subscription(TFMessage,'/tf',on_tf,10))
    subscriptions.append(node.create_subscription(TFMessage,'/tf_static',on_tf,
        QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)))
    command = None if options.cli_command else node.create_publisher(Twist,cfg['topics']['cmd_vel'],1)
    process = None
    cli_process = None
    log = tempfile.TemporaryFile(mode='w+')

    def spin_until(predicate,timeout=15.):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            if predicate():
                return
            if process and process.poll() is not None:
                raise AssertionError('Simulator exited before interface check completed')
            rclpy.spin_once(node,timeout_sec=.02)
        raise AssertionError(f'Timed out. Received topics: {sorted(latest)}')

    def wait(seconds):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            rclpy.spin_once(node,timeout_sec=.02)

    def service(kind,path,request):
        client = node.create_client(kind,path)
        assert client.wait_for_service(timeout_sec=3.)
        future = client.call_async(request)
        spin_until(future.done,5.)
        result = future.result()
        node.destroy_client(client)
        return result

    try:
        process = subprocess.Popen([sys.executable,'-m','limo_parking_sim.app','--headless'],stdout=log,stderr=log)
        spin_until(lambda:len(latest)==len(specs) and len(tf)>=12)
        assert latest['compressed'].header.frame_id == 'camera_color_optical_frame'
        jpeg = np.array(PILImage.open(io.BytesIO(bytes(latest['compressed'].data))))
        assert jpeg.shape == (480,640,3)
        assert latest['color'].encoding == 'rgb8' and len(latest['color'].data) == 640*480*3
        depth = latest['depth']
        assert depth.encoding == '16UC1' and len(depth.data) == 640*400*2
        assert depth.header.frame_id == 'camera_depth_optical_frame'
        mm = np.frombuffer(depth.data,dtype='<u2')
        assert np.count_nonzero(mm)>100 and mm.max() <= 3000
        for key in ('color','depth'):
            info = latest[key+'_info']
            assert info.width == cfg['camera'][key]['width']
            assert info.height == cfg['camera'][key]['height']
            assert info.k[0]>0 and info.k[4]>0 and info.distortion_model == 'plumb_bob'
        scan = latest['scan']
        initial_ranges = np.array(scan.ranges)
        assert scan.header.frame_id == 'laser_link' and len(scan.ranges) == 245
        assert math.isclose(scan.angle_min,math.radians(-110),abs_tol=1e-6)
        assert math.isclose(scan.angle_max,math.radians(110),abs_tol=1e-6)
        assert math.isclose(scan.angle_min+(len(scan.ranges)-1)*scan.angle_increment,scan.angle_max,abs_tol=1e-6)
        assert math.isclose(scan.scan_time,.1,abs_tol=1e-6) and scan.time_increment == 0
        assert all(x == 0 or .02 <= x <= 12 for x in scan.ranges)
        assert latest['imu'].header.frame_id == 'imu_link'
        assert latest['odom'].header.frame_id == 'odom' and latest['odom'].child_frame_id == 'base_link'
        assert tf['laser_link'].header.frame_id == 'base_link'
        assert math.isclose(tf['laser_link'].transform.translation.z,-.034)
        assert tf['camera_mount'].header.frame_id == 'base_link'
        assert tf['camera_color_optical_frame'].header.frame_id == 'camera_color_frame'
        assert tf['base_link'].header.frame_id == 'odom' and tf['odom'].header.frame_id == 'map'
        if command:
            spin_until(lambda:command.get_subscription_count()>0)
        initial = latest['ground_truth'].pose.pose.position.x
        if options.cli_command:
            wrapper = Path(__file__).resolve().parents[1]/'ros2.sh'
            cli_process = subprocess.Popen([str(wrapper),'topic','pub','-r','20',
                cfg['topics']['cmd_vel'],'geometry_msgs/msg/Twist',
                '{linear: {x: 0.2}, angular: {z: 0.12}}'],stdout=log,stderr=log)
            spin_until(lambda:latest['ground_truth'].pose.pose.position.x>initial+.18,15.)
            cli_process.terminate()
            cli_process.wait(timeout=5.)
        else:
            end,next_send = time.monotonic()+1.5,0.
            while time.monotonic()<end:
                now = time.monotonic()
                if now>=next_send:
                    msg = Twist()
                    msg.linear.x,msg.angular.z = .2,.12
                    command.publish(msg)
                    next_send = now+.05
                rclpy.spin_once(node,timeout_sec=.01)
        spin_until(lambda:latest['ground_truth'].pose.pose.position.x>initial+.08,5.)
        spin_until(lambda:json.loads(latest['status'].data)['command_stale']
                   and abs(latest['ground_truth'].twist.twist.linear.x)<1e-9,5.)
        pose = latest['ground_truth'].pose.pose
        assert pose.orientation.z > .01
        # Camera, scan and odometry run at different rates/QoS. Compare actual
        # post-motion sensor frames, not an earlier frame still in the queue.
        motion_stamp = latest['ground_truth'].header.stamp
        motion_ns = motion_stamp.sec*1_000_000_000+motion_stamp.nanosec
        def after_motion(key):
            s = latest[key].header.stamp
            return s.sec*1_000_000_000+s.nanosec>=motion_ns
        spin_until(lambda:after_motion('compressed') and after_motion('scan'),5.)
        moved_rgb = np.array(PILImage.open(io.BytesIO(bytes(latest['compressed'].data))))
        assert np.mean(np.abs(moved_rgb.astype(float)-jpeg)) > 1
        assert np.max(np.abs(np.array(latest['scan'].ranges)-initial_ranges)) > .01
        pause = SetBool.Request(data=True)
        assert service(SetBool,'/sim/pause',pause).success
        wait(.3)
        stopped_clock = latest['clock'].clock
        wait(.3)
        assert latest['clock'].clock == stopped_clock
        assert service(SetBool,'/sim/pause',SetBool.Request(data=False)).success
        service(Empty,'/sim/reset',Empty.Request())
        spin_until(lambda:abs(latest['ground_truth'].pose.pose.position.x-cfg['simulation']['spawn'][0])<1e-6,5.)
        state = json.loads(latest['status'].data)
        assert 'score' not in state
        result = {'result':'PASS','ros_domain_id':node.context.get_domain_id(),
                  'command_source':'simul/ros2.sh topic pub -r 20' if options.cli_command else 'rclpy publisher',
                  'forward_distance_m':float(pose.position.x-initial),
                  'yaw_radians':float(2*math.atan2(pose.orientation.z,pose.orientation.w)),
                  'stopped_after_publisher_exit':True,
                  'topics':sorted(cfg['topics'][key] for key in latest),
                  'received_messages':counts,'tf_children':sorted(tf),
                  'checks':['JPEG decode','raw RGB and 16UC1 depth','CameraInfo','LaserScan angle/range/frame',
                            'IMU/odometry','static/dynamic TF','Twist motion/turn and changing camera pixels',
                            'command watchdog','pause/reset services']}
        print(json.dumps(result,indent=2))
    except Exception:
        log.seek(0)
        print(log.read(),file=sys.stderr)
        raise
    finally:
        if cli_process and cli_process.poll() is None:
            cli_process.terminate()
            try:
                cli_process.wait(timeout=5.)
            except subprocess.TimeoutExpired:
                cli_process.kill()
                cli_process.wait()
        if process and process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=5.)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        log.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
