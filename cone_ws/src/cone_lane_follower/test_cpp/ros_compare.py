"""Run on an isolated DDS domain; never connects to the vehicle's command mux."""
import argparse
import atexit
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray
from ackermann_msgs.msg import AckermannDriveStamped


def run(command,config,asynchronous,domain,check_seven=False,check_exit=False):
    assert domain != 96, 'Must isolate tests from the vehicle'
    os.environ['ROS_DOMAIN_ID']=str(domain)
    rclpy.init(domain_id=domain)
    n=Node('cone_parity_probe')
    clock=n.create_publisher(Clock,'/clock',10)
    scan_pub=n.create_publisher(LaserScan,'/scan',qos_profile_sensor_data)
    tf=TransformBroadcaster(n)
    seen={}
    subs=[]
    for name,kind in [('roi_points',Marker),('boundaries',MarkerArray),('center_path',MarkerArray),('center_spot',MarkerArray),('control_points',MarkerArray),('backup_path',MarkerArray),('control_status',Marker)]:
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL) if name=='control_status' else 10
        subs.append(n.create_subscription(kind,'/cone_lane/'+name,lambda m,k=name:seen.__setitem__(k,m),qos))
    commands=[]
    subs.append(n.create_subscription(AckermannDriveStamped,'/mission/cone/command_04',commands.append,10))
    log=open('/tmp/cone_ros_'+('cpp' if 'roi_node' in command[0] else 'python')+f'_{asynchronous}.log','w')
    proc=subprocess.Popen(command+['--ros-args','--params-file',config,'-p','use_sim_time:=true','-p',f'planner_async:={str(asynchronous).lower()}','-p','control_max_steering_rate:=100.0'],stdout=log,stderr=subprocess.STDOUT)
    def cleanup():
        if proc.poll() is None:
            proc.terminate()
            try:proc.wait(timeout=5)
            except subprocess.TimeoutExpired:proc.kill();proc.wait()
    atexit.register(cleanup)
    def spin(seconds=.035):
        until=time.monotonic()+seconds
        while time.monotonic()<until:rclpy.spin_once(n,timeout_sec=.002)
        assert proc.poll() is None, Path(log.name).read_text()
    def stamp(t):
        m=Clock();m.clock.sec=int(t);m.clock.nanosec=round((t-int(t))*1e9);return m
    current=10.
    discovery_deadline=time.monotonic()+20.
    while time.monotonic()<discovery_deadline:
        clock.publish(stamp(current));spin(.02)
        if scan_pub.get_subscription_count():break
    assert scan_pub.get_subscription_count()
    def frame(kind):
        nonlocal current
        current+=.05
        c=stamp(current);clock.publish(c)
        transform=TransformStamped();transform.header.frame_id='odom';transform.child_frame_id='base_link';transform.header.stamp=c.clock;transform.transform.rotation.w=1.
        if kind not in ('empty','exit4','exit5'):tf.sendTransform(transform)
        spin(.01)
        msg=LaserScan();msg.header.stamp=c.clock;msg.header.frame_id='base_link';msg.angle_min=-math.pi;msg.angle_increment=math.pi/1800;msg.range_min=.01;msg.range_max=10.;msg.ranges=[float('inf')]*3601
        centers=[]
        if kind in ('exit4','exit5'):
            centers=[(.1,-.6),(.6,-.6),(.1,.6),(.6,.6)]
            if kind=='exit5':centers.append((-.2,.05))
        elif kind!='empty':
            xs=[.2,.45,.7,.95] if kind!='few' else [.2]
            centers=[(x,y) for y in [-.3,.3] for x in xs
                     if not (kind=='seven' and y>0 and x==.95)]
        for x,y in centers:
            index=round((math.atan2(y,x)-msg.angle_min)/msg.angle_increment)
            for offset in (-1,0,1):msg.ranges[index+offset]=math.hypot(x,y)
        if kind=='bad_tf':msg.header.frame_id='missing_sensor_frame'
        scan_pub.publish(msg);spin(.045)
    def summarize():
        def markers(k):
            return {m.ns:[[q.x,q.y,q.z] for q in m.points] or [[m.pose.position.x,m.pose.position.y,m.pose.position.z]] for m in seen[k].markers if m.action==Marker.ADD}
        return dict(status=seen['control_status'].text,points=len(seen['roi_points'].points),boundaries=markers('boundaries'),path=markers('center_path'),sections=markers('center_spot'),control=markers('control_points'),speed=commands[-1].drive.speed if commands else 0.,steering=commands[-1].drive.steering_angle if commands else 0.)
    results={}
    try:
        if check_seven:
            for _ in range(8):frame('seven')
            assert 'mission: False' in summarize()['status'], 'Seven candidates must not start mission'
        if check_exit:
            for _ in range(18):frame('normal')
            assert 'mission: True' in summarize()['status'], 'Eight candidates must start mission'
            for _ in range(48):frame('exit5')
            assert 'mission: True' in summarize()['status'], 'Five candidates must keep mission active'
            assert summarize()['speed']==0., 'No live or saved path must stop vehicle'
            for _ in range(12):frame('exit4')
            assert 'mission: True' in summarize()['status'], 'Exit must wait 1.5 seconds'
            for _ in range(36):frame('exit4')
            assert 'mission: False' in summarize()['status'], 'Four candidates without paths must end mission'
            return results
        for kind,count in [('few',8),('normal',18),('bad_tf',8),('normal',18),('empty',48)]:
            for _ in range(count):frame(kind)
            results[kind+str(len(results))]=summarize()
        assert 'mission: False' in results['few0']['status']
        assert 'mission: True' in results['normal1']['status']
        assert results['normal1']['speed']>0
        assert results['bad_tf2']['speed']==0
        assert 'mission: False' in results['empty4']['status']
    finally:
        cleanup()
        atexit.unregister(cleanup)
        n.destroy_node();rclpy.shutdown();log.close()
    return results


def compare(a,b,path=''):
    if isinstance(a,dict):
        assert a.keys()==b.keys(),(path,a.keys(),b.keys())
        for k in a:compare(a[k],b[k],path+'/'+k)
    elif isinstance(a,list):
        assert len(a)==len(b),(path,len(a),len(b))
        for i,(x,y) in enumerate(zip(a,b)):compare(x,y,path+f'/{i}')
    elif isinstance(a,str):assert a==b,(path,a,b)
    else:assert math.isclose(a,b,rel_tol=1e-6,abs_tol=1e-7),(path,a,b)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--binary',required=True);p.add_argument('--config',required=True);p.add_argument('--reference',required=True);p.add_argument('--entry-eight-only',action='store_true');args=p.parse_args()
    os.environ['PYTHONPATH']=args.reference+os.pathsep+os.environ.get('PYTHONPATH','')
    if args.entry_eight_only:
        run([args.binary],args.config,True,183,check_seven=True,check_exit=True)
        print('Native thresholds passed: entry >=8; no-path exit <=4 after 1.5s; five candidates retain mission',flush=True)
        sys.exit(0)
    for asynchronous in (False,True):
        py=run([sys.executable,'-c','from cone_lane_follower.node import main; main()'],args.config,asynchronous,181)
        cpp=run([args.binary],args.config,asynchronous,182)
        compare(py,cpp)
        print('ROS output parity passed; planner_async =',asynchronous,flush=True)
