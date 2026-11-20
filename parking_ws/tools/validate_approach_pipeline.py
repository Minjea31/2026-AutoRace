#!/usr/bin/env python3
"""Private-domain EKF/conditioning/parking/lane test; no hardware drivers."""
import argparse,json,math,os,signal,subprocess,sys,time
from pathlib import Path

p=argparse.ArgumentParser();p.add_argument('--ekf-binary',type=Path,required=True);p.add_argument('--lane-binary',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--failure',action='store_true');a=p.parse_args()
root=Path(__file__).resolve().parents[2]
if os.environ.get('ROS_LOCALHOST_ONLY')!='1' or not 160<=int(os.environ.get('ROS_DOMAIN_ID','0'))<200:raise RuntimeError('Use a private local ROS domain 160..199')
sys.path[:0]=[str(root/'parking_ws/src/autorace_parking'),str(root/'sensor_ws/src/autorace_sensor_bringup/scripts'),str(root/'slow_ws/src/autorace_slow'),str(root/'mux_ws/src/autorace_mux')]
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool,Header,Float32MultiArray
from std_srvs.srv import Trigger
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_fusion import SensorFusion
from autorace_parking.node import ParkingNode
from autorace_mux.node import MuxNode
from geometry_msgs.msg import Twist

rclpy.init();probe=Node('approach_validation_probe');fusion=SensorFusion(parameter_overrides=[Parameter('imu_frame',value='base_link')])
parking=ParkingNode(parameter_overrides=[Parameter('parking_calibration_stop',value=True),Parameter('parking_calibration_timeout',value=3. if a.failure else 10.),Parameter('white_roi_enabled',value=True),Parameter('lane_white_config',value=str(root/'lane_fusion_ws/src/autorace_lane_fusion/config/real/lane.yaml'))])
mux=MuxNode(parameter_overrides=[Parameter('output_topic',value='/approach_validation/cmd_vel')])
ex=SingleThreadedExecutor()
for n in (probe,fusion,parking,mux):ex.add_node(n)
children=[];commands=[];events=[];releases=[];trace=[];outputs=[]
start=time.monotonic();phase='';failed_at=ready_at=retry_at=None;reset_future=None;position=0.;last_publish=last_sign=0.;signs=0
pub_odom=probe.create_publisher(Odometry,'/odom',1);pub_imu=probe.create_publisher(Imu,'/imu',1);pub_target=probe.create_publisher(Float32MultiArray,'/lane_fusion/target',1);pub_sign=probe.create_publisher(Bool,'/yolo/parking_sign',1);pub_oldready=probe.create_publisher(Header,'/parking/approach_ready',1);pub_other=probe.create_publisher(AckermannDriveStamped,'/mission/slow/command_04',1)
subscriptions=[probe.create_subscription(AckermannDriveStamped,'/lane_fusion_pid/command',lambda m:commands.append((time.monotonic()-start,m.drive.speed)),10),probe.create_subscription(Header,'/parking/approach_start',events.append,10),probe.create_subscription(Header,'/parking/approach_ready',releases.append,10)]
subscriptions.append(probe.create_subscription(Twist,'/approach_validation/cmd_vel',lambda m:outputs.append((time.monotonic()-start,m.linear.x)),10))
reset=probe.create_client(Trigger,'/parking/reset')
log=a.output.with_suffix('.process.log');log.parent.mkdir(parents=True,exist_ok=True);stream=log.open('w')
report={}
try:
 children.append(subprocess.Popen([str(a.ekf_binary),'--ros-args','--params-file',str(root/'sensor_ws/src/autorace_sensor_bringup/config/limo_ekf.yaml'),'-r','__node:=ekf_filter_node_odom'],stdout=stream,stderr=subprocess.STDOUT,start_new_session=True))
 children.append(subprocess.Popen([str(a.lane_binary),'--ros-args','-p','parking_calibration_stop:=true','-p','speed:=0.6','-p','enable_vehicle_output:=true'],stdout=stream,stderr=subprocess.STDOUT,start_new_session=True))
 while time.monotonic()-start<18.:
  wall=time.monotonic();t=wall-start
  assert all(c.poll() is None for c in children),'pipeline process exited'
  job=parking.approach_calibration
  if wall-last_publish>=.01:
   dt=wall-last_publish if last_publish else .01;last_publish=wall
   moving_until=4.5 if a.failure else 1.2
   speed=.3 if t<moving_until else 0.
   position+=speed*min(.03,dt)
   od=Odometry();od.header.stamp=probe.get_clock().now().to_msg();od.header.frame_id='odom';od.child_frame_id='base_link';od.pose.pose.orientation.w=1.;od.pose.pose.position.x=position;od.twist.twist.linear.x=speed;pub_odom.publish(od)
   imu=Imu();imu.header=od.header;imu.header.frame_id='base_link';imu.orientation.w=1.;imu.angular_velocity.z=.015;pub_imu.publish(imu)
   pub_target.publish(Float32MultiArray(data=[1.,0.,0.,.5,0.,1.]))
   if job.pending or job.phase=='FAILED':
    other=AckermannDriveStamped();other.header.stamp=probe.get_clock().now().to_msg();other.drive.speed=.4;pub_other.publish(other)
  if t>.8 and wall-last_sign>.12:
   last_sign=wall;pub_sign.publish(Bool(data=True));signs+=1
  if job.phase!=phase:
   trace.append({'t':t,'phase':job.phase,'generation':fusion.calibration_generation,'bias':fusion.calibration.bias});phase=job.phase
   if phase=='FAILED':failed_at=t
   if phase=='READY':ready_at=t
  if events and signs==3:
   old=Header();old.stamp.sec=1;pub_oldready.publish(old)
  if a.failure and failed_at is not None and retry_at is None and t>7.2:
   # Bias can finish late, but a failed attempt must still hold indefinitely.
   assert job.phase=='FAILED' and fusion.calibration_generation==1
   assert not any(v>.0001 for ct,v in commands if ct>failed_at+.1)
   assert not any(v>.0001 for ct,v in outputs if failed_at+.1<ct<t)
   assert not any(m.stamp==events[-1].stamp for m in releases)
   assert reset.service_is_ready();reset_future=reset.call_async(Trigger.Request());retry_at=t
  if ready_at is not None and t>ready_at+1.1:
   assert len(events)==1,'repeated signs restarted the slowdown/calibration'
   assert fusion.calibration_generation==(2 if a.failure else 1)
   assert any(.05<v<=.30001 for ct,v in commands if ct>ready_at)
   hold_start=.95 if not a.failure else retry_at
   first_release=ready_at
   assert max((abs(v) for ct,v in commands if hold_start+.15<ct<first_release-.05),default=0.)<1e-6
   assert max((abs(v) for ct,v in outputs if hold_start+.15<ct<first_release-.05),default=0.)<1e-6
   assert any(.05<v<=.30001 for ct,v in outputs if ct>ready_at+.4)
   report={'passed':True,'failure_then_manual_reset':a.failure,'calibration_requests':fusion.calibration_generation,'approach_events':len(events),'failed_at_s':failed_at,'explicit_reset_at_s':retry_at,'ready_at_s':ready_at,'trace':trace,'commands':commands,'mux_outputs':outputs,'stationary_bias_rad_s':fusion.calibration.bias,'ekf_samples':len(parking.odometry.samples)}
   break
  ex.spin_once(timeout_sec=.002)
 assert report,'pipeline did not reach a verified release'
 a.output.write_text(json.dumps(report,indent=2));print(json.dumps({k:v for k,v in report.items() if k!='commands'}),flush=True)
finally:
 for c in reversed(children):
  if c.poll() is None:os.killpg(c.pid,signal.SIGINT)
 for c in children:
  try:c.wait(timeout=5)
  except subprocess.TimeoutExpired:os.killpg(c.pid,signal.SIGKILL);c.wait()
 stream.close();ex.shutdown()
 for n in (parking,fusion,probe,mux):n.destroy_node()
 rclpy.shutdown()
