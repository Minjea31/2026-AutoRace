"""C++ automatic-trigger inhibition and three-second startup on private DDS."""
import json,os,subprocess,time
from pathlib import Path
from uuid import uuid4
import cv2,numpy as np,pytest,rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy
from rclpy.signals import SignalHandlerOptions
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CompressedImage,Image,LaserScan
from std_msgs.msg import Bool,String
ROOT=Path(__file__).resolve().parent.parent
@pytest.mark.parametrize('mode',['startup','status','command','mux'])
def test_actual_cpp_trigger_and_startup(mode):
    prefix='/obj_gate_'+uuid4().hex;domain=193
    ctx=Context();rclpy.init(context=ctx,domain_id=domain,signal_handler_options=SignalHandlerOptions.NO)
    probe=Node('obj_gate_probe',context=ctx);ex=SingleThreadedExecutor(context=ctx);ex.add_node(probe)
    env={**os.environ,'ROS_DOMAIN_ID':str(domain),'ROS_LOCALHOST_ONLY':'1'}
    args=[str(ROOT/'objavoid_ws/install/autorace_objavoid_v3_cpp/lib/autorace_objavoid_v3_cpp/objavoid_v3_node'),'--ros-args','-r','__node:=obj_gate_cpp']
    values=dict(use_sim_time='false',auto_activate='false',marker_rate='0.0',start_entrance='true',start_rearm='0.0',lane_rate='20.0',
        lane_config=str(ROOT/'lane_fusion_ws/src/autorace_lane_deep_lite/config/real/mission_vision.yaml'),scan_topic=prefix+'/scan',odom_topic=prefix+'/odom',
        camera_topic=prefix+'/image',lane_mask_topic=prefix+'/mask',slow_active_topic=prefix+'/slow_active',slow_command_topic=prefix+'/slow_cmd',command_topic=prefix+'/cmd')
    if mode=='startup':values.update(fork_enable='false',road_outline='true',lane_align='false')
    for key,value in values.items():args+=['-p',key+':='+value]
    args+=['-r','/objavoid_v3/state:='+prefix+'/state','-r','/objavoid_v3/activate:='+prefix+'/activate','-r','/mux/active_source:='+prefix+'/mux']
    states=[];commands=[]
    subs=[probe.create_subscription(String,prefix+'/state',lambda m:states.append(json.loads(m.data)),10),probe.create_subscription(AckermannDriveStamped,prefix+'/cmd',lambda m:commands.append((time.monotonic(),m)),10)]
    od=probe.create_publisher(Odometry,prefix+'/odom',10);scan_pub=probe.create_publisher(LaserScan,prefix+'/scan',qos_profile_sensor_data)
    cam=probe.create_publisher(CompressedImage,prefix+'/image',qos_profile_sensor_data);mask_pub=probe.create_publisher(Image,prefix+'/mask',qos_profile_sensor_data)
    active=probe.create_publisher(Bool,prefix+'/slow_active',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    slow=probe.create_publisher(AckermannDriveStamped,prefix+'/slow_cmd',10)
    mux=probe.create_publisher(String,prefix+'/mux',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    activate=probe.create_publisher(Bool,prefix+'/activate',10)
    path=ROOT/'objavoid_ws/build/autorace_objavoid_v3_cpp'/('trigger_'+mode+'.log');log=path.open('w')
    process=subprocess.Popen(args,env=env,stdout=log,stderr=subprocess.STDOUT);send_slow=[False];stream_camera=[True];last_camera=[0.]
    def pump(duration):
        end=time.monotonic()+duration;next_sensor=0
        while time.monotonic()<end:
            assert process.poll() is None,path.read_text()[-1200:]
            if time.monotonic()>=next_sensor:
                stamp=probe.get_clock().now().to_msg();o=Odometry();o.header.stamp=stamp;o.header.frame_id='odom';o.child_frame_id='base_link';o.pose.pose.orientation.w=1.
                s=LaserScan();s.header.stamp=stamp;s.header.frame_id='laser';s.angle_min=-3.14;s.angle_increment=6.28/360;s.range_min=.02;s.range_max=10.;s.ranges=[8.]*360
                od.publish(o);scan_pub.publish(s)
                if stream_camera[0] and time.monotonic()-last_camera[0]>=.1:
                    image();last_camera[0]=time.monotonic()
                if send_slow[0]:m=AckermannDriveStamped();m.header.stamp=stamp;slow.publish(m)
                next_sensor=time.monotonic()+.04
            ex.spin_once(timeout_sec=.005)
    def until(check,timeout=4.):
        end=time.monotonic()+timeout
        while not check():
            assert time.monotonic()<end,(states[-1]['start_gate'] if states else path.read_text()[-1200:]);pump(.02)
    def image():
        stamp=probe.get_clock().now().to_msg();ok,jpeg=cv2.imencode('.jpg',np.zeros((480,640,3),np.uint8));assert ok
        m=CompressedImage(format='jpeg',data=jpeg.tobytes());m.header.stamp=stamp
        mask=Image(height=480,width=640,encoding='mono8',step=640,data=bytes(480*640));mask.header.stamp=stamp
        cam.publish(m);mask_pub.publish(mask)
    try:
        until(lambda:all(p.get_subscription_count()>0 for p in (od,scan_pub,cam,mask_pub,active,slow,mux,activate)),8.)
        active.publish(Bool(data=mode=='status'));mux.publish(String(data='slow' if mode=='mux' else 'lane'));send_slow[0]=mode=='command'
        image();pump(.3)
        if mode=='startup':
            until(lambda:states and states[-1]['start_gate']['last'] is not None)
            # No crosswalk input exists: detection is already enabled.
            trigger=time.monotonic();activate.publish(Bool(data=True))
            until(lambda:commands and states[-1].get('startup',{}).get('waiting'))
            pump(max(0.,trigger+2.8-time.monotonic()));assert all(m.drive.speed==0 for _,m in commands)
            assert states[-1]['replans']>0
            until(lambda:any(m.drive.speed>0 for _,m in commands),3.)
            assert next(at for at,m in commands if m.drive.speed>0)-trigger>=2.99
            return
        until(lambda:states and states[-1]['start_gate']['status']=='wait_slow_bump')
        assert states[-1]['start_gate']['last'] is None
        stream_camera[0]=False
        active.publish(Bool(data=False));mux.publish(String(data='lane'));send_slow[0]=False
        until(lambda:states[-1]['start_gate']['status']=='wait_post_slow_image')
        assert states[-1]['start_gate']['last'] is None
        stream_camera[0]=True;image();until(lambda:states[-1]['start_gate']['last'] is not None)
        assert states[-1]['state']=='IDLE'
    finally:
        process.terminate()
        try:process.wait(timeout=5.)
        except subprocess.TimeoutExpired:process.kill();process.wait(timeout=5.)
        log.close();ex.shutdown();probe.destroy_node();ctx.shutdown()
