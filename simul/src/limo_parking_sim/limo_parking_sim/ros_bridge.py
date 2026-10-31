"""ROS 2 hardware interface; topic defaults match docs and sensor_ws."""
import io
from array import array
import json
import math
import os

from PIL import Image as PILImage
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, Imu
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Empty, SetBool
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

from .model import intrinsics, wrap


def quaternion(roll=0., pitch=0., yaw=0.):
    cr,sr,cp,sp,cy,sy = (math.cos(roll/2),math.sin(roll/2),math.cos(pitch/2),
                        math.sin(pitch/2),math.cos(yaw/2),math.sin(yaw/2))
    return (sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy,
            cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy)


def set_q(target, values):
    target.x,target.y,target.z,target.w = map(float,values)


def stamp(seconds):
    ns = round(seconds*1_000_000_000)
    return Time(sec=ns//1_000_000_000,nanosec=ns%1_000_000_000)


class RosBridge(Node):
    def __init__(self,cfg,model,scene):
        super().__init__('limo_parking_sim')
        self.cfg,self.model,self.scene = cfg,model,scene
        self.scene.ros_domain = self.context.get_domain_id()
        t = cfg['topics']
        # limo_driver.cpp: /cmd_vel depth 10, /odom 50, /imu 10.
        command_qos = QoSProfile(depth=10,reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(Twist,t['cmd_vel'],self._command,command_qos)
        # astra_stereo_u3: "default" -> rmw_qos_profile_default (depth 10).
        self.color_pub = self.create_publisher(Image,t['color'],10)
        self.jpeg_pub = self.create_publisher(CompressedImage,t['compressed'],10)
        self.depth_pub = self.create_publisher(Image,t['depth'],10)
        self.info_pub = {s:self.create_publisher(CameraInfo,t[s+'_info'],10) for s in ('color','depth')}
        self.scan_pub = self.create_publisher(LaserScan,t['scan'],qos_profile_sensor_data)
        self.imu_pub = self.create_publisher(Imu,t['imu'],10)
        self.odom_pub = self.create_publisher(Odometry,t['odom'],50)
        self.truth_pub = self.create_publisher(Odometry,t['ground_truth'],10)
        self.status_pub = self.create_publisher(String,t['status'],1)
        self.clock_pub = self.create_publisher(Clock,t['clock'],1)
        self.tf = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        self.create_service(Empty,'/sim/reset',self._reset)
        self.create_service(SetBool,'/sim/pause',self._pause)
        self._static_transforms()
        self.get_logger().info(
            f"Panda3D simulator ready: ROS_DOMAIN_ID={self.scene.ros_domain}, "
            f"ROS_LOCALHOST_ONLY={os.environ.get('ROS_LOCALHOST_ONLY','0')}; "
            f"sensors {t['compressed']}, {t['scan']}; control {t['cmd_vel']}")

    def _command(self,msg):
        if self.scene.keyboard_enabled:
            return
        if not self.model.command(msg.linear.x,msg.angular.z):
            self.get_logger().warning('Rejected non-finite /cmd_vel and stopped')

    def _reset(self,request,response):
        self.scene.reset()
        return response

    def _pause(self,request,response):
        self.scene.paused = request.data
        response.success = True
        response.message = 'paused' if request.data else 'running'
        return response

    def transform(self,parent,child,xyz,rpy=(0,0,0)):
        t = TransformStamped()
        t.header.stamp = stamp(self.model.time)
        t.header.frame_id,t.child_frame_id = parent,child
        t.transform.translation.x,t.transform.translation.y,t.transform.translation.z = map(float,xyz)
        set_q(t.transform.rotation,quaternion(*rpy))
        return t

    def _static_transforms(self):
        cfg = self.cfg
        f,v,c = cfg['frames'],cfg['vehicle'],cfg['camera']
        x,y,yaw = cfg['simulation']['spawn']
        specifications = [
            ('map','odom',(x,y,v['base_link_height']),(0,0,yaw)),
            ('base','footprint',(0,0,-v['base_link_height']),(0,0,0)),
            ('base','laser',cfg['lidar']['xyz'],(0,0,cfg['lidar']['yaw'])),
            ('base','imu',cfg['imu']['xyz'],(0,0,0)),
            ('base','camera_mount',c['mount_xyz'],(0,0,0)),
            ('camera_mount','camera_rotate',(0,0,0),(0,c['pitch'],0)),
            ('camera_rotate','camera_link',c['link_xyz'],(0,0,0)),
        ]
        for s in ('color','depth'):
            specifications += [('camera_link',s,c[s]['xyz'],c[s]['rpy']),
                               (s,s+'_optical',(0,0,0),(-math.pi/2,0,-math.pi/2))]
        self.static_tf.sendTransform([self.transform(f[p],f[ch],xyz,rpy) for p,ch,xyz,rpy in specifications])

    def publish_clock(self):
        self.clock_pub.publish(Clock(clock=stamp(self.model.time)))

    def _header(self,msg,frame):
        msg.header.stamp = stamp(self.model.time)
        msg.header.frame_id = self.cfg['frames'][frame]

    def _camera_info(self,stream):
        c = self.cfg['camera'][stream]
        fx,fy,cx,cy = intrinsics(c)
        msg = CameraInfo()
        self._header(msg,stream+'_optical')
        msg.width,msg.height = c['width'],c['height']
        msg.distortion_model = 'plumb_bob'
        msg.d = [0.]*5
        msg.k = [fx,0.,cx,0.,fy,cy,0.,0.,1.]
        msg.r = [1.,0.,0.,0.,1.,0.,0.,0.,1.]
        msg.p = [fx,0.,cx,0.,0.,fy,cy,0.,0.,0.,1.,0.]
        return msg

    def publish_camera(self,stream):
        c = self.cfg['camera'][stream]
        image = self.scene.color_image() if stream == 'color' else self.scene.depth_image()
        msg = Image()
        self._header(msg,stream+'_optical')
        msg.height,msg.width = c['height'],c['width']
        msg.encoding = 'rgb8' if stream == 'color' else '16UC1'
        msg.is_bigendian = 0
        msg.step = c['width']*(3 if stream == 'color' else 2)
        # array('B') avoids the generated ROS setter checking every byte in
        # Python (otherwise raw 30 Hz images become CPU-bound).
        msg.data = array('B',image.tobytes())
        (self.color_pub if stream == 'color' else self.depth_pub).publish(msg)
        self.info_pub[stream].publish(self._camera_info(stream))
        if stream == 'color':
            compressed = CompressedImage()
            compressed.header = msg.header
            compressed.format = 'rgb8; jpeg compressed bgr8'
            encoded = io.BytesIO()
            PILImage.fromarray(image).save(encoded,format='JPEG',quality=self.cfg['camera']['jpeg_quality'])
            compressed.data = array('B',encoded.getvalue())
            self.jpeg_pub.publish(compressed)

    def publish_scan(self,ranges):
        cfg,m = self.cfg['lidar'],self.model
        msg = LaserScan()
        self._header(msg,'laser')
        msg.angle_min,msg.angle_max = float(m.angles[0]),float(m.angles[-1])
        msg.angle_increment = float(m.angles[1]-m.angles[0])
        msg.scan_time = 1./cfg['frequency']
        msg.time_increment = 0.  # snapshot scan; no fictitious rolling acquisition
        msg.range_min,msg.range_max = float(cfg['range_min']),float(cfg['range_max'])
        msg.ranges = ranges.tolist()
        msg.intensities = [100. if math.isfinite(x) and x>0 else 0. for x in ranges]
        self.scan_pub.publish(msg)

    def publish_telemetry(self):
        cfg,m = self.cfg,self.model
        f = cfg['frames']
        x0,y0,yaw0 = cfg['simulation']['spawn']
        dx,dy = m.x-x0,m.y-y0
        x = math.cos(yaw0)*dx+math.sin(yaw0)*dy
        y = -math.sin(yaw0)*dx+math.cos(yaw0)*dy
        yaw = wrap(m.yaw-yaw0)
        odom = Odometry()
        self._header(odom,'odom')
        odom.child_frame_id = f['base']
        odom.pose.pose.position.x,odom.pose.pose.position.y = x,y
        set_q(odom.pose.pose.orientation,quaternion(yaw=yaw))
        odom.twist.twist.linear.x = m.speed
        odom.twist.twist.angular.z = m.yaw_rate
        odom.pose.covariance = [0.]*36
        odom.twist.covariance = [0.]*36
        for i in (0,7,14,21,28,35):
            odom.pose.covariance[i] = odom.twist.covariance[i] = 1e-4
        self.odom_pub.publish(odom)
        self.tf.sendTransform(self.transform(f['odom'],f['base'],(x,y,0),(0,0,yaw)))
        truth = Odometry()
        self._header(truth,'map')
        truth.child_frame_id = f['base']
        truth.pose.pose.position.x,truth.pose.pose.position.y = m.x,m.y
        truth.pose.pose.position.z = cfg['vehicle']['base_link_height']
        set_q(truth.pose.pose.orientation,quaternion(yaw=m.yaw))
        truth.twist = odom.twist
        self.truth_pub.publish(truth)
        imu = Imu()
        self._header(imu,'imu')
        set_q(imu.orientation,quaternion(yaw=m.yaw))
        imu.angular_velocity.z = m.yaw_rate
        imu.linear_acceleration.x,imu.linear_acceleration.y,imu.linear_acceleration.z = m.imu_acceleration()
        imu.orientation_covariance = [1e-6,0.,0.,0.,1e-6,0.,0.,0.,1e-6]
        imu.angular_velocity_covariance = list(imu.orientation_covariance)
        imu.linear_acceleration_covariance = list(imu.orientation_covariance)
        self.imu_pub.publish(imu)

    def publish_status(self):
        self.status_pub.publish(String(data=json.dumps(self.model.status(),separators=(',',':'))))
