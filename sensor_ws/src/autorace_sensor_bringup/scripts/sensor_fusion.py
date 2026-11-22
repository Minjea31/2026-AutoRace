#!/usr/bin/env python3
"""Condition measured wheel/IMU inputs for the parking EKF; never commands motion."""
from copy import deepcopy
import json
import math
import time
import signal
from threading import Event

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.clock import Clock, ClockType
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from std_srvs.srv import Trigger
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformListener, StaticTransformBroadcaster, TransformException

from fusion_math import GyroCalibration, rotate_vector


class SensorFusion(Node):
    def __init__(self, **kwargs):
        super().__init__('sensor_fusion', **kwargs)
        def param(name, default):
            return self.declare_parameter(name, default).value
        self.timeout = param('sensor_timeout', .3)
        self.base_frame = param('base_frame', 'base_link')
        self.odom_frame = param('odom_frame', 'odom')
        self.world_frame = param('world_frame', 'parking_odom')
        self.imu_frame = param('imu_frame', 'imu_link')
        self.wheel_scale = param('wheel_speed_scale', 1.)
        self.gyro_scale = param('gyro_scale', 1.)
        self.speed_std = param('wheel_speed_stddev', .03)
        self.gyro_std = param('gyro_stddev', .015)
        self.calibration = GyroCalibration(param('gyro_calibration_seconds', 2.))
        self.calibration_generation = 0
        for value in (self.timeout, self.speed_std, self.gyro_std, self.calibration.duration):
            if not math.isfinite(value) or value <= 0:
                raise ValueError('Timeout, noise and calibration duration must be positive')
        if not all(math.isfinite(v) and v != 0 for v in (self.wheel_scale, self.gyro_scale)):
            raise ValueError('Sensor scales must be finite and nonzero')
        self.wheel = None
        self.imu_stamp = self.wheel_received = self.imu_received = None
        self.wheel_stamp = None
        self.last_now = None
        self.error = ''
        self.wheel_pose = None
        self.anchor_sent = False
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.anchor_pub = StaticTransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, '/localization/wheel', 10)
        self.imu_pub = self.create_publisher(Imu, '/localization/imu', 10)
        self.health_pub = self.create_publisher(String, '/localization/health', 10)
        self.create_service(Trigger, '/localization/calibrate_gyro', self.recalibrate_gyro)
        self.create_subscription(Odometry, param('wheel_topic', '/odom'), self.on_wheel, qos_profile_sensor_data)
        self.create_subscription(Imu, param('imu_topic', '/imu'), self.on_imu, qos_profile_sensor_data)
        self.health_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(.1, self.publish_health, clock=self.health_clock)

    def now(self):
        return self.get_clock().now().nanoseconds*1e-9

    def recalibrate_gyro(self, request, response):
        # Keep the EKF and its world anchor alive. Only sensor bias is renewed;
        # moving samples cannot complete GyroCalibration.
        self.calibration = GyroCalibration(self.calibration.duration)
        self.calibration_generation += 1
        self.error = ''
        response.success = True
        response.message = json.dumps({'generation': self.calibration_generation})
        return response

    @staticmethod
    def stamp(msg):
        return msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9

    def fresh(self, stamp):
        return math.isfinite(stamp) and -.05 <= self.now()-stamp <= self.timeout

    def on_wheel(self, msg):
        stamp = self.stamp(msg)
        v, w = msg.twist.twist.linear.x*self.wheel_scale, msg.twist.twist.angular.z
        if (msg.header.frame_id != self.odom_frame or msg.child_frame_id != self.base_frame
                or not self.fresh(stamp) or not all(math.isfinite(x) for x in (v, w))
                or abs(v) > 1.5 or abs(w) > 3.):
            self.error = 'Invalid wheel frame/timestamp/velocity'
            return
        if self.wheel_stamp is not None and stamp <= self.wheel_stamp:
            return
        self.wheel_stamp, self.wheel_received = stamp, time.monotonic()
        self.wheel = (v, w)
        self.wheel_pose = deepcopy(msg.pose.pose)
        conditioned = deepcopy(msg)
        conditioned.twist.twist.linear.x = v
        conditioned.twist.twist.linear.y = 0.  # planar nonholonomic constraint
        conditioned.twist.covariance = [0.]*36
        for axis in range(6):
            conditioned.twist.covariance[axis*7] = 1e3
        conditioned.twist.covariance[0] = self.speed_std**2
        conditioned.twist.covariance[7] = .02**2
        conditioned.twist.covariance[35] = .10**2
        if self.calibration.bias is not None:
            self.odom_pub.publish(conditioned)

    def on_imu(self, msg):
        stamp = self.stamp(msg)
        velocity = msg.angular_velocity
        gyro_vector = (velocity.x, velocity.y, velocity.z)
        try:
            if self.imu_frame != self.base_frame:
                transform = self.tf_buffer.lookup_transform(self.base_frame, self.imu_frame, rclpy.time.Time())
                q = transform.transform.rotation
                gyro_vector = rotate_vector(gyro_vector, (q.x, q.y, q.z, q.w))
        except (TransformException, ValueError):
            self.error = 'Waiting for valid base_link to IMU mounting TF'
            return
        gyro = gyro_vector[2]*self.gyro_scale
        if (msg.header.frame_id != self.imu_frame or not self.fresh(stamp)
                or not math.isfinite(gyro) or abs(gyro) > 3.):
            self.error = 'Invalid IMU frame/timestamp/gyro'
            return
        if self.imu_stamp is not None and stamp <= self.imu_stamp:
            return
        self.imu_stamp, self.imu_received = stamp, time.monotonic()
        if self.wheel is None or not self.fresh(self.wheel_stamp):
            return
        was_ready = self.calibration.bias is not None
        if not self.calibration.update(stamp, gyro, *self.wheel):
            return
        if not was_ready:
            self.get_logger().info(f'Gyro calibrated at rest: bias={self.calibration.bias:.6f} rad/s')
        if not self.anchor_sent and self.wheel_pose is not None:
            pose = self.wheel_pose
            q = pose.orientation
            if (not all(math.isfinite(v) for v in (pose.position.x, pose.position.y, q.x, q.y, q.z, q.w))
                    or abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1.) > .01):
                self.error = 'Invalid initial wheel pose for parking_odom anchor'
                return
            anchor = TransformStamped()
            anchor.header.stamp = msg.header.stamp
            anchor.header.frame_id, anchor.child_frame_id = self.odom_frame, self.world_frame
            anchor.transform.translation.x, anchor.transform.translation.y = pose.position.x, pose.position.y
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            anchor.transform.rotation.z, anchor.transform.rotation.w = math.sin(yaw/2), math.cos(yaw/2)
            self.anchor_pub.sendTransform(anchor)
            self.anchor_sent = True
        conditioned = deepcopy(msg)
        conditioned.header.frame_id = self.base_frame
        corrected = gyro-self.calibration.bias
        if abs(self.wheel[0]) < .01 and abs(self.wheel[1]) < .02 and abs(corrected) < .008:
            corrected = 0.
        conditioned.angular_velocity.z = corrected
        conditioned.angular_velocity.x = float(gyro_vector[0])
        conditioned.angular_velocity.y = float(gyro_vector[1])
        conditioned.angular_velocity_covariance = [0.]*9
        conditioned.angular_velocity_covariance[0] = 1e3
        conditioned.angular_velocity_covariance[4] = 1e3
        conditioned.angular_velocity_covariance[8] = self.gyro_std**2
        # The current driver supplies overly optimistic orientation covariance;
        # use measured gyro only, not magnetic yaw or integrated acceleration.
        conditioned.orientation_covariance[0] = -1.
        conditioned.linear_acceleration_covariance[0] = -1.
        self.imu_pub.publish(conditioned)
        self.error = ''

    def publish_health(self):
        now = self.now()
        if self.last_now is not None and now < self.last_now-.01:
            self.calibration = GyroCalibration(self.calibration.duration)
            self.wheel_stamp = self.imu_stamp = self.wheel_received = self.imu_received = None
            self.wheel = None
            self.anchor_sent, self.wheel_pose = False, None
        self.last_now = now
        wall = time.monotonic()
        ready = (self.calibration.bias is not None and self.anchor_sent and self.wheel_stamp is not None and self.imu_stamp is not None
                 and self.fresh(self.wheel_stamp) and self.fresh(self.imu_stamp)
                 and self.wheel_received is not None and wall-self.wheel_received <= self.timeout
                 and self.imu_received is not None and wall-self.imu_received <= self.timeout)
        self.health_pub.publish(String(data=json.dumps({
            'stamp': now, 'ready': bool(ready), 'gyro_bias_rad_s': self.calibration.bias,
            'calibration_generation': self.calibration_generation,
            'calibrating': self.calibration.bias is None,
            'wheel_speed_m_s': self.wheel[0] if self.wheel is not None else None,
            'wheel_yaw_rate_rad_s': self.wheel[1] if self.wheel is not None else None,
            'wheel_age_s': now-self.wheel_stamp if self.wheel_stamp is not None else None,
            'imu_age_s': now-self.imu_stamp if self.imu_stamp is not None else None,
            'reason': self.error or ('' if ready else 'Keep stationary for gyro calibration; check fresh wheel/IMU inputs'),
        }, allow_nan=False)))


def main(args=None):
    stopped = Event()
    previous = {}
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = SensorFusion()
    executor = SingleThreadedExecutor(context=node.context)
    executor.add_node(node)
    try:
        for signum in (signal.SIGINT, signal.SIGTERM):
            previous[signum] = signal.signal(signum, lambda *_: stopped.set())
        while rclpy.ok() and not stopped.is_set():
            executor.spin_once(timeout_sec=.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.remove_node(node)
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for signum, handler in previous.items():
            signal.signal(signum, handler)


if __name__ == '__main__':
    main()
