"""One reset transaction per accepted sign/floor event; retry while stopped."""
import json
import math
import time

from robot_localization.srv import SetPose
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool, Header, String
from std_srvs.srv import Trigger
from .localization import OdomSample
from .core import advance
from .tracking import compose, inverse


class StartSequence:
    def setup_start_sequence(self):
        self.start_phase = 'IDLE'
        self.start_future = None
        self.start_deadline = self.start_retry_at = 0.
        self.start_retries = 0
        self.start_generation = None
        self.start_health_stamp = None
        self.start_stationary = 0
        self.start_event_stamp = self.pose_reset_stamp = 0.
        self.ready_until = 0.
        self.start_status_wall = 0.
        self.start_wait_log_wall = 0.
        self.wheel_sample = None
        self.departure_imu = None
        self.departure_wheel = False
        self.yellow_segments = []
        self.yellow_pending = None
        self.start_pub = self.create_publisher(Bool, '/parking_start', 10)
        self.ready_pub = self.create_publisher(Header, '/parking/ready', 10)
        self.pose_reset_client = self.create_client(SetPose, '/set_pose')
        from nav_msgs.msg import Odometry, Path
        self.create_subscription(Odometry, '/odom', self.on_measured_wheel, qos_profile_sensor_data)
        from sensor_msgs.msg import Imu
        self.create_subscription(Imu, '/localization/imu', self.on_departure_imu, qos_profile_sensor_data)
        self.create_subscription(Path, '/lane_fusion/yellow_boundary', self.on_yellow_boundary, 1)

    def accepts_parking_start(self):
        if not self.prevent_active_retrigger or not self.activation.active or self.follower.exited:
            return True
        if self.start_phase != 'IDLE':
            return False
        if self.manual_interrupted or self.motion_failed or self.follower.exit_retry_exhausted:
            return True
        exhausted=(self.search_started is not None and (self.search_distance>=self.search_max_distance
                   or self.now_seconds()-self.search_started>=self.search_timeout))
        return bool(exhausted and not self.follower.path)

    def request_parking_start(self):
        if not self.accepts_parking_start():
            return
        if time.monotonic() < self.sign_ignore_until:
            return
        if self.start_future is not None:
            self.start_future.cancel()
        self.start_future = None
        self.manual_interrupted = self.motion_failed = False
        self.motion_stall_since = None
        self.approach_exhausted = False
        self.guard_reset = False
        self.ready_until = 0.
        self.departure_wheel = False
        self.yellow_segments = []
        self.yellow_pending = None
        was_active = self.activation.active
        now = self.now_seconds()
        self.activation.restart(now)
        self.begin_parking(was_active, now)
        self.odometry.samples.clear()
        self.odometry.reason = ''
        self.odometry.discontinuity = False
        self.fusion_health = self.fusion_health_received = None
        self.localization_not_before = self.scan_not_before = now
        self.start_event_stamp = now
        self.start_phase = 'STOP'
        self.start_wait_log_wall = 0.
        self.start_retries = self.start_stationary = 0
        self.start_health_stamp = None
        self.start_deadline = time.monotonic()+10.
        self.publish_command(0., 0.)
        self.start_pub.publish(Bool(data=True))
        self.get_logger().debug('주차 시작: 정지·EKF 초기화')

    def retry_start(self, reason):
        if self.start_future is not None:
            self.start_future.cancel()
        self.start_future = None
        self.start_phase = 'RETRY'
        self.start_retry_at = time.monotonic()+1.
        self.start_retries += 1
        self.start_stationary = 0
        self.start_health_stamp = None
        self.get_logger().warning(f'EKF 초기화 재시도 {self.start_retries}회: {reason}')

    def tick_start_sequence(self):
        wall, now = time.monotonic(), self.now_seconds()
        if self.start_phase == 'IDLE':
            if wall < self.ready_until:
                self.ready_pub.publish(Header(stamp=self.get_clock().now().to_msg(), frame_id=str(self.attempt)))
            return False
        self.publish_command(0., 0.)
        self.state, self.reason = 'WAITING_CALIBRATION', '정지·EKF 초기화 중'
        if not self.vehicle_automatic(): self.reason = self.vehicle_wait_reason()
        self.publish_guard()
        if wall-self.start_status_wall >= .2:
            self.report_operation()
            self.status_pub.publish(String(data=json.dumps(dict(self.operation,active=True,
                attempt=self.attempt,start_phase=self.start_phase))))
            self.start_status_wall = wall
        if not self.vehicle_automatic():
            # Waiting for vehicle status is not an EKF initialization failure.
            if self.start_phase != 'WAIT_AUTO':
                if self.start_future is not None: self.start_future.cancel()
                self.start_future = None
                self.start_phase = 'WAIT_AUTO'
                self.start_stationary = 0
                self.start_health_stamp = None
            if wall-self.start_wait_log_wall >= 2.:
                self.get_logger().warning('주차 대기: '+self.vehicle_wait_reason())
                self.start_wait_log_wall = wall
            return True
        if self.start_phase == 'WAIT_AUTO':
            self.start_phase = 'STOP'
            self.start_deadline = wall+10.
            self.get_logger().debug('자동 모드 확인: 정지·초기화 재개')
        if self.start_phase == 'RETRY':
            if wall < self.start_retry_at: return True
            self.start_phase = 'STOP'
            self.start_deadline = wall+10.
        if wall >= self.start_deadline:
            self.retry_start('응답·센서 시간 초과')
            return True
        wheel = self.wheel_sample
        stationary = (wheel is not None and -.05 <= now-wheel[0] <= .3
                      and wall-wheel[3] <= .3 and abs(wheel[1]) < .01 and abs(wheel[2]) < .02)
        if self.start_phase == 'STOP':
            if not stationary:
                self.start_stationary = 0
                return True
            if wheel[0] != self.start_health_stamp:
                self.start_stationary += 1
                self.start_health_stamp = wheel[0]
            if self.start_stationary < 3: return True
            if not self.parking_calibration_stop:
                self.start_phase = 'SETPOSE'
            elif self.approach_calibration_client.service_is_ready():
                self.start_future = self.approach_calibration_client.call_async(Trigger.Request())
                self.start_phase = 'GYRO'
                self.start_deadline = wall+10.
            return True
        if not stationary:
            self.retry_start('정지 상태 재확인')
            return True
        if self.start_phase == 'GYRO' and self.start_future.done():
            try:
                result = self.start_future.result()
                if not result.success: raise ValueError()
                generation = json.loads(result.message)['generation']
                if not isinstance(generation, int) or isinstance(generation, bool): raise ValueError()
                self.start_generation = generation
                self.start_phase = 'GYRO_WAIT'
                self.start_future = None
            except Exception:
                self.retry_start('자이로 보정 실패')
            return True
        if self.start_phase == 'GYRO_WAIT':
            health = self.fusion_health
            if (health and health.get('calibration_generation') == self.start_generation
                    and health.get('ready') is True and not health.get('calibrating', True)
                    and self.fusion_health_received is not None
                    and wall-self.fusion_health_received <= self.localization_timeout
                    and 0 <= now-health['stamp'] <= self.localization_timeout):
                self.start_phase = 'SETPOSE'
            return True
        if self.start_phase == 'SETPOSE' and self.pose_reset_client.service_is_ready():
            request = SetPose.Request()
            request.pose.header.stamp = self.get_clock().now().to_msg()
            request.pose.header.frame_id = self.odom_frame
            request.pose.pose.pose.orientation.w = 1.
            for i in (0,7,14,21,28,35): request.pose.pose.covariance[i] = 1e-4
            self.pose_reset_stamp = now
            self.localization_not_before = self.scan_not_before = now
            self.odometry.samples.clear()
            self.odometry.reason = ''
            self.odometry.discontinuity = False
            self.tracker = self.new_tracker()
            self.epoch += 1
            if self.scan_job is not None:
                self.scan_job[0].cancel()
                self.scan_job = None
            self.latest_scan_job = None
            self.scan = self.scan_time = None
            self.start_future = self.pose_reset_client.call_async(request)
            self.start_phase = 'POSE_RESPONSE'
            self.start_deadline = wall+10.
            return True
        if self.start_phase == 'POSE_RESPONSE' and self.start_future.done():
            try:
                self.start_future.result()
                self.start_future = None
                # A SetPose response defines a NEW coordinate epoch. No scan,
                # object track or async result from before it can enter this map.
                if self.scan_job is not None:
                    self.scan_job[0].cancel()
                    self.scan_job = None
                self.reset_motion()
                self.odometry.samples.clear()
                self.odometry.reason = ''
                self.odometry.discontinuity = False
                cutoff = self.now_seconds()
                self.pose_reset_stamp = cutoff
                self.localization_not_before = self.scan_not_before = cutoff
                self.approach_reference.reset(cutoff)
                self.last_localization_rejection = None
                self.start_phase = 'POSE_WAIT'
                self.start_deadline = wall+10.
                self.get_logger().debug('EKF 좌표·지도 기준 초기화 완료')
            except Exception:
                self.retry_start('EKF 초기화 요청 실패')
            return True
        if self.start_phase == 'POSE_WAIT':
            if (self.odometry.healthy(now) and self.tracker.valid
                    and self.tracker.last_stamp is not None and self.tracker.last_stamp > self.pose_reset_stamp
                    and self.scan_time is not None and 0 <= now-self.scan_time <= self.scan_timeout):
                self.start_phase = 'IDLE'
                self.localization_recovering = False
                self.localization_hold_since = None
                self.manual_interrupted = self.motion_failed = False
                # Legacy approach code must not take a second calibration/lease.
                self.approach_calibration.phase = 'READY'
                self.approach_calibration_event = None
                self.ready_until = wall+1.
                self.state, self.reason = 'PLANNING', ''
                self.ready_pub.publish(Header(stamp=self.get_clock().now().to_msg(), frame_id=str(self.attempt)))
                self.get_logger().debug('EKF 초기화 완료: 주차 진행')
            return True
        return True

    def on_departure_imu(self,msg):
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        yaw_rate=msg.angular_velocity.z
        if (msg.header.frame_id==self.base_frame and math.isfinite(yaw_rate)
                and abs(yaw_rate)<=3. and -.05<=self.now_seconds()-stamp<=.3):
            self.departure_imu=(stamp,yaw_rate,time.monotonic())

    def on_measured_wheel(self, msg):
        now = self.now_seconds()
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        v, w = msg.twist.twist.linear.x, msg.twist.twist.angular.z
        if (not all(math.isfinite(x) for x in (stamp,v,w)) or not -.05 <= now-stamp <= .3
                or abs(v)>1.5 or abs(w)>3. or msg.child_frame_id != self.base_frame): return
        if self.wheel_sample is not None and stamp <= self.wheel_sample[0]: return
        self.wheel_sample = stamp,v,w,time.monotonic()
        if not self.departure_active() or self.localization_source != 'fused': return
        self.lock_departure_localization()
        if not self.departure_wheel or not self.odometry.samples: return
        imu=self.departure_imu
        if (imu is not None and -.05<=stamp-imu[0]<=.3
                and time.monotonic()-imu[2]<=.3):
            w=imu[1]  # calibrated gyro is independent of EKF position/reset faults
        previous = self.odometry.samples[-1]
        dt = stamp-previous.stamp
        if dt <= 0: return
        if dt > self.localization_timeout:
            # Rebase time without inventing motion across an unobserved interval.
            pose = previous.pose
        else:
            speed = .5*(previous.speed+v)
            yaw_rate = .5*(previous.yaw_rate+w)
            delta = advance((0.,0.,0.),speed*dt,yaw_rate/speed) if abs(speed)>1e-8 else (0.,0.,yaw_rate*dt)
            pose = compose(previous.pose,delta)
        self.odometry.reason = ''
        self.odometry.discontinuity = False
        self.odometry.add(OdomSample(stamp,pose,v,w,previous.variance))

    def on_yellow_boundary(self, msg):
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        if (msg.header.frame_id != self.base_frame or len(msg.poses)<2
                or self.start_phase != 'IDLE' or not self.activation.active or self.follower.exited
                or not -.05 <= self.now_seconds()-stamp <= .3
                or stamp <= self.pose_reset_stamp): return
        points = [(p.pose.position.x,p.pose.position.y) for p in msg.poses]
        if not all(math.isfinite(v) for p in points for v in p): return
        sample = self.odometry.at(stamp)
        if sample is None or self.tracker.origin is None:
            self.yellow_pending = msg
            return
        self.yellow_pending = None
        pose = compose(inverse(self.tracker.origin),compose(self.tracker.offset,sample.pose))
        points = [compose(pose,(*p,0.))[:2] for p in points]
        # Resample densely spaced BEV points before making finite segments.
        compact = [points[0]]
        for point in points[1:]:
            if math.dist(compact[-1],point)>=.04: compact.append(point)
        if math.dist(compact[-1],points[-1])>=.02: compact.append(points[-1])
        points = compact
        segments = []
        for a,b in zip(points,points[1:]):
            cross = (b[0]-a[0])*(-a[1])-(b[1]-a[1])*(-a[0])
            if math.dist(a,b)<.02 or abs(cross)<1e-6: continue
            segments.append((a,b,1 if cross>=0 else -1))
        if not segments: return
        # Refresh overlapping observations, retain the boundary outside current view.
        kept = []
        for a,b,side in self.yellow_segments:
            mid = ((a[0]+b[0])/2,(a[1]+b[1])/2)
            if not any(math.dist(mid,((c[0]+d[0])/2,(c[1]+d[1])/2))<.15 for c,d,_ in segments):
                kept.append((a,b,side))
        self.yellow_segments = (kept+segments)[-1000:]
        self.planning_cache_key = None
