"""Parking ROS adapter. Motion is emitted only through the documented mux input."""
from concurrent.futures import ProcessPoolExecutor
from collections import deque
from dataclasses import fields, replace
import json
import math
import time
import multiprocessing
import signal
import struct
from threading import Event

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from rclpy.executors import ExternalShutdownException
from rclpy.signals import SignalHandlerOptions
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_msgs.msg import Header
from std_msgs.msg import Bool, String, Float32
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from .core import Activation, Scan, Settings, Gap, CollisionChecker, cluster_points, front_clearance
from .control import Follower, build_plan_report, local_goal,needs_reverse_for_goal,minimum_recovery_path
from .regions import with_occupied_slots,first_occupied_path_collision
from .tracking import ScanTracker, compose, inverse
from .localization import FusedTracker, PoseHistory, OdomSample
from .obstacle_memory import ObstacleMemory,MemoryScan
from .diagnostics import operator_reason, reason_text, state_text

SCAN_FUTURE_TOLERANCE = .1


def planner_process_init():
    signal.signal(signal.SIGINT, signal.SIG_IGN)


def validate_settings(cfg):
    for f in fields(Settings):
        value = getattr(cfg, f.name)
        if isinstance(value, (int, float)) and not isinstance(value, bool) and not math.isfinite(value):
            raise ValueError(f'{f.name} 설정은 유한한 숫자여야 합니다.')
    for name in ('cluster_distance', 'max_range', 'vehicle_length', 'vehicle_width', 'wheelbase',
                 'max_steering', 'resolution', 'yaw_resolution', 'step', 'collision_step', 'planning_timeout',
                 'goal_tolerance', 'goal_yaw_tolerance', 'recovery_front_clearance', 'steering_gain','steering_response_scale','control_preview_time',
                 'boundary_tolerance','target_heading_step','center_stop_tolerance','pair_stop_tolerance'):
        if getattr(cfg, name) <= 0:
            raise ValueError(f'{name} 설정은 0보다 커야 합니다.')
    if cfg.recovery_min_distance<0 or (cfg.recovery_min_distance>0 and minimum_recovery_path(cfg)>cfg.recovery_reverse_distance):
        raise ValueError('최소 후진 거리와 정지 여유가 1회 최대 후진 거리 안에 들어가야 합니다.')
    if (cfg.recovery_reverse_distance <= 0 or cfg.recovery_speed <= 0
            or not isinstance(cfg.max_recovery_attempts, int) or cfg.max_recovery_attempts < 0):
        raise ValueError('복구 후진 거리·속도는 0보다 커야 하고 복구 횟수는 0 이상의 정수여야 합니다.')
    if (not 0 < cfg.max_steering < math.pi / 2 or cfg.clearance < 0
            or not 0 <= cfg.max_inner_steering < math.pi / 2
            or cfg.steering_track <= 0):
        raise ValueError('조향각·윤거 또는 여유 공간 설정이 올바르지 않습니다.')
    if not math.pi/180<=cfg.target_heading_step<=math.pi/4 or max(cfg.center_stop_tolerance,cfg.pair_stop_tolerance)>.20:
        raise ValueError('목표 방향 간격은 1~45도, 도착 허용 거리는 20cm 이하여야 합니다.')
    if (not isinstance(cfg.slot_columns,int) or not isinstance(cfg.slot_rows,int)
            or min(cfg.slot_columns,cfg.slot_rows)<1 or not 3<=cfg.slot_columns*cfg.slot_rows<=12):
        raise ValueError('주차 격자는 3~12칸이며 행·열은 양의 정수여야 합니다.')
    if (min(cfg.slot_width,cfg.slot_depth,cfg.slot_match_tolerance,cfg.slot_fit_max_error)<=0
            or min(cfg.slot_border,cfg.slot_ambiguity_margin)<0
            or cfg.slot_match_tolerance>=.49*min(cfg.slot_width+cfg.slot_border,cfg.slot_depth+cfg.slot_border)
            or cfg.slot_fit_max_error>cfg.slot_match_tolerance):
        raise ValueError('주차칸 치수·매칭 오차 설정을 확인하세요. 매칭 오차는 칸 간격의 절반보다 작아야 합니다.')
    if cfg.steering_response_scale > 2.:
        raise ValueError('실측 조향 응답 비율은 0보다 크고 2 이하여야 합니다.')
    if cfg.preferred_turn_radius < 0 or min(cfg.max_entry_path_length,cfg.path_tracking_margin,cfg.exit_grid_margin)<0:
        raise ValueError('선호 회전 반경·진입 경로 길이·추종 여유는 0 이상이어야 합니다.')
    if cfg.max_turn_arc_length <= 0 or cfg.recovery_straight_distance < 0:
        raise ValueError('회전 호 길이는 0보다 크고 직선 후진 거리는 0 이상이어야 합니다.')
    if min(cfg.blind_reverse_distance,cfg.recovery_total_distance) < 0:
        raise ValueError('미관측 후진 허용 거리는 0 이상이어야 합니다.')
    if min(cfg.obstacle_min_cluster_size, cfg.small_cluster_filter_max_distance) < 0:
        raise ValueError('작은 장애물 군집 제외 크기와 적용 거리는 0 이상이어야 합니다.')
    if min(cfg.min_gap,cfg.row_tolerance,cfg.min_cluster_size,cfg.min_turn_radius) < 0:
        raise ValueError('최소 간격·나란함 허용치·군집 크기·회전 반경은 0 이상이어야 합니다.')
    if cfg.max_cluster_size < cfg.min_cluster_size or cfg.max_cluster_size <= 0:
        raise ValueError('최대 군집 크기는 최소 군집 크기 이상이며 양수여야 합니다.')
    if cfg.max_gap < max(cfg.min_gap,cfg.vehicle_width+2*cfg.clearance):
        raise ValueError('최대 빈칸 폭은 최소 폭과 차량 폭+여유 공간 이상이어야 합니다.')
    if cfg.continuous_replanning and cfg.hold_entry_path:
        raise ValueError('연속 재계획과 진입 경로 고정을 함께 켤 수 없습니다.')
    if not 0 < cfg.goal_update_gain <= 1 or min(cfg.goal_update_step,cfg.enclosure_grid_resolution,cfg.enclosure_support_radius)<=0:
        raise ValueError('목표 보정 비율은 0~1, 보정 거리·격자·지원 반경은 양수여야 합니다.')
    if cfg.enclosure_min_objects < 3 or cfg.enclosure_confirm_objects < cfg.enclosure_min_objects:
        raise ValueError('둘러싼 객체는 최소 3개이며 확정 개수는 최소 개수 이상이어야 합니다.')
    for name in ('cluster_min_points','max_expansions','enclosure_min_objects','enclosure_confirm_objects','recovery_failure_count'):
        value = getattr(cfg,name)
        if isinstance(value,bool) or not isinstance(value,int) or value < 1:
            raise ValueError(name+' must be a positive integer')


class ParkingNode(Node):
    def __init__(self, **kwargs):
        super().__init__('parking_node', **kwargs)
        def parameter(name, default):
            return self.declare_parameter(name, default).value
        self.cfg = Settings(**{f.name: parameter(f.name, getattr(Settings(), f.name)) for f in fields(Settings)})
        validate_settings(self.cfg)
        self.base_frame = parameter('base_frame', 'base_link')
        self.lidar_frame = parameter('lidar_frame', 'laser_link')
        self.sensor_pose = (parameter('lidar_x', 0.103), parameter('lidar_y', 0.0), parameter('lidar_yaw', 0.0))
        required = parameter('recognition_frames', 3)
        self.required_detections = required
        self.sign_timeout = parameter('sign_timeout', 0.5)
        self.scan_timeout = parameter('scan_timeout', 0.5)
        self.plan_period = parameter('plan_period', 1.5)
        self.speed = parameter('parking_speed', 1.0)
        self.hold_seconds = parameter('parking_hold_seconds', 3.0)
        self.exit_heading_tolerance = parameter('exit_heading_tolerance', 0.05)
        self.deceleration = parameter('max_deceleration', 1.0)
        self.max_yaw_rate = parameter('max_yaw_rate', 1.0)
        self.lookahead = parameter('lookahead', 0.18)
        self.control_hz = parameter('control_hz', 20.0)
        self.require_parking_heading = parameter('require_parking_heading', True)
        self.require_path_heading = parameter('require_path_heading', self.require_parking_heading)
        self.cfg = replace(self.cfg, require_goal_heading=self.require_path_heading)
        self.parking_position_tolerance = parameter('parking_position_tolerance', self.cfg.goal_tolerance)
        self.max_steering_rate = parameter('max_steering_rate', 0.)
        self.min_turn_speed = parameter('min_turn_speed', 0.)
        if (self.cfg.min_turn_radius < 0 or self.max_steering_rate < 0
                or not math.isfinite(self.max_steering_rate)
                or not math.isfinite(self.parking_position_tolerance) or self.parking_position_tolerance <= 0):
            raise ValueError('조향 변화율·회전 반경 또는 도착 허용 오차 설정이 올바르지 않습니다.')
        self.localization_source = parameter('localization_source', 'auto')
        if self.localization_source == 'auto':
            self.localization_source = 'scan' if self.get_parameter('use_sim_time').value else 'fused'
        if self.localization_source not in ('scan', 'fused'):
            raise ValueError('localization_source 설정은 auto, scan, fused 중 하나여야 합니다.')
        self.localization_timeout = parameter('localization_timeout', .3)
        self.odom_frame = parameter('odom_frame', 'parking_odom' if self.localization_source == 'fused' else 'odom')
        self.odometry = PoseHistory(timeout=self.localization_timeout,
                                   max_std=parameter('localization_max_stddev', .5))
        self.fusion_health = None
        self.fusion_health_received = None
        if (required < 1 or not all(math.isfinite(v) for v in
                (self.sign_timeout, self.scan_timeout, self.plan_period, self.speed,
                 self.lookahead, self.control_hz, self.hold_seconds, self.exit_heading_tolerance, self.deceleration, self.max_yaw_rate)) or
                min(self.sign_timeout, self.scan_timeout, self.plan_period, self.speed,
                    self.lookahead, self.control_hz, self.exit_heading_tolerance, self.deceleration, self.max_yaw_rate) <= 0 or self.hold_seconds < 0):
            raise ValueError('감지 횟수·수신 제한 시간·속도·제어 주기는 0보다 커야 하며 대기 시간은 음수일 수 없습니다.')
        self.activation = Activation(required, self.sign_timeout)
        self.sign_ignore_until = 0.  # monotonic seconds; retained across reset
        self.follower = self.new_follower()
        self.tracker = self.new_tracker()
        self.scan, self.scan_header, self.scan_time = None, None, None
        self.scan_error = ''
        self.last_scan_stamp = None
        self.scan_not_before = None
        self.pending_scans = deque(maxlen=10)
        self.scan_sequence = 0
        self.controlled_sequence = -1
        self.last_now = None
        self.state = 'WAITING_SIGN'
        self.attempt = 0
        self.reason = ''
        self.epoch = 0
        self.last_plan_wall = -math.inf
        self.last_status_wall = -math.inf
        self.reported_state = 'WAITING_SIGN'
        self.reported_reason = None
        self.last_operator_log_wall = -math.inf
        self.planning_info = {}
        self.had_control_path = False
        self.selected_pair = []
        self.selected_track_ids = set()
        self.planned_goal = None
        self.tracked_goal = None
        self.slot_map = None
        self.last_recovery_stop = None
        self.path_hidden_by_slot = False
        self.last_planner_error = ''
        self.recovery_attempts = 0
        self.failed_plans = 0
        self.recovery_remaining = self.cfg.recovery_total_distance
        self.blind_reverse_remaining = self.cfg.blind_reverse_distance
        self.recovery_running = False
        self.observed_corridor = deque(maxlen=2000)
        # Hybrid A* is Python CPU work. Isolate it so its GIL cannot starve
        # timestamped sensor callbacks and the command/health watchdog.
        self.worker = ProcessPoolExecutor(max_workers=1, mp_context=multiprocessing.get_context('spawn'),
                                          initializer=planner_process_init)
        self.pending = None
        self.memory_enabled = parameter('obstacle_memory_enabled', False)
        self.memory = ObstacleMemory()
        self.memory_marker_ids = set()
        self.last_memory_wall = -math.inf
        self.memory_pub = self.create_publisher(PointCloud2, '/parking/obstacle_memory', 1)
        self.obstacles_pub = self.create_publisher(MarkerArray, '/parking/obstacles', 1)
        self.obstacle_marker_count = 0
        self.target_pub = self.create_publisher(MarkerArray, '/parking/target', 1)
        self.slot_map_pub = self.create_publisher(MarkerArray, '/parking/slot_map', 1)
        self.path_pub = self.create_publisher(Path, '/parking/path', 1)
        self.active_pub = self.create_publisher(Bool, '/parking/active', 1)
        self.status_pub = self.create_publisher(String, '/parking/status', 1)
        self.command_pub = self.create_publisher(AckermannDriveStamped,
                                               '/mission/parking/command_03', QoSProfile(depth=1))
        self.localization_pub = self.create_publisher(Odometry, '/parking/localization/odometry', 10)
        self.ldf_pub = self.create_publisher(Float32, '/parking/ldf', 1)
        self.lookahead_pub = self.create_publisher(MarkerArray, '/parking/lookahead', 1)
        if self.localization_source == 'fused':
            self.create_subscription(Odometry, parameter('localization_topic', '/odometry/filtered'),
                                     self.on_localization, qos_profile_sensor_data)
            self.create_subscription(String, '/localization/health', self.on_fusion_health, 10)
        self.create_subscription(Bool, '/yolo/parking_sign', self.on_sign, QoSProfile(depth=1))
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_service(Trigger, '/parking/reset', self.on_reset)
        self.control_timer = self.create_timer(1.0 / self.control_hz, self.tick)
        self.add_on_set_parameters_callback(self.on_tuning_parameters)
        if self.cfg.recovery_uses_memory_map:
            self.get_logger().info('후진 충돌 판정은 기억 장애물 지도만 사용합니다. 지도에 장애물 점이 없는 후방은 빈 공간으로 간주합니다.')
        source_label = '휠·관성 센서·라이다 융합' if self.localization_source == 'fused' else '라이다 스캔 정합'
        self.get_logger().info(f'주차 노드 준비 완료: 연속 시작 신호 {required}회로 시작합니다. '
                               f'위치 추정: {source_label}, 최소 회전 반경: {self.cfg.turn_radius*100:.1f}cm, '
                               f'중앙 조향 한계: {math.degrees(self.cfg.steering_limit):.1f}도. '
                               '경로: /parking/path, 제어 출력: /mission/parking/command_03')
        self.get_logger().info(f'조향 추종 비율 {self.cfg.steering_gain:.2f}, 실측 응답 비율 {self.cfg.steering_response_scale:.2f}, '
                               f'조향 변화율 제한 {self.follower.max_steering_rate:.2f}rad/s (0이면 즉시 출력).')
        if self.cfg.allow_reverse:
            recovery_condition = (f'전진 진입. 목표 도착 전 앞범퍼 {self.cfg.recovery_front_clearance*100:.0f}cm 이내 '
                                  '정면 장애물 근접 시 후진 복구: '
                                  if self.cfg.recovery_on_front_obstacle_only else '전진·후진 진입 허용. 재탐색용 후진 복구: ')
            self.get_logger().info(recovery_condition+
                                   f'한 번에 최대 {self.cfg.recovery_reverse_distance*100:.0f}cm, '
                                   f'{self.cfg.recovery_speed:.2f}m/s, 최대 {self.cfg.max_recovery_attempts}회. '
                                   +('후방은 기억 장애물 지도에서 검사합니다.' if self.cfg.recovery_uses_memory_map else
                                     '후방 공간 확인 또는 이미 지나온 구간이 필요합니다.'))
            if self.cfg.recovery_min_distance>0:
                self.get_logger().info(f'1회 실제 후진 최소 {self.cfg.recovery_min_distance*100:.0f}cm와 정지 여유가 확보된 경로만 채택합니다.')
            if self.cfg.recovery_on_path_failure:
                self.get_logger().info(f'목표 옆 또는 정지한 확정 경로에서 전진 계획 {self.cfg.recovery_failure_count}회 실패 시에도 '
                                       f'기억 지도에서 비어 있는 후방으로 복구 허용. 미션 누적 후진 한도 {self.cfg.recovery_total_distance*100:.0f}cm.')

    def on_tuning_parameters(self, parameters):
        updates = {p.name:p.value for p in parameters}
        config_names = {f.name for f in fields(Settings)}
        attrs = {'recognition_frames':'required_detections','sign_timeout':'sign_timeout',
                 'scan_timeout':'scan_timeout','plan_period':'plan_period','parking_speed':'speed',
                 'parking_hold_seconds':'hold_seconds','exit_heading_tolerance':'exit_heading_tolerance',
                 'max_deceleration':'deceleration','max_yaw_rate':'max_yaw_rate','lookahead':'lookahead',
                 'control_hz':'control_hz','require_parking_heading':'require_parking_heading',
                 'require_path_heading':'require_path_heading','parking_position_tolerance':'parking_position_tolerance',
                 'max_steering_rate':'max_steering_rate','min_turn_speed':'min_turn_speed'}
        unsupported = set(updates)-config_names-set(attrs)
        if unsupported:
            return SetParametersResult(successful=False,reason='재시작이 필요한 설정: '+', '.join(sorted(unsupported)))
        try:
            cfg = replace(self.cfg,**{k:v for k,v in updates.items() if k in config_names})
            values = {name:updates.get(name,getattr(self,attr)) for name,attr in attrs.items()}
            if 'require_path_heading' in updates:
                cfg = replace(cfg,require_goal_heading=values['require_path_heading'])
            elif 'require_goal_heading' in updates:
                values['require_path_heading'] = cfg.require_goal_heading
            validate_settings(cfg)
            for name,value in updates.items():
                if isinstance(value,(int,float)) and not isinstance(value,bool) and not math.isfinite(value):
                    raise ValueError(name+' must be finite')
            positive = ('sign_timeout','scan_timeout','plan_period','parking_speed','max_deceleration',
                        'max_yaw_rate','lookahead','control_hz','parking_position_tolerance')
            if any(values[k] <= 0 for k in positive):
                raise ValueError('주기·속도·LDF·허용 오차는 양수여야 합니다.')
            if (values['recognition_frames'] < 1 or values['parking_hold_seconds'] < 0
                    or values['max_steering_rate'] < 0):
                raise ValueError('감지 횟수는 1 이상이며 대기 시간·조향 변화율은 0 이상이어야 합니다.')
            template = Follower(cfg,values['parking_speed'],values['lookahead'],values['parking_hold_seconds'],
                                values['max_deceleration'],values['max_yaw_rate'],values['exit_heading_tolerance'],
                                values['require_parking_heading'],values['parking_position_tolerance'],
                                values['max_steering_rate'],values['min_turn_speed'])
        except (ValueError,TypeError) as exc:
            return SetParametersResult(successful=False,reason=str(exc))
        if any(name.startswith('slot_') for name in updates):
            self.slot_map=None
            if self.follower.goal is not None and self.follower.goal.slot_id>=0:
                self.follower.awaiting_target_path=True
        old_cfg,old_hz = self.cfg,self.control_hz
        self.cfg = cfg
        for name,attr in attrs.items():
            setattr(self,attr,values[name])
        self.activation.required,self.activation.timeout = self.required_detections,self.sign_timeout
        self.follower.cfg = cfg
        if 'hold_entry_path' in updates:
            self.follower.entry_locked = bool(cfg.hold_entry_path and self.follower.path
                                             and not self.follower.recovering and not self.follower.exiting)
            self.follower.entry_path = self.follower.path.copy() if self.follower.entry_locked else []
        for name in ('speed','lookahead','hold_seconds','deceleration','max_yaw_rate','exit_heading_tolerance',
                     'require_parking_heading','parking_position_tolerance','max_steering_rate','min_turn_speed'):
            setattr(self.follower,name,getattr(template,name))
        self.follower.recovery_distance_limit = min(self.follower.recovery_distance_limit,cfg.recovery_reverse_distance)
        consumed = max(0.,old_cfg.blind_reverse_distance-self.blind_reverse_remaining)
        self.blind_reverse_remaining = max(0.,cfg.blind_reverse_distance-consumed)
        consumed_total = max(0.,old_cfg.recovery_total_distance-self.recovery_remaining)
        self.recovery_remaining = max(0.,cfg.recovery_total_distance-consumed_total)
        if self.localization_source == 'scan':
            self.tracker.max_interval = self.scan_timeout
            self.tracker.max_speed = self.speed
            self.tracker.max_yaw_rate = self.speed*math.tan(cfg.steering_limit)*cfg.steering_response_scale/cfg.wheelbase
        planning_changes = config_names-{'steering_gain','max_steering_on_curves','control_preview_time'}
        if set(updates)&(planning_changes|{'require_path_heading'}):
            self.epoch += 1
            if self.pending is not None:
                self.pending[0].cancel()
                self.pending = None
            self.last_plan_wall = -math.inf
        if self.control_hz != old_hz:
            self.destroy_timer(self.control_timer)
            self.control_timer = self.create_timer(1./self.control_hz,self.tick)
        return SetParametersResult(successful=True)

    def now_seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def new_follower(self):
        return Follower(self.cfg, self.speed, self.lookahead, self.hold_seconds, self.deceleration,
                        self.max_yaw_rate, self.exit_heading_tolerance,
                        self.require_parking_heading, self.parking_position_tolerance, self.max_steering_rate,
                        self.min_turn_speed)

    def new_tracker(self):
        if self.localization_source == 'fused':
            return FusedTracker(self.odometry)
        return ScanTracker(self.speed, self.speed * math.tan(self.cfg.steering_limit)*self.cfg.steering_response_scale / self.cfg.wheelbase,
                           self.scan_timeout)

    def on_fusion_health(self, msg):
        try:
            health = json.loads(msg.data)
            if not isinstance(health.get('ready'), bool) or not math.isfinite(health['stamp']):
                return
            self.fusion_health = health
            self.fusion_health_received = time.monotonic()
        except (ValueError, TypeError, KeyError, AttributeError):
            return

    def on_localization(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        q, p = msg.pose.pose.orientation, msg.pose.pose.position
        values = (q.x, q.y, q.z, q.w, p.x, p.y, *msg.pose.covariance)
        if (msg.header.frame_id != self.odom_frame or msg.child_frame_id != self.base_frame
                or not -.05 <= self.now_seconds()-stamp <= self.localization_timeout
                or not all(math.isfinite(v) for v in values)
                or abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1.) > .01):
            self.odometry.reason = 'Invalid EKF frame, timestamp, quaternion or covariance'
            return
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        accepted = self.odometry.add(OdomSample(stamp, (p.x, p.y, yaw), msg.twist.twist.linear.x,
                                               msg.twist.twist.angular.z,
                                               tuple(msg.pose.covariance[i] for i in (0, 7, 35))))
        if accepted:
            self.drain_pending_scans()

    def drain_pending_scans(self):
        now = self.now_seconds()
        ready = None
        remaining = deque(maxlen=self.pending_scans.maxlen)
        for msg in self.pending_scans:
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
            if (now-stamp > self.scan_timeout or
                    (self.last_scan_stamp is not None and stamp <= self.last_scan_stamp)):
                continue
            if self.odometry.at(stamp) is not None:
                if ready is None or stamp > ready.header.stamp.sec+ready.header.stamp.nanosec*1e-9:
                    ready = msg
            else:
                remaining.append(msg)
        self.pending_scans = remaining
        if ready is not None:
            self.on_scan(ready)

    def localization_ready(self, now):
        if self.localization_source == 'scan':
            return True
        health = self.fusion_health
        if health is None or self.fusion_health_received is None:
            self.reason = 'Fusion health unavailable'
            return False
        if (time.monotonic()-self.fusion_health_received > self.localization_timeout
                or not -.05 <= now-health['stamp'] <= self.localization_timeout):
            self.reason = 'Fusion health is stale or future-dated'
            return False
        if not health['ready']:
            self.reason = 'Fusion inputs not ready: '+str(health.get('reason') or
                           'Wheel/IMU inputs unavailable or gyro calibration not complete')
            return False
        if not self.tracker.healthy(now):
            self.reason = self.tracker.reason
            return False
        return True

    def clear_visuals(self, clear_obstacles=True):
        held = (self.follower.entry_locked or (self.cfg.continuous_replanning and self.follower.path)) and not self.follower.exiting
        self.path_pub.publish(self.path_message(self.visible_path() if held else []))
        publishers = [self.target_pub, self.lookahead_pub]
        if self.slot_map is None:publishers.append(self.slot_map_pub)
        if clear_obstacles:
            self.memory_marker_ids.clear()
            self.publish_memory_cloud([])
            publishers.append(self.obstacles_pub)
            self.obstacle_marker_count = 0
        for publisher in publishers:
            clear = Marker()
            clear.action = Marker.DELETEALL
            publisher.publish(MarkerArray(markers=[clear]))

    def reset_motion(self):
        self.memory.reset()
        self.last_memory_wall = -math.inf
        self.epoch += 1
        if self.pending is not None:
            self.pending[0].cancel()
            self.pending = None
        self.follower = self.new_follower()
        if self.odometry.discontinuity:
            self.odometry.samples.clear()
            self.odometry.discontinuity = False
            self.odometry.reason = ''
        self.tracker = self.new_tracker()
        self.scan, self.scan_header, self.scan_time = None, None, None
        self.last_scan_stamp = None
        self.scan_not_before = None
        self.pending_scans.clear()
        self.scan_error = ''
        self.controlled_sequence = -1
        self.reason = ''
        self.last_plan_wall = -math.inf
        self.planning_info = {}
        self.had_control_path = False
        self.selected_pair = []
        self.selected_track_ids = set()
        self.planned_goal = None
        self.tracked_goal = None
        self.slot_map = None
        self.last_recovery_stop = None
        self.path_hidden_by_slot = False
        self.last_planner_error = ''
        self.recovery_attempts = 0
        self.failed_plans = 0
        self.recovery_remaining = self.cfg.recovery_total_distance
        self.blind_reverse_remaining = self.cfg.blind_reverse_distance
        self.recovery_running = False
        self.observed_corridor.clear()
        self.clear_visuals()

    def reset_state(self):
        self.reset_motion()
        self.activation.reset()
        self.state = 'WAITING_SIGN'
        self.active_pub.publish(Bool(data=False))

    def on_reset(self, request, response):
        # Explicit reset is also allowed to release the mux after a final stop.
        if self.activation.active:
            self.publish_command(0.0, 0.0)
        self.reset_state()
        response.success = True
        response.message = f'주차 미션을 초기화했습니다. 연속 시작 신호 {self.required_detections}회를 기다립니다.'
        self.get_logger().info(response.message)
        return response

    def on_sign(self, msg):
        if time.monotonic() < self.sign_ignore_until:
            return
        was_active = self.activation.active
        generation = self.activation.generation
        mission_now = self.now_seconds()
        if was_active:
            if self.cfg.continuous_replanning and self.follower.path and not self.follower.exited:
                return  # Repeated detections must not interrupt a route in motion.
            if not msg.data:
                return
            self.activation.restart(mission_now)
        else:
            self.activation.update(msg.data, mission_now)
        if self.activation.generation != generation:
            self.sign_ignore_until = time.monotonic() + 5.
            if was_active and (self.had_control_path or self.follower.path):
                self.publish_command(0.0, 0.0)
            self.reset_motion()
            if was_active:
                # Ignore queued scans acquired before the new start signal.
                self.scan_not_before = mission_now
            self.attempt += 1
            # Registration origin is the first scan acquired after activation.
            self.state, self.reason = 'WAITING_SCAN', ''
            self.active_pub.publish(Bool(data=True))
            self.get_logger().info(f'주차 미션 시작: {self.attempt}번째 시도. 최신 센서와 경로를 확인합니다.')

    def on_scan(self, msg):
        now = self.now_seconds()
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        valid_geometry = (msg.header.frame_id in (self.base_frame, self.lidar_frame)
                          and len(msg.ranges) >= 3 and math.isfinite(msg.angle_min)
                          and math.isfinite(msg.angle_increment) and msg.angle_increment != 0
                          and math.isfinite(msg.range_min) and math.isfinite(msg.range_max)
                          and 0 <= msg.range_min < msg.range_max)
        if not valid_geometry or not -SCAN_FUTURE_TOLERANCE <= now - stamp <= self.scan_timeout:
            self.scan = None
            self.scan_error = self.reason = 'Invalid scan geometry/frame or stale timestamp'
            self.clear_visuals()
            return
        if self.scan_not_before is not None and stamp < self.scan_not_before:
            return
        if self.last_scan_stamp is not None and stamp <= self.last_scan_stamp:
            return
        if self.localization_source == 'fused' and self.odometry.at(stamp) is None:
            # DDS callbacks for the two topics need not arrive in timestamp
            # order. Wait for actual matching EKF samples, without discarding
            # an accepted route or extrapolating through an input outage.
            if not any(m.header.stamp == msg.header.stamp for m in self.pending_scans):
                self.pending_scans.append(msg)
            return
        self.last_scan_stamp = stamp
        sensor_pose = (0.0, 0.0, 0.0) if msg.header.frame_id == self.base_frame else self.sensor_pose
        raw_scan = Scan(msg.ranges, msg.angle_min, msg.angle_increment, msg.range_min,
                        msg.range_max, sensor_pose, self.cfg.max_range)
        self.scan = raw_scan.filtered_obstacles(self.cfg)
        self.scan_time, self.scan_header = stamp, msg.header
        self.scan_error = ''
        if self.reason in ('Waiting for a fresh scan', 'Invalid scan geometry/frame or stale timestamp'):
            self.reason = ''
        self.scan_sequence += 1
        clusters = cluster_points(self.scan.points, self.cfg.cluster_distance, self.cfg.cluster_min_points)
        if not (self.memory_enabled and self.activation.active):
            self.publish_obstacles(clusters)
        was_tracking = self.tracker.valid
        if (self.activation.active or self.localization_source == 'fused') and not self.tracker.update(raw_scan.points, stamp=stamp):
            self.state, self.reason = 'TRACKING_LOST', f'Scan matching failed: {self.tracker.reason}; retry required'
            if was_tracking:
                self.get_logger().warning(reason_text(self.reason))
            if not (self.follower.entry_locked or self.cfg.continuous_replanning):
                self.follower.path = []
            self.path_pub.publish(self.path_message(self.visible_path()
                                                   if self.follower.entry_locked or self.cfg.continuous_replanning else []))
        if self.memory_enabled and self.activation.active and self.tracker.valid:
            self.memory.update(raw_scan,self.scan,self.tracker.pose,stamp,self.cfg)
        if self.localization_source == 'fused' and self.tracker.valid:
            if self.state in ('WAITING_LOCALIZATION', 'TRACKING_LOST') or self.reason.startswith('Scan matching failed:'):
                self.reason = ''
            pose = self.tracker.global_pose
            output = Odometry()
            output.header.stamp = msg.header.stamp
            output.header.frame_id, output.child_frame_id = self.odom_frame, self.base_frame
            output.pose.pose.position.x, output.pose.pose.position.y = pose[:2]
            output.pose.pose.orientation.z, output.pose.pose.orientation.w = math.sin(pose[2]/2), math.cos(pose[2]/2)
            for i, variance in zip((0, 7, 35), self.tracker.sample.variance):
                output.pose.covariance[i] = variance
            output.twist.twist.linear.x = self.tracker.sample.speed
            output.twist.twist.angular.z = self.tracker.sample.yaw_rate
            self.localization_pub.publish(output)

    def planning_scan(self):
        scan=self.scan
        if (self.memory_enabled and self.activation.active and self.scan is not None
                and self.memory.stamp is not None
                and (self.memory.stamp==self.scan_time or self.cfg.recovery_uses_memory_map)):
            scan=self.memory.snapshot(self.scan,self.tracker.pose,self.cfg)
        if scan is not None and self.slot_map is not None and self.slot_map.get('locked'):
            from .core import Cluster
            from .enclosed import physical_objects
            from .slot_map import fit_slot_map
            world=[Cluster([compose(self.tracker.pose,(*p,0.))[:2] for p in c.points],c.track_id) for c in scan.candidate_clusters(self.cfg)]
            objects=[(ids,c) for ids,c in physical_objects(world,self.cfg,right_only=False)
                     if self.cfg.min_cluster_size<=max(c.bounds[1]-c.bounds[0],c.bounds[3]-c.bounds[2])<=self.cfg.max_cluster_size]
            self.slot_map=fit_slot_map(objects,self.cfg,self.slot_map)
        return with_occupied_slots(scan,self.slot_map,self.tracker.pose,self.cfg) if scan is not None else None

    def visible_path(self):
        path=self.follower.local_path(self.tracker.pose)
        self.path_hidden_by_slot=False
        scan=self.planning_scan()
        if scan is not None and path:
            checker=CollisionChecker(scan,self.cfg)
            if first_occupied_path_collision(checker,path,self.cfg.collision_step):
                self.path_hidden_by_slot=True
                remaining=[compose(inverse(self.tracker.pose),p) for p in self.follower.path[self.follower.index:]]
                if not first_occupied_path_collision(checker,remaining,self.cfg.collision_step):return remaining
                return []
        return path

    def publish_memory_cloud(self, points):
        msg = PointCloud2()
        msg.header = Header(frame_id=self.base_frame,stamp=self.get_clock().now().to_msg())
        msg.height,msg.width = 1,len(points)
        msg.fields = [PointField(name=name,offset=i*4,datatype=PointField.FLOAT32,count=1)
                      for i,name in enumerate(('x','y','z'))]
        msg.is_bigendian,msg.is_dense = False,True
        msg.point_step,msg.row_step = 12,12*len(points)
        msg.data = b''.join(struct.pack('<fff',x,y,.04) for x,y in points)
        self.memory_pub.publish(msg)

    def publish_memory(self):
        if not (self.memory_enabled and self.activation.active and self.tracker.valid):
            return
        wall = time.monotonic()
        if wall-self.last_memory_wall < .1:
            return
        self.last_memory_wall = wall
        pose = self.tracker.current_pose(self.now_seconds()) if self.localization_source == 'fused' else self.tracker.pose
        self.publish_memory_cloud(self.memory.local_points(pose))
        boxes = self.memory.boxes(pose)
        ids = {tid for tid,_,_ in boxes}
        markers = []
        for tid in self.memory_marker_ids-ids:
            removed = self.marker('cluster_box',tid,Marker.LINE_STRIP)
            removed.action = Marker.DELETE
            markers.append(removed)
        for tid,observed,corners in boxes:
            box = self.marker('cluster_box',tid,Marker.LINE_STRIP)
            box.header.stamp = self.get_clock().now().to_msg()
            box.scale.x = .015
            box.color.r,box.color.g,box.color.b = ((1.,0.,1.) if tid in self.selected_track_ids
                                                 else ((0.,.3,1.) if observed else (1.,.6,0.)))
            if tid in self.selected_track_ids:
                box.scale.x = .025
            box.points = [Point(x=float(x),y=float(y),z=.04) for x,y in corners]
            markers.append(box)
        self.memory_marker_ids = ids
        self.obstacles_pub.publish(MarkerArray(markers=markers))

    def path_message(self, points):
        msg = Path()
        msg.header.frame_id = self.base_frame
        msg.header.stamp = self.scan_header.stamp if self.scan_header else self.get_clock().now().to_msg()
        for x, y, yaw in points:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x, pose.pose.position.y = float(x), float(y)
            pose.pose.orientation.z, pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            msg.poses.append(pose)
        return msg

    def marker(self, namespace, index, kind):
        m = Marker()
        m.header.frame_id = self.base_frame
        m.header.stamp = self.scan_header.stamp if self.scan_header else self.get_clock().now().to_msg()
        m.ns, m.id, m.type = namespace, index, kind
        m.pose.orientation.w = 1.0
        m.lifetime.sec, m.lifetime.nanosec = 0, 600000000
        m.color.a = 1.0
        return m

    def publish_obstacles(self, clusters):
        # Updating existing IDs keeps boxes visible between scan callbacks.
        # Delete only IDs that vanished instead of clearing the entire display.
        markers = []
        for index in range(len(clusters), self.obstacle_marker_count):
            removed = self.marker('cluster_box', index, Marker.LINE_STRIP)
            removed.action = Marker.DELETE
            markers.append(removed)
        self.obstacle_marker_count = len(clusters)
        for index, cluster in enumerate(clusters):
            xmin, xmax, ymin, ymax = cluster.bounds
            box = self.marker('cluster_box', index, Marker.LINE_STRIP)
            box.scale.x = .015
            center = compose(self.tracker.pose,(*cluster.center,0.))[:2]
            chosen = any(math.dist(center,p['center']) <= .25 for p in self.selected_pair)
            box.color.r, box.color.g, box.color.b = (1.,0.,1.) if chosen else (0.,.3,1.)
            box.points = [Point(x=float(x), y=float(y), z=.04) for x, y in
                          ((xmin, ymin), (xmax, ymin), (xmax, ymax), (xmin, ymax), (xmin, ymin))]
            markers.append(box)
        self.obstacles_pub.publish(MarkerArray(markers=markers))

    def publish_slot_map(self):
        if self.slot_map is None or not self.tracker.valid:return
        layout=self.slot_map;transform=inverse(self.tracker.pose)
        yaw=layout['yaw'];co,si=math.cos(yaw),math.sin(yaw)
        selected=self.tracked_goal or self.follower.goal
        chosen=selected.slot_id if selected is not None else -1
        clear=Marker();clear.action=Marker.DELETEALL;markers=[clear]
        for cell in layout['cells']:
            color=(1.,.15,.7) if cell['id']==chosen else ((1.,.25,.10) if cell['occupied'] else
                  ((.1,.85,1.) if cell['eligible_empty'] else (1.,.75,.15)))
            x,y=cell['center'];hw,hd=layout['cell_width']/2,layout['cell_depth']/2
            m=self.marker('slot_grid',cell['id'],Marker.LINE_STRIP);m.scale.x=.018
            m.color.r,m.color.g,m.color.b=color
            for dx,dy in [(-hw,-hd),(hw,-hd),(hw,hd),(-hw,hd),(-hw,-hd)]:
                px,py,_=compose(transform,(x+co*dx-si*dy,y+si*dx+co*dy,0.))
                m.points.append(Point(x=px,y=py,z=.06))
            markers.append(m)
            text=self.marker('slot_labels',cell['id'],Marker.TEXT_VIEW_FACING)
            text.pose.position.x,text.pose.position.y,_=compose(transform,(x,y,0.));text.pose.position.z=.15
            text.scale.z=.055;text.color.r,text.color.g,text.color.b=color
            text.text=f"{cell['id']+1} "+('TARGET' if cell['id']==chosen else cell['state'])
            markers.append(text)
        info=self.marker('slot_fit',0,Marker.TEXT_VIEW_FACING)
        info.pose.position.x,info.pose.position.y,_=compose(transform,(*layout['origin'],0.));info.pose.position.z=.25
        info.scale.z=.055;info.color.r=info.color.g=info.color.b=1.
        error=layout.get('fit_rmse_m');error_text='--' if error is None else f'{error:.2f}m'
        info.text=f"{layout['columns']}x{layout['rows']} | matched {layout['matched_cells']} | error {error_text} | "+('locked' if layout['locked'] else 'partial')
        markers.append(info);self.slot_map_pub.publish(MarkerArray(markers=markers))

    def publish_target(self):
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers = [clear]
        display_goal = self.tracked_goal or self.follower.goal
        if display_goal is None and self.planned_goal is not None:
            display_goal = Gap(*self.planned_goal,0.,-1,-1)
        if display_goal is not None and self.tracker.valid:
            goal = local_goal(display_goal, self.tracker.pose)
            m = self.marker('parking_goal', 0, Marker.ARROW)
            m.pose.position.x, m.pose.position.y = goal.x, goal.y
            m.pose.position.z = .05
            m.pose.orientation.z, m.pose.orientation.w = math.sin(goal.yaw / 2), math.cos(goal.yaw / 2)
            m.scale.x, m.scale.y, m.scale.z = .18, .04, .04
            m.color.g, m.color.b = 1.0, .2
            markers.append(m)
            center = self.marker('parking_goal_center',0,Marker.SPHERE)
            center.pose.position.x,center.pose.position.y,center.pose.position.z = goal.x,goal.y,.05
            center.scale.x = center.scale.y = center.scale.z = .06
            center.color.g,center.color.b = 1.,.2
            markers.append(center)
        self.target_pub.publish(MarkerArray(markers=markers))

    def publish_lookahead(self):
        distance = self.follower.lookahead_distance
        self.ldf_pub.publish(Float32(data=float(distance)))
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers = [clear]
        aim = self.follower.lookahead_point
        if self.activation.active and self.tracker.valid and self.scan is not None and aim is not None:
            stamp = self.get_clock().now().to_msg()
            circle = self.marker('ldf_circle', 0, Marker.LINE_STRIP)
            circle.header.stamp = stamp
            circle.scale.x = .008
            circle.color.r, circle.color.g = 1., .8
            circle.points = [Point(x=float(distance*math.cos(2*math.pi*i/32)),
                                   y=float(distance*math.sin(2*math.pi*i/32)), z=.03)
                             for i in range(33)]
            target = self.marker('ldf_target', 1, Marker.SPHERE)
            target.header.stamp = stamp
            target.pose.position.x, target.pose.position.y = float(aim[0]), float(aim[1])
            target.pose.position.z = .05
            target.scale.x = target.scale.y = target.scale.z = .06
            target.color.g = target.color.b = 1.
            ray = self.marker('ldf_ray', 2, Marker.LINE_STRIP)
            ray.header.stamp = stamp
            ray.scale.x = .01
            ray.color.g = ray.color.b = 1.
            ray.points = [Point(x=0., y=0., z=.05), Point(x=float(aim[0]), y=float(aim[1]), z=.05)]
            markers.extend((circle, target, ray))
        self.lookahead_pub.publish(MarkerArray(markers=markers))

    def publish_command(self, speed, steering):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base_frame
        msg.drive.speed, msg.drive.steering_angle = float(speed), float(steering)
        self.command_pub.publish(msg)

    def poll_plan(self):
        if self.pending is None or not self.pending[0].done():
            return
        future, epoch, origin, started = self.pending
        self.pending = None
        try:
            result = future.result()
            if isinstance(result, dict):
                goal, path, final = result['goal'], result['path'], result['final']
                diagnostics = result['diagnostics']
            else:
                # Preserve the public three-value planner/test interface.
                goal, path, final = result
                diagnostics = {}
        except Exception as exc:
            self.last_planner_error = str(exc)
            self.get_logger().error('주차 경로 계산 오류: '+reason_text(str(exc)))
            self.reason = 'Planner failure'
            return
        if (epoch != self.epoch or not self.activation.active or not self.tracker.valid
                or (self.follower.parked and not (self.cfg.automatic_target_selection
                    and not self.cfg.auto_exit_after_parking and self.follower.goal is not None
                    and (self.follower.goal.kind=='pair_midpoint' or not self.follower.goal.confirmed))) or (self.follower.final_approach and self.follower.path
                                           and self.state != 'FRONT_BLOCKED' and not self.cfg.continuous_replanning)):
            return
        if self.follower.parked and diagnostics.get('target_kind')!='enclosed_center':
            return
        if self.follower.entry_locked and (self.state != 'FRONT_BLOCKED'
                or diagnostics.get('reason_code') != 'recovery_ready'):
            return
        self.planning_info = diagnostics
        if self.cfg.slot_map_enabled and diagnostics.get('slot_map') is not None:
            self.slot_map = diagnostics['slot_map']
        # Transform the captured scan frame to the mission origin. Once the
        # final entry path is accepted, keep it through arrival confirmation.
        if self.now_seconds() - started > self.scan_timeout + self.cfg.planning_timeout:
            if not self.follower.path:
                self.reason = 'Planner result expired before acceptance; check planning load'
            return
        if 'selected_pair' in diagnostics:
            self.selected_pair = [dict(p,center=compose(origin,(*p['center'],0.))[:2])
                                  for p in diagnostics['selected_pair']]
            self.selected_track_ids = {p['track_id'] for p in self.selected_pair if p.get('track_id') is not None}
        elif self.follower.goal is None:
            self.selected_pair = []
            self.selected_track_ids = set()
        if diagnostics.get('goal_invalidated_by_slot_map'):
            self.tracked_goal=None
            self.follower.goal=None
            self.follower.awaiting_target_path=bool(self.follower.path)
        target = diagnostics.get('target')
        self.planned_goal = compose(origin,(target['x'],target['y'],target['yaw'])) if target else None
        if self.cfg.continuous_replanning and self.planned_goal is not None:
            self.tracked_goal = Gap(*self.planned_goal,0.,-1,-1,
                                    support_ids=tuple(diagnostics.get('support_ids',())),
                                    confirmed=diagnostics.get('goal_confirmed',False),
                                    kind=diagnostics.get('target_kind','legacy'),
                                    parking_grid=diagnostics.get('parking_grid'),
                                    slot_id=diagnostics.get('slot_id',-1),
                                    bay_yaw=diagnostics.get('bay_yaw'),boundary_bounds=tuple(diagnostics.get('boundary_bounds',())),
                                    heading_options=tuple(math.atan2(math.sin(a+origin[2]),math.cos(a+origin[2]))
                                        for a in diagnostics.get('target_heading_options',())),
                                    stop_tolerance=diagnostics.get('target_stop_tolerance',0.))
            if self.follower.parked and self.tracked_goal.kind=='enclosed_center':
                pose=self.tracker.current_pose(self.now_seconds()) if self.localization_source=='fused' else self.tracker.pose
                local=local_goal(self.tracked_goal,pose)
                if (local.heading_options and math.hypot(local.x,local.y)<=local.stop_tolerance
                        and min(abs(a) for a in local.heading_options)<=self.cfg.goal_yaw_tolerance):
                    self.follower.goal=self.tracked_goal
                    return
                self.follower.parked=False
                self.follower.parked_since=None
                self.follower.arrival_count=0
                self.follower.awaiting_target_path=True
            if self.follower.goal is not None:
                self.follower.goal = self.tracked_goal

        replacement_index = 0
        if (path and self.cfg.require_complete_entry_path and not final
                and diagnostics.get('reason_code')!='recovery_ready'):
            return
        if (path and final and self.cfg.require_complete_entry_path and self.follower.final_approach
                and self.follower.path and not self.follower.recovering and not self.follower.exiting
                and self.state not in ('ROUTE_BLOCKED','FRONT_BLOCKED','PATH_HELD','REPLAN')
                and self.follower.path_goal is not None and self.tracked_goal is not None
                and math.hypot(self.follower.path_goal.x-self.tracked_goal.x,
                               self.follower.path_goal.y-self.tracked_goal.y)<=.01):
            pose=self.tracker.current_pose(self.now_seconds()) if self.localization_source=='fused' else self.tracker.pose
            nearest=min(self.follower.path[self.follower.index:],key=lambda p:math.dist(p[:2],pose[:2]))
            checker=CollisionChecker(self.planning_scan(),self.cfg)
            transform=inverse(self.tracker.pose)
            if (math.dist(nearest[:2],pose[:2])<=.10 and
                    abs(math.atan2(math.sin(nearest[2]-pose[2]),math.cos(nearest[2]-pose[2])))<=.35 and
                    all(checker.free(compose(transform,p)) for p in self.follower.path[self.follower.index:])):
                diagnostics['retained_valid_path']=True
                self.failed_plans=0
                return
        if path and self.slot_map is not None:
            checker=CollisionChecker(self.planning_scan(),self.cfg)
            local_path=[compose(inverse(self.tracker.pose),compose(origin,p)) for p in path]
            blocked=first_occupied_path_collision(checker,local_path,self.cfg.collision_step)
            if blocked:
                diagnostics['candidate_path_rejected_occupied_slot']=blocked
                return
        if path and self.cfg.continuous_replanning and self.follower.path and diagnostics.get('reason_code') != 'recovery_ready':
            pose = self.tracker.current_pose(self.now_seconds()) if self.localization_source == 'fused' else self.tracker.pose
            global_path = [compose(origin,p) for p in path]
            replacement_index = min(range(len(global_path)),key=lambda i:math.hypot(global_path[i][0]-pose[0],global_path[i][1]-pose[1]))
            nearest = global_path[replacement_index]
            if (replacement_index >= len(global_path)-1 or math.hypot(nearest[0]-pose[0],nearest[1]-pose[1]) > .15
                    or abs(math.atan2(math.sin(nearest[2]-pose[2]),math.cos(nearest[2]-pose[2]))) > .35):
                return  # Retain the old feasible route when a delayed replacement cannot join.

        if not path and diagnostics.get('reason_code') in ('no_observed_path','budget_exceeded','start_blocked'):
            self.failed_plans += 1
        elif path and diagnostics.get('reason_code') != 'recovery_ready':
            self.failed_plans = 0

        if path:
            recovery = diagnostics.get('reason_code') == 'recovery_ready'
            if recovery:
                # Validate the proposal against current pose and measurements.
                pose = self.tracker.current_pose(self.now_seconds()) if self.localization_source == 'fused' else self.tracker.pose
                if (math.dist(origin[:2],pose[:2]) > .08
                        or abs(math.atan2(math.sin(origin[2]-pose[2]),math.cos(origin[2]-pose[2]))) > .15):
                    return
                global_goal = self.tracked_goal
                if global_goal is None and goal is not None:
                    global_goal = replace(goal, **dict(zip(('x','y','yaw'),compose(origin,(goal.x,goal.y,goal.yaw)))))
                local = local_goal(global_goal,pose) if global_goal else None
                clearance = front_clearance(self.scan,self.cfg) if self.scan else None
                failure_recovery = diagnostics.get('recovery_trigger') == 'path_failure'
                front_close = clearance is not None and clearance <= self.cfg.recovery_front_clearance
                length = diagnostics.get('recovery_distance_m',sum(math.dist(a[:2],b[:2]) for a,b in zip(path,path[1:])))
                if ((local is None and (failure_recovery or self.cfg.recovery_on_front_obstacle_only))
                        or (local is not None and math.hypot(local.x,local.y) <= (local.stop_tolerance or self.follower.parking_position_tolerance))
                        or self.scan is None
                        or self.recovery_attempts >= self.cfg.max_recovery_attempts
                        or length > self.recovery_remaining+1e-8
                        or length+1e-8 < minimum_recovery_path(self.cfg)
                        or (self.cfg.recovery_on_front_obstacle_only and not front_close and not
                            (failure_recovery and self.cfg.recovery_on_path_failure
                             and self.failed_plans >= self.cfg.recovery_failure_count and
                             (needs_reverse_for_goal(local,self.cfg) or (local.confirmed and self.state in
                              ('PATH_HELD','ROUTE_BLOCKED','FRONT_BLOCKED','PLANNING','REPLAN')))))):
                    return
                known = [compose(inverse(pose),p) for p in self.follower.entry_trace]+[(0.,0.,0.)]
                cfg = (replace(self.cfg,assume_unobserved_free=True) if self.cfg.recovery_uses_memory_map else
                       (replace(self.cfg,assume_unobserved_free=False) if failure_recovery else self.cfg))
                if self.cfg.recovery_uses_memory_map and (not self.memory_enabled or self.memory.stamp is None):return
                rear = CollisionChecker(self.planning_scan(),cfg,known,inverse(pose)[:2])
                local_path = [compose(inverse(pose),compose(origin,p)) for p in path]
                operator_cleared=(length <= self.blind_reverse_remaining+1e-8 and
                                  (not failure_recovery or diagnostics.get('blind_reverse_distance_m',0.)>0))
                if any(not rear.free(p,allow_unknown=operator_cleared) for p in local_path):
                    return
            if recovery:
                self.recovery_attempts += 1
                self.recovery_remaining = max(0.,self.recovery_remaining-length)
                self.failed_plans = 0
                self.blind_reverse_remaining = max(0., self.blind_reverse_remaining-
                                                   diagnostics.get('blind_reverse_distance_m', 0.))
            self.recovery_running = recovery
            if not recovery:self.last_recovery_stop=None
            self.follower.accept(goal, path, final, origin, recovery=recovery)
            if self.cfg.continuous_replanning and not recovery:
                self.follower.index = replacement_index
            if not diagnostics.get('blind_reverse_distance_m', 0.) and not diagnostics.get('recovery_uses_memory_map'):
                self.observed_corridor.extend((self.now_seconds(), pose) for pose in self.follower.path)
            self.reason = ''
            kind = '주변 재탐색을 위한 후진 복구' if recovery else ('최종 진입' if final else '관측 구간 접근')
            target_text = f'목표 전방 {goal.x:.2f}m / 좌우 {goal.y:.2f}m' if goal is not None else '후진 후 빈 칸 재탐색'
            self.get_logger().info(f'{kind} 경로 생성: {len(path)}개 점, '
                                   f'{target_text}, '
                                   f'계산 {diagnostics.get("duration_s", 0.)*1000:.0f}ms.')
        elif not self.follower.path:
            self.reason = 'No visible reachable gap/path'

    def tick(self):
        now = self.now_seconds()
        if self.last_now is not None and now < self.last_now - .01:
            if self.activation.active:
                self.publish_command(0.0, 0.0)
            self.reset_state()
            self.reason = 'Clock moved backwards'
        self.last_now = now
        self.activation.expire(now)
        self.poll_plan()
        speed, steering = 0.0, 0.0
        if not self.activation.active:
            self.state = 'WAITING_SIGN'
            self.clear_visuals(clear_obstacles=False)
        elif not self.localization_ready(now):
            self.state = 'WAITING_LOCALIZATION'
            self.clear_visuals(clear_obstacles=False)
        # /scan and /clock may be delivered in either order. Match on_scan's
        # accepted time window instead of discarding an already accepted scan.
        elif (self.scan is None or self.scan_time is None
              or not -SCAN_FUTURE_TOLERANCE <= now-self.scan_time <= self.scan_timeout):
            self.state = 'WAITING_SCAN'
            self.reason = self.scan_error or 'Waiting for a fresh scan'
            # Preserve the recorded departure route across a sensor interruption.
            if not self.follower.parked and not (self.follower.entry_locked or self.cfg.continuous_replanning):
                self.follower.path = []
            self.clear_visuals()
        elif not self.tracker.valid:
            self.state = 'TRACKING_LOST'
            self.reason = self.tracker.reason
        else:
            control_pose = self.tracker.current_pose(now) if self.localization_source == 'fused' else self.tracker.pose
            previous_state = self.state
            obstacle_scan=self.planning_scan()
            delta=compose(inverse(control_pose),self.tracker.pose)
            if math.hypot(delta[0],delta[1])>1e-6 or abs(delta[2])>1e-6:
                source=obstacle_scan.filtered_obstacles(self.cfg)
                obstacle_scan=MemoryScan.__new__(MemoryScan)
                obstacle_scan.__dict__=source.__dict__.copy()
                obstacle_scan.points=[compose(delta,(*p,0.))[:2] for p in source.points]
                obstacle_scan.point_owners={}
                obstacle_scan.sensor_pose=compose(delta,source.sensor_pose)
                obstacle_scan.occupied_slots=[dict(r,pose=compose(delta,r['pose'])) for r in getattr(source,'occupied_slots',())]
            speed, steering, self.state = self.follower.command(
                self.scan, control_pose, self.controlled_sequence != self.scan_sequence, now=now, obstacle_scan=obstacle_scan)
            if self.state in ('APPROACHING', 'PARKING', 'REVERSING', 'RECOVERING', 'CHANGING_DIRECTION',
                              'RECOVERY_BLOCKED', 'REOBSERVE', 'REPLAN', 'PATH_HELD', 'ROUTE_BLOCKED', 'CONFIRMING', 'PARKED', 'EXITING', 'EXIT_CONFIRMING', 'EXITED'):
                self.reason = ''
            if self.state=='RECOVERY_BLOCKED':
                self.last_recovery_stop=(dict(self.follower.last_collision,frame=self.base_frame,stamp=now,
                    base_pose=list(control_pose)) if self.follower.last_collision else None)
            if self.state == 'FRONT_BLOCKED':
                self.reason = 'Front obstacle too close before arrival'
                self.planning_info['front_clearance_m'] = front_clearance(self.scan,self.cfg)
                if previous_state != 'FRONT_BLOCKED':
                    self.epoch += 1
                    if self.pending is not None:
                        self.pending[0].cancel()
                        self.pending = None
                    self.last_plan_wall = -math.inf
            if not self.follower.recovering:
                self.recovery_running = False
            self.controlled_sequence = self.scan_sequence
            wall_now = time.monotonic()
            observe_pair=(self.cfg.automatic_target_selection and not self.cfg.auto_exit_after_parking
                          and self.follower.goal is not None
                          and (self.follower.goal.kind=='pair_midpoint' or not self.follower.goal.confirmed))
            if ((not self.follower.parked or observe_pair) and self.pending is None
                    and not self.recovery_running
                    and not (self.follower.entry_locked and self.state != 'FRONT_BLOCKED')
                    and not (self.follower.final_approach and self.follower.path and self.state != 'FRONT_BLOCKED' and not self.cfg.continuous_replanning)
                    and wall_now-self.last_plan_wall >= self.plan_period):
                cfg = replace(self.cfg, row_yaw=self.cfg.row_yaw - self.tracker.pose[2],
                              recovery_reverse_distance=min(self.cfg.recovery_reverse_distance,max(.001,self.recovery_remaining)))
                goal_source = self.tracked_goal or self.follower.goal
                goal = local_goal(goal_source, self.tracker.pose) if goal_source else None
                transform = inverse(self.tracker.pose)
                known = [compose(transform, p) for p in self.follower.entry_trace]
                while self.observed_corridor and now-self.observed_corridor[0][0] > 5.:
                    self.observed_corridor.popleft()
                if not cfg.assume_unobserved_free:
                    known.extend(compose(transform, p) for _, p in self.observed_corridor)
                known.append((0., 0., 0.))
                known = [p for p in known if math.hypot(*p[:2]) <= cfg.max_range+1.]
                recovery_allowed = (self.recovery_attempts < self.cfg.max_recovery_attempts and self.recovery_remaining >= 3*cfg.collision_step
                    and (not cfg.recovery_uses_memory_map or (self.memory_enabled and self.memory.stamp is not None)))
                self.pending = (self.worker.submit(build_plan_report, self.planning_scan(), cfg, goal, self.tracker.pose,
                                                   known, recovery_allowed, self.blind_reverse_remaining, True, self.scan, self.failed_plans,
                                                   bool(self.follower.path) and self.state in ('PATH_HELD','ROUTE_BLOCKED','FRONT_BLOCKED','PLANNING','REPLAN'),self.slot_map),
                                self.epoch, self.tracker.pose, now)
                self.last_plan_wall = wall_now
            self.path_pub.publish(self.path_message(self.visible_path()))
            self.publish_target()
            self.publish_slot_map()
        has_control_path = self.activation.active and bool(self.follower.path)
        if has_control_path:
            self.publish_command(speed, steering)
        elif self.had_control_path:
            # Stop the previous route once, then let the mux input expire.
            self.publish_command(0.,0.)
        self.had_control_path = has_control_path
        self.publish_lookahead()
        self.publish_memory()
        self.planning_info.update(recovery_stop=self.last_recovery_stop,
            recovery_remaining_m=self.recovery_remaining,recovery_attempts=self.recovery_attempts,
            max_recovery_attempts=self.cfg.max_recovery_attempts,path_hidden_by_slot=self.path_hidden_by_slot)
        explanation = operator_reason(self.state, self.reason, self.planning_info,
                                      self.activation.count, self.required_detections)
        log_now = time.monotonic()
        state_changed = self.state != self.reported_state
        reason_changed = explanation != self.reported_reason and log_now-self.last_operator_log_wall >= 1.
        stopped_heartbeat = self.activation.active and speed == 0 and log_now-self.last_operator_log_wall >= 5.
        if state_changed or reason_changed or stopped_heartbeat:
            transition = f'{state_text(self.reported_state)} → {state_text(self.state)}' if state_changed else state_text(self.state)
            label = '정지 이유' if speed == 0 else '상태 설명'
            self.get_logger().info(f'주차 상태: {transition} (시도 {self.attempt})'
                                   +(f' | {label}: {explanation}' if explanation else ''))
            self.reported_state = self.state
            self.reported_reason = explanation
            self.last_operator_log_wall = log_now
        if time.monotonic() - self.last_status_wall >= .2:
            control_pose = self.tracker.current_pose(now) if self.localization_source == 'fused' else self.tracker.pose
            goal_error = (local_goal(self.follower.goal, control_pose)
                          if self.follower.goal is not None and self.tracker.valid else None)
            localization = (self.tracker.diagnostics(now) if self.localization_source == 'fused'
                            else {'source': 'scan', 'valid': self.tracker.valid})
            if 'reason' in localization:
                localization['reason'] = reason_text(localization['reason'])
            if 'lidar_status' in localization:
                localization['lidar_status'] = reason_text(localization['lidar_status'])
            self.active_pub.publish(Bool(data=self.activation.active))
            self.status_pub.publish(String(data=json.dumps({
                'state': self.state, 'state_text': state_text(self.state), 'active': self.activation.active,
                'mission_attempt': self.attempt,
                'consecutive_detections': self.activation.count, 'reason': explanation,
                'path_poses': len(self.follower.path),
                'path_hidden_by_occupied_slot':self.path_hidden_by_slot,
                'last_recovery_stop':self.last_recovery_stop,
                'steering_gain': self.cfg.steering_gain,
                'steering_response_scale': self.cfg.steering_response_scale,
                'lookahead_base_m': self.follower.lookahead,
                'publishing_control': has_control_path,
                'goal_confirmed': bool((self.tracked_goal or self.follower.goal) and (self.tracked_goal or self.follower.goal).confirmed),
                'target_kind': (self.tracked_goal or self.follower.goal).kind if (self.tracked_goal or self.follower.goal) else 'none',
                'auto_exit_after_parking': self.cfg.auto_exit_after_parking,
                'goal_support_count': len((self.tracked_goal or self.follower.goal).support_ids) if (self.tracked_goal or self.follower.goal) else 0,
                'selected_pair': self.selected_pair,
                'slot_map':self.slot_map,
                'target_slot_id':(self.tracked_goal or self.follower.goal).slot_id if (self.tracked_goal or self.follower.goal) else -1,
                'entry_path_locked': self.follower.entry_locked,
                'entry_path_poses': len(self.follower.entry_path), 'scan_match_error_m': self.tracker.error,
                'localization': localization, 'planning': self.planning_info,
                'recovery_attempts': self.recovery_attempts,
                'recovery_remaining_m': self.recovery_remaining,
                'failed_plans': self.failed_plans,
                'max_recovery_attempts': self.cfg.max_recovery_attempts,
                'blind_reverse_remaining_m': self.blind_reverse_remaining,
                'speed': speed, 'steering_angle': steering,
                'obstacle_memory': self.memory.diagnostics(),
                'ldf_m': self.follower.lookahead_distance,
                'lookahead_point': list(self.follower.lookahead_point[:2]) if self.follower.lookahead_point is not None else None,
                'parking_completed': self.follower.parked, 'exit_completed': self.follower.exited,
                'exit_heading_tolerance_rad': self.exit_heading_tolerance,
                'exit_path_length_m': self.follower.exit_path_length,
                'entry_trace_poses': len(self.follower.entry_trace),
                'goal_error': ({'x': goal_error.x, 'y': goal_error.y, 'yaw': goal_error.yaw}
                               if goal_error is not None else None),
                'parking_position_tolerance_m': self.parking_position_tolerance,
                'arrival_reference':'rear_axle' if self.cfg.arrival_use_rear_axle else 'base_link',
                'rear_axle_offset_base':[self.cfg.rear_axle_offset_x,self.cfg.rear_axle_offset_y],
                'rear_axle_error_base':self.follower.arrival_error(goal_error),
                'require_parking_heading': self.require_parking_heading,
                'require_path_heading': self.require_path_heading,
                'min_turn_radius_m': self.cfg.turn_radius,
                'effective_max_steering_rad': self.cfg.steering_limit,
                'arrival_confirmations': self.follower.arrival_count,
                'ignored_small_clusters': self.scan.ignored_small_clusters if self.scan else 0,
                'ignored_small_points': self.scan.ignored_small_points if self.scan else 0,
                'pending_scan_alignment': len(self.pending_scans),
                'hold_remaining_seconds': (max(0., self.hold_seconds - (now - self.follower.parked_since))
                                           if self.follower.parked_since is not None else 0.),
            }, allow_nan=False, ensure_ascii=False)))
            self.last_status_wall = time.monotonic()

    def destroy_node(self):
        if self.activation.active and rclpy.ok(context=self.context):
            self.publish_command(0.0, 0.0)
        self.worker.shutdown(wait=True, cancel_futures=True)
        return super().destroy_node()


def main(args=None):
    stopped = Event()
    previous = {}
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    try:
        node = ParkingNode()
    except Exception as exc:
        rclpy.logging.get_logger('parking_node').error('주차 노드를 시작하지 못했습니다: '+reason_text(str(exc)))
        rclpy.shutdown()
        return 1
    exit_code = 0
    try:
        for signum in (signal.SIGINT, signal.SIGTERM):
            previous[signum] = signal.signal(signum, lambda *_: stopped.set())
        while rclpy.ok() and not stopped.is_set():
            rclpy.spin_once(node, timeout_sec=.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as exc:
        exit_code = 1
        node.get_logger().error('주차 노드 실행 중 오류가 발생해 종료합니다: '+reason_text(str(exc)))
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for signum, handler in previous.items():
            signal.signal(signum, handler)
    return exit_code


if __name__ == '__main__':
    raise SystemExit(main())
