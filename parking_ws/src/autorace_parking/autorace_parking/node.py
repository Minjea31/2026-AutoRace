"""Parking ROS adapter. Motion is emitted only through the documented mux input."""
from concurrent.futures import ProcessPoolExecutor, ThreadPoolExecutor
from collections import deque
from copy import deepcopy, copy
from dataclasses import fields, replace
import json
import math
import time
import multiprocessing
import signal
import struct
import traceback
from threading import Event

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from rcl_interfaces.srv import GetParameters
from rclpy.qos import QoSProfile, qos_profile_sensor_data, ReliabilityPolicy, DurabilityPolicy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.signals import SignalHandlerOptions
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_msgs.msg import Header
from std_msgs.msg import Bool, String, Float32, Float32MultiArray
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from .start_sequence import StartSequence
from .sign_motion import SignMotionGate
from .sign_sets import ParkingSignSet
from .departure import single_reverse_route, reverse_entry_route, arc_straight_departure, arc_then_straight, anchored_reverse_entry
from .lane_handoff import ExitLaneGate,ExitStopGate
from .core import Activation, Scan, Settings, Gap, CollisionChecker, RecoveryCollisionChecker, cluster_points, front_clearance, wrap
from .control import Follower, build_plan_report, local_goal,needs_reverse_for_goal,minimum_recovery_path,supported_recovery_target
from .regions import with_occupied_slots,first_occupied_path_collision,first_path_collision,outside_grid
from .tracking import ScanTracker, compose, inverse
from .localization import FusedTracker, PoseHistory, OdomSample
from .approach_reference import ApproachReference, relative_covariance
from .approach_calibration import ApproachCalibration
from .obstacle_memory import ObstacleMemory,MemoryScan
from .diagnostics import operator_reason, reason_text, state_text
from .roi import candidate_view
from .steering_command import parking_steering_command, LEGACY_INNER_ANGLE_SCALE

SCAN_FUTURE_TOLERANCE = .1


def planner_process_init():
    signal.signal(signal.SIGINT, signal.SIG_IGN)


def planner_warmup():
    return True


def scan_match_job(tracker, raw, cfg, update):
    # A private tracker/history snapshot; only the executor thread commits it.
    scan = raw.filtered_obstacles(cfg)
    clusters = cluster_points(scan.points, cfg.cluster_distance, cfg.cluster_min_points)
    matched = not update or tracker.update(raw.points, stamp=raw.job_stamp)
    return tracker, raw, scan, clusters, matched


def validate_settings(cfg):
    if (isinstance(cfg.candidate_plan_attempts,bool) or not isinstance(cfg.candidate_plan_attempts,int)
            or cfg.candidate_plan_attempts<1 or cfg.candidate_plan_timeout<=0):
        raise ValueError('Candidate planning attempts and timeout must be positive')
    if cfg.single_empty_slot and (not cfg.slot_map_enabled or cfg.fixed_parking_slot_id >= 0
                                  or cfg.fixed_slot_empty_known or cfg.slot_columns*cfg.slot_rows != 6):
        raise ValueError("single_empty_slot requires a dynamic six-cell parking grid")
    if cfg.fixed_slot_empty_known and (not cfg.slot_map_enabled or cfg.fixed_parking_slot_id<0):
        raise ValueError('A known empty slot requires a fixed parking-grid target')
    if (isinstance(cfg.fixed_parking_slot_id,bool) or not isinstance(cfg.fixed_parking_slot_id,int)
            or not -1<=cfg.fixed_parking_slot_id<cfg.slot_columns*cfg.slot_rows):
        raise ValueError('fixed_parking_slot_id is outside the parking grid')
    if (not 1<=cfg.exit_max_plan_attempts<=10 or cfg.exit_retry_timeout<=0
            or not 0<=cfg.parking_center_adjustment<=.05 or not 0<=cfg.entry_departure_reserve<=.05):
        raise ValueError('Invalid departure retry or parking center adjustment settings')
    if min(cfg.slot_object_length,cfg.slot_object_width,cfg.slot_object_fit_tolerance) <= 0:
        raise ValueError('장애물 사각형 크기·허용 오차는 0보다 커야 합니다.')
    if cfg.yellow_boundary_clearance < 0:
        raise ValueError('노란선 여유는 0 이상이어야 합니다.')
    for f in fields(Settings):
        value = getattr(cfg, f.name)
        if isinstance(value, (int, float)) and not isinstance(value, bool) and not math.isfinite(value):
            raise ValueError(f'{f.name} 설정은 유한한 숫자여야 합니다.')
    for name in ('cluster_distance', 'max_range', 'vehicle_length', 'vehicle_width', 'wheelbase',
                 'max_steering', 'resolution', 'yaw_resolution', 'step', 'collision_step', 'planning_timeout',
                 'goal_tolerance', 'goal_yaw_tolerance', 'recovery_front_clearance', 'steering_gain','steering_response_scale','reverse_steering_response_scale','control_preview_time',
                 'boundary_tolerance','target_heading_step','center_stop_tolerance','pair_stop_tolerance'):
        if getattr(cfg, name) <= 0:
            raise ValueError(f'{name} 설정은 0보다 커야 합니다.')
    if (min(cfg.path_preferred_clearance,cfg.path_clearance_weight,cfg.terminal_recovery_guard_distance,
            cfg.terminal_approach_distance,cfg.terminal_approach_speed,cfg.exit_path_tracking_margin)<0
            or not isinstance(cfg.path_clearance_candidates,int) or not 1<=cfg.path_clearance_candidates<=32):
        raise ValueError('Path clearance and terminal approach settings are invalid')
    if min(cfg.candidate_roi_forward_min,cfg.candidate_roi_right_min)<0 or cfg.candidate_roi_forward_min>=cfg.candidate_roi_forward_max or cfg.candidate_roi_right_min>=cfg.candidate_roi_right_max:
        raise ValueError('후보 ROI의 전방·오른쪽 범위는 0 이상이고 최소가 최대보다 작아야 합니다.')
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
    if (min(cfg.steering_command_gain_expected,cfg.steering_command_limit_expected)<=0
            or min(cfg.steering_command_deadband_expected,cfg.exit_start_speed,cfg.exit_steering_settle_time,cfg.exit_lane_tolerance)<0):
        raise ValueError('조향 보정과 출차 준비 속도·시간 설정을 확인하세요.')
    if cfg.require_steering_calibration:
        if cfg.steering_command_limit_expected * LEGACY_INNER_ANGLE_SCALE >= math.pi / 2:
            raise ValueError('주차 조향 보정 전송값 상한이 기본 드라이버의 변환 범위를 넘습니다.')
        inner=cfg.steering_command_gain_expected*cfg.steering_command_limit_expected-cfg.steering_command_deadband_expected
        if not 0<inner<math.pi/2 or cfg.turn_radius<cfg.wheelbase/math.tan(inner)+cfg.steering_track/2-.005:
            raise ValueError('계획 회전 반경이 보정된 조향 명령 한계보다 작습니다.')
    if max(cfg.steering_response_scale,cfg.reverse_steering_response_scale) > 2.:
        raise ValueError('실측 조향 응답 비율은 0보다 크고 2 이하여야 합니다.')
    if min(cfg.max_planning_turn_radius,cfg.entry_outward_penalty,cfg.steering_preview_time,cfg.visualization_period)<0 or (0<cfg.max_planning_turn_radius<cfg.turn_radius):
        raise ValueError('계획 최대 회전 반경과 바깥 우회 비용을 확인하세요.')
    if not 0<cfg.exit_arrival_yaw_tolerance<math.pi/2 or cfg.exit_planning_timeout<0:
        raise ValueError('출차 방향 허용 오차와 계산 시간을 확인하세요.')
    if cfg.preferred_turn_radius < 0 or min(cfg.max_entry_path_length,cfg.path_tracking_margin,cfg.exit_grid_margin)<0:
        raise ValueError('선호 회전 반경·진입 경로 길이·추종 여유는 0 이상이어야 합니다.')
    if cfg.max_turn_arc_length <= 0 or cfg.recovery_straight_distance < 0:
        raise ValueError('회전 호 길이는 0보다 크고 직선 후진 거리는 0 이상이어야 합니다.')
    if min(cfg.blind_reverse_distance,cfg.recovery_total_distance) < 0:
        raise ValueError('미관측 후진 허용 거리는 0 이상이어야 합니다.')
    if min(cfg.obstacle_min_cluster_size, cfg.small_cluster_filter_max_distance,cfg.pair_min_cluster_size) < 0:
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


class ParkingNode(StartSequence, Node):
    def __init__(self, **kwargs):
        super().__init__('parking_node', **kwargs)
        def parameter(name, default):
            return self.declare_parameter(name, default).value
        self.cfg = Settings(**{f.name: parameter(f.name, getattr(Settings(), f.name)) for f in fields(Settings)})
        validate_settings(self.cfg)
        self.steering_model=None
        self.steering_model_time=-math.inf
        self.steering_query=None
        self.steering_last_request=-math.inf
        self.steering_driver_node=parameter('steering_driver_node','/limo/limo_base')
        self.steering_client=self.create_client(GetParameters,self.steering_driver_node.rstrip('/')+'/get_parameters')
        self.base_frame = parameter('base_frame', 'base_link')
        self.lidar_frame = parameter('lidar_frame', 'laser_link')
        self.sensor_pose = (parameter('lidar_x', 0.103), parameter('lidar_y', 0.0), parameter('lidar_yaw', 0.0))
        required = parameter('recognition_frames', 2)
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
        if self.cfg.parking_center_adjustment and self.cfg.parking_center_adjustment+.015 > self.parking_position_tolerance+1e-9:
            raise ValueError('Parking center adjustment exceeds the configured arrival tolerance')
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
        self.health_path_hold_timeout = parameter('health_path_hold_timeout', 0.)
        if not math.isfinite(self.health_path_hold_timeout) or self.health_path_hold_timeout < 0:
            raise ValueError('Health path hold timeout must be nonnegative')
        self.health_path_tracking = False
        self.odom_frame = parameter('odom_frame', 'parking_odom' if self.localization_source == 'fused' else 'odom')
        self.odometry = PoseHistory(timeout=self.localization_timeout,
                                   max_std=parameter('localization_max_stddev', .5),
                                   max_yaw_std=parameter('localization_max_yaw_stddev', .5))
        self.odometry.samples=deque(maxlen=16)  # Small interpolation window; never a processing backlog.
        self.approach_reference = ApproachReference(self.localization_timeout)
        self.parking_calibration_stop = parameter('parking_calibration_stop', False)
        self.approach_calibration = ApproachCalibration(
            parameter('parking_calibration_timeout', 10.), self.localization_timeout)
        self.approach_calibration_event = None
        self.approach_calibration_future = None
        self.last_approach_ready_wall = -math.inf
        self.localization_preserve_timeout = parameter('localization_preserve_timeout', 0.)
        if not math.isfinite(self.localization_preserve_timeout) or self.localization_preserve_timeout < 0:
            raise ValueError('Localization preserve timeout must be nonnegative')
        self.localization_hold_since = None
        self.localization_hold_samples = 0
        self.search_enabled = parameter('search_enabled', False)
        self.search_speed = parameter('search_speed', .1)
        self.search_max_distance = parameter('search_max_distance', 1.5)
        self.search_timeout = parameter('search_timeout', 20.)
        self.search_resume_scans = parameter('search_resume_scans', 2)
        if not (0 < self.search_speed <= .2 and self.search_max_distance > 0 and self.search_timeout > 0
                and all(math.isfinite(v) for v in (self.search_speed,self.search_max_distance,self.search_timeout))):
            raise ValueError('Invalid parking search speed, distance or timeout')
        if isinstance(self.search_resume_scans,bool) or not isinstance(self.search_resume_scans,int) or self.search_resume_scans < 1:
            raise ValueError('search_resume_scans must be a positive integer')
        self.search_started = self.search_last_pose = None
        self.search_distance = 0.
        self.search_check_sequence = -1
        self.search_clear = False
        self.search_clear_count = 0
        self.search_check_pose = None
        self.search_block_detail = None
        self.search_target_phase = None
        self.search_stop_count = 0
        self.search_stop_stamp = None
        self.search_replan_after = None
        self.search_candidate_attempts = 0
        self.search_candidate_started = None
        self.search_excluded_targets = []
        self.fusion_health = None
        self.fusion_health_received = None
        self.localization_not_before = None
        self.last_localization_rejection = None
        self.localization_recovering = False
        self.localization_restart_count = 0
        self.last_localization_restart = None
        self.planning_cache_key = None
        self.planning_cache = None
        self.visual_path_cache_key = None
        self.visual_path_cache_blocked = False
        self.last_tick_duration = 0.
        self.last_control_visuals_wall=-math.inf
        self.last_control_visuals_key=None
        if (required < 1 or not all(math.isfinite(v) for v in
                (self.sign_timeout, self.scan_timeout, self.plan_period, self.speed,
                 self.lookahead, self.control_hz, self.hold_seconds, self.exit_heading_tolerance, self.deceleration, self.max_yaw_rate)) or
                min(self.sign_timeout, self.scan_timeout, self.plan_period, self.speed,
                    self.lookahead, self.control_hz, self.exit_heading_tolerance, self.deceleration, self.max_yaw_rate) <= 0 or self.hold_seconds < 0):
            raise ValueError('감지 횟수·수신 제한 시간·속도·제어 주기는 0보다 커야 하며 대기 시간은 음수일 수 없습니다.')
        self.activation = Activation(required, self.sign_timeout)
        self.sign_ignore_until = 0.  # monotonic seconds; retained across reset
        self.parking_sign_motion_guard = parameter('parking_sign_motion_guard', False)
        parameter('parking_sign_departure_ignore_sec', 0.)
        parameter('parking_sign_stop_ignore_sec', 0.)
        if any(k in self._parameter_overrides for k in ('parking_sign_departure_ignore_sec','parking_sign_stop_ignore_sec')):
            self.get_logger().warning('주차 고정 무시 시간 설정은 사용 중단되었습니다. parking_sign_set_gap_sec 묶음 판정을 사용합니다.')
        self.sign_motion = SignMotionGate(0.)
        self.sign_sets = ParkingSignSet(parameter('parking_sign_set_gap_sec', 1.0), required)
        self.sign_retry_latched = False
        self.parking_stop_require_lane_owner = parameter('parking_stop_require_lane_owner', False)
        self.parking_mux_source = self.parking_mux_source_wall = None
        self.exit_on_lane = parameter('exit_on_lane_reacquired', True)
        self.exit_lane_gate = ExitLaneGate(parameter('exit_lane_timeout_sec', .25),
                                           parameter('exit_lane_min_confidence', .1))
        self.exit_lane_handed_off = False
        self.parking_commands_released = False
        self.exit_stop_gate=ExitStopGate()
        self.follower = self.new_follower()
        self.tracker = self.new_tracker()
        self.scan, self.scan_header, self.scan_time = None, None, None
        self.scan_error = ''
        self.last_scan_stamp = None
        self.last_scan_received_stamp = None
        self.scan_not_before = None
        self.pending_scans = deque(maxlen=1)
        self.scan_sequence = 0
        self.scan_idle = False  # 11-19: 주차 표지 확정 전 스캔 처리 생략 중
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
        self.last_map_log_key = None
        self.last_map_log_wall = -math.inf
        self.last_recovery_stop = None
        self.path_hidden_by_slot = False
        self.last_planner_error = ''
        self.recovery_attempts = 0
        self.failed_plans = 0
        self.recovery_remaining = self.cfg.recovery_total_distance
        self.blind_reverse_remaining = self.cfg.blind_reverse_distance
        self.recovery_running = False
        self.recovery_lane_released = False
        self.recovery_stop_confirmations = 0
        self.observed_corridor = deque(maxlen=2000)
        # Hybrid A* is Python CPU work. Isolate it so its GIL cannot starve
        # timestamped sensor callbacks and the command/health watchdog.
        self.worker = ProcessPoolExecutor(max_workers=1, mp_context=multiprocessing.get_context('spawn'),
                                          initializer=planner_process_init)
        self.pending = None
        self.exit_pending = None
        self.async_scan_matching = parameter('async_scan_matching', False)
        self.scan_worker = ThreadPoolExecutor(max_workers=1, thread_name_prefix='parking-scan')
        self.scan_job = None
        self.latest_scan_job = None
        self.scan_job_duration = 0.
        self.prevent_active_retrigger = parameter('prevent_active_retrigger',False)
        self.retain_parking_control = parameter('retain_parking_control',False)
        self.manual_mode_guard = parameter('manual_mode_guard', False)
        self.control_mode_timeout = parameter('control_mode_timeout_sec', 2.)
        if not math.isfinite(self.control_mode_timeout) or self.control_mode_timeout <= 0:
            raise ValueError('차량 상태 제한 시간은 0보다 커야 합니다.')
        self.control_mode = self.control_mode_received = None
        self.manual_interrupted = False
        self.guard_reset = False
        self.approach_exhausted = False
        self.motion_stall_since = None
        self.motion_failed = False
        self.approach_check_sequence = -1
        self.approach_corridor_clear = False
        self.calibration_stop_samples = 0
        self.calibration_stop_stamp = None
        self.planner_ready_future = None
        if parameter('planner_prewarm', False):
            self.planner_ready_future = self.worker.submit(planner_warmup)

        self.memory_enabled = parameter('obstacle_memory_enabled', False)
        self.memory = ObstacleMemory()
        self.memory_marker_ids = set()
        self.last_memory_wall = -math.inf
        self.memory_pub = self.create_publisher(PointCloud2, '/parking/obstacle_memory', 1)
        self.obstacles_pub = self.create_publisher(MarkerArray, '/parking/obstacles', 1)
        self.obstacle_marker_count = 0
        self.target_pub = self.create_publisher(MarkerArray, '/parking/target', 1)
        self.slot_map_pub = self.create_publisher(MarkerArray, '/parking/slot_map', 1)
        self.roi_pub = self.create_publisher(MarkerArray, '/parking/candidate_roi', 1)
        self.path_pub = self.create_publisher(Path, '/parking/path', 1)
        self.active_pub = self.create_publisher(Bool, '/parking/active', 1)
        self.status_pub = self.create_publisher(String, '/parking/status', 1)
        self.guard_pub = self.create_publisher(String, '/parking/control_state', 1)
        if self.manual_mode_guard:
            from limo_msgs.msg import LimoStatus
            self.create_subscription(LimoStatus, '/limo_status', self.on_vehicle_status, 1)
        self.command_pub = self.create_publisher(AckermannDriveStamped,
                                               '/mission/parking/command_03', QoSProfile(depth=1))
        self.localization_pub = self.create_publisher(Odometry, '/parking/localization/odometry', 10)
        self.approach_pub = self.create_publisher(Odometry, '/parking/approach_odometry', 10)
        self.approach_ready_pub = self.create_publisher(Header, '/parking/approach_ready', 10)
        self.approach_calibration_client = (self.create_client(Trigger, '/localization/calibrate_gyro')
                                            if self.parking_calibration_stop else None)
        self.lane_handoff_pub = self.create_publisher(Header, '/parking/lane_handoff', 1)
        if self.exit_on_lane:
            self.create_subscription(Float32MultiArray, '/lane_fusion/target', self.on_exit_lane_target, 1)
            self.create_subscription(AckermannDriveStamped, '/mission/lane', self.on_exit_lane_command, 1)
        self.ldf_pub = self.create_publisher(Float32, '/parking/ldf', 1)
        self.lookahead_pub = self.create_publisher(MarkerArray, '/parking/lookahead', 1)
        if self.localization_source == 'fused':
            self.create_subscription(Header, '/parking/approach_start', self.on_approach_start, 10)
            self.create_subscription(Odometry, parameter('localization_topic', '/odometry/filtered'),
                                     self.on_localization, QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))
            self.create_subscription(String, '/localization/health', self.on_fusion_health,
                                     QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))
        # Launch profiles enable the floor gate; bare controller construction stays usable without a camera.
        self.floor_gate = None
        if parameter('white_roi_enabled', False):
            from .floor_gate import ParkingFloorGate
            self.floor_gate = ParkingFloorGate(self, self.start_from_floor)
        if self.parking_stop_require_lane_owner:
            self.create_subscription(String, '/mux/active_source', self.on_parking_mux_source,
                QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(Bool, '/yolo/parking_sign', self.on_sign, QoSProfile(depth=1))
        self.create_subscription(LaserScan, '/scan', self.on_scan, QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))
        self.setup_start_sequence()
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
        self.get_logger().info('충돌 검사: '+('점유 칸 전체와 실제 장애물' if self.cfg.block_occupied_slots else '관측·기억한 실제 장애물과 차체·안전 여유. 칸 영역은 목표 선택에만 사용합니다.'))
        self.get_logger().info(f'조향 추종 비율 {self.cfg.steering_gain:.2f}, 실측 응답 비율 {self.cfg.steering_response_scale:.2f}, '
                               f'조향 변화율 제한 {self.follower.max_steering_rate:.2f}rad/s (0이면 즉시 출력).')
        if self.cfg.require_steering_calibration:
            self.get_logger().info('주차 명령에만 조향 보정을 적용합니다. LIMO 드라이버의 조향 보정은 꺼져 있어야 합니다.')
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
        localization_attrs = {'localization_max_stddev':'max_std',
                              'localization_max_yaw_stddev':'max_yaw_std'}
        localization_values = {name:updates.get(name,getattr(self.odometry,attr))
                               for name,attr in localization_attrs.items()}
        unsupported = set(updates)-config_names-set(attrs)-set(localization_attrs)
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
            if cfg.parking_center_adjustment and cfg.parking_center_adjustment+.015 > values['parking_position_tolerance']+1e-9:
                raise ValueError('Parking center adjustment exceeds the configured arrival tolerance')
            if any(isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) or v < 0
                   for v in localization_values.values()):
                raise ValueError('위치·방향 표준편차 한계는 0 이상의 유한한 숫자여야 합니다. 0은 상한 해제입니다.')
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
        if any(name.startswith(('slot_','candidate_roi_')) for name in updates):
            self.slot_map=None
            if self.follower.goal is not None and self.follower.goal.slot_id>=0:
                self.follower.awaiting_target_path=True
        old_cfg,old_hz = self.cfg,self.control_hz
        self.cfg = cfg
        for name,attr in localization_attrs.items():
            setattr(self.odometry,attr,localization_values[name])
        for name,attr in attrs.items():
            setattr(self,attr,values[name])
        self.activation.required,self.activation.timeout = self.required_detections,self.sign_timeout
        self.follower.cfg = cfg
        if any(name.startswith('candidate_roi_') for name in updates) and self.follower.goal is not None:
            self.follower.awaiting_target_path=bool(self.follower.path)
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
        planning_changes = config_names-{'steering_gain','curvature_feedforward','steering_preview_time','visualization_period','max_steering_on_curves','control_preview_time'}
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
        follower = Follower(self.cfg, self.speed, self.lookahead, self.hold_seconds, self.deceleration,
                        self.max_yaw_rate, self.exit_heading_tolerance,
                        self.require_parking_heading, self.parking_position_tolerance, self.max_steering_rate,
                        self.min_turn_speed)
        follower.exit_planner=self.queue_exit_plan
        return follower

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
            if self.localization_not_before is not None and health['stamp']<self.localization_not_before:return
            if self.fusion_health is not None and health['stamp'] <= self.fusion_health['stamp']:
                return
            if self.start_phase != 'IDLE':
                self.fusion_health, self.fusion_health_received = health, time.monotonic()
                return
            if self.departure_active():
                # Diagnostic only after parking. The route is driven using
                # validated odometry and fresh collision scans, not this heartbeat.
                self.fusion_health = health
                self.fusion_health_received = time.monotonic()
                return
            if self.approach_calibration.pending and not self.activation.active:
                self.fusion_health = health
                self.fusion_health_received = time.monotonic()
                return
            now = self.now_seconds()
            age = now-health['stamp']
            if age < -.05:
                self.hold_localization(now, 'Fusion health is stale or future-dated')
                return
            if age > self.localization_timeout:
                # Do not replace a newer usable heartbeat or discard freshly
                # recovered EKF/scan buffers because a queued heartbeat is late.
                if not self.can_follow_saved_path(now):
                    self.wait_for_fusion_health(now, 'Fusion health is stale or future-dated')
                return
            if not health['ready']:
                reason='Fusion inputs not ready: '+str(health.get('reason') or 'Wheel/IMU inputs unavailable or gyro calibration not complete')
                self.hold_localization(self.now_seconds(),reason)
            self.fusion_health = health
            self.fusion_health_received = time.monotonic()
        except (ValueError, TypeError, KeyError, AttributeError):
            return

    def vehicle_automatic(self):
        return (not self.manual_mode_guard or (self.control_mode == 1
            and self.control_mode_received is not None
            and time.monotonic()-self.control_mode_received <= self.control_mode_timeout))

    def vehicle_wait_reason(self):
        if self.control_mode_received is None:
            return '차량 상태 미수신: /limo_status 확인'
        if self.control_mode != 1:
            return f'수동 모드: control_mode={self.control_mode}'
        age = time.monotonic()-self.control_mode_received
        return f'차량 상태 지연: {age:.1f}초 (허용 {self.control_mode_timeout:.1f}초)'

    def on_vehicle_status(self, msg):
        self.control_mode, self.control_mode_received = msg.control_mode, time.monotonic()
        if (self.manual_mode_guard and msg.control_mode != 1 and self.activation.active
                and not self.follower.exited):
            if not self.manual_interrupted:
                self.manual_interrupted = True
                self.epoch += 1
                self.follower.path = []
                self.follower.exited = False
                self.follower.parked = False
                self.get_logger().warning('수동 개입: 새 주차 감지 대기')
            self.publish_command(0., 0.)

    def publish_guard(self):
        if self.parking_commands_released:
            return
        phase = 'IDLE'
        ready = self.vehicle_automatic() and not self.manual_interrupted
        if self.manual_interrupted or self.motion_failed:
            phase = 'FAILED'
        elif self.parking_calibration_stop and self.approach_calibration.phase != 'IDLE':
            phase = 'HOLD' if self.approach_calibration.phase != 'READY' else 'APPROACH'
        floor_approach=self.floor_gate is not None and self.floor_gate.armed and not self.activation.active
        if floor_approach:
            self.floor_gate.update_approach()
            phase='FAILED' if self.floor_gate.failure else 'APPROACH'
            ready=ready and not self.floor_gate.failure
        if not self.manual_interrupted and not self.motion_failed and self.activation.active:
            phase = 'RELEASE' if self.follower.exited else 'DRIVE'
        if self.guard_reset and phase == 'IDLE':
            phase = 'RESET'
        if phase == 'DRIVE' and self.localization_source == 'fused' and self.departure_active():
            ready = bool(ready and self.tracker.healthy(self.now_seconds())
                and self.scan is not None and self.scan_time is not None
                and -SCAN_FUTURE_TOLERANCE <= self.now_seconds()-self.scan_time <= self.scan_timeout)
        elif phase == 'APPROACH' or (phase == 'DRIVE' and self.localization_source == 'fused'):
            ready = ready and self.fusion_health is not None and self.fusion_health_received is not None
            fresh = bool(ready and self.fusion_health.get('ready') is True
                and time.monotonic()-self.fusion_health_received <= self.localization_timeout
                and -.05 <= self.now_seconds()-self.fusion_health['stamp'] <= self.localization_timeout)
            ready = bool(ready and (fresh or (phase == 'DRIVE' and self.can_follow_saved_path(self.now_seconds()))))
        if self.planner_ready_future is not None and phase in ('APPROACH','DRIVE'):
            ready = ready and self.planner_ready_future.done() and self.planner_ready_future.exception() is None
        limit = self.floor_gate.approach_speed if floor_approach else .3
        if phase == 'APPROACH' and not floor_approach and self.approach_calibration.ready_at is not None:
            elapsed = time.monotonic()-self.approach_calibration.ready_at
            relative = (self.approach_reference.relative(self.odometry.samples[-1])
                if self.odometry.samples else None)
            distance = math.hypot(*relative[:2]) if relative is not None else 0.
            self.approach_exhausted = self.approach_exhausted or elapsed >= 20. or distance >= 3.
            if self.approach_exhausted:
                phase,ready = 'FAILED',False
                self.state,self.reason = 'APPROACH_LIMIT','Parking start line not confirmed within 20 s / 3 m; reset required'
            elif elapsed >= 10.:
                limit = .1
                if self.approach_check_sequence != self.scan_sequence:
                    self.approach_corridor_clear = False
                    if self.scan is not None:
                        checker=CollisionChecker(self.scan,replace(self.cfg,assume_unobserved_free=False),known_free_poses=[(0.,0.,0.)])
                        self.approach_corridor_clear=all(checker.free((i*.02,0.,0.)) for i in range(14))
                    self.approach_check_sequence=self.scan_sequence
                ready=ready and self.approach_corridor_clear and self.scan_time is not None and 0<=self.now_seconds()-self.scan_time<=self.scan_timeout
        if self.start_phase != 'IDLE':
            phase, ready = 'HOLD', False
        self.guard_pub.publish(String(data=json.dumps({'stamp':self.now_seconds(),
            'limit':limit,
            'attempt':self.attempt,'phase':phase,'ready':bool(ready)},separators=(',',':'))))

    def on_approach_start(self, msg):
        if self.floor_gate is not None and (self.activation.active or not self.floor_gate.armed or self.floor_gate.failure):
            return
        self.guard_reset = False

        stamp = msg.stamp.sec + msg.stamp.nanosec * 1e-9
        if (self.floor_gate is None and self.parking_calibration_stop and not self.activation.active
                and 0 <= self.now_seconds()-stamp <= self.localization_timeout
                and self.approach_calibration.start(stamp, time.monotonic())):
            self.approach_calibration_event = deepcopy(msg)
            self.approach_calibration_future = None
            self.get_logger().info('주차 접근 정지: 이번 감지에 대해 정지 자이로 보정을 1회 요청합니다.')
        if self.approach_reference.request(stamp, self.now_seconds()):
            self.get_logger().info('주차 접근 기준점 요청: 감속 시작 시각의 EKF 위치를 기다립니다.')
            self.resolve_approach_reference()
        elif self.approach_reference.reason:
            self.get_logger().warning(self.approach_reference.reason)

    def approach_can_start(self):
        return not self.approach_exhausted and (not self.parking_calibration_stop or self.approach_calibration.phase == 'READY')

    def begin_floor_approach(self):
        # An accepted new sign retires a completed/failed route, while keeping
        # the live sensors and coordinate frame intact until the white line.
        self.guard_reset=False
        self.parking_commands_released=False
        self.manual_interrupted=self.motion_failed=False
        self.motion_stall_since=None
        self.approach_exhausted=False
        if self.activation.active or self.follower.exited:
            self.epoch+=1
            for name in ('pending','exit_pending'):
                pending=getattr(self,name)
                if pending is not None:
                    pending[0].cancel()
                    setattr(self,name,None)
            self.activation.reset()
            self.follower=self.new_follower()
            self.tracked_goal=self.planned_goal=None
            self.had_control_path=False
        if self.approach_calibration_future is not None:
            self.approach_calibration_future.cancel()
        self.approach_calibration_future=self.approach_calibration_event=None
        self.approach_calibration=ApproachCalibration(self.approach_calibration.timeout,self.localization_timeout)
        self.approach_reference.reset()

    def poll_approach_calibration(self):
        if self.floor_gate is not None:
            return  # 시작 트랜잭션에서 보정·EKF 초기화를 함께 처리합니다.
        job = self.approach_calibration
        if not self.parking_calibration_stop:
            return
        wall, now = time.monotonic(), self.now_seconds()
        previous = job.phase
        health = self.fusion_health
        stationary_values = [health.get(k) for k in ('stamp','wheel_speed_m_s','wheel_yaw_rate_rad_s')] if health else []
        stationary = (len(stationary_values)==3 and all(isinstance(v,(int,float)) and not isinstance(v,bool) and math.isfinite(v) for v in stationary_values)
            and health is not None and self.fusion_health_received is not None
            and wall-self.fusion_health_received <= self.localization_timeout
            and 0 <= now-health.get('stamp', -math.inf) <= self.localization_timeout
            and abs(health.get('wheel_speed_m_s', math.inf)) < .01
            and abs(health.get('wheel_yaw_rate_rad_s', math.inf)) < .02)
        if job.pending and stationary and self.vehicle_automatic():
            if health['stamp'] != self.calibration_stop_stamp:
                self.calibration_stop_samples += 1
                self.calibration_stop_stamp = health['stamp']
        else:
            self.calibration_stop_samples = 0
        if (job.pending and not job.requested and self.calibration_stop_samples >= 2
                and self.approach_calibration_client.service_is_ready()):
            job.requested = True
            self.approach_calibration_future = self.approach_calibration_client.call_async(Trigger.Request())
        future = self.approach_calibration_future
        if future is not None and future.done():
            self.approach_calibration_future = None
            try:
                response = future.result()
                generation = json.loads(response.message).get('generation') if response.success else None
                job.response(response.success, generation, wall)
            except Exception:
                job.fail('Parking calibration request failed; reset required')
        sample = self.odometry.samples[-1] if self.odometry.samples and not self.odometry.discontinuity else None
        health = self.fusion_health
        if (self.fusion_health_received is None or wall-self.fusion_health_received > self.localization_timeout):
            health = None
        job.update(wall, now, health, sample)
        if job.phase == 'READY' and previous != 'READY':
            # Drop the approach's zero parking lease before lane resumes.
            # Keep the correlation stamp on approach_ready unchanged.
            self.lane_handoff_pub.publish(Header(stamp=self.get_clock().now().to_msg()))
        if job.phase == 'READY' and previous != 'READY' and self.approach_reference.origin is None:
            self.approach_reference.reset()
            self.approach_reference.request(sample.stamp, now)
            self.resolve_approach_reference()
        if job.phase == 'FAILED' and previous != 'FAILED':
            self.get_logger().warning('주차 접근 보정 실패: 자동 재시도하지 않습니다. 정지 상태에서 /parking/reset으로 재시도하세요.')
        if (job.phase == 'READY' and self.approach_calibration_event is not None
                and wall-job.ready_at <= 1. and wall-self.last_approach_ready_wall >= .1):
            self.approach_ready_pub.publish(self.approach_calibration_event)
            self.last_approach_ready_wall = wall

    def resolve_approach_reference(self):
        previous = self.approach_reference.reason
        if self.approach_reference.resolve(self.odometry, self.now_seconds()):
            self.get_logger().info('주차 접근 기준점 설정: 상대 위치·방향 계산을 시작합니다.')
        elif self.approach_reference.reason != previous and self.approach_reference.reason:
            self.get_logger().warning(self.approach_reference.reason)

    def on_localization(self, msg):
        if self.departure_active():
            self.lock_departure_localization()
        if self.departure_active() and self.departure_wheel:
            return  # 출차 위치는 휠 이동량으로 계산합니다.
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.localization_not_before is not None and stamp<self.localization_not_before:return
        now = self.now_seconds()
        if self.odometry.samples and stamp <= self.odometry.samples[-1].stamp:
            if not -.05 <= now-self.odometry.samples[-1].stamp <= self.localization_timeout:
                self.hold_localization(now, 'EKF odometry is stale or future-dated')
            return
        stale = not -.05 <= now-stamp <= self.localization_timeout
        previous_stale = (self.odometry.samples and
                          not -.05 <= now-self.odometry.samples[-1].stamp <= self.localization_timeout)
        if stale or previous_stale:
            self.hold_localization(now, 'EKF odometry is stale or future-dated')
            if stale or (self.localization_not_before is not None and stamp < self.localization_not_before):
                return
        q, p = msg.pose.pose.orientation, msg.pose.pose.position
        values = (q.x, q.y, q.z, q.w, p.x, p.y, *msg.pose.covariance)
        if (msg.header.frame_id != self.odom_frame or msg.child_frame_id != self.base_frame
                or not -.05 <= self.now_seconds()-stamp <= self.localization_timeout
                or not all(math.isfinite(v) for v in values)
                or abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1.) > .01):
            self.last_localization_rejection={'stamp':stamp,'age_s':self.now_seconds()-stamp,'frame':msg.header.frame_id,'child_frame':msg.child_frame_id}
            rejection=self.last_localization_rejection
            reason='Invalid EKF frame, timestamp, quaternion or covariance'
            self.restart_localization(now,reason)
            self.odometry.reason=reason
            self.last_localization_rejection=dict(rejection,reason=reason)
            return
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        accepted = self.odometry.add(OdomSample(stamp, (p.x, p.y, yaw), msg.twist.twist.linear.x,
                                               msg.twist.twist.angular.z,
                                               tuple(msg.pose.covariance[i] for i in (0, 7, 35))))
        if not accepted:
            self.last_localization_rejection={
                'stamp':stamp,'age_s':self.now_seconds()-stamp,'reason':self.odometry.reason,
                'variance':[msg.pose.covariance[i] for i in (0,7,35)],
                'xy_variance_limit':self.odometry.max_std**2 if self.odometry.max_std > 0 else None,
                'yaw_variance_limit':self.odometry.max_yaw_std**2 if self.odometry.max_yaw_std > 0 else None}
            rejection=self.last_localization_rejection
            self.restart_localization(now,rejection['reason'])
            self.odometry.reason=rejection['reason']
            self.last_localization_rejection=rejection
            return
        if accepted:
            if self.localization_hold_since is not None and stamp >= self.localization_hold_since:
                self.localization_hold_samples += 1
            self.last_localization_rejection=None
            self.resolve_approach_reference()
            relative = self.approach_reference.relative(self.odometry.samples[-1])
            if relative is not None and not self.odometry.discontinuity:
                output = deepcopy(msg)
                output.header.frame_id = 'parking_approach'
                output.pose.pose.position.x, output.pose.pose.position.y = relative[:2]
                output.pose.pose.position.z = 0.
                output.pose.pose.orientation.x = output.pose.pose.orientation.y = 0.
                output.pose.pose.orientation.z = math.sin(relative[2]/2)
                output.pose.pose.orientation.w = math.cos(relative[2]/2)
                output.pose.covariance = relative_covariance(msg.pose.covariance, self.approach_reference.origin[2])
                self.approach_pub.publish(output)
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

    def synchronization_status(self,now):
        stamp=self.odometry.samples[-1].stamp if self.odometry.samples else None
        return {'method':'scan_stamp_odometry_interpolation',
            'localization_recovering':self.localization_recovering,
            'health_path_tracking':self.health_path_tracking,
            'health_path_hold_timeout_s':self.health_path_hold_timeout,
            'localization_restart_count':self.localization_restart_count,
            'localization_hold_since':self.localization_hold_since,
            'last_localization_restart':self.last_localization_restart,
            'scan_age_s':None if self.scan_time is None else now-self.scan_time,
            'odom_age_s':None if stamp is None else now-stamp,
            'health_age_s':None if self.fusion_health is None else now-self.fusion_health['stamp'],
            'health_callback_age_s':None if self.fusion_health_received is None else time.monotonic()-self.fusion_health_received,
            'scan_to_latest_odom_s':None if stamp is None or self.scan_time is None else stamp-self.scan_time,
            'pending_scans':len(self.pending_scans),'scan_queue_capacity':self.pending_scans.maxlen,
            'odom_history_capacity':self.odometry.samples.maxlen,
            'latest_scan_received_age_s':None if self.last_scan_received_stamp is None else now-self.last_scan_received_stamp,
            'last_odom_rejection':self.last_localization_rejection}

    def departure_active(self):
        return self.follower.parked and not self.follower.exited

    def lock_departure_localization(self):
        if (self.localization_source != 'fused' or not self.departure_active()
                or self.tracker.odometry_only):
            return
        self.tracker.freeze_corrections()
        sample = self.tracker.sample
        if sample is not None:
            self.odometry.samples = deque([sample],maxlen=64)
            self.odometry.reason = ''
            self.odometry.discontinuity = False
            self.tracker.latched = False
            self.departure_wheel = True
            self.get_logger().info('탈출 후진: 휠 위치 계산 사용')
        # A pre-parking scan job must not overwrite the now-fixed transform.
        if self.scan_job is not None:
            self.scan_job[0].cancel()
            self.scan_job = None
        self.latest_scan_job = None
        if self.pending is not None:
            self.pending[0].cancel()
            self.pending = None

    def hold_departure(self, reason):
        # Do not throw away the goal, parked timestamp, trace, route, map, or
        # coordinate frame. A discontinuous odometry frame still requires reset.
        if self.activation.active:
            self.publish_command(0., 0.)
        self.state, self.reason = 'WAITING_LOCALIZATION', reason

    def hold_localization(self, now, reason):
        if self.start_phase != 'IDLE':
            self.reason = reason
            return
        if self.follower.exited:
            return
        if self.departure_active():
            self.hold_departure(reason)
            return
        established = getattr(self.tracker, 'origin', None) is not None
        if (not established or self.localization_preserve_timeout <= 0
                or self.odometry.discontinuity
                or (self.localization_hold_since is not None
                    and now-self.localization_hold_since > self.localization_preserve_timeout)):
            self.restart_localization(now, reason)
            return
        if self.localization_hold_since is None:
            self.localization_hold_since = now
            self.localization_hold_samples = 0
            self.epoch += 1
            if self.pending is not None:
                self.pending[0].cancel()
                self.pending = None
        if self.activation.active:
            self.publish_command(0., 0.)
        self.state, self.reason = 'WAITING_LOCALIZATION', reason

    def restart_localization(self, now, reason):
        if self.start_phase != 'IDLE':
            self.reason = reason
            return
        # Every fault discards ALL partial recovery data too. Keep the first
        # cutoff fixed during an outage so callback latency cannot starve recovery.
        if self.follower.exited:
            return
        if self.departure_active():
            self.tracker.valid, self.tracker.reason = False, reason
            self.hold_departure(reason)
            return
        self.localization_hold_since = None
        self.localization_hold_samples = 0
        first_fault = not self.localization_recovering
        rejection = None if first_fault else self.last_localization_rejection
        cutoff = now if self.localization_not_before is None else self.localization_not_before
        if first_fault:
            before = self.synchronization_status(now)
            self.last_localization_restart = dict(stamp=now, reason=reason,
                **{key:before[key] for key in ('scan_age_s','odom_age_s','health_age_s')})
            cutoff = now
        if self.activation.active:
            self.publish_command(0., 0.)
        budgets = (self.recovery_attempts, self.recovery_remaining, self.blind_reverse_remaining)
        search_budget = (self.search_started,self.search_distance)
        parked = self.follower.parked
        self.reset_motion()
        self.recovery_attempts, self.recovery_remaining, self.blind_reverse_remaining = budgets
        self.search_started,self.search_distance = search_budget
        self.follower.parked = parked
        self.odometry.samples.clear()
        self.odometry.discontinuity = False
        self.odometry.reason = rejection.get('reason','') if rejection else ''
        self.approach_reference.reset(cutoff)
        self.fusion_health = self.fusion_health_received = None
        self.last_localization_rejection = rejection  # Diagnostic metadata only; no pose is retained.
        self.scan_not_before = self.localization_not_before = cutoff
        self.planning_cache_key = self.planning_cache = None
        self.visual_path_cache_key = None
        self.visual_path_cache_blocked = False
        self.localization_recovering = True
        if first_fault:
            self.localization_restart_count += 1
            self.get_logger().warning('EKF 입력 오류: 이전 위치·스캔·매칭 기준·지도·경로·접근 기준점을 모두 폐기했습니다. 최신 정상 데이터로 다시 매칭합니다.')
        self.state, self.reason = 'WAITING_LOCALIZATION', reason

    def can_follow_saved_path(self, now):
        """A late heartbeat may use an existing route, never stale EKF or scans."""
        health = self.fusion_health
        if (self.health_path_hold_timeout <= 0 or self.localization_source != 'fused'
                or not self.activation.active or self.start_phase != 'IDLE'
                or self.motion_failed or self.manual_interrupted or self.follower.exited
                or self.departure_active() or not self.follower.path
                or self.localization_recovering or self.localization_hold_since is not None
                or health is None or health.get('ready') is not True
                or self.fusion_health_received is None):
            return False
        age, callback_age = now-health['stamp'], time.monotonic()-self.fusion_health_received
        if (not -.05 <= age <= self.health_path_hold_timeout
                or not 0 <= callback_age <= self.health_path_hold_timeout
                or (age <= self.localization_timeout and callback_age <= self.localization_timeout)):
            return False
        received = self.last_scan_received_stamp
        return bool(self.scan is not None and self.scan_time is not None and received is not None
            and -SCAN_FUTURE_TOLERANCE <= now-self.scan_time <= self.scan_timeout
            and -SCAN_FUTURE_TOLERANCE <= now-received <= min(self.scan_timeout,self.localization_timeout)
            and self.odometry.healthy(now) and self.tracker.healthy(now))

    def saved_path_collision(self, scan, pose):
        """Recheck the stopping corridor even in profiles without continuous replanning."""
        cfg = self.cfg
        checker = CollisionChecker(scan, replace(cfg, initial_clear_radius=cfg.max_range+1.))
        if not checker.free((0.,0.,0.)):
            return dict(checker.last_block_detail or {}, kind=checker.last_block)
        path = self.follower.path
        start = self.follower.index
        stop = min(len(path), start+30, self.follower.segment_end+1)
        if stop <= start:
            return None
        nearest = min(range(start,stop), key=lambda i: math.dist(path[i][:2],pose[:2]))
        transform = inverse(pose)
        velocity = max(abs(self.speed), self.follower.measured_speed)
        horizon = max(.25,velocity*cfg.control_preview_time+velocity**2/(2*self.deceleration))
        distance, previous = 0., (0.,0.,0.)
        for point in path[nearest:self.follower.segment_end+1]:
            local = compose(transform,point)
            length = math.dist(previous[:2],local[:2])
            covered = min(length,max(0.,horizon-distance))
            count = max(1,math.ceil(covered/cfg.collision_step))
            for i in range(1,count+1):
                fraction = covered*i/count/length if length>1e-9 else 1.
                sample = (previous[0]+fraction*(local[0]-previous[0]),
                          previous[1]+fraction*(local[1]-previous[1]),
                          wrap(previous[2]+fraction*wrap(local[2]-previous[2])))
                if not checker.free(sample):
                    return dict(checker.last_block_detail or {}, kind=checker.last_block)
            distance += length
            if distance >= horizon:
                break
            previous = local
        return None

    def wait_for_fusion_health(self, now, reason):
        self.health_path_tracking = False
        # A heartbeat delay does not invalidate a measured coordinate frame.
        # Keep partial recovery too, so fresh scans can finish matching.
        if (self.health_path_hold_timeout > 0 and not self.odometry.discontinuity
                and (not self.odometry.samples or self.odometry.healthy(now))):
            if self.activation.active:
                self.publish_command(0., 0.)
            self.state, self.reason = 'WAITING_LOCALIZATION', reason
            return
        self.hold_localization(now, reason)

    def localization_ready(self, now):
        self.health_path_tracking = False
        if self.localization_source == 'scan':
            return True
        if self.departure_active():
            self.lock_departure_localization()
            if not self.tracker.healthy(now):
                self.hold_departure(self.tracker.reason)
                return False
            return True
        health = self.fusion_health
        if health is None or self.fusion_health_received is None:
            self.reason = 'Fusion health unavailable'
            self.wait_for_fusion_health(now,self.reason)
            return False
        if (time.monotonic()-self.fusion_health_received > self.localization_timeout
                or not -.05 <= now-health['stamp'] <= self.localization_timeout):
            if self.can_follow_saved_path(now):
                self.health_path_tracking = True
                return True
            self.reason = 'Fusion health is stale or future-dated'
            self.wait_for_fusion_health(now, self.reason)
            return False
        if not health['ready']:
            self.reason = 'Fusion inputs not ready: '+str(health.get('reason') or
                           'Wheel/IMU inputs unavailable or gyro calibration not complete')
            self.hold_localization(now,self.reason)
            return False
        if not self.tracker.healthy(now):
            self.reason = self.tracker.reason
            if self.reason == 'EKF odometry is stale or future-dated':
                self.hold_localization(now, self.reason)
            return False
        if self.localization_hold_since is not None:
            if now-self.localization_hold_since > self.localization_preserve_timeout:
                self.restart_localization(now, 'Localization outage exceeded preserve timeout')
                return False
            if (self.localization_hold_samples < 2 or self.tracker.last_stamp is None
                    or self.tracker.last_stamp < self.localization_hold_since):
                self.reason = 'Waiting for fresh odometry and scan after localization hold'
                return False
            self.localization_hold_since = None
            self.last_plan_wall = -math.inf
        if self.localization_recovering:
            self.localization_recovering = False
            self.get_logger().info('최신 융합 위치·라이다 재매칭 완료: 주차 경로를 다시 계산합니다.')
        return True

    def clear_visuals(self, clear_obstacles=True):
        held = bool(self.follower.path) and (self.follower.entry_locked or self.cfg.continuous_replanning or self.follower.parked)
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

    def queue_exit_plan(self,trace,pose,targets,scan,cfg):
        grid=self.follower.goal.parking_grid if self.follower.goal is not None else None
        # Reuse the accepted arc/straight entry corridor when the measured join
        # is feasible. A fit through a slightly different stop pose can cut a cone.
        route=reverse_entry_route(self.follower.entry_path,pose,scan,cfg)
        if not route:
            route=anchored_reverse_entry(self.follower.entry_path,pose,scan,cfg)
        if route and grid is not None and not outside_grid(route[-1],grid,cfg):
            route=[]
        if route and arc_then_straight(route):
            if self.exit_pending is not None:
                self.exit_pending[0].cancel()
                self.exit_pending=None
            self.get_logger().info('탈출 경로: 진입 곡선·직선 재사용')
            return route,''
        if self.exit_pending is None:
            future=self.worker.submit(arc_straight_departure,list(trace),pose,scan,cfg)
            self.exit_pending=(future,self.epoch,self.follower,pose)
        return None

    def poll_exit_plan(self):
        if self.exit_pending is None or not self.exit_pending[0].done():return
        future,epoch,follower,origin=self.exit_pending
        self.exit_pending=None
        if epoch!=self.epoch or follower is not self.follower or not self.activation.active or follower.exited or follower.exit_retry_exhausted:return
        try:route,reason=future.result()
        except Exception as exc:
            self.get_logger().error('출차 경로 계산 오류: '+str(exc))
            route,reason=[],'planner_failure'
        if route:
            pose=self.tracker.pose
            if not self.tracker.valid:
                follower.accept_exit_route([],'observation_unavailable')
                return
            if math.dist(origin[:2],pose[:2])>.05 or abs(wrap(origin[2]-pose[2]))>.15:
                route,reason=[],'pose_changed'
            elif self.cfg.exit_collision_check_enabled:
                source=self.planning_scan()
                if source is None:
                    follower.accept_exit_route([],'observation_unavailable')
                    return
                checker=CollisionChecker(source,replace(self.cfg,
                    clearance=self.cfg.clearance+self.cfg.exit_path_tracking_margin))
                if first_path_collision(checker,[(0.,0.,0.)]+[compose(inverse(pose),p) for p in route],self.cfg.collision_step):
                    route,reason=[],'map_changed'
        follower.accept_exit_route(route,reason,now=self.now_seconds())

    def reset_motion(self):
        self.health_path_tracking = False
        if self.localization_source == 'fused':
            self.odometry.samples = deque(self.odometry.samples,maxlen=16)
        self.departure_wheel = False
        self.yellow_segments = []
        self.yellow_pending = None
        self.latest_scan_job = None
        self.search_started = self.search_last_pose = None
        self.search_distance = 0.
        self.search_check_sequence = -1
        self.search_clear = False
        self.search_clear_count = 0
        self.search_check_pose = None
        self.search_block_detail = None
        self.search_target_phase = None
        self.search_stop_count = 0
        self.search_stop_stamp = None
        self.search_replan_after = None
        self.search_candidate_attempts = 0
        self.search_candidate_started = None
        self.search_excluded_targets = []
        self.localization_hold_since = None
        self.localization_hold_samples = 0
        self.exit_lane_gate.reset()
        self.exit_stop_gate.reset()
        self.exit_lane_handed_off = False
        self.parking_commands_released = False
        if self.exit_pending is not None:
            self.exit_pending[0].cancel()
            self.exit_pending=None
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
        self.last_scan_received_stamp = None
        self.scan_not_before = None
        self.localization_not_before = None
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
        self.last_map_log_key = None
        self.last_map_log_wall = -math.inf
        self.last_recovery_stop = None
        self.path_hidden_by_slot = False
        self.last_planner_error = ''
        self.recovery_attempts = 0
        self.failed_plans = 0
        self.recovery_remaining = self.cfg.recovery_total_distance
        self.blind_reverse_remaining = self.cfg.blind_reverse_distance
        self.recovery_running = False
        self.recovery_lane_released = False
        self.recovery_stop_confirmations = 0
        self.observed_corridor.clear()
        self.clear_visuals()
        self.roi_pub.publish(MarkerArray(markers=[Marker(action=Marker.DELETEALL)]))

    def reset_state(self):
        self.localization_recovering = False
        self.last_localization_restart = None
        self.approach_reference.reset(self.now_seconds())
        self.reset_motion()
        self.activation.reset()
        if self.floor_gate is not None:
            self.floor_gate.reset()
        self.state = 'WAITING_SIGN'
        self.active_pub.publish(Bool(data=False))

    def on_reset(self, request, response):
        if self.start_future is not None:
            self.start_future.cancel()
        self.start_future = None
        self.start_phase = 'IDLE'
        self.ready_until = 0.
        self.manual_interrupted = False
        self.motion_failed = False
        self.motion_stall_since = None
        self.guard_reset = True
        self.approach_exhausted = False
        self.calibration_stop_samples = 0
        retry_event = self.approach_calibration_event if self.approach_calibration.phase == 'FAILED' else None
        if self.approach_calibration_future is not None:
            self.approach_calibration_future.cancel()
        self.approach_calibration_future = None
        self.approach_calibration = ApproachCalibration(self.approach_calibration.timeout, self.localization_timeout)
        self.approach_calibration_event = retry_event
        if retry_event is not None:
            stamp = retry_event.stamp.sec+retry_event.stamp.nanosec*1e-9
            self.approach_calibration.start(stamp, time.monotonic())
        # Explicit reset is also allowed to release the mux after a final stop.
        if self.activation.active and not self.follower.exited:
            self.publish_command(0.0, 0.0)
        self.reset_state()
        response.success = True
        response.message = f'주차 미션을 초기화했습니다. 연속 시작 신호 {self.required_detections}회를 기다립니다.'
        self.get_logger().info(response.message)
        return response

    def sync_sign_retry(self):
        if not self.parking_sign_motion_guard:
            return
        exhausted=(self.activation.active and self.search_started is not None
                   and not self.follower.path and (self.search_distance>=self.search_max_distance
                   or self.now_seconds()-self.search_started>=self.search_timeout))
        failed=bool((self.floor_gate is not None and self.floor_gate.failure)
                    or self.motion_failed or self.manual_interrupted
                    or self.follower.exit_retry_exhausted or self.follower.exited or exhausted)
        if failed and not self.sign_retry_latched:
            self.sign_motion.require_departure()
        self.sign_retry_latched=failed

    def on_sign(self, msg):
        # Always track ignored true messages, before any mission/owner guard.
        confirmed=self.sign_sets.receive(msg.data,time.monotonic(),self.required_detections)
        if self.floor_gate is not None:
            self.floor_gate.update_approach()
            self.floor_gate.confirmation.count=self.sign_sets.count
        else:
            self.activation.count=self.sign_sets.count
        self.sync_sign_retry()
        if not confirmed:return
        if self.floor_gate is not None and self.floor_gate.armed and not self.floor_gate.failure:return
        exhausted=(self.activation.active and self.search_started is not None and not self.follower.path
                   and (self.search_distance>=self.search_max_distance or self.now_seconds()-self.search_started>=self.search_timeout))
        retry=bool(self.manual_interrupted or self.motion_failed or self.follower.exit_retry_exhausted or exhausted)
        if self.activation.active and not self.follower.exited and not retry:return
        if not self.accepts_parking_start():return
        if not self.sign_retry_latched and not self.parking_stop_line_allowed():return
        if self.parking_sign_motion_guard and not self.sign_motion.allows(time.monotonic()):return
        if time.monotonic() < self.sign_ignore_until:return
        if self.floor_gate is not None:
            self.floor_gate.on_confirmed_sign()
            return
        was_active=self.activation.active
        self.activation.restart(self.now_seconds())
        self.begin_parking(was_active,self.now_seconds())

    def on_parking_mux_source(self, msg):
        self.parking_mux_source, self.parking_mux_source_wall = msg.data, time.monotonic()

    def parking_stop_line_allowed(self):
        if not self.parking_stop_require_lane_owner:
            return True
        return (self.parking_mux_source == 'lane' and self.parking_mux_source_wall is not None
                and 0 <= time.monotonic()-self.parking_mux_source_wall <= .3)

    def start_from_floor(self):
        if not self.parking_stop_line_allowed():
            return
        if self.parking_sign_motion_guard and not self.sign_motion.allows(time.monotonic()):
            return
        self.request_parking_start()

    def begin_parking(self, was_active, mission_now):
        if was_active and (self.had_control_path or self.follower.path):
            self.publish_command(0.0, 0.0)
        self.reset_motion()
        if was_active:
            # Ignore queued scans acquired before the new start signal.
            self.scan_not_before = mission_now
            self.localization_not_before = mission_now
            self.odometry.samples.clear()
            self.odometry.discontinuity=False;self.odometry.reason=''
            self.fusion_health=None;self.fusion_health_received=None
        self.attempt += 1
        # Registration origin is the first scan acquired after activation.
        self.state, self.reason = 'WAITING_SCAN', ''
        self.active_pub.publish(Bool(data=True))
        self.get_logger().info(f'주차 미션 시작: {self.attempt}번째 시도. 최신 센서와 경로를 확인합니다.')

    def scan_processing_needed(self):
        """11-19: before the parking sign is confirmed nothing consumes the scan results (the start
        sequence replaces the tracker, clears odometry and ignores earlier scans at SETPOSE), so idle
        scans are skipped to save CPU. The subscription stays alive: resuming costs one scan (<= 0.1 s)."""
        return (self.floor_gate is None or self.floor_gate.armed or self.activation.active
                or self.start_phase != 'IDLE' or self.departure_active())

    def on_scan(self, msg):
        if not self.scan_processing_needed():
            self.scan_idle = True
            return
        if self.scan_idle:
            # Resume from a fresh tracker: its keyframe/offset from before the idle period are stale.
            self.scan_idle = False
            self.pending_scans.clear()
            if self.scan_job is None:
                self.tracker = self.new_tracker()
            self.get_logger().info('주차 표지 확인: LiDAR 스캔 처리 재개')
        now = self.now_seconds()
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_scan_received_stamp is not None and stamp<self.last_scan_received_stamp:return
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
        self.last_scan_received_stamp=stamp
        if self.localization_source == 'fused' and self.odometry.at(stamp) is None:
            # DDS callbacks for the two topics need not arrive in timestamp
            # order. Wait for actual matching EKF samples, without discarding
            # an accepted route or extrapolating through an input outage.
            if not any(m.header.stamp == msg.header.stamp for m in self.pending_scans):
                self.pending_scans.append(msg)
            return
        if self.async_scan_matching:
            if self.scan_job is not None:
                self.latest_scan_job = msg
                return
            tracker = copy(self.tracker)
            if self.localization_source == 'fused':
                tracker.history = copy(self.odometry)
                tracker.history.samples = deque(self.odometry.samples, maxlen=self.odometry.samples.maxlen)
            sensor_pose = (0.,0.,0.) if msg.header.frame_id == self.base_frame else self.sensor_pose
            raw = Scan(msg.ranges,msg.angle_min,msg.angle_increment,msg.range_min,msg.range_max,sensor_pose,self.cfg.max_range)
            raw.job_stamp = stamp
            future = self.scan_worker.submit(scan_match_job,tracker,raw,self.cfg,
                self.activation.active or self.localization_source == 'fused')
            self.scan_job = (future,self.epoch,msg,time.monotonic())
            return
        self.last_scan_stamp = stamp
        self.pending_scans.clear()
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
        matched = not (self.activation.active or self.localization_source == 'fused') or self.tracker.update(raw_scan.points, stamp=stamp)
        self.accept_scan(msg,raw_scan,clusters,matched)

    def poll_scan_match(self):
        if self.scan_job is None or not self.scan_job[0].done():
            return
        future,epoch,msg,started = self.scan_job
        self.scan_job = None
        self.scan_job_duration = time.monotonic()-started
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        try:
            tracker,raw,scan,clusters,matched = future.result()
        except Exception as exc:
            self.get_logger().error('Scan worker failed: '+str(exc))
        else:
            if epoch == self.epoch and -.05 <= self.now_seconds()-stamp <= self.scan_timeout:
                if self.localization_source == 'fused':tracker.history = self.odometry
                self.tracker = tracker
                self.scan_error=''
                self.scan,self.scan_time,self.scan_header = scan,stamp,msg.header
                self.last_scan_stamp = stamp
                self.scan_sequence += 1
                if not (self.memory_enabled and self.activation.active):self.publish_obstacles(clusters)
                self.accept_scan(msg,raw,clusters,matched)
        latest,self.latest_scan_job = self.latest_scan_job,None
        if latest is not None:self.on_scan(latest)

    def accept_scan(self,msg,raw_scan,clusters,matched):
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        was_tracking = self.tracker.valid
        if (self.activation.active or self.localization_source == 'fused') and not matched:
            self.state, self.reason = 'TRACKING_LOST', f'Scan matching failed: {self.tracker.reason}; retry required'
            if was_tracking:
                self.get_logger().warning(reason_text(self.reason))
            if not (self.follower.entry_locked or self.cfg.continuous_replanning or self.departure_active()):
                self.follower.path = []
            self.path_pub.publish(self.path_message(self.visible_path()
                                                   if self.follower.entry_locked or self.cfg.continuous_replanning or self.departure_active() else []))
        if (self.memory_enabled and self.activation.active and self.tracker.valid
                and self.start_phase in ('IDLE','POSE_WAIT')):
            self.memory.update(raw_scan,self.scan,self.tracker.pose,stamp,self.cfg,groups=clusters)
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
        def key():
            layout=self.slot_map
            geometry=(id(layout),tuple(layout['origin']),layout['yaw'],layout['cell_width'],layout['cell_depth'],layout['border'],
                      tuple((c['id'],c['occupied'],tuple(c['center'])) for c in layout['cells'])) if layout else None
            return (id(self.scan),self.scan_sequence,self.memory.stamp,tuple(self.tracker.pose),
                    id(self.cfg),geometry,self.activation.active)
        current=key()
        if current==self.planning_cache_key:return self.planning_cache
        scan=self.scan
        if (self.memory_enabled and self.activation.active and self.scan is not None
                and self.memory.stamp is not None
                and (self.memory.stamp==self.scan_time or self.cfg.recovery_uses_memory_map)):
            scan=self.memory.snapshot(self.scan,self.tracker.pose,self.cfg)
        if (scan is not None and self.slot_map is not None and self.slot_map.get('locked')
                and not self.departure_active()):
            from .core import Cluster
            from .enclosed import physical_objects
            from .slot_map import fit_slot_map
            world=[Cluster([compose(self.tracker.pose,(*p,0.))[:2] for p in c.points],c.track_id) for c in candidate_view(scan,self.cfg,self.tracker.pose).candidate_clusters(self.cfg)]
            objects=[(ids,c) for ids,c in physical_objects(world,self.cfg,right_only=False)
                     if self.cfg.min_cluster_size<=max(c.bounds[1]-c.bounds[0],c.bounds[3]-c.bounds[2])<=self.cfg.max_cluster_size]
            self.slot_map=fit_slot_map(objects,self.cfg,self.slot_map,
                sensor_origin=compose(self.tracker.pose,self.sensor_pose)[:2])
        if scan is not None:
            scan = copy(scan)
            transform = inverse(self.tracker.pose)
            scan.yellow_segments = [(compose(transform,(*a,0.))[:2],compose(transform,(*b,0.))[:2],side)
                                    for a,b,side in self.yellow_segments]
        self.planning_cache=with_occupied_slots(scan,self.slot_map,self.tracker.pose,self.cfg) if scan is not None else None
        self.planning_cache_key=key()
        return self.planning_cache

    def publish_control_visuals(self):
        wall=time.monotonic()
        key=(id(self.follower.path),self.follower.parked,self.follower.exiting,self.state)
        if wall-self.last_control_visuals_wall<self.cfg.visualization_period and key==self.last_control_visuals_key:return
        self.path_pub.publish(self.path_message(self.visible_path()))
        if self.yellow_segments:
            marker = self.marker('yellow_boundary', 0, Marker.LINE_LIST)
            marker.scale.x = .02
            marker.color.r = marker.color.g = marker.color.a = 1.
            transform = inverse(self.tracker.pose)
            marker.points = [Point(x=float(q[0]),y=float(q[1]),z=.03)
                for a,b,_ in self.yellow_segments for q in (compose(transform,(*a,0.)),compose(transform,(*b,0.)))]
            self.roi_pub.publish(MarkerArray(markers=[marker]))
        self.publish_target()
        self.publish_slot_map()
        self.publish_lookahead()
        self.last_control_visuals_wall=wall
        self.last_control_visuals_key=key

    def visible_path(self):
        path=self.follower.local_path(self.tracker.pose)
        self.path_hidden_by_slot=False
        if not self.cfg.block_occupied_slots:return path
        scan=self.planning_scan()
        if scan is not None and path:
            layout=self.slot_map
            source=self.follower.displayed_path()
            signature=(tuple(tuple(p) for p in source),id(self.cfg),
                       tuple(layout['origin']),layout['yaw'],layout['cell_width'],layout['cell_depth'],layout['border'],
                       tuple(c['id'] for c in layout['cells'] if c['occupied'])) if layout else None
            checker=None
            if signature is None or signature!=self.visual_path_cache_key:
                checker=CollisionChecker(scan,self.cfg)
                self.visual_path_cache_blocked=bool(first_occupied_path_collision(checker,path,self.cfg.collision_step))
                self.visual_path_cache_key=signature
            if self.visual_path_cache_blocked:
                checker=checker or CollisionChecker(scan,self.cfg)
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
        # The ROS setter otherwise validates every byte in Python. Supplying
        # its native byte array avoids that work without changing the payload.
        from array import array
        msg.data = array('B',b''.join(struct.pack('<fff',x,y,.04) for x,y in points))
        self.memory_pub.publish(msg)

    def publish_memory(self):
        self.publish_candidate_roi()
        if not (self.memory_enabled and self.activation.active and self.tracker.valid):
            return
        wall = time.monotonic()
        if wall-self.last_memory_wall < max(.1,self.cfg.visualization_period):
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

    def publish_candidate_roi(self):
        if not self.activation.active or not self.cfg.candidate_roi_enabled:
            self.roi_pub.publish(MarkerArray(markers=[Marker(action=Marker.DELETEALL)]))
            return
        if not self.tracker.valid:return
        cfg=self.cfg;transform=inverse(self.tracker.pose)
        corners=[(cfg.candidate_roi_forward_min,-cfg.candidate_roi_right_min),
                 (cfg.candidate_roi_forward_max,-cfg.candidate_roi_right_min),
                 (cfg.candidate_roi_forward_max,-cfg.candidate_roi_right_max),
                 (cfg.candidate_roi_forward_min,-cfg.candidate_roi_right_max)]
        marker=self.marker('candidate_roi',0,Marker.LINE_STRIP)
        marker.lifetime.sec=marker.lifetime.nanosec=0
        marker.scale.x=.012;marker.color.r,marker.color.g,marker.color.b=.2,1.,.3
        for x,y in corners+[corners[0]]:
            px,py,_=compose(transform,(x,y,0.));marker.points.append(Point(x=px,y=py,z=.03))
        self.roi_pub.publish(MarkerArray(markers=[marker]))

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
        for index,body in enumerate(layout.get('object_boxes',())):
            outline=self.marker('registered_limo',index,Marker.LINE_STRIP)
            outline.scale.x=.014;outline.color.g=.85;outline.color.b=1.
            outline.points=[Point(x=float(q[0]),y=float(q[1]),z=.08)
                for q in (compose(transform,(*point,0.)) for point in body['corners'])]
            markers.append(outline)
            center=self.marker('registered_limo_center',index,Marker.SPHERE)
            center.pose.position.x,center.pose.position.y,_=compose(transform,(*body['center'],0.))
            center.pose.position.z=.08
            center.scale.x=center.scale.y=center.scale.z=.03
            center.color.g=1.
            markers.append(center)
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
        if self.parking_commands_released:
            return
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base_frame
        command_angle = parking_steering_command(steering, self.cfg) if abs(speed) >= 1e-6 else 0.0
        msg.drive.speed, msg.drive.steering_angle = float(speed), float(command_angle)
        self.command_pub.publish(msg)
        if (self.exit_on_lane and speed < 0 and self.follower.parked and self.follower.exiting
                and not self.follower.recovering and not self.follower.exited):
            stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            self.exit_lane_gate.begin(stamp, time.monotonic())

    def on_exit_lane_target(self, message):
        self.exit_lane_gate.observe_target(message.data, time.monotonic())
        self.try_exit_lane_handoff()

    def on_exit_lane_command(self, message):
        stamp = message.header.stamp
        drive = message.drive
        if (stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000
                or not all(math.isfinite(v) for v in (drive.speed, drive.steering_angle,
                    drive.steering_angle_velocity, drive.acceleration, drive.jerk))):
            self.exit_lane_gate.command = None
            return
        self.exit_lane_gate.observe_command(message.drive.speed, message.drive.steering_angle,
                                           stamp.sec * 1_000_000_000 + stamp.nanosec, time.monotonic())
        self.try_exit_lane_handoff()

    def release_parking_commands(self):
        if not self.follower.exited or self.parking_commands_released:
            return
        self.epoch += 1
        for name in ('pending','exit_pending'):
            pending=getattr(self,name)
            if pending is not None:
                pending[0].cancel()
                setattr(self,name,None)
        if not self.exit_lane_handed_off:
            # One final stop precedes the stamped release; thereafter even
            # stale mode/health callbacks may not republish parking commands.
            self.publish_command(0.,0.)
            self.lane_handoff_pub.publish(Header(stamp=self.get_clock().now().to_msg(),frame_id=self.base_frame))
        self.guard_pub.publish(String(data=json.dumps(dict(stamp=self.now_seconds(),limit=.3,
            attempt=self.attempt,phase='RELEASE',ready=True),separators=(',',':'))))
        self.parking_commands_released=True
        self.exit_lane_handed_off=True
        self.had_control_path=False
        self.follower.path=[]
        self.follower.entry_path=[]
        self.follower.exit_path=[]
        self.state,self.reason='EXITED',''
        if not self.parking_sign_motion_guard:
            self.sign_ignore_until=max(self.sign_ignore_until,time.monotonic()+10.)
        if self.floor_gate is not None:self.floor_gate.reset()
        self.active_pub.publish(Bool(data=False))
        self.clear_visuals()
        self.get_logger().info('곡선 후진 완료: parking 명령 종료' if self.cfg.exit_curve_only else '탈출 완료: parking 명령 종료')

    def try_exit_lane_handoff(self):
        if (not self.exit_on_lane or not self.activation.active or self.exit_lane_handed_off
                or not self.follower.parked or not self.follower.exiting or self.follower.recovering
                or self.follower.exited):
            return
        now = self.now_seconds()
        if not self.tracker.valid:
            return
        if self.scan_time is None or not -SCAN_FUTURE_TOLERANCE<=now-self.scan_time<=self.scan_timeout:
            return
        if self.localization_source == 'fused':
            if not self.localization_ready(now):
                return
            pose = self.tracker.current_pose(now)
        else:
            pose = self.tracker.pose
        wall=time.monotonic()
        eligible=(self.exit_lane_gate.ready(self.get_clock().now().nanoseconds,wall)
            and self.follower.exit_curve_complete(pose))
        if self.cfg.exit_curve_handoff:
            sample=self.follower.previous_sample
            stationary=(sample is not None and 0<=now-sample[1]<=min(.2,self.scan_timeout)
                and math.isfinite(self.follower.measured_speed) and 0<=self.follower.measured_speed<=.03)
            action=self.exit_stop_gate.update(eligible,stationary,wall)
            self.follower.exit_handoff_hold=action=='hold'
            if action=='hold':
                self.state,self.reason='EXIT_LANE_STOPPING',''
                self.publish_command(0.,0.)
                return
            if action!='release':return
        elif not eligible:return
        # End the route permanently, including any asynchronous planning result.
        self.epoch += 1
        for name in ('pending', 'exit_pending'):
            pending = getattr(self, name)
            if pending is not None:
                pending[0].cancel()
                setattr(self, name, None)
        self.follower.exited = True
        self.follower.path = []
        self.follower.entry_path = []
        self.follower.exit_path = []
        self.follower.stop('EXITED', self.now_seconds())
        self.exit_lane_handed_off = True
        self.state, self.reason = 'EXITED', ''
        if self.floor_gate is not None:
            self.floor_gate.reset()
        if not self.parking_sign_motion_guard:
            self.sign_ignore_until = max(self.sign_ignore_until, time.monotonic() + 10.)
        # The stop precedes the stamped release. A delayed stop/drive packet
        # cannot reclaim the mux after that release.
        self.publish_command(0., 0.)
        self.had_control_path = False
        release = Header(stamp=self.get_clock().now().to_msg(), frame_id=self.base_frame)
        self.lane_handoff_pub.publish(release)
        self.active_pub.publish(Bool(data=False))
        self.clear_visuals()
        self.get_logger().info('곡선 후진 완료·차선 재인식: 남은 직선 후진/전진 경로를 생략하고 lane으로 제어권을 반환합니다.')

    def poll_plan(self):
        if self.can_follow_saved_path(self.now_seconds()):
            return  # Keep the accepted route until the heartbeat is fresh again.
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
        if (epoch != self.epoch or not self.activation.active or self.follower.exited or not self.tracker.valid
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
        if self.now_seconds() - started > self.scan_timeout + self.cfg.planning_timeout:
            if not self.follower.path:
                self.reason = 'Planner result expired before acceptance; check planning load'
            return
        if self.search_target_phase == 'STOPPING':
            return
        if self.search_target_phase == 'PLANNING':
            if self.search_stationary_stamp(self.now_seconds()) is None:
                self.search_target_phase = 'STOPPING'
                self.search_stop_count = 0
                self.search_stop_stamp = None
                self.search_replan_after = None
                return
            if started < self.search_replan_after:
                return  # Never use a route or target calculated before the stop.
        self.planning_info = diagnostics
        if self.cfg.slot_map_enabled and 'enclosure_objects' in diagnostics:
            layout = diagnostics.get('slot_map')
            objects = diagnostics['enclosure_objects']
            matched = bool(layout and layout.get('locked'))
            hypotheses = layout.get('hypotheses',0) if layout else 0
            key = (objects,matched,hypotheses)
            wall = time.monotonic()
            if key != self.last_map_log_key and wall-self.last_map_log_wall >= 1.:
                text = (f'주차 빈 공간 후보 {diagnostics.get("candidates",0)}개: 주변 장애물 간격으로 인식' if diagnostics.get('slot_map_fallback') else
                        f'주차 지도 매칭 완료: 객체 {objects}개' if matched else
                        f'주차 지도 대기: 객체 {objects}개, 배치 후보 {hypotheses}개')
                self.get_logger().info(text)
                self.last_map_log_key, self.last_map_log_wall = key, wall
        if self.recovery_lane_released:
            diagnostics['recovery_paused_for_lane']=True
            diagnostics['recovery_limit_reached']=False
            if diagnostics.get('reason_code')=='recovery_ready':return
        if self.cfg.slot_map_enabled and diagnostics.get('slot_map') is not None:
            self.slot_map = diagnostics['slot_map']
        # Transform the captured scan frame to the mission origin. Once the
        # final entry path is accepted, keep it through arrival confirmation.
        if 'selected_pair' in diagnostics:
            self.selected_pair = [dict(p,center=compose(origin,(*p['center'],0.))[:2])
                                  for p in diagnostics['selected_pair']]
            self.selected_track_ids = {p['track_id'] for p in self.selected_pair if p.get('track_id') is not None}
        elif self.follower.goal is None:
            self.selected_pair = []
            self.selected_track_ids = set()
        if (diagnostics.get('goal_invalidated_by_slot_map') or diagnostics.get('goal_invalidated_by_candidate_roi')
                or diagnostics.get('goal_invalidated_by_exclusion')):
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

        candidate_goal=goal
        if candidate_goal is None and target is not None and self.tracked_goal is not None and diagnostics.get('candidates',0)>0:
            candidate_goal=local_goal(self.tracked_goal,origin)
        if self.search_target_phase == 'PLANNING':
            if self.cfg.candidate_approach_enabled and not path:
                self.search_candidate_attempts += 1
                if self.search_candidate_attempts>=self.cfg.candidate_plan_attempts:
                    self.reject_search_candidate('진입 경로 계산 실패: 다른 빈 공간을 탐색합니다.')
                    return
            if goal is None and diagnostics.get('reason_code') == 'no_gap' and self.tracked_goal is None:
                self.search_target_phase = None
        elif (self.search_enabled and self.search_started is not None and not self.follower.path
                and not self.follower.parked and candidate_goal is not None
                and diagnostics.get('reason_code') != 'recovery_ready'):
            # Keep the detected target in the mission frame, discard the moving
            # snapshot's route, and plan again only after measured standstill.
            world_goal = compose(origin,(candidate_goal.x,candidate_goal.y,candidate_goal.yaw))
            self.tracked_goal = replace(candidate_goal,x=world_goal[0],y=world_goal[1],yaw=world_goal[2],
                heading_options=tuple(wrap(a+origin[2]) for a in candidate_goal.heading_options))
            self.search_target_phase = 'STOPPING'
            self.search_stop_count = 0
            self.search_stop_stamp = self.search_stationary_stamp(self.now_seconds())
            self.search_replan_after = None
            self.search_candidate_attempts = 0
            self.search_candidate_started = None
            self.state, self.reason = 'SEARCH_STOPPING', '주차 후보 발견: 정지 확인 후 경로를 계산합니다.'
            self.publish_command(0.,0.)
            self.path_pub.publish(self.path_message([]))
            self.get_logger().info(self.reason)
            return

        replacement_index = 0
        if (path and self.cfg.require_complete_entry_path and not final
                and diagnostics.get('reason_code')!='recovery_ready'
                and not (self.cfg.candidate_approach_enabled and diagnostics.get('reason_code')=='candidate_approach_ready')):
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
        if path and self.cfg.block_occupied_slots and self.slot_map is not None:
            checker=CollisionChecker(self.planning_scan(),self.cfg)
            local_path=[compose(inverse(self.tracker.pose),compose(origin,p)) for p in path]
            blocked=first_occupied_path_collision(checker,local_path,self.cfg.collision_step)
            if blocked:
                diagnostics['candidate_path_rejected_occupied_slot']=blocked
                return
        if path and self.cfg.continuous_replanning and (self.follower.path or self.recovery_lane_released) and diagnostics.get('reason_code') != 'recovery_ready':
            pose = self.tracker.current_pose(self.now_seconds()) if self.localization_source == 'fused' else self.tracker.pose
            global_path = [compose(origin,p) for p in path]
            replacement_index = min(range(len(global_path)),key=lambda i:math.hypot(global_path[i][0]-pose[0],global_path[i][1]-pose[1]))
            nearest = global_path[replacement_index]
            if (replacement_index >= len(global_path)-1 or math.hypot(nearest[0]-pose[0],nearest[1]-pose[1]) > .15
                    or abs(math.atan2(math.sin(nearest[2]-pose[2]),math.cos(nearest[2]-pose[2]))) > .35):
                return  # Retain the old feasible route when a delayed replacement cannot join.

        if path and self.cfg.continuous_replanning and diagnostics.get('reason_code')!='recovery_ready':
            # The worker planned against an older snapshot. Validate all points
            # and the joining segment before replacing the accepted route.
            source=self.planning_scan()
            if source is None:
                diagnostics['candidate_path_rejected']='observation_unavailable'
                return
            pose=self.tracker.current_pose(self.now_seconds()) if self.localization_source=='fused' else self.tracker.pose
            global_path=[compose(origin,p) for p in path]
            start=min(range(len(global_path)),key=lambda i:math.dist(global_path[i][:2],pose[:2]))
            transform=inverse(self.tracker.pose)
            local_path=[compose(transform,pose)]+[compose(transform,p) for p in global_path[start:]]
            known=[compose(transform,p) for p in self.follower.entry_trace]
            checker=CollisionChecker(source,self.cfg,known,inverse(self.tracker.pose)[:2])
            blocked=first_path_collision(checker,local_path,self.cfg.collision_step)
            if blocked:
                diagnostics['candidate_path_rejected']=blocked
                self.failed_plans+=1
                return

        if not path and diagnostics.get('reason_code') in ('no_observed_path','budget_exceeded','start_blocked'):
            self.failed_plans += 1
        elif path and diagnostics.get('reason_code') != 'recovery_ready':
            self.failed_plans = 0
        if (self.cfg.candidate_approach_enabled and not path and self.failed_plans>=self.cfg.candidate_plan_attempts
                and not self.follower.parked and not self.follower.recovering and not self.follower.exiting
                and self.state in ('PATH_HELD','REPLAN','REOBSERVE','ROUTE_BLOCKED','PLANNING')
                and self.search_stationary_stamp(self.now_seconds()) is not None):
            self.reject_search_candidate('접근 경로를 잇지 못했습니다. 다른 빈 공간을 탐색합니다.')
            return

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
                if (failure_recovery and not front_close and local is not None
                        and self.follower.final_approach and self.follower.path
                        and not self.follower.recovering and not self.follower.exiting
                        and self.state in ('PARKING','CONFIRMING')
                        and math.hypot(local.x,local.y)<=self.cfg.terminal_recovery_guard_distance):
                    # A fresh planning failure must not replace the final safe
                    # centimetres of an accepted path with a 30 cm retreat.
                    diagnostics['terminal_recovery_deferred']=True
                    self.failed_plans=0
                    return
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
                             (needs_reverse_for_goal(local,self.cfg) or (supported_recovery_target(local) and self.state in
                              ('PATH_HELD','ROUTE_BLOCKED','FRONT_BLOCKED','PLANNING','REPLAN'))
                              or (self.cfg.fixed_slot_empty_known and local.confirmed and diagnostics.get('entry_reserve_blocked')))))):
                    return
                known = [compose(inverse(pose),p) for p in self.follower.entry_trace]+[(0.,0.,0.)]
                cfg = (replace(self.cfg,assume_unobserved_free=True) if self.cfg.recovery_uses_memory_map else
                       (replace(self.cfg,assume_unobserved_free=False) if failure_recovery else self.cfg))
                if self.cfg.recovery_uses_memory_map and (not self.memory_enabled or self.memory.stamp is None):return
                rear = RecoveryCollisionChecker(self.planning_scan(),cfg,known,inverse(pose)[:2])
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
            was_released=self.recovery_lane_released
            if was_released and not recovery:
                # Lane commands are forward-only; avoid treating lane motion
                # as if the car were still in its previous reverse gear.
                self.follower.last_drive_direction=1
            self.follower.accept(goal, path, final, origin, recovery=recovery)
            if self.follower.path and not recovery:
                self.recovery_lane_released=False
                self.recovery_stop_confirmations=0
                self.search_target_phase = None
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

    def poll_steering_model(self):
        if not self.cfg.require_steering_calibration:return
        wall=time.monotonic()
        if self.steering_query is not None:
            if self.steering_query.done():
                try:
                    values=self.steering_query.result().values
                    if len(values)==4 and values[0].type==1 and all(v.type==3 for v in values[1:]):
                        self.steering_model={'enabled':values[0].bool_value,'gain':values[1].double_value,
                            'deadband':values[2].double_value,'limit':values[3].double_value}
                        self.steering_model_time=wall
                    else:self.steering_model=None
                except Exception:self.steering_model=None
                self.steering_query=None
            elif wall-self.steering_last_request>2.:
                self.steering_query.cancel();self.steering_query=None;self.steering_model=None
        if self.steering_query is None and wall-self.steering_last_request>=1. and self.steering_client.service_is_ready():
            request=GetParameters.Request(names=['steering_command_calibration','steering_command_gain',
                'steering_command_deadband','steering_command_limit'])
            self.steering_query=self.steering_client.call_async(request);self.steering_last_request=wall

    def steering_model_ready(self):
        if not self.cfg.require_steering_calibration:return True
        model=self.steering_model
        # Parking owns the command calibration. Refuse an enabled shared-driver
        # calibration, which would apply it twice and also affect lane commands.
        return bool(model and not model['enabled'] and time.monotonic()-self.steering_model_time<3.)

    def update_recovery_handoff(self,new_scan):
        if self.recovery_lane_released:
            self.state='LANE_REPLAN'
            return
        if not self.follower.recovery_finished or self.follower.recovering:return
        if new_scan:
            sample=getattr(self.tracker,'sample',None)
            measured=max(self.follower.measured_speed,abs(sample.speed) if sample is not None else 0.)
            self.recovery_stop_confirmations=self.recovery_stop_confirmations+1 if measured<.03 else 0
        self.state='RECOVERY_STOPPING'
        if self.recovery_stop_confirmations<3:return
        self.recovery_lane_released=True
        self.follower.path=[]
        self.follower.entry_path=[]
        self.follower.index=0
        self.follower.final_approach=False
        self.follower.awaiting_target_path=False
        self.state='LANE_REPLAN'
        self.reason=''

    def reject_search_candidate(self, reason):
        now=self.now_seconds()
        target=self.tracked_goal or self.follower.goal
        if target is not None:
            self.search_excluded_targets.append((target.x,target.y,now+self.cfg.candidate_plan_timeout))
        if self.pending is not None:
            self.pending[0].cancel()
            self.pending=None
        self.epoch+=1
        self.follower.path=[]
        self.follower.entry_path=[]
        self.follower.goal=self.follower.path_goal=None
        self.follower.final_approach=self.follower.awaiting_target_path=False
        self.tracked_goal=self.planned_goal=None
        self.search_target_phase=None
        self.search_replan_after=self.search_candidate_started=None
        self.search_stop_count=self.search_candidate_attempts=self.failed_plans=0
        self.planning_info={'reason_code':'no_gap','candidate_rejected_reason':reason}
        self.last_plan_wall=-math.inf
        self.publish_command(0.,0.)
        self.path_pub.publish(self.path_message([]))
        self.get_logger().info(reason)

    def search_stationary_stamp(self, now):
        wheel = self.wheel_sample
        if wheel is not None:
            stamp, speed, yaw_rate, received = wheel
            if (not -.05 <= now-stamp <= .3 or time.monotonic()-received > .3
                    or abs(speed) >= .01 or abs(yaw_rate) >= .02):
                return None
            return stamp
        if self.localization_source == 'fused':
            return None  # Real vehicle stops require fresh measured wheel data.
        sample = self.follower.previous_sample
        if sample is None or not 0 <= now-sample[1] <= .3 or self.follower.measured_speed >= .01:
            return None
        return sample[1]

    def search_command(self, now, pose, scan):
        if (self.cfg.candidate_approach_enabled and self.search_target_phase=='PLANNING'
                and self.search_candidate_started is not None
                and time.monotonic()-self.search_candidate_started>=self.cfg.candidate_plan_timeout):
            self.reject_search_candidate('경로 계산 제한 시간 도달: 다른 빈 공간을 탐색합니다.')
        if self.search_enabled and self.search_target_phase is not None:
            stamp = self.search_stationary_stamp(now)
            if stamp is None:
                self.search_target_phase = 'STOPPING'
                self.search_stop_count = 0
                self.search_stop_stamp = None
                self.search_replan_after = None
                if self.pending is not None:
                    self.pending[0].cancel()
                    self.pending = None
                self.epoch += 1
            elif self.search_target_phase == 'STOPPING' and stamp != self.search_stop_stamp:
                self.search_stop_count += 1
                self.search_stop_stamp = stamp
                if self.search_stop_count >= 3:
                    self.search_target_phase = 'PLANNING'
                    self.search_replan_after = now
                    self.search_candidate_started = time.monotonic()
                    self.last_plan_wall = -math.inf
                    self.get_logger().info('탐색 정지 확인 완료: 현재 위치에서 주차 경로를 다시 계산합니다.')
            self.state = 'SEARCH_STOPPING' if self.search_target_phase == 'STOPPING' else 'SEARCH_PLANNING'
            self.reason = ('주차 후보 발견: 정지를 확인하고 있습니다.' if self.search_target_phase == 'STOPPING' else
                           '정지 위치에서 주차 후보까지 이동할 경로를 계산하고 있습니다.')
            return 0.,0.
        if (not self.search_enabled or self.follower.parked or self.follower.path
                or self.follower.entry_locked or self.follower.recovering
                or self.planning_info.get('reason_code') not in ('no_gap','no_observed_path','center_pose_blocked','budget_exceeded','entry_departure_unavailable')):
            return None
        if self.search_started is None:
            self.search_started = now
            self.search_last_pose = pose
        if self.search_last_pose is None:
            self.search_last_pose = pose
        self.search_distance += math.dist(pose[:2],self.search_last_pose[:2])
        self.search_last_pose = pose
        if self.search_distance >= self.search_max_distance or now-self.search_started >= self.search_timeout:
            self.state,self.reason = 'SEARCH_LIMIT','Parking search limit reached; inspect detection and reset'
            return 0.,0.
        # A 10 Hz scan is already about 0.1 s old when delivered. Separate
        # input freshness from completed-job age; preserve the existing hard
        # scan and localization time bounds instead of toggling at 0.2 s.
        received=self.last_scan_received_stamp if self.last_scan_received_stamp is not None else self.scan_time
        if (self.scan_time is None or received is None
                or not 0 <= now-self.scan_time <= self.scan_timeout
                or not 0 <= now-received <= min(self.scan_timeout,self.localization_timeout)):
            self.search_clear_count=0
            self.state,self.reason = 'SEARCH_BLOCKED','Waiting for fresh observed search corridor'
            return 0.,0.
        new_scan=self.search_check_sequence != self.scan_sequence
        moved=(self.search_check_pose is None or math.dist(pose[:2],self.search_check_pose[:2])>.02
               or abs(wrap(pose[2]-self.search_check_pose[2]))>.02)
        if new_scan or moved:
            # Search follows the configured visibility policy too. Zero returns
            # must not override assume_unobserved_free; real obstacles still block.
            checker=CollisionChecker(scan,self.cfg,known_free_poses=[(0.,0.,0.)])
            distance=min(.3,max(.25,self.search_max_distance-self.search_distance))
            self.search_clear = True
            self.search_block_detail = None
            for i in range(16):
                checked_pose = (distance*i/15,0.,0.)
                if not checker.free(checked_pose):
                    self.search_clear = False
                    self.search_block_detail = dict(checker.last_block_detail or {},
                        kind=checker.last_block, checked_pose=list(checked_pose))
                    break
            self.search_check_sequence=self.scan_sequence
            self.search_check_pose=pose
            if not self.search_clear:self.search_clear_count=0
            elif new_scan:self.search_clear_count=min(self.search_resume_scans,self.search_clear_count+1)
        ready=self.search_clear and self.search_clear_count>=self.search_resume_scans
        self.state='SEARCHING' if ready else 'SEARCH_BLOCKED'
        self.reason=('Looking for parking candidates along a clear corridor' if ready else
                     'Waiting for stable observed search corridor' if self.search_clear else
                     {'yellow_boundary':'노란선 경계로 탐색 정지',
                      'obstacle':'탐색 전방 장애물로 정지',
                      'unknown_space':'탐색 구간 미관측으로 정지',
                      'occupied_slot':'점유 칸 경계로 탐색 정지'}.get(
                          (self.search_block_detail or {}).get('kind'),
                          '탐색 통로 확인 대기'))
        if not self.search_clear and self.search_block_detail:
            point = self.search_block_detail.get('point')
            if point is not None:
                self.reason += f' (장애물 {point[0]:.2f}, {point[1]:.2f}m)'
        return (self.search_speed if ready else 0.),0.

    def tick(self):
        tick_started=time.monotonic()
        self.lock_departure_localization()
        self.poll_scan_match()
        if self.yellow_pending is not None:
            pending, self.yellow_pending = self.yellow_pending, None
            self.on_yellow_boundary(pending)
        if self.tick_start_sequence():
            return
        if self.floor_gate is not None:
            self.floor_gate.update_approach()
        self.poll_approach_calibration()
        self.resolve_approach_reference()
        now = self.now_seconds()
        self.poll_steering_model()
        if self.last_now is not None and now < self.last_now - .01:
            if self.activation.active:
                self.publish_command(0.0, 0.0)
            self.reset_state()
            self.reason = 'Clock moved backwards'
        self.last_now = now
        self.activation.expire(now)
        self.poll_plan()
        self.poll_exit_plan()
        speed, steering = 0.0, 0.0
        search_control = False
        if not self.activation.active:
            self.state = 'WAITING_WHITE_LINE' if self.floor_gate is not None and self.floor_gate.armed else 'WAITING_SIGN'
            if self.floor_gate is not None and self.floor_gate.armed:
                self.state='APPROACH_LIMIT' if self.floor_gate.failure else 'WAITING_WHITE_LINE'
                self.reason=self.floor_gate.failure or f'흰선을 확인하며 {self.floor_gate.approach_speed:.2f}m/s 이하로 접근합니다.'
            if self.approach_calibration.pending or self.approach_calibration.phase == 'FAILED':
                self.state = 'WAITING_CALIBRATION' if self.approach_calibration.pending else 'CALIBRATION_FAILED'
                self.reason = self.approach_calibration.reason
            self.clear_visuals(clear_obstacles=False)
        elif self.follower.exited:
            # Completion is terminal until an explicit reset or a new sign.
            # Idle sensor health must not reopen a finished parking mission.
            self.state, self.reason = 'EXITED', ''
            self.follower.path = []
            if self.reported_state != 'EXITED':
                self.clear_visuals(clear_obstacles=False)
        elif self.motion_failed:
            self.state,self.reason='MISSION_FAILED','Parking made no progress for 20 s; reset required'
        elif self.manual_interrupted:
            self.state,self.reason = 'MANUAL_INTERRUPTED','Manual control or stale vehicle mode; reset required after takeover'
        elif not self.vehicle_automatic():
            self.state,self.reason = 'MANUAL_INTERRUPTED','Manual control or stale vehicle mode; reset required after takeover'
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
        elif not self.steering_model_ready():
            self.state='WAITING_STEERING'
            self.reason='Waiting for uncalibrated limo_base; parking owns steering calibration'
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
                c,s=math.cos(delta[2]),math.sin(delta[2])
                obstacle_scan.points=[(delta[0]+c*x-s*y,delta[1]+s*x+c*y) for x,y in source.points]
                obstacle_scan.point_owners={}
                obstacle_scan.yellow_segments=[(compose(delta,(*a,0.))[:2],compose(delta,(*b,0.))[:2],side)
                    for a,b,side in getattr(source,'yellow_segments',())]
                obstacle_scan.sensor_pose=compose(delta,source.sensor_pose)
                obstacle_scan.occupied_slots=[dict(r,pose=compose(delta,r['pose'])) for r in getattr(source,'occupied_slots',())]
            fresh_control_scan=self.controlled_sequence != self.scan_sequence
            if self.recovery_lane_released:
                self.follower.observe_pose(control_pose,now,fresh_control_scan)
                self.follower.lookahead_point=None
                self.state='LANE_REPLAN'
            else:
                collision = self.saved_path_collision(obstacle_scan,control_pose) if self.health_path_tracking else None
                if collision:
                    self.follower.observe_pose(control_pose,now,fresh_control_scan)
                    self.follower.last_collision = collision
                    speed, steering, self.state = self.follower.stop('ROUTE_BLOCKED',now)
                else:
                    speed, steering, self.state = self.follower.command(
                        self.scan, control_pose, fresh_control_scan, now=now, obstacle_scan=obstacle_scan)
                self.lock_departure_localization()
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
            search = None if self.health_path_tracking else self.search_command(now,control_pose,obstacle_scan)
            if search is not None:
                speed,steering = search
                search_control = True
            self.update_recovery_handoff(fresh_control_scan)
            self.controlled_sequence = self.scan_sequence
            self.try_exit_lane_handoff()
            if self.follower.exit_handoff_hold:
                speed,steering=0.,0.
                self.state='EXIT_LANE_STOPPING'
            wall_now = time.monotonic()
            search_exhausted=(self.search_enabled and self.search_target_phase is None and not self.follower.path
                and self.search_started is not None
                and (self.search_distance>=self.search_max_distance or now-self.search_started>=self.search_timeout))
            observe_pair=(self.cfg.automatic_target_selection and not self.cfg.auto_exit_after_parking
                          and self.follower.goal is not None
                          and (self.follower.goal.kind=='pair_midpoint' or not self.follower.goal.confirmed))
            retain_entry=(self.follower.path and self.follower.final_approach
                and not self.follower.recovering and not self.follower.exiting
                and self.state in ('PARKING','CONFIRMING') and self.follower.goal is not None
                and ((not self.cfg.replan_while_following and self.follower.goal.confirmed)
                     or math.dist(control_pose[:2],(self.follower.goal.x,self.follower.goal.y))
                        <=self.cfg.terminal_recovery_guard_distance))
            if (not self.health_path_tracking and self.search_target_phase != 'STOPPING'
                    and (self.search_target_phase != 'PLANNING' or self.scan_time >= self.search_replan_after)
                    and not search_exhausted and not retain_entry and not self.follower.parked and self.pending is None
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
                recovery_allowed = (not self.recovery_lane_released and self.recovery_attempts < self.cfg.max_recovery_attempts and self.recovery_remaining >= 3*cfg.collision_step
                    and (not cfg.recovery_uses_memory_map or (self.memory_enabled and self.memory.stamp is not None)))
                self.search_excluded_targets=[t for t in self.search_excluded_targets if now<t[2]]
                excluded=tuple((x,y,.15) for x,y,_ in self.search_excluded_targets)
                self.pending = (self.worker.submit(build_plan_report, self.planning_scan(), cfg, goal, self.tracker.pose,
                                                   known, recovery_allowed, self.blind_reverse_remaining, True, self.scan, self.failed_plans,
                                                   bool(self.follower.path) and self.state in ('PATH_HELD','ROUTE_BLOCKED','FRONT_BLOCKED','PLANNING','REPLAN'),self.slot_map,excluded),
                                self.epoch, self.tracker.pose, now)
                self.last_plan_wall = wall_now
            self.publish_control_visuals()
        if self.follower.exited:
            self.release_parking_commands()
        has_control_path = self.activation.active and not self.follower.exited and not self.recovery_lane_released and (bool(self.follower.path) or search_control)
        if self.parking_commands_released:
            pass
        elif self.floor_gate is not None and self.floor_gate.failure:
            self.publish_command(0.,0.)
        elif self.motion_failed or self.manual_interrupted or (self.activation.active and not self.vehicle_automatic()):
            self.publish_command(0., 0.)
        elif self.parking_calibration_stop and (self.approach_calibration.pending or self.approach_calibration.phase == 'FAILED'):
            self.publish_command(0., 0.)
        elif (self.localization_recovering or self.localization_hold_since is not None) and self.activation.active and not self.follower.exited:
            # Keep ownership of the parking mux input while awaiting a fresh match.
            self.publish_command(0., 0.)
        elif has_control_path:
            self.publish_command(speed, steering)
        elif self.retain_parking_control and self.activation.active and not self.follower.exited and not self.recovery_lane_released:
            self.publish_command(0.,0.)
        elif self.had_control_path:
            # Stop the previous route once, then let the mux input expire.
            self.publish_command(0.,0.)
        waiting_for_localization = (self.state in ('WAITING_LOCALIZATION','WAITING_SCAN','WAITING_CALIBRATION')
            or self.localization_recovering or self.localization_hold_since is not None)
        if waiting_for_localization:
            self.motion_stall_since = None
        elif (self.retain_parking_control and self.activation.active and not self.follower.exited
                and not self.follower.parked and not self.manual_interrupted and self.vehicle_automatic()):
            if (self.follower.previous_sample is not None and 0<=now-self.follower.previous_sample[1]<=.2
                    and self.follower.measured_speed>.01):
                self.motion_stall_since=None
            elif self.motion_stall_since is None:self.motion_stall_since=now
            elif now-self.motion_stall_since >= 20.:
                self.motion_failed=True
                self.publish_command(0.,0.)
        self.publish_guard()
        self.had_control_path = has_control_path
        self.publish_memory()
        self.planning_info.update(search_block=self.search_block_detail,
            recovery_stop=self.last_recovery_stop,
            route_collision=self.follower.last_collision if self.state=='ROUTE_BLOCKED' else None,
            recovery_remaining_m=self.recovery_remaining,recovery_attempts=self.recovery_attempts,
            max_recovery_attempts=self.cfg.max_recovery_attempts,path_hidden_by_slot=self.path_hidden_by_slot,
            exit_route_reason=self.follower.exit_route_reason,
            fusion_health_age_s=now-self.fusion_health['stamp'] if self.fusion_health else None,
            fusion_health_callback_age_s=time.monotonic()-self.fusion_health_received if self.fusion_health_received is not None else None)
        explanation = operator_reason(self.state, self.reason, self.planning_info,
                                      self.activation.count, self.required_detections)
        log_now = time.monotonic()
        state_changed = self.state != self.reported_state
        reason_changed = explanation != self.reported_reason and log_now-self.last_operator_log_wall >= 1.
        stopped_heartbeat = self.activation.active and speed == 0 and log_now-self.last_operator_log_wall >= 5.
        # Announce completion once; completed missions need no periodic stop log.
        if state_changed or (self.state != 'EXITED' and (reason_changed or stopped_heartbeat)):
            transition = f'{state_text(self.reported_state)} → {state_text(self.state)}' if state_changed else state_text(self.state)
            label = '정지 이유' if speed == 0 and not self.recovery_lane_released else '상태 설명'
            log_explanation=explanation
            if self.state=='WAITING_LOCALIZATION':
                sync=self.synchronization_status(now)
                ages=', '.join(f'{key}='+('--' if sync[key] is None else f'{sync[key]:.2f}s') for key in ('scan_age_s','odom_age_s','health_age_s'))
                log_explanation+=f' [{ages}; 정렬 대기 {sync["pending_scans"]}개]'
                rejection=self.last_localization_rejection
                if rejection and 'variance' in rejection:
                    vx,vy,vyaw=rejection['variance']
                    xy_limit=rejection['xy_variance_limit']
                    yaw_limit=rejection['yaw_variance_limit']
                    xy_text='해제' if xy_limit is None else f'{xy_limit:.3f}m²'
                    yaw_text='해제' if yaw_limit is None else f'{yaw_limit:.3f}rad²'
                    log_explanation+=(f' [EKF 거부 분산: x={vx:.3f}, y={vy:.3f}m² / '
                                      f'한계 {xy_text}; yaw={vyaw:.3f}rad² / 한계 {yaw_text}]')
                if self.localization_recovering and self.last_localization_restart:
                    discarded = self.last_localization_restart
                    prior = ', '.join(f'{key}='+('--' if discarded[key] is None else f'{discarded[key]:.2f}s')
                                      for key in ('scan_age_s','odom_age_s','health_age_s'))
                    log_explanation += f' [폐기 당시 {prior}]'

            self.get_logger().info(f'주차 상태: {transition} (시도 {self.attempt})'
                                   +(f' | {label}: {log_explanation}' if log_explanation else ''))
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
            self.active_pub.publish(Bool(data=self.activation.active and not self.follower.exited))
            self.status_pub.publish(String(data=json.dumps({
                'scan_worker_duration_s': self.scan_job_duration,
                'manual_interrupted':self.manual_interrupted,
                'state': self.state, 'state_text': state_text(self.state), 'active': self.activation.active and not self.follower.exited,
                'synchronization':self.synchronization_status(now),
                'approach_calibration': {'phase':self.approach_calibration.phase,
                    'generation':self.approach_calibration.generation,
                    'requested':self.approach_calibration.requested,
                    'reason':self.approach_calibration.reason},
                'search':{'distance_m':self.search_distance,'max_distance_m':self.search_max_distance,
                    'started':self.search_started,'speed_m_s':self.search_speed,
                    'target_phase':self.search_target_phase,'stop_samples':self.search_stop_count},
                'mission_attempt': self.attempt,
                'consecutive_detections': self.floor_gate.confirmation.count if self.floor_gate is not None else self.activation.count, 'reason': explanation,
                'white_roi_armed': self.floor_gate.armed if self.floor_gate is not None else False,
                'parking_stop_line_guard': {'requires_lane_owner':self.parking_stop_require_lane_owner,
                    'mux_source':self.parking_mux_source,'allowed':self.parking_stop_line_allowed()},
                'parking_sign_motion': {'enabled':self.parking_sign_motion_guard,
                    'allowed':self.sign_motion.allows(time.monotonic()),
                    'scope':'parking_stop_line',
                    'stop_ignore_sec':self.sign_motion.ignore_sec,
                    'ignore_remaining_sec':max(0.,self.sign_motion.ignore_until-time.monotonic())},
                'white_approach':{'speed_m_s':self.floor_gate.approach_speed,
                    'distance_m':self.floor_gate.approach_distance,
                    'max_distance_m':self.floor_gate.approach_max_distance,
                    'timeout_s':self.floor_gate.approach_timeout,
                    'failure':self.floor_gate.failure} if self.floor_gate is not None else None,
                'white_roi_ratio': self.floor_gate.ratio if self.floor_gate is not None else None,
                'path_poses': len(self.follower.path),
                'path_hidden_by_occupied_slot':self.path_hidden_by_slot,
                'last_recovery_stop':self.last_recovery_stop,
                'steering_gain': self.cfg.steering_gain,
                'curvature_feedforward':self.cfg.curvature_feedforward,
                'block_occupied_slots':self.cfg.block_occupied_slots,
                'collision_mode':'occupied_slots_and_obstacles' if self.cfg.block_occupied_slots else 'observed_and_remembered_obstacles',
                'previous_control_tick_s':self.last_tick_duration,
                'fusion_health_age_s':now-self.fusion_health['stamp'] if self.fusion_health else None,
                'fusion_health_callback_age_s':time.monotonic()-self.fusion_health_received if self.fusion_health_received is not None else None,
                'steering_response_scale': self.cfg.steering_response_scale,
                'steering_model_ready':self.steering_model_ready(),'driver_steering_model':self.steering_model,
                'steering_calibration_location':'parking' if self.cfg.require_steering_calibration else 'none',
                'command_steering_angle':parking_steering_command(steering,self.cfg) if abs(speed)>=1e-6 else 0.,
                'lookahead_base_m': self.follower.lookahead,
                'publishing_control': has_control_path,
                'recovery_lane_released':self.recovery_lane_released,
                'recovery_stop_confirmations':self.recovery_stop_confirmations,
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
                'exit_lane_handoff': self.exit_lane_handed_off,
                'exit_handoff_stopping': self.follower.exit_handoff_hold,
                'exit_heading_tolerance_rad': self.exit_heading_tolerance,
                'exit_path_length_m': self.follower.exit_path_length,
                'exit_route_reason': self.follower.exit_route_reason,
                'exit_route_ready': self.follower.exit_route_ready,
                'exit_plan_pending': self.exit_pending is not None,
                'entry_trace_poses': len(self.follower.entry_trace),
                'goal_error': ({'x': goal_error.x, 'y': goal_error.y, 'yaw': goal_error.yaw}
                               if goal_error is not None else None),
                'parking_position_tolerance_m': self.parking_position_tolerance,
                'effective_stop_tolerance_m': (.07 if self.follower.exiting else
                    ((goal_error.stop_tolerance if goal_error is not None and goal_error.kind in
                      ('enclosed_center','pair_midpoint') else 0.) or self.parking_position_tolerance)),
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
        self.last_tick_duration=time.monotonic()-tick_started

    def destroy_node(self):
        if self.activation.active and rclpy.ok(context=self.context):
            self.publish_command(0.0, 0.0)
        self.scan_worker.shutdown(wait=True, cancel_futures=True)
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
    executor = SingleThreadedExecutor(context=node.context)
    executor.add_node(node)
    try:
        for signum in (signal.SIGINT, signal.SIGTERM):
            previous[signum] = signal.signal(signum, lambda *_: stopped.set())
        while rclpy.ok() and not stopped.is_set():
            executor.spin_once(timeout_sec=.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as exc:
        exit_code = 1
        node.get_logger().error('주차 노드 실행 중 오류가 발생해 종료합니다: '+reason_text(str(exc))
                               +f'\n{type(exc).__name__}: {exc}\n'+traceback.format_exc())
    finally:
        executor.remove_node(node)
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for signum, handler in previous.items():
            signal.signal(signum, handler)
    return exit_code


if __name__ == '__main__':
    raise SystemExit(main())
