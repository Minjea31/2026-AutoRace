"""ROS 2 노드 objavoid_v3_node.

스레드 구성
  - executor 스레드 2개: 센서 콜백(scan 저장, odom 이력)과 제어 타이머(25 Hz)를 서로 다른 콜백 그룹으로 분리
  - 작업 스레드 1개: 최신 스캔만 꺼내 인지 + (필요할 때만) 경로 계획. 밀린 스캔은 버린다.
제어 타이머는 계획 중에도 저장된 경로로 명령을 계속 낸다.

토픽
  입력  /scan, /odom, /objavoid_v3/activate (std_msgs/Bool, true=시작, false=중지)
  출력  command_topic (기본 /objavoid_v3/cmd, AckermannDriveStamped)
        /objavoid_v3/state (String JSON), /objavoid_v3/done (Bool, latched), /objavoid_v3/markers, /objavoid_v3/path
"""
import json
import math
import threading
import time
from pathlib import Path

import numpy as np
import yaml

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data

from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry, Path as PathMsg
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Bool, String
from visualization_msgs.msg import MarkerArray

from .lane_boundary import GroundProjection
from .markers import build_markers, preview_markers, to_base
from .mission import DONE, IDLE, ObjAvoidCore
from .params import Params, param_items


def stamp_sec(st):
    return st.sec + st.nanosec * 1e-9


class ObjAvoidV3Node(Node):
    def __init__(self):
        super().__init__('objavoid_v3_node')
        p = Params()
        for name, default in param_items():
            setattr(p, name, self.declare_parameter(name, default).value)
        self.p = p
        self.command_topic = self.declare_parameter('command_topic', '/objavoid_v3/cmd').value
        scan_topic = self.declare_parameter('scan_topic', '/scan').value
        odom_topic = self.declare_parameter('odom_topic', '/odom').value
        self.auto_activate = self.declare_parameter('auto_activate', False).value
        self.marker_rate = self.declare_parameter('marker_rate', 5.0).value
        # 차선(노란 실선): 퓨전 차선 인식의 딥 마스크, 바닥 투영은 퓨전 lane.yaml
        repo = Path(__file__).resolve().parents[4]
        lane_cfg = self.declare_parameter(
            'lane_config', str(repo / 'lane_fusion_ws/src/autorace_lane_fusion/config/sim/lane.yaml')).value
        mask_topic = self.declare_parameter('lane_mask_topic', '/lane_fusion/deep_mask').value
        self.lane_rate = self.declare_parameter('lane_rate', 5.0).value
        # 차별 바닥 투영(이미지 px -> base_link 바닥 m, 3x3). 0이면 lane_config의 BEV 사용. 04번은 bag 보정값.
        ground_h = [float(v) for v in self.declare_parameter('ground_homography', [0.0] * 9).value]
        lane_range = float(self.declare_parameter('lane_max_range', 2.5).value)
        ground_scale = [float(v) for v in self.declare_parameter('ground_scale', [1.0, 1.0]).value]
        # 자세: odom 그대로 / imu(방향은 IMU 자이로 적분, 위치는 odom 전진 속도 x odom_scale로 적분)
        self.yaw_source = self.declare_parameter('yaw_source', 'odom').value
        self.odom_scale = float(self.declare_parameter('odom_scale', 1.0).value)
        imu_topic = self.declare_parameter('imu_topic', '/imu').value
        # RViz 마커 좌표: odom(내부 자세 = 드라이버 odom일 때) / base_link(내부 자세가 TF odom과 다를 때)
        self.marker_frame = self.declare_parameter('marker_frame', 'odom').value
        self.gp = None
        if p.use_lane:
            try:
                with open(lane_cfg, encoding='utf-8') as f:
                    self.gp = GroundProjection(yaml.safe_load(f)['lane_lite'], max_range=lane_range,
                                               homography=ground_h if any(ground_h) else None)
                if any(v != 1.0 for v in ground_scale):
                    self.gp.set_scale(*ground_scale)
            except Exception as e:  # 보정 파일이 없으면 차선 없이(사전 경계) 동작
                self.get_logger().error(f'lane_config load failed ({lane_cfg}): {e!r} -> prior boundary only')
                p.use_lane = False
        self.core = ObjAvoidCore(p)
        self.lane_lock = threading.Lock()
        self.latest_mask, self.last_lane_wall = None, 0.0

        sensors = MutuallyExclusiveCallbackGroup()
        control = MutuallyExclusiveCallbackGroup()
        self.create_subscription(LaserScan, scan_topic, self.on_scan, qos_profile_sensor_data, callback_group=sensors)
        self.create_subscription(Odometry, odom_topic, self.on_odom, QoSProfile(depth=20), callback_group=sensors)
        self.imu_state = None  # (t, yaw, wz)
        self.dead = None       # (t, x, y)
        if self.yaw_source == 'imu':
            from sensor_msgs.msg import Imu
            self.create_subscription(Imu, imu_topic, self.on_imu, QoSProfile(depth=50), callback_group=sensors)
        self.create_subscription(Bool, '/objavoid_v3/activate', self.on_activate, 10, callback_group=sensors)
        if self.gp is not None:
            self.create_subscription(Image, mask_topic, self.on_mask, qos_profile_sensor_data, callback_group=sensors)
        self.cmd_pub = self.create_publisher(AckermannDriveStamped, self.command_topic, QoSProfile(depth=1))
        self.state_pub = self.create_publisher(String, '/objavoid_v3/state', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.done_pub = self.create_publisher(Bool, '/objavoid_v3/done', latched)
        self.marker_pub = self.create_publisher(MarkerArray, '/objavoid_v3/markers', 2)
        self.path_pub = self.create_publisher(PathMsg, '/objavoid_v3/path', 2)
        self.done_pub.publish(Bool(data=False))

        self.scan_lock = threading.Lock()
        self.latest_scan = None
        self.scan_recv_wall = 0.0
        self.dropped_scans = 0
        self.scan_event = threading.Event()
        self.running = True
        self.worker = threading.Thread(target=self.work_loop, daemon=True)
        self.worker.start()

        # 제어는 벽시계 고정 주기(시뮬 시간이 멈추면 odom 만료로 정지)
        self.ctrl_period = 1.0 / p.control_rate
        self.create_timer(self.ctrl_period, self.on_control, callback_group=control,
                          clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.create_timer(0.2, self.on_status, callback_group=control,
                          clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.create_timer(1.0, self.check_conflict, callback_group=control,
                          clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.last_ctrl_wall = None
        self.cmd_periods = []
        self.ctrl_ms = []
        self.conflict = []
        self.done_sent = False
        self.last_marker_wall = 0.0
        self.pending_activate = self.auto_activate
        self.get_logger().info(f'objavoid_v3 ready: cmd={self.command_topic} scan={scan_topic} odom={odom_topic} '
                               f'auto_activate={self.auto_activate}')

    # ---------------------------------------------------------------- 입력
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_imu(self, msg):
        t, wz = stamp_sec(msg.header.stamp), msg.angular_velocity.z
        if self.imu_state is None or t <= self.imu_state[0] or t - self.imu_state[0] > 0.5:
            self.imu_state = (t, 0.0 if self.imu_state is None else self.imu_state[1], wz)
            return
        t0, yaw, w0 = self.imu_state
        self.imu_state = (t, yaw + 0.5 * (w0 + wz) * (t - t0), wz)

    def on_odom(self, msg):
        t = stamp_sec(msg.header.stamp)
        v = msg.twist.twist.linear.x * self.odom_scale
        if self.yaw_source == 'imu':
            if self.imu_state is None:
                return
            _, yaw, wz = self.imu_state
            if self.dead is None or t <= self.dead[0] or t - self.dead[0] > 0.5:
                self.dead = (t, 0.0, 0.0)
            else:
                t0, x, y = self.dead
                self.dead = (t, x + v * math.cos(yaw) * (t - t0), y + v * math.sin(yaw) * (t - t0))
            self.core.on_odom(t, self.dead[1], self.dead[2], yaw, v, wz)
        else:
            q = msg.pose.pose.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            s = self.odom_scale
            self.core.on_odom(t, msg.pose.pose.position.x * s, msg.pose.pose.position.y * s,
                              yaw, v, msg.twist.twist.angular.z)
        if self.pending_activate:
            self.try_activate()

    def on_scan(self, msg):
        with self.scan_lock:
            if self.latest_scan is not None:
                self.dropped_scans += 1  # 처리 못한 이전 스캔은 버림
            self.latest_scan = msg
            self.scan_recv_wall = time.perf_counter()
        self.scan_event.set()

    def on_mask(self, msg):
        with self.lane_lock:
            self.latest_mask = msg

    def process_lane(self):
        """최신 딥 마스크 1장 -> 차 기준 노란 점 -> 코어. 작업 스레드에서 lane_rate 이하로."""
        if self.gp is None or time.monotonic() - self.last_lane_wall < 1.0 / max(self.lane_rate, 0.1):
            return
        with self.lane_lock:
            m, self.latest_mask = self.latest_mask, None
        if (m is None or m.encoding != 'mono8' or not m.height or not m.width
                or m.step < m.width or len(m.data) < m.step * m.height):
            return
        self.last_lane_wall = time.monotonic()
        st = stamp_sec(m.header.stamp)
        if self.now() - st > 0.5 or self.now() - st < -0.05:
            return
        mask = np.frombuffer(m.data, np.uint8).reshape(m.height, m.step)[:, :m.width]
        pts = self.gp.yellow_points(mask)
        wpts = self.gp.white_points(mask)
        self.core.on_lane(st, pts, self.gp.view_points, wpts)

    def on_activate(self, msg):
        if msg.data:
            if self.core.state not in (IDLE, DONE):
                return  # 이미 동작 중이면 무시(재시작은 false 후 true)
            self.pending_activate = True
            self.try_activate()
        else:
            self.pending_activate = False
            self.core.deactivate()
            self.get_logger().info('deactivated')

    def try_activate(self):
        if self.core.activate(self.now()):
            self.pending_activate = False
            self.done_sent = False
            self.cmd_periods, self.ctrl_ms = [], []
            self.done_pub.publish(Bool(data=False))
            self.get_logger().info('ACTIVATED: road frame fixed at current odom pose')

    # ---------------------------------------------------------------- 작업 스레드
    def work_loop(self):
        while self.running:
            if not self.scan_event.wait(0.2):
                continue
            self.scan_event.clear()
            with self.scan_lock:
                msg, self.latest_scan = self.latest_scan, None
                recv = self.scan_recv_wall
            self.core.scan_queue_ms.add((time.perf_counter() - recv) * 1000)
            try:
                self.process_lane()
            except Exception as e:
                self.get_logger().error(f'lane processing failed: {e!r}', throttle_duration_sec=5.0)
            if msg is None:
                continue
            if self.core.frame is None:
                self.preview(msg)  # 활성화 전: 장애물 검출만 미리보기(계획·명령 없음)
                continue
            t_now = self.now()
            st = stamp_sec(msg.header.stamp)
            if t_now - st > self.p.scan_timeout:
                continue  # 너무 오래된 스캔은 처리하지 않음
            try:
                self.core.process_scan(st, msg.ranges, msg.angle_min, msg.angle_increment, t_now)
            except Exception as e:  # 인지/계획 예외가 노드를 죽이지 않도록 (경로 없음 -> 정지)
                self.get_logger().error(f'process_scan failed: {e!r}')
                self.core.path = None
            w = time.monotonic()
            if self.marker_rate > 0 and w - self.last_marker_wall >= 1.0 / self.marker_rate:
                self.last_marker_wall = w
                self.publish_viz()

    def preview(self, msg):
        if self.marker_rate <= 0 or time.monotonic() - self.last_marker_wall < 1.0 / self.marker_rate:
            return
        self.last_marker_wall = time.monotonic()
        st = stamp_sec(msg.header.stamp)
        pose = self.core.pose_at(st, extrapolate=0.3)
        if pose is None:
            return
        try:
            arr = preview_markers(self.core, msg, pose[:3], self.get_clock().now().to_msg())
            if self.marker_frame != 'odom':
                to_base(arr, pose[:3], self.marker_frame)
            self.marker_pub.publish(arr)
        except Exception as e:
            self.get_logger().warning(f'preview failed: {e!r}', throttle_duration_sec=5.0)

    # ---------------------------------------------------------------- 제어
    def on_control(self):
        w0 = time.perf_counter()
        t = self.now()
        dt = self.ctrl_period
        if self.last_ctrl_wall is not None:
            dt = w0 - self.last_ctrl_wall
        self.last_ctrl_wall = w0
        out = self.core.control(t, dt)
        if out is not None and not self.conflict:
            v, delta = out
            msg = AckermannDriveStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'
            msg.drive.speed = float(v)
            msg.drive.steering_angle = float(delta)
            self.cmd_pub.publish(msg)
            self.cmd_periods.append(dt)
        if self.core.state == DONE and not self.done_sent:
            self.done_sent = True
            self.done_pub.publish(Bool(data=True))
            self.get_logger().info(f'DONE ({self.core.done_reason}): stop publishing commands')
            self.publish_status()
        self.ctrl_ms.append((time.perf_counter() - w0) * 1000)

    def check_conflict(self):
        """같은 명령 토픽에 다른 발행자(예: 기존 objavoid_node)가 있으면 발행 중단."""
        infos = self.get_publishers_info_by_topic(self.command_topic)
        others = sorted({i.node_name for i in infos if i.node_name != self.get_name()})
        if others != self.conflict:
            if others:
                self.get_logger().error(f'other publishers on {self.command_topic}: {others} -> commands paused')
            self.conflict = others

    # ---------------------------------------------------------------- 보고
    def on_status(self):
        self.publish_status()

    def publish_status(self):
        st = self.core.status()
        per = sorted(self.cmd_periods[-5000:])
        if per:
            st['cmd_period_ms'] = {'n': len(self.cmd_periods), 'mean': round(1000 * sum(per) / len(per), 2),
                                   'p99': round(1000 * per[int(0.99 * (len(per) - 1))], 2),
                                   'max': round(1000 * per[-1], 2)}
        cm = sorted(self.ctrl_ms[-5000:])
        if cm:
            st['ctrl_ms'] = {'mean': round(sum(cm) / len(cm), 3), 'max': round(cm[-1], 3)}
        st['dropped_scans'] = self.dropped_scans
        st['conflict'] = self.conflict
        st['t'] = round(self.now(), 3)
        veh = self.core.vehicle(self.now()) if self.core.frame is not None else None
        if veh is not None:
            st['s'], st['d'], st['psi'] = round(veh[0], 3), round(veh[1], 3), round(veh[2], 3)
        self.state_pub.publish(String(data=json.dumps(st, separators=(',', ':'))))

    def publish_viz(self):
        core = self.core
        if core.frame is None:
            return
        stamp = self.get_clock().now().to_msg()
        try:
            arr = build_markers(core, stamp, self.now())
            if self.marker_frame != 'odom':
                pose = core.pose_at(self.now(), extrapolate=0.3)
                if pose is None:
                    return
                to_base(arr, pose[:3], self.marker_frame)
            self.marker_pub.publish(arr)
        except Exception as e:
            self.get_logger().warning(f'marker build failed: {e!r}', throttle_duration_sec=5.0)
        path = core.path
        msg = PathMsg()
        msg.header.stamp, msg.header.frame_id = stamp, 'odom'
        if self.marker_frame != 'odom':
            return  # Path는 odom 좌표만(마커의 path로 확인)
        if path is not None:
            x, y = core.frame.to_odom(path.s[::2], path.d[::2])
            yaw = core.frame.yaw_to_odom(path.psi[::2])
            for xi, yi, ya in zip(x, y, yaw):
                ps = PoseStamped()
                ps.header = msg.header
                ps.pose.position.x, ps.pose.position.y = float(xi), float(yi)
                ps.pose.orientation.z, ps.pose.orientation.w = math.sin(ya / 2), math.cos(ya / 2)
                msg.poses.append(ps)
        self.path_pub.publish(msg)

    def destroy_node(self):
        self.running = False
        self.scan_event.set()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ObjAvoidV3Node()
    ex = MultiThreadedExecutor(num_threads=2)
    ex.add_node(node)
    try:
        ex.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except (KeyboardInterrupt, Exception):
            pass


if __name__ == '__main__':
    main()
