#!/usr/bin/env python3
"""평가 노드(시험 하네스). 정답 자세(/sim/ground_truth)·맵 장애물 좌표는 여기서만 쓴다.

- odom 수신 후 activate_at(시뮬 초)에 /objavoid_v3/activate=true 발행(수동 활성화 대행)
- 최소 장애물 간격(차체-장애물 직사각형 거리), 충돌, 조향·조향 변화율, 명령 주기, 상태 기록
- out 파일(JSON)을 0.5 s마다 갱신. 오케스트레이터가 종료 시점을 정한다.
"""
import argparse
import json
import math
import os
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(ROOT, 'simul/src/limo_parking_sim'))
sys.path.insert(0, HERE)
from limo_parking_sim.config import load_config  # noqa: E402
from metrics import Rect, rect_distance, section_obstacles  # noqa: E402


class Evaluator(Node):
    def __init__(self, a):
        super().__init__('objavoid_v3_evaluator')
        self.a = a
        self.cfg = load_config(None, a.map or None)
        v = self.cfg['vehicle']
        self.L, self.W = v['length'], v['width']
        self.obs = section_obstacles(self.cfg)
        self.res = {'map': os.path.basename(a.map) if a.map else 'sim_default', 'activated_t': None,
                    'done': False, 'done_t': None, 'collisions': 0, 'collision_names': [],
                    'min_gap': None, 'min_gap_obstacle': None, 'cmd_count': 0, 'steer_max': 0.0,
                    'steer_rate_max': 0.0, 'steer_sign_reversals': 0, 'speed_max': 0.0,
                    'cmd_gap_max_ms': 0.0, 'cmd_period_ms': None, 'stop_time_s': 0.0,
                    'pose': None, 'min_x': None, 'state': None, 'lateral_max_dev': 0.0,
                    'yellow_min_margin': None, 'cmd_publishers': None}
        self.min_gap = np.inf
        self.last_cmd = None
        self.ext, self.dir = None, 0
        self.cmd_wall = []
        self.pose = None
        self.sim_t = 0.0
        self.act_pub = self.create_publisher(Bool, '/objavoid_v3/activate', 10)
        self.create_subscription(Odometry, '/sim/ground_truth', self.on_truth, 20)
        self.create_subscription(String, '/sim/parking_status', self.on_status, 10)
        self.create_subscription(String, a.state_topic, self.on_state, 10)
        self.create_subscription(AckermannDriveStamped, a.cmd_topic, self.on_cmd, 50)
        self.create_subscription(Bool, a.done_topic,
                                 self.on_done, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_timer(0.5, self.flush)
        self.create_timer(0.1, self.maybe_activate)

    def maybe_activate(self):
        if self.res['activated_t'] is None and self.pose is not None and self.sim_t >= self.a.activate_at:
            self.act_pub.publish(Bool(data=True))
            st = self.res['state'] or {}
            if st.get('state') not in (None, 'IDLE') or self.sim_t > self.a.activate_at + 2.0:
                self.res['activated_t'] = round(self.sim_t, 2)

    def on_truth(self, m):
        q = m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        x, y = m.pose.pose.position.x, m.pose.pose.position.y
        self.sim_t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        self.pose = (x, y, yaw)
        body = Rect(x, y, yaw, self.L, self.W)
        for i, o in enumerate(self.obs):
            g = rect_distance(body, o)
            if g < self.min_gap:
                self.min_gap = g
                self.res['min_gap_obstacle'] = [round(o.x, 3), round(o.y, 3)]
        self.res['min_gap'] = round(float(self.min_gap), 4)
        self.res['pose'] = [round(x, 3), round(y, 3), round(yaw, 3)]
        self.res['min_x'] = round(min(x, self.res['min_x'] or 99), 3)
        if 4.0 < x < 8.6:  # 노란 실선 안쪽 y=0.171~1.708 대비 차체 모서리 여유
            ys = body.corners()[:, 1]
            mm = float(min(ys.min() - 0.171, 1.708 - ys.max()))
            if x < 8.05:
                self.res['yellow_min_margin'] = round(min(mm, self.res['yellow_min_margin'] or 9), 4)

    def on_status(self, m):
        s = json.loads(m.data)
        if s.get('collisions', 0) > self.res['collisions']:
            self.res['collision_names'] = sorted(set(self.res['collision_names']) | set(s.get('collision_names', [])))
        self.res['collisions'] = max(self.res['collisions'], s.get('collisions', 0))

    def on_state(self, m):
        self.res['state'] = json.loads(m.data)

    def on_done(self, m):
        if m.data and not self.res['done']:
            self.res['done'] = True
            self.res['done_t'] = round(self.sim_t, 2)

    def on_cmd(self, m):
        w = time.monotonic()
        d, v = m.drive.steering_angle, m.drive.speed
        r = self.res
        r['cmd_count'] += 1
        r['steer_max'] = round(max(r['steer_max'], abs(d)), 4)
        r['speed_max'] = round(max(r['speed_max'], v), 3)
        if self.last_cmd is not None:
            lw, ld, lv = self.last_cmd
            dt = w - lw
            if dt > 1e-3:
                rate = (d - ld) / dt
                r['steer_rate_max'] = round(max(r['steer_rate_max'], abs(rate)), 3)
            # 진동: 진폭 0.02 rad 이상 되돌림(히스테리시스 극값) 횟수
            if self.ext is None:
                self.ext, self.dir = d, 0
            elif self.dir >= 0 and d < self.ext - 0.02:
                if self.dir > 0:
                    r['steer_sign_reversals'] += 1
                self.dir, self.ext = -1, d
            elif self.dir <= 0 and d > self.ext + 0.02:
                if self.dir < 0:
                    r['steer_sign_reversals'] += 1
                self.dir, self.ext = 1, d
            elif (self.dir > 0 and d > self.ext) or (self.dir < 0 and d < self.ext) or self.dir == 0:
                self.ext = d if self.dir != 0 else self.ext
            r['cmd_gap_max_ms'] = round(max(r['cmd_gap_max_ms'], dt * 1000), 1)
            if v <= 1e-3:
                r['stop_time_s'] = round(r['stop_time_s'] + min(dt, 0.2), 2)
            self.cmd_wall.append(dt)
        self.last_cmd = (w, d, v)

    def flush(self):
        if self.cmd_wall:
            a = np.array(self.cmd_wall)
            self.res['cmd_period_ms'] = {'mean': round(1000 * a.mean(), 2), 'p99': round(1000 * np.percentile(a, 99), 2),
                                         'max': round(1000 * a.max(), 2)}
        infos = self.get_publishers_info_by_topic(self.a.cmd_topic)
        self.res['cmd_publishers'] = sorted({i.node_name for i in infos})
        self.res['command_05_publishers'] = sorted({i.node_name for i in
                                                     self.get_publishers_info_by_topic('/mission/objavoid/command_05')})
        self.res['nodes'] = sorted(self.get_node_names())
        self.res['sim_t'] = round(self.sim_t, 2)
        tmp = self.a.out + '.tmp'
        with open(tmp, 'w') as f:
            json.dump(self.res, f, ensure_ascii=False)
        os.replace(tmp, self.a.out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--map', default='')
    ap.add_argument('--out', required=True)
    ap.add_argument('--cmd-topic', default='/objavoid_v3/cmd')
    ap.add_argument('--activate-at', type=float, default=2.0)
    ap.add_argument('--state-topic', default='/objavoid_v3/state')
    ap.add_argument('--done-topic', default='/objavoid_v3/done')
    a, rest = ap.parse_known_args()
    rclpy.init(args=['objavoid_v3_evaluator'] + rest)
    n = Evaluator(a)
    try:
        rclpy.spin(n)
    except Exception:
        pass
    finally:
        try:
            n.flush()
        except Exception:
            pass


if __name__ == '__main__':
    main()
