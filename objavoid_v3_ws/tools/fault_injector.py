#!/usr/bin/env python3
"""시험용 센서 고장 주입 중계: /scan -> /objavoid_v3/scan_f, /odom -> /objavoid_v3/odom_f.

옵션(JSON 문자열 --faults):
  scan_drop: 프레임 누락 확률, scan_blackout: [t0, t1] 시뮬 초 동안 스캔 차단(t1<0 = 영구)
  scan_delay: 지연 초, odom_delay: 지연 초, odom_blackout: [t0, t1]
  hide_box: [xmin, xmax, ymin, ymax] 맵 영역의 레이저 반사를 숨김, hide_until_x: 차량 x가 이보다 작아지면 공개
            (경로 추종 중 새 장애물이 '나타나는' 상황 재현. 차체 충돌은 시뮬레이터에 그대로 존재)
"""
import argparse
import json
import math
import random
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


class Injector(Node):
    def __init__(self, f):
        super().__init__('objavoid_v3_fault_injector')
        self.f = f
        self.rng = random.Random(f.get('seed', 0))
        self.scan_pub = self.create_publisher(LaserScan, '/objavoid_v3/scan_f', qos_profile_sensor_data)
        self.odom_pub = self.create_publisher(Odometry, '/objavoid_v3/odom_f', QoSProfile(depth=20))
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.on_odom, QoSProfile(depth=20))
        self.create_subscription(Odometry, '/sim/ground_truth', self.on_truth, 20)
        self.q = deque()
        self.truth = None
        self.revealed = False
        self.counts = {'scan_in': 0, 'scan_out': 0, 'odom_in': 0, 'odom_out': 0}
        self.create_timer(0.002, self.release)

    @staticmethod
    def t(msg):
        return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def blocked(self, key, t):
        w = self.f.get(key)
        return bool(w) and t >= w[0] and (w[1] < 0 or t <= w[1])

    def on_truth(self, m):
        q = m.pose.pose.orientation
        self.truth = (m.pose.pose.position.x, m.pose.pose.position.y,
                      math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
        if 'hide_until_x' in self.f and self.truth[0] < self.f['hide_until_x'] and not self.revealed:
            self.revealed = True
            self.get_logger().info(f'hidden obstacle revealed at x={self.truth[0]:.2f}')

    def on_scan(self, m):
        self.counts['scan_in'] += 1
        t = self.t(m)
        if self.blocked('scan_blackout', t) or self.rng.random() < self.f.get('scan_drop', 0.0):
            return
        if 'hide_box' in self.f and not self.revealed and self.truth is not None:
            x0, x1, y0, y1 = self.f['hide_box']
            x, y, yaw = self.truth
            lx, ly = x + 0.103 * math.cos(yaw), y + 0.103 * math.sin(yaw)
            r = list(m.ranges)
            for i, rr in enumerate(r):
                if rr > 0 and math.isfinite(rr):
                    a = yaw + m.angle_min + i * m.angle_increment
                    px, py = lx + rr * math.cos(a), ly + rr * math.sin(a)
                    if x0 <= px <= x1 and y0 <= py <= y1:
                        r[i] = 0.0  # 무반사(관측 안 됨)로 숨김
            m.ranges = r
        self.q.append((time.monotonic() + self.f.get('scan_delay', 0.0), 's', m))

    def on_odom(self, m):
        self.counts['odom_in'] += 1
        if self.blocked('odom_blackout', self.t(m)):
            return
        self.q.append((time.monotonic() + self.f.get('odom_delay', 0.0), 'o', m))

    def release(self):
        now = time.monotonic()
        keep = deque()
        while self.q:
            due, k, m = self.q.popleft()
            if due <= now:
                (self.scan_pub if k == 's' else self.odom_pub).publish(m)
                self.counts['scan_out' if k == 's' else 'odom_out'] += 1
            else:
                keep.append((due, k, m))
        self.q = keep


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--faults', default='{}')
    a, rest = ap.parse_known_args()
    rclpy.init(args=['inj'] + rest)
    n = Injector(json.loads(a.faults))
    try:
        rclpy.spin(n)
    except Exception:
        pass


if __name__ == '__main__':
    main()
