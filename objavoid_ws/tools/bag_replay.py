#!/usr/bin/env python3
"""실차 bag을 V3 코어에 재생(ROS 노드 없이)해 경로가 스캔마다 나오는지 측정.

bag의 /scan, /odom, /imu, /lane_fusion/deep_mask, /camera/color/image_raw/compressed를 시간순으로 넣는다.
차의 움직임은 녹화된 그대로(개루프)라서 '그 자세에서 계획이 경로를 내는가'를 본다. 관찰 모드로 돈다.

  PYTHONNOUSERSITE=1 python3 tools/bag_replay.py ~/dev/bags/v3_1114_1838 --car 04 [--set plan_min_radius=0.6 ...]
"""
import argparse
import json
import math
import os
import sys
from collections import Counter

import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
WS = os.path.dirname(HERE)
ROOT = os.path.dirname(WS)
sys.path.insert(0, os.path.join(WS, 'src/autorace_objavoid_v3'))
from autorace_objavoid_v3.lane_boundary import GroundProjection  # noqa: E402
from autorace_objavoid_v3.mission import ObjAvoidCore  # noqa: E402
from autorace_objavoid_v3.params import Params  # noqa: E402


def read_bag(path, topics):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=path, storage_id='sqlite3'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    r.set_filter(rosbag2_py.StorageFilter(topics=[t for t in topics if t in types]))
    cls = {t: get_message(types[t]) for t in topics if t in types}
    while r.has_next():
        t, d, _ = r.read_next()
        m = deserialize_message(d, cls[t])
        yield t, m


def stamp(m):
    return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--car', default='04')
    ap.add_argument('--set', nargs='*', default=[])
    ap.add_argument('--lane-config', default=os.path.join(ROOT, 'lane_fusion_ws/src/autorace_lane_deep_lite/config/real/mission_vision.yaml'))
    ap.add_argument('--t-end', type=float, default=None, help='bag 시작부터 이 초까지만')
    ap.add_argument('--win', nargs=2, type=float, default=[11, 16.5], help='-v 상세 출력 구간(s)')
    ap.add_argument('-v', action='store_true')
    a = ap.parse_args()
    p = Params()
    node_only = {}
    cfgf = os.path.join(WS, f'src/autorace_objavoid_v3/config/real_car{a.car}.yaml')
    for k, v in yaml.safe_load(open(cfgf))['objavoid_v3_node']['ros__parameters'].items():
        if hasattr(p, k):
            setattr(p, k, v)
        else:
            node_only[k] = v
    for kv in a.set:
        k, v = kv.split('=')
        cur = getattr(p, k)
        setattr(p, k, json.loads(v) if isinstance(cur, list) else type(cur)(float(v)) if not isinstance(cur, bool)
                else v.lower() == 'true')
    p.observe_mode = True
    gp = GroundProjection(yaml.safe_load(open(a.lane_config))['lane_lite'],
                          max_range=node_only.get('lane_max_range', 2.5),
                          homography=node_only.get('ground_homography'))
    if node_only.get('ground_scale'):
        gp.set_scale(*node_only['ground_scale'])
    yaw_imu = node_only.get('yaw_source', 'odom') == 'imu'
    oscale = float(node_only.get('odom_scale', 1.0))
    core = ObjAvoidCore(p)
    imu_state, dead, activated = None, None, False
    rows = []
    topics = ['/imu', '/odom', '/scan', '/lane_fusion/deep_mask',
              '/objavoid_v3/activate']
    first_cmd = None
    tb0 = None
    for t, m in read_bag(a.bag, topics):
        if hasattr(m, 'header'):
            tb = stamp(m)
            tb0 = tb if tb0 is None else tb0
            if a.t_end is not None and tb - tb0 > a.t_end:
                break
        if t == '/imu':
            ts, wz = stamp(m), m.angular_velocity.z
            if imu_state is None or ts - imu_state[0] > 0.5 or ts <= imu_state[0]:
                imu_state = (ts, 0.0 if imu_state is None else imu_state[1], wz)
            else:
                imu_state = (ts, imu_state[1] + 0.5 * (imu_state[2] + wz) * (ts - imu_state[0]), wz)
        elif t == '/odom':
            ts, v = stamp(m), m.twist.twist.linear.x * oscale
            if yaw_imu:
                if imu_state is None:
                    continue
                if dead is None or ts - dead[0] > 0.5:
                    dead = (ts, 0.0, 0.0)
                else:
                    t0, x, y = dead
                    dead = (ts, x + v * math.cos(imu_state[1]) * (ts - t0), y + v * math.sin(imu_state[1]) * (ts - t0))
                core.on_odom(ts, dead[1], dead[2], imu_state[1], v, imu_state[2])
            else:
                q = m.pose.pose.orientation
                yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
                core.on_odom(ts, m.pose.pose.position.x * oscale, m.pose.pose.position.y * oscale, yaw, v,
                             m.twist.twist.angular.z)
            if not activated and core.odom:
                activated = core.activate(ts)
        elif t == '/lane_fusion/deep_mask' and activated:
            mask = np.frombuffer(m.data, np.uint8).reshape(m.height, m.step)[:, :m.width]
            core.on_lane(stamp(m), gp.yellow_points(mask), gp.view_points, gp.white_points(mask))
        elif t == '/scan' and activated:
            ts = stamp(m)
            core.process_scan(ts, m.ranges, m.angle_min, m.angle_increment, ts + 0.02)
            core.control(ts + 0.02, 0.04)
            veh = core.vehicle(ts + 0.02)
            pa = core.path
            rem = None if pa is None or veh is None else pa.end_s - veh[0]
            rows.append({'t': ts, 'has': pa is not None, 'complete': bool(pa is not None and pa.complete),
                         'remain': rem, 'state': core.state, 'reason': core.reason,
                         'plan': core.last_plan_reason, 'fail': core.last_fail,
                         'veh': None if veh is None else [round(v, 3) for v in veh[:3]],
                         'pd': None if pa is None or veh is None else
                         [round(float(np.interp(veh[0] + k, pa.s, pa.d)), 2) for k in (0.3, 0.6, 0.9, 1.2)]})
    t0 = rows[0]['t']
    n = len(rows)
    has = sum(r['has'] for r in rows)
    long_ = sum(1 for r in rows if r['remain'] is not None and r['remain'] >= 0.8)
    print(f"{os.path.basename(a.bag)} 스캔 {n}: 경로 있음 {has / n:.0%}, 남은 경로 >=0.8 m {long_ / n:.0%}, "
          f"재계획 {core.replans}, 경로변경 {core.route_changes}, 실패 {core.plan_failures}, 이유 {core.plan_reasons}")
    print('  탈락 집계', core.fail_checks, 'LiDAR로 버린 차선 점', core.lane_lidar_dropped)
    gaps, cur = [], None
    for r in rows:
        bad = r['remain'] is None or r['remain'] < 0.4
        if bad and cur is None:
            cur = r
        elif not bad and cur is not None:
            gaps.append((cur, r))
            cur = None
    if cur is not None:
        gaps.append((cur, rows[-1]))
    for s, e in gaps:
        if e['t'] - s['t'] >= 0.5:
            f = s['fail'] or {}
            print(f"  끊김 {s['t'] - t0:5.1f}~{e['t'] - t0:5.1f}s 자세 {s['veh']} 탈락 {f.get('rejected')} "
                  f"여유 {f.get('ego_clear')} 박스 {f.get('boxes')}")
    if a.v:
        c = Counter(r['state'] + ':' + r['reason'] for r in rows)
        print('  상태 분포', dict(c))
        for r in rows:
            if a.win[0] <= r['t'] - t0 <= a.win[1]:
                print(f"   {r['t'] - t0:5.1f} {r['state']}:{r['reason']} plan={r['plan']} fail={r['fail'] and {k: r['fail'].get(k) for k in ('start', 'rejected', 'ego_clear', 'boxes')}} remain={r['remain'] and round(r['remain'], 2)} veh={r['veh']} 경로d(+0.3..1.2)={r['pd']}")


if __name__ == '__main__':
    main()
