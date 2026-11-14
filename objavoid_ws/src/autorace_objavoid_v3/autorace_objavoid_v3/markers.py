"""RViz 표시(odom 프레임): 장애물·추정 크기, Gap 후보, 선택 경로, 차량·목표점, 상태·정지 원인, 도로 경계."""
import math

import numpy as np
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray


def _m(ns, mid, mtype, stamp, color, scale=(0.02, 0.02, 0.02)):
    m = Marker()
    m.header.frame_id = 'odom'
    m.header.stamp = stamp
    m.ns, m.id, m.type, m.action = ns, mid, mtype, Marker.ADD
    m.pose.orientation.w = 1.0
    m.scale.x, m.scale.y, m.scale.z = (float(v) for v in scale)  # ROS 메시지는 실수형만 받음(정수 0이면 배열 전체 실패)
    m.color.r, m.color.g, m.color.b, m.color.a = (float(v) for v in color)
    return m


def _pt(frame, s, d, z=0.02):
    x, y = frame.to_odom(s, d)
    return Point(x=float(x), y=float(y), z=z)


def _rect(m, frame, s0, s1, d0, d1, z=0.03):
    for s, d in ((s0, d0), (s1, d0), (s1, d1), (s0, d1), (s0, d0)):
        m.points.append(_pt(frame, s, d, z))


def build_markers(core, stamp, t):
    f, p = core.frame, core.p
    arr = MarkerArray()
    clear = Marker()
    clear.action = Marker.DELETEALL
    arr.markers.append(clear)

    # 장애물 감지: 이번 스캔의 도로 안 LiDAR 점(분홍)과 그 스캔의 검출 박스(초록), 추적 박스(빨강/주황, 굵게)
    pts = getattr(core, 'last_points', None)
    if pts is not None and len(pts):
        pm = _m('lidar_roi_points', 0, Marker.POINTS, stamp, (1.0, 0.3, 0.8, 1.0), (0.025, 0.025, 0))
        pm.points = [_pt(f, s_, d_, 0.06) for s_, d_ in pts]
        arr.markers.append(pm)
    for k, b in enumerate(getattr(core, 'last_boxes', []) or []):
        dm = _m('scan_detections', k, Marker.LINE_STRIP, stamp, (0.2, 1.0, 0.3, 0.9), (0.01, 0, 0))
        _rect(dm, f, b.s0, b.s1, b.d0, b.d1, 0.05)
        arr.markers.append(dm)
    for tr in core.tracker.tracks:
        b = tr.box
        col = (1.0, 0.1, 0.1, 1.0) if tr.confirmed else (1.0, 0.6, 0.0, 0.8)
        m = _m('obstacles', tr.id, Marker.LINE_STRIP, stamp, col, (0.03, 0, 0))
        _rect(m, f, b.s0, b.s1, b.d0, b.d1)
        arr.markers.append(m)
        txt = _m('obstacle_label', tr.id, Marker.TEXT_VIEW_FACING, stamp, (1, 1, 1, 1), (0, 0, 0.06))
        txt.pose.position = _pt(f, b.cs, b.cd, 0.25)
        txt.text = f'#{tr.id} {b.s1 - b.s0:.2f}x{b.d1 - b.d0:.2f}'
        arr.markers.append(txt)

    # Gap 후보: 정거장마다 통과 가능한 실제 통로(초록 선분), 노드(파랑 점)
    last = core.planner.last or {}
    gm = _m('gaps', 0, Marker.LINE_LIST, stamp, (0.1, 0.9, 0.2, 0.9), (0.02, 0, 0))
    for sk, gaps in last.get('gaps', []):
        for a, b in gaps:
            gm.points += [_pt(f, sk, a, 0.01), _pt(f, sk, b, 0.01)]
    arr.markers.append(gm)
    nm = _m('gap_nodes', 0, Marker.POINTS, stamp, (0.2, 0.4, 1.0, 1.0), (0.03, 0.03, 0))
    for sk, nodes in last.get('layers', []):
        for d in nodes:
            nm.points.append(_pt(f, sk, d, 0.015))
    arr.markers.append(nm)

    # 후보 경로(통과한 간선, 회색)
    cm = _m('candidates', 0, Marker.LINE_LIST, stamp, (0.6, 0.6, 0.6, 0.5), (0.008, 0, 0))
    for c in last.get('cand', [])[:40]:
        for a, b in zip(c[:-1:2], c[1::2]):
            cm.points += [_pt(f, a[0], a[1], 0.02), _pt(f, b[0], b[1], 0.02)]
    arr.markers.append(cm)

    # 선택 경로
    path = core.path
    if path is not None:
        pm = _m('path', 0, Marker.LINE_STRIP, stamp, (1.0, 0.2, 0.6, 1.0), (0.035, 0, 0))  # 선택 경로: 진한 분홍(후보 회색·통로 초록과 구분)
        for s, d in zip(path.s[::2], path.d[::2]):
            pm.points.append(_pt(f, s, d, 0.04))
        arr.markers.append(pm)
        if core.verified_s is not None:
            vm = _m('verified_end', 0, Marker.SPHERE, stamp, (1.0, 1.0, 0.0, 0.9), (0.05, 0.05, 0.05))
            vm.pose.position = _pt(f, core.verified_s, path.d_at(core.verified_s) if
                                   core.verified_s <= path.end_s else path.d[-1], 0.05)
            arr.markers.append(vm)

    # 차량 차체와 목표점(제어 기준점 + 선행 조향 지점)
    veh = core.vehicle(t)
    if veh is not None:
        s, d, psi, _ = veh
        em = _m('ego', 0, Marker.LINE_STRIP, stamp, (0.3, 0.6, 1.0, 1.0), (0.015, 0, 0))
        c, sn = math.cos(psi), math.sin(psi)
        wh = p.body_width / 2
        for ex, ey in ((p.body_front, wh), (p.body_front, -wh), (-p.body_rear, -wh), (-p.body_rear, wh),
                       (p.body_front, wh)):
            em.points.append(_pt(f, s + ex * c - ey * sn, d + ex * sn + ey * c, 0.05))
        arr.markers.append(em)
        info = core.last_info
        if info:
            tm = _m('target', 0, Marker.SPHERE, stamp, (0.1, 0.3, 1.0, 1.0), (0.05, 0.05, 0.05))
            tm.pose.position = _pt(f, info['ref_s'], info['ref_d'], 0.05)
            arr.markers.append(tm)
            if path is not None and 's_ff' in info:
                # LFD: 곡률을 읽는 앞 점(기준점 + 속도 x ff_preview_time). 차 기준점에서 그 점까지 선 + 거리
                sf = min(max(info['s_ff'], float(path.s[0])), float(path.s[-1]))
                df = float(np.interp(sf, path.s, path.d))
                lm = _m('lookahead', 0, Marker.LINE_STRIP, stamp, (1.0, 0.6, 0.0, 1.0), (0.012, 0, 0))
                lm.points = [_pt(f, s, d, 0.06), _pt(f, sf, df, 0.06)]
                arr.markers.append(lm)
                pm2 = _m('lookahead', 1, Marker.SPHERE, stamp, (1.0, 0.6, 0.0, 1.0), (0.045, 0.045, 0.045))
                pm2.pose.position = _pt(f, sf, df, 0.06)
                arr.markers.append(pm2)
                lt = _m('lookahead', 2, Marker.TEXT_VIEW_FACING, stamp, (1.0, 0.7, 0.2, 1.0), (0, 0, 0.06))
                lt.pose.position = _pt(f, sf, df, 0.15)
                lt.text = f'LFD {math.hypot(sf - s, df - d):.2f} m (k_ff {info["k_ff"]:+.2f})'
                arr.markers.append(lt)

    # 도로 경계: 카메라로 관측된 노란 실선 구간(노랑) / 관측 안 된 사전 경계(회색)
    bd = core.boundary
    ss = np.linspace(bd.s_min, bd.s_max, 120)
    right, left = bd.limits(ss)
    known = hasattr(bd, 'known')
    for side, dd in ((-1, right), (1, left)):
        kn = bd.known(ss, side) if known else np.zeros(len(ss), bool)
        ym = _m('lane_boundary_observed', side + 1, Marker.LINE_LIST, stamp, (1.0, 0.85, 0.0, 1.0), (0.02, 0, 0))
        pm = _m('lane_boundary_prior', side + 1, Marker.LINE_LIST, stamp, (0.55, 0.55, 0.55, 0.7), (0.01, 0, 0))
        for i in range(len(ss) - 1):
            (ym if kn[i] and kn[i + 1] else pm).points += [_pt(f, ss[i], dd[i], 0.01), _pt(f, ss[i + 1], dd[i + 1], 0.01)]
        arr.markers += [ym, pm]
    ymap = getattr(core, 'ymap', None)
    wc = getattr(core, 'white_cells', None)
    if wc is not None and len(wc):
        wm = _m('lane_white_cells', 0, Marker.POINTS, stamp, (0.9, 0.9, 1.0, 0.8), (0.02, 0.02, 0))
        wm.points = [Point(x=float(x), y=float(y), z=0.005) for x, y in wc[::2]]
        arr.markers.append(wm)
    if ymap is not None:
        cells = getattr(core, 'lane_cells', None)
        cells = ymap.confirmed() if cells is None else cells
        lm = _m('lane_yellow_cells', 0, Marker.POINTS, stamp, (1.0, 0.5, 0.0, 0.9), (0.03, 0.03, 0))
        lm.points = [Point(x=float(x), y=float(y), z=0.005) for x, y in cells[::2]]
        arr.markers.append(lm)
    em = _m('section_end', 0, Marker.LINE_LIST, stamp, (1.0, 0.3, 1.0, 0.8), (0.01, 0, 0))
    em.points = [_pt(f, p.section_end, -0.8, 0.01), _pt(f, p.section_end, 0.8, 0.01)]
    arr.markers.append(em)

    # 상태·정지 원인
    st = _m('state', 0, Marker.TEXT_VIEW_FACING, stamp, (1, 1, 1, 1), (0, 0, 0.09))
    if veh is not None:
        st.pose.position = _pt(f, veh[0], veh[1] + 0.35, 0.4)
    st.text = (f'{core.state}: {core.reason}  replan={core.replans} passed={core.tracker.passed}'
               + (f'  lane={"aligned" if core.aligned else "unaligned"}' if getattr(core, 'ymap', None) is not None else '')
               + (f'  DONE({core.done_reason})' if core.done_reason else ''))
    arr.markers.append(st)
    return arr


def to_base(arr, pose, frame='base_link'):
    """odom 좌표 마커를 현재 차체(base_link) 기준으로 변환(내부 자세가 드라이버 odom TF와 다를 때)."""
    x0, y0, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)

    def tf(p):
        dx, dy = p.x - x0, p.y - y0
        p.x, p.y = c * dx + s * dy, -s * dx + c * dy

    for m in arr.markers:
        m.header.frame_id = frame
        for p in m.points:
            tf(p)
        if m.type in (Marker.SPHERE, Marker.TEXT_VIEW_FACING):
            tf(m.pose.position)


def preview_markers(core, scan, pose, stamp):
    """활성화 전 미리보기: 지금 자세를 도로 원점으로 보고 이번 스캔의 장애물 검출(점·박스)만 표시."""
    from .perception import detect
    from .road import RoadFrame
    p = core.p
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    lp = (x + c * p.lidar_x - s * p.lidar_y, y + s * p.lidar_x + c * p.lidar_y, yaw + p.lidar_yaw)
    f = RoadFrame(x, y, yaw)
    boxes, pts = detect(scan.ranges, scan.angle_min, scan.angle_increment, lp, f, core.prior, 0.0, p)
    arr = MarkerArray()
    clear = Marker()
    clear.action = Marker.DELETEALL
    arr.markers.append(clear)
    pm = _m('lidar_roi_points', 0, Marker.POINTS, stamp, (1.0, 0.3, 0.8, 1.0), (0.025, 0.025, 0))
    pm.points = [_pt(f, s_, d_, 0.06) for s_, d_ in pts]
    arr.markers.append(pm)
    for k, b in enumerate(boxes):
        dm = _m('scan_detections', k, Marker.LINE_STRIP, stamp, (0.2, 1.0, 0.3, 0.9), (0.015, 0, 0))
        _rect(dm, f, b.s0, b.s1, b.d0, b.d1, 0.05)
        arr.markers.append(dm)
        txt = _m('obstacle_label', k, Marker.TEXT_VIEW_FACING, stamp, (1, 1, 1, 1), (0, 0, 0.06))
        txt.pose.position = _pt(f, b.cs, b.cd, 0.25)
        txt.text = f'{b.s1 - b.s0:.2f}x{b.d1 - b.d0:.2f}'
        arr.markers.append(txt)
    st = _m('state', 0, Marker.TEXT_VIEW_FACING, stamp, (1, 1, 1, 1), (0, 0, 0.09))
    st.pose.position = _pt(f, 0.0, 0.35, 0.4)
    st.text = f'IDLE (미리보기): 검출 {len(boxes)}개 — 활성화하면 추적·경로 계획'
    arr.markers.append(st)
    return arr
