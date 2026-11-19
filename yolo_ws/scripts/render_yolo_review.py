#!/usr/bin/env python3
"""Render a side-by-side YOLO review video from a recorded sensor bag.

Left: recorded camera. Right: YOLO re-run at 10 Hz with the same thresholds,
2-frame confirmation and box colours as yolo_detect.py (imported directly).
Bottom: /cmd_vel, /mission/lane, /odometry/filtered, /rosout, YOLO topic values,
and a /mux/active_source timeline.

Usage (on the car, ROS sourced, dl_env site-packages on PYTHONPATH):
  python3 render_yolo_review.py BAG_DIR OUT.mp4 [--model best.engine] [--fps 30]
"""
import argparse
import bisect
import os
import sys
import time
from datetime import datetime, timedelta, timezone
from pathlib import Path

import cv2
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

SRC = Path(__file__).resolve().parents[1] / 'src'
sys.path.insert(0, str(SRC))
import yolo_detect as yd  # noqa: E402

KST = timezone(timedelta(hours=9))
W, H = 1280, 720
PANEL_W, PANEL_H = 624, 468
LEFT_X, RIGHT_X, PANEL_Y = 8, 648, 66
BG = (18, 18, 18)
FG = (235, 235, 235)
DIM = (150, 150, 150)
ACCENT = (230, 200, 90)  # BGR: light blue-ish cyan
TRUE_C = (90, 200, 255)
FALSE_C = (120, 220, 120)
FONT = cv2.FONT_HERSHEY_SIMPLEX
TOPICS = ['/camera/color/image_raw/compressed', '/cmd_vel', '/mission/lane',
          '/odometry/filtered', '/mux/active_source', '/rosout']


def read_bag(bag_dir):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    reader.set_filter(rosbag2_py.StorageFilter(topics=[t for t in TOPICS if t in types]))
    classes = {t: get_message(types[t]) for t in TOPICS if t in types}
    data = {t: [] for t in TOPICS}
    while reader.has_next():
        topic, raw, t = reader.read_next()
        msg = deserialize_message(raw, classes[topic])
        if topic == TOPICS[0]:
            data[topic].append((t, bytes(msg.data)))
        elif topic == '/cmd_vel':
            data[topic].append((t, (msg.linear.x, msg.angular.z)))
        elif topic == '/mission/lane':
            data[topic].append((t, (msg.drive.speed, msg.drive.steering_angle)))
        elif topic == '/odometry/filtered':
            data[topic].append((t, msg.twist.twist.linear.x))
        elif topic == '/mux/active_source':
            data[topic].append((t, msg.data))
        elif topic == '/rosout':
            data[topic].append((t, f'{msg.name}: {msg.msg}'))
    for v in data.values():
        v.sort(key=lambda x: x[0])
    return data


class Series:
    def __init__(self, items):
        self.t = [x[0] for x in items]
        self.v = [x[1] for x in items]

    def at(self, t, max_age=None):
        i = bisect.bisect_right(self.t, t) - 1
        if i < 0 or (max_age is not None and t - self.t[i] > max_age):
            return None
        return self.v[i]

    def index(self, t):
        return bisect.bisect_right(self.t, t) - 1


def classify(rows, names, width, height):
    """Same as YoloDetect.classify_detections (kept outside the ROS node)."""
    detected, boxes = {}, []
    for row in rows:
        if not np.isfinite(row).all():
            continue
        name = yd.detection_class_name(row[5], names)
        if name is None:
            continue
        x0, y0, x1, y1 = row[:4]
        x0, x1 = np.clip((int(x0), int(x1)), 0, width)
        y0, y1 = np.clip((int(y0), int(y1)), 0, height)
        bw, bh = max(0, int(x1 - x0)), max(0, int(y1 - y0))
        eligible = bool(yd.filter_detections([row], names, width, height))
        boxes.append({'class': name, 'xyxy': (int(x0), int(y0), int(x1), int(y1)),
                      'width': bw, 'height': bh, 'area': bw * bh,
                      'eligible': eligible, 'confidence': float(row[4])})
        if eligible:
            detected[name] = max(detected.get(name, 0.), float(row[4]))
    return detected, boxes


def run_yolo(cam, model_path, t_start, t_end):
    """Emulate the node's 10 Hz timer on bag receive time; return per-tick results."""
    from ultralytics import YOLO
    model = YOLO(str(model_path), task='detect')
    names = model.names
    min_conf = min(t['confidence'] for t in yd.CLASS_THRESHOLDS.values())
    model.predict(source=np.zeros((yd.IMAGE_SIZE, yd.IMAGE_SIZE, 3), np.uint8), imgsz=yd.IMAGE_SIZE,
                  rect=False, conf=min_conf, device='0', verbose=False, save=False)
    state = yd.DetectionState()
    presence = dict.fromkeys(yd.PRESENCE_TOPICS, False)
    last_idx, last_processed = None, -1e30
    ticks = []  # (t, cam_idx or None, boxes, values)
    period = int(1e9 / yd.PUBLISH_HZ)
    max_age = int(yd.MAX_IMAGE_AGE_SEC * 1e9)
    n_ticks = (t_end - t_start) // period + 1
    started = time.monotonic()
    for k in range(n_ticks):
        t = t_start + k * period
        i = cam.index(t)
        if i < 0 or t - cam.t[i] > max_age:
            state.reset()
            presence = dict.fromkeys(yd.PRESENCE_TOPICS, False)
            ticks.append((t, None, [], dict(presence)))
            continue
        if i == last_idx:
            if t - last_processed > max_age:
                state.reset()
                presence = dict.fromkeys(yd.PRESENCE_TOPICS, False)
            ticks.append((t, i, ticks[-1][2] if ticks else [], dict(presence)))
            continue
        last_idx = i
        if t - last_processed > max_age:
            state.reset()
        image = cv2.imdecode(np.frombuffer(cam.v[i], np.uint8), cv2.IMREAD_COLOR)
        result = model.predict(source=image, imgsz=yd.IMAGE_SIZE, rect=False, conf=min_conf,
                               device='0', verbose=False, save=False)[0]
        rows = result.boxes.data.cpu().numpy() if result.boxes is not None else []
        detected, boxes = classify(rows, names, image.shape[1], image.shape[0])
        last_processed = t
        values = state.update(detected)
        presence = {tp: values[tp] for tp in yd.PRESENCE_TOPICS}
        ticks.append((t, i, boxes, values))
        if k % 200 == 0:
            print(f'  yolo {k}/{n_ticks} ({time.monotonic() - started:.0f}s)', flush=True)
    return ticks


def put(img, text, org, scale=0.55, colour=FG, thick=1):
    cv2.putText(img, text, org, FONT, scale, colour, thick, cv2.LINE_AA)


def fit(image):
    return cv2.resize(image, (PANEL_W, PANEL_H), interpolation=cv2.INTER_AREA)


def mux_events(mux):
    events, prev = [], None
    for t, v in zip(mux.t, mux.v):
        if v != prev:
            events.append((t, v))
            prev = v
    return events


def bool_text(v):
    return ('TRUE', TRUE_C) if v else ('FALSE', FALSE_C)


def render(args):
    bag_dir = Path(args.bag)
    print('reading bag ...', flush=True)
    data = read_bag(bag_dir)
    cam = Series(data[TOPICS[0]])
    if not cam.t:
        raise SystemExit('no camera frames in bag')
    series = {t: Series(data[t]) for t in TOPICS[1:]}
    all_t = [s.t[0] for s in [cam, *series.values()] if s.t] + [s.t[-1] for s in [cam, *series.values()] if s.t]
    t0, t1 = min(all_t), max(all_t)
    dur = (t1 - t0) / 1e9
    print(f'duration {dur:.2f}s, camera {len(cam.t)} frames', flush=True)
    print('running YOLO ...', flush=True)
    ticks = run_yolo(cam, args.model, t0, t1)
    tick_t = [x[0] for x in ticks]
    events = mux_events(series['/mux/active_source'])

    out = Path(args.out)
    tmp = out.with_suffix('.part.mp4')
    writer = cv2.VideoWriter(str(tmp), cv2.VideoWriter_fourcc(*'mp4v'), args.fps, (W, H))
    if not writer.isOpened():
        raise SystemExit('VideoWriter failed')
    n_frames = int(dur * args.fps) + 1
    cache = {}

    def decode(i):
        if i not in cache:
            if len(cache) > 8:
                cache.pop(next(iter(cache)))
            cache[i] = cv2.imdecode(np.frombuffer(cam.v[i], np.uint8), cv2.IMREAD_COLOR)
        return cache[i]

    # Static timeline layout.
    bar_x0, bar_x1, bar_y = 14, W - 14, 640
    def tx(t):
        return int(bar_x0 + (bar_x1 - bar_x0) * (t - t0) / max(1, t1 - t0))

    # Greedy rows so event labels never overlap (3 rows, drop if no room).
    event_labels, row_end = [], [-1e9] * 3
    for et, ev in events:
        label = f'{(et - t0) / 1e9:.2f}s {ev}'
        tw = cv2.getTextSize(label, FONT, 0.42, 1)[0][0]
        lx = min(max(bar_x0, tx(et) - 30), W - 8 - tw)
        for row in range(3):
            if lx > row_end[row] + 6:
                row_end[row] = lx + tw
                event_labels.append((et, ev, lx, row, label))
                break
    last_lr, last_sig = None, None
    started = time.monotonic()
    for f in range(n_frames):
        t = t0 + int(f * 1e9 / args.fps)
        frame = np.full((H, W, 3), BG, np.uint8)
        # Top bar.
        put(frame, f'{(t - t0) / 1e9:06.2f} / {dur:.2f} s', (12, 30), 0.8, FG, 2)
        mux = series['/mux/active_source'].at(t)
        put(frame, '/mux/active_source', (300, 30), 0.75, ACCENT, 2)
        put(frame, mux or '--', (590, 30), 0.75, ACCENT, 2)
        clock = datetime.fromtimestamp(t / 1e9, KST).strftime('%H:%M:%S.%f')[:-3] + ' KST'
        put(frame, clock, (W - 230, 30), 0.6)
        # Left panel: recorded camera.
        ci = cam.index(t)
        put(frame, 'Recorded camera', (LEFT_X, PANEL_Y - 8))
        if ci >= 0:
            frame[PANEL_Y:PANEL_Y + PANEL_H, LEFT_X:LEFT_X + PANEL_W] = fit(decode(ci))
        # Right panel: YOLO replay.
        k = bisect.bisect_right(tick_t, t) - 1
        put(frame, f'YOLO replay | {yd.PUBLISH_HZ:g} Hz', (RIGHT_X, PANEL_Y - 8))
        values = {}
        if k >= 0:
            _, yi, boxes, values = ticks[k]
            if yi is not None:
                put(frame, f'Camera frame +{(cam.t[yi] - t0) / 1e9:.3f}s', (RIGHT_X + 395, PANEL_Y - 8), 0.5)
                frame[PANEL_Y:PANEL_Y + PANEL_H, RIGHT_X:RIGHT_X + PANEL_W] = fit(
                    yd.draw_bbox_publications(decode(yi), boxes, values))
            else:
                put(frame, 'no fresh camera frame', (RIGHT_X + 180, PANEL_Y + PANEL_H // 2), 0.7, DIM, 2)
            if yd.LR_TOPIC in values:
                last_lr = (ticks[k][0], values[yd.LR_TOPIC])
            if yd.SIGNAL_TOPIC in values:
                last_sig = (ticks[k][0], values[yd.SIGNAL_TOPIC])
        # Bottom left: control topics.
        y = 562
        cv = series['/cmd_vel'].at(t)
        put(frame, '/cmd_vel', (LEFT_X, y))
        put(frame, f'v {cv[0]:+.2f} m/s   wz {cv[1]:+.2f} rad/s' if cv else '--', (150, y))
        ml = series['/mission/lane'].at(t)
        put(frame, '/mission/lane', (LEFT_X, y + 26))
        put(frame, f'v {ml[0]:+.2f} m/s   steer {ml[1]:+.2f} rad' if ml else '--', (150, y + 26))
        od = series['/odometry/filtered'].at(t)
        put(frame, '/odometry/filtered', (LEFT_X, y + 52))
        put(frame, f'v {od:+.2f} m/s' if od is not None else '--', (200, y + 52))
        ro = series['/rosout'].at(t, max_age=3e9)
        ro_txt = f'Recorded /rosout: {ro or "--"}'
        while len(ro_txt) > 20 and cv2.getTextSize(ro_txt, FONT, 0.42, 1)[0][0] > PANEL_W:
            ro_txt = ro_txt[:-4] + '..'
        put(frame, ro_txt, (LEFT_X, y + 72), 0.42, DIM)
        # Bottom right: YOLO topic values.
        put(frame, '/yolo/{slow_sign, crosswalk_sign, parking_sign}', (RIGHT_X, y))
        x = RIGHT_X
        for i, tp in enumerate(yd.PRESENCE_TOPICS):
            txt, col = bool_text(values.get(tp, False))
            put(frame, txt, (x, y + 26), 0.6, col, 2)
            x += cv2.getTextSize(txt, FONT, 0.6, 2)[0][0] + 12
            if i < 2:
                put(frame, '/', (x, y + 26), 0.6, DIM)
                x += 26
        def held(last, true_txt, false_txt):
            if last is None:
                return '--', DIM
            age = (t - last[0]) / 1e9
            txt = true_txt if last[1] else false_txt
            return (txt, TRUE_C) if age < 0.15 else (f'{txt} ({age:.1f}s ago)', DIM)
        lr_txt, lr_col = held(last_lr, 'RIGHT', 'LEFT')
        sg_txt, sg_col = held(last_sig, 'GREEN', 'RED')
        put(frame, '/yolo/LR_sign', (RIGHT_X, y + 52))
        put(frame, lr_txt, (RIGHT_X + 140, y + 52), 0.55, lr_col, 2 if lr_col == TRUE_C else 1)
        put(frame, '/yolo/signal', (RIGHT_X + 330, y + 52))
        put(frame, sg_txt, (RIGHT_X + 445, y + 52), 0.55, sg_col, 2 if sg_col == TRUE_C else 1)
        put(frame, 'Colored box = confirmed topic | Gray = unconfirmed or filtered',
            (RIGHT_X, y + 72), 0.42, DIM)
        # Timeline.
        cv2.line(frame, (bar_x0, bar_y), (bar_x1, bar_y), (90, 90, 90), 3, cv2.LINE_AA)
        cv2.line(frame, (bar_x0, bar_y), (tx(t), bar_y), ACCENT, 3, cv2.LINE_AA)
        for et, ev, lx, row, label in event_labels:
            cv2.circle(frame, (tx(et), bar_y), 4, ACCENT if et <= t else (170, 170, 170), -1, cv2.LINE_AA)
            put(frame, label, (lx, bar_y + 20 + row * 18), 0.42, ACCENT if et <= t else DIM)
        cv2.circle(frame, (tx(t), bar_y), 6, (255, 255, 255), -1, cv2.LINE_AA)
        writer.write(frame)
        if f % 600 == 0:
            print(f'  render {f}/{n_frames} ({time.monotonic() - started:.0f}s)', flush=True)
    writer.release()
    os.replace(tmp, out)
    print(f'done: {out}', flush=True)


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('bag')
    p.add_argument('out')
    p.add_argument('--model', default=str(yd.MODEL_PATH))
    p.add_argument('--fps', type=float, default=30.)
    render(p.parse_args())


if __name__ == '__main__':
    main()
