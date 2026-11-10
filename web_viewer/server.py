#!/usr/bin/env python3
"""Read ROS images on the robot and serve JPEG/MJPEG over HTTP."""
import argparse
import base64
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import signal
import threading
import time
from urllib.parse import urlparse

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import CompressedImage, Image
from parking_bridge import ParkingState
from roi_bag import BagImages
from roi_preview import defaults as roi_defaults, render_preview, validate as validate_roi


SOURCES = (
    ('camera', '카메라', '/camera/color/image_raw/compressed', CompressedImage),
    ('yolo', 'YOLO 검출', '/yolo/debug/compressed', CompressedImage),
    ('mask', '차선 마스크', '/lane_fusion/deep_mask', Image),
    ('lane', '차선 디버그', '/lane_fusion/debug', Image),
    ('roi_camera', 'ROI 편집용 원본', '/camera/color/image_raw', Image),
    ('white_roi', '서행·방지턱 ROI', '/slow/white_roi/debug/compressed', CompressedImage),
    # Latched: one picture per fork state change (lane_fusion_node), kept until the next one.
    ('fork', '갈림길 스냅샷', '/lane_fusion/fork_debug', CompressedImage),
)
DIST = Path(__file__).resolve().parent / 'dist'
LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def latest_fork_file():
    # lane_fusion_node also saves every fork snapshot here; after a node
    # restart (latched picture lost) the newest file is shown instead.
    folder = Path(os.environ.get('FORK_SNAPSHOT_DIR', Path.home() / 'fork_snapshots'))
    files = sorted(folder.glob('fork_*.jpg'), key=lambda f: f.stat().st_mtime) if folder.is_dir() else []
    return files[-1] if files else None


def jpeg_dimensions(data):
    if data[:2] != b'\xff\xd8':
        raise ValueError('JPEG 헤더가 없는 압축 영상입니다.')
    index = 2
    while index + 8 < len(data):
        if data[index] != 255:
            index += 1
            continue
        marker = data[index + 1]
        index += 2
        if marker in (0xD8, 0xD9) or 0xD0 <= marker <= 0xD7:
            continue
        length = int.from_bytes(data[index:index + 2], 'big')
        if length < 2:
            break
        if marker in (0xC0, 0xC1, 0xC2, 0xC3, 0xC5, 0xC6, 0xC7,
                      0xC9, 0xCA, 0xCB, 0xCD, 0xCE, 0xCF):
            return (int.from_bytes(data[index + 5:index + 7], 'big'),
                    int.from_bytes(data[index + 3:index + 5], 'big'))
        index += length
    return 0, 0


def raw_jpeg(message):
    channels = {'mono8': 1, '8UC1': 1, 'bgr8': 3, 'rgb8': 3,
                'bgra8': 4, 'rgba8': 4}.get(message.encoding)
    if channels is None:
        raise ValueError('지원하지 않는 영상 형식: ' + message.encoding)
    height, width, step = message.height, message.width, message.step
    if not height or not width or step < width * channels or len(message.data) < height * step:
        raise ValueError('영상 크기 또는 행 길이가 올바르지 않습니다.')
    image = np.frombuffer(message.data, dtype=np.uint8, count=height * step)
    image = image.reshape(height, step)[:, :width * channels]
    image = image.reshape((height, width) if channels == 1 else (height, width, channels))
    if message.encoding == 'rgb8':
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    elif message.encoding == 'rgba8':
        image = cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)
    elif channels == 4:
        image = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
    ok, encoded = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 82])
    if not ok:
        raise ValueError('JPEG 변환에 실패했습니다.')
    return encoded.tobytes(), width, height


class Stream:
    def __init__(self, key, title, topic, rate):
        self.key, self.title, self.topic = key, title, topic
        self.interval = 1.0 / rate
        self.condition = threading.Condition()
        self.jpeg = None
        self.sequence = 0
        self.updated = 0.0
        self.width = self.height = self.clients = 0
        self.times = deque(maxlen=50)
        self.error = ''
        self.preview_until = 0.0

    def active(self):
        # The fork snapshot is latched and rare: always subscribed (no stream needed).
        return self.key == 'fork' or self.clients > 0 or time.monotonic() < self.preview_until

    def receive(self, message):
        # A parking-only viewer should not encode unused lane/debug streams.
        if not self.active():
            return
        now = time.monotonic()
        if now - self.updated < self.interval:
            return
        try:
            if isinstance(message, CompressedImage):
                data = bytes(message.data)
                width, height = jpeg_dimensions(data)
            else:
                data, width, height = raw_jpeg(message)
        except (ValueError, cv2.error) as error:
            with self.condition:
                self.error = str(error)
            return
        with self.condition:
            self.jpeg, self.width, self.height = data, width, height
            self.updated = now
            self.sequence += 1
            self.times.append(now)
            self.error = ''
            self.condition.notify_all()

    def status(self):
        if self.key == 'fork' and self.jpeg is None and latest_fork_file():
            f = latest_fork_file()
            width, height = jpeg_dimensions(f.read_bytes())
            return dict(id=self.key, title=self.title, topic=self.topic + ' (saved file)', status='stale',
                        age_ms=round((time.time() - f.stat().st_mtime) * 1000), fps=0, width=width,
                        height=height, bytes=f.stat().st_size, clients=self.clients, error='')
        with self.condition:
            age = time.monotonic() - self.updated if self.jpeg else None
            fps = ((len(self.times) - 1) / (self.times[-1] - self.times[0])
                   if len(self.times) > 1 else 0)
            return dict(id=self.key, title=self.title, topic=self.topic,
                        status='waiting' if age is None else ('live' if age < 2 else 'stale'),
                        age_ms=None if age is None else round(age * 1000),
                        fps=round(fps, 1) if age is not None and age < 2 else 0,
                        width=self.width, height=self.height, bytes=len(self.jpeg or b''),
                        clients=self.clients, error=self.error)


class Bridge(Node):
    def __init__(self, streams, parking):
        super().__init__('autorace_web_viewer')
        self.streams = streams
        self.image_subscriptions = {}
        self.image_subscription_timer = self.create_timer(.5, self.update_image_subscriptions)
        parking.attach(self)

    def update_image_subscriptions(self):
        # Avoid even deserializing raw debug images unless a browser displays them.
        for key, _, topic, kind in SOURCES:
            if self.streams[key].active() and key not in self.image_subscriptions:
                self.image_subscriptions[key] = self.create_subscription(
                    kind, topic, self.streams[key].receive,
                    LATCHED if key == 'fork' else qos_profile_sensor_data)
            elif not self.streams[key].active() and key in self.image_subscriptions:
                self.destroy_subscription(self.image_subscriptions.pop(key))


class Handler(BaseHTTPRequestHandler):
    protocol_version = 'HTTP/1.1'

    def log_message(self, *args):
        pass

    def send_body(self, body, content_type, status=200, extra=None):
        self.send_response(status)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.send_header('X-Content-Type-Options', 'nosniff')
        for key, value in (extra or {}).items():
            self.send_header(key, value)
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        try:
            self.route()
        except (BrokenPipeError, ConnectionResetError, TimeoutError):
            self.close_connection = True

    def route(self):
        path = urlparse(self.path).path
        if path in ('/', '/app.js', '/style.css', '/parking', '/parking/', '/parking.js', '/parking.css',
                    '/roi', '/roi/', '/roi.js', '/roi.css', '/roi-sample.jpg'):
            name, content_type = {
                '/': ('index.html', 'text/html; charset=utf-8'),
                '/app.js': ('app.js', 'text/javascript; charset=utf-8'),
                '/style.css': ('style.css', 'text/css; charset=utf-8'),
                '/parking': ('parking.html', 'text/html; charset=utf-8'),
                '/parking/': ('parking.html', 'text/html; charset=utf-8'),
                '/parking.js': ('parking.js', 'text/javascript; charset=utf-8'),
                '/parking.css': ('parking.css', 'text/css; charset=utf-8'),
                '/roi': ('roi.html', 'text/html; charset=utf-8'),
                '/roi/': ('roi.html', 'text/html; charset=utf-8'),
                '/roi.js': ('roi.js', 'text/javascript; charset=utf-8'),
                '/roi.css': ('roi.css', 'text/css; charset=utf-8'),
                '/roi-sample.jpg': ('roi-sample.jpg', 'image/jpeg'),
            }[path]
            self.send_body((DIST / name).read_bytes(), content_type)
        elif path == '/api/roi/bag':
            try:
                payload = self.server.roi_bag.latest()
                self.send_body(json.dumps(payload, ensure_ascii=False).encode(), 'application/json; charset=utf-8')
            except ValueError as error:
                self.send_body(json.dumps({'error': str(error)}, ensure_ascii=False).encode(), 'application/json; charset=utf-8', 404)
        elif path == '/api/roi/config':
            self.send_body(json.dumps(roi_defaults(), ensure_ascii=False).encode(),
                           'application/json; charset=utf-8')
        elif path == '/api/parking':
            payload = self.server.parking.snapshot()
            payload['camera'] = self.server.streams['camera'].status()
            self.send_body(json.dumps(payload, ensure_ascii=False, allow_nan=False).encode(),
                           'application/json; charset=utf-8')
        elif path == '/api/state':
            payload = dict(streams=[stream.status() for stream in self.server.streams.values()],
                           uptime_s=round(time.monotonic() - self.server.started),
                           domain_id=os.environ.get('ROS_DOMAIN_ID', '0'))
            self.send_body(json.dumps(payload, ensure_ascii=False).encode(), 'application/json; charset=utf-8')
        elif path.startswith('/snapshot/') and path.endswith('.jpg'):
            key = path[len('/snapshot/'):-4]
            stream = self.server.streams.get(key)
            if stream is None:
                self.send_body(b'Unknown image', 'text/plain', 404)
                return
            with stream.condition:
                data, age = stream.jpeg, time.monotonic() - stream.updated
            if data is None and key == 'fork' and latest_fork_file():
                data, age = latest_fork_file().read_bytes(), 0.
            if data is None or (age >= 2 and key != 'fork'):  # fork: last picture stays valid
                self.send_body('최신 영상이 없습니다.'.encode(), 'text/plain; charset=utf-8', 503)
                return
            filename = f'{key}-{time.strftime("%Y%m%d-%H%M%S")}.jpg'
            self.send_body(data, 'image/jpeg', extra={'Content-Disposition': f'attachment; filename="{filename}"'})
        elif path.startswith('/stream/') and path.endswith('.mjpg'):
            key = path[len('/stream/'):-5]
            stream = self.server.streams.get(key)
            if stream is None:
                self.send_body(b'Unknown image', 'text/plain', 404)
            elif not self.server.slots.acquire(blocking=False):
                self.send_body(b'Too many viewers', 'text/plain', 503)
            else:
                try:
                    self.stream(stream)
                finally:
                    self.server.slots.release()
        elif path == '/favicon.ico':
            self.send_body(b'', 'image/x-icon', 204)
        else:
            self.send_body(b'Not found', 'text/plain', 404)

    def do_POST(self):
        if urlparse(self.path).path != '/api/roi/preview':
            self.send_body(b'Not found', 'text/plain', 404)
            return
        try:
            length = int(self.headers.get('Content-Length', '0'))
            if not 0 < length <= 8_000_000:
                self.close_connection = True
                raise ValueError('요청 이미지가 너무 큽니다. 6MB 이하 이미지를 사용하세요.')
            payload = json.loads(self.rfile.read(length))
            if not isinstance(payload, dict):
                raise ValueError('요청 형식이 올바르지 않습니다.')
            params = payload.get('params')
            validate_roi(params)
            source = payload.get('source')
            if source == 'live':
                candidates = []
                for key in ('camera', 'roi_camera'):
                    stream = self.server.streams[key]
                    stream.preview_until = time.monotonic() + 4.
                    with stream.condition:
                        if stream.jpeg and time.monotonic() - stream.updated < 2.:
                            candidates.append((stream.updated, stream.jpeg, stream.topic))
                if not candidates:
                    self.send_body(json.dumps({'error': '실시간 카메라를 기다립니다. 센서와 ROS 도메인을 확인하거나 bag 샘플을 사용하세요.'}, ensure_ascii=False).encode(),
                                   'application/json; charset=utf-8', 503)
                    return
                updated, data, topic = max(candidates, key=lambda item: item[0])
                metadata = dict(source='live', source_label=topic,
                                age_ms=round((time.monotonic()-updated)*1000))
            elif source == 'bag':
                data, metadata = self.server.roi_bag.frame(payload.get('frame_index'), payload.get('bag_name'))
            elif source == 'sample':
                data = (DIST / 'roi-sample.jpg').read_bytes()
                metadata = dict(source='sample', source_label=roi_defaults()['sample'], age_ms=None)
            elif source == 'upload':
                encoded = payload.get('image', '')
                if not isinstance(encoded, str) or not encoded.startswith(('data:image/jpeg;base64,', 'data:image/png;base64,')):
                    raise ValueError('JPEG 또는 PNG 이미지를 선택하세요.')
                data = base64.b64decode(encoded.split(',', 1)[1], validate=True)
                metadata = dict(source='upload', source_label='업로드 이미지', age_ms=None)
            else:
                raise ValueError('영상 소스를 선택하세요.')
            result = render_preview(data, params)
            result.update(metadata)
            self.send_body(json.dumps(result, ensure_ascii=False, allow_nan=False).encode(),
                           'application/json; charset=utf-8')
        except (ValueError, TypeError, cv2.error) as error:
            self.send_body(json.dumps({'error': str(error)}, ensure_ascii=False).encode(),
                           'application/json; charset=utf-8', 400)
        except (BrokenPipeError, ConnectionResetError, TimeoutError):
            self.close_connection = True

    def stream(self, source):
        self.connection.settimeout(5)
        self.send_response(200)
        self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
        self.send_header('Cache-Control', 'no-store, no-cache, must-revalidate')
        self.send_header('Connection', 'close')
        self.end_headers()
        self.close_connection = True
        sequence = -1
        with source.condition:
            source.clients += 1
        try:
            while not self.server.stopping.is_set():
                with source.condition:
                    source.condition.wait_for(
                        lambda: source.sequence != sequence or self.server.stopping.is_set(), timeout=2)
                    data, next_sequence = source.jpeg, source.sequence
                if not data or next_sequence == sequence:
                    self.wfile.write(b'\r\n')
                    self.wfile.flush()
                    sequence = next_sequence
                    continue
                sequence = next_sequence
                header = f'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: {len(data)}\r\n\r\n'.encode()
                self.wfile.write(header + data + b'\r\n')
                self.wfile.flush()
        finally:
            with source.condition:
                source.clients -= 1


def main():
    # Leave the final CPU available for the vehicle's localization processes.
    if hasattr(os, 'sched_getaffinity'):
        cpus = sorted(os.sched_getaffinity(0))
        if len(cpus) >= 4:
            os.sched_setaffinity(0, cpus[:-1])
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='0.0.0.0')
    parser.add_argument('--port', type=int, default=8080)
    parser.add_argument('--max-fps', type=float, default=10)
    args = parser.parse_args()
    if not 1 <= args.max_fps <= 30:
        parser.error('--max-fps must be between 1 and 30')
    http = ThreadingHTTPServer((args.host, args.port), Handler)
    http.daemon_threads = True
    http.stopping = threading.Event()
    http.started = time.monotonic()
    http.slots = threading.BoundedSemaphore(12)
    http.streams = {key: Stream(key, title, topic, args.max_fps) for key, title, topic, _ in SOURCES}
    http.parking = ParkingState()
    http.roi_bag = BagImages(Path(__file__).resolve().parents[2])
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: http.stopping.set())
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    bridge = Bridge(http.streams, http.parking)

    def spin():
        try:
            rclpy.spin(bridge)
        except ExternalShutdownException:
            pass
        except Exception as error:
            print(f'ROS image receiver stopped: {error}', flush=True)
        finally:
            http.stopping.set()

    receiver = threading.Thread(target=spin, name='ros-image-receiver', daemon=True)
    receiver.start()
    http.timeout = .5
    print(f'AutoRace Viewer listening on {args.host}:{args.port}; ROS_DOMAIN_ID={os.environ.get("ROS_DOMAIN_ID", "0")}', flush=True)
    try:
        while not http.stopping.is_set():
            http.handle_request()
    finally:
        http.stopping.set()
        for stream in http.streams.values():
            with stream.condition:
                stream.condition.notify_all()
        http.server_close()
        if rclpy.ok():
            rclpy.shutdown()
        receiver.join(timeout=3)
        bridge.destroy_node()


if __name__ == '__main__':
    main()
