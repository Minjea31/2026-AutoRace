#!/usr/bin/env python3
"""Read ROS images on the robot and serve JPEG/MJPEG over HTTP."""
import argparse
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
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import CompressedImage, Image


SOURCES = (
    ('camera', '카메라', '/camera/color/image_raw/compressed', CompressedImage),
    ('yolo', 'YOLO 검출', '/yolo/debug/compressed', CompressedImage),
    ('mask', '차선 마스크', '/lane_fusion/deep_mask', Image),
    ('lane', '차선 디버그', '/lane_fusion/debug', Image),
    ('white_roi', '서행·방지턱 ROI', '/slow/white_roi/debug/compressed', CompressedImage),
)
DIST = Path(__file__).resolve().parent / 'dist'


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

    def receive(self, message):
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
    def __init__(self, streams):
        super().__init__('autorace_web_viewer')
        self.image_subscriptions = [
            self.create_subscription(kind, topic, streams[key].receive, qos_profile_sensor_data)
            for key, _, topic, kind in SOURCES
        ]


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
        if path in ('/', '/app.js', '/style.css'):
            name, content_type = {
                '/': ('index.html', 'text/html; charset=utf-8'),
                '/app.js': ('app.js', 'text/javascript; charset=utf-8'),
                '/style.css': ('style.css', 'text/css; charset=utf-8'),
            }[path]
            self.send_body((DIST / name).read_bytes(), content_type)
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
            if data is None or age >= 2:
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
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: http.stopping.set())
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    bridge = Bridge(http.streams)

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
