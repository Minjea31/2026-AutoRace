#!/usr/bin/env python3
"""Read recorded camera images and preview six-bound HSV thresholds over HTTP."""
import argparse
import base64
from collections import OrderedDict
import hashlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import importlib.util
import json
import math
from pathlib import Path
import sqlite3
import sys
import threading
from urllib.parse import parse_qs, urlparse

import cv2
import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CompressedImage
import yaml

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
TOPIC = '/camera/color/image_raw/compressed'


class Tuner:
    def __init__(self, roots, repo=REPO):
        self.roots = roots
        detector = repo / 'crosswalk_ws/src/autorace_crosswalk/autorace_crosswalk/white_roi.py'
        spec = importlib.util.spec_from_file_location('_hsv_platform_detector', detector)
        self.module = importlib.util.module_from_spec(spec)
        sys.modules[spec.name] = self.module
        spec.loader.exec_module(self.module)
        params = yaml.safe_load((repo / 'crosswalk_ws/src/autorace_crosswalk/config/crosswalk.yaml').read_text())['crosswalk_node']['ros__parameters']
        self.roi = self.module.WhiteROI(**{
            key: float(params.get('white_roi_' + key, getattr(self.module.WhiteROI(), key)))
            for key in ('left', 'right', 'top', 'bottom', 'minimum_ratio')})
        self.s, self.v = self.module.load_lane_white(repo / 'lane_fusion_ws/src/autorace_lane_fusion/config/real/lane.yaml')
        self.bags = {}
        self.indices = OrderedDict()
        self.images = OrderedDict()
        self.lock = threading.RLock()
        self.scan()

    @staticmethod
    def connect(path):
        return sqlite3.connect('file:' + str(path) + '?mode=ro', uri=True)

    def scan(self):
        found = {}
        for root in self.roots:
            if not root.is_dir():
                continue
            for metadata in root.glob('*/metadata.yaml'):
                files = sorted(metadata.parent.glob('*.db3'))
                count = 0
                for path in files:
                    with self.connect(path) as db:
                        row = db.execute('SELECT id FROM topics WHERE name=? AND type=?', (TOPIC, 'sensor_msgs/msg/CompressedImage')).fetchone()
                        if row:
                            count += db.execute('SELECT count(*) FROM messages WHERE topic_id=?', row).fetchone()[0]
                if count:
                    ident = hashlib.sha256(str(metadata.parent.resolve()).encode()).hexdigest()[:16]
                    found[ident] = dict(id=ident, name=metadata.parent.name, count=count, files=files)
        with self.lock:
            self.bags = found
        return sorted((dict(id=b['id'], name=b['name'], count=b['count']) for b in found.values()), key=lambda b: b['name'], reverse=True)

    def info(self):
        return dict(bags=self.scan(), defaults=dict(h_min=0, h_max=179, s_min=0, s_max=self.s, v_min=self.v, v_max=255),
                    roi={key: getattr(self.roi, key) for key in ('left', 'right', 'top', 'bottom', 'minimum_ratio')})

    def bag(self, ident):
        with self.lock:
            if ident not in self.bags:
                raise ValueError('bag을 찾을 수 없습니다. 목록을 새로고침하세요.')
            if ident in self.indices:
                self.indices.move_to_end(ident)
                return self.indices[ident]
            entries = []
            for path in self.bags[ident]['files']:
                with self.connect(path) as db:
                    row = db.execute('SELECT id FROM topics WHERE name=?', (TOPIC,)).fetchone()
                    if row:
                        entries.extend((stamp, mid, path) for mid, stamp in db.execute('SELECT id,timestamp FROM messages WHERE topic_id=? ORDER BY timestamp,id', row))
            entries.sort(key=lambda e: (e[0], e[1]))
            if not entries:
                raise ValueError('카메라 프레임이 없는 bag입니다.')
            first = entries[0][0]
            value = dict(entries=entries, times=[(t-first)/1e9 for t, _, _ in entries], name=self.bags[ident]['name'])
            self.indices[ident] = value
            while len(self.indices) > 8:
                self.indices.popitem(last=False)
            return value

    def metadata(self, ident):
        bag = self.bag(ident)
        return dict(id=ident, name=bag['name'], count=len(bag['entries']), times=bag['times'], duration=bag['times'][-1])

    @staticmethod
    def thresholds(params):
        result = {}
        for channel, maximum in [('h', 179), ('s', 255), ('v', 255)]:
            for bound in ['min', 'max']:
                key = channel + '_' + bound
                value = params.get(key)
                if isinstance(value, bool) or not isinstance(value, int) or not 0 <= value <= maximum:
                    raise ValueError(f'{key}: 0~{maximum} 범위의 정수를 사용하세요.')
                result[key] = value
            if result[channel + '_min'] > result[channel + '_max']:
                raise ValueError('HSV 하한은 상한보다 클 수 없습니다.')
        return result

    def prepare(self, ident, index):
        bag = self.bag(ident)
        if isinstance(index, bool) or not isinstance(index, int) or not 0 <= index < len(bag['entries']):
            raise ValueError('프레임 번호가 bag 범위를 벗어났습니다.')
        key = ident, index
        with self.lock:
            if key in self.images:
                self.images.move_to_end(key)
                return self.images[key]
        _, mid, path = bag['entries'][index]
        with self.connect(path) as db:
            row = db.execute('SELECT data FROM messages WHERE id=?', (mid,)).fetchone()
        msg = deserialize_message(row[0], CompressedImage)
        data = bytes(msg.data)
        image = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            raise ValueError('카메라 JPEG를 읽을 수 없습니다.')
        height, width = image.shape[:2]
        _, bounds, _ = self.roi.measure(image, self.s, self.v)
        x0, y0, x1, y1 = bounds
        full_hsv = cv2.cvtColor(cv2.GaussianBlur(image, (5, 5), 0), cv2.COLOR_BGR2HSV)
        # The mission blurs the crop independently. Preserve its ROI boundary pixels.
        crop_hsv = cv2.cvtColor(cv2.GaussianBlur(image[y0:y1, x0:x1], (5, 5), 0), cv2.COLOR_BGR2HSV)
        value = dict(jpeg=base64.b64encode(data).decode(), hsv=full_hsv, roi_hsv=crop_hsv, bounds=bounds, width=width, height=height)
        with self.lock:
            self.images[key] = value
            while len(self.images) > 8:
                self.images.popitem(last=False)
        return value

    def render(self, ident, index, params):
        params = self.thresholds(params)
        image = self.prepare(ident, index)
        lower = tuple(params[c + '_min'] for c in ('h', 's', 'v'))
        upper = tuple(params[c + '_max'] for c in ('h', 's', 'v'))
        mask = cv2.inRange(image['hsv'], lower, upper)
        roi_mask = cv2.inRange(image['roi_hsv'], lower, upper)
        x0, y0, x1, y1 = image['bounds']
        mask[y0:y1, x0:x1] = roi_mask
        white = int(np.count_nonzero(roi_mask))
        ratio = white / roi_mask.size
        ok, png = cv2.imencode('.png', mask)
        if not ok:
            raise ValueError('마스크 생성에 실패했습니다.')
        return dict(index=index, time=self.bag(ident)['times'][index], image='data:image/jpeg;base64,' + image['jpeg'],
                    mask='data:image/png;base64,' + base64.b64encode(png).decode(),
                    width=image['width'], height=image['height'], bounds=list(image['bounds']),
                    white_pixels=white, roi_pixels=int(roi_mask.size), ratio=ratio,
                    detected=ratio >= self.roi.minimum_ratio, threshold=self.roi.minimum_ratio, params=params)


class Handler(BaseHTTPRequestHandler):
    server_version = 'AutoRaceHSV/1.0'

    def json(self, value, status=200):
        body = json.dumps(value, ensure_ascii=False, allow_nan=False).encode()
        self.send_response(status)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        route = urlparse(self.path)
        try:
            if route.path == '/api/info':
                return self.json(self.server.tuner.info())
            if route.path == '/api/bag':
                return self.json(self.server.tuner.metadata(parse_qs(route.query).get('id', [''])[0]))
            files = {'/': ('index.html', 'text/html; charset=utf-8'), '/app.js': ('app.js', 'text/javascript; charset=utf-8'), '/style.css': ('style.css', 'text/css; charset=utf-8')}
            if route.path not in files:
                return self.json(dict(error='페이지가 없습니다.'), 404)
            name, content_type = files[route.path]
            data = (HERE / 'static' / name).read_bytes()
            self.send_response(200)
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', str(len(data)))
            self.send_header('Cache-Control', 'no-cache')
            self.end_headers()
            self.wfile.write(data)
        except (ValueError, KeyError) as error:
            self.json(dict(error=str(error)), 400)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def do_POST(self):
        if self.path != '/api/frame':
            return self.json(dict(error='지원하지 않는 요청입니다.'), 404)
        try:
            length = int(self.headers.get('Content-Length', '0'))
            if not 0 < length <= 8192:
                raise ValueError('요청 크기가 올바르지 않습니다.')
            data = json.loads(self.rfile.read(length))
            if not isinstance(data, dict) or not isinstance(data.get('params'), dict):
                raise ValueError('bag, 프레임, HSV 값을 지정하세요.')
            result = self.server.tuner.render(data.get('bag', ''), data.get('index'), data['params'])
            self.json(result)
        except (ValueError, KeyError, TypeError) as error:
            self.json(dict(error=str(error)), 400)
        except (BrokenPipeError, ConnectionResetError):
            pass
        except Exception:
            import traceback
            traceback.print_exc()
            self.json(dict(error='프레임을 읽지 못했습니다. 다른 프레임이나 bag을 선택하세요.'), 500)

    def log_message(self, message, *args):
        if '/api/frame' not in str(args[0] if args else ''):
            super().log_message(message, *args)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='0.0.0.0')
    parser.add_argument('--port', type=int, default=8091)
    parser.add_argument('--bag-root', action='append', type=Path)
    args = parser.parse_args()
    roots = args.bag_root or [REPO / 'bags']
    cv2.setNumThreads(1)
    server = ThreadingHTTPServer((args.host, args.port), Handler)
    server.daemon_threads = True
    server.tuner = Tuner(roots)
    print(f'HSV tuner ready: http://{args.host}:{args.port} ({len(server.tuner.bags)} camera bags)', flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == '__main__':
    main()
