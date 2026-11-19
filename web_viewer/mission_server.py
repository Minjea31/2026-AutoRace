#!/usr/bin/env python3
"""Read-only AutoRace mission web server."""
import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import signal
import threading

from mission_ros import MonitorState, start_ros

DIST = Path(__file__).resolve().parent / 'dist'
HTML = ('mission.html', 'text/html; charset=utf-8')
ASSETS = {
    '/': HTML,
    '/missions': HTML,
    '/mission.css': ('mission.css', 'text/css; charset=utf-8'),
    '/mission.js': ('mission.js', 'text/javascript; charset=utf-8'),
}


class Monitor(ThreadingHTTPServer):
    daemon_threads = True
    request_queue_size = 8

    def __init__(self, address):
        self.request_slots = threading.BoundedSemaphore(8)
        self.state = MonitorState()
        self.map_snapshot = self.state.map_snapshot
        super().__init__(address, Handler)

    def process_request(self, request, address):
        if not self.request_slots.acquire(blocking=False):
            self.shutdown_request(request)
            return
        try:
            super().process_request(request, address)
        except Exception:
            self.request_slots.release()
            raise

    def process_request_thread(self, request, address):
        try:
            super().process_request_thread(request, address)
        finally:
            self.request_slots.release()


class Handler(BaseHTTPRequestHandler):
    def setup(self):
        super().setup()
        self.connection.settimeout(5)

    def log_message(self, *args):
        pass

    def send(self, payload, content_type, status=200):
        self.send_response(status)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(payload)))
        self.send_header('Cache-Control', 'no-store')
        self.send_header('X-Content-Type-Options', 'nosniff')
        self.end_headers()
        self.wfile.write(payload)

    def send_json(self, payload):
        body = json.dumps(payload, ensure_ascii=False, separators=(',', ':')).encode()
        self.send(body, 'application/json; charset=utf-8')

    def do_GET(self):
        path = self.path.partition('?')[0]
        try:
            if path in ASSETS:
                filename, content_type = ASSETS[path]
                self.send((DIST / filename).read_bytes(), content_type)
            elif path == '/health':
                self.send(b'AutoRace mission web OK\n', 'text/plain; charset=utf-8')
            elif path == '/api/state':
                self.send_json(self.server.state.status_snapshot())
            elif path == '/api/map':
                self.send_json(self.server.map_snapshot())
            elif path == '/favicon.ico':
                self.send(b'', 'image/x-icon', 204)
            else:
                self.send_error(404)
        except (BrokenPipeError, ConnectionResetError, TimeoutError):
            self.close_connection = True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='0.0.0.0')
    parser.add_argument('--port', type=int, default=8080)
    parser.add_argument('--demo', action='store_true')
    args = parser.parse_args()
    stopping = threading.Event()
    for signum in (signal.SIGINT, signal.SIGTERM):
        signal.signal(signum, lambda *_: stopping.set())

    with Monitor((args.host, args.port)) as http:
        ros = None
        if args.demo:
            from mission_demo import load_demo
            preview_map = load_demo(http.state)
            http.map_snapshot = lambda: preview_map
        else:
            ros, bridge = start_ros(http.state)
            thread = threading.Thread(target=ros.spin, args=(bridge,), daemon=True)
            thread.start()

        http.timeout = 0.3
        mode = 'demo' if args.demo else 'live'
        print(f'AutoRace Mission Web http://{args.host}:{http.server_port} mode={mode}', flush=True)
        try:
            while not stopping.is_set():
                http.handle_request()
        finally:
            if ros:
                ros.try_shutdown()
                thread.join()
                bridge.destroy_node()


if __name__ == '__main__':
    main()
