"""Read-only, on-demand access to the newest local camera bag."""
from pathlib import Path
import sqlite3
import threading

import cv2
import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CompressedImage, Image


class BagImages:
    def __init__(self, root):
        self.root = Path(root)
        self.lock = threading.Lock()
        self.path = None
        self.frames = []
        self.start = 0
        self.topic = None

    @staticmethod
    def connect(path):
        return sqlite3.connect('file:' + str(path) + '?mode=ro', uri=True)

    def latest(self):
        candidates = sorted(self.root.glob('*/metadata.yaml'), key=lambda p: p.stat().st_mtime, reverse=True)
        for metadata in candidates:
            files = sorted(metadata.parent.glob('*.db3'))
            if not files:
                continue
            entries, beginnings, topic = [], [], None
            for dbfile in files:
                with self.connect(dbfile) as db:
                    found = db.execute("SELECT id,name,type FROM topics WHERE name IN ('/camera/color/image_raw','/camera/color/image_raw/compressed') ORDER BY CASE name WHEN '/camera/color/image_raw' THEN 0 ELSE 1 END").fetchall()
                    if not found:
                        continue
                    if topic is None:
                        topic = found[0][1]
                    chosen = next((row for row in found if row[1] == topic), None)
                    if chosen is None:
                        continue
                    beginnings.append(db.execute('SELECT min(timestamp) FROM messages').fetchone()[0])
                    entries.extend((t, ident, dbfile, chosen[2]) for ident, t in db.execute(
                        'SELECT id,timestamp FROM messages WHERE topic_id=? ORDER BY timestamp,id', (chosen[0],)))
            if not entries:
                continue
            entries.sort(key=lambda row: (row[0], row[1]))
            with self.lock:
                self.path, self.frames, self.start, self.topic = metadata.parent, entries, min(beginnings), topic
                return dict(name=self.path.name, count=len(entries), times=[round((row[0]-self.start)/1e9, 6) for row in entries], topic=topic)
        raise ValueError('카메라 영상이 있는 SQLite bag을 찾지 못했습니다.')

    def frame(self, index, name):
        if isinstance(index, bool) or not isinstance(index, int):
            raise ValueError('프레임 번호는 정수여야 합니다.')
        with self.lock:
            if self.path is None or self.path.name != name:
                raise ValueError('최신 bag을 다시 불러오세요.')
            if not 0 <= index < len(self.frames):
                raise ValueError('프레임 번호가 bag 범위를 벗어났습니다.')
            stamp, ident, dbfile, kind = self.frames[index]
            meta = dict(source='bag', source_label=self.path.name, bag_name=self.path.name,
                        frame_index=index, frame_count=len(self.frames),
                        bag_time_sec=(stamp-self.start)/1e9, age_ms=None)
        with self.connect(dbfile) as db:
            row = db.execute('SELECT data FROM messages WHERE id=?', (ident,)).fetchone()
        if kind == 'sensor_msgs/msg/CompressedImage':
            return bytes(deserialize_message(row[0], CompressedImage).data), meta
        if kind != 'sensor_msgs/msg/Image':
            raise ValueError('지원하지 않는 카메라 메시지입니다.')
        msg = deserialize_message(row[0], Image)
        channels = {'bgr8': 3, 'rgb8': 3, 'mono8': 1}.get(msg.encoding)
        if channels is None:
            raise ValueError('지원하지 않는 카메라 형식: ' + msg.encoding)
        image = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, :msg.width*channels]
        image = image.reshape((msg.height, msg.width, channels) if channels > 1 else (msg.height, msg.width))
        if msg.encoding == 'rgb8':
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        # Lossless input preserves the recorded pixels for HSV measurement.
        ok, encoded = cv2.imencode('.png', image)
        if not ok:
            raise ValueError('bag 이미지 변환에 실패했습니다.')
        return encoded.tobytes(), meta
