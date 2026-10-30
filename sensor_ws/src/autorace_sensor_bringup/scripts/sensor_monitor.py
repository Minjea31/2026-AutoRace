#!/usr/bin/env python3
"""Record first samples, missing streams, stale streams and recovery without commands."""
import json
import time
from datetime import datetime, timezone
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.utilities import get_message


class SensorMonitor(Node):
    def __init__(self):
        super().__init__('sensor_monitor')
        topics = self.declare_parameter('topics', ['/imu']).value
        types = self.declare_parameter('types', ['sensor_msgs/msg/Imu']).value
        if len(topics) != len(types) or len(set(topics)) != len(topics):
            raise ValueError('topics and types must have equal lengths and unique topics')
        self.startup_timeout = self.declare_parameter('startup_timeout', 15.0).value
        self.stale_timeout = self.declare_parameter('stale_timeout', 5.0).value
        if self.startup_timeout <= 0 or self.stale_timeout <= 0:
            raise ValueError('timeouts must be positive')
        root = Path(self.declare_parameter('log_dir', str(Path.home() / '.ros/log')).value)
        root.mkdir(parents=True, exist_ok=True)
        self.event_file = root / 'sensor_events.jsonl'
        self.started = time.monotonic()
        self.streams = {topic: {'last': None, 'state': 'WAITING'} for topic in topics}
        self.subscriptions_ = []
        for topic, msg_type in zip(topics, types):
            self.emit(topic, 'WAITING')
            self.subscriptions_.append(self.create_subscription(
                get_message(msg_type), topic,
                lambda msg, topic=topic: self.received(topic), qos_profile_sensor_data,
            ))
        self.timer = self.create_timer(1.0, self.check)

    def emit(self, topic, event):
        data = {'time': datetime.now(timezone.utc).isoformat(), 'topic': topic, 'event': event}
        with self.event_file.open('a') as stream:
            stream.write(json.dumps(data) + '\n')
        message = f'{event}: {topic}'
        if event in ('NO_DATA', 'STALE'):
            self.get_logger().warning(message)
        else:
            self.get_logger().info(message)

    def received(self, topic):
        stream = self.streams[topic]
        if stream['last'] is None:
            self.emit(topic, 'READY')
        elif stream['state'] == 'STALE':
            self.emit(topic, 'RECOVERED')
        stream.update(last=time.monotonic(), state='READY')

    def check(self):
        now = time.monotonic()
        for topic, stream in self.streams.items():
            if stream['last'] is None:
                if now - self.started >= self.startup_timeout and stream['state'] != 'NO_DATA':
                    self.emit(topic, 'NO_DATA')
                    stream['state'] = 'NO_DATA'
            elif now - stream['last'] >= self.stale_timeout and stream['state'] != 'STALE':
                self.emit(topic, 'STALE')
                stream['state'] = 'STALE'


def main():
    rclpy.init()
    node = SensorMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
