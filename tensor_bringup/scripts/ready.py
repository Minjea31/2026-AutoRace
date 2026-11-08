#!/usr/bin/env python3
"""Check mission inputs, then report the final bringup node state."""
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.utilities import get_message

MISSION_NODES = (
    '/parking_node', '/slow_node', '/lane_fusion_deep_mask',
    '/lane_fusion_node', '/lane_fusion_pid_controller', '/yolo_detect',
)
INPUT_TOPICS = (
    '/camera/color/image_raw/compressed', '/scan', '/odometry/filtered',
    '/lane_fusion/deep_mask', '/mission/lane', '/yolo/parking_sign',
    '/yolo/slow_sign', '/yolo/bump_sign',
)


def missing_inputs(node, expected_nodes, seen, now):
    names = {f"{namespace.rstrip('/')}/{name}"
             for name, namespace in node.get_node_names_and_namespaces()}
    missing_nodes = sorted(set(expected_nodes) - names)
    missing_topics = [topic for topic in INPUT_TOPICS
                      if now - seen.get(topic, -math.inf) >= 1.0]
    return missing_nodes, missing_topics


def main():
    rclpy.init()
    node = Node('tensor_readiness')
    try:
        timeout = float(node.declare_parameter('timeout', 60.0).value)
        phase = node.declare_parameter('phase', 'inputs').value
        if not math.isfinite(timeout) or timeout <= 0 or phase not in ('inputs', 'nodes'):
            raise ValueError('timeout must be positive and phase must be inputs or nodes')
        expected = MISSION_NODES + (('/mux',) if phase == 'nodes' else ())
        seen, subscriptions = {}, {}
        started = time.monotonic()
        stable_since = None
        missing_nodes, missing_topics = list(expected), list(INPUT_TOPICS)
        while rclpy.ok() and time.monotonic() - started < timeout:
            types = dict(node.get_topic_names_and_types())
            for topic in INPUT_TOPICS:
                if topic in types and topic not in subscriptions:
                    subscriptions[topic] = node.create_subscription(
                        get_message(types[topic][0]), topic,
                        lambda message, topic=topic: seen.update({topic: time.monotonic()}),
                        qos_profile_sensor_data,
                    )
            rclpy.spin_once(node, timeout_sec=0.1)
            now = time.monotonic()
            missing_nodes, missing_topics = missing_inputs(node, expected, seen, now)
            if missing_nodes or missing_topics:
                stable_since = None
                continue
            if stable_since is None:
                stable_since = now
            if phase == 'inputs' or now - stable_since >= 2.0:
                if phase == 'inputs':
                    node.get_logger().info('Inputs ready; starting mux')
                else:
                    node.get_logger().info('실행 확인: ' + ', '.join(expected))
                return 0
        node.get_logger().error(
            '[bringup 확인 실패] 미실행 노드: ' + (', '.join(missing_nodes) or '없음')
            + ' | 미수신·지연 토픽: ' + (', '.join(missing_topics) or '없음')
            + (' | 전체 상태가 2초간 안정되지 않음' if not missing_nodes and not missing_topics else '')
        )
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
