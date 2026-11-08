"""Real ROS callbacks and JPEG inputs on an isolated DDS domain; no vehicle output."""
import time
from pathlib import Path

import cv2
import numpy as np
import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from autorace_parking.node import ParkingNode


class Harness:
    def __init__(self):
        self.context = Context()
        rclpy.init(context=self.context, domain_id=216, signal_handler_options=SignalHandlerOptions.NO)
        root = Path(__file__).resolve().parents[4]
        self.node = ParkingNode(context=self.context, parameter_overrides=[
            Parameter('localization_source', value='scan'),
            Parameter('white_roi_enabled', value=True),
            Parameter('lane_white_config', value=str(root / 'lane_fusion_ws/src/autorace_lane_fusion/config/real/lane.yaml')),
            Parameter('lane_white_node', value='/parking_gate_test_lane')])
        self.driver = Node('parking_gate_test_driver', context=self.context)
        self.lane = Node('parking_gate_test_lane', context=self.context)
        self.lane.declare_parameter('white_s_max', 70)
        self.lane.declare_parameter('white_v_min', 120)
        self.signs = self.driver.create_publisher(Bool, '/yolo/parking_sign', 1)
        self.camera = self.driver.create_publisher(CompressedImage, '/camera/color/image_raw/compressed', 1)
        self.executor = SingleThreadedExecutor(context=self.context)
        for node in (self.node, self.driver, self.lane):
            self.executor.add_node(node)
        end = time.monotonic() + 3.
        while self.signs.get_subscription_count() < 1 or self.camera.get_subscription_count() < 1:
            assert time.monotonic() < end
            self.pump(.02)

    def pump(self, duration):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            self.executor.spin_once(timeout_sec=.005)

    def sign(self, value=True):
        self.signs.publish(Bool(data=value))
        self.pump(.04)

    def image(self, colour=255, age=0.):
        frame = np.full((240, 320, 3), colour, dtype=np.uint8)
        ok, encoded = cv2.imencode('.jpg', frame)
        assert ok
        msg = CompressedImage(format='jpeg', data=encoded.tobytes())
        stamp = self.driver.get_clock().now().nanoseconds - int(age * 1e9)
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
        self.camera.publish(msg)
        self.pump(.12)
        return msg

    def close(self):
        self.executor.shutdown()
        for node in (self.node, self.driver, self.lane):
            node.destroy_node()
        self.context.shutdown()


@pytest.fixture
def h():
    harness = Harness()
    try:
        yield harness
    finally:
        harness.close()


def test_two_signs_then_new_white_starts_and_old_white_is_not_cached(h):
    old = h.image()
    h.sign()
    h.image()
    assert not h.node.activation.active and not h.node.floor_gate.armed
    h.sign()
    assert h.node.floor_gate.armed and not h.node.activation.active
    assert h.node.state == 'WAITING_WHITE_LINE'
    h.camera.publish(old)
    h.pump(.12)
    assert not h.node.activation.active
    h.image(colour=0)
    assert not h.node.activation.active
    h.image()
    assert h.node.activation.active and h.node.attempt == 1
    assert not h.node.floor_gate.armed


def test_preconfirmation_stale_future_and_invalid_images_cannot_start(h):
    h.sign()
    h.sign()
    h.image(age=.15)  # still fresh, but acquired before the second sign
    assert not h.node.activation.active
    h.image(age=1.)
    h.image(age=-1.)
    assert not h.node.activation.active
    msg = CompressedImage(format='jpeg', data=b'broken JPEG')
    msg.header.stamp = h.driver.get_clock().now().to_msg()
    h.camera.publish(msg)
    h.pump(.12)
    assert not h.node.activation.active
    h.image()
    assert h.node.activation.active


def test_sign_disappearing_after_confirmation_keeps_floor_gate_armed(h):
    h.sign()
    h.sign()
    h.sign(False)
    h.pump(.6)
    assert h.node.floor_gate.armed and not h.node.activation.active
    h.image()
    assert h.node.activation.active


def test_false_and_long_gap_reset_sign_count_and_reset_clears_armed_gate(h):
    h.sign()
    h.sign(False)
    h.sign()
    h.image()
    assert not h.node.activation.active and not h.node.floor_gate.armed
    h.pump(.6)
    h.sign()
    assert not h.node.floor_gate.armed
    h.sign()
    assert h.node.floor_gate.armed
    h.node.on_reset(None, Trigger.Response())
    h.image()
    assert not h.node.activation.active and not h.node.floor_gate.armed


def test_restart_also_requires_two_new_signs_and_new_white_after_cooldown(h):
    h.sign()
    h.sign()
    h.image()
    assert h.node.attempt == 1
    h.sign()
    h.sign()
    h.image()
    assert h.node.attempt == 1 and not h.node.floor_gate.armed
    h.node.sign_ignore_until = time.monotonic() - 1.
    h.sign()
    h.image()
    assert h.node.attempt == 1 and not h.node.floor_gate.armed
    h.sign()
    assert h.node.floor_gate.armed and h.node.attempt == 1
    h.image()
    assert h.node.attempt == 2


def test_roi_uses_live_lane_hsv_thresholds(h):
    gate = h.node.floor_gate
    h.lane.set_parameters([Parameter('white_v_min', value=240)])
    end = time.monotonic() + 2.
    while gate.v_min != 240:
        gate.poll_thresholds()
        h.pump(.02)
        assert time.monotonic() < end
    h.sign()
    h.sign()
    h.image(colour=200)
    assert not h.node.activation.active and gate.ratio == 0.
    h.image(colour=255)
    assert h.node.activation.active
