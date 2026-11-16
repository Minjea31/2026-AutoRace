"""Real DDS pub/sub checks on private topics, with no vehicle output."""

import math
import os
import time
from uuid import uuid4

import pytest

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('ackermann_msgs.msg')
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions

from autorace_mux.core import INPUTS
from autorace_mux.node import MuxNode


class Harness:
    def __init__(self, simulated=False, parking_guard=False):
        self.context = Context()
        rclpy.init(context=self.context, domain_id=180 + os.getpid() % 20,
                   signal_handler_options=SignalHandlerOptions.NO)
        prefix = '/mux_test_' + uuid4().hex
        self.topics = {name: prefix + '/' + name for name, _ in INPUTS}
        parameters = {
            'output_topic': prefix + '/output', 'publish_rate_hz': 60.0,
            'parking_guard_enabled': parking_guard,
            'active_source_topic': prefix + '/active_source',
            'input_timeout_sec': 0.18,
            'max_acceleration_mps2': 100.0, 'max_deceleration_mps2': 100.0,
            **{'topics.' + key: value for key, value in self.topics.items()},
        }
        if simulated:
            parameters.update(use_sim_time=True, require_stamp=False)
        self.mux = MuxNode(
            context=self.context,
            parameter_overrides=[Parameter(key, value=value) for key, value in parameters.items()])
        self.probe = Node('mux_probe_' + uuid4().hex, context=self.context)
        qos = QoSProfile(depth=1)
        self.publishers = {key: self.probe.create_publisher(AckermannDriveStamped, value, qos)
                           for key, value in self.topics.items()}
        self.samples = []
        self.subscription = self.probe.create_subscription(
            Twist, prefix + '/output',
            lambda msg: self.samples.append((msg.linear.x, msg.angular.z)), qos)
        self.sources = []
        self.source_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.source_subscription = self.probe.create_subscription(
            String, prefix + '/active_source',
            lambda msg: self.sources.append(msg.data), self.source_qos)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.mux)
        self.executor.add_node(self.probe)
        try:
            self.until(lambda: all(p.get_subscription_count() for p in self.publishers.values())
                       and self.probe.count_publishers(self.subscription.topic_name) == 1)
        except BaseException:
            self.close()
            raise

    def until(self, predicate, commands=None, timeout=3.0):
        deadline = time.monotonic() + timeout
        next_publish = 0.0
        while time.monotonic() < deadline:
            now = time.monotonic()
            if commands and now >= next_publish:
                for key, (speed, angle) in commands.items():
                    msg = AckermannDriveStamped()
                    msg.header.stamp = self.mux.get_clock().now().to_msg()
                    msg.drive.speed = float(speed)
                    msg.drive.steering_angle = float(angle)
                    self.publishers[key].publish(msg)
                next_publish = now + 0.02
            self.executor.spin_once(timeout_sec=0.01)
            if predicate():
                return
        pytest.fail(f'Timed out waiting for mux output; recent samples={self.samples[-5:]}')

    def output(self, speed, yaw, commands=None):
        self.samples.clear()
        self.until(lambda: bool(self.samples) and
                   self.samples[-1] == pytest.approx((speed, yaw), abs=1e-5), commands)

    def close(self):
        self.mux.stop()
        self.executor.shutdown()
        self.probe.destroy_node()
        self.mux.destroy_node()
        self.context.shutdown()


def test_real_ros_lane_mission_preemption_reverse_and_timeout():
    h = Harness()
    try:
        lane = {'lane': (0.4, 0.1)}
        h.output(0.0, 0.0)  # Startup watchdog.
        h.output(0.4, 0.4 * math.tan(0.1) / 0.2, lane)
        h.output(0.2, 0.0, {**lane, 'slow': (0.2, 0.0)})
        h.output(0.0, 0.0, {**lane, 'slow': (0.2, 0.0), 'crosswalk': (0.0, 0.2)})
        h.output(0.4, 0.4 * math.tan(0.1) / 0.2, lane)  # Expired missions -> lane.
        h.output(0.4, 0.4 * math.tan(0.1) / 0.2, {**lane, 'signal': (-0.2, 0.0)})
        h.output(-0.2, -0.2 * math.tan(0.1) / 0.2, {**lane, 'parking': (-0.2, 0.1)})
        h.output(0.0, 0.0)  # All inputs expire, including lane.
    finally:
        h.close()


def test_real_ros_every_priority_preempts_and_releases_in_order():
    h = Harness()
    try:
        speeds = {
            'crosswalk': 0.11, 'signal': 0.12, 'parking': 0.13,
            'cone': 0.14, 'objavoid': 0.15, 'slow': 0.18, 'lane': 0.4,
        }
        commands = {'lane': (speeds['lane'], 0.1)}
        h.output(speeds['lane'], speeds['lane'] * math.tan(0.1) / 0.2, commands)
        # Add inputs from lowest to highest priority; each must take control.
        for source, _ in reversed(INPUTS[:-1]):
            commands[source] = (speeds[source], 0.1)
            h.output(speeds[source], speeds[source] * math.tan(0.1) / 0.2, commands)
        # Stop each winner while continuing all lower-priority inputs.
        for index, (source, _) in enumerate(INPUTS[:-1]):
            del commands[source]
            next_source = INPUTS[index + 1][0]
            speed = speeds[next_source]
            h.output(speed, speed * math.tan(0.1) / 0.2, commands)
        h.output(0.0, 0.0)  # Lane also expires.
    finally:
        h.close()


def test_real_ros_stale_out_of_order_message_does_not_cancel_stop():
    h = Harness()
    try:
        lane = {'lane': (0.4, 0.1)}
        h.output(0.4, 0.4 * math.tan(0.1) / 0.2, lane)
        h.output(0.0, 0.0, {**lane, 'crosswalk': (0.0, 0.1)})

        old_message = AckermannDriveStamped()
        old_message.header.stamp = h.mux.get_clock().now().to_msg()
        old_message.header.stamp.sec -= 1
        old_message.drive.speed = 0.4
        h.publishers['crosswalk'].publish(old_message)
        h.until(lambda: 'crosswalk' in h.mux._last_warning, lane)

        h.samples.clear()
        h.until(lambda: len(h.samples) >= 3, lane)
        assert all(sample == (0.0, 0.0) for sample in h.samples)
        # The ignored packet must not prevent normal timeout and lane fallback.
        h.output(0.4, 0.4 * math.tan(0.1) / 0.2, lane)
    finally:
        h.close()


def test_watchdog_keeps_running_without_simulated_clock_messages():
    h = Harness(simulated=True)
    try:
        assert h.mux.get_clock().now().nanoseconds == 0
        h.output(0.3, 0.0, {'lane': (0.3, 0.0)})
        h.output(0.0, 0.0)
    finally:
        h.close()


def test_active_source_tracks_stopping_missions_timeout_late_join_and_shutdown(capfd):
    h = Harness()
    try:
        h.until(lambda: bool(h.sources) and h.sources[-1] == 'stop')
        lane = {'lane': (0.4, 0.0)}
        h.output(0.4, 0.0, lane)
        h.until(lambda: h.sources[-1] == 'lane', lane)
        capfd.readouterr()
        stable_until = time.monotonic() + 1.15
        h.until(lambda: time.monotonic() >= stable_until, lane)
        captured = capfd.readouterr()
        assert 'Selected:' not in captured.out + captured.err
        stop_mission = {**lane, 'crosswalk': (0.0, 0.0)}
        h.output(0.0, 0.0, stop_mission)
        h.until(lambda: h.sources[-1] == 'crosswalk', stop_mission)
        h.output(0.4, 0.0, lane)
        h.until(lambda: h.sources[-1] == 'lane', lane)
        h.output(0.0, 0.0)
        h.until(lambda: h.sources[-1] == 'stop')
        parking = {'parking': (-0.2, 0.0)}
        h.output(-0.2, 0.0, parking)
        h.until(lambda: h.sources[-1] == 'parking', parking)

        # A late monitor must receive the retained selection without another tick.
        h.mux._timer.cancel()
        retained = []
        late_subscription = h.probe.create_subscription(
            String, h.source_subscription.topic_name,
            lambda msg: retained.append(msg.data), h.source_qos)
        h.until(lambda: retained == ['parking'])
        h.mux.stop()
        h.until(lambda: h.sources[-1] == 'stop' and retained[-1] == 'stop')
        h.probe.destroy_subscription(late_subscription)
        captured = capfd.readouterr()
        assert 'Selected: lane | input=' in captured.out + captured.err
        assert 'Selected: crosswalk | input=' in captured.out + captured.err
        assert (captured.out + captured.err).count('Selected: STOP (shutdown)') == 1
        h.mux.stop()
        captured = capfd.readouterr()
        assert 'Selected:' not in captured.out + captured.err
    finally:
        h.close()



def test_parking_handoff_allows_only_lane_and_repeats_for_new_parking_trigger():
    from std_msgs.msg import Header
    h=Harness()
    try:
        lane={'lane':(.4,0.)}
        others={source:(0.,0.) for source,_ in INPUTS if source not in ('lane','parking')}
        h.output(.4,0.,lane)
        first=h.mux.get_clock().now().to_msg()
        h.mux._release_parking(Header(stamp=first))
        h.output(.4,0.,{**lane,**others})
        assert h.mux._last_source=='lane' and h.mux._parking_lane_mode=='lane'
        # No lane command means STOP, never another available mission.
        h.output(0.,0.,others)
        h.output(.4,0.,{**lane,**others})
        deadline=h.mux._parking_lane_hold_until
        assert 8.0 < deadline-time.monotonic() <= 10.0
        h.output(.4,0.,{**lane,**others,'parking':(-.15,0.)})
        # Repeated release messages must not restart the ten-second window.
        h.mux._release_parking(Header(stamp=h.mux.get_clock().now().to_msg()))
        assert h.mux._parking_lane_hold_until==deadline
        h.mux._parking_lane_hold_until=time.monotonic()-.001
        delayed=Header(stamp=h.mux.get_clock().now().to_msg())
        h.output(-.15,0.,{**lane,**others,'parking':(-.15,0.)})
        assert h.mux._last_source=='parking' and h.mux._parking_lane_mode=='parking'
        # This packet is still fresh, but predates the NEW parking run.
        h.mux._release_parking(delayed)
        h.mux._release_parking(Header(stamp=first))
        h.output(-.15,0.,{**lane,**others,'parking':(-.15,0.)})
        # Missing parking commands do not fall through to lane while parking runs.
        h.output(0.,0.,{**lane,**others})
        h.output(-.15,0.,{**lane,**others,'parking':(-.15,0.)})
        h.mux._release_parking(Header(stamp=h.mux.get_clock().now().to_msg()))
        h.output(.4,0.,{**lane,**others})
        assert h.mux._last_source=='lane' and h.mux._parking_lane_mode=='lane'
    finally:h.close()


def test_handoff_header_clears_guard_and_old_drive_cannot_rearm_it():
    from std_msgs.msg import Header
    import json
    h=Harness(parking_guard=True)
    try:
        lane={'lane':(.4,0.)}
        now=h.mux.get_clock().now().nanoseconds*1e-9
        drive=String(data=json.dumps(dict(stamp=now,phase='DRIVE',ready=True)))
        h.mux._parking_state(drive)
        h.output(-.15,0.,{**lane,'parking':(-.15,0.),'crosswalk':(0.,0.)})
        h.mux._release_parking(Header(stamp=h.mux.get_clock().now().to_msg()))
        h.mux._parking_state(drive)
        assert not h.mux.parking_guard.armed
        h.output(.4,0.,{**lane,'signal':(0.,0.),'objavoid':(.2,0.)})
        # Fresh HOLD/DRIVE/start messages during the mandatory lane window are ignored.
        now=h.mux.get_clock().now().nanoseconds*1e-9
        h.mux._parking_state(String(data=json.dumps(dict(stamp=now,phase='HOLD',ready=False))))
        h.mux._parking_start(Header(stamp=h.mux.get_clock().now().to_msg()))
        assert not h.mux.parking_guard.armed
        h.output(.4,0.,{**lane,'parking':(-.15,0.)})
        h.mux._parking_lane_hold_until=time.monotonic()-.001
        # A fresh start after expiry re-arms parking before its first command.
        now=h.mux.get_clock().now().nanoseconds*1e-9
        h.mux._parking_state(String(data=json.dumps(dict(stamp=now,phase='HOLD',ready=False))))
        h.output(0.,0.,lane)
        now=h.mux.get_clock().now().nanoseconds*1e-9
        h.mux._parking_state(String(data=json.dumps(dict(stamp=now,phase='DRIVE',ready=True))))
        h.output(-.15,0.,{**lane,'parking':(-.15,0.),'crosswalk':(0.,0.)})
        now=h.mux.get_clock().now().nanoseconds*1e-9
        h.mux._parking_state(String(data=json.dumps(dict(stamp=now,phase='RELEASE',ready=True))))
        h.output(.4,0.,{**lane,'crosswalk':(0.,0.)})
        assert not h.mux.parking_guard.armed and h.mux._parking_lane_mode=='lane'
    finally:h.close()


def test_lane_hold_expires_at_ten_seconds_without_queuing_a_trigger(monkeypatch):
    from std_msgs.msg import Header
    import autorace_mux.node as module
    h=Harness(parking_guard=True)
    try:
        now=h.mux.get_clock().now().to_msg()
        h.mux._release_parking(Header(stamp=now))
        deadline=h.mux._parking_lane_hold_until
        with monkeypatch.context() as patch:
            patch.setattr(module.time,'monotonic',lambda:deadline-.001)
            assert h.mux._lane_hold_active()
            h.mux._parking_start(Header(stamp=h.mux.get_clock().now().to_msg()))
            assert not h.mux.parking_guard.armed
            patch.setattr(module.time,'monotonic',lambda:deadline)
            assert not h.mux._lane_hold_active()
            assert h.mux._parking_lane_mode=='lane' and not h.mux.parking_guard.armed
    finally:h.close()
