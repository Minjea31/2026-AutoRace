"""Mission routing, expiration and real ROS message conversion."""
import json
import math
from pathlib import Path
import time

import pytest

from limo_parking_sim.config import load_config
from limo_parking_sim.mission_adapter import CommandRelay, MISSION_TOPICS, MissionRouter, create_adapter


@pytest.fixture
def relay():
    result = CommandRelay(load_config())
    result.select('/mission/cone/command_04')
    return result


def test_presets_match_repository_contract():
    architecture = Path(__file__).resolve().parents[4] / 'docs/architecture-topics.json'
    edges = json.loads(architecture.read_text())
    inputs = {topic['name'] for edge in edges for topic in edge['topics']
              if topic['type'] == 'ackermann_msgs/msg/AckermannDriveStamped'
              and topic['name'].startswith('/mission/')}
    assert {topic for _, topic in MISSION_TOPICS} == inputs


def test_only_selected_input_can_move_vehicle(relay):
    assert not relay.receive('/mission/parking/command_03', .2, .1, 10., 10., 50.)
    assert relay.output(10., 50.) == (0., 0.)
    assert relay.receive(relay.topic, .2, .1, 10., 10., 50.)
    assert relay.output(10., 50.) == pytest.approx((.2, .2*math.tan(.1)/.2))
    relay.select('/mission/objavoid/command_05')
    assert relay.output(10., 50.) == (0., 0.)
    assert not relay.receive('/mission/cone/command_04', .2, .1, 10., 10., 50.)
    assert relay.receive(relay.topic, -.2, .1, 10., 10., 50.)
    assert relay.output(10., 50.)[1] < 0
    relay.select(None)
    assert relay.output(10., 50.) == (0., 0.)


@pytest.mark.parametrize('speed,steering,stamp', [
    (float('nan'), .1, 10.), (.2, float('inf'), 10.),
    (.2, .1, float('nan')), (.2, .1, 9.7), (.2, .1, 10.1),
])
def test_invalid_input_clears_preceding_command(relay, speed, steering, stamp):
    assert relay.receive(relay.topic, .2, .1, 10., 10., 50.)
    assert not relay.receive(relay.topic, speed, steering, stamp, 10., 50.01)
    assert relay.output(10., 50.01) == (0., 0.)


def test_timeout_with_frozen_clock_and_clock_reset(relay):
    relay.receive(relay.topic, .2, .1, 10., 10., 50.)
    assert relay.output(10.1, 50.1)[0] == .2
    assert relay.output(10., 50.3) == (0., 0.)  # paused simulation clock
    assert relay.output(10.3, 50.1) == (0., 0.)  # simulation clock advanced
    assert relay.output(0., 50.1) == (0., 0.)  # clock moved backwards


def test_vehicle_limits_and_zero_speed(relay):
    relay.receive(relay.topic, 100., 100., 10., 10., 50.)
    wheelbase, max_speed, max_steering = relay.limits
    assert relay.output(10., 50.) == pytest.approx(
        (max_speed, max_speed*math.tan(max_steering)/wheelbase))
    relay.receive(relay.topic, 0., .4, 10., 10., 50.)
    assert relay.output(10., 50.) == (0., 0.)


def test_multiple_toggles_priority_timeout_and_lane_fallback():
    router = MissionRouter(load_config())
    lane = '/mission/lane'
    parking = '/mission/parking/command_03'
    cone = '/mission/cone/command_04'
    for topic in (cone, lane, parking):
        router.enable(topic)
        router.receive(topic, .2, .1, 10., 10., 50.)
    assert router.selected(10., 50.) == parking
    router.disable(parking)
    assert router.selected(10., 50.) == cone
    router.enable(parking)
    assert router.selected(10., 50.) == cone  # re-enabled input has no old command
    router.receive(lane, .2, .1, 10.2, 10.2, 50.2)
    assert router.selected(10.3, 50.3) == lane  # numbered commands expired
    assert router.output(10.5, 50.5) == (0., 0.)


def test_reverse_is_only_valid_for_parking():
    router = MissionRouter(load_config())
    for _, topic in MISSION_TOPICS:
        router.enable(topic)
        accepted = router.receive(topic, -.2, .1, 10., 10., 50.)
        assert accepted == (topic == '/mission/parking/command_03')
    assert router.output(10., 50.)[0] == -.2


def test_ros_selection_switch_and_disconnect():
    """Use one DDS participant so the test needs no external ROS processes."""
    import rclpy
    from ackermann_msgs.msg import AckermannDriveStamped
    from geometry_msgs.msg import Twist
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rosgraph_msgs.msg import Clock

    # create_adapter follows the default context, like the command-line entry point.
    rclpy.init(domain_id=179)
    context = rclpy.get_default_context()
    cfg = load_config()
    cfg['topics']['cmd_vel'] = '/adapter_test/cmd_vel'
    adapter = create_adapter(cfg)
    probe = Node('mission_adapter_test_probe', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(adapter)
    executor.add_node(probe)
    outputs = []
    probe.create_subscription(Twist, cfg['topics']['cmd_vel'],
                              lambda msg: outputs.append((msg.linear.x, msg.angular.z)), 10)
    clock_pub = probe.create_publisher(Clock, '/clock', 10)
    inputs = {topic: probe.create_publisher(AckermannDriveStamped, topic, 1)
              for topic in ('/adapter_test/cone', '/adapter_test/objavoid')}

    def pump(duration, active=None, speed=.2):
        end = time.monotonic()+duration
        while time.monotonic() < end:
            clock = Clock()
            clock.clock.sec = 10
            clock_pub.publish(clock)
            if active is not None:
                command = AckermannDriveStamped()
                command.header.stamp.sec = 10
                command.drive.speed = speed
                command.drive.steering_angle = .1
                inputs[active].publish(command)
            executor.spin_once(timeout_sec=.005)

    try:
        pump(.1)
        assert not outputs  # opening the window does not seize control
        with pytest.raises(ValueError):
            adapter.select_topic('invalid topic')
        adapter.select_topic('/adapter_test/cone')
        pump(.3, '/adapter_test/cone')
        assert any(math.isclose(v, .2, rel_tol=1e-6) and w > 0 for v, w in outputs)
        count = adapter.router.relays['/adapter_test/cone'].count
        adapter.enable_topic('/adapter_test/objavoid')
        pump(.1, '/adapter_test/cone')
        assert adapter.router.relays['/adapter_test/cone'].count > count
        assert len(adapter.inputs) == 2
        adapter.enable_topic('/adapter_test/objavoid', False)
        old_callback = adapter.inputs['/adapter_test/cone'].callback
        adapter.select_topic('/adapter_test/objavoid')
        old_command = AckermannDriveStamped()
        old_command.header.stamp.sec = 10
        old_command.drive.speed = .9
        old_callback(old_command)  # a callback queued before the switch
        assert adapter.router.relays['/adapter_test/objavoid'].count == 0
        assert adapter.router.output(adapter.sim_time(), time.monotonic()) == (0., 0.)
        pump(.05, '/adapter_test/cone')  # drain output already queued before the switch
        outputs.clear()
        pump(.15, '/adapter_test/cone')
        assert outputs and all(v == 0 for v, _ in outputs)
        pump(.2, '/adapter_test/objavoid', speed=.3)
        assert any(math.isclose(v, .3, rel_tol=1e-6) and w > 0 for v, w in outputs)
        pump(.4)
        assert outputs[-1] == (0., 0.)
        outputs.clear()
        adapter.disconnect()
        assert adapter.publisher is None and not adapter.inputs
        pump(.1)
        assert outputs == [(0., 0.)]
        pump(.1)
        assert outputs == [(0., 0.)]  # disconnected adapter no longer publishes
        # Another cmd_vel publisher prevents the adapter from taking control.
        competing = probe.create_publisher(Twist, cfg['topics']['cmd_vel'], 1)
        pump(.1)
        with pytest.raises(ValueError, match='발행자'):
            adapter.select_topic('/adapter_test/cone')
        probe.destroy_publisher(competing)
    finally:
        adapter.disconnect()
        executor.shutdown()
        adapter.destroy_node()
        probe.destroy_node()
        rclpy.shutdown()
