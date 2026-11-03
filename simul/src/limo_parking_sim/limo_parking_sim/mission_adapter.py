"""Toggle mission inputs and forward the freshest command with highest priority."""
import argparse
import math
import sys
import time

from .config import load_config
from .model import Simulation
from .parking_adapter import drive_to_twist


MISSION_TOPICS = (
    ('횡단보도 (crosswalk)', '/mission/crosswalk/command_01'),
    ('신호등 (signal)', '/mission/signal/command_02'),
    ('주차 (parking)', '/mission/parking/command_03'),
    ('콘 (cone)', '/mission/cone/command_04'),
    ('장애물 회피 (objavoid)', '/mission/objavoid/command_05'),
    ('서행 · 방지턱 (slow)', '/mission/slow/command_08'),
    ('차선 · 갈림길 (lane)', '/mission/lane'),
)
ACKERMANN_TYPE = 'ackermann_msgs/msg/AckermannDriveStamped'
TIMEOUT = .25
PRIORITIES = {topic: int(topic.rsplit('_', 1)[-1]) if '/command_' in topic else 1000
              for _, topic in MISSION_TOPICS}


class CommandRelay:
    """No ROS/GUI dependencies: selection, conversion and command expiration."""

    def __init__(self, cfg):
        vehicle = cfg['vehicle']
        self.limits = (vehicle['wheelbase'], vehicle['max_speed'], Simulation(cfg).max_steering)
        self.topic = None
        self.count = 0
        self.select(None)

    def clear(self):
        self.value = (0., 0.)
        self.received = -math.inf
        self.source_stamp = None

    def select(self, topic):
        self.topic = topic
        self.count = 0
        self.clear()

    def receive(self, topic, speed, steering, stamp, sim_time, wall_time):
        if self.topic is None or topic != self.topic:
            return False
        if (not all(math.isfinite(x) for x in (speed, steering, stamp, sim_time))
                or not -.05 <= sim_time-stamp <= TIMEOUT):
            self.clear()
            return False
        self.value = drive_to_twist(speed, steering, *self.limits)
        self.received, self.source_stamp = wall_time, stamp
        self.count += 1
        return True

    def fresh(self, sim_time, wall_time):
        return (self.topic is not None and self.source_stamp is not None
                and 0 <= wall_time-self.received <= TIMEOUT
                and -.05 <= sim_time-self.source_stamp <= TIMEOUT)

    def output(self, sim_time, wall_time):
        return self.value if self.fresh(sim_time, wall_time) else (0., 0.)


class MissionRouter:
    """The docs contract: enabled, fresh numbered commands first, lane last."""

    def __init__(self, cfg):
        self.cfg = cfg
        self.relays = {}

    def enable(self, topic):
        relay = CommandRelay(self.cfg)
        relay.select(topic)
        self.relays[topic] = relay

    def disable(self, topic):
        self.relays.pop(topic, None)

    def receive(self, topic, speed, steering, stamp, sim_time, wall_time):
        relay = self.relays.get(topic)
        if relay is None:
            return False
        if speed < 0 and topic != '/mission/parking/command_03':
            relay.clear()
            return False
        return relay.receive(topic, speed, steering, stamp, sim_time, wall_time)

    def selected(self, sim_time, wall_time):
        for topic in sorted(self.relays, key=lambda item: (PRIORITIES.get(item, 100), item)):
            if self.relays[topic].fresh(sim_time, wall_time):
                return topic
        return None

    def output(self, sim_time, wall_time):
        topic = self.selected(sim_time, wall_time)
        return self.relays[topic].value if topic else (0., 0.)


def create_adapter(cfg):
    from geometry_msgs.msg import Twist
    from rclpy.clock import Clock, ClockType
    from rclpy.exceptions import InvalidTopicNameException
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rclpy.validate_full_topic_name import validate_full_topic_name
    try:
        from ackermann_msgs.msg import AckermannDriveStamped
    except ImportError as exc:
        raise RuntimeError('Ackermann 메시지가 필요합니다: sudo apt install ros-humble-ackermann-msgs') from exc

    class Adapter(Node):
        def __init__(self):
            super().__init__('mission_sim_adapter')
            self.set_parameters([Parameter('use_sim_time', value=True)])
            self.router = MissionRouter(cfg)
            self.output_topic = cfg['topics']['cmd_vel']
            self.publisher = None
            self.inputs = {}
            self.tokens = {}
            self.generation = 0
            self.create_timer(1./30, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

        def sim_time(self):
            return self.get_clock().now().nanoseconds/1e9

        def enable_topic(self, topic, enabled=True):
            if not enabled:
                self.tokens.pop(topic, None)
                subscription = self.inputs.pop(topic, None)
                if subscription is not None:
                    self.destroy_subscription(subscription)
                self.router.disable(topic)
                self.tick()
                if not self.inputs and self.publisher is not None:
                    self.destroy_publisher(self.publisher)
                    self.publisher = None
                return
            if topic in self.inputs:
                return
            try:
                validate_full_topic_name(topic)
            except InvalidTopicNameException as exc:
                raise ValueError(str(exc)) from exc
            if topic == self.output_topic:
                raise ValueError('입력 토픽과 출력 토픽은 달라야 합니다.')
            types = dict(self.get_topic_names_and_types()).get(topic, [])
            if types and types != [ACKERMANN_TYPE]:
                raise ValueError(f'입력 타입은 {ACKERMANN_TYPE}이어야 합니다: {types}')
            if self.count_publishers(self.output_topic) > int(self.publisher is not None):
                raise ValueError(f'{self.output_topic} 발행자가 이미 있습니다. mux 또는 다른 어댑터를 종료하세요.')
            self.router.enable(topic)
            if self.publisher is None:
                self.publisher = self.create_publisher(Twist, self.output_topic, QoSProfile(depth=1))
            self.generation += 1
            generation = self.generation
            self.tokens[topic] = generation

            def receive(msg):
                if self.tokens.get(topic) != generation:
                    return
                stamp = msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
                self.router.receive(topic, msg.drive.speed, msg.drive.steering_angle,
                                    stamp, self.sim_time(), time.monotonic())

            # A best-effort subscription accepts both best-effort and reliable mission publishers.
            self.inputs[topic] = self.create_subscription(
                AckermannDriveStamped, topic, receive,
                QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
            self.get_logger().info(f'ON: {topic} -> {self.output_topic}')

        def select_topic(self, topic):
            self.disconnect()
            self.enable_topic(topic)

        def publish(self, value):
            if self.publisher is not None:
                msg = Twist()
                msg.linear.x, msg.angular.z = value
                self.publisher.publish(msg)

        def tick(self):
            self.publish(self.router.output(self.sim_time(), time.monotonic()))

        def disconnect(self):
            self.publish((0., 0.))
            self.tokens.clear()
            for topic, subscription in self.inputs.items():
                self.destroy_subscription(subscription)
                self.router.disable(topic)
            self.inputs.clear()
            if self.publisher is not None:
                self.destroy_publisher(self.publisher)
                self.publisher = None

    return Adapter()


def main(args=None):
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.utilities import remove_ros_args

    argv = sys.argv[1:] if args is None else args
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config')
    parser.add_argument('--topic', action='append', help='시작할 때 켤 입력 토픽 (여러 번 지정 가능)')
    parser.add_argument('--no-gui', action='store_true', help='선택 창 없이 실행 (--topic 필요)')
    options = parser.parse_args(remove_ros_args(['mission_adapter', *argv])[1:])
    if options.no_gui and not options.topic:
        parser.error('--no-gui에는 --topic이 필요합니다.')
    rclpy.init(args=argv)
    node, window = None, None
    try:
        node = create_adapter(load_config(options.config))
        for topic in options.topic or []:
            node.enable_topic(topic)
        if options.no_gui:
            rclpy.spin(node)
        else:
            from .adapter_window import AdapterWindow
            window = AdapterWindow(node)
            window.run()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except (ValueError, RuntimeError) as exc:
        raise SystemExit(str(exc)) from exc
    finally:
        if window is not None:
            window.close()
        if node is not None:
            if rclpy.ok():
                node.disconnect()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
