"""ROS contract checks; run in a private ROS_DOMAIN_ID (e.g. 176)."""
import time

import pytest

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from visualization_msgs.msg import MarkerArray

from autorace_parking.node import ParkingNode


def test_ros_activation_markers_path_control_and_stale_stop():
    from limo_parking_sim.config import load_config
    from limo_parking_sim.model import Simulation
    rclpy.init()
    node = ParkingNode(parameter_overrides=[Parameter('plan_period', value=.1),
                                            Parameter('exit_heading_tolerance', value=.04)])
    assert node.follower.exit_heading_tolerance == .04
    driver = Node('parking_test_driver')
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(driver)
    commands, paths, markers = [], [], []
    subscriptions = [
        driver.create_subscription(AckermannDriveStamped, '/mission/parking/command_03', commands.append, 10),
        driver.create_subscription(Path, '/parking/path', paths.append, 10),
        driver.create_subscription(MarkerArray, '/parking/obstacles', markers.append, 10),
    ]
    signs = driver.create_publisher(Bool, '/yolo/parking_sign', 1)
    def spin_for(duration):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.005)
    try:
        spin_for(.25)
        assert not commands
        for _ in range(2):
            signs.publish(Bool(data=True))
            spin_for(.06)
        assert not node.activation.active
        signs.publish(Bool(data=False))
        spin_for(.06)
        for _ in range(3):
            signs.publish(Bool(data=True))
            spin_for(.06)
        assert node.activation.active
        sim = Simulation(load_config())
        moving = False
        for _ in range(30):
            msg = LaserScan()
            msg.header.frame_id = 'laser_link'
            msg.header.stamp = driver.get_clock().now().to_msg()
            msg.angle_min, msg.angle_increment = float(sim.angles[0]), float(sim.angles[1] - sim.angles[0])
            msg.range_min, msg.range_max = .01, 64.
            msg.ranges = sim.scan().tolist()
            node.on_scan(msg)
            spin_for(.08)
            if any(command.drive.speed > 0 for command in commands):
                moving = True
                break
        assert moving and any(path.poses for path in paths)
        assert any(any(m.ns == 'cluster_box' for m in batch.markers) for batch in markers)
        assert all(abs(command.drive.steering_angle) <= node.cfg.max_steering for command in commands)
        signs.publish(Bool(data=False))
        spin_for(.06)
        assert node.activation.active
        spin_for(.65)
        assert commands[-1].drive.speed == 0.0
        assert not paths[-1].poses
        assert node.state == 'WAITING_SCAN'
        # Each new group of three messages restarts even a latched failure or
        # a completed/departing mission, and clears its recorded route.
        for state in ('TRACKING_LOST', 'PARKED', 'EXITING', 'EXITED'):
            old_follower, old_epoch, old_attempt = node.follower, node.epoch, node.attempt
            node.state = state
            node.tracker.valid = False
            old_follower.entry_trace = [(0., 0., 0.), (.5, 0., 0.)]
            old_follower.path = old_follower.entry_trace.copy()
            for _ in range(2):
                signs.publish(Bool(data=True))
                spin_for(.06)
            assert node.attempt == old_attempt and node.follower is old_follower
            signs.publish(Bool(data=True))
            spin_for(.06)
            assert node.attempt == old_attempt + 1 and node.epoch == old_epoch + 1
            assert node.follower is not old_follower and not node.follower.entry_trace
            assert not node.follower.path and not node.follower.parked
            assert node.tracker.valid and node.state == 'WAITING_SCAN'
            assert commands[-1].drive.speed == 0.
        node.on_reset(None, Trigger.Response())
        assert node.follower.exit_heading_tolerance == .04
        spin_for(.08)
        before = len(commands)
        spin_for(.15)
        assert len(commands) == before  # inactive node releases the mux
    finally:
        executor.remove_node(node)
        executor.remove_node(driver)
        node.destroy_node()
        driver.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


@pytest.mark.parametrize('future_seconds', [1e-9, .05])
def test_fresh_scan_ahead_of_clock_does_not_discard_entry_path(future_seconds):
    from autorace_parking.control import build_plan
    from autorace_parking.core import Scan
    from limo_parking_sim.config import load_config
    from limo_parking_sim.model import Simulation

    rclpy.init()
    node = ParkingNode()
    node.now_seconds = lambda: 20.
    sim = Simulation(load_config())
    try:
        for _ in range(3):
            node.on_sign(Bool(data=True))
        msg = LaserScan()
        ns = round((20.+future_seconds)*1e9)
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(ns, 10**9)
        msg.header.frame_id = 'laser_link'
        msg.angle_min = float(sim.angles[0])
        msg.angle_increment = float(sim.angles[1]-sim.angles[0])
        msg.range_min, msg.range_max = .01, 64.
        msg.ranges = sim.scan().tolist()
        node.on_scan(msg)
        assert node.scan is not None and node.tracker.valid
        scan = Scan(msg.ranges, msg.angle_min, msg.angle_increment, .01, 64.)
        node.follower.accept(*build_plan(scan, node.cfg), node.tracker.pose)
        route = node.follower.path.copy()
        assert route
        node.last_plan_wall = time.monotonic()
        node.tick()
        assert node.state == 'APPROACHING'
        assert node.follower.path == route
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_pending_replan_does_not_replace_accepted_final_entry_path():
    from concurrent.futures import Future
    from autorace_parking.core import Gap

    rclpy.init()
    node = ParkingNode()
    node.now_seconds = lambda: 20.
    try:
        for _ in range(3):
            node.on_sign(Bool(data=True))
        goal = Gap(1., 0., 0., .8, 0, 1)
        route = [(i/100, 0., 0.) for i in range(101)]
        node.follower.accept(goal, route, True, (0., 0., 0.))
        node.state = 'CONFIRMING'
        future = Future()
        future.set_result((Gap(2., 0., 0., .8, 0, 1), [(0., 0., 0.), (2., 0., 0.)], False))
        node.pending = future, node.epoch, (0., 0., 0.), 20.
        node.poll_plan()
        assert node.pending is None
        assert node.follower.path == route and node.follower.final_approach
        assert node.follower.goal == goal
    finally:
        node.destroy_node()
        rclpy.shutdown()
