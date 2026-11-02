"""ROS contract checks; run in a private ROS_DOMAIN_ID (e.g. 176)."""
import time

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
    node = ParkingNode(parameter_overrides=[Parameter('plan_period', value=.1)])
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
