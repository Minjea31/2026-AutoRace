"""Completed missions release control; rectangular clearance remains physical."""
from dataclasses import replace
import math
from unittest.mock import Mock, patch

import pytest
from std_msgs.msg import Bool

from autorace_parking.core import CollisionChecker, Settings, Scan
from autorace_parking.departure import single_reverse_route
from test_operator_ros import node, activate
from test_obstacle_collision_mode import profile


@pytest.mark.parametrize('yaw', [0., math.pi/4, math.pi/2])
def test_five_cm_margin_expands_same_rectangular_body(yaw):
    cfg = profile()
    assert cfg.clearance == .05
    assert (cfg.vehicle_length, cfg.vehicle_width) == (.322, .22)
    scan = Scan([math.inf]*361, -math.pi, math.pi/180, .01, 12.)
    checker = CollisionChecker(scan, cfg)
    assert checker.half_length == pytest.approx(.211)
    assert checker.half_width == pytest.approx(.160)
    # A point between 4 and 5 cm from a body face enters the new box.
    for x, y in ((cfg.vehicle_length/2+.045, 0.),
                 (-cfg.vehicle_length/2-.045, 0.),
                 (0., cfg.vehicle_width/2+.045),
                 (0., -cfg.vehicle_width/2-.045)):
        scan.points = [(x*math.cos(yaw)-y*math.sin(yaw),
                        x*math.sin(yaw)+y*math.cos(yaw))]
        assert CollisionChecker(scan, replace(cfg, clearance=.04)).free((0., 0., yaw))
        assert not CollisionChecker(scan, cfg).free((0., 0., yaw))
    scan.points = [(.212*math.cos(yaw), .212*math.sin(yaw))]
    assert CollisionChecker(scan, cfg).free((0., 0., yaw))


def test_exit_alignment_reaches_validated_lane_return_with_enlarged_body():
    cfg = profile()
    scan = Scan([math.inf]*361, -math.pi, math.pi/180, .01, 12.)
    pose = (.83, .04, .10)
    targets = [(.49, .025, .025), (.15, 0., 0.), (-.3, 0., 0.)]
    trace = [(0., 0., 0.), (.49, .025, .025), pose]
    route, reason = single_reverse_route(trace, pose, targets, scan, cfg)
    assert route, reason
    # A 6 cm alignment arc ends too early; it must return to the safe lane.
    assert route[-1][0] <= targets[0][0] + .02


def test_completed_exit_survives_unavailable_sensors_and_releases_mux(node):
    activate(node)
    node.follower.exited = True
    node.follower.parked = True
    node.follower.path = [(0., 0., 0.), (.2, 0., 0.)]
    node.had_control_path = True
    node.state = node.reported_state = 'EXITED'
    node.reason = 'Old localization error'
    node.scan = None
    node.scan_time = None
    node.tracker.valid = False
    node.localization_ready = Mock(return_value=False)
    node.steering_model_ready = Mock(return_value=False)
    node.follower.command = Mock()
    node.publish_command = Mock()
    node.worker.submit = Mock()
    for _ in range(3):
        node.tick()
        assert node.state == 'EXITED' and node.reason == ''
        assert not node.follower.path and not node.had_control_path
    node.localization_ready.assert_not_called()
    node.follower.command.assert_not_called()
    node.worker.submit.assert_not_called()
    node.publish_command.assert_called_once_with(0., 0.)


def test_new_sign_after_completion_starts_fresh_sensor_gated_mission(node):
    with patch('autorace_parking.node.time.monotonic', return_value=100.):
        activate(node)
    completed = node.follower
    completed.exited = completed.parked = True
    node.state = 'EXITED'
    with patch('autorace_parking.node.time.monotonic', return_value=105.):
        node.on_sign(Bool(data=True))
    assert node.attempt == 2 and node.activation.active
    assert node.follower is not completed and not node.follower.exited
    node.localization_ready = Mock(return_value=False)
    node.tick()
    assert node.state == 'WAITING_LOCALIZATION'
    node.localization_ready.assert_called_once()
