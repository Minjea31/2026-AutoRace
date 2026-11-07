"""Parking calibration must survive the real mux and legacy driver conversions."""
import ast
from dataclasses import replace
import importlib.util
import math
from pathlib import Path
import subprocess
import sys

import pytest

from autorace_parking.core import Settings
from autorace_parking.steering_command import parking_steering_command


ROOT = Path(__file__).resolve().parents[4]


def calibrated_settings():
    return replace(Settings(), require_steering_calibration=True, min_turn_radius=.55)


@pytest.fixture(scope='module')
def driver_probe(tmp_path_factory):
    """Compile the driver's actual geometry and command model without hardware IO."""
    base = ROOT / 'sensor_ws/src/limo_ros2/limo_base'
    text = (base / 'src/limo_driver.cpp').read_text()
    start = text.index('double LimoDriver::convertCentralAngleToInner(')
    end = text.index('\n}', start) + 2
    geometry = text[start:end].replace('LimoDriver::', '')
    start = text.index('            double r = msg->linear.x / msg->angular.z;')
    end = text.index('\n            double inner_angle =', start)
    conversion = text[start:end].replace('msg->linear.x', 'speed').replace('msg->angular.z', 'yaw')
    folder = tmp_path_factory.mktemp('legacy_driver_probe')
    source, binary = folder / 'probe.cpp', folder / 'probe'
    header = base / 'include/limo_base/steering_command_model.hpp'
    source.write_text('#include "' + str(header) + '"\n' + r'''
#include <iostream>
#include <iomanip>
constexpr double wheelbase_ = .2, track_ = .172;
''' + geometry + r'''
int main() {
    double speed, yaw; bool calibration;
    std::cout << std::setprecision(17);
    while (std::cin >> speed >> yaw >> calibration) {
        if (std::abs(speed) < 1e-6) { std::cout << 0. << '\n'; continue; }
''' + conversion + r'''
        AgileX::SteeringCommandModel model; model.enabled = calibration;
        std::cout << model.encode(convertCentralAngleToInner(central_angle), 2.47) << '\n';
    }
}
''')
    subprocess.run(['g++', '-std=c++14', '-Wall', '-Wextra', str(source), '-o', str(binary)],
                   check=True, capture_output=True)
    return binary


def test_parking_wire_output_is_preserved_through_mux_and_legacy_driver(driver_probe):
    path = ROOT / 'mux_ws/src/autorace_mux/autorace_mux/core.py'
    spec = importlib.util.spec_from_file_location('parking_calibration_mux_test', path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    cfg = calibrated_settings()
    rows = []
    # Forward/reverse and mux speed ramping must preserve the signed angle.
    for speed in (.2, -.2):
        for angle in (0., 1e-8, -1e-8, .01, -.01, .2, -.2, cfg.steering_limit, -cfg.steering_limit, .6, -.6):
            command = parking_steering_command(angle, cfg)
            converter = module.TwistConverter(module.Config())
            for tick in range(3):
                candidate = module.Candidate('parking', speed, command, 1, 0.)
                v, yaw = converter.convert(candidate, tick * .05)
                # Desired physical angle through the old calibrated driver.
                rows.append((v, v * math.tan(angle) / cfg.wheelbase, 1))
                # Encoded parking angle through the now-legacy shared driver.
                rows.append((v, yaw, 0))
    payload = ''.join(f'{v:.17g} {w:.17g} {calibration}\n' for v, w, calibration in rows)
    result = subprocess.run([str(driver_probe)], input=payload, text=True, capture_output=True, check=True)
    wire = [float(line) for line in result.stdout.splitlines()]
    assert len(wire) == len(rows)
    for expected, actual in zip(wire[::2], wire[1::2]):
        assert actual == pytest.approx(expected, abs=1e-12)
        assert abs(actual) <= .35 + 1e-12


def test_neutral_symmetry_and_opt_out_preserve_unencoded_angles():
    cfg = calibrated_settings()
    assert parking_steering_command(0., cfg) == 0.
    for angle in (.01, .2, cfg.steering_limit, .6):
        assert parking_steering_command(-angle, cfg) == -parking_steering_command(angle, cfg)
        assert parking_steering_command(angle, replace(cfg, require_steering_calibration=False)) == angle


@pytest.mark.parametrize('value', [math.nan, math.inf, -math.inf, math.pi / 2, -math.pi / 2])
def test_invalid_physical_angles_are_rejected(value):
    with pytest.raises(ValueError):
        parking_steering_command(value, calibrated_settings())


def test_node_encodes_only_published_angles_and_sends_neutral_stop():
    import rclpy
    from rclpy.parameter import Parameter
    from autorace_parking.node import ParkingNode
    rclpy.init()
    node = ParkingNode(parameter_overrides=[Parameter('localization_source', value='scan'),
        Parameter('require_steering_calibration', value=True), Parameter('min_turn_radius', value=.55)])
    try:
        from unittest.mock import Mock
        node.command_pub = Mock()
        before = node.cfg.steering_limit
        node.publish_command(.2, .2)
        msg = node.command_pub.publish.call_args.args[0]
        assert msg.drive.speed == .2
        assert msg.drive.steering_angle == pytest.approx(parking_steering_command(.2, node.cfg))
        assert msg.header.frame_id == node.base_frame
        assert node.follower.cfg.steering_limit == before
        node.publish_command(0., .2)
        msg = node.command_pub.publish.call_args.args[0]
        assert msg.drive.speed == msg.drive.steering_angle == 0.
        result = node.set_parameters_atomically([Parameter('steering_command_limit_expected', value=.7)])
        assert not result.successful
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_sensor_launch_defaults_to_legacy_encoding_for_lane():
    path = ROOT / 'sensor_ws/src/autorace_sensor_bringup/launch/sensors.launch.py'
    tree = ast.parse(path.read_text())
    defaults = next(n.value for n in ast.walk(tree)
                    if isinstance(n, ast.Assign) and any(isinstance(t, ast.Name) and t.id == 'defaults' for t in n.targets))
    values = {key.value: value for key, value in zip(defaults.keys, defaults.values) if isinstance(key, ast.Constant)}
    assert ast.literal_eval(values['steering_command_calibration']) == 'false'
