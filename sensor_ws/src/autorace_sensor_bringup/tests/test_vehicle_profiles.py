import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument


def load_launch(monkeypatch, tmp_path):
    path = Path(__file__).resolve().parents[1] / 'launch/sensors.launch.py'
    spec = importlib.util.spec_from_file_location('vehicle_sensor_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, 'launch_config', SimpleNamespace(log_dir=str(tmp_path)))
    monkeypatch.setattr(module, 'serial_port_owners', lambda paths: {})
    # Inspect what the launch passes to nodes without executing a driver.
    monkeypatch.setattr(module, 'Node', lambda **kwargs: SimpleNamespace(**kwargs))
    context = LaunchContext()
    for action in module.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    for feature in ('lidar', 'camera', 'description', 'camera_tf', 'ekf'):
        context.launch_configurations['enable_' + feature] = 'false'
    return module, context


@pytest.mark.parametrize('vehicle, expected', [(None, 0.0), ('06', 0.0), ('04', 0.0)])
def test_vehicle_selects_driver_neutral_offset(monkeypatch, tmp_path, vehicle, expected):
    module, context = load_launch(monkeypatch, tmp_path)
    if vehicle is not None:
        context.launch_configurations['vehicle'] = vehicle
    actions = module._setup(context)
    base = next(action for action in actions if getattr(action, 'package', None) == 'limo_base')
    assert base.parameters[0]['steering_command_offset'] == expected
    assert base.parameters[0]['steering_command_calibration'] is False
    session = json.loads((tmp_path / 'session.json').read_text())
    assert session['options']['vehicle'] == (vehicle or '06')
    assert session['steering_command_offset'] == expected


@pytest.mark.parametrize('vehicle, override, expected', [
    (None, None, 0.35), ('06', None, 0.35), ('04', None, 0.48), ('04', '0.42', 0.42)])
def test_vehicle_selects_steering_command_limit(monkeypatch, tmp_path, vehicle, override, expected):
    module, context = load_launch(monkeypatch, tmp_path)
    if vehicle is not None:
        context.launch_configurations['vehicle'] = vehicle
    if override is not None:
        context.launch_configurations['steering_command_limit'] = override
    actions = module._setup(context)
    base = next(action for action in actions if getattr(action, 'package', None) == 'limo_base')
    assert base.parameters[0]['steering_command_limit'] == expected
    assert json.loads((tmp_path / 'session.json').read_text())['steering_command_limit'] == expected


@pytest.mark.parametrize('vehicle', ['4', '07', ''])
def test_unknown_vehicle_fails_before_accessing_ports(monkeypatch, tmp_path, vehicle):
    module, context = load_launch(monkeypatch, tmp_path)
    context.launch_configurations['vehicle'] = vehicle
    def unexpected_access(paths):
        pytest.fail('Invalid vehicle reached serial port checks')
    monkeypatch.setattr(module, 'serial_port_owners', unexpected_access)
    with pytest.raises(ValueError, match='vehicle must be 04 or 06'):
        module._setup(context)
