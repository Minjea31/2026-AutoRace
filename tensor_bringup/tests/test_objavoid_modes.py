"""Recovery mode selection without starting vehicle controllers."""
import importlib.util
from pathlib import Path

import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, TimerAction

REPO = Path(__file__).resolve().parents[2]


def load(relative):
    spec = importlib.util.spec_from_file_location('objavoid_mode_under_test', REPO / relative)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def walk(actions):
    for action in actions:
        yield action
        if isinstance(action, GroupAction):
            yield from walk(action.get_sub_entities())
        elif isinstance(action, TimerAction):
            yield from walk(action.actions)


@pytest.mark.parametrize('mode', ['normal', 'reverse'])
def test_integrated_mode_reaches_objavoid_launch(monkeypatch, tmp_path, mode):
    module = load('tensor_bringup/launch/tensor_bringup.launch.py')
    monkeypatch.setattr(module, 'share', lambda package: str(tmp_path / package))
    monkeypatch.setattr(module, '_lane', lambda *args: [])
    defaults = module._defaults()
    assert defaults['objavoid_mode'] == 'normal'
    context = LaunchContext()
    context.launch_configurations.update(defaults)
    context.launch_configurations['objavoid_mode'] = mode
    for key in ('lane_model', 'yolo_model', 'parking_params', 'cone_params'):
        path = tmp_path / (key + '.yaml')
        path.write_text('')
        context.launch_configurations[key] = str(path)
    includes = [a for a in walk(module._setup(context)) if isinstance(a, IncludeLaunchDescription)]
    for action in includes:
        source = action.launch_description_source
        monkeypatch.setattr(source, '_get_launch_description', lambda location: None)
        source.get_launch_description(context)
    objavoid = next(a for a in includes if 'objavoid_v3.launch.py' in a.launch_description_source.location)
    arguments = dict(objavoid.launch_arguments)
    assert arguments['recovery_mode'].perform(context) == mode
    assert arguments['command_topic'] == '/mission/objavoid/command_05'


@pytest.mark.parametrize('mode', ['normal', 'reverse'])
@pytest.mark.parametrize('car', ['', '04'])
@pytest.mark.parametrize('impl', ['cpp', 'py'])
def test_selected_mode_overrides_vehicle_config(monkeypatch, tmp_path, mode, car, impl):
    module = load('objavoid_ws/src/autorace_objavoid_v3/launch/objavoid_v3.launch.py')
    monkeypatch.setattr(module, 'get_package_share_directory', lambda package: str(tmp_path))
    monkeypatch.setattr(module, 'Node', lambda **kwargs: kwargs)
    description = module.generate_launch_description()
    context = LaunchContext()
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument):
            context.launch_configurations[action.name] = ''.join(
                substitution.perform(context) for substitution in action.default_value)
    assert context.launch_configurations['recovery_mode'] == 'normal'
    context.launch_configurations.update(recovery_mode=mode, car=car, impl=impl,
                                        lane_config=str(tmp_path / 'lane.yaml'))
    node = module._nodes(context)[0]
    merged = {}
    for params in node['parameters']:
        if isinstance(params, dict):
            merged.update(params)
    assert merged['reverse_enable'] is (mode == 'reverse')
    if car:
        assert str(tmp_path / 'config' / 'real_car04.yaml') in node['parameters']


def test_unknown_recovery_mode_is_rejected(monkeypatch, tmp_path):
    module = load('objavoid_ws/src/autorace_objavoid_v3/launch/objavoid_v3.launch.py')
    monkeypatch.setattr(module, 'get_package_share_directory', lambda package: str(tmp_path))
    context = LaunchContext()
    context.launch_configurations['recovery_mode'] = 'typo'
    with pytest.raises(RuntimeError, match='Unknown recovery_mode'):
        module._nodes(context)
