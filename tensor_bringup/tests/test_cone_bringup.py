"""Cone startup wiring and readiness, without launching vehicle controllers."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest
from launch import LaunchContext
from launch.actions import GroupAction, IncludeLaunchDescription, TimerAction


ROOT = Path(__file__).resolve().parents[1]


def load(relative):
    spec = importlib.util.spec_from_file_location('bringup_under_test', ROOT / relative)
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


@pytest.mark.parametrize('mode', ['fusion', 'opencv', 'deep'])
def test_cone_launch_uses_real_clock_and_selected_config(monkeypatch, tmp_path, mode):
    module = load('launch/tensor_bringup.launch.py')
    monkeypatch.setattr(module, 'share', lambda package: str(tmp_path / package))
    monkeypatch.setattr(module, '_lane', lambda *args: [])
    defaults = module._defaults()
    context = LaunchContext()
    for key, value in defaults.items():
        context.launch_configurations[key] = value
    context.launch_configurations['lane_mode'] = mode
    config = tmp_path / 'custom-cone.yaml'
    config.write_text('cone_lane_roi: {}')
    context.launch_configurations['cone_params'] = str(config)
    for key in ('lane_model', 'yolo_model', 'parking_params'):
        path = tmp_path / (key + '.yaml')
        path.write_text('')
        context.launch_configurations[key] = str(path)
    actions = module._setup(context)
    includes = [a for a in walk(actions) if isinstance(a, IncludeLaunchDescription)]
    for action in includes:
        source = action.launch_description_source
        monkeypatch.setattr(source, '_get_launch_description', lambda location: None)
        source.get_launch_description(context)  # Resolve paths without loading mission nodes.
    cone = [a for a in includes if 'cone_lane_follower' in
            a.launch_description_source.location]
    assert len(cone) == 1
    arguments = dict(cone[0].launch_arguments)
    assert arguments['use_sim_time'] == 'false'
    assert arguments['rviz'] == 'false'
    assert arguments['params_file'].perform(context) == str(config)
    config.unlink()
    with pytest.raises(RuntimeError, match='Missing file: .*custom-cone.yaml'):
        module._setup(context)


def test_readiness_requires_cone_node_and_fresh_status_but_no_drive_command():
    module = load('scripts/ready.py')
    assert '/cone_lane_roi' in module.BASE_NODES
    assert '/cone_lane/control_status' in module.BASE_TOPICS
    assert '/mission/cone/command_06' not in module.BASE_TOPICS
    names = [(name.lstrip('/'), '/') for name in module.BASE_NODES
             if name != '/cone_lane_roi']
    graph = SimpleNamespace(get_node_names_and_namespaces=lambda: names)
    seen = dict.fromkeys(module.INPUT_TOPICS, 10.0)
    missing_nodes, missing_topics = module.missing_inputs(graph, module.BASE_NODES, seen, 10.1)
    assert missing_nodes == ['/cone_lane_roi']
    assert missing_topics == []
    names.append(('cone_lane_roi', '/'))
    seen['/cone_lane/control_status'] = 9.0
    assert module.missing_inputs(graph, module.BASE_NODES, seen, 10.1) == (
        [], ['/cone_lane/control_status'])
    seen['/cone_lane/control_status'] = 10.1
    assert module.missing_inputs(graph, module.BASE_NODES, seen, 10.1) == ([], [])
