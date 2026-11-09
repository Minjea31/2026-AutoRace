import os
import shlex
from pathlib import Path

from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription,
    LogInfo, RegisterEventHandler, GroupAction, SetLaunchConfiguration, TimerAction, OpaqueFunction,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration as C
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _setup(context):
    repo = Path.home() / '2026-AutoRace'

    def include(package, filename, arguments):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(share(package) + '/launch/' + filename),
            launch_arguments=arguments.items(),
        )

    defaults = _defaults()
    for key in ('lane_model', 'yolo_model', 'parking_params'):
        if not Path(defaults[key]).is_file():
            raise RuntimeError('Missing file: ' + defaults[key])

    runner = Path(share('autorace_sensor_bringup')) / 'scripts/performance_runner.py'
    process_prefix = shlex.join(['/usr/bin/python3', str(runner), '--role', 'vision', '--'])

    def heavy(action, delay):
        group = GroupAction(actions=[SetLaunchConfiguration('launch-prefix', process_prefix), action])
        return TimerAction(period=delay, actions=[group]) if delay else group

    mux = include('autorace_mux', 'mux.launch.py', {'output_topic': C('output_topic')})
    timeout = ParameterValue(C('startup_timeout'), value_type=float)
    ready = Node(
        package='tensor_bringup', executable='ready.py',
        parameters=[{'timeout': timeout, 'phase': 'inputs'}], output='screen',
    )
    report = Node(
        package='tensor_bringup', executable='ready.py', name='tensor_startup_report',
        parameters=[{'timeout': timeout, 'phase': 'nodes'}], output='screen',
    )

    def inputs_ready(event, context):
        if event.returncode == 0:
            return [mux, report]
        return [
            LogInfo(msg='[bringup 결과] 시작 실패: 위의 미실행 노드·미수신 토픽을 확인하세요.'),
            EmitEvent(event=Shutdown(reason='Mission inputs not ready')),
        ]

    def reported(event, context):
        if event.returncode == 0:
            return [LogInfo(msg='[bringup 결과] 전체 노드 7/7 실행 확인 완료 (mux 포함, 입력 토픽 정상).')]
        return [LogInfo(msg='[bringup 결과] 전체 실행 확인 실패: 위의 누락 항목을 확인하세요.')]

    return [
        LogInfo(msg='Mission startup stagger: parking 0s, slow 2s, lane 4s, YOLO 8s'),
        RegisterEventHandler(OnProcessExit(target_action=ready, on_exit=inputs_ready)),
        RegisterEventHandler(OnProcessExit(target_action=report, on_exit=reported)),
        heavy(include('autorace_parking', 'parking.launch.py', {
            'use_sim_time': 'false', 'rviz': 'false', 'params_file': C('parking_params'),
        }), 0.),
        heavy(include('autorace_slow', 'slow.launch.py', {'use_sim_time': 'false'}), 2.),
        heavy(include('autorace_lane_fusion_pid', 'lane_fusion_pid.launch.py', {
            'profile': 'real', 'use_sim_time': 'false', 'deep': 'true',
            'model_path': C('lane_model'), 'debug': 'true', 'gui': 'false', 'lane_gui': 'false',
            'lane_speed': C('lane_speed'),
        }), 4.),
        heavy(Node(
            package='yolo_detect', executable='yolo_detect',
            parameters=[{'use_sim_time': False, 'model_path': C('yolo_model')}],
            additional_env={'PYTHONPATH': str(repo / 'yolo_ws/yolo26') + ':' + os.environ.get('PYTHONPATH', '')},
            output='screen',
        ), 8.),
        TimerAction(period=8., actions=[ready]),
    ]


def _defaults():
    repo = Path.home() / '2026-AutoRace'
    return {
        'lane_model': str(repo / 'lane_fusion_ws/src/autorace_lane_fusion_deep/model/lane_640_mask.engine'),
        'yolo_model': str(repo / 'yolo_ws/model/best.engine'),
        'parking_params': share('autorace_parking') + '/config/parking_real.yaml',
        'startup_timeout': '60.0', 'output_topic': '/cmd_vel',
        'lane_speed': '0.5',
    }


def generate_launch_description():
    return LaunchDescription([
        *[DeclareLaunchArgument(key, default_value=value) for key, value in _defaults().items()],
        OpaqueFunction(function=_setup),
    ])
