import math
import os
import shlex
from pathlib import Path

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory as share
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


# lane_mode: which lane-keeping stack publishes /mission/lane.
#   fusion: lane_fusion_ws  deep mask + HSV -> lane_fusion_node -> fusion PID   (default)
#   opencv: lane_ws         lane_lite (HSV/OpenCV) -> lane PID   (source lane_ws too)
#   deep:   lane_fusion_ws  deep mask -> lane_deep_lite (lane_ws logic on the mask) -> lane PID
#                           (source lane_ws too: the PID is lane_ws autorace_lane_pid)
LANE_NODES = {
    'fusion': ('/lane_fusion_deep_mask', '/lane_fusion_node', '/lane_fusion_pid_controller'),
    'opencv': ('/lane_lite_node', '/lane_pid_controller'),
    'deep': ('/lane_fusion_deep_mask', '/lane_deep_lite_node', '/lane_pid_controller'),
}


def _lane(mode, speed_text, model):
    if speed_text.strip():
        value = float(speed_text)
        if not math.isfinite(value) or not 0.0 <= value <= 1.0:
            raise ValueError('lane_speed must be a finite speed between 0 and 1 m/s')
    if mode == 'opencv':
        # lane_ws as-is: lane_lite perception + lane PID with their real configs.
        lite, pid = share('autorace_lane_lite'), share('autorace_lane_pid')
        speed = {'speed': float(speed_text)} if speed_text.strip() else {}
        return [
            Node(package='autorace_lane_lite', executable='lane_lite_node', output='screen',
                 parameters=[{'config_file': lite + '/config/real/lane.yaml', 'use_sim_time': False,
                              'debug': True}]),
            Node(package='autorace_lane_pid', executable='lane_pid_controller', output='screen',
                 parameters=[pid + '/config/real/pid.yaml',
                             {'use_sim_time': False, 'enable_vehicle_output': True}, speed]),
        ]
    if mode == 'deep':
        # Detection = deep mask, everything after = lane_ws (lane_deep_lite), lane_ws PID.
        deep_lite, pid = share('autorace_lane_deep_lite'), share('autorace_lane_pid')
        speed = {'speed': float(speed_text)} if speed_text.strip() else {}
        return [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(share('autorace_lane_fusion_deep') + '/launch/deep_mask.launch.py'),
                launch_arguments={'model_path': model, 'use_sim_time': 'false'}.items(),
            ),
            Node(package='autorace_lane_deep_lite', executable='lane_deep_lite_node', output='screen',
                 parameters=[{'config_file': deep_lite + '/config/real/lane.yaml', 'use_sim_time': False,
                              'debug': True}]),
            Node(package='autorace_lane_pid', executable='lane_pid_controller', output='screen',
                 parameters=[pid + '/config/real/pid.yaml',
                             {'use_sim_time': False, 'enable_vehicle_output': True,
                              'target_topic': '/lane_deep/target'}, speed]),
        ]
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(share('autorace_lane_fusion_pid') + '/launch/lane_fusion_pid.launch.py'),
        launch_arguments={
            'profile': 'real', 'use_sim_time': 'false',
            'deep': 'true', 'hsv': 'true',
            'model_path': model, 'debug': 'true', 'gui': 'false', 'lane_gui': 'false',
            'lane_speed': speed_text,
        }.items(),
    )]


def _setup(context):
    repo = Path.home() / '2026-AutoRace'
    mode = C('lane_mode').perform(context)
    if mode not in LANE_NODES:
        raise RuntimeError('lane_mode must be fusion, opencv or deep: ' + mode)
    try:
        lane = _lane(mode, C('lane_speed').perform(context), C('lane_model').perform(context))
    except PackageNotFoundError as error:
        ws = {'opencv': ['lane_ws'], 'deep': ['lane_fusion_ws', 'lane_ws']}.get(mode, ['lane_fusion_ws'])
        raise RuntimeError(f'lane_mode:={mode}: lane package not found, source '
                           + ' and '.join(f'~/2026-AutoRace/{w}/install/setup.bash' for w in ws)) from error
    node_count = 6 + len(LANE_NODES[mode]) + 1  # parking, slow, crosswalk, signal, cone, YOLO, lane, mux

    def include(package, filename, arguments):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(share(package) + '/launch/' + filename),
            launch_arguments=arguments.items(),
        )

    needed = {'lane_model': C('lane_model').perform(context), 'yolo_model': C('yolo_model').perform(context),
              'parking_params': C('parking_params').perform(context),
              'cone_params': C('cone_params').perform(context)}
    if mode == 'opencv':
        del needed['lane_model']
    for path in needed.values():
        if not Path(path).is_file():
            raise RuntimeError('Missing file: ' + path)

    runner = Path(share('autorace_sensor_bringup')) / 'scripts/performance_runner.py'
    process_prefix = shlex.join(['/usr/bin/python3', str(runner), '--role', 'vision', '--'])

    def heavy(action, delay):
        group = GroupAction(actions=[SetLaunchConfiguration('launch-prefix', process_prefix), action])
        return TimerAction(period=delay, actions=[group]) if delay else group

    # 장애물 회피 V3(objavoid_ws, C++): 비활성 동안 명령 없음. 흰 정지선+점선+장애물을 보면 스스로 켜져
    # /mission/objavoid/command_05(MUX에서 lane보다 우선)로 주행, 갈림길 진입 후 DONE이면 명령을 멈춰 lane으로 넘어감.
    objavoid = []
    if C('objavoid').perform(context) == 'true':
        try:
            objavoid_launch = share('autorace_objavoid_v3') + '/launch/objavoid_v3.launch.py'
        except PackageNotFoundError as error:
            raise RuntimeError('objavoid:=true: autorace_objavoid_v3 not found, '
                               'source ~/2026-AutoRace/objavoid_ws/install/setup.bash') from error
        objavoid = [TimerAction(period=4., actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(objavoid_launch),
            launch_arguments={'profile': 'real', 'car': C('objavoid_car'),
                              'command_topic': '/mission/objavoid/command_05'}.items(),
        )])]

    mux = include('autorace_mux', 'mux.launch.py', {'output_topic': C('output_topic'), 'parking_guard_enabled':'true'})
    timeout = ParameterValue(C('startup_timeout'), value_type=float)
    ready = Node(
        package='tensor_bringup', executable='ready.py',
        parameters=[{'timeout': timeout, 'phase': 'inputs', 'lane_mode': mode}], output='screen',
    )
    report = Node(
        package='tensor_bringup', executable='ready.py', name='tensor_startup_report',
        parameters=[{'timeout': timeout, 'phase': 'nodes', 'lane_mode': mode}], output='screen',
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
            return [LogInfo(msg=f'[bringup 결과] 전체 노드 {node_count}/{node_count} 실행 확인 완료 '
                                f'(차선 {mode}, mux 포함, 입력 토픽 정상).')]
        return [LogInfo(msg='[bringup 결과] 전체 실행 확인 실패: 위의 누락 항목을 확인하세요.')]

    return [
        LogInfo(msg=f'Lane mode: {mode} ({", ".join(LANE_NODES[mode])})'),
        LogInfo(msg='Mission startup stagger: parking 0s, slow/crosswalk/signal/cone 2s, lane 4s, objavoid V3 4s, YOLO 8s'),
        RegisterEventHandler(OnProcessExit(target_action=ready, on_exit=inputs_ready)),
        RegisterEventHandler(OnProcessExit(target_action=report, on_exit=reported)),
        heavy(include('autorace_parking', 'parking.launch.py', {
            'use_sim_time': 'false', 'rviz': 'false', 'params_file': C('parking_params'),
        }), 0.),
        heavy(include('autorace_slow', 'slow.launch.py', {'use_sim_time': 'false'}), 2.),
        heavy(include('autorace_crosswalk', 'crosswalk.launch.py', {'use_sim_time': 'false'}), 2.),
        # Traffic light: red /yolo/signal (x2) -> stop on /mission/signal/command_02 (mux priority 2).
        heavy(include('autorace_signal', 'signal.launch.py', {'use_sim_time': 'false'}), 2.),
        heavy(include('cone_lane_follower', 'roi.launch.py', {
            'use_sim_time': 'false', 'rviz': 'false', 'params_file': C('cone_params'),
        }), 2.),
        heavy(GroupAction(actions=lane), 4.),
        *objavoid,
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
        'cone_params': share('cone_lane_follower') + '/config/clustering.yaml',
        'startup_timeout': '60.0', 'output_topic': '/cmd_vel',
        'lane_speed': '0.5',
        'lane_mode': 'fusion',
        'objavoid': 'true', 'objavoid_car': '04',
    }


def generate_launch_description():
    return LaunchDescription([
        *[DeclareLaunchArgument(key, default_value=value,
                                **({'choices': list(LANE_NODES),
                                    'description': 'fusion (deep+HSV) | opencv (lane_ws lane_lite + lane PID) | deep (deep mask + lane_ws logic + lane PID)'}
                                   if key == 'lane_mode' else {}))
          for key, value in _defaults().items()],
        OpaqueFunction(function=_setup),
    ])
