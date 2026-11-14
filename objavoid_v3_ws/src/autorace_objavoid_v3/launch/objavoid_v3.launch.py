"""objavoid_v3 노드 실행.

  ros2 launch autorace_objavoid_v3 objavoid_v3.launch.py                       # 시뮬(profile:=sim): sim time, 시뮬 BEV
  ros2 launch autorace_objavoid_v3 objavoid_v3.launch.py profile:=real rviz:=true  # 실차: 시스템 시간, 실차 BEV(lane_fusion real/lane.yaml)
  ros2 launch autorace_objavoid_v3 objavoid_v3.launch.py profile:=real car:=04     # 04번: config/real_car04.yaml(바닥 투영·IMU 방향·v_max 0.2)
  ... observe:=true   # 관찰 모드: 수동 조종하며 경로만 확인(DONE·이탈 정지 없이 계속 계획)
  ... impl:=py        # Python 노드(기본은 C++ autorace_objavoid_v3_cpp, 같은 토픽·파라미터·설정 파일)
  ... command_topic:=/mission/objavoid/command_05   # MUX 연결(기존 objavoid_node를 끈 뒤). 기본은 V3 전용 /objavoid_v3/cmd
"""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node


def _nodes(context):
    get = lambda k: context.launch_configurations[k]
    share = get_package_share_directory('autorace_objavoid_v3')
    real = get('profile') == 'real'
    sim_time = get('use_sim_time') if get('use_sim_time') else ('false' if real else 'true')
    lane_cfg = get('lane_config')
    if not lane_cfg:
        # 저장소 루트 = 이 패키지 소스(심볼릭 설치)의 상위. 퓨전 차선 BEV 보정을 그대로 쓴다.
        src = Path(os.path.realpath(os.path.join(share, 'package.xml'))).parent
        repo = src.parents[2]
        lane_cfg = str(repo / 'lane_fusion_ws/src/autorace_lane_fusion/config' / ('real' if real else 'sim') / 'lane.yaml')
    params = [get('params_file')]
    car = get('car')
    if car:
        params.append(os.path.join(share, 'config', f'real_car{car}.yaml'))
    params += [{
        'use_sim_time': sim_time == 'true', 'command_topic': get('command_topic'),
        'scan_topic': get('scan_topic'), 'odom_topic': get('odom_topic'),
        'auto_activate': get('auto_activate') == 'true',
        'lane_config': lane_cfg, 'use_lane': get('use_lane') == 'true',
        'observe_mode': get('observe') == 'true'}]
    if get('v_max'):
        params.append({'v_max': float(get('v_max'))})
    pkg = 'autorace_objavoid_v3_cpp' if get('impl') == 'cpp' else 'autorace_objavoid_v3'
    out = [Node(package=pkg, executable='objavoid_v3_node', name='objavoid_v3_node',
                output='screen', parameters=params)]
    if get('rviz') == 'true':
        rv = 'objavoid_v3_base.rviz' if car else 'objavoid_v3.rviz'
        out.append(Node(package='rviz2', executable='rviz2', name='objavoid_v3_rviz', output='log',
                        arguments=['-d', os.path.join(share, 'rviz', rv)],
                        parameters=[{'use_sim_time': sim_time == 'true'}]))
    return out


def generate_launch_description():
    share = get_package_share_directory('autorace_objavoid_v3')
    args = [('profile', 'sim'), ('use_sim_time', ''), ('command_topic', '/objavoid_v3/cmd'), ('scan_topic', '/scan'),
            ('odom_topic', '/odom'), ('auto_activate', 'false'), ('v_max', ''), ('use_lane', 'true'), ('car', ''), ('observe', 'false'),
            ('lane_config', ''), ('params_file', os.path.join(share, 'config', 'objavoid_v3.yaml')),
            ('rviz', 'false'), ('impl', 'cpp')]
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k, v in args] +
                             [OpaqueFunction(function=_nodes)])
