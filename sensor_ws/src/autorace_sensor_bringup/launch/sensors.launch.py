"""Start the existing LIMO sensor stack and persist process/topic events."""
import json
import re
from datetime import datetime, timezone
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction,
    RegisterEventHandler, SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit, OnProcessIO, OnProcessStart, OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.logging import launch_config
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LifecycleNode


def _setup(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    def enabled(name):
        text = value(name).lower()
        if text not in ('true', 'false'):
            raise ValueError(f'{name} must be true or false')
        return text == 'true'

    if value('tf_profile') not in ('existing', 'wego_limo_pro'):
        raise ValueError('tf_profile must be existing or wego_limo_pro')

    log_dir = Path(launch_config.log_dir)
    log_dir.mkdir(parents=True, exist_ok=True)
    raw_dir = log_dir / 'process_output'
    raw_dir.mkdir(exist_ok=True)
    options = {key: value(key) for key in (
        'enable_base', 'enable_lidar', 'enable_camera', 'enable_description',
        'enable_camera_tf', 'enable_ekf', 'port_name', 'lidar_params', 'ekf_params',
        'camera_name', 'camera_launch', 'degree', 'startup_timeout', 'stale_timeout', 'tf_profile',
        'camera_tf_publish_rate', 'viz', 'rviz_config',
    )}
    (log_dir / 'session.json').write_text(json.dumps({
        'started_at': datetime.now(timezone.utc).isoformat(), 'options': options,
        'base_executable': 'limo_base', 'use_mcnamu': False,
        'base_pub_odom_tf': not enabled('enable_ekf'),
    }, indent=2) + '\n')

    def record(kind, event=None):
        data = {'time': datetime.now(timezone.utc).isoformat(), 'event': kind}
        if kind == 'PROCESS_START':
            name = re.sub(r'[^a-zA-Z0-9_.-]', '_', event.process_name)
            for channel in ('stdout', 'stderr'):
                (raw_dir / f'{name}.{channel}.log').touch()
        if event is not None:
            data.update(process=event.process_name, pid=event.pid, cmd=event.cmd)
            if kind == 'PROCESS_EXIT':
                data['returncode'] = event.returncode
        with (log_dir / 'process_events.jsonl').open('a') as stream:
            stream.write(json.dumps(data) + '\n')
        return [LogInfo(msg=json.dumps(data))]

    def capture(event, channel):
        name = re.sub(r'[^a-zA-Z0-9_.-]', '_', event.process_name)
        with (raw_dir / f'{name}.{channel}.log').open('ab') as stream:
            stream.write(event.text)
        return []

    def include(package, filename, arguments=None):
        path = Path(get_package_share_directory(package)) / 'launch' / filename
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(path)),
            launch_arguments=(arguments or {}).items(),
        )

    # Register before launching children, including the camera component container.
    actions = [
        SetEnvironmentVariable('ROS_LOG_DIR', str(log_dir)),
        RegisterEventHandler(OnProcessStart(on_start=lambda e, c: record('PROCESS_START', e))),
        RegisterEventHandler(OnProcessExit(on_exit=lambda e, c: record('PROCESS_EXIT', e))),
        RegisterEventHandler(OnProcessIO(
            on_stdout=lambda e: capture(e, 'stdout'),
            on_stderr=lambda e: capture(e, 'stderr'),
        )),
        RegisterEventHandler(OnShutdown(on_shutdown=lambda e, c: record('LAUNCH_SHUTDOWN'))),
        LogInfo(msg=f'Sensor session logs: {log_dir}'),
    ]
    topics, types = [], []

    def watch(topic, msg_type):
        topics.append(topic)
        types.append(msg_type)

    if enabled('enable_base'):
        actions.append(Node(
            package='limo_base', executable='limo_base', namespace='limo',
            output='both', parameters=[{
                'port_name': value('port_name'), 'use_mcnamu': False,
                'odom_frame': 'odom', 'base_frame': 'base_link',
                'pub_odom_tf': not enabled('enable_ekf'), 'control_rate': 50,
            }],
        ))
        watch('/imu', 'sensor_msgs/msg/Imu')
        watch('/odom', 'nav_msgs/msg/Odometry')
        watch('/limo_status', 'limo_msgs/msg/LimoStatus')
    if enabled('enable_lidar'):
        actions.append(LifecycleNode(
            package='ydlidar_ros2_driver', executable='ydlidar_ros2_driver_node',
            name='ydlidar_ros2_driver_node', namespace='/',
            output='both', emulate_tty=True,
            parameters=[value('lidar_params')],
        ))
        watch('/scan', 'sensor_msgs/msg/LaserScan')
    if enabled('enable_camera'):
        actions.append(include('orbbec_camera', value('camera_launch'), {
            'camera_name': value('camera_name'),
            # Rigid camera internal geometry belongs on /tf_static.
            'tf_publish_rate': value('camera_tf_publish_rate'),
        }))
        camera = '/' + value('camera_name').strip('/')
        watch(camera + '/color/image_raw', 'sensor_msgs/msg/Image')
        watch(camera + '/depth/image_raw', 'sensor_msgs/msg/Image')
    if enabled('enable_description'):
        actions.append(include('limo_description', 'load_urdf_ackerman.launch.py', {
            'tf_profile': value('tf_profile'),
        }))
    if enabled('enable_camera_tf'):
        actions.append(include('autorace_sensor_bringup', 'camera_tf.launch.py', {
            'degree': value('degree'), 'camera_name': value('camera_name'),
            'tf_profile': value('tf_profile'),
        }))
    if enabled('enable_ekf'):
        actions.append(Node(
            package='robot_localization', executable='ekf_node',
            name='ekf_filter_node_odom', output='both', parameters=[value('ekf_params')],
        ))
        watch('/odometry/filtered', 'nav_msgs/msg/Odometry')
    if topics:
        actions.append(Node(
            package='autorace_sensor_bringup', executable='sensor_monitor.py',
            name='sensor_monitor', output='both', parameters=[{
                'topics': topics, 'types': types, 'log_dir': str(log_dir),
                'startup_timeout': float(value('startup_timeout')),
                'stale_timeout': float(value('stale_timeout')),
            }],
        ))
    if enabled('viz'):
        actions.append(Node(
            package='rviz2', executable='rviz2', name='rviz2',
            output='both', arguments=['-d', value('rviz_config')],
        ))
    return actions


def generate_launch_description():
    share = Path(get_package_share_directory('autorace_sensor_bringup'))
    defaults = {
        'enable_base': 'true', 'enable_lidar': 'true', 'enable_camera': 'true',
        'enable_description': 'true', 'enable_camera_tf': 'true', 'enable_ekf': 'true',
        'port_name': 'ttylimo',
        'lidar_params': str(share / 'config' / 'ydlidar.yaml'),
        'ekf_params': str(share / 'config' / 'limo_ekf.yaml'),
        'camera_name': 'camera', 'camera_launch': 'astra_stereo_u3.launch.py',
        'degree': '0.0', 'startup_timeout': '15.0', 'stale_timeout': '5.0',
        'tf_profile': 'wego_limo_pro',
        'camera_tf_publish_rate': '0.0',
        'viz': 'false',
        'rviz_config': str(share / 'config' / 'sensors.rviz'),
    }
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=default) for name, default in defaults.items()],
        OpaqueFunction(function=_setup),
    ])
