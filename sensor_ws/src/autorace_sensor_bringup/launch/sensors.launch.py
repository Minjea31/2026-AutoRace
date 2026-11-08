"""Start the existing LIMO sensor stack and persist process/topic events."""
import json
import re
from datetime import datetime, timezone
from pathlib import Path
import yaml

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


def serial_port_owners(paths):
    targets = {Path(p).resolve(): str(p) for p in paths if Path(p).exists()}
    owners = {}
    for process in Path('/proc').iterdir():
        if not process.name.isdigit():
            continue
        try:
            for descriptor in (process/'fd').iterdir():
                try:
                    target = descriptor.resolve()
                except OSError:
                    continue
                if target in targets:
                    owners.setdefault(targets[target], set()).add(int(process.name))
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            continue
    return {port: sorted(pids) for port, pids in owners.items()}


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

    vehicle = value('vehicle')
    offsets = {'04': 0.04, '06': 0.0}
    if vehicle not in offsets:
        raise ValueError('vehicle must be 04 or 06')
    steering_offset = offsets[vehicle]

    ports = []
    if enabled('enable_lidar'):
        config = yaml.safe_load(Path(value('lidar_params')).read_text())
        ports.append(config['ydlidar_ros2_driver_node']['ros__parameters']['port'])
    if enabled('enable_base'):
        port = value('port_name')
        ports.append(port if port.startswith('/') else '/dev/'+port)
    busy = serial_port_owners(ports)
    if busy:
        raise RuntimeError(f'Sensor serial ports already open: {busy}; stop the previous sensor launch first')

    log_dir = Path(launch_config.log_dir)
    log_dir.mkdir(parents=True, exist_ok=True)
    raw_dir = log_dir / 'process_output'
    raw_dir.mkdir(exist_ok=True)
    options = {key: value(key) for key in (
        'vehicle', 'enable_base', 'enable_lidar', 'enable_camera', 'enable_description',
        'enable_camera_tf', 'enable_ekf', 'port_name', 'lidar_params', 'ekf_params', 'fusion_params',
        'camera_name', 'camera_launch', 'degree', 'startup_timeout', 'stale_timeout', 'tf_profile',
        'camera_tf_publish_rate', 'enable_depth', 'enable_ir',
        'enable_point_cloud', 'enable_colored_point_cloud',
    )}
    (log_dir / 'session.json').write_text(json.dumps({
        'started_at': datetime.now(timezone.utc).isoformat(), 'options': options,
        'base_executable': 'limo_base', 'use_mcnamu': False,
        'base_pub_odom_tf': True,
        'steering_command_offset': steering_offset,
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
        LogInfo(msg=f'Vehicle {vehicle}: Ackermann steering offset {steering_offset:+.3f} rad (wire units)'),
    ]
    topics, types = [], []
    expected_nodes = ['/sensor_monitor']

    def watch(topic, msg_type):
        topics.append(topic)
        types.append(msg_type)

    if enabled('enable_base'):
        expected_nodes.append('/limo/limo_base')
        actions.append(Node(
            package='limo_base', executable='limo_base', namespace='limo',
            output='both', parameters=[{
                'port_name': value('port_name'), 'use_mcnamu': False,
                'odom_frame': 'odom', 'base_frame': 'base_link',
                'pub_odom_tf': True, 'control_rate': 50,
                'steering_limit_enabled': enabled('steering_limit_enabled'),
                'max_inner_angle': float(value('max_inner_angle')),
                'steering_command_calibration': enabled('steering_command_calibration'),
                'steering_command_gain': float(value('steering_command_gain')),
                'steering_command_deadband': float(value('steering_command_deadband')),
                'steering_command_limit': float(value('steering_command_limit')),
                'steering_command_offset': steering_offset,
            }],
        ))
        watch('/imu', 'sensor_msgs/msg/Imu')
        watch('/odom', 'nav_msgs/msg/Odometry')
        watch('/limo_status', 'limo_msgs/msg/LimoStatus')
    if enabled('enable_lidar'):
        expected_nodes.append('/ydlidar_ros2_driver_node')
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
            'enable_color': 'true',
            'enable_depth': value('enable_depth'),
            'enable_ir': value('enable_ir'),
            'enable_point_cloud': 'true' if enabled('enable_depth') and enabled('enable_point_cloud') else 'false',
            'enable_colored_point_cloud': 'true' if enabled('enable_depth') and enabled('enable_colored_point_cloud') else 'false',
            'depth_registration': 'false', 'enable_d2c_viewer': 'false',
            # Rigid camera internal geometry belongs on /tf_static.
            'tf_publish_rate': value('camera_tf_publish_rate'),
        }))
        camera = '/' + value('camera_name').strip('/')
        watch(camera + '/color/image_raw', 'sensor_msgs/msg/Image')
        expected_nodes.extend([camera + '/camera_container', camera + '/' + value('camera_name').strip('/')])
        if enabled('enable_depth'):
            watch(camera + '/depth/image_raw', 'sensor_msgs/msg/Image')
        if enabled('enable_ir'):
            watch(camera + '/ir/image_raw', 'sensor_msgs/msg/Image')
    if enabled('enable_description'):
        expected_nodes.extend(['/robot_state_publisher', '/joint_state_publisher'])
        actions.append(include('limo_description', 'load_urdf_ackerman.launch.py', {
            'tf_profile': value('tf_profile'),
        }))
    if enabled('enable_camera_tf'):
        expected_nodes.extend(['/base_link_to_camera_mount', '/camera_mount_to_rotate', '/rotate_to_camera_link'])
        actions.append(include('autorace_sensor_bringup', 'camera_tf.launch.py', {
            'degree': value('degree'), 'camera_name': value('camera_name'),
            'tf_profile': value('tf_profile'),
        }))
    if enabled('enable_ekf'):
        expected_nodes.extend(['/sensor_fusion', '/ekf_filter_node_odom'])
        actions.append(Node(
            package='autorace_sensor_bringup', executable='sensor_fusion.py',
            name='sensor_fusion', output='both',
            additional_env={'OPENBLAS_NUM_THREADS': '1', 'OMP_NUM_THREADS': '1'},
            parameters=[value('fusion_params')],
        ))
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
                'expected_nodes': expected_nodes,
                'startup_timeout': float(value('startup_timeout')),
                'stale_timeout': float(value('stale_timeout')),
            }],
        ))
    return actions


def generate_launch_description():
    share = Path(get_package_share_directory('autorace_sensor_bringup'))
    defaults = {
        'vehicle': '06',
        'enable_base': 'true', 'enable_lidar': 'true', 'enable_camera': 'true',
        'enable_description': 'true', 'enable_camera_tf': 'true', 'enable_ekf': 'true',
        'port_name': 'ttylimo',
        'lidar_params': str(share / 'config' / 'ydlidar.yaml'),
        'ekf_params': str(share / 'config' / 'limo_ekf.yaml'),
        'fusion_params': str(share / 'config' / 'fusion.yaml'),
        'camera_name': 'camera', 'camera_launch': 'astra_stereo_u3.launch.py',
        'degree': '0.0', 'startup_timeout': '15.0', 'stale_timeout': '5.0',
        'tf_profile': 'wego_limo_pro',
        'camera_tf_publish_rate': '0.0',
        # The driving stack consumes RGB only; do not start unused streams/processors.
        'enable_depth': 'false', 'enable_ir': 'false',
        'enable_point_cloud': 'false', 'enable_colored_point_cloud': 'false',
        'steering_limit_enabled': 'false', 'max_inner_angle': '0.80',
        'steering_command_calibration': 'false', 'steering_command_gain': '1.92',
        'steering_command_deadband': '0.23', 'steering_command_limit': '0.35',
    }
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=default) for name, default in defaults.items()],
        OpaqueFunction(function=_setup),
    ])
