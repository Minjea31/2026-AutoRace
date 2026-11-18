"""lanedeep U-Net segmentation-only node (publishes /lane_fusion/deep_mask).

  ros2 launch autorace_lane_fusion_deep deep_mask.launch.py   # installed engine, else ONNX
  ros2 launch autorace_lane_fusion_deep deep_mask.launch.py model_path:=/abs/lane_640.engine
  max_rate:=20.0   masks per second at most (asynchronous; lane_fusion_node never waits)
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    defaults = {'model_path': '', 'backend': 'auto', 'device': 'auto', 'input_width': '640',
                'input_height': '480', 'confidence_threshold': '0.5', 'use_sim_time': 'false',
                'camera_topic': '/camera/color/image_raw/compressed', 'max_rate': '20.0', 'log_performance': 'false'}
    typed = {'input_width': int, 'input_height': int, 'confidence_threshold': float, 'use_sim_time': bool,
             'max_rate': float, 'log_performance': bool}
    params = {k: ParameterValue(LaunchConfiguration(k), value_type=typed.get(k, str)) for k in defaults}
    return LaunchDescription([
        *[DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()],
        Node(package='autorace_lane_fusion_deep', executable='lane_fusion_deep_mask',
             name='lane_fusion_deep_mask', output='screen', parameters=[params]),
    ])
