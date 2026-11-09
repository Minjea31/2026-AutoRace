#!/usr/bin/env bash
# Start the shared mission viewer. Uses the terminal's ROS domain and display.
set -e
autorace_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
for autorace_ws in sensor_ws mux_ws lane_fusion_ws parking_ws slow_ws yolo_ws tensor_bringup; do
    if [[ -f "$autorace_root/$autorace_ws/install/local_setup.bash" ]]; then
        source "$autorace_root/$autorace_ws/install/local_setup.bash"
    fi
done
exec ros2 launch tensor_bringup missions_rviz.launch.py "$@"
