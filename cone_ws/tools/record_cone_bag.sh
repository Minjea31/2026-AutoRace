#!/usr/bin/env bash
# Car 06: rosbag for the traffic-cone (라바콘) mission.
# LaserScan /scan + odometry/IMU + TF + cone node outputs + commands + /rosout.
# Usage: ./record_cone_bag.sh [output_dir]   (Ctrl+C to stop)
set -eo pipefail
source /opt/ros/humble/setup.bash  # ROS setup scripts use unset variables: no -u while sourcing
set -u
out="${1:-$HOME/bags/cone_$(date +%Y%m%d_%H%M%S)}"
mkdir -p "$(dirname "$out")"
exec ros2 bag record -o "$out" \
  /scan \
  /odom /odometry/filtered /imu /joint_states /limo_status \
  /tf /tf_static \
  /cone_lane/roi_marker /cone_lane/roi_points /cone_lane/clusters /cone_lane/cone_centers \
  /cone_lane/boundaries /cone_lane/center_path /cone_lane/control_status \
  /mission/cone/command_04 /mission/lane /cmd_vel \
  /rosout
