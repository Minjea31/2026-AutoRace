#!/usr/bin/env bash
# Source in each terminal. Keep the caller's DDS domain/network settings on the robot.
_lane_env_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
_lane_ros_setup="/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
if [[ ! -f "$_lane_ros_setup" ]]; then
  echo "ROS setup not found: $_lane_ros_setup" >&2
  return 1 2>/dev/null || exit 1
fi
source "$_lane_ros_setup" || { return 1 2>/dev/null || exit 1; }
if [[ -f "$_lane_env_root/install/local_setup.bash" ]]; then
  source "$_lane_env_root/install/local_setup.bash" || { return 1 2>/dev/null || exit 1; }
fi
unset _lane_env_root _lane_ros_setup
