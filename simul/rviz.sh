#!/usr/bin/env bash
set -eo pipefail
_autorace_rviz_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$_autorace_rviz_dir/ros_env.sh"
exec "/opt/ros/$ROS_DISTRO/bin/rviz2" -d "$_autorace_rviz_dir/config/sensors.rviz" \
  --ros-args -p use_sim_time:=true "$@"
