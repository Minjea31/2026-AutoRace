#!/usr/bin/env bash
set -eo pipefail
_parking_rviz_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$_parking_rviz_root/env.sh"
exec "/opt/ros/$ROS_DISTRO/bin/rviz2" \
  -d "$_parking_rviz_root/src/autorace_parking/config/parking.rviz" \
  --ros-args -p use_sim_time:=true "$@"
