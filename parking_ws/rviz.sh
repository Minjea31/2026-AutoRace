#!/usr/bin/env bash
set -eo pipefail
_parking_rviz_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$_parking_rviz_root/env.sh"
export PYTHONPATH="$_parking_rviz_root/src/autorace_parking${PYTHONPATH:+:$PYTHONPATH}"
/usr/bin/python3 -m autorace_parking.robot_visual &
_parking_model_pid=$!
trap 'kill "$_parking_model_pid" 2>/dev/null || true; wait "$_parking_model_pid" 2>/dev/null || true' EXIT
trap 'exit 130' INT TERM
"/opt/ros/$ROS_DISTRO/bin/rviz2" \
  -d "$_parking_rviz_root/src/autorace_parking/config/parking.rviz" \
  --ros-args -p use_sim_time:=true "$@"
