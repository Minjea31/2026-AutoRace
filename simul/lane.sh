#!/usr/bin/env bash
# Simulator lane keeping: lane_lite perception + error-based PID (autorace_lane_pid).
# Opens the PID tuner window; add lane_gui:=true for the separate lane tuner.
# --lane-camera uses the lane-camera (pitch 0.35) perception profile.
set -eo pipefail
_autorace_lane_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
_autorace_lane_args=()
if [[ "${1:-}" == --lane-camera ]]; then
  shift
  _autorace_lane_args+=("lane_config:=$_autorace_lane_root/lane_ws/install/autorace_lane_lite/share/autorace_lane_lite/config/sim/lane_camera.yaml")
fi
source "$_autorace_lane_root/simul/ros_env.sh"
if [[ ! -f "$_autorace_lane_root/lane_ws/install/autorace_lane_pid/share/autorace_lane_pid/package.xml" ]]; then
  echo '먼저 lane_ws에서 colcon build --packages-select autorace_lane_lite autorace_lane_pid 를 실행하세요.' >&2
  exit 1
fi
source "$_autorace_lane_root/lane_ws/install/setup.bash"
exec /opt/ros/humble/bin/ros2 launch autorace_lane_pid lane_pid.launch.py \
  use_sim_time:=true enable_vehicle_output:=true "${_autorace_lane_args[@]}" "$@"
