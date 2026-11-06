#!/usr/bin/env bash
# Lane keeping, FUSION version (U-Net mask + lane_fusion perception + PID).
#   ./fusion.sh sim  [launch args]   simulator: DDS domain of simul/, use_sim_time (deep off)
#   ./fusion.sh real [launch args]   real car: config/real, deep mask node on
# Publishes /mission/lane like lane_ws/lane.sh — run only ONE of the two.
# e.g. ./fusion.sh real model_path:=$PWD/model/lane_640.engine lane_gui:=true
set -eo pipefail
here="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
profile="${1:-}"
if [[ "$profile" != sim && "$profile" != real ]]; then
  echo "사용법: $0 sim|real [launch 인자...]" >&2
  exit 2
fi
shift
if [[ "$profile" == sim ]]; then
  source "$here/../simul/ros_env.sh"
else
  source /opt/ros/humble/setup.bash
fi
if [[ ! -f "$here/install/autorace_lane_fusion_pid/share/autorace_lane_fusion_pid/package.xml" ]]; then
  echo '먼저 lane_fusion_ws에서 colcon build 를 실행하세요 (README 참고).' >&2
  exit 1
fi
source "$here/install/setup.bash"
exec /opt/ros/humble/bin/ros2 launch autorace_lane_fusion_pid lane_fusion_pid.launch.py \
  profile:="$profile" enable_vehicle_output:=true "$@"
