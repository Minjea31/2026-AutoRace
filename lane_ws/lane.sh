#!/usr/bin/env bash
# Lane keeping (lane_lite perception + lane_pid control) for sim OR real car.
#   ./lane.sh sim  [launch args]   simulator: DDS domain of simul/, use_sim_time
#   ./lane.sh real [launch args]   real car:  config/real of both packages
# Publishes /mission/lane (enable_vehicle_output:=true); the mux or
# simul/adapter.sh forwards it to the vehicle. Extra args go to ros2 launch,
# e.g. lane_gui:=true (perception tuner) or gui:=false (no PID tuner).
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
if [[ ! -f "$here/install/autorace_lane_pid/share/autorace_lane_pid/package.xml" ]]; then
  echo '먼저 lane_ws에서 colcon build --packages-select autorace_lane_lite autorace_lane_pid 를 실행하세요.' >&2
  exit 1
fi
source "$here/install/setup.bash"
exec /opt/ros/humble/bin/ros2 launch autorace_lane_pid lane_pid.launch.py \
  profile:="$profile" enable_vehicle_output:=true "$@"
