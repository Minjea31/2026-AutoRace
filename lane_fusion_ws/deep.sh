#!/usr/bin/env bash
# Run the same deep perception + PID stack used by tensor_bringup.
set -eo pipefail
autorace_lane_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
profile="${1:-}"
if [[ "$profile" != sim && "$profile" != real ]]; then
  echo "사용법: $0 sim|real [launch 인자...]" >&2
  exit 2
fi
shift
if [[ "$profile" == sim ]]; then
  source "$autorace_lane_root/simul/ros_env.sh"
else
  source /opt/ros/humble/setup.bash
fi
for workspace in lane_ws lane_fusion_ws; do
  setup="$autorace_lane_root/$workspace/install/local_setup.bash"
  if [[ ! -f "$setup" ]]; then
    echo "먼저 $workspace를 빌드하세요: $setup 파일이 없습니다." >&2
    exit 1
  fi
  source "$setup"
done
exec /opt/ros/humble/bin/ros2 launch autorace_lane_deep_lite lane_deep_lite.launch.py \
  profile:="$profile" enable_vehicle_output:=true "$@"
