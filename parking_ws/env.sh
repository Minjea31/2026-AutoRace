#!/usr/bin/env bash
# Source ROS, optional local dependencies, and the built parking workspace.
_parking_env_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$_parking_env_root/../simul/ros_env.sh" || { return 1 2>/dev/null || exit 1; }
_parking_dep_prefix="$_parking_env_root/.ros-deps/opt/ros/$ROS_DISTRO"
if [[ -d "$_parking_dep_prefix/share/ackermann_msgs" ]]; then
  export AMENT_PREFIX_PATH="$_parking_dep_prefix${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
  export PYTHONPATH="$_parking_dep_prefix/local/lib/python3.10/dist-packages${PYTHONPATH:+:$PYTHONPATH}"
  export LD_LIBRARY_PATH="$_parking_dep_prefix/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
fi
# Shared lane HSV / white ROI library used by the parking start gate.
for _parking_support_ws in lane_fusion_ws slow_ws; do
  if [[ -f "$_parking_env_root/../$_parking_support_ws/install/local_setup.bash" ]]; then
    source "$_parking_env_root/../$_parking_support_ws/install/local_setup.bash" || { return 1 2>/dev/null || exit 1; }
  fi
done
unset _parking_support_ws
if [[ -f "$_parking_env_root/install/local_setup.bash" ]]; then
  source "$_parking_env_root/install/local_setup.bash" || { return 1 2>/dev/null || exit 1; }
fi
unset _parking_env_root _parking_dep_prefix

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo '현재 터미널에 환경을 적용하려면 다음을 실행하세요:'
  printf '  source %q\n' "${BASH_SOURCE[0]}"
fi
