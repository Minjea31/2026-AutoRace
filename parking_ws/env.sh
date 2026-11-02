#!/usr/bin/env bash
# Source ROS plus an optional workspace-local extraction of ackermann_msgs.
_parking_env_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$_parking_env_root/../simul/ros_env.sh" || { return 1 2>/dev/null || exit 1; }
_parking_dep_prefix="$_parking_env_root/.ros-deps/opt/ros/$ROS_DISTRO"
if [[ -d "$_parking_dep_prefix/share/ackermann_msgs" ]]; then
  export AMENT_PREFIX_PATH="$_parking_dep_prefix${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
  export PYTHONPATH="$_parking_dep_prefix/local/lib/python3.10/dist-packages${PYTHONPATH:+:$PYTHONPATH}"
  export LD_LIBRARY_PATH="$_parking_dep_prefix/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
fi
unset _parking_env_root _parking_dep_prefix
