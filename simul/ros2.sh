#!/usr/bin/env bash
# Use from base or Conda; no simulator packages are needed for the ROS CLI.
set -eo pipefail
_autorace_cli_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$_autorace_cli_dir/ros_env.sh"
# Graph queries must use this process's DDS environment. A daemon started by
# another terminal can remain reachable over XML-RPC while seeing no sensors.
# Allow discovery to settle; explicit user --spin-time arguments take priority.
if [[ "${1:-}" == topic ]]; then
  case "${2:-}" in
    list|info|echo|type|find)
      exec "/opt/ros/$ROS_DISTRO/bin/ros2" "$1" "$2" \
        --no-daemon --spin-time 2 "${@:3}"
      ;;
  esac
fi
exec "/opt/ros/$ROS_DISTRO/bin/ros2" "$@"
