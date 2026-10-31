#!/usr/bin/env bash
# Use from base or Conda; no simulator packages are needed for the ROS CLI.
set -eo pipefail
_autorace_cli_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$_autorace_cli_dir/ros_env.sh"
exec "/opt/ros/$ROS_DISTRO/bin/ros2" "$@"
