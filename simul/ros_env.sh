#!/usr/bin/env bash
# Shared DDS settings for the Conda simulator and the system ROS CLI.
_autorace_ros_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
_autorace_ros_setup="/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
if [[ ! -f "$_autorace_ros_setup" ]]; then
  echo "ROS 2 환경 파일을 찾지 못했습니다: $_autorace_ros_setup" >&2
  return 1 2>/dev/null || exit 1
fi
source "$_autorace_ros_setup"
# A normal desktop terminal often inherits the real-robot default domain 0.
# Simulator tools must join domain 42 in that case, just as a clean terminal
# does. Preserve nonzero test domains; an explicit simulator override also
# supports intentionally using domain 0.
if [[ -n "${AUTORACE_SIM_DOMAIN_ID:-}" ]]; then
  export ROS_DOMAIN_ID="$AUTORACE_SIM_DOMAIN_ID"
elif [[ -z "${ROS_DOMAIN_ID:-}" || "$ROS_DOMAIN_ID" == 0 ]]; then
  export ROS_DOMAIN_ID=42
fi
# Keep this simulator local even if the parent shell exported 0.
export ROS_LOCALHOST_ONLY=1
export ROS_LOG_DIR="${ROS_LOG_DIR:-$_autorace_ros_root/log/ros}"
unset _autorace_ros_root _autorace_ros_setup
