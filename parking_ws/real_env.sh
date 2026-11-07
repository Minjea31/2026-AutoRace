#!/usr/bin/env bash
# Source from a real vehicle terminal. Preserve its DDS domain, including zero.
_parking_real_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
_parking_real_domain="${ROS_DOMAIN_ID:-0}"
_parking_real_local="${ROS_LOCALHOST_ONLY:-0}"
_parking_real_log="${ROS_LOG_DIR:-$_parking_real_root/log/ros-real}"
source "$_parking_real_root/env.sh" || { return 1 2>/dev/null || exit 1; }
for _parking_real_ws in sensor_ws mux_ws; do
  if [[ -f "$_parking_real_root/../$_parking_real_ws/install/local_setup.bash" ]]; then
    source "$_parking_real_root/../$_parking_real_ws/install/local_setup.bash" || { return 1 2>/dev/null || exit 1; }
  fi
done
if [[ -f "$_parking_real_root/install/local_setup.bash" ]]; then
  source "$_parking_real_root/install/local_setup.bash" || { return 1 2>/dev/null || exit 1; }
fi
export ROS_DOMAIN_ID="$_parking_real_domain"
export ROS_LOCALHOST_ONLY="$_parking_real_local"
export ROS_LOG_DIR="$_parking_real_log"
unset _parking_real_root _parking_real_domain _parking_real_local _parking_real_log _parking_real_ws
