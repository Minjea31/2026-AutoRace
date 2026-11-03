#!/usr/bin/env bash
# Apply with: source signal_ws/env.sh
_signal_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash" || { return 1 2>/dev/null || exit 1; }
if [[ -f "$_signal_root/install/local_setup.bash" ]]; then
  source "$_signal_root/install/local_setup.bash"
fi
unset _signal_root
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo '현재 터미널에 적용하려면 source signal_ws/env.sh를 사용하세요.'
fi
