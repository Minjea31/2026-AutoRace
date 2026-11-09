#!/usr/bin/env bash
# Apply with: source crosswalk_ws/env.sh
_crosswalk_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash" || { return 1 2>/dev/null || exit 1; }
if [[ -f "$_crosswalk_root/install/local_setup.bash" ]]; then
  source "$_crosswalk_root/install/local_setup.bash"
fi
unset _crosswalk_root
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo '현재 터미널에 적용하려면 source crosswalk_ws/env.sh를 사용하세요.'
fi
