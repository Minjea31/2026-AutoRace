#!/usr/bin/env bash
# Apply in each terminal with: source env.sh
_slow_env_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$_slow_env_root/../simul/ros_env.sh" || { return 1 2>/dev/null || exit 1; }
if [[ -f "$_slow_env_root/install/local_setup.bash" ]]; then
  source "$_slow_env_root/install/local_setup.bash" || { return 1 2>/dev/null || exit 1; }
fi
unset _slow_env_root
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo '현재 터미널에 적용하려면 source env.sh를 사용하세요.'
fi
