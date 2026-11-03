#!/usr/bin/env bash
set -eo pipefail
_parking_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$_parking_root/env.sh"
export PYTHONPATH="$_parking_root/src/autorace_parking${PYTHONPATH:+:$PYTHONPATH}"
exec /usr/bin/python3 -m autorace_parking.node --ros-args \
  --params-file "$_parking_root/src/autorace_parking/config/parking.yaml" \
  -p use_sim_time:=true "$@"
