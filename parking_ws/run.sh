#!/usr/bin/env bash
set -eo pipefail
_parking_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$_parking_root/env.sh"
export PYTHONPATH="$_parking_root/src/autorace_parking${PYTHONPATH:+:$PYTHONPATH}"
if [[ "${1:-}" == '--with-adapter' ]]; then
  shift
  "$_parking_root/adapter.sh" &
  _parking_adapter_pid=$!
  trap 'kill "$_parking_adapter_pid" 2>/dev/null || true' EXIT
  trap 'exit 130' INT TERM
  /usr/bin/python3 -m autorace_parking.node --ros-args \
    --params-file "$_parking_root/src/autorace_parking/config/parking.yaml" \
    -p use_sim_time:=true "$@"
  exit $?
fi
exec /usr/bin/python3 -m autorace_parking.node --ros-args \
  --params-file "$_parking_root/src/autorace_parking/config/parking.yaml" \
  -p use_sim_time:=true "$@"
