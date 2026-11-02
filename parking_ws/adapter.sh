#!/usr/bin/env bash
# Parking-only /cmd_vel bridge for simulator development without the real mux.
set -eo pipefail
_parking_adapter_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$_parking_adapter_root/env.sh"
export PYTHONPATH="$_parking_adapter_root/../simul/src/limo_parking_sim${PYTHONPATH:+:$PYTHONPATH}"
exec /usr/bin/python3 -m limo_parking_sim.parking_adapter "$@"
