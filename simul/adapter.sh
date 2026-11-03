#!/usr/bin/env bash
# Select one mission command and send it directly to the simulator.
set -eo pipefail
_autorace_adapter_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$_autorace_adapter_dir/env.sh"
exec "$CONDA_PREFIX/bin/python" -m limo_parking_sim.mission_adapter "$@"
