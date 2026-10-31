#!/usr/bin/env bash
set -eo pipefail
_autorace_run_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$_autorace_run_dir/env.sh"
exec "$CONDA_PREFIX/bin/python" -m limo_parking_sim.app "$@"
