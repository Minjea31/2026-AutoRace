#!/usr/bin/env bash
# Start at the track start line, with the whole map and obstacle editor open.
set -eo pipefail
_autorace_entry_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
_autorace_default_args=(--mission track --view overview --editor)

# Reuse a prepared simulator environment, or select the installed named one.
if [[ -n "${CONDA_PREFIX:-}" ]] && "$CONDA_PREFIX/bin/python" -c \
    'import sys; assert sys.version_info[:2] == (3, 10); import panda3d, numpy, yaml, PIL' 2>/dev/null; then
    exec "$_autorace_entry_dir/simul/run.sh" "${_autorace_default_args[@]}" "$@"
fi
_autorace_conda_exe="${CONDA_EXE:-$(command -v conda || true)}"
if [[ -n "$_autorace_conda_exe" ]]; then
    exec "$_autorace_conda_exe" run --no-capture-output -n autorace-simul \
        "$_autorace_entry_dir/simul/run.sh" "${_autorace_default_args[@]}" "$@"
fi
echo 'Conda의 autorace-simul 환경을 활성화한 후 다시 실행하세요.' >&2
exit 1
