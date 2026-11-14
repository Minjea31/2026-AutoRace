#!/usr/bin/env bash
# 시뮬레이터(장애물 구간 시작 8.55, 0.94, pi). 시나리오: ./scripts/sim.sh --map scenarios/default7.yaml
# 원본 맵·원본 장애물 파일은 수정하지 않는다(시나리오는 별도 파일).
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
export PYTHONNOUSERSITE=1
source "$ROOT/simul/env.sh" >/dev/null || exit 1
exec "$ROOT/simul/run.sh" --mission obstacle --view follow "$@"
