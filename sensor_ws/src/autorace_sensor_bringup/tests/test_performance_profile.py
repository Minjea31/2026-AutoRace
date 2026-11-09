import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys

import pytest

RUNNER = Path(__file__).resolve().parents[1] / 'scripts/performance_runner.py'
spec = importlib.util.spec_from_file_location('performance_runner', RUNNER)
runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)


def test_partitions_only_available_cores_and_handles_small_hosts():
    assert runner.cpu_partition({0, 1, 2, 3, 4, 5}, 'localization') == [5]
    assert runner.cpu_partition({0, 1, 2, 3, 4, 5}, 'vision') == [0, 1, 2, 3, 4]
    assert runner.cpu_partition({2, 4, 6, 8}, 'localization') == [8]
    assert runner.cpu_partition({2, 4, 6, 8}, 'vision') == [2, 4, 6]
    assert runner.cpu_partition({3}, 'vision') == [3]
    assert runner.cpu_partition({0, 1, 2}, 'localization') == [0, 1, 2]
    with pytest.raises(ValueError): runner.cpu_partition({0}, 'invalid')


@pytest.mark.parametrize('role', ['localization', 'vision'])
def test_exec_applies_affinity_priority_and_thread_limits_only_to_child(role):
    before = set(os.sched_getaffinity(0))
    nice = os.nice(0)
    code = "import os,json; print(json.dumps({'cpus':sorted(os.sched_getaffinity(0)), 'nice':os.nice(0), 'threads':[os.environ[k] for k in ('OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS','NUMEXPR_NUM_THREADS')]}))"
    result = subprocess.run([sys.executable, str(RUNNER), '--role', role, '--', sys.executable, '-c', code],
                            text=True, capture_output=True, check=True)
    child = json.loads(result.stdout.splitlines()[-1])
    assert child['cpus'] == runner.cpu_partition(before, role)
    assert child['nice'] == (max(5, nice) if role == 'vision' else nice)
    assert child['threads'] == ['1']*4
    assert set(os.sched_getaffinity(0)) == before and os.nice(0) == nice
