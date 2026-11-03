"""Exercise bootstrap without downloads or changes to any real Conda env."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

import pytest


@pytest.fixture
def bootstrap(tmp_path):
    if not Path('/opt/ros/humble/setup.bash').exists():
        pytest.skip('System ROS prerequisite is absent')
    base = tmp_path / 'conda root'
    (base / 'bin').mkdir(parents=True)
    (base / 'etc/profile.d').mkdir(parents=True)
    (base / 'bin/python').symlink_to(sys.executable)
    prefix = base / 'envs/autorace-simul'
    (prefix / 'bin').mkdir(parents=True)
    python = prefix / 'bin/python'
    python.write_text('''#!/bin/bash
if [[ "$1" == - ]]; then
  cat >/dev/null
  [[ -f "$MOCK_BASE/ready" ]]
else
  exit 0
fi
''')
    python.chmod(0o755)
    conda = base / 'bin/conda'
    conda.write_text('''#!/bin/bash
case "$*" in
  'info --base') echo "$MOCK_BASE" ;;
  'env list --json')
    [[ "$MOCK_FAILURE" == list ]] && exit 1
    if [[ -f "$MOCK_BASE/exists" ]]; then cat "$MOCK_BASE/envs.json"; else echo '{"envs": []}'; fi ;;
  'env create '*|'env update '*)
    echo "$*" >> "$MOCK_BASE/actions"
    [[ "$MOCK_FAILURE" == install ]] && exit 1
    touch "$MOCK_BASE/exists" "$MOCK_BASE/ready" ;;
  *) exit 2 ;;
esac
''')
    conda.chmod(0o755)
    (base / 'envs.json').write_text(json.dumps({'envs': [str(prefix)]}))
    (base / 'etc/profile.d/conda.sh').write_text('''conda() {
  if [[ "$1" == activate ]]; then
    echo "activate $2" >> "$MOCK_BASE/actions"
    [[ "$MOCK_FAILURE" == activate ]] && return 1
    export CONDA_PREFIX="$MOCK_BASE/envs/autorace-simul"
    export CONDA_DEFAULT_ENV=autorace-simul
  else
    "$CONDA_EXE" "$@"
  fi
}
''')
    sim = tmp_path / 'simul'
    sim.mkdir()
    source = Path(__file__).resolve().parents[3]
    shutil.copy(source / 'env.sh', sim / 'env.sh')
    shutil.copy(source / 'environment.yml', sim / 'environment.yml')
    # DDS settings are covered separately by test_ros_environment.py.
    (sim / 'ros_env.sh').write_text('export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1\n')

    def run(exists=False, ready=False, failure='', direct=False):
        for filename, present in (('exists', exists), ('ready', ready)):
            if present:
                (base / filename).touch()
        environment = os.environ.copy()
        environment.update(CONDA_EXE=str(conda), MOCK_BASE=str(base), MOCK_FAILURE=failure,
                           ROS_DISTRO='humble', CONDA_DEFAULT_ENV='different-env')
        command = ('bash "$1"' if direct else
                   'source "$1" && source "$1" && '
                   '[[ "$CONDA_DEFAULT_ENV" == autorace-simul && "$ROS_DOMAIN_ID" == 42 ]]')
        result = subprocess.run(['/bin/bash', '--noprofile', '--norc', '-c',
                                 command, 'test', str(sim / 'env.sh')],
                                env=environment, text=True, capture_output=True)
        actions = (base / 'actions').read_text() if (base / 'actions').exists() else ''
        return result, actions

    return run


@pytest.mark.parametrize('exists,ready,action', [
    (False, False, 'env create'), (True, False, 'env update'), (True, True, None),
])
def test_create_repair_and_reuse(bootstrap, exists, ready, action):
    result, actions = bootstrap(exists, ready)
    assert result.returncode == 0, result.stderr
    assert actions.count('env create') == int(action == 'env create')
    assert actions.count('env update') == int(action == 'env update')
    assert actions.count('activate autorace-simul') >= 2


@pytest.mark.parametrize('failure,exists', [('install', False), ('activate', True), ('list', True)])
def test_failure_is_returned_without_recreating_existing_env(bootstrap, failure, exists):
    result, actions = bootstrap(exists, ready=True, failure=failure)
    assert result.returncode != 0
    assert '준비 완료' not in result.stdout
    if exists:
        assert 'env create' not in actions


def test_direct_execution_explains_parent_shell_limit(bootstrap):
    result, actions = bootstrap(exists=True, ready=True, direct=True)
    assert result.returncode == 0, result.stderr
    assert '현재 터미널에 적용하려면' in result.stdout
    assert 'source ' in result.stdout
