#!/usr/bin/env python3
"""params.py 기본값으로 config/objavoid_v3.yaml 재생성(주석 보존). 파라미터를 바꾸면 실행."""
import os
import re
import sys

PKG = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'src/autorace_objavoid_v3')
sys.path.insert(0, PKG)
from autorace_objavoid_v3.params import param_items  # noqa: E402

HEAD = [
    '# objavoid_v3 파라미터. 기본값은 autorace_objavoid_v3/params.py와 같다(tools/gen_config.py로 생성, test_config.py가 일치 검사).',
    '# 튜닝: 이 파일을 고치거나 launch 인자로 덮어쓴다. 단위: m, rad, s.',
    'objavoid_v3_node:', '  ros__parameters:',
    '    command_topic: /objavoid_v3/cmd   # 기본은 V3 전용(기존 MUX와 분리). 시뮬 어댑터는 이 토픽을 직접 연결',
    '    scan_topic: /scan', '    odom_topic: /odom',
    '    auto_activate: false   # 수동 활성화: ros2 topic pub --once /objavoid_v3/activate std_msgs/msg/Bool "{data: true}"',
    '    marker_rate: 5.0']


def fmt(v):
    if isinstance(v, bool):
        return str(v).lower()
    if isinstance(v, list):
        return '[' + ', '.join(repr(float(x)) for x in v) + ']'
    return repr(v)


def main():
    src = open(os.path.join(PKG, 'autorace_objavoid_v3/params.py'), encoding='utf-8').read()
    vals = dict(param_items())
    lines = list(HEAD)
    for ln in src.splitlines():
        m = re.match(r'\s+# --- (.*) ---', ln)
        if m:
            lines.append(f'    # --- {m.group(1)} ---')
            continue
        m = re.match(r'\s+(\w+): [\w\[\]]+ = .*?(#.*)?$', ln)
        if m and m.group(1) in vals:
            c = ('   ' + m.group(2)) if m.group(2) else ''
            lines.append(f'    {m.group(1)}: {fmt(vals[m.group(1)])}{c}')
    out = os.path.join(PKG, 'config/objavoid_v3.yaml')
    open(out, 'w', encoding='utf-8').write('\n'.join(lines) + '\n')
    print(out)


if __name__ == '__main__':
    main()
