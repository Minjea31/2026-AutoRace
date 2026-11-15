#!/usr/bin/env python3
"""시뮬레이터 원본 맵(simul/.../parking_map.yaml)을 읽어 장애물 배치만 바꾼 시나리오 맵을 만든다.

원본 맵·원본 장애물 파일은 수정하지 않는다. 시나리오 맵 옆에는 .obstacles.yaml을 두지 않으므로
(load_config가 그 파일로 덮어쓰기 때문) 아래 obstacles 목록이 그대로 사용된다.
정답 좌표는 평가(tools/evaluator.py)에서만 읽는다.
"""
import json
import math
import os

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
WS = os.path.dirname(HERE)
ROOT = os.path.dirname(WS)
SRC = os.path.join(ROOT, 'simul/src/limo_parking_sim/config/parking_map.yaml')
OUT = os.path.join(WS, 'scenarios')

X_MIN, X_MAX, COLS = 4.268, 8.030, 8
ROW = {'upper': 1.46, 'middle': 0.94, 'lower': 0.42}


def col_x(c):
    return X_MIN + (c - 0.5) * (X_MAX - X_MIN) / COLS


def grid(*cells):
    return [(col_x(c), ROW[r], math.pi) for r, c in cells]


SCENARIOS = {
    # 1. 단일 장애물
    'single_mid': ('가운데 차로 1대(정중앙)', 'done', [(6.6, 0.94, math.pi)]),
    'single_left': ('가운데 차로 1대, 오른쪽으로 6 cm 치우침 -> 왼쪽(아래 차로) 회피', 'done', [(6.6, 1.00, math.pi)]),
    'single_right': ('가운데 차로 1대, 왼쪽으로 6 cm 치우침 -> 오른쪽(위 차로) 회피', 'done', [(6.6, 0.88, math.pi)]),
    'single_oblique': ('가운데 차로 0.5 rad 비스듬한 1대', 'done', [(6.6, 0.94, math.pi + 0.5)]),
    # 2. 2~3대 연속
    'multi2': ('같은 가운데 차로 연속 2대', 'done', [(7.0, 0.94, math.pi), (6.2, 0.94, math.pi)]),
    'multi3': ('차로를 바꿔 가며 3대', 'done', [(7.0, 0.94, math.pi), (6.1, 0.42, math.pi), (5.2, 0.94, math.pi)]),
    # 3. 지그재그
    'zigzag4': ('3차로 지그재그 4대(아래8, 가운데6, 위4, 가운데2)', 'done',
                grid(('lower', 8), ('middle', 6), ('upper', 4), ('middle', 2))),
    'alternating': ('점선 위 + 반대편 번갈아 3대', 'done',
                    [(7.0, 0.72, math.pi), (5.7, 1.16, math.pi), (5.7, 0.42, math.pi)]),
    # 4. 실제 대회형 7대
    'default7': ('시뮬 기본 격자 7대(위3·8, 가운데4·5·7, 아래2·6): 통로 0.30 m, 차폭 0.22 m', 'done',
                 grid(('upper', 3), ('upper', 8), ('middle', 4), ('middle', 5), ('middle', 7),
                      ('lower', 2), ('lower', 6))),
    'local7': ('사용자 로컬 시뮬 배치 7대(위8, 가운데4·5·6, 아래3·6·7) = 원래 user7', 'done',
               grid(('upper', 8), ('middle', 4), ('middle', 5), ('middle', 6),
                    ('lower', 3), ('lower', 6), ('lower', 7))),
    'user7': ('사용자 로컬 편집 배치 7대(위8, 가운데4·5·6, 아래3·6·7). 편집기 저장본 user7.obstacles.yaml이 있으면 그것이 우선', 'done',
              grid(('upper', 8), ('middle', 4), ('middle', 5), ('middle', 6),
                   ('lower', 3), ('lower', 6), ('lower', 7))),
    'orig7': ('objavoid_test orig7과 동일 배치', 'done',
              grid(('lower', 5), ('lower', 6), ('lower', 7), ('upper', 7), ('upper', 8),
                   ('middle', 3), ('middle', 2))),
    'rand7_1': ('objavoid_test rand7_1과 동일', 'done',
                grid(('middle', 6), ('lower', 8), ('upper', 3), ('upper', 8), ('lower', 6), ('lower', 3), ('lower', 7))),
    'rand7_2': ('objavoid_test rand7_2와 동일', 'done',
                grid(('middle', 6), ('upper', 6), ('middle', 3), ('upper', 5), ('lower', 1), ('lower', 8), ('upper', 2))),
    'rand7_3': ('objavoid_test rand7_3과 동일', 'done',
                grid(('lower', 5), ('lower', 1), ('lower', 4), ('middle', 7), ('upper', 3), ('lower', 6), ('middle', 1))),
    # 7. 경로 추종 중 새 장애물(시험에서 2번째 차를 x<7.45까지 LiDAR에서 숨김)
    'newobs': ('가운데 1대 + 회피 경로 위 숨겨진 1대(고장 주입기로 늦게 공개)', 'done',
               [(7.0, 0.94, math.pi), (6.0, 0.60, math.pi)]),
    # 5. 좁은 통로·차단
    'narrow': ('폭 0.38 m 통로 하나만 열림(차폭 0.22 m)', 'done',
               [(6.6, 0.42, math.pi), (6.6, 1.02, math.pi), (6.6, 1.46, math.pi)]),
    'blocked': ('세 차로 모두 막힘(차 사이 틈 0.25 m, 경계 쪽 0.15/0.23 m < 차폭+여유) -> 정지', 'stop',
                [(6.6, 0.43, math.pi), (6.6, 0.90, math.pi), (6.6, 1.37, math.pi)]),
}


def main():
    with open(SRC, encoding='utf-8') as f:
        base = yaml.safe_load(f)
    os.makedirs(OUT, exist_ok=True)
    index = {}
    for name, (desc, expect, cars) in SCENARIOS.items():
        m = dict(base)
        m['avoidance_layout'] = dict(base['avoidance_layout'], occupied={})
        parked = [o for o in base['obstacles'] if not o['name'].startswith('avoid_')]
        m['obstacles'] = parked + [{'name': f'v3_{i + 1:02d}', 'pose': [round(x, 6), round(y, 6), round(yaw, 6)]}
                                   for i, (x, y, yaw) in enumerate(cars)]
        path = os.path.join(OUT, name + '.yaml')
        with open(path, 'w', encoding='utf-8') as f:
            f.write(f'# objavoid_v3 시나리오: {desc}\n# tools/make_scenarios.py로 생성. 원본: simul parking_map.yaml\n')
            yaml.safe_dump(m, f, allow_unicode=True, sort_keys=False)
        side = path[:-5] + '.obstacles.yaml'
        if os.path.exists(side):
            # 시뮬 편집기(E)로 저장한 배치는 지우지 않는다(이 맵을 열면 그 배치가 우선 적용됨)
            print(f'  keep editor layout: {os.path.relpath(side, WS)} (overrides {name}.yaml)')
        index[name] = {'desc': desc, 'expect': expect, 'cars': len(cars)}
    with open(os.path.join(OUT, 'index.json'), 'w', encoding='utf-8') as f:
        json.dump(index, f, ensure_ascii=False, indent=1)
    print(f'{len(SCENARIOS)} scenarios -> {OUT}')


if __name__ == '__main__':
    main()
