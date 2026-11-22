# 미사용 코드 정리와 삭제 전후 검증

현재 main의 deep 실차 경로를 기준으로 정리했습니다. Lane fusion의 알고리즘·노드·설정과 HSV 대체 경로는 보존했습니다. 실행 가능한 다른 설정을 단순히 현재 꺼져 있다는 이유로 삭제하지 않았습니다.

## 삭제한 코드

| 위치 | 삭제 내용 | 기존 동작이 유지되는 이유 |
| --- | --- | --- |
| `cone_ws/.../cpp/roi_node.cpp` | `if (live_errors)` 다음의 `else if (live_errors && ...)`, 이어지는 마지막 else | 앞의 if가 참을 처리한 뒤에는 같은 조건이 참일 수 없습니다. 남은 거짓 경로의 동작만 else로 남겼습니다. |
| `parking_ws/.../control.py` | 사용하지 않는 `cluster_points`, `straight_turn_candidates` import | 구현 함수는 원래 모듈에 유지하며 호출 경로는 바뀌지 않습니다. |
| `parking_ws/.../node.py` | 사용하지 않는 import 2개, 결과를 읽지 않는 `observe_pair` 계산 | 미션 상태·명령·분기에서 값을 사용하지 않습니다. |
| `parking_ws/.../targets.py` | 사용하지 않는 `hull`, `inside`, `rotation` import | 목표 생성에서 실제 쓰는 함수는 유지했습니다. |
| `objavoid_ws/.../planner.py` | 사용하지 않는 `dmax = max(dl_max, dr_max)` | 후보 생성은 기존 `dl_max`, `dr_max`를 그대로 사용합니다. C++ 실차 실행 코드도 유지합니다. |
| `cone_ws/backups/`, `*.before_speed_gains` | 실행/빌드에서 참조하지 않는 이전 소스 사본 | 현재 빌드 소스·설정·검증용 reference Python은 유지합니다. 이전 코드는 Git 이력에서 확인할 수 있습니다. |
| `gpu_monitor.py`, `gpu_monitor.sh`, `yolo_ws/scripts/render_yolo_review.py` | 이전에 삭제 요청한 개발 도구 | 미션 launch에서 실행하지 않는 도구이며 주행 후보와 센서 흐름에 연결되지 않습니다. |

토픽명·타입·PID 게인·속도·우선순위·미션 종료 조건은 이번 정리로 바꾸지 않았습니다.

## 비교 결과

| 검사 | 삭제 전 | 삭제 후 | 판정 |
| --- | --- | --- | --- |
| 콘 C++ 동기/비동기 ROS 출력 비교 | 기존 바이너리의 5개 시나리오 출력 저장 | 같은 스캔·TF에서 속도·조향·상태·경계·중앙 경로·제어점 비교 | 동일, float 비교 허용 오차 1e-7 / 상대 오차 1e-6 |
| 콘 C++ 기하·경계·메모리 관련 CTest | 7개 통과 | 7개 통과 | 동일 |
| 주차 선택 회귀검사 | 127개 통과, 1개 실패 | 127개 통과, 동일 1개 실패 | 신규 실패 없음 |
| OBJ Python 회귀검사 | 20개 통과, 1개 실패 | 20개 통과, 동일 1개 실패 | 신규 실패 없음 |
| Python 실행 구문 비교 | 원본 AST | 미사용 import·대입만 제외해 AST 비교 | 나머지 실행 구문 동일 |
| mux core·도식/JSON 연결 검사 | 기존 문서의 연결 누락 | 66개 통과 | 현재 토픽 계약 확인 |
| 미사용 import·지역변수 검사 | 정리 대상 존재 | 수정한 parking·OBJ 패키지 검사 통과 | 정리 확인 |

콘 비교 시나리오는 후보 부족, 정상 콘 통로, TF 누락, 정상 복구, 콘 소실입니다. 실제 ROS 2 메시지를 별도 DDS 도메인에 발행하고 기존/수정 바이너리의 명령과 Marker/경로 필드를 비교했습니다. 주차 검사는 core·목표 기하·주차 규칙·제어 지연·health·정지/휠·수동/자동·곡선 출차 관련 8개 파일입니다.

## 기존에 실패하던 항목

- 주차: `test_requested_parking_rules.py::test_unobserved_sectors_and_zero_ranges_are_free_but_measured_obstacles_are_not` — 기존 충돌 검사 기대값 불일치. 삭제 전부터 실패하며 이번 정리에서 동작을 바꾸지 않았습니다.
- OBJ: `test_core.py::test_config_yaml_matches_params` — 기본 YAML에 `start_score_min`이 없어 실패. 삭제 전후 동일합니다.
- 기존 콘 C++/초기 Python 비교용 `ros_output_parity`는 Python reference가 현재 `boundary_direction_weight=0` 설정을 거부해 시작하지 못합니다. 이 검사를 통과했다고 계산하지 않고, 현재 C++의 삭제 전/후 바이너리를 직접 비교했습니다.

이 결과는 검사한 입력에서의 동작 동일성을 확인합니다. 실제 차량·카메라·LiDAR를 연결한 주행 검증은 수행하지 않았습니다.

## 문서와 도식 반영

12개 `_ws/README.md`에 알고리즘·입출력·관련 논문/공식 자료를 정리했습니다. 자체 상태기계를 특정 논문 구현으로 포장하지 않습니다.

Draw.io의 기존 배치·간단한 박스 설명·진한 직각 연결선·하단 visualization은 유지합니다. 현재 코드와 달랐던 활성 연결 두 곳만 정리했습니다.

- YOLO → slow의 `/yolo/bump_sign`: 현재 YOLO가 발행하지 않아 활성 선 삭제. slow의 선택 입력 자체는 README에 표시합니다.
- lane → objavoid의 `/mission/lane`: 현재 car06의 `fork_shape=false`에서 구독하지 않아 활성 선 삭제. 조건부 기능은 코드와 README에 유지합니다.

`docs/2026_AutoRace.drawio`, PNG, SVG와 `architecture.md`, `architecture-topics.json`을 같은 기준으로 맞췄습니다. JSON은 기존 리스트 형식을 유지하고 주차 인계·mux 상태·시각화 관측 연결을 포함합니다.
