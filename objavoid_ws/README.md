# objavoid_ws

LiDAR 정적 장애물 회피용 ROS 2 Humble 패키지다. **실행 노드는 하나**이며,
제어·센서 처리는 `node.py`, RViz 발행은 `visualization.py`에 있다. 별도 launch는 없다.

## 실행 명령어

시뮬레이터와 lane을 먼저 켠다. 기본 시작 차선은 **중앙 2차선**이다.
필수 입력은 `/scan`, `/odom`, 유효한 `/lane_lite/target`이다.

### 1. 빌드 — 코드 변경 후 한 번

```bash
cd ~/2026-AutoRace/objavoid_ws
source /opt/ros/humble/setup.bash
PYTHONNOUSERSITE=1 /usr/bin/python3 /usr/bin/colcon build \
  --symlink-install --packages-select autorace_objavoid
```

### 2. objavoid 실행 — 빌드 성공 후

기존 objavoid는 `Ctrl+C`로 종료하고 실행한다.

```bash
cd ~/2026-AutoRace/objavoid_ws
source ../simul/ros_env.sh
source install/setup.bash
ros2 run autorace_objavoid objavoid_node --ros-args \
  --params-file src/autorace_objavoid/config/objavoid.yaml -p use_sim_time:=true
```

1차선/3차선에서 시작하면 마지막에 `-p initial_lane:=1` 또는 `-p initial_lane:=3`을 추가한다.

### 3. RViz2 — 별도 터미널, 선택

```bash
cd ~/2026-AutoRace/objavoid_ws
source ../simul/ros_env.sh
source install/setup.bash
rviz2 -d src/autorace_objavoid/config/objavoid.rviz --ros-args -p use_sim_time:=true
```

### lane 또는 제어 전달이 아직 실행되지 않았다면

각각 별도 터미널에서 실행한다. 이미 실행 중이면 중복 실행하지 않는다.

**lane** — lane_ws가 먼저 빌드되어 있어야 한다.

```bash
source ~/2026-AutoRace/simul/ros_env.sh
source ~/2026-AutoRace/lane_ws/install/setup.bash
ros2 launch autorace_lane_lite lane_lite.launch.py profile:=sim
```

**시뮬 명령 전달** — 기존 mux 없이 objavoid만 시험할 때 사용한다.

```bash
cd ~/2026-AutoRace
./simul/adapter.sh --no-gui --topic /mission/objavoid/command_05
```

### 실차

실차 센서·위치 추정·lane·기존 mux를 먼저 실행한다. 시뮬 환경 파일과 시뮬 어댑터는 쓰지 않는다.

```bash
cd ~/2026-AutoRace/objavoid_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run autorace_objavoid objavoid_node --ros-args \
  --params-file src/autorace_objavoid/config/objavoid.yaml -p use_sim_time:=false
```

차량 치수·LiDAR 장착 위치·차선 폭·시작 차선은 실제 환경에 맞게 설정한다.

## 회피 로직

| 단계 | 조건과 동작 |
| --- | --- |
| `STRAIGHT` | 직진 통로가 안전하면 조향 0으로 진행한다. 기본 앞보기는 1.0m다. |
| 방향 선택 | 정면 장애물을 먼저 피한다. 그 뒤에 검출된 클러스터 중 가까운 장애물의 반대편을 선호한다. |
| `TURN` | 기존 방향에 안전한 후보가 있으면 유지한다. 해당 방향의 후보가 막히면 반대편도 평가한다. |
| 통과 확인 | 진입 때 관측 외곽을 odom 기준으로 기억한다. 장애물 가까운 면이 차체 앞쪽을 지나 옆으로 넘어가고 차체 여유가 확보되면 방향 잠금을 해제한다. |
| `ALIGN` | 차선 중심의 횡오차 없이 도로 방향으로 정렬한다. 새 전방 장애물이 있으면 즉시 방향을 다시 선택한다. |
| `LANE` | 헤딩 오차 5도 이내로 정렬되면 최신 lane 중심을 따른다. lane이 만료되면 odom 추정 중심을 보조로 쓴다. |

### 뒤 클러스터로 방향을 고르는 방법

1. 정면 차체 통로를 막는 가까운 클러스터를 찾는다.
2. 그보다 앞쪽, 즉 정면 장애물의 뒤에 검출된 클러스터를 비교한다.
3. 차체 기준 거리가 가장 가까운 클러스터가 **오른쪽이면 왼쪽, 왼쪽이면 오른쪽**을 선호한다.
4. 뒤 클러스터가 없거나 중앙에 있거나 좌우 거리가 같으면 가까운 패스의 여유로 결정한다.

회피 중인 기존 방향이 우선이며, 실제 패스가 막힌 경우에만 다른 방향을 평가한다.
선호 방향이라도 충돌 검사를 통과하지 못하면 선택하지 않는다.

### 실제 패스 선택

- **0.7m 후보 우선**, 모두 막히면 **최소 0.15m까지** 가능한 긴 구간을 8회 이분 탐색한다.
- 같은 길이에서 기존 방향 → 뒤 클러스터 선호 방향 → 차체 여유 순으로 후보를 좁힌다.
- 여유가 비슷하면 작은 조향을 우선한다. 짧은 패스라고 자동으로 크게 꺾지는 않는다.
- 안전한 최소 구간도 없으면 정지한다. 가변 길이 진행 속도는 최대 0.2m/s다.

**정렬 후 2m 직진 가능 조건이나 출구 연결 검사는 없다.** 뒤 클러스터는 방향 선택에 사용한다.
`visual_road_distance=2.0`은 RViz 참고선 표시 길이이며 제어 조건이 아니다.

## 충돌 검사와 도로 보호

- 차체: 길이 **0.322m**, 폭 **0.220m**, 주변 여유 **0.02m**.
- LiDAR 장착 위치·각도를 적용한 모든 유효 스캔 점과 작은 군집의 채워진 박스를 검사한다.
- 현재 위치를 포함한 25개 차체 표본과 표본 사이 이동·회전 여유를 반영한다.
- 추가 도로 외곽 여유 0.1m는 사용하지 않는다. 차체 전체가 추정 도로 안에 있어야 한다.
- 내부 차선 이동은 허용한다. 도로 외곽은 최초 lane/odom 기준의 직선 3차로 추정이다.
- 외곽 기준은 최신 차선 경계로 계속 갱신하지 않는다. 차선 중심 보정과는 별개다.
- 스캔 만료·정면 미관측·최소 정지 거리·odom 이상 조건은 유지한다.

밀집 점군은 NumPy로 일괄 검사하며 점을 생략하지 않는다. 스캔은 최신 1개만 보관하고,
계산 전후에 입력 만료를 확인한다. 최종 발행 명령도 같은 충돌 검사를 다시 통과해야 한다.

## 주요 설정

YAML: `src/autorace_objavoid/config/objavoid.yaml`. YAML이 코드 기본값보다 우선한다.

| 항목 | 현재 기본값 |
| --- | --- |
| 일반 주행 `cruise_speed` | 0.4m/s |
| 회피·정렬·차선 보정 `avoid_speed` | 0.25m/s |
| 근접·가변 길이 `near_speed` | 0.2m/s |
| 일반 / 근접 최대 조향 | 0.30 / 0.32rad ≈ 17.2 / 18.3도 |
| 정렬·차선 보정 최대 조향 | 0.15rad ≈ 8.6도 |
| 최대 / 최소 패스 | 0.7 / 0.15m |
| 앞보기 / 근접 / 정지 거리 | 1.0 / 0.5 / 0.05m |
| 스캔 / lane / odom 만료 | 0.25 / 0.5 / 0.5초 |
| 제어 주기 | 20Hz |
| 차선 폭 / 시작 차선 | 0.52m / 2 |
| 군집 연결 간격 / 최소 점 수 / 최대 거리 | 0.12m / 3개 / 3.0m |
| 작은 군집 최대 외곽 대각 | 1.0m |

## 로그와 정지 원인

`hold=LEFT/RIGHT/NONE`은 기억 방향, `phase`는 현재 단계, `path`는 선택한 검사 길이다.
`prefer_left/right`는 뒤 클러스터에 따른 방향 선호를 뜻한다.

| 로그 | 의미 |
| --- | --- |
| `WAIT_SCAN` / `STOP_SCAN_EXPIRED` | 스캔 없음 / stamp 또는 수신 경과 만료. `scan_age`, `receive_age` 확인. |
| `WAIT_ODOM` / `STOP_ODOM_EXPIRED` | odom 없음 / 만료. |
| `WAIT_ROAD_REFERENCE` | 신선한 초기 lane 기준을 받지 못했거나 초기화 조건 미충족. |
| `STOP_PATH_BLOCKED` | 안전한 패스 없음. `long_rejected_obstacles/road`는 긴 후보의 탈락 이유. |
| `STOP_ROAD_BOUNDARY` | 현재 차체가 추정 도로 외곽을 벗어남. |
| `STOP_ALIGNMENT_REFERENCE` | 도로 방향을 맞출 기준 없음. |
| `STOP_FRONT_UNKNOWN` / `STOP_TOO_CLOSE` | 정면 미관측 / 최소 정지 거리 이내. |
| `CREEP/...` | 0.7m보다 짧은 가변 길이로 저속 진행. |

실행 중 비활성화: `ros2 param set /objavoid_node manual_active false`.
비활성화하면 제어 발행을 멈추므로 mux/어댑터의 timeout 처리를 따른다.

## RViz와 입출력

| 토픽 | 용도 |
| --- | --- |
| `/scan` | LiDAR 입력, Best Effort·최신 1개 |
| `/lane_lite/target` | 초기 도로 기준 및 최신 차선 중심 입력 |
| `/odom` | 차량 위치·정렬 기준 입력 |
| `/mission/objavoid/command_05` | Ackermann 속도·조향 출력. `/cmd_vel` 직접 발행 없음. |
| `/objavoid/path` | 최종 명령의 실제 검사 길이 궤적 |
| `/objavoid/markers` | 청록 패스·노란 차선 중심 참고선·흰 추정 도로 외곽·차선 번호 |
| `/objavoid/clusters` | 작은 장애물 군집의 관측 외곽 박스 |

정지하면 패스를 지운다. 사라진 군집도 삭제한다. RViz Fixed Frame은 `base_link`이며
마커/Path는 Reliable·Transient Local이다. `/lane_lite/debug`는 lane 노드의 검출 영상이다.
부채꼴·화살표·차체·거리/상태 라벨·군집 점/라벨은 제거했다.

## 이번 커밋의 정리 내용

- RViz 발행을 `visualization.py`로 분리하고 사용하지 않는 표시를 삭제.
- 미사용 `rays`, `objects`, `turning`, 버려지는 헤딩 계산·인자 제거.
- 실행 명령과 회피 단계, 설정, 정지 원인을 README에 통합.
- 현재 사용 중인 제어·충돌·입력 만료 조건은 유지.

## 현재 한계와 검증

- 가까운 거리에서 시작하면 약한 조향으로 진행하다 회피 공간을 잃을 수 있다.
- 긴 패스 우선·작은 조향 우선·후보 차단 시 반대 방향 평가가 현재 동작이다.
  근접 시 조향 강화나 방향 전환 억제는 아직 반영하지 않았다.
- 관측된 정적 장애물 기준의 로컬 회피이며 후진·전역 경로 계획은 없다.
- 초기 lane 중심 오차가 0.26m를 넘으면 도로 기준 초기화를 기다린다.
- F5 초기화 감지는 없다. 반복 시험은 objavoid 종료 → 시뮬 초기화 → objavoid 재실행 순서다.
- 테스트는 로컬 `test/`에 보관하며 `.gitignore`로 업로드하지 않는다.

시각화 분리 전후 핵심 제어·안전 함수 17개 코드 구조와 스캔/군집 입력 40건의 결과를
비교했다. 로컬 테스트 30개와 빌드는 통과했다. 전체 GUI 배치와 실차 주행 검증은 별도다.
[코드 구성 설명](docs/code_review.md)도 참고한다.
