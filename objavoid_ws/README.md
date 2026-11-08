# objavoid_ws

정적 장애물을 LiDAR로 감지하고 좌·정면·우를 비교해 회피하는 ROS 2 Humble 노드다.
실행 노드는 `objavoid_node` 하나다. 감지·충돌 검사·제어 판단·ROS·RViz를
`node.py`에서 수행하며 별도 런치나 고정 횡위치 목표 추종 모듈은 사용하지 않는다.
전역 지도는 사용하지 않는다. 기본 3차로 보호 기능은 `/odom`과 시작 시 유효한 차선 중심 정보를
사용하며, 초기화 후에는 차선이 일시적으로 가려져도 odom 기준 도로 범위를 유지한다.

## 빠른 실행: 시뮬레이션

현재 버전은 **objavoid 노드 한 개 + 별도 RViz2** 구성이다.
objavoid용 launch 파일은 없다. 아래 명령은 `~/2026-AutoRace` 설치 경로 기준이다.
시뮬레이터를 먼저 켜고 차량을 **중앙 2차선**에 둔다.
기본 도로 보호를 사용하므로 `/scan`, `/odom`, 유효한 `/lane_lite/target` 입력이 필요하다.

### 1. 빌드: 최초 실행 또는 코드 변경 후

기존 objavoid 노드를 종료한 뒤 실행한다. RViz2도 아래 별도 터미널에서 실행한다.

```bash
cd ~/2026-AutoRace/objavoid_ws
source /opt/ros/humble/setup.bash
PYTHONNOUSERSITE=1 /usr/bin/python3 /usr/bin/colcon build \
  --symlink-install --packages-select autorace_objavoid
```

빌드가 성공한 뒤 다음 단계로 진행한다.

### 2. 터미널 A: lane 실행

lane_ws가 이미 빌드되어 있어야 한다. lane이 실행 중이면 중복 실행하지 않는다.

```bash
source ~/2026-AutoRace/simul/ros_env.sh
source ~/2026-AutoRace/lane_ws/install/setup.bash
ros2 launch autorace_lane_lite lane_lite.launch.py profile:=sim
```

### 3. 터미널 B: objavoid 실행

```bash
source ~/2026-AutoRace/simul/ros_env.sh
source ~/2026-AutoRace/objavoid_ws/install/setup.bash
ros2 run autorace_objavoid objavoid_node --ros-args \
  --params-file ~/2026-AutoRace/objavoid_ws/src/autorace_objavoid/config/objavoid.yaml \
  -p use_sim_time:=true
```

`manual_active=true`가 기본이다. 시작 위치가 1/3차선이면 마지막에
`-p initial_lane:=1` 또는 `-p initial_lane:=3`을 추가한다.

### 4. 터미널 C: 시뮬 차량에 명령 전달

mux 없이 시험할 때만 실행한다. 기존 mux/어댑터가 이미 명령을 전달하고 있으면
중복 실행하지 않는다. objavoid는 `/cmd_vel`을 직접 발행하지 않는다.

```bash
cd ~/2026-AutoRace
./simul/adapter.sh --no-gui --topic /mission/objavoid/command_05
```

### 5. 터미널 D: RViz2 실행 (선택)

objavoid 노드가 시각화 토픽을 발행한다. RViz 창은 별도로 실행한다.

```bash
source ~/2026-AutoRace/simul/ros_env.sh
source ~/2026-AutoRace/objavoid_ws/install/setup.bash
rviz2 -d ~/2026-AutoRace/objavoid_ws/src/autorace_objavoid/config/objavoid.rviz \
  --ros-args -p use_sim_time:=true
```

### 실차 실행

실차 센서·위치 추정·lane·기존 mux를 먼저 실행한다. 시뮬레이터 환경 파일 대신
실차 ROS 환경을 사용하고, 시뮬용 어댑터는 실행하지 않는다.
차량 치수·LiDAR 장착 위치·차선 폭·실제 시작 차선을 확인한다.

```bash
source /opt/ros/humble/setup.bash
source ~/2026-AutoRace/objavoid_ws/install/setup.bash
ros2 run autorace_objavoid objavoid_node --ros-args \
  --params-file ~/2026-AutoRace/objavoid_ws/src/autorace_objavoid/config/objavoid.yaml \
  -p use_sim_time:=false
```

### 정지 상태 확인과 재시작

| 로그 | 의미 |
| --- | --- |
| `WAIT_SCAN` / `STOP_SCAN_EXPIRED` | 스캔 대기 / 만료 |
| `WAIT_ODOM` / `WAIT_ROAD_REFERENCE` | 위치 또는 초기 차선 기준 대기 |
| `STOP_PATH_BLOCKED` | 진입/일반 주행 후보가 장애물 또는 추정 도로 외곽에 막힘 |
| `STOP_ROAD_BOUNDARY` | 현재 차체가 추정 도로 외곽을 벗어남 |
| `AVOID_LEFT` / `AVOID_RIGHT` | 선택한 방향으로 회피 중 |
| `ALIGN` / `STOP_ALIGNMENT_REFERENCE` | 도로 방향 정렬 / 정렬 기준 없음 |
| `LANE_FOLLOW` | 전방과 복귀 패스가 안전하여 차선 중심 보정 중 |

F5 초기화 감지는 구현하지 않았다. 반복 시험은 objavoid를 Ctrl+C로 종료하고,
시뮬레이터를 초기화한 뒤 lane 입력이 정상인 상태에서 objavoid를 다시 실행한다.
도로 외곽은 초기 차선 기준의 추정값이며 현재 차선 경계로 지속 갱신하지 않는다.

실행 중 비활성화는 아래 명령을 사용한다. 비활성화하면 제어 발행을 멈추며
연결된 mux/어댑터의 입력 만료 처리를 따른다.

```bash
ros2 param set /objavoid_node manual_active false
```

## 현재 제어 구성

- 직진 → 회피 방향 유지 → 도로 방향 정렬 → 최신 차선 중심 주행.
- 뒤쪽 관측 장애물/통로를 반영한 최초 방향 선택.
- 최소 0.15m~최대 0.7m 사이에서 긴 안전 패스 우선.
- 고정 옆 차선 목표 추종과 목표 오차 비용 계산은 제거.
- 실차 센서·차체 충돌·입력 만료·추정 도로 이탈 검사 유지.

## 입출력과 기본 구성

제어 노드는 `objavoid_node` 하나이며 `node.py`가 감지·충돌 검사·판단·ROS·RViz를,
회피 제어도 `node.py`에 포함한다. objavoid용 launch는 없다.

| 방향 | 토픽 | 용도 |
| --- | --- | --- |
| 구독 | `/scan` | LiDAR 점·군집·차체 패스 검사 |
| 구독 | `/lane_lite/target` | 초기 도로 기준, 최신 차선 중심 보정 |
| 구독 | `/odom` | 정렬 기준·차선 위치 추정 |
| 발행 | `/mission/objavoid/command_05` | Ackermann 속도·조향 명령 |
| 발행 | `/objavoid/path` | 최종 명령의 실제 검사 길이 궤적 |
| 발행 | `/objavoid/markers`, `/objavoid/clusters` | RViz 참고선·패스·장애물 표시 |

`/cmd_vel`은 직접 발행하지 않는다. scan은 Best Effort/최신 1개, odom은 SensorDataQoS,
명령은 Reliable depth 1, 시각화는 Reliable/Transient Local이다.

## 회피·차선 복귀 동작

1. `STRAIGHT`: 직진 경로가 안전하면 조향 0으로 진행한다.
   직진 앞보기 길이는 기본 1.0m(`max(preview_distance, far_distance)`)다.
2. 직진 앞보기가 막히면 정면 장애물을 우선 회피한다. 정면 장애물 뒤에 검출된
   클러스터 중 가장 가까운 장애물의 반대편을 선호한다. 예를 들어 뒤 클러스터가
   오른쪽이면 왼쪽으로 진입한다. 클러스터가 없거나 정중앙이면 차체 패스 여유로 결정한다.
   선택 방향도 실제 0.15~0.7m 차체 충돌 검사를 통과해야 한다.
3. `TURN`: 피하던 장애물이 아직 앞에 있고 가까운 패스가 안전하면 선택 방향을 유지한다.
   해당 방향의 실제 패스가 막히면 반대 방향의 안전한 후보도 재평가한다.
   진입 때 관측 박스를 odom 기준으로 저장하고, 장애물의 가까운 면이 차체 앞쪽을
   지나 옆으로 넘어가고 횡방향 차체 여유가 확보되면 방향 잠금을 해제한다.
   정면 1.15m/직진 1m 확보를 정렬 진입 조건으로 쓰지 않는다.
4. `ALIGN`: 차선 중심의 횡오차는 적용하지 않고 도로 방향으로 정렬한다.
   헤딩 오차가 5도 이내면 `LANE`로 전환한다. 새 전방 장애물이 있으면
   좌우를 다시 비교해 새 회피를 선택한다. 이전과 반대 방향도 가능하다. 정렬 기준이 없거나 안전한 보정 패스가 없으면 정지한다.
5. `LANE`: 최신 lane 중심을 따라 주행한다. lane이 0.5초 만료되면 odom 추정 중심을
   보조로 쓴다. 차선 보정이 막히면 절반 보정/직진을 검사하며 직진 경로 자체가 막히면
   새 회피 방향을 고른다.

방향 기준은 기본 도로 좌표, 도로 보호를 끈 경우 진입 odom 방향, 마지막으로 신선한
lane 헤딩을 사용한다. 로그의 `phase=STRAIGHT/TURN/ALIGN/LANE`으로 단계를 확인한다.

## 가변 길이 패스

0.7m 후보가 모두 막히면 **0.25m 같은 고정 단거리로 바꾸지 않는다.**
현재 관측점·박스·도로 기준으로 8회 이분 탐색하여 가능한 긴 패스를 찾는다.
선택한 실제 검사 길이와 조향은 최종 재검사 후 제어와 RViz에 동일하게 사용한다.
길이와 속도는 줄여도 차체 크기·장애물 박스·도로 외곽 조건은 완화하지 않는다.

무한히 짧은 공간에 명령을 내지 않도록 최소 검사 길이는
`max(min_preview_distance, near_speed × (scan_timeout + 1/publish_rate))`다.
기본 최소 패스 길이는 **0.15m**이며, 0.15~0.7m 사이에서 가변 길이를 탐색한다.
이 구간조차 막히면 정지한다. 이 하한은 입력 만료와 제어 주기를 고려한 값이며
실측 제동거리 모델은 아니다. 가변 패스 속도는 최대 near_speed=0.2m/s다.

회피의 가변 패스도 가까운 패스가 안전한 방향을 우선 유지하고 직진 후보로 바꾸지 않는다.
같은 길이에서 뒤 클러스터 방향 선호와 차체 여유가 비슷하면 작은 조향을 우선한다.
현재 보이는 점·박스와 일정 조향 곡선을 반복 검사하는 로컬 방식이며 다단계 우회·후진·
전역 경로 탐색은 아니다. 눈에 보이는 모든 빈 공간에 반드시 진입할 수 있다는 보장은 없다.

## 뒤 클러스터로 방향 선택

정면 통로를 막는 가까운 클러스터를 우선 찾고, 그보다 앞쪽(거리상 뒤)에 검출된
클러스터 중 차체 기준 거리가 가장 가까운 것을 선택한다. 그 클러스터의 좌우 위치와
반대 방향을 회피 선호로 사용한다. 관측되지 않은 장애물을 추정해 빈 공간으로 만들지 않는다.

정렬 후 2m 직진 가능 여부는 검사하지 않는다. 해당 출구 연결 검사와
`STOP_EXIT_BLOCKED`, 가중 평균 통로 점수는 제거했다.
실제 이동 후보는 0.15~0.7m 범위에서 차체 충돌·도로 이탈만 검사한다.
`visual_road_distance=2.0`은 RViz 도로 참고선의 표시 길이이며 제어 조건이 아니다.

## 충돌 검사와 도로 보호

차체는 base_link 기준 0.322×0.220m 직사각형이다. LiDAR 장착 위치
(x=0.103m, y=0, yaw=0)를 적용한 모든 유효 스캔 점과 작은 군집의 채워진 박스를
검사한다. 패스는 현재 위치를 포함한 25개 표본으로 검사하며 표본 사이 이동·회전
여유도 포함한다. 작은 군집은 최대 외곽 대각 1.0m, 연결 간격 0.12m, 최소 3점이다.
0/NaN/범위 밖 값은 빈 공간이 아니다. 기본 inf_is_clear=false다.

추가 도로 외곽 여유 0.1m 조건은 제거했다. **기존 차체 여유 0.02m와 몸체 전체가
추정 도로 안에 있어야 한다는 검사는 유지한다.** 내부 차선 이동은 허용한다.

도로 외곽은 시작 시 lane 중심/방향과 odom으로 한 번 잡는 직선 3차로 추정이다.
차선 폭은 0.52m, 전체 폭은 1.56m이며 기본 시작 차선은 2다.
이후 odom으로 차선 번호를 추정한다. 최신 lane 중심 보정과 별개로 외곽 원점은
계속 갱신하지 않는다. 실제 차선 경계를 직접 받는 기능은 없다.

초기 lane 중심 오차가 반 차선 폭(0.26m)을 넘으면 WAIT_ROAD_REFERENCE로 기다린다.
이 문제와 가변 패스 선택은 별개다. 센서/odom 만료·정면 미관측·정면 0.05m 이내·
도로 밖 차체 조건도 정지한다. F5 초기화 감지는 구현하지 않았다.

## 주요 설정

| 설정 | 기본값 | 용도 |
| --- | --- | --- |
| cruise_speed | 0.4m/s | 일반 주행·직진 통과 |
| avoid_speed | 0.25m/s | 회피·차선 보정·정렬 |
| near_speed | 0.2m/s | 근접·가변 길이 진행 |
| far_steering / near_steering | 0.30 / 0.32rad | 일반 / 근접 회피 최대 조향 (약 17.2 / 18.3도) |
| max_lane_steering | 0.15rad | 정렬·차선 보정 최대 조향 (약 8.6도) |
| preview_distance | 0.7m | 우선 검사 길이·RViz 최대 패스 |
| min_preview_distance | 0.15m | 허용하는 최소 패스 길이 |
| far_distance / near_distance / stop_distance | 1.0 / 0.5 / 0.05m | 개입·근접·정지 거리 |
| clearance_margin | 0.02m | 차체 안전 여유 |
| road_guard_enabled | true | 추정 도로 외곽 보호 |
| lane_width / initial_lane | 0.52m / 2 | 실제 도로 폭·시작 차선 |
| scan_timeout / odom_timeout / lane_timeout | 0.25 / 0.5 / 0.5초 | 입력 만료 |
| publish_rate | 20Hz | 제어 주기 |

고정 단거리 설정 `creep_preview_distance`와 추가 외곽 설정 `road_clearance_margin`은
제거했다. 코드 기본값보다 `--params-file` YAML이 우선한다.

## RViz 표시 설명

중앙 화면에는 다음 정보를 표시한다. 전방은 위, 차량 왼쪽은 화면 왼쪽이다.

- 색상 박스: 감지된 작은 장애물 군집의 관측 외곽(표시 여유 2cm).
- 노란 선: `/lane_lite/target`의 가까운/먼 **차선 중심 두 점을 연결한 참고선**.
  좌우 차선 경계가 아니며 토픽이 무효하거나 0.5초 만료되면 사라진다.
- 흰색 점선: 초기 차선·폭·odom으로 추정한 3차로의 바깥 경계.
  실제 영상에서 검출한 경계가 아니며, 차체 패스 검사와 같은 도로 범위다.
- `LANE1/2/3`와 `EST`: 실제 횡이동으로 계산한 현재 추정 차선.
- 청록 패스: 현재 명령 조향을 유지했을 때의 예상 궤적. 일반 주행은 최대 0.7m,
  `CREEP` 저속 진행은 장애물에 맞춰 실제 검사한 가변 길이만 표시한다.
  제어에서 차체 충돌 검사를 통과한 최종 명령과 같은 곡선이다.
  정지/비활성/입력 만료 시에는 Marker를 삭제하고 빈 Path를 발행해 이전 궤적을 남기지 않는다.

`Lane detections` 영상 패널에는 `/lane_lite/debug`의 실제 차선 검출 결과(BEV)를 표시한다.
빨강은 검출한 경계, 보라는 유지 중인 경계, 주황 점선은 한쪽 경계로 추정한 반대 경계다.
파랑은 차선 중심 경로다. 이 영상은 중앙 화면의 노란 중심 참고선과 별개다.
`autorace_lane_lite`가 실행 중이고 카메라 입력이 들어와야 표시되며,
영상 구독은 발행 노드와 맞게 Best Effort / Volatile로 설정했다.

격자·차체·거리·상태 글자·원본 스캔·부채꼴·화살표는 기본으로 숨긴다.
Displays 설정 패널은 숨겼다. 필요하면 Panels → Add New Panel → Displays로 추가할 수 있다.
`Lane reference and estimated road limits`는 `/objavoid/markers`의
`lane_center`, `road_limits`, `lane_id`를 표시하고, `Command path`는 `command_path`를 표시한다.
`Detected obstacles`는
`/objavoid/clusters`의 `cluster_boxes`만 표시한다.
점/라벨이 필요하면 같은 항목에서 `cluster_points`/`cluster_labels`를 켠다.

Path 메시지를 다른 도구에서 쓸 때는 `/objavoid/path`(nav_msgs/Path)를 사용한다.
마커와 Path는 Reliable / Transient Local, Fixed Frame은 `base_link`다.
장애물 박스에는 `base_link`와 스캔 프레임 사이 기존 TF가 필요하다.
마커의 수명을 짧게 두지 않고 동일 ID를 갱신해 발행 사이 화면 깜빡임을 줄였다.
마우스로 회전했다면 RViz를 위 설정으로 다시 열어 기본 시점을 복원한다.


## 검증

```bash
cd ~/2026-AutoRace
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=objavoid_ws/src/autorace_objavoid \
  python3 -m unittest discover -s objavoid_ws/src/autorace_objavoid/test -v
```

조기 회피·목표 유지·차선 복귀·가변 패스 길이·충돌/도로 외곽 정지를 검사한다.
실제 GUI 주행 검증을 대신하지 않는다. [코드 구성](docs/code_review.md)도 참고한다.

통과 판단은 정적 장애물의 최초 관측 외곽과 odom을 사용하는 방식이며 객체 ID 추적은 아니다.
odom/관측 외곽을 저장하지 못한 경우 현재 관측의 옆 통과 여부를 보수적으로 확인한다.
새 회피 후보와 정렬 곡선도 모든 장애물·도로 외곽 검사를 통과해야 한다.
도로 외곽에 가까워 직진 앞보기가 막힌 것만으로 정렬을 취소하지 않는다.

## 스캔 만료와 처리 시간

`STOP_SCAN_EXPIRED`는 lane 소실이 아니라 스캔 stamp/수신 경과가 scan_timeout=0.25초를
넘긴 상태다. 로그의 scan_age/receive_age로 확인한다. 시뮬 일시정지·발행 지연·전송 문제뿐
아니라 같은 실행 흐름에서 긴 경로 계산이 수신 처리를 지연해 발생할 수도 있다.

밀집 점군은 NumPy로 동일한 차체 직사각형/여유 계산을 묶어 처리한다.
포인트를 생략하지 않으며 작은 점군은 기존 개별 계산을 사용한다. 스캔 구독 대기열은
최신 1개로 제한하고, 계산 완료 후에도 만료를 재확인해 오래된 입력의 이동 명령을 막는다.
기존 출구 검사 때문에 계산이 길어지는 문제가 있었으나 출구 검사 자체를 제거했다.
같은 720점·앞/뒤 차량 재현 조건에서 전체 방향/패스 결정은 약 1~2ms였다.
환경/배치에 따라 달라지므로 전체 제어 주기 보장 수치는 아니다.
