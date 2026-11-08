# objavoid_ws

정적 장애물을 LiDAR로 감지하고 좌·정면·우를 비교해 회피하는 ROS 2 Humble 노드다.
실행 노드는 `objavoid_node` 하나다. ROS·감지·충돌 검사·RViz 발행은 `node.py`,
고정 목표 횡위치와 조향 계산은 `tracking.py`에서 수행한다. 별도 런치는 사용하지 않는다.
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
| `STOP_TARGET_ODOM` | 회피 목표 추종에 필요한 odom 만료·없음·프레임 불일치 |
| `STOP_TARGET_BLOCKED` | 목표 조향과 절반 조향의 긴·짧은 패스가 모두 막힘 |
| `STOP_PATH_BLOCKED` | 진입/일반 주행 후보가 장애물 또는 추정 도로 외곽에 막힘 |
| `STOP_ROAD_BOUNDARY` | 현재 차체가 추정 도로 외곽을 벗어남 |
| `TARGET_SHIFT` / `TARGET_ALIGN` | 고정 목표로 횡이동 / 방향 정렬 |

F5 초기화 감지는 구현하지 않았다. 반복 시험은 objavoid를 Ctrl+C로 종료하고,
시뮬레이터를 초기화한 뒤 lane 입력이 정상인 상태에서 objavoid를 다시 실행한다.
도로 외곽은 초기 차선 기준의 추정값이며 현재 차선 경계로 지속 갱신하지 않는다.

실행 중 비활성화는 아래 명령을 사용한다. 비활성화하면 제어 발행을 멈추며
연결된 mux/어댑터의 입력 만료 처리를 따른다.

```bash
ros2 param set /objavoid_node manual_active false
```

## 이번 변경의 기능 키워드

- 회피 목표 횡위치 고정 및 odom 기반 추종 (`tracking.py`).
- 횡오차·헤딩 기반 조향 감소와 반대 조향 정렬, 목표 도달 후 통과 단계 전환.
- 단거리 패스에서도 목표/조향 기준 유지, 회피 진입 시 큰 조향 우선.
- 목표 패스 차단·odom 이상 정지 및 목표 오차/헤딩 진단 로그.
- 목표 추종 회귀 테스트와 실행 문서 추가.

속도/거리 기본값과 RViz 설정은 기존과 동일하다. 현재 구성은 노드 내 RViz 발행을
유지하며 별도 시각화 모듈·런치·추가 대체 조향은 포함하지 않는다.

## 입출력

| 방향 | 토픽 | 타입 | 용도 |
| --- | --- | --- | --- |
| 필수 구독 | `/scan` | `sensor_msgs/msg/LaserScan` | 장애물 거리·군집·회피 패스 점 검사. SensorDataQoS |
| 초기 기준/참고 | `/lane_lite/target` | `std_msgs/msg/Float32MultiArray` | 시작 차선 중심·방향과 노란 참고선. Reliable, depth 1 |
| 차선 위치 구독 | `/odom` | `nav_msgs/msg/Odometry` | 실제 횡이동으로 차선 번호·도로 내 패스 판정. SensorDataQoS |
| 클러스터 발행 | `/objavoid/clusters` | `visualization_msgs/msg/MarkerArray` | 작은 장애물 군집 박스(점·라벨 선택), 10Hz |
| Path 발행 | `/objavoid/path` | `nav_msgs/msg/Path` | 최종 조향 명령의 예상 궤적, base_link, 10Hz |
| 표시 발행 | `/objavoid/markers` | `visualization_msgs/msg/MarkerArray` | 차선 중심 참고선·명령 패스·선택 진단 표시, 10Hz |
| 제어 발행 | `/mission/objavoid/command_05` | `ackermann_msgs/msg/AckermannDriveStamped` | 속도 m/s, 왼쪽 + rad, 현재 stamp, base_link. Reliable, depth 1 |

`lane_ws`, mux, 시뮬레이터 소스는 수정하지 않는다. `/cmd_vel`은 직접 발행하지 않는다.
도로 보호 초기화 이후에는 차선이 가려져도 LiDAR와 odom으로 회피를 계속한다.
일반 주행은 현재 추정 차선 중심과 진입 때의 도로 방향을 참고한다.

## 동작

정면 거리는 ±25도 안의 유효한 반환 중 최솟값이다. 거리 원점은 LiDAR,
각도와 조향의 부호는 정면 0, 왼쪽 +다. 기본 거리는 far=1.0 / near=0.5 / stop=0.05m다.

1. 정면이 1.0m 이상이고 회피 방향 기억이 없으면 `CLEAR`:
   현재 추정 차선 중심을 참고한 조향 또는 직진, 0.25m/s.
   회피 중이면 목표 횡위치와 차체 방향에 도달한 뒤 `PASS` 단계로 전환한다. 차선 방향 정보가 있으면 `PASS_ALIGN`으로
   차체를 차선과 평행하게 맞추고, 방향 보정이 없으면 `PASS_FORWARD`로 직진한다.
   이 단계에서는 차선 중심으로 당기는 횡오차 보정은 적용하지 않는다.
2. **회피를 시작하기 전에는 직진 차체 패스를 먼저 검사**한다.
   0.8m 직진 패스가 비어 있으면 `PATH_FORWARD`, 조향 0, 0.25m/s로 진행한다.
   양옆 차량이 가까워도 직진할 공간이 있으면 새 회피 조향을 만들지 않는다.
   회피를 시작한 뒤에는 이 짧은 패스가 잠깐 비어도 조향을 바로 0으로 바꾸지 않는다.
3. 직진 패스가 막히면 좌우 회피 방향을 선택하고 옆 차선 중심을 고정 목표로 저장한다.
   이후 목표 횡오차와 헤딩으로 조향을 계산한다. 진입 방향을 고를 때는 왼쪽·오른쪽의 큰 조향과
   작은 조향(0.12rad) 및 중간 조향을 모두 계산해 0.8m 근거리 패스가 충돌하는 후보를 제외한다.
   남은 방향 중 1.5~2.0m 앞 좌우 통로의 관측된 빈 공간이 더 많은 쪽을 선택한다.
   먼 거리 점수가 비슷하거나 관측이 부족하면 근거리 차체 여유로 결정한다.
   진입 후보의 큰 조향은 정면 0.5~1.0m에서 0.38rad(약 21.8도) / 0.20m/s,
   정면 0.5m 이내에서 0.40rad(약 22.9도) / 0.08m/s다.
4. 정면 0.05m 이내, 정면 미관측, 입력 만료 시 정지한다.
   옆 장애물이 0.5m 이내라는 이유만으로 그쪽 전체를 회피 후보에서 제외하지 않는다.
   실제 이동 가능 여부와 여유는 차체 패스로 검사한다. 좌우 최단거리는 먼 거리와
   근거리 여유가 비슷한 후보 사이에서 방향을 선택할 때 사용한다.
5. 0.8m 예상 패스를 따라 차량 직사각형과 실제 스캔 점 및 채워진 장애물 박스의 겹침을 검사한다.
   박스는 RViz의 관측 외곽과 동일하며 LiDAR 장착 위치·각도를 적용한다.
   차량 크기·2cm 여유와 패스 표본 사이의 이동/회전도 포함해 중심선만 빠져나가는 후보를 막는다.
   먼 거리 점수가 좋아도 근거리 패스가 막힌 방향으로는 진행하지 않는다.
   긴 후보가 모두 막히면 아래의 저속 단거리 후보도 검사하며, 이것까지 막히면 정지한다.
   `CLEAR` 차선 조향과 통과/복귀 조향에도 동일하게 적용한다.
   차선 조향이 막히면 직진 또는 좌우 대안을 검사한다. 최종 명령 발행 직전에도 재확인한다.
   같은 차체 패스가 초기 3차로 외곽을 넘는 경우에도 후보를 제외한다.

스캔 점이 있는 실제 제어에서는 옆/뒤 차량까지의 반경 거리만으로 회전을 금지하지 않는다.
가까운 차량이어도 선택한 차체 패스와 겹치면 탈락하며, 벽과 다른 반환 점도 모두 검사한다.
작은 군집의 외곽 박스는 내부 전체를 장애물로 사용해 반사점 사이를 관통하지 않도록 한다.

좌우 최단거리에는 연속 레이저 점 군집 중 외곽 대각 길이가 `object_max_extent=1.0m`
이하인 작은 군집을 사용한다. 근거리 패스 검사에는 크기와 무관하게 모든 반환 점을 쓴다.
차량 옆·바로 뒤 모서리까지 검사하므로 기존 ±90도 바깥의 옆 차량도 반영한다.
좌우에 작은 군집이 없고 해당 방향의 유효 빔이 충분하면 로그에 `clear`로 표시한다.
벽을 포함한 **모든 유효 스캔 점은 패스 겹침 검사에 유지**한다.
크기 기준은 차량 분류기가 아니며 짧게 관측된 벽은 작은 군집으로 남을 수 있다.

회피 진입 때 방향과 옆 차선 중심의 목표 횡위치를 저장한다. 이후 횡오차와 도로 대비
헤딩으로 조향을 줄이고 반대 조향으로 정렬한다. 횡오차 약 3cm 이내, 헤딩 5도
이내면 통과 단계로 전환한다. 전방에 다른 장애물이 남아 있어도 같은 방향 회전을
계속하라는 조건으로 사용하지 않는다.

목표 조향과 절반 조향을 0.8m, 0.25m 패스로 검사한다. 짧은 패스로 바뀌어도 목표와
조향 계산 기준은 유지한다. 모두 막히면 `STOP_TARGET_BLOCKED`로 정지한다.
고정 목표 후보가 막힌 상태에서 임의의 대체 방향으로 전환하지 않는다.
회피 진입과 목표 추종에는 신선한 odom이
필요하며 없거나 만료되면 `STOP_TARGET_ODOM`으로 정지한다.

선택한 방향 안에서는 차체 최소 여유가 최댓값의 25% 이내
(최대 `switch_margin=0.15m`)인 후보 중 회피 진입부터 큰 조향을 우선한다.
큰 조향의 여유가 부족하면 작은 조향을 사용하고, 충돌하는 후보는 선택하지 않는다.
좌우 유효 빔이 3개 미만이고 장애물 점으로도 확인하지 못한 방향은 unknown으로 제외한다.
0/NaN/범위 밖 값은 빈 공간이 아니다. 양의 Inf를 무반사로 해석할 수 있는 센서에서만
`inf_is_clear=true`로 설정한다.

주행 속도는 기본/회피/근접 각각 0.25/0.20/0.08m/s다. 정지는 0이다.
회피 진입 전의 직진 및 통과 단계에는 `cruise_speed`, 일반 회피 조향에는 `avoid_speed`를 적용한다.
`--params-file`의 YAML이 코드 기본값보다 우선하고, 적용 거리는 시작 로그에 표시한다.

패스 검사는 현재 보이는 점·관측 박스와 일정 조향의 예상 궤적을 사용한다.
도로 외곽은 아래의 초기 차선·폭·odom으로 추정한다. 실제 경계를 직접 검출하는 기능은 아니며,
좁은 배치는 정지할 수 있다. 차량 크기·LiDAR 장착 위치·차선 폭을 실제 차량과 맞춘다.

| 파라미터 | 기본값 | 의미 |
| --- | --- | --- |
| `vehicle_length`, `vehicle_width` | 0.322 / 0.220m | base_link 중심 직사각형 차체 |
| `lidar_x`, `lidar_y`, `lidar_yaw_deg` | 0.103 / 0m / 0도 | base_link 기준 LiDAR 장착 위치·각도 |
| `clearance_margin` | 0.02m | 패스 검사 시 차체 주변 여유 |
| `preview_distance` | 0.8m | 파란 패스 표시와 차체 충돌 검사가 공유하는 길이 |
| `creep_preview_distance` | 0.25m | 긴 후보가 모두 막혔을 때 저속으로 검사하는 길이 |

## 점선과 정지 판정

`lane_ws` 내부에서는 노랑과 흰색을 색상 마스크로 나누지만 `/lane_lite/target`에는
색상이나 실선/점선 종류가 들어 있지 않다. objavoid는 흰 점선을 장애물 또는 금지 경계로
등록하지 않는다. 1↔2↔3차선의 내부 구분선을 넘어가는 패스는 허용하며, 초기화 이후
점선 공백 때문에 차선 검출이 끊겨도 기존 도로 기준을 유지한다.

`STOP_PATH_BLOCKED`는 색상 판정이 아니라 장애물·도로 외곽 검사에서 후보가 탈락했다는 뜻이다.
예를 들어 3차선의 가까운 왼쪽 차량과 오른쪽 도로 외곽이 동시에 있으면 모든 0.8m 고정
조향 곡선이 막힐 수 있다. 이때 0.25m의 짧은 구간을 다시 같은 차체·박스·도로 검사로
확인한다. 안전한 구간이 있으면 `CREEP/...` 상태로 `near_speed`(기본 0.08m/s) 이하에서
진행하며 매 주기 재평가한다. 짧은 구간도 막히면 정지한다.

최소 정지 거리, 입력 만료, 미관측 정면, 도로 밖 차체 조건은 이 방식으로 우회하지 않는다.
보호 여유나 도로 폭을 줄이지 않고 미래 검사 길이와 속도만 함께 줄인다.
`path=0.25m` 또는 `path=0.80m` 로그가 실제 검사·표시한 길이다.
`long_rejected_obstacles`, `long_rejected_road`는 긴 후보 7개 중 장애물/도로 때문에 탈락한
개수이며 한 후보가 양쪽 검사에서 모두 탈락할 수 있다. 점선 때문이라는 뜻이 아니다.

## 3차로 외곽 보호와 차선 기억

기본 시작 위치는 **중앙 2차선**이다. 시작할 때 신선한 odom과 유효한 차선 중심/방향을
받아 직선 도로 기준을 한 번 잡는다. 이후 실제 횡위치가 왼쪽으로 이동하면 1차선,
오른쪽이면 3차선으로 추정한다. 조향 명령이나 회피 횟수로 번호를 올리거나 내리지 않는다.
차선 경계 부근에서는 3cm 히스테리시스를 사용한다.

각 차선 폭은 기본 0.52m(시뮬 차량 배치의 차로 간격), 전체 폭은 1.56m다.
1~3차선 사이의 이동은 허용하되, 차체와 여유를 포함한 미래 패스가 전체 외곽을 넘으면
직진·회피·통과·복귀 모두 거부한다. 먼 공간 비교도 도로 방향을 기준으로 하며,
1차선 왼쪽 또는 3차선 오른쪽 통로를 빈 공간으로 선호하지 않는다.
도로 안에 장애물을 피할 후보가 없으면 정지한다.

일반 주행은 현재 추정 차선 중심을 참고하므로 옆 차선으로 이동한 것을 무시하고
항상 처음의 2차선으로 되돌리지 않는다. 이후 카메라가 다른 차선을 잡아도 초기 도로 원점은
옮기지 않는다. 초기화 뒤 카메라가 가려지거나 잠깐 odom이 끊겨도 기준을 재설정하지 않는다.
odom이 없거나 0.5초 만료되면 정지하며, 좌표 프레임이 바뀌거나 차체가 이미 외곽을 넘은
경우도 정지한다. 시뮬 시간이 초기화되면 새 위치에서 기준을 다시 잡는다.

**노드를 재시작할 때 실제 시작 차선에 맞게 `initial_lane`을 지정해야 한다.**
중앙에서 시작하면 기본값 2를 사용하고, 1/3차선에서 재시작한다면 실행 인자에
`-p initial_lane:=1` 또는 `-p initial_lane:=3`을 추가한다.
이 기능은 초기 방향을 유지하는 직선 회피 구간용이다. 실제 곡선 도로의 경계 추적은 제공하지 않는다.

| 파라미터 | 기본값 | 의미 |
| --- | --- | --- |
| `road_guard_enabled` | true | odom 기반 3차로 보호 사용 |
| `odom_topic` | /odom | 위치 추정 토픽 |
| `initial_lane` | 2 | 노드 시작 시 실제 차선 번호(1/2/3) |
| `lane_width` | 0.52m | 차선당 폭; 실제 도로 폭에 맞춰 설정 |
| `lane_change_hysteresis` | 0.03m | 차선 번호 전환 여유 |
| `odom_timeout` | 0.5초 | odom 시각·수신 만료 기준 |

`WAIT_ODOM`, `WAIT_ROAD_REFERENCE`, `STOP_ODOM_EXPIRED`, `STOP_ODOM_FRAME_CHANGED`,
`STOP_ROAD_BOUNDARY` 로그로 기준 대기/위치 만료/프레임 변경/외곽 초과를 구분한다.
로그의 `lane`, `road_offset`은 현재 추정 차선과 진입 도로 중앙 기준 횡위치다.
LiDAR 단독 검출 테스트에서 `road_guard_enabled=false`로 시작하면 도로 보호가 꺼진다.

## 먼 거리 방향 선택

차량 기준 전방 1.5~2.0m, 좌우 각각 0.5m 옆의 직사각형 통로를 비교한다.
통로는 차체 크기와 `clearance_margin`을 포함한다. 라이다 빔이 그 통로의 끝까지
통과한 것으로 관측된 비율을 0~1 점수로 사용한다. 통로 안에서 장애물을 맞거나,
가까운 장애물에 가리거나, 유효하지 않거나 센서 시야 밖인 부분은 빈 공간 점수로 더하지 않는다.
유효 빔이 3개 미만이면 `unknown`이다. `inf_is_clear` 설정도 동일하게 적용한다.

이 통로는 우회 방향 비교용이다. 2m 고정 조향 곡선을 따라 주행하라는 계획 경로가 아니다.
실제 명령과 RViz 패스는 기본 0.8m 근거리 후보로 정하며, 모두 막히면 0.25m 저속 후보를
검사한다. 도로 보호가 켜져 있으면 추정 외곽 안의 후보만 선택한다.
좌우 통로는 현재 추정 차선을 기준으로 도로 방향에 맞춰 비교한다.

| 파라미터 | 기본값 | 의미 |
| --- | --- | --- |
| `direction_near_distance` | 1.5m | 먼 통로의 가까운 차체 중심 거리 |
| `direction_far_distance` | 2.0m | 먼 통로의 먼 차체 중심 거리 |
| `direction_offset` | 0.5m | 좌우 통로 중심의 횡방향 거리 |
| `direction_score_margin` | 0.10 | 좌우 빈 공간 비율 차이가 이를 넘으면 먼 거리 선호 적용 |
| `direction_release_time` | 1.5초 | 전방·옆이 비었다고 연속 확인해야 하는 시간 |

로그의 `far_left`, `far_right`가 먼 거리 빈 공간 점수이며, `hold=LEFT/RIGHT/NONE`은
기억 중인 방향이다. `phase=TURN/PASS/NONE`은 회피 진행/통과 직진/일반 주행 단계다.
`TARGET_SHIFT`, `TARGET_ALIGN`은 목표 횡이동/정렬 상태다.
`target_error`는 목표 횡오차, `heading`은 기준 방향 대비 차체 각도다.

## 클러스터 설정

| 파라미터 | 기본값 | 의미 |
| --- | --- | --- |
| `cluster_jump_distance` | 0.12m | 연속 점 사이 간격이 이를 넘으면 분리 |
| `cluster_min_points` | 3 | 이보다 작은 군집 제외 |
| `cluster_max_distance` | 3.0m | 군집 생성의 최대 반환 거리 |
| `object_max_extent` | 1.0m | 표시·좌우 선택에 사용할 군집 외곽 대각 길이 상한 |

0/NaN/Inf/범위 밖 값은 군집 연결을 끊는다. 360도 스캔만 시작/끝 인접 군집을 합친다.
박스는 실제로 관측한 표면의 외곽이며 차량 전체 크기 추정값이 아니다.
라벨 번호는 스캔별 번호로 지속 추적 ID가 아니다. 가림 때문에 차량이 분리될 수 있다.
만료되거나 사라진 군집은 화면에서 제거한다. `manual_active=false`여도 최신 장애물은 표시한다.
고정 목표 추종 검증은 `src/autorace_objavoid/test/test_tracking.py`에 있다.

## RViz 표시 설명

중앙 화면에는 다음 정보를 표시한다. 전방은 위, 차량 왼쪽은 화면 왼쪽이다.

- 색상 박스: 감지된 작은 장애물 군집의 관측 외곽(표시 여유 2cm).
- 노란 선: `/lane_lite/target`의 가까운/먼 **차선 중심 두 점을 연결한 참고선**.
  좌우 차선 경계가 아니며 토픽이 무효하거나 0.5초 만료되면 사라진다.
- 흰색 점선: 초기 차선·폭·odom으로 추정한 3차로의 바깥 경계.
  실제 영상에서 검출한 경계가 아니며, 차체 패스 검사와 같은 도로 범위다.
- `LANE1/2/3`와 `EST`: 실제 횡이동으로 계산한 현재 추정 차선.
- 청록 패스: 현재 명령 조향을 유지했을 때의 예상 궤적. 일반 주행은 0.8m,
  `CREEP` 저속 진행은 실제 검사한 0.25m만 표시한다.
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

목표 고정·반대 조향 정렬, 좌우 bicycle 운동 모델 수렴, 후보 차단 시 정지를 검사한다.
전체 GUI 주행 검증을 대신하지 않는다. [코드 구성 설명](docs/code_review.md)도 참고한다.
