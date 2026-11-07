# parking_ws — LiDAR 주차 실행 안내

`/yolo/parking_sign`을 연속 **3회 true**로 받으면 주차를 시작합니다. **미션 시작 시 차량 오른쪽에 주차 차량과 빈 칸이 있는 배치**를 사용합니다. 직진한 뒤 우회전해 전진 주차하고, 시뮬레이션 시간으로 3초 정지한 다음 **진입할 때 기록한 회전 구간을 그대로 후진**합니다. 회전 시작점 부근에서 멈추며 초기 출발점까지 돌아가지 않습니다.

아래는 **시뮬레이터와 주차 노드를 ros2 launch, 어댑터 창을 `simul` 폴더에서 `./adapter`로 실행**하는 절차입니다. 장애물 편집과 전체 시뮬레이터 기능은 [simul README](../simul/README.md)를 참고하세요.

## 1. 환경 설정과 최초 빌드

Ubuntu 22.04 / ROS 2 Humble 기준입니다. Conda와 시스템 ROS가 설치되어 있어야 합니다. 시뮬레이터는 Conda `autorace-simul`의 Python 3.10, 주차 패키지는 시스템 `/usr/bin/python3`으로 빌드합니다.

**환경 파일은 각 터미널에서 source로 적용합니다.** `./env.sh`는 부모 터미널의 환경을 바꾸지 못합니다. 각 명령 블록의 `cd` 경로에서 실행하며, 어댑터는 `~/2026-AutoRace/simul` 폴더에서 실행합니다.

### 시뮬레이터 패키지 빌드

~~~bash
cd ~/2026-AutoRace
source simul/env.sh
python -m pip install colcon-common-extensions
cd simul
python -m colcon build --symlink-install --packages-select limo_parking_sim
~~~

`source simul/env.sh`는 `autorace-simul` 환경이 없으면 생성하고, 있으면 활성화한 뒤 필수 패키지와 ROS 환경을 준비합니다. ROS 실행 파일은 빌드 당시 Python을 사용하므로 **시뮬레이터 빌드는 이 Conda 환경에서** 수행합니다. 기존에 다른 Python으로 빌드했다면 빌드·설치 경로를 분리해야 합니다. [시뮬레이터 빌드 안내](../simul/README.md#7-ros-패키지로-빌드하기-선택)도 참고하세요.

### 주차 패키지 빌드

~~~bash
cd ~/2026-AutoRace
source parking_ws/env.sh
cd parking_ws
/usr/bin/python3 -m colcon build --symlink-install --packages-select autorace_parking
cd ..
source parking_ws/env.sh
~~~

`parking_ws/env.sh`는 시스템 ROS, 로컬 의존성, 빌드된 주차 패키지를 연결합니다. `ackermann_msgs`가 없다면 `sudo apt install ros-humble-ackermann-msgs`로 설치합니다. 이미 `parking_ws/.ros-deps`에 추출한 로컬 패키지가 있으면 환경 파일이 연결합니다. NumPy, colcon, RViz와 패키지에 선언된 ROS 의존성도 필요합니다.

처음 실행하거나 실행 파일·launch 구성을 변경했을 때 빌드합니다. `build`, `install`, `log`, `.ros-deps`는 Git에 포함하지 않습니다.

## 2. 시뮬레이션 실행 — 터미널 4개

환경 파일의 기본값은 **ROS_DOMAIN_ID=42, ROS_LOCALHOST_ONLY=1**입니다. 이미 지정된 0 이외의 도메인은 유지하므로 네 터미널의 값이 같아야 합니다. 42로 고정하려면 각 터미널에서 환경을 적용하기 전에 `export AUTORACE_SIM_DOMAIN_ID=42`를 실행하세요.

### 터미널 1 — 시뮬레이터

~~~bash
cd ~/2026-AutoRace
source simul/env.sh
source simul/install/local_setup.bash
ros2 launch limo_parking_sim parking.launch.py \
  mission:=parking view:=parking parking_adapter:=false
~~~

주차 미션 시작 위치와 주차장 시점으로 실행합니다. `parking_adapter:=false`로 두고 터미널 3에서 공용 어댑터를 실행합니다.

| 키 | 동작 |
| --- | --- |
| K | 키보드 제어 전환. 주차 노드 시험 시 **ROS 제어**로 둡니다. |
| P | 일시정지 / 계속 |
| F5 | 주차 미션의 시작 위치로 차량 리셋 |

실제 센서 드라이버나 두 번째 시뮬레이터는 함께 실행하지 않습니다. 센서와 `/clock`은 이 시뮬레이터에서 발행합니다.

### 터미널 2 — 주차 노드와 RViz

~~~bash
cd ~/2026-AutoRace
source parking_ws/env.sh
ros2 launch autorace_parking parking.launch.py use_sim_time:=true rviz:=true
~~~

주차 노드, 차량 모형 시각화 노드, RViz를 실행합니다. RViz가 필요 없으면 `rviz:=false`로 바꿉니다. 주차 launch는 어댑터를 자동 실행하지 않습니다.

### 터미널 3 — simul 공용 어댑터 창

~~~bash
cd ~/2026-AutoRace/simul
source env.sh
./adapter
~~~

`./adapter`는 기존 `adapter.sh`를 실행하므로 어댑터용 colcon 빌드나 `source install/local_setup.bash`가 필요 없습니다. **주차 (parking)** 행의 **/mission/parking/command_03을 ON**으로 켜고, 주차 단독 시험에서는 나머지 입력을 OFF로 둡니다. 처음에는 모두 OFF입니다. 창과 실행 터미널을 유지하세요.

어댑터는 주차 노드의 `AckermannDriveStamped`를 시뮬레이터의 `/cmd_vel` (`Twist`)로 변환합니다. 출력 토픽이 `/cmd_vel`인지 확인하고, 같은 토픽을 발행하는 기존 mux나 다른 어댑터는 종료합니다.

창 없이 주차 입력만 연결하려면 같은 `simul` 폴더에서 위 GUI 실행 대신 사용합니다.

~~~bash
./adapter --no-gui --topic /mission/parking/command_03
~~~

### 터미널 4 — 주차 시작 신호

~~~bash
cd ~/2026-AutoRace
source parking_ws/env.sh
ros2 topic pub -r 10 --times 3 /yolo/parking_sign std_msgs/msg/Bool '{data: true}'
~~~

YOLO 대신 시작 신호를 수동으로 3회 보냅니다. 이는 **시작 조건**이며, 이후 속도·조향 명령은 주차 노드가 발행합니다. 매 시도마다 `--times 3`을 사용하세요. true를 계속 발행하면 3회마다 미션을 다시 시작합니다.

false 또는 0.5초를 넘는 감지 공백은 연속 횟수만 초기화하며 진행 중인 미션을 취소하지 않습니다.

## 3. 주차 노드만 ros2 run으로 실행

터미널 2의 launch 대신 주차 노드 하나만 실행할 수도 있습니다.

~~~bash
cd ~/2026-AutoRace
source parking_ws/env.sh
ros2 run autorace_parking parking_node --ros-args \
  --params-file parking_ws/src/autorace_parking/config/parking.yaml \
  -p use_sim_time:=true
~~~

이 경우 RViz와 차량 모형 노드는 자동 실행되지 않습니다. 시뮬레이터, 공용 어댑터, 시작 신호는 앞 절과 동일합니다. **주차 노드를 launch와 run으로 중복 실행하지 않습니다.**

## 4. 상태 확인·재시도·종료

터미널 4에서 JSON 상태 전체를 확인합니다.

~~~bash
ros2 topic echo /parking/status --once --field data --full-length
~~~

정상 흐름은 다음과 같습니다. 상태 전환은 주차 노드 터미널에도 출력합니다.

~~~text
WAITING_SIGN → WAITING_SCAN → PLANNING → APPROACHING/PARKING
→ CONFIRMING → PARKED (3초 정지) → EXITING → EXIT_CONFIRMING → EXITED
~~~

`EXITED`는 출차 완료이며, 이후에는 0 속도 명령을 계속 발행합니다. `PLANNING`은 주차 완료가 아니라 진입 경로를 찾는 상태입니다. 상태 JSON의 `reason`, `goal_error`, `entry_trace_poses`, `exit_path_length_m`으로 정지 이유와 경로 기록을 확인할 수 있습니다.

**실차 설정에서는 목표 도착 전 앞범퍼 10cm 이내의 정면 장애물로 정지하고, 아래의 제한된 후진 복구를 수행합니다.** 실패했다면 아래 순서로 재시도합니다.

1. 터미널 4에서 주차 제어를 대기 상태로 초기화합니다.

   ~~~bash
   ros2 service call /parking/reset std_srvs/srv/Trigger '{}'
   ~~~

2. 시뮬레이터 창에서 **F5**로 차량을 시작 위치로 되돌립니다.
3. 시작 신호를 새로 3회 보냅니다.

   ~~~bash
   ros2 topic pub -r 10 --times 3 /yolo/parking_sign std_msgs/msg/Bool '{data: true}'
   ~~~

최초 시작과 `/parking/reset` 후에는 연속 3회 true가 필요합니다. 시작·재시작 후 5초 동안은 모든 시작 신호를 무시합니다. 5초 이후 활성 미션에 새 true 메시지가 들어오면 진행·실패·완료 여부와 관계없이 즉시 정지 명령을 보내고 목표·경로·위치 기준·출차 궤적·후진 횟수를 초기화해 새 시도를 시작합니다. 재시작 신호 이전에 측정된 스캔은 사용하지 않고 최신 스캔으로 빈칸과 경로를 다시 계산합니다. false는 활성 미션을 재시작하지 않습니다. true가 계속 발행되면 5초마다 다음 true에서 다시 시작합니다. 노드를 매번 재시작할 필요는 없습니다. `/parking/reset`은 차량을 이동시키지 않으며, **F5만 눌러서는 노드가 새 미션을 시작하지 않습니다.**

센서 수신 지연·위치 추정 실패·경로 오류에 따른 정지는 유지합니다. `/scan`이 0.5초 넘게 오래되면 `WAITING_SCAN`, 스캔 정합이 실패하면 `TRACKING_LOST`가 됩니다. 위치 추정이 실패했다면 위 재시도 순서를 사용하세요. 시뮬레이터 자체의 물리 충돌 처리는 주차 노드와 별개입니다.

종료할 때는 주차 노드, 어댑터, 시뮬레이터를 실행한 각 터미널에서 Ctrl+C를 누릅니다.

## 5. 주차 노드의 입출력

| 방향 | 토픽 | 타입 | 내용 |
| --- | --- | --- | --- |
| 구독 | `/scan` | `sensor_msgs/msg/LaserScan` | Best Effort 라이다 입력 |
| 구독 | `/yolo/parking_sign` | `std_msgs/msg/Bool` | 연속 3회 true로 시작·재시도 |
| 발행 | `/mission/parking/command_03` | `ackermann_msgs/msg/AckermannDriveStamped` | 활성 미션에서 20 Hz 속도(m/s)·조향각(rad) |
| 발행 | `/parking/path` | `nav_msgs/msg/Path` | 현재 차량 기준 진입·후진 경로 |
| 발행 | `/parking/obstacles` | `visualization_msgs/msg/MarkerArray` | 라이다 군집의 파란색 박스 테두리 |
| 발행 | `/parking/target` | `visualization_msgs/msg/MarkerArray` | 목표 위치·방향 화살표 |
| 발행 | `/parking/ldf` | `std_msgs/msg/Float32` | 현재 Pure Pursuit 추종 거리(m) |
| 발행 | `/parking/lookahead` | `visualization_msgs/msg/MarkerArray` | 노란색 LDF 원·청록색 실제 추종점과 연결선 |
| 발행 | `/parking/active` | `std_msgs/msg/Bool` | 미션 활성 여부 |
| 발행 | `/parking/status` | `std_msgs/msg/String` | 상태·시도 번호·명령·추정 오차 JSON |

주차 노드가 `/cmd_vel`을 직접 발행하지는 않습니다. 공용 어댑터가 `/mission/parking/command_03`을 받아 `/cmd_vel`로 전달합니다. `/parking/reset`은 `std_srvs/srv/Trigger` 서비스입니다.

`rviz:=true`일 때 별도 `parking_robot_visual` 노드가 `/parking/robot_description` (`std_msgs/msg/String`)을 발행합니다. RViz의 Fixed Frame은 **base_link**이며, 원시 라이다 표시용 `laser_link → base_link` TF는 시뮬레이터가 제공합니다. 차량 모형은 저장소의 `sensor_ws` 메시를 재사용하므로 해당 워크스페이스를 빌드할 필요는 없습니다.

## 6. 경로와 설정

주차 후보는 **미션 시작 좌표계의 오른쪽(y < 0)에 있는 차량 군집 사이**에서만 찾습니다. 왼쪽에 더 가까운 빈 공간이 보여도 선택하지 않습니다. 회전·이동 후에도 시작 위치와 방향을 기준으로 판단하므로, 오른쪽 주차 열이 현재 차량의 왼쪽에 보이더라도 좌측 주차로 바뀌지 않습니다. 위치 추정에는 원본 스캔을 사용합니다. 실차 설정의 `obstacle_min_cluster_size: 0.12`, `small_cluster_filter_max_distance: 1.0`은 차량 1m 이내에서 점이 3개 이상 모인 군집 중 가로·세로 최대 크기가 12cm 미만인 작은 표지판 형태의 군집을 장애물 시각화·충돌 검사·주차 후보 탐색에서 제외합니다. 1m 밖의 작은 장애물은 크기만으로 제거하지 않습니다. 접근 경로를 다시 계산할 때 기존 목표와 일치하는 주차 후보가 없고 다른 후보가 명확히 보이면 목표를 다시 선택합니다. 후보가 일시적으로 가려진 경우와 이미 확정한 최종 진입 경로는 유지합니다. 원본 거리와 미관측 영역 판정은 유지하므로, 표지판 뒤의 가려진 공간까지 자유 공간으로 간주하지 않습니다. 제외한 군집과 점 개수는 `/parking/status`의 `ignored_small_clusters`, `ignored_small_points`에서 확인할 수 있습니다.

직진 → 우회전 원호 → 칸 안쪽 직진 경로를 우선 생성합니다. 차체 치수·조향 한계·관측 형상은 **경로 생성 단계**에 반영합니다. 칸이 가려지면 관측된 구간까지만 접근해 다시 계획하고, 최종 진입 경로를 받으면 이를 유지합니다. 이동 추정은 연속 LiDAR 스캔 정합을 사용하며 주차 노드는 odom·IMU·TF를 구독하지 않습니다.

주차 완료 후에는 기록한 진입 궤적을 역순으로 따라갑니다. 칸 안쪽 직진과 회전 구간을 되짚고, 진입 전 방향과 다시 맞는 마지막 기록 자세에서 멈춥니다. 앞서 길게 직진한 구간은 후진하지 않습니다. 실차의 전후진 진입을 사용했다면 출차 중에도 기록된 구간에 맞춰 진행 방향을 바꾸며, 전환 전에 정지를 확인합니다.

설정 파일은 [parking.yaml](src/autorace_parking/config/parking.yaml)입니다. 값을 바꾸면 주차 노드를 재시작해 적용합니다.

| 설정 | 기본값 | 의미 |
| --- | --- | --- |
| `recognition_frames` | `3` | 연속 true 수 |
| `row_yaw` | `0.0` | 시작 차량 방향 기준 주차 열 방향(rad) |
| `parking_speed` | `1.0` | 기준 속도(m/s). 곡선·정지 지점 앞에서 감속 |
| `parking_hold_seconds` | `3.0` | 주차 후 대기 시간(시뮬레이션 초) |
| `exit_heading_tolerance` | `0.05` | 후진 끝점을 찾을 때 초기 방향과의 허용 차이(rad, 약 2.9°) |
| `allow_reverse` | `false` | 진입은 전진만 사용. 출차는 기록한 궤적을 후진 |
| `scan_timeout` | `0.5` | 스캔 수신 제한 시간(초) |

빈 칸이 관측되지 않거나 진입 경로를 생성할 수 없으면 대기합니다. 카메라 주차선 인식·영상 중심 보정·후진 T자 주차는 포함하지 않습니다. 실차 보정·검증은 수행하지 않았습니다.

### 실차 LIMO Pro의 경로 생성

실차에서는 `source parking_ws/real_env.sh` 후 `parking_real.yaml`과 `use_sim_time:=false`를 사용합니다.

~~~bash
cd ~/2026-AutoRace
source parking_ws/real_env.sh
ros2 run autorace_parking parking_node --ros-args \
  --params-file "$PWD/parking_ws/src/autorace_parking/config/parking_real.yaml" \
  -p use_sim_time:=false
~~~

차체는 길이 322mm·폭 220mm·축간거리 200mm를 사용합니다. 선택한 F안은 반경 0.40m의 90도 진입 원호입니다. `max_steering`은 경로와 제어에 쓰는 등가 중앙 조향각이고 `max_inner_steering`은 안쪽 바퀴 각도입니다. F안의 중앙 조향각은 약 26.6도, 같은 반경에 대응하는 안쪽 바퀴 각도는 약 32.5도입니다.

계산에 사용하는 최소 반경은 `max(min_turn_radius, wheelbase / tan(max_steering), wheelbase / tan(max_inner_steering) + steering_track / 2)`입니다. F안의 유효 반경은 0.40m이며, 90도 원호 길이는 `0.40 × pi / 2 = 0.628m`입니다. `max_turn_arc_length: 0.70`으로 이 원호가 잘리지 않게 설정합니다.

실차 설정의 `goal_from_observed_front: false`는 선택한 두 장애물의 마주 보는 가장자리 사이를 목표로 사용합니다. 주차 열 방향은 두 가장자리의 중간, 수직 방향은 두 군집 중심의 평균으로 잡습니다. 목표를 주차 방향 안쪽으로 밀거나 회전 반경에 맞춰 연장하지 않습니다. 생성된 경로는 실제 장애물 충돌 검사를 통과해야 합니다.

선택한 F안은 `min_turn_radius: 0.4`, `max_steering_paths_only: true`로 반경 40cm의 곡선을 사용합니다. `require_path_heading: true`는 경로를 주차 열에 대해 90도 방향으로 진입하도록 만들고, `require_parking_heading: false`는 도착 판정을 위치 중심으로 유지합니다. 두 조건은 별개입니다. `max_steering_rate: 0.0`이므로 계산된 목표 조향각을 바로 발행합니다. `lookahead`는 기본 0.25m이며 속도에 따라 최대 0.15m를 더합니다. 회전 감속 하한과 복구 후진 속도는 0.2m/s이며 최종 제동과 정지는 우선합니다. 후진 복구는 최대 30cm 직선입니다.

경로 탐색 시간은 전체 후보가 하나의 `planning_timeout` 예산을 공유합니다. 경로를 찾지 못하거나 결과가 늦어 폐기되면, `PLANNING`이 유지되더라도 이유가 터미널과 `/parking/status`에 표시됩니다.

`test/data/limo_pro_stationary_scans.json`의 실차 정지 스캔 6개(서로 다른 두 배치)로 경로 생성 회귀 검사를 수행합니다. 실제 차량의 주행·주차·출차 완료 검증은 별도로 필요합니다.

실차의 주차칸 후보 폭은 군집 가장자리 사이 주차 열 방향 거리 30~70cm입니다. 두 군집의 나란한 정도를 다시 검사하며, 주차 열 수직 방향의 중심 차이가 16cm 이하일 때만 후보로 사용합니다. 사이 장애물 검사와 실제 검출된 장애물에 대한 경로 충돌 검사는 유지합니다.

실차는 `assume_unobserved_free: true`로 라이다 시야 밖, 가려진 영역, 거리값 0·무효인 방향도 경로 생성 시 자유 공간으로 가정합니다. 이 가정은 실제로 해당 공간이 측정되었다는 뜻이 아닙니다. 검출된 장애물의 차체·여유 공간 충돌 검사와 정면 10cm 후진 조건, 최대 30cm·1회 제한은 적용합니다. 원본 라이다 값은 변경하지 않습니다.

실차는 `max_steering_on_curves: false`로 Pure Pursuit 조향을 사용합니다. 계산한 조향각에 `steering_gain`(기본 1.0)을 곱한 뒤 차량 조향 한계로 제한합니다. `steering_gain`과 기본 LDF `lookahead`는 ROS 파라미터 서비스로 실행 중 변경할 수 있으며 경로·지도·미션은 초기화하지 않습니다. 최대각 고정 모드는 선택 기능으로만 남습니다.

### 실차 후진 복구와 한글 진단

`parking_real.yaml`은 `recovery_on_front_obstacle_only: true`, `recovery_front_clearance: 0.10`을 사용합니다. 목표에 도착하기 전에 차량 정면의 장애물이 앞범퍼 10cm 이내에 들어온 경우에만 후진 복구를 허용합니다. 정면 범위는 차폭과 좌우 여유 공간 안쪽입니다. 빈 칸 미검출·계산 실패·미관측 공간만으로는 자동 후진하지 않습니다. 일반 진입 경로는 전진으로 탐색하며, 후진 복구는 0.2m/s 제한으로 최대 30cm 직선, 미션당 최대 1회입니다. 끝점에서는 제동하며 `/parking/reset` 후 새 시작 신호로 횟수를 초기화합니다. 방향이 바뀌는 지점에서는 측정 속도가 0.03m/s 미만인 상태를 새 스캔 3회로 확인합니다. 복구 끝점 도착을 주차 완료로 처리하지 않습니다.

후진에는 현재 스캔의 자유 공간, 실제 지나온 차체 영역, 최근 5초 안에 관측·검사한 경로 영역을 사용할 수 있습니다. 현재 실차의 미관측 자유 공간 가정은 위 설명을 따릅니다. 초기 30cm 자유 공간 가정은 미션 시작 위치에 고정하므로 재계획할 때마다 미관측 후방으로 넓어지지 않습니다. 새 장애물은 과거 자유 공간보다 우선하며, 복구 주행 중 새 장애물이 검출돼도 복구 경로를 중단하고 재계획합니다.

라이다는 하드웨어 특성에 맞는 기존 전방 220도(±110도) 기본 설정을 사용합니다. 별도의 360도 주차 프로파일은 사용하지 않습니다. `assume_unobserved_free: false`에서는 무효·누락된 거리와 후방 미관측 영역을 미관측으로 취급하며, 사용자가 비어 있다고 확인한 후방에 대해서만 `blind_reverse_distance: 0.30`으로 미션 전체 최대 30cm의 미관측 후진을 허용합니다. 한 번의 복구 경로도 최대 30cm입니다. 실차의 `recovery_left_when_goal_right: false` 설정에서는 주차 칸이 어느 쪽에 있어도 좌측 곡선 없이 직선 후진합니다. 계획을 채택할 때 허용 거리를 차감하므로 재계획을 반복해 허용 범위가 계속 늘어나지 않습니다. 관측된 장애물은 이 허용보다 우선하며, 허용한 미관측 경로는 주행하기 전 관측된 공간으로 기록하지 않습니다.

~~~bash
cd ~/2026-AutoRace
source parking_ws/real_env.sh
ros2 launch autorace_sensor_bringup sensors.launch.py
~~~

터미널에는 시작·상태 전환·경로 생성·초기화·오류·정지 원인을 한글로 출력합니다. 같은 이유로 정지해 있어도 5초마다 안내합니다. `/parking/status`의 `state`는 기존 통신 코드를 유지하고 `state_text`, `reason`과 위치 추정 설명은 한글입니다. `planning`에는 후보 수·차단 원인·계산 시간·복구 거리·스캔 시야가 들어갑니다. 측정 속도가 0인 상태가 지속되더라도 주행 무응답 타이머로 정지하거나 주차 제어권을 해제하지 않습니다. 활성 미션의 경로 추종 명령을 계속 발행합니다. 시작 신호 대기 중에는 빈 경로와 목표·추종점 삭제 메시지를 반복 발행해 RViz에 이전 경로가 남지 않도록 합니다. 장애물 박스는 새 스캔의 같은 ID를 갱신하고 사라진 ID만 삭제하므로 대기 타이머가 박스를 지우지 않습니다. 스캔 무효·초기화 시에는 박스도 삭제하며, 갱신이 끊긴 박스는 0.6초 뒤 만료됩니다.

~~~text
주차 상태: 주차 시작 신호 대기 → 주차 경로 계산 중 (시도 1)
관측 구간 접근 경로 생성: 36개 점, ...
주차 상태: 전진·후진 전환을 위한 정지 확인 중
주차 상태: 후진 복구 후 재탐색 중
~~~

2026-11-07 최대 좌회전 수동 주행의 휠 속도·보정된 관성 센서로 추정한 반경 중앙값은 약 36.5cm, 등가 중앙 조향각은 약 28.7도였습니다. 이는 바퀴 각도 센서의 직접 측정이 아닙니다. 해당 구간의 원시 `/odom` 회전속도와 관성 센서 회전속도가 달랐고, 반대 방향 및 ROS 조향 명령에 대한 응답은 아직 보정하지 않았습니다. 따라서 이 단일 수동 측정만으로 ROS 조향 한계를 확대하지 않고 문서 기반의 보수적 제한을 유지합니다. 기록은 로컬 `parking_ws/captures/max_steering_20261107/`에 보관합니다.

## 7. 실행 문제 확인

### 토픽 목록에 /parameter_events와 /rosout만 보일 때

기존 터미널에서 환경 파일을 다시 읽고 조회합니다.

~~~bash
cd ~/2026-AutoRace
source parking_ws/env.sh
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY"
ros2 topic list -t
ros2 topic echo /scan --once --qos-reliability best_effort --field header
~~~

공통 환경 파일은 `ros2 topic list` 등의 조회를 데몬 없이 직접 DDS에 연결합니다. 파일 수정이 기존 터미널에 자동 적용되지는 않습니다. 계속 보이지 않으면 시뮬레이터 실행 상태와 터미널 간 도메인을 확인하세요.

### 시작 신호를 보냈는데 움직이지 않을 때

- 시뮬레이터가 ROS 제어이고 일시정지 상태가 아닌지 확인합니다.
- 공용 어댑터에서 parking 입력이 ON인지 확인합니다.
- `/parking/status`의 전체 `state`와 `reason`을 확인합니다.
- `ros2 topic info /scan`, `ros2 topic info /clock`으로 발행자가 각각 하나인지 확인합니다.

### ROS 패키지를 찾지 못할 때

`autorace_parking`은 주차 패키지 빌드 후 `source parking_ws/env.sh`, `limo_parking_sim`은 Conda에서 시뮬레이터 빌드 후 `source simul/env.sh`와 `source simul/install/local_setup.bash`를 적용합니다. Python 경로가 달라 실행에 실패하면 해당 패키지를 지정된 Python으로 빌드했는지도 확인하세요.

### 실차 주차용 RViz만 실행

실차 시각화 설정은 [parking_real.rviz](parking_real.rviz)입니다. 이미 실행 중인 주차 노드와 별도로 아래 명령을 실행합니다. 차량 제어 노드나 주차 시작 신호는 실행하지 않습니다. 실차 주차 동작의 보정·검증 여부와는 별개인 시각화 설정입니다.

~~~bash
cd ~/2026-AutoRace
source parking_ws/env.sh
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
ros2 run rviz2 rviz2 -d "$PWD/parking_ws/parking_real.rviz" \
  --ros-args -p use_sim_time:=false
~~~

`ROS_DOMAIN_ID`는 차량 드라이버·주차 노드와 동일하게 맞춥니다. 차량의 그래픽 데스크톱 터미널 또는 X11 전달이 설정된 SSH 세션에서 실행하세요. 일반 SSH에는 화면 연결이 없으므로 RViz 창이 열리지 않을 수 있습니다.

차량을 따라가는 `base_link` 기준의 위에서 내려다보는 화면에 빨간 라이다 점, 파란 장애물 박스, 초록 주차·후진 경로와 목표 화살표를 표시합니다. 격자는 20 cm이며, 오른쪽 주차 공간을 함께 보도록 시점을 맞췄습니다. 차량 모형은 센서 bringup의 `/robot_description`을 사용하며 별도 `parking_robot_visual` 노드는 필요 없습니다. 센서 bringup의 `enable_description`은 켜 둡니다. 라이다 표시에는 `base_link`와 `laser_link` 사이의 TF가 필요합니다.

## 8. 검증 도구

저장소 루트에서 실행합니다. 테스트에는 실제 실행 중인 시뮬레이터와 다른 ROS 도메인을 사용합니다.

~~~bash
cd ~/2026-AutoRace
source parking_ws/env.sh
ROS_DOMAIN_ID=176 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /usr/bin/python3 -m pytest \
  -p no:cacheprovider -q parking_ws/src/autorace_parking/test

# 시뮬레이터 운동·라이다 모델로 주차와 후진을 검사하고 그림/JSON 저장
/usr/bin/python3 parking_ws/tools/check_parking_sim.py --output /tmp/parking-check
~~~

연속 감지·재시도, 시작 기준 오른쪽 후보 선택, 회전 후 좌표 변환, 직진 후 우회전, 최종 경로 유지, 주차 대기와 진입 회전 후진, 센서 단절·시각 허용 범위·초기화와 ROS 메시지를 검사합니다. 검증 도구의 실제 차량 위치는 결과 판정과 그림에만 사용하며 주차 알고리즘에 입력하지 않습니다.

RViz에서 LDF를 보려면 `MarkerArray` 표시를 추가하고 토픽을 `/parking/lookahead`로 선택합니다. 거리 수치는 `ros2 topic echo /parking/ldf`로 확인합니다. 추종점은 실제 제어기가 선택한 현재 `base_link` 좌표이며, 추종을 중단하면 오래된 점 마커를 지웁니다.


### 주차 장애물 기억 지도

실차의 `obstacle_memory_enabled: true`는 미션 시작 좌표계에 표지판 필터를 통과한 라이다 점을 누적합니다. 측정 시각 위치로 변환하며 위치 추정이 유효할 때만 갱신합니다. 2cm 격자, 차량 주변 6m, 최대 30,000개 격자로 제한합니다. 장애물 중심은 정적 2차원 KF로 보정하고 25cm 이내 최근접 일대일 연결로 ID를 유지합니다. 실제 충돌 검사는 KF 표시 박스가 아닌 누적 측정 점을 사용합니다.

누적 점은 후보 탐색과 경로 충돌 검사에 사용하고, 정면 10cm 복구 후진 조건은 최신 스캔으로 판단합니다. 원본 라이다 점을 사용하는 차량 위치 추정에는 누적 점을 되먹임하지 않습니다. 시야 밖이나 0·NaN 거리로 기억점을 지우지 않으며, 유효 광선이 해당 위치보다 3cm 이상 멀리 통과한 것이 서로 다른 스캔 3회에서 연속 확인되면 삭제합니다. 무한대 거리도 센서 유효 거리까지만 빈 공간으로 인정합니다.

RViz에서 `PointCloud2`의 `/parking/obstacle_memory`를 추가하면 청록색 기억점을 볼 수 있습니다. `parking_real.rviz`에는 해당 표시를 추가했습니다. `/parking/obstacles`의 파란 박스는 이번 스캔에서 관측된 장애물, 주황 박스는 기억 중인 장애물입니다. `/parking/status`의 `obstacle_memory`에는 `cells`, `tracks`, `observed_tracks`가 들어갑니다.

새 true로 재시작하거나 `/parking/reset`을 호출하면 지도·KF·표시를 모두 초기화합니다. true가 계속 들어오면 기존 규칙대로 약 5초마다 지도도 초기화됩니다. 센서 일시 중단에는 저장 내용을 보존하며, 전체 SLAM의 루프 폐쇄나 디스크 지도 저장은 수행하지 않습니다. 지도 정확도는 차량 위치 추정 오차의 영향을 받습니다.


### 후진 전 진입 경로 고정

실차의 `hold_entry_path: true`는 최초 채택한 진입 경로를 미션 좌표계에 고정합니다. 직선·곡선의 위치와 모양을 주행 중 새 경로로 교체하지 않으며 `/parking/path`에도 지나온 구간을 포함한 전체 경로를 표시합니다. 차량 기준 화면에서 보이는 이동·회전은 좌표 변환이며 경로 재계산이 아닙니다.

정면 10cm 조건으로 멈추더라도 기존 경로를 유지하다가 후진 복구 경로를 채택할 때 고정을 해제합니다. 주차 완료 후 대기 동안도 진입 경로를 표시하고, 출차를 시작하면 기록된 출차 경로로 바꿉니다. 짧은 부분 경로만 채택된 경우에는 그 끝에서 `PATH_HELD`로 정지해 경로를 유지합니다. 자동으로 나머지 구간을 새로 만들지는 않습니다. 센서·위치 추정 문제에는 정지하면서 경로를 보존하고, 새 true 재시작·`/parking/reset`에서는 기존 규칙대로 모두 초기화합니다.

### 선택한 두 장애물과 경로 제어권

선택한 주차칸을 만드는 두 군집의 박스는 자홍색으로 표시합니다. 누적 지도 점의 KF 추적 ID를 계획 결과에 연결하므로 현재 관측 여부와 관계없이 선택된 두 객체를 표시합니다. 다른 객체는 기존 파란색·주황색을 유지합니다. `/parking/status`의 `selected_pair`에는 선택한 군집과 추적 ID가 들어갑니다. 경로가 아직 없더라도 선택한 후보의 목표 화살표는 표시할 수 있습니다.

주차 경로가 없을 때 `/mission/parking/command_03`을 반복 발행하지 않습니다. 경로가 채택되면 제어를 발행하고, 주행 중 경로가 없어지면 0 명령을 한 번 보낸 뒤 중단합니다. mux 입력이 0.25초 뒤 만료되면 다른 상위 미션이 없는 경우 최신 lane 입력을 선택합니다. 경로를 보유한 상태의 정면 정지·주차 대기·센서 이상 정지는 기존처럼 parking의 정지 명령을 유지합니다. `publishing_control`은 반복 제어 발행 여부를 나타냅니다.


### 객체 쌍·공간 검증과 실시간 조향 조절

누적 지도 후보는 점의 유클리드 분할이 아닌 KF 객체 ID별 점 집합으로 만듭니다. 같은 ID의 조각끼리는 주차칸을 만들 수 없고, ID 없는 점은 후보 객체에서 제외하지만 충돌 검사에는 사용합니다. 표시 박스도 같은 객체 점 집합의 실제 경계를 사용하며 KF 중심 때문에 박스를 점에서 옮겨 그리지 않습니다.

실차 `validate_parking_space: true`는 서로 다른 객체 쌍에 대해 30~70cm 폭과 16cm 나란함 조건 외에 객체 사이의 모든 장애물 점을 검사하고 목표 위치의 차체 및 4cm 여유 공간이 비었는지 검사합니다. `/parking/status`의 `planning.candidate_checks`에 폭·필요 폭·통과 여부·거부 이유가 들어갑니다. 빈 공간 검사에서 미관측만으로는 후보를 제외하지 않습니다.

원격 노드를 최신 코드로 재시작하면 로컬 `parking_ws/tune.sh` 조절창에서 조향 비율과 기본 LDF를 설정할 수 있습니다. 또는 `ros2 param set /parking_node steering_gain 0.8`, `ros2 param set /parking_node lookahead 0.30`을 사용합니다. 두 파라미터는 유한한 양수여야 하며 잘못된 묶음 변경은 전체 거부합니다. 재시작 시 YAML 기본값으로 돌아갑니다.


### 전체 실시간 주차 튜닝

로컬 `tune.sh`는 이 버전의 원격 노드를 재시작한 뒤 원격 노드가 지원하는 숫자·논리 파라미터를 네 탭으로 표시합니다. 위치 추정·기억 지도·장착 좌표·토픽·시계 설정은 대상에서 제외합니다. 군집·후보 선택, 차량·경로 계획, 조향·속도 추종, 후진·미션 설정을 바꿀 수 있습니다. 모든 변경은 실제 계산 객체에 반영하며 잘못된 묶음은 전체 거부합니다. 필터·기하 조건을 바꾸면 이전 계산 결과는 취소하지만 기존 고정 경로는 보존합니다. 센서 갱신과 다음 계획부터 새 조건을 사용합니다.

최소 폭 `min_gap`의 실차 기본값은 0.30m이며 실제 최소 폭은 차폭과 좌우 여유 공간까지 고려한 큰 값입니다. `pair_filter_counts`는 오른쪽 아님·객체 크기·간격 부족/초과·나란함 불일치·같은 객체·사이 객체의 제외 횟수를 알려줍니다. `candidate_checks`는 추가 빈 공간 검사 결과를 알려줍니다. 두 정보는 터미널과 조절창 상태 영역에 표시합니다.


### 둘러싼 공간과 끊김 없는 재계획

최신 실차 기본값은 둘러싼 빈 공간 탐색과 연속 재계획을 사용합니다. 3개 이상의 서로 다른 KF 객체가 지지하는 공간에서 차체와 여유 공간 전체가 객체 중심들의 볼록 경계 안에 들어가는 위치를 찾습니다. 점이 아니라 객체 ID 수를 세며, 물체가 한쪽에만 나란히 놓인 경우에는 둘러싼 공간으로 인정하지 않습니다. 서로 붙은 지도 조각은 같은 물체로 묶어 셉니다. 3~4개에서는 임시 목표를 보정하고 5개 이상이면 주차칸을 확정합니다. 확정 후에도 같은 주차칸의 빈 공간 중심 좌표를 계속 보정하며 다른 칸으로 뛰어가지 않습니다. 공간 계산은 고정 미션 좌표계에서 수행합니다.

최소 객체 크기 0.14m, 최대 0.65m, 주변 객체 범위 1.0m, 목표 탐색 간격 0.05m로 설정했습니다. 임시 목표의 보정 비율은 0.25, 1회 최대 보정 거리는 0.08m입니다. 주차 속도는 0.25m/s, 조향 비율은 1.0, 기본 LDF는 0.25m, 재계획 주기는 0.30초, 탐색 제한 시간은 0.35초, 도착 오차는 0.06m입니다. 위치 추정과 기억 지도 알고리즘의 파라미터는 변경하지 않았습니다.

경로가 있는 동안 반복 true는 경로를 초기화하지 않습니다. 계획 실패·지연으로 새 경로가 연결되지 않으면 기존 경로와 제어를 계속 유지합니다. 새 경로는 현재 위치로부터 15cm, 방향 0.35rad 이내의 연결 가능 지점에서 교체합니다. 가까운 주행 구간에 실제 장애물이 나타나거나 기존 경로 끝에서 새 경로가 없으면 경로를 표시한 채 정지합니다. 경로 없이 계속 이동하는 보장은 하지 않습니다. 미션 초기화 시 확정 목표도 초기화합니다.

`goal_confirmed`, `goal_support_count`와 `planning.enclosure_objects`, `enclosure_candidates`로 확인할 수 있습니다. 로컬 조절창에는 새로운 파라미터와 임시·확정 상태 표시를 추가했습니다. 둘러싼 공간에서도 `min_gap`, `max_gap`으로 장애물 경계까지 확보되는 자유 공간 지름을 제한합니다. `row_tolerance`와 앞면 기준 깊이 보정은 기존 두 객체 방식에만 적용합니다. 목표 가까이에서는 12cm보다 짧은 Ackermann 이동 단위로 마지막 진입을 탐색합니다. 초록 구의 중심이 목표 좌표이며 화살표는 방향입니다. 원격 노드와 로컬 조절창을 재시작해야 새 모듈과 기본값을 적용할 수 있습니다.


### bag 분석 후 후진 복구와 검증

`recovery_on_path_failure: true`이면 목표가 앞쪽 25cm 이내 또는 뒤에 있고 전진 계획이 `recovery_failure_count: 3`회 연속 실패했을 때도 후진 복구를 허용합니다. 먼저 정지 상태를 센서 3회로 확인하고, 실제 지나온 차체 구간 또는 유효한 라이다 관측으로 확인한 후방만 사용합니다. 새 장애물과 계산 이후의 위치 이동을 다시 검사합니다. 복구는 0.20m/s 이하 직선으로 한 번 최대 30cm이며, `recovery_total_distance: 0.30`으로 미션 누적 계획 거리도 제한합니다. 복구할 수 없으면 기존 경로를 유지하며 정지합니다. 앞범퍼 10cm 근접 복구 조건도 유지합니다.

### 기록 bag으로 검증

`tools/analyze_parking_bag.py`는 센서 메시지만 직접 읽어 노드에 입력하고 제어 출력을 파일로 수집합니다. `tools/check_parking_bag_model.py`는 그 지도에서 차량이 새 명령에 반응하는 Ackermann 모델을 실행합니다. 두 도구는 `ROS_DOMAIN_ID=160..199`, `ROS_LOCALHOST_ONLY=1`을 요구하며 실차 제어 토픽으로 출력하지 않습니다. `--source`는 `autorace_parking/`과 `config/`를 포함하는 패키지 디렉터리입니다. bag에는 주차 시작 신호와 주차 명령이 없으므로 기본적으로 첫 센서 시점에 시작 신호를 주입합니다. 모델 결과는 실제 타이어 미끄러짐이나 하드웨어 응답을 실증하지 않습니다.

목표의 기본 보정 위치가 새 장애물과 겹치면 1회 최대 8cm 한도 안에서 보정 비율을 높여 비어 있는 중심으로 이동합니다. 후진 복구가 끝나거나 새 장애물로 중단되면 마지막 경로와 정지 제어권을 유지하며 새 전진 경로를 기다립니다. 이 대기 중에는 lane 제어로 전환하지 않습니다.


### 2026-11-07 19:36 bag에 따른 실차 조향 보정

이 기록에는 parking 조향, mux /cmd_vel, IMU와 실제 위치가 함께 있습니다. 최대 중앙 조향 약 26.6도에서 mux 출력은 요구 yaw rate를 제한하지 않았지만, 실제 IMU·스캔 회전은 명령 기하의 약 0.38~0.41배였습니다. 따라서 `steering_response_scale: 0.38`을 추가했습니다. 계획 최소 반경은 명령 기하의 0.40m에서 실측 기반 약 1.05m로 커집니다. 실제 명령 상한은 그대로 유지하며, 추종기의 곡률 명령을 응답 비율로 보상합니다. 위치 추정과 기억 지도 설정은 변경하지 않았습니다.

`steering_gain`은 1.0에서 **1.5**로 높였습니다. 기본 LDF는 0.25m, `max_steering_rate`는 0이므로 다음 제어 주기에 계산한 조향을 바로 출력합니다. 소프트웨어 출력 변화와 실제 서보 응답은 다릅니다. 조향 반응 보상 비율과 추종 비율은 로컬 tune의 조향 탭에서 수정할 수 있습니다. 응답 비율을 바꾸면 계획의 실제 최소 회전 반경도 다시 계산합니다.

같은 주차칸의 빈 중심이 여러 곳이면 완성 진입 경로가 있는 중심을 먼저 고릅니다. 기존 중심이 도달 불가능하면 같은 객체들로 지지되는 60cm 이내의 검증된 중심으로 교체할 수 있습니다. 이 경우 일반적인 8cm 저역 보정 한도보다 크게 목표 표시가 움직일 수 있지만, 새 경로는 현재 위치와 연결 검사를 통과해야 합니다. 목표 15cm 이내에서는 이미 비어 있다고 검증된 확정 중심을 유지해 도착 직전 흔들림을 막습니다.

`require_complete_entry_path: true`로 목표까지 이어진 경로부터 parking 제어를 시작합니다. 짧은 부분 경로가 기존 완성 경로를 덮어쓰지 않습니다. parking 경로가 아직 없으면 기존 정책대로 parking 명령을 보내지 않아 mux가 신선한 lane 입력을 사용할 수 있습니다. 탐색 제한 시간은 0.45초, 도착 위치 허용 오차는 7cm입니다.

목표 옆 또는 확정된 경로에서 정지한 상태로 전진 계획이 3회 실패하면 복구를 시도할 수 있습니다. 관측된 후방·실제 지나온 구간을 검사하며, 미관측 구간은 사용자가 사전 확인한 `blind_reverse_distance: 0.30`의 남은 범위에서만 허용합니다. 검출된 장애물은 항상 우선합니다. 0.20m/s 이하, 미션 누적 30cm 및 최대 1회 제한을 유지합니다. 이 한도를 이미 쓴 미션에서 추가 후진하지 않습니다.

이 bag은 미션 중간부터 기록되어 시작 상태와 초기 기억 지도를 완전히 복구할 수 없습니다. 기록된 지도에서 실측 응답·0.15초 지연·0.25초 조향 응답 시간을 넣은 모델로 분석했습니다. 모델은 실차 검증을 대신하지 않으며, 초기 관측이 부족하거나 복구 한도가 소진되면 여전히 정지할 수 있습니다.

확정 목표가 안전하면, 후보 순서나 차량과의 거리만으로 위치를 바꾸지 않고 실제 관측 점과의 여유가 개선될 때 보정합니다. 목표가 그대로이고 이미 받은 완성 경로의 남은 구간 전체가 비어 있으며 차가 경로 근처에 있으면 기존 경로를 유지합니다. 새 목표나 장애물 변경으로 경로 교체가 필요할 때만 연결 검사를 거쳐 바꿉니다. 플랫폼 CPU의 계산 시간 차이로 직선·곡선 모양이 반복해서 바뀌는 현상을 줄입니다.

실제 조향 명령과 측정된 회전으로 이동 궤적을 예측해, 명목 경로가 비어 있어도 차가 코너를 잘라 장애물로 들어갈 때 정지합니다. `control_preview_time: 0.40`과 제동 거리를 사용합니다. 위치 예측 시각에 맞춰 기억 장애물 점도 제어 좌표계로 옮겨 검사합니다. 목표 25cm 근처의 도착 제동은 0.2m/s 회전 하한보다 우선하며, 목표에서 멈추는 거리를 검사 범위에 반영합니다.
