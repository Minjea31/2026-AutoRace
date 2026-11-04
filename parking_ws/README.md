# parking_ws — LiDAR 주차 실행 안내

`/yolo/parking_sign`을 연속 **3회 true**로 받으면 주차를 시작합니다. **미션 시작 시 차량 오른쪽에 주차 차량과 빈 칸이 있는 배치**를 사용합니다. 직진한 뒤 우회전해 전진 주차하고, 시뮬레이션 시간으로 3초 정지한 다음 **진입할 때 기록한 회전 구간을 그대로 후진**합니다. 회전 시작점 부근에서 멈추며 초기 출발점까지 돌아가지 않습니다.

아래는 **시뮬레이터와 주차 노드를 ros2 launch, 어댑터 창을 ros2 run으로 실행**하는 절차입니다. 장애물 편집과 전체 시뮬레이터 기능은 [simul README](../simul/README.md)를 참고하세요.

## 1. 환경 설정과 최초 빌드

Ubuntu 22.04 / ROS 2 Humble 기준입니다. Conda와 시스템 ROS가 설치되어 있어야 합니다. 시뮬레이터는 Conda `autorace-simul`의 Python 3.10, 주차 패키지는 시스템 `/usr/bin/python3`으로 빌드합니다.

**환경 파일은 각 터미널에서 source로 적용합니다.** `./env.sh`는 부모 터미널의 환경을 바꾸지 못합니다. 모든 명령은 저장소 루트 `~/2026-AutoRace` 기준입니다.

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
cd ~/2026-AutoRace
source simul/env.sh
source simul/install/local_setup.bash
ros2 run limo_parking_sim mission_sim_adapter
~~~

`simul/adapter.sh`와 같은 공용 어댑터 창입니다. **주차 (parking)** 행의 **/mission/parking/command_03을 ON**으로 켜고, 주차 단독 시험에서는 나머지 입력을 OFF로 둡니다. 처음에는 모두 OFF입니다. 창과 실행 터미널을 유지하세요.

어댑터는 주차 노드의 `AckermannDriveStamped`를 시뮬레이터의 `/cmd_vel` (`Twist`)로 변환합니다. 출력 토픽이 `/cmd_vel`인지 확인하고, 같은 토픽을 발행하는 기존 mux나 다른 어댑터는 종료합니다.

창 없이 주차 입력만 연결하려면 위 ros2 run 대신 실행합니다.

~~~bash
ros2 run limo_parking_sim mission_sim_adapter \
  --no-gui --topic /mission/parking/command_03
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

**주행 중 장애물·충돌 판정으로 정지하거나 최종 경로를 폐기하지 않습니다.** 충돌·실패 여부는 사용자가 판단합니다. 실패했다면 아래 순서로 재시도합니다.

1. 터미널 4에서 주차 제어를 대기 상태로 초기화합니다.

   ~~~bash
   ros2 service call /parking/reset std_srvs/srv/Trigger '{}'
   ~~~

2. 시뮬레이터 창에서 **F5**로 차량을 시작 위치로 되돌립니다.
3. 시작 신호를 새로 3회 보냅니다.

   ~~~bash
   ros2 topic pub -r 10 --times 3 /yolo/parking_sign std_msgs/msg/Bool '{data: true}'
   ~~~

새로운 3회 true는 이전 미션의 진행·실패·완료 여부와 관계없이 경로와 위치 추정을 초기화해 새 시도를 시작합니다. 노드를 매번 재시작할 필요는 없습니다. `/parking/reset`은 차량을 이동시키지 않으며, **F5만 눌러서는 노드가 새 미션을 시작하지 않습니다.**

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
| 발행 | `/parking/active` | `std_msgs/msg/Bool` | 미션 활성 여부 |
| 발행 | `/parking/status` | `std_msgs/msg/String` | 상태·시도 번호·명령·추정 오차 JSON |

주차 노드가 `/cmd_vel`을 직접 발행하지는 않습니다. 공용 어댑터가 `/mission/parking/command_03`을 받아 `/cmd_vel`로 전달합니다. `/parking/reset`은 `std_srvs/srv/Trigger` 서비스입니다.

`rviz:=true`일 때 별도 `parking_robot_visual` 노드가 `/parking/robot_description` (`std_msgs/msg/String`)을 발행합니다. RViz의 Fixed Frame은 **base_link**이며, 원시 라이다 표시용 `laser_link → base_link` TF는 시뮬레이터가 제공합니다. 차량 모형은 저장소의 `sensor_ws` 메시를 재사용하므로 해당 워크스페이스를 빌드할 필요는 없습니다.

## 6. 경로와 설정

주차 후보는 **미션 시작 좌표계의 오른쪽(y < 0)에 있는 차량 군집 사이**에서만 찾습니다. 왼쪽에 더 가까운 빈 공간이 보여도 선택하지 않습니다. 회전·이동 후에도 시작 위치와 방향을 기준으로 판단하므로, 오른쪽 주차 열이 현재 차량의 왼쪽에 보이더라도 좌측 주차로 바뀌지 않습니다. 시각화와 위치 추정에는 전체 스캔을 사용합니다.

직진 → 우회전 원호 → 칸 안쪽 직진 경로를 우선 생성합니다. 차체 치수·조향 한계·관측 형상은 **경로 생성 단계**에 반영합니다. 칸이 가려지면 관측된 구간까지만 접근해 다시 계획하고, 최종 진입 경로를 받으면 이를 유지합니다. 이동 추정은 연속 LiDAR 스캔 정합을 사용하며 주차 노드는 odom·IMU·TF를 구독하지 않습니다.

주차 완료 후에는 기록한 진입 궤적을 역순으로 따라갑니다. 칸 안쪽 직진과 회전 구간을 되짚고, 진입 전 방향과 다시 맞는 마지막 기록 자세에서 멈춥니다. 앞서 길게 직진한 구간은 후진하지 않습니다.

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
