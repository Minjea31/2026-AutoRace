# parking_ws — LiDAR 주차와 RViz 시각화

`/yolo/parking_sign`을 연속 **3회 true**로 받으면 주차 미션을 시작합니다. 주차 완료 후 **시뮬레이션 시간으로 3초간 정지**하고, 실제 진입 궤적을 역순으로 따라 후진 출차합니다. 기준 속도는 **1.0 m/s**이며 곡선과 정지 지점 앞에서는 감속합니다.

새로운 연속 3회 true는 기존 미션의 실패·완료 여부와 관계없이 **이전 경로·이동 추정·실패 상태를 초기화하고 새 시도**를 시작합니다. 실패 후 차량을 직접 처음 위치로 옮긴 다음 다시 3회 발행하면 됩니다. 실패만으로 자동 재시작하지 않습니다. false 또는 0.5초 초과 수신 공백은 연속 횟수만 초기화하며 실행 중인 미션을 취소하지 않습니다. 한 번의 시작/재시도마다 `--times 3`을 사용합니다. true를 계속 발행하면 매 3회마다 재시작합니다. 사용자가 별도의 state 메시지를 보낼 필요는 없습니다.

## 빠른 실행

모든 명령은 저장소 루트(`/home/a/2026-AutoRace`) 기준입니다. 기본 ROS domain은 시뮬레이터와 동일한 42입니다. 아래 스크립트는 시스템 ROS Python을 사용하므로 Conda 활성화나 colcon 빌드 없이 실행할 수 있습니다.

```bash
# 터미널 1: 주차 미션 시작 위치
./simul.sh --mission parking --view parking

# 터미널 2: 주차 노드와 주차 전용 /cmd_vel 어댑터
./parking_ws/run.sh --with-adapter

# 터미널 3: LIMO Pro 모형, 원시 라이다, 목표 화살표, 경로
./parking_ws/rviz.sh
```

**기존 mux가 `/cmd_vel`을 발행하고 있다면 `--with-adapter`를 빼고 `./parking_ws/run.sh`만 실행합니다.** 주차 전용 어댑터와 mux를 동시에 실행하지 않습니다. 어댑터는 기존 `simul`의 변환·명령 만료 검사를 재사용합니다. 시뮬레이터가 `KEYBOARD` 제어라면 K로 ROS 제어로 바꿉니다.

YOLO가 준비되어 있지 않으면 별도 터미널에서 3회 true를 보내 시험할 수 있습니다. 이 명령은 실제 주차 제어를 시작시킵니다.

```bash
source parking_ws/env.sh
ros2 topic pub -r 10 --times 3 /yolo/parking_sign std_msgs/msg/Bool '{data: true}'

# 상태 확인
ros2 topic echo /parking/status

# 주차/출차를 중단하고 표지판 대기로 초기화
ros2 service call /parking/reset std_srvs/srv/Trigger '{}'
```

## 토픽

| 방향 | 토픽 | 타입 | 내용 |
| --- | --- | --- | --- |
| 구독 | `/scan` | `sensor_msgs/msg/LaserScan` | Best Effort 센서 입력 |
| 구독 | `/yolo/parking_sign` | `std_msgs/msg/Bool` | 연속 감지 시작 조건 |
| 발행 | `/parking/obstacles` | `visualization_msgs/msg/MarkerArray` | 장애물 군집별 파란색 박스 테두리 (점·글자 없음) |
| RViz 전용 발행 | `/parking/robot_description` | `std_msgs/msg/String` | LIMO Pro 차체·바퀴 메시를 사용하는 시각화 전용 URDF |
| 발행 | `/parking/target` | `visualization_msgs/msg/MarkerArray` | 선택한 빈 공간과 진입 방향 |
| 발행 | `/parking/path` | `nav_msgs/msg/Path` | 현재 차량 기준 진입 경로, 주차 완료 후에는 역순 후진 출차 경로 |
| 발행 | `/parking/active` | `std_msgs/msg/Bool` | 미션 활성 여부 |
| 발행 | `/parking/status` | `std_msgs/msg/String` | 확인용 JSON 상태·시도 번호·대기 잔여 시간·명령·스캔 정합 오차 |
| 발행 | `/mission/parking/command_03` | `ackermann_msgs/msg/AckermannDriveStamped` | 20 Hz 속도(m/s)·조향각(rad), 좌측 양수 |

RViz 설정의 Fixed Frame은 **`base_link`**입니다. 원시 LaserScan을 함께 보려면 `laser_link → base_link` 정적 TF가 필요하며 시뮬레이터가 제공합니다. 군집·목표·경로는 설정 파일의 센서 장착값으로 `base_link`에 직접 변환하므로 노드가 TF를 구독하지 않습니다. 기본 화면은 원시 라이다 점과 LIMO Pro 모형을 표시하며, 장애물 군집은 파란색 박스 테두리만 표시합니다. 군집별 색상 점·번호·글자와 기존 민트색 상자는 표시하지 않습니다. 활성화 후 경로·주행 명령을 발행합니다. 주차 완료 시 출차 경로를 바로 표시하고 3초 대기 후 후진 명령을 발행합니다. Path의 점 순서는 실제 이동 순서이고 각 pose의 방향은 차량이 바라보는 방향입니다. 출차 완료 후에는 0 속도 명령을 계속 발행합니다. 초기화 후 비활성 상태에서는 명령을 발행하지 않아 mux 제어권을 반환합니다.

차량 모형은 `./parking_ws/rviz.sh`가 함께 시작하는 `parking_robot_visual` 노드에서 제공합니다. `sensor_ws`의 기존 차체·바퀴 메시를 LIMO Pro 치수에 맞춰 재사용하며, 센서 워크스페이스를 빌드하지 않아도 저장소에서 바로 실행할 수 있습니다. 모형의 모든 시각 요소는 `base_link`에 속하고 별도의 TF나 joint state를 발행하지 않습니다. 주차 노드가 꺼져 있어도 모형을 표시합니다. colcon launch도 `rviz:=true`이면 이 시각화 노드를 함께 실행합니다.

## 동작과 범위

1. 유효한 라이다 점을 거리 12 cm 이내의 연결 군집으로 묶습니다. 3점 이상의 군집을 장애물 하나로 처리합니다. 각 군집에는 파란색 박스 테두리만 표시합니다. 군집 ID는 **현재 스캔 안에서만** 유효하며 영구 추적 ID가 아닙니다. 가려진 물체가 여러 군집으로 나뉘거나 붙어 있는 물체가 합쳐질 수 있습니다.
2. 주차 열 방향으로 나란한 차량 크기의 군집 사이에서 차폭+양쪽 여유 폭 이상의 빈 공간을 찾습니다. 벽처럼 긴 군집은 주차 후보에서 제외하지만 모든 라이다 점을 충돌 검사에 사용합니다. 1~2점짜리 작은 장애물도 충돌 검사에서 제외하지 않습니다.
3. 차량 크기, 축간거리, 최대 조향각을 반영한 원호 경로를 탐색합니다. 원호 후보로 찾지 못하면 제한 시간 안에서 hybrid A*를 사용합니다. 차체와 주변 점, 경로의 중간 자세까지 검사하고 0/NaN·가려진 영역을 빈 공간으로 간주하지 않습니다. 단, 후방 마스킹 때문에 **현재 차체 중심 반경 30 cm는 장애물이 없는 초기 공간이라고 가정**합니다. 이 범위에도 실제 관측된 장애물은 충돌 검사에 반영합니다.
4. 칸 안쪽이 가려져 있으면 칸 앞의 관측 가능한 위치까지 먼저 접근하고 다시 계획합니다. 활성화 이후 연속 스캔의 point-to-line ICP로 짧은 구간의 상대 이동을 추정해 목표 위치를 유지합니다. odom/IMU/TF 입력은 없습니다.
5. Pure pursuit로 **전진 진입**합니다. 기준 속도 1.0 m/s, 조향 한계 약 23.4°입니다. 현재 속도에 따른 제동거리까지 충돌을 검사하며, 곡률·회전 속도·정지까지 남은 거리에 따라 감속합니다. 목표 7 cm·방향 오차 약 11.5° 이내이며 추정 속도가 0.03 m/s 미만인 것을 3개의 새 스캔에서 확인하면 `PARKED`로 전환합니다.
6. `PARKED`에서 3초간 정지한 뒤 `EXITING`으로 전환합니다. 중간 접근과 재계획 구간을 포함해 기록한 실제 진입 궤적을 역순으로 따라 후진합니다. 기준 속도 크기는 동일하게 1.0 m/s이며 후진 명령은 음수입니다. 시작 위치 근처에 정지한 것을 새 스캔 3회에서 확인하면 `EXITED`가 됩니다. 후방 마스킹 구간은 저장된 진입 궤적의 차체 영역을 이용해 확인합니다. 현재 관측되는 장애물은 이 기록보다 우선하며, 새 장애물이 있으면 `EXIT_BLOCKED`로 정지하고 출차 경로를 유지합니다. 후방 시야 밖에 새로 들어온 장애물은 검출할 수 없습니다.

상태는 `WAITING_SIGN → WAITING_SCAN → PLANNING → APPROACHING/PARKING → CONFIRMING → PARKED → EXITING → EXIT_CONFIRMING → EXITED`로 진행합니다. `BLOCKED`, `REPLAN`, `REOBSERVE`에서는 정지하고 진입 경로를 다시 찾습니다. 센서가 0.5초 이상 오래되면 정지하고 표시 경로를 지웁니다. 기록한 출차 경로는 보존해 새 스캔이 돌아오면 재검사 후 이어갑니다. 스캔 정합이 실패하면 `TRACKING_LOST`로 정지합니다. 차량을 처음 위치에 둔 뒤 true를 새로 3회 보내 재시도하거나 `/parking/reset`으로 대기 상태로 돌아갈 수 있습니다. 선택 가능한 빈 공간이 없으면 정지 상태를 유지합니다.

현재 구현은 **LiDAR 군집 사이로 전진 주차한 후 진입 궤적을 따라 후진 출차하는 제어**입니다. 설계 문서의 카메라 주차선 인식, 영상으로 실제 칸 중심 보정, 후진 T자 주차는 아직 구현하지 않았습니다. 초기 차량 방향이 주차 열과 나란한 배치를 기본으로 하며, 다른 방향은 `row_yaw`를 조정합니다. 실차 보정·검증은 수행하지 않았습니다.

## 설정·의존성

설정은 [parking.yaml](src/autorace_parking/config/parking.yaml)에 있습니다. `parking_speed: 1.0`, `parking_hold_seconds: 3.0`, `max_deceleration: 1.0`, `max_yaw_rate: 1.0`이 기본입니다. 진입 계획은 `allow_reverse: false`를 유지하며, 후진 출차는 저장한 궤적으로 수행합니다. 센서 설치 위치와 frame 이름, 군집 거리, 주차 열 방향, 차체 치수, 초기 여유 공간 등도 조정할 수 있습니다. 실시간 시계를 쓰려면 `./parking_ws/run.sh -p use_sim_time:=false`로 실행합니다.

ROS 2 Humble, NumPy, `ackermann_msgs`, `visualization_msgs`가 필요합니다. `ackermann_msgs`가 없으면 `sudo apt install ros-humble-ackermann-msgs`로 설치하거나 해당 Debian 패키지를 `parking_ws/.ros-deps`에 로컬 추출할 수 있습니다. `env.sh`가 이 로컬 의존성을 연결합니다. 현재 PC에는 로컬 추출을 준비해 두었으며, `.ros-deps`는 Git에 포함하지 않습니다.

colcon으로 실행하려면:

```bash
source parking_ws/env.sh
cd parking_ws
/usr/bin/python3 -m colcon build --symlink-install
source install/setup.bash
source env.sh
export ROS_DOMAIN_ID=42
ros2 launch autorace_parking parking.launch.py use_sim_time:=true rviz:=true
```

이 launch는 주차 노드와 선택적 RViz를 실행합니다. mux가 없으면 저장소 루트의 별도 터미널에서 `./parking_ws/adapter.sh`도 실행합니다.

## 검증

```bash
source parking_ws/env.sh
ROS_DOMAIN_ID=176 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /usr/bin/python3 -m pytest -q parking_ws/src/autorace_parking/test

# 실제 시뮬레이터 운동·라이다 모델로 주차하고 경로 그림/JSON 저장
/usr/bin/python3 parking_ws/tools/check_parking_sim.py
```

연속 감지/false/수신 공백과 3회 단위 재시도, 군집·노이즈·막힌 공간, 경로의 차체 충돌·곡률, 스캔 정합, 새 장애물 정지, 3초 대기·후진 경로·출차 완료, ROS 명령·시각화 발행·센서 단절·초기화를 검사합니다. 모델 단독 검사에서는 4.3초에 주차, 3.0초 정지 후 후진해 총 11.4초에 출차를 완료했습니다. 충돌 0회, 출차 후 시작 위치와 거리 오차 약 2.9 cm였습니다. 이 배치에서 곡선·제동 감속을 적용한 실제 최고 속도는 약 0.87 m/s였습니다.

실제 `ros2 launch` 주차 노드와 어댑터를 별도 domain 177에서 시뮬레이터 운동·LiDAR 모델에 연결한 검사에서도 3.0초 정지와 출차 Path 발행, 11.8초에 `EXITED`, 충돌 0회, 시작 위치와 거리 오차 약 1.3 cm를 확인했습니다. 결과는 `captures/reverse-exit-ros/report.json`에 있습니다. 기존 `captures/ros-check`는 이전 전진 주차 전용 버전의 기록입니다. 실차 검증은 포함하지 않습니다.
