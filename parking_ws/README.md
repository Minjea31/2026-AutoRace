# parking_ws

차량 오른쪽의 장애물과 빈 주차칸을 찾고, 경로 생성·추종·주차·출차를 수행합니다.
위치는 sensor_ws의 휠·IMU 융합 결과를 사용하고, 라이다 관측을 기억 지도에 누적합니다.

## 감속 시작 기준 상대 위치

lane이 주차 표지를 연속 2회 확인해 감속을 시작하면 `/parking/approach_start`
(`std_msgs/Header`, 시작 시각)를 발행합니다. fused 모드의 parking 노드는 그 시각에 맞춘
EKF 위치·방향을 기준점으로 저장합니다. EKF는 계속 실행하며 기존 주차 지도·경로·TF는 유지합니다.

`/parking/approach_odometry` (`nav_msgs/Odometry`)는 기준점에 대한 상대 x/y/yaw를
`parking_approach` 좌표계로 출력합니다. 기준점의 위치·방향은 0이며 twist는 기존
base_link 기준을 유지합니다. pose covariance는 고정 기준점 좌표계로 회전합니다.
이 토픽은 별도 관측 출력으로, 기존 경로 제어의 입력 토픽을 대체하지 않습니다. TF는 추가하지 않습니다.

시작 시각을 사이에 둔 EKF 샘플로 위치를 보간하고, 최대 `localization_timeout`(기본 0.3초)까지만 기다립니다.
오래된·미래·초기화 이전 신호는 거부하며, 데이터 누락 시 기준점을 만들지 않고 로그를 남깁니다.
중복 시작 신호는 현재 기준점을 바꾸지 않습니다. `/parking/reset` 또는 기존 시계 역행 초기화 때
기준점을 해제합니다. 신호는 volatile QoS로 발행되므로 수신 노드는 감속 시작 전에 실행해야 합니다.

```bash
ros2 topic echo /parking/approach_odometry
```

## 빌드와 실차 실행

센서와 mux를 먼저 실행합니다. ROS 2 Humble 시스템 환경에서 빌드하세요.

```bash
cd ~/2026-AutoRace/parking_ws
source /opt/ros/humble/setup.bash
source ../sensor_ws/install/setup.bash
/usr/bin/python3 -m colcon build --symlink-install
export ROS_DOMAIN_ID=94 ROS_LOCALHOST_ONLY=0
source real_env.sh
ros2 run autorace_parking parking_node --ros-args \
  --params-file ~/2026-AutoRace/parking_ws/src/autorace_parking/config/parking_real.yaml
```

실차는 `parking_real.yaml`을 사용합니다. bare run과 기본 launch의 설정은 실차 프로필과 다르며, 기본 launch는 시뮬레이션 시간을 사용합니다.
실차 launch를 쓰려면 명시적으로 설정합니다.

```bash
ros2 launch autorace_parking parking.launch.py use_sim_time:=false \
  params_file:=$HOME/2026-AutoRace/parking_ws/src/autorace_parking/config/parking_real.yaml
```

RViz는 같은 ROS 도메인과 환경에서 실행합니다.

```bash
rviz2 -d ~/2026-AutoRace/parking_ws/parking_real.rviz
```

## 동작 흐름

1. `/yolo/parking_sign`의 true를 연속 3회 받으면 시작합니다. 이후 5초의 무시 구간이 지난 뒤 true를 다시 받으면 위치 추정의 미션 기준·기억 지도·목표·경로를 초기화해 새로 시작합니다. 공용 EKF 자체를 초기화하는 것은 아닙니다.
2. 미션 시작 위치에 고정한 ROI에서 후보를 찾습니다. 전방 x는 **0~3m**, 오른쪽 거리는 **0~2m**이며 ROS 좌표 y로는 **-2~0m**입니다.
3. 장애물 배치를 두 객체 사이 또는 6칸 지도와 정합해 빈 공간의 목표를 선택합니다. 칸 영역 전체가 아닌 실제 관측·기억 장애물 점과 차체·여유 공간으로 충돌을 검사합니다.
4. 연결 가능한 경로를 추종해 주차합니다. 차량 중앙을 목표에 맞추고 도착 오차는 뒤축 기준으로 계산합니다.
5. 도착 후 3초 정지하고 기록한 진입 경로를 이용해 후진 출차합니다.

## 후진과 제어권

- 복구 후진은 필요한 조건에서 정지 확인 후 수행합니다. 한 번에 실제 **최소 20cm**, 최대 **30cm**, 속도 **0.2m/s**, 최대 **5회**이며 누적 한도도 검사합니다.
- 후진 충돌 여부는 기억 지도로 판단합니다. 지도에 점이 없는 후방은 빈 공간으로 취급하므로 지도에 없는 실제 장애물은 검출하지 못할 수 있습니다.
- 경로가 없을 때는 주행 명령을 계속 발행하지 않습니다. 복구 후 새 진입 경로가 없으면 정지를 확인하고 제어를 해제하며, mux의 입력 만료 후 다른 유효 미션 또는 lane이 선택됩니다.
- 관측과 재계획은 계속합니다. 연결 가능한 새 진입 경로를 확보하면 parking 제어를 다시 시작합니다.

## 주요 토픽

| 방향 | 토픽 | 내용 |
| --- | --- | --- |
| 입력 | `/yolo/parking_sign` | 주차 시작 신호 |
| 입력 | `/scan` | 라이다 |
| 입력 | `/odometry/filtered`, `/localization/health` | 융합 위치와 준비 상태 |
| 출력 | `/mission/parking/command_03` | mux에 전달하는 주차 명령 |
| 출력 | `/parking/path` | 주차 경로 |
| 출력 | `/parking/lookahead` | 추종 목표점·lookahead 표시 |
| 서비스 | `/parking/reset` | 미션 초기화 |

지도·장애물·목표·ROI 표시는 포함된 RViz 설정에서 함께 확인할 수 있습니다.

## 조향과 설정

경로와 추종기는 실제 중앙 조향각을 기준으로 계산합니다. 발행 직전에만 gain **1.92**, deadband **0.23**, 전송값 상한 **0.35**의 보정을 기존 드라이버 변환에 맞춰 적용합니다.
따라서 명령 토픽의 조향각 숫자는 추종기가 계산한 실제 목표각과 다를 수 있습니다. 드라이버 보정은 OFF여야 하며, 중복 보정이 확인되면 주행을 대기합니다.
조향 변화율 제한은 기본 0으로 즉시 출력합니다.

실차 파라미터는 `src/autorace_parking/config/parking_real.yaml`에서 확인하세요.
분석·검증 도구는 `tools/`, 회귀 테스트는 `src/autorace_parking/test/`에 있습니다.
시뮬레이터 사용법은 [simul 문서](../simul/README.md)를 참고하세요.
