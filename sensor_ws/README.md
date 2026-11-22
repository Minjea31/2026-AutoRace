# sensor_ws

LIMO 차체, 라이다, 카메라를 실행하고 휠·IMU 데이터를 융합해 위치를 추정합니다. ROS 2 Humble 기준입니다.

## 알고리즘

1. **센서 수집**: LIMO 차체에서 휠 오도메트리·IMU·차량 상태를 받고, 카메라와 LiDAR 드라이버가 영상과 스캔을 발행합니다.
2. **휠·IMU 전처리**: 좌표계·시간·유한값을 검사하고, IMU 장착 회전을 적용합니다. 정지 구간의 자이로 평균으로 바이어스를 추정해 제거합니다. 이 준비 상태를 health로 알립니다.
3. **EKF 위치 추정**: `robot_localization`의 확장 칼만 필터로 보정된 휠 전후·좌우 속도와 IMU Z축 각속도를 융합합니다. 현재 설정은 2D, 30 Hz, `parking_odom` 기준이며 EKF의 TF 발행은 꺼져 있습니다. [1]

구현: [sensor_fusion.py](src/autorace_sensor_bringup/scripts/sensor_fusion.py), [fusion_math.py](src/autorace_sensor_bringup/scripts/fusion_math.py), [EKF 설정](src/autorace_sensor_bringup/config/limo_ekf.yaml).

## 입력·출력 토픽

드라이버 출력이 같은 워크스페이스의 전처리·EKF 입력으로도 연결됩니다. 아래 이름은 기본 센서 launch 기준입니다.

| 방향 / 담당 | 토픽 | 타입 | 내용 / 연결 |
| --- | --- | --- | --- |
| 입력 · 차체 | `/cmd_vel` | `geometry_msgs/msg/Twist` | mux가 선택한 차량 속도·각속도 |
| 출력 · 카메라 | `/camera/color/image_raw` | `sensor_msgs/msg/Image` | 원본 컬러 영상 |
| 출력 · 카메라 | `/camera/color/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | YOLO·차선·바닥 ROI 입력 |
| 출력 · LiDAR | `/scan` | `sensor_msgs/msg/LaserScan` | 주차·콘·장애물 회피 입력 |
| 출력 · 차체 → 전처리 | `/odom` | `nav_msgs/msg/Odometry` | 원본 휠 위치·속도 |
| 출력 · 차체 → 전처리 | `/imu` | `sensor_msgs/msg/Imu` | 원본 IMU |
| 출력 · 차체 | `/limo_status` | `limo_msgs/msg/LimoStatus` | 차량 제어 모드 등 |
| 출력 · 전처리 → EKF | `/localization/wheel` | `nav_msgs/msg/Odometry` | 검증한 휠 속도 |
| 출력 · 전처리 → EKF | `/localization/imu` | `sensor_msgs/msg/Imu` | 장착 방향·바이어스 보정 IMU |
| 출력 · 전처리 | `/localization/health` | `std_msgs/msg/String` | 최신성·보정 준비 상태 |
| 출력 · EKF | `/odometry/filtered` | `nav_msgs/msg/Odometry` | 융합 위치·방향 |
| 출력 · TF 발행 노드 | `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage` | 차체·센서 좌표 변환 |
| 출력 · 모델 노드 | `/robot_description` | `std_msgs/msg/String` | 차량 모델, description 활성화 시 |

Depth·IR·포인트클라우드는 선택 실행입니다. 기본 실차 인지 입력은 RGB 압축 영상과 `/scan`입니다.

| 서비스 | 타입 | 역할 |
| --- | --- | --- |
| `/localization/calibrate_gyro` | `std_srvs/srv/Trigger` | 정지 자이로 보정 재시작 |
| `/set_pose` | `robot_localization/srv/SetPose` | EKF 기준 자세 설정 |

## References

- [1] T. Moore, D. Stouch, **A Generalized Extended Kalman Filter Implementation for the Robot Operating System**, IAS-13, 2014. [논문](https://docs.ros.org/kinetic/api/robot_localization/html/_downloads/robot_localization_ias13_revised.pdf). 실제 사용하는 `robot_localization`의 EKF 구현을 설명합니다. 정지 보정·입력 검증은 이 저장소의 별도 전처리입니다.

## 빌드와 실행

시스템 ROS 환경에서 최초 빌드합니다. 다른 센서 launch가 실행 중이면 먼저 종료하세요.

```bash
cd ~/2026-AutoRace/sensor_ws
source /opt/ros/humble/setup.bash
/usr/bin/python3 -m colcon build --symlink-install
source install/setup.bash
export ROS_DOMAIN_ID=94 ROS_LOCALHOST_ONLY=0
ros2 launch autorace_sensor_bringup sensors.launch.py
```

실차의 다른 노드와 RViz도 같은 ROS_DOMAIN_ID를 사용합니다. 도메인 94는 현재 플랫폼 설정입니다.
센서 장치를 열어 사용하는 이전 프로세스가 있으면 중복 실행을 차단합니다.
카메라는 기본 RGB만 실행하며 IR·depth·일반/컬러 포인트클라우드는 끕니다.
필요할 때 `enable_depth:=true`, `enable_ir:=true`, `enable_point_cloud:=true`,
`enable_colored_point_cloud:=true`로 선택합니다. 포인트클라우드는 depth가 켜진 경우에만 실행됩니다.
시작 검사가 끝나면 `[sensor bringup 결과]`에 전체 노드 실행·토픽 수신 결과와 누락 항목을 출력합니다.

## 주요 출력

| 토픽 | 내용 |
| --- | --- |
| `/scan` | 라이다 거리값 |
| `/camera/color/image_raw` | 카메라 원본 컬러 영상 |
| `/camera/color/image_raw/compressed` | 카메라 압축 컬러 영상 |
| `/camera/depth/image_raw` | 깊이 영상 (`enable_depth:=true`일 때만) |
| `/odom`, `/imu` | 차체 드라이버의 원본 오도메트리·IMU |
| `/localization/wheel`, `/localization/imu` | 검증·보정한 EKF 입력 |
| `/odometry/filtered` | EKF 융합 위치, `parking_odom` 기준 |
| `/localization/health` | 센서 최신성·자이로 보정 준비 상태 |

## 위치 추정과 설정

- 원본 `/odom`의 좌표계는 `odom`이며 그대로 유지합니다.
- 정지 자이로 보정이 완료되는 초기 위치·방향을 기준으로 `parking_odom`을 잡고, `odom → parking_odom` 고정 TF를 등록합니다. 시작 시 약 2초간 정지하세요.
- `config/fusion.yaml`에서 휠·IMU 전처리를 설정하고, `config/limo_ekf.yaml`에서 EKF를 설정합니다. 두 파일은 `src/autorace_sensor_bringup/` 아래에 있습니다.
- EKF는 휠 전후·좌우 속도와 IMU Z축 각속도를 사용하며 목표 갱신 주기는 **30Hz**입니다. 센서 자체 입력 주기를 의미하지는 않습니다.
- 라이다 설정은 `config/ydlidar.yaml`이며 각도 범위는 **-110°~110°**입니다. 후방까지 모두 관측하지는 않습니다.
- 드라이버의 조향 명령 보정과 조향각 제한은 기본 OFF입니다. parking이 자신의 출력에만 보정을 적용하므로 센서 launch에서 `steering_command_calibration:=true`를 추가하지 마세요.

상세 장착 좌표와 TF는 [bringup 문서](src/autorace_sensor_bringup/FRAMES.md)를 참고하세요.
