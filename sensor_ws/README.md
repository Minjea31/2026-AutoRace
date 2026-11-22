# sensor_ws

LIMO 차체, 라이다, 카메라를 실행하고 휠·IMU 데이터를 융합해 위치를 추정합니다. ROS 2 Humble 기준입니다.

## 알고리즘

정지 구간에서 자이로 바이어스를 구해 IMU를 보정합니다. 보정된 휠 속도와 IMU 각속도를 EKF로 융합해 차량 위치·방향을 추정합니다. 카메라와 LiDAR 드라이버는 영상·스캔을 별도로 발행합니다. [1]

![sensor_ws 알고리즘 개념 그림](../docs/logic/sensor_ws.svg)

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
