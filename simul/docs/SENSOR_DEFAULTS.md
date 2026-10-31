# 실차 센서의 기본 QoS와 시뮬레이터

기준은 현재 저장소의 `sensor_ws` 소스다. 모든 항목은 Keep Last / Volatile을 사용한다.

| 토픽 | 방향 | Reliability | 큐 깊이 | 실제 코드 근거 |
| --- | --- | --- | --- | --- |
| `/scan` | 발행 | Best Effort | 5 | `ydlidar_ros2_driver_node.cpp`: SensorDataQoS |
| 카메라 RGB·Depth·CameraInfo | 발행 | Reliable | 10 | `astra_stereo_u3.launch.py`: default → `utils.cpp`: rmw_qos_profile_default |
| `/odom` | 발행 | Reliable | 50 | `limo_driver.cpp` |
| `/imu` | 발행 | Reliable | 10 | `limo_driver.cpp` |
| `/cmd_vel` | 구독 | Reliable | 10 | `limo_driver.cpp` |

시뮬레이터의 압축 카메라 영상도 RGB와 같은 Reliable / 깊이 10을 사용한다. 라이다 frame `laser_link`, −110~110도, 10 Hz, 메시지 거리 범위 0.01~64 m, 무효 값 0은 `autorace_sensor_bringup/config/ydlidar.yaml`과 맞춘다. 카메라는 RGB 640×480, Depth 640×400, 각 30 Hz이며 기존 설정의 명목 광학 모델을 사용한다. 장치별 카메라 보정과 모터·노출 등 물리 재현 범위는 [SOURCES.md](SOURCES.md)에 설명한다.

## RViz 기본값

`simul/config/sensors.rviz`는 `/scan`을 Best Effort / 깊이 5로 구독하고 RGB를 Reliable / 깊이 10으로 구독한다. 시뮬레이터용 실행 명령은 다음과 같다.

```bash
cd ~/2026-AutoRace
./simul/rviz.sh
```

이 스크립트는 `ros_env.sh`로 도메인·localhost 설정을 적용하고 `use_sim_time=true`로 RViz를 실행한다. 기본 화면에는 Grid·TF·LaserScan·RGB가 있다. 시뮬레이터가 제공하는 센서·TF로 화면을 구성한다.

현재 사용자 PC의 `~/.rviz2/default.rviz`에도 같은 화면 설정을 저장했다. 이후 일반 `rviz2`로 실행해도 `/scan`의 기본 구독 QoS는 Best Effort다. 실차 RViz에서는 해당 실차의 ROS 도메인을 사용하고 `use_sim_time=false`로 실행한다.

## RELIABILITY 경고의 의미

2026-11-01 실제 DDS 그래프에서 발행자 `limo_parking_sim`은 Best Effort, 구독자 `rviz`는 Reliable을 사용하고 있음을 확인했다. 이 조합에서는 RViz에 스캔이 전달되지 않는다. 실차 YDLIDAR 발행자도 Best Effort이므로 RViz 구독 설정을 맞춘다.

기본 파일 적용 후 시뮬레이터와 RViz를 재시작했다. 적용 직후 `ros2 topic info /scan --verbose`에서 발행자 `limo_parking_sim`과 구독자 `rviz`가 모두 Best Effort임을 확인했다. RViz의 `use_sim_time=true`, 실제 `/scan`·RGB 메시지 수신과 재시작 로그의 QoS 경고 없음도 확인했다. 별도 도메인 144의 CLI 주행·카메라·Depth·라이다·IMU·odom·TF·정지/리셋 검사는 [PASS](ros-cli-check.json)였다. RViz를 닫으면 구독자 수가 0이 되는 것은 정상이다.

자신의 Python 구독자도 다음 기본 프로필을 사용한다.

```python
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

self.create_subscription(LaserScan, '/scan', self.scan_callback,
                         qos_profile_sensor_data)
```

설정 확인과 메시지 수신:

```bash
cd ~/2026-AutoRace
./simul/ros2.sh topic info /scan --verbose
./simul/ros2.sh topic echo /scan --once --field header --qos-reliability best_effort
```

RViz에 LaserScan 화면을 새로 추가할 때도 Topic 아래 Reliability Policy를 Best Effort로 선택한다. 기본 파일을 수정한 뒤에는 저장해야 다음 실행에도 유지된다.
