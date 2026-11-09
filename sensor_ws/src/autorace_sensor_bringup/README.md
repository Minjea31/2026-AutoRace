# autorace_sensor_bringup

ROS 2 Humble 센서 통합 패키지. 작업 위치: `/home/wego/2026-AutoRace/sensor_ws/src/autorace_sensor_bringup`.

## 빌드·실행·종료

```bash
source /opt/ros/humble/setup.bash
cd /home/wego/2026-AutoRace/sensor_ws
./build_standalone.sh
source install/setup.bash
ros2 launch autorace_sensor_bringup sensors.launch.py
# 04번 차량: 조향 중립 보정 사용
ros2 launch autorace_sensor_bringup sensors.launch.py vehicle:=04
# 06번 차량: 보정 없음 (인자를 생략한 기본 실행과 동일)
ros2 launch autorace_sensor_bringup sensors.launch.py vehicle:=06
```

종료는 실행 터미널에서 Ctrl+C. 다른 터미널도 같은 순서로 source한다.
기존 teleop_launch.py와 동시에 실행하면 드라이버·TF가 중복된다.
이 패키지는 teleop, navigation, /cmd_vel 발행 노드를 포함하지 않는다.
기존 limo_base는 /cmd_vel을 구독하며 시작 시 commanded mode를 설정하므로 하드웨어 없는 검증에서는 enable_base:=false를 사용한다.

## 실행 인자

| 인자 | 기본값 | 의미 |
|---|---|---|
| vehicle | 06 | 실제 차량 번호. 04: 조향 중립 보정 +0.04 rad, 06: 보정 없음 |
| enable_base | true | limo_base: IMU, Odometry, 상태 |
| enable_lidar | true | YDLIDAR |
| enable_camera | true | Orbbec Astra RGB |
| enable_depth | false | 깊이 스트림 |
| enable_ir | false | 적외선 스트림 |
| enable_point_cloud | false | 일반 포인트클라우드 (depth 활성 시에만) |
| enable_colored_point_cloud | false | 컬러 포인트클라우드 (depth 활성 시에만) |
| enable_description | true | Ackermann 모델·레이저·IMU TF |
| enable_camera_tf | true | 카메라 장착 TF, 카메라 드라이버와 독립 제어 |
| enable_ekf | true | robot_localization EKF |
| port_name | ttylimo | 기존 드라이버가 /dev/ 접두사 추가 |
| lidar_params | 패키지 config/ydlidar.yaml | 파라미터 파일 절대 경로 |
| ekf_params | 패키지 config/limo_ekf.yaml | 파라미터 파일 절대 경로 |
| camera_name | camera | 카메라 토픽 접두사·루트 TF |
| camera_launch | astra_stereo_u3.launch.py | 기존 Orbbec launch 파일명 |
| camera_tf_publish_rate | 0.0 | 카메라 내부 TF를 /tf_static으로 발행. 양수는 동적 발행 Hz |
| degree | 0.0 | 카메라 pitch 보정, TF 단위는 라디안 |
| tf_profile | wego_limo_pro | 공식 기본 장착 위치. existing은 이전 로컬 보정 |
| startup_timeout | 15.0 | 첫 메시지 제한 시간(초) |
| stale_timeout | 5.0 | 수신 중단 제한 시간(초), 확인 간격 1초 |

차량 번호는 IP 주소와 독립적이다. `vehicle:=04`는 limo_base의
`steering_command_offset`을 +0.04 rad로 설정하고, 기본 `06`은 0으로 설정한다.
04의 값은 리모컨으로 실제 직진을 맞춘 구간에서 얻은 초기 시험값이며 실제 바퀴 각도가 아닌
차량 내부 명령 단위이다. ROS 주행 명령의 아커만 조향에만 적용되고 정지 명령은 그대로 정지한다.
중립 보정은 기존 `steering_command_calibration`의 gain/deadband 보정과 별개이다.
04의 보정 후 명령은 `steering_command_limit`(기본 ±0.35 rad) 이내로 제한한다.
06은 기존 명령 변환과 제한 동작을 그대로 사용한다. 조향 피드백 및 오도메트리 계산은 변경하지 않는다.
외부 `autorace_calibration` 실행 파일은 사용하지 않는다. 기존 센서를 종료한 뒤 이 패키지의
launch 하나만 실행한다. 빌드가 필요한 변경이므로 두 차량에서 다음 명령을 먼저 실행한다.

```bash
cd ~/2026-AutoRace/sensor_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --packages-select limo_base autorace_sensor_bringup --symlink-install
source install/setup.bash
```

```bash
# 하드웨어 접근 없이 모델/TF/EKF만 실행
ros2 launch autorace_sensor_bringup sensors.launch.py enable_base:=false enable_lidar:=false enable_camera:=false
# 모든 항목 비활성화
ros2 launch autorace_sensor_bringup sensors.launch.py enable_base:=false enable_lidar:=false enable_camera:=false enable_description:=false enable_camera_tf:=false enable_ekf:=false
```

## 기존 설정과 TF

wego/launch/teleop_launch.py와 관련 드라이버를 분석하고 이전 패키지를 재사용했다.
limo_base 실행 파일, use_mcnamu=false, /cmd_vel 구독 계약을 유지한다.
limo_driver.cpp publishOdometry()의 기존 MODE_ACKERMANN 수정은 그대로 복사하여 이 워크스페이스에서 빌드한다.
EKF 활성화 시 base pub_odom_tf=false, 비활성화 시 true로 odom→base_link 중복을 방지한다.
외부 TF 발행자가 있다면 해당 기능을 비활성화해야 한다. 사용자 지정 EKF 설정도 TF 소유권을 지켜야 한다.

limo_description/load_urdf_ackerman.launch.py가 Ackermann 모델 및 레이저·IMU TF를 발행한다.
기본 TF는 공식 WeGo LIMO Pro 소스의 기본 장착 위치를 사용한다:
base_link→camera_mount (0.103,0,0.06), camera_mount→camera_rotate (pitch=degree),
camera_rotate→camera_link (0.03,-0.05,0).
LiDAR는 base_link 기준 (0.103,0,-0.034), IMU는 (0,0,-0.1)이다.
기존 보정은 `tf_profile:=existing`으로 선택한다.
프레임별 토픽·좌표·출처와 검증 명령은 [FRAMES.md](FRAMES.md)를 참고한다.
Orbbec이 camera_link 아래 내부·광학 TF를 발행한다. camera_name 변경 시 루트 이름도 반영한다.
YDLIDAR YAML은 기존 ydlidar_ros2_driver/params/ydlidar.yaml,
EKF YAML은 robot_localization/params/limo_ekf.yaml과 동일하다.
Orbbec 해상도와 내부 보정은 기존 Astra launch 및 장치 보정을 사용한다.

## 로그

기본 실행에서 base·IMU 전처리·EKF를 허용된 CPU의 마지막 코어에 배치하고,
카메라는 나머지 코어에서 4초 뒤 시작합니다. 카메라 nice는 5로 낮추고 수학 라이브러리
스레드는 1개로 제한합니다. 허용 코어가 4개 미만이면 CPU를 분할하지 않습니다.
OS의 전용 코어 예약이나 전역 클럭 변경은 하지 않으며, EKF 설정의 30Hz는 유지합니다.
추가 실행 인자는 없습니다.

매 실행 ROS launch 고유 로그 폴더를 사용한다(기본 ~/.ros/log/<시간-호스트-PID>).
ROS_LOG_DIR을 실행 전에 지정하면 그 루트 아래 생성한다.
화면의 `Sensor session logs:` 경로를 확인한다.

- session.json: UTC 시작 시각, 실행 인자, base 실행 파일·고정 설정.
- process_events.jsonl: PROCESS_START/PROCESS_EXIT, 프로세스명, PID, 명령, 종료 코드 및 LAUNCH_SHUTDOWN 시각.
- process_output/<프로세스명>.stdout.log 및 .stderr.log: 프로세스별 원본 출력. 빈 스트림도 파일 생성.
- sensor_events.jsonl: WAITING, READY(첫 메시지), NO_DATA(처음부터 수신 없음), STALE(중단), RECOVERED(재수신).
- launch.log 및 ROS 기본 노드 로그.

PROCESS_START는 센서 데이터 수신 성공이 아니다. READY를 별도로 확인한다.
활성 기능에 따라 /imu, /odom, /limo_status, /scan, /camera/color/image_raw,
/odometry/filtered를 best-effort/volatile QoS로 구독한다.
깊이·IR 스트림은 기본 OFF이며 켠 경우에만 해당 image_raw 토픽을 감시한다.
RGB만 사용할 때는 depth·IR 획득과 일반/컬러 포인트클라우드 생성을 실행하지 않는다.
압축 컬러 영상 토픽은 `/camera/color/image_raw/compressed` (`sensor_msgs/msg/CompressedImage`)이며,
image_transport의 compressed 플러그인이 필요하다. 현재 PC에는 설치되어 있다.
센서 모니터는 압축 영상이 아니라 활성화된 원본 영상의 수신을 확인한다.
시작 시 `[sensor bringup 결과]`로 활성 기능의 예상 노드 수와 센서 토픽 수신 결과를 요약한다.
`startup_timeout`까지 준비되지 않으면 미실행 노드·미수신 토픽을 나열하고, 나중에 준비되면 정상 결과를 출력한다.
카메라 component들은 container PID와 출력 파일을 공유한다.
다른 camera_launch를 선택하면 영상 토픽 계약을 확인해야 한다.
LAUNCH_SHUTDOWN은 종료 요청 시각이며 그 이후 PROCESS_EXIT가 기록될 수 있다.
SIGKILL·전원 차단 시 종료 기록은 보장되지 않는다.

```bash
ls -td ~/.ros/log/*/ | head
# 실제 실행 로그 경로를 대입
cat <로그폴더>/session.json
cat <로그폴더>/process_events.jsonl
cat <로그폴더>/sensor_events.jsonl
tail -f <로그폴더>/process_output/*.stderr.log
```

## 의존성

ament_cmake, launch, launch_ros, ros2launch, ament_index_python, rclpy,
rosidl_runtime_py, sensor_msgs, nav_msgs, limo_msgs, limo_base,
ydlidar_ros2_driver, orbbec_camera, limo_description, robot_state_publisher,
joint_state_publisher, xacro, tf2_ros, robot_localization.
package.xml에 의존성을 선언했다. 드라이버·메시지 패키지는 이제 이 sensor_ws 안에서 함께 빌드된다.
USB 권한·udev와 시스템 의존성은 아래 독립 빌드 안내를 참고한다.

## 하드웨어 없는 검증

빌드와 launch 인자 조회, 전체 비활성화 실행 및 모델·TF·EKF 실행을 확인했다.
격리 도메인에서 가상 Odometry만 발행하여 NO_DATA → READY → STALE → RECOVERED,
7개 프로세스의 시작/종료 PID 대응, 종료 코드 및 14개 stdout/stderr 파일 생성을 검증했다.
실제 센서 데이터 수신과 로봇 동작은 이번 검증 범위에 포함하지 않았다.

```bash
# 위 source 순서 적용 후 sensor_ws에서 실행
ROS_DOMAIN_ID=179 ROS_LOCALHOST_ONLY=1 ROS_LOG_DIR="$PWD/log/verification" \
  /usr/bin/python3 src/autorace_sensor_bringup/tests/hardware_free_check.py
```

geometry 없는 visual은 TF 구성 정리 과정에서 제거했다.
기존 laser/imu material 표현에 대한 URDF 경고는 남아 있으나 프레임 로딩은 성공한다.
기존 EKF YAML은 odom0_differential와 odom0_relative가 모두 true여서
EKF가 differential 모드를 선택한다는 오류 수준 메시지를 출력한다.
이 패키지는 기존 보정·설정을 우선 재사용하므로 이 원본 동작을 유지한다.
Ctrl+C 시 종료 코드 -2(SIGINT)가 기록될 수 있다.


## 드라이버 포함 독립 워크스페이스

`wego_ws` source 없이 `/opt/ros/humble`과 이 워크스페이스만 사용한다.
기존 원본은 삭제하지 않았다. 복사한 파일의 원본 경로·SHA256은 `sensor_ws/DRIVER_IMPORT.json`에 기록했다.

- src/limo_ros2/limo_base: 기존 Ackermann 수정 포함, 실행 파일은 limo_base 유지.
- src/limo_ros2/limo_msgs, src/limo_ros2/limo_description: 메시지·모델·센서 TF.
- src/ydlidar_ros2_driver: YDLIDAR ROS 드라이버.
- src/OrbbecSDK_ROS2/orbbec_camera: Astra 드라이버 및 SDK(arm64 포함).
- src/OrbbecSDK_ROS2/orbbec_camera_msgs: 카메라 메시지.
- src/robot_localization: 기존 EKF 구현과 설정.

새 터미널에서 빌드/실행한다. `.bashrc`가 wego_ws를 자동 source하더라도
`./build_standalone.sh`는 깨끗한 환경에서 `/opt/ros/humble`만 source하고 전체 의존 패키지를 빌드한다.
이전에 overlay로 빌드한 build/install 캐시는 재사용하지 않았다.
빌드 후에는 `source /home/wego/2026-AutoRace/sensor_ws/install/setup.bash`와 launch 명령만 필요하다.
`local_setup.bash`를 사용한다면 `/opt/ros/humble/setup.bash`를 먼저 source해야 한다.

ROS 드라이버 소스 이동과 OS 라이브러리 설치는 별개다. 현재 머신의 시스템 의존성:
YDLIDAR SDK 1.2.17 (`/usr/local/lib/libydlidar_sdk.a`, `/usr/local/include`,
`/usr/local/lib/cmake/ydlidar_sdk`), OpenCV, Eigen3, Boost, GeographicLib,
libusb, gflags, glog, nlohmann-json 및 package.xml에 나열된 ROS 패키지.
Orbbec SDK는 소스의 SDK 디렉터리에 포함되며 함께 설치된다.
다른 PC에서는 ROS Humble 설치 후 `rosdep install --from-paths src --ignore-src -r -y`로
선언된 의존성을 설치하고, 별도로 YDLIDAR SDK 및 USB udev 규칙을 준비한다.
기존 규칙 참고 위치: ydlidar_ros2_driver/startup 및 orbbec_camera/scripts/99-obsensor-libusb.rules.
udev 규칙 설치는 시스템 관리자 작업이며 이번 이전에서 시스템 설정은 변경하지 않았다.

로컬 변경은 limo_base/package.xml의 누락된 geometry_msgs, sensor_msgs,
tf2_geometry_msgs, ackermann_msgs 의존성 선언과 LIMO 모델의 TF 프로파일 선택이다.
드라이버 C++와 Ackermann 수정은 유지하며, 센서 장착 TF는 FRAMES.md의 공식 기본값을 적용한다.

독립 빌드 결과: 8 packages finished. 원본의 obsolete header·unused parameter 경고와
일부 패키지가 사용하지 않는 CMake PYTHON_EXECUTABLE 인자 경고는 있으나 빌드 오류는 없다.
빌드 스크립트는 vendor 자체 테스트를 BUILD_TESTING=OFF로 끄며 통합 기능은 별도로 검증한다.

하드웨어 접근 없이 패키지·동적 라이브러리 경로 확인:

```bash
# 새 터미널에서 sensor_ws 기준
source install/setup.bash
/usr/bin/python3 src/autorace_sensor_bringup/tests/verify_independent.py
```

8개 패키지가 모두 sensor_ws/install로 검색되고 LIMO·YDLIDAR·EKF 실행 파일 및
Orbbec 라이브러리의 의존성에 wego_ws 경로와 누락된 라이브러리가 없는지 검사한다.
Orbbec 공유 라이브러리를 로드하되 카메라 노드는 생성하지 않는다.

## RViz에서 로봇 이동과 LiDAR 점 보기

```bash
source /home/wego/2026-AutoRace/sensor_ws/install/setup.bash
# 센서 실행 터미널과 ROS_DOMAIN_ID / ROS_LOCALHOST_ONLY 값을 동일하게 맞춘다.
export ROS_LOCALHOST_ONLY=1
rviz2 -d /home/wego/2026-AutoRace/sensors.rviz
```

제공한 설정은 Fixed Frame=odom, View Target Frame=odom,
LaserScan=/scan, Reliability=Best Effort, Durability=Volatile을 사용한다.
/scan 발행자가 Best Effort이면 RViz를 Reliable로 구독할 경우 데이터가 전달되지 않는다.
변경 후에도 예전 구독 상태가 남으면 LaserScan Display를 제거하고 다시 추가한다.

Decay Time=0.2는 최근 스캔을 표시한다. 이동 중 점을 잠시 누적해서 보려면
LaserScan의 Decay Time을 5~10초로 늘린다. 이는 SLAM 지도가 아니라 odom 기준의 시각화 잔상이다.
정적인 벽은 odom 화면에서 제자리에 남고, 로봇과 새 스캔의 측정 위치가 이동하는 것이 정상이다.
TF·QoS가 정상이어도 ranges가 모두 0이면 표시 가능한 점이 없으므로 데이터도 별도로 확인한다.


RViz 설정의 카메라 창은 `rviz_default_plugins/Image`로 압축 영상만 표시한다.
`Camera` Display는 카메라 모델에 3D 장면을 겹쳐 표시하고 영상 시각의 TF도 요구한다.
단순 영상 확인에는 Image를 사용하면 LiDAR 점이 영상 위에 겹치지 않는다.
현재 Humble RViz는 Transport Hint 속성 대신 토픽 이름의 `/compressed` 접미사에서
전송 방식을 선택하므로, Image의 Topic 값에 `/camera/color/image_raw/compressed`를 직접 입력한다.
타입으로 필터링된 드롭다운 목록에는 이 항목이 빠질 수 있다.

LiDAR는 Color Transformer=Intensity, Channel Name=intensity,
Autocompute Intensity Bounds=true로 저장했다. 기존 /scan의 ranges와 intensities를 그대로 사용한다.

카메라 내부 extrinsics는 고정 보정이므로 통합 launch의 `camera_tf_publish_rate=0.0`은
기존 Orbbec 구현의 /tf_static 경로를 선택한다. 동적 TF 지연 때문에 Camera Display의
메시지 필터 큐가 차는 상황을 줄이며, 내부 보정값이나 영상 타임스탬프를 수정하지 않는다.
이 launch 설정은 센서 launch를 재실행해야 적용된다. RViz 설정 파일 변경만 적용하려면
RViz만 위의 `-d .../sensors.rviz` 명령으로 다시 열면 된다.

## /scan 수신과 유효 거리 구분

`READY: /scan`은 첫 메시지 수신만 의미하며, ranges가 유효하다는 뜻은 아니다.
QoS 불일치는 구독 전달을 막지만 수신된 ranges를 0으로 바꾸지는 않는다.
현재 LiDAR 소스와 설정은 `/home/wego/wego_ws/src/ydlidar_ros2_driver`와 동일하게 복원했다.
추가했던 SCAN_EMPTY/SCAN_RECOVERED 및 sdk_debug 코드는 제거했다.
통합 launch도 기존과 같은 LifecycleNode, namespace=/, emulate_tty=true를 사용한다.
프로세스 출력 저장을 위한 output=both는 유지한다.
기존 wego_ws와 같은 최적화 없는 빌드는 다음 명령으로 수행한다:

```bash
cd /home/wego/2026-AutoRace/sensor_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select ydlidar_ros2_driver autorace_sensor_bringup --symlink-install --cmake-args -DCMAKE_BUILD_TYPE:STRING= -DBUILD_TESTING=OFF
```

`build_standalone.sh`도 마지막에 LiDAR를 기존과 같은 최적화 없는 빌드로 다시 맞춘다.
이전 진단 소스는 log/lidar_diagnostics/restore_wego_baseline에 보관했다.


빈 스캔이 지속될 때 사용자가 정지 상태에서 수동 비교할 수 있는 기존 서비스:

```bash
ros2 service call /stop_scan std_srvs/srv/Empty '{}'
ros2 service call /start_scan std_srvs/srv/Empty '{}'
```

측정이 잠시 중단되므로 LiDAR 데이터에 의존하는 주행 중에는 사용하지 않는다.
2026-10-31 진단에서 같은 PID·설정으로 이 서비스만 실행했을 때,
이전 41개 스캔은 유효점 0개였고 이후 15초의 152개 스캔은 모두 유효점을 포함했다.
RViz 종료만으로는 회복되지 않았다. 이는 스캔 상태 재시작으로 복구됨을 확인한 결과이며,
SDK 버그·장치 펌웨어·전원 중 어느 것이 최초 원인인지는 아직 분리되지 않았다.
자동 재시작이나 측정값 보정은 추가하지 않았다.


LiDAR 단독 수신 시험(실제 LiDAR를 켜므로 다른 드라이버를 먼저 종료):

```bash
source /home/wego/2026-AutoRace/sensor_ws/install/setup.bash
/usr/bin/python3 /home/wego/2026-AutoRace/sensor_ws/src/autorace_sensor_bringup/tests/lidar_isolated_check.py --duration 240
```

같은 포트를 사용하는 프로세스가 있으면 실행을 거부한다. 차체·카메라 노드를 실행하지 않고
격리 도메인 187에서 LiDAR만 켜며, 시간 종료 후 SIGINT로 정상 종료하고 결과를
sensor_ws/log/lidar_diagnostics/<시각>_isolated에 저장한다.

직렬 포트를 열지 않는 오프라인 패킷 분석:

```bash
/usr/bin/python3 src/autorace_sensor_bringup/tests/analyze_lidar_packets.py <시험폴더>/stdout.log
```

SDK 1.2.17의 삼각 프로토콜 + intensity 3바이트 패킷을 체크섬까지 독립 검증한다.
2026-10-31 단독 시험에서도 약 100초 뒤 정상 체크섬 아래 거리·강도가 전부 0으로 바뀌었다.
관련 FINDINGS.md에 관찰 사실과 아직 분리하지 못한 장치·펌웨어·전원 원인을 구분해 기록했다.

과거 원시 패킷 기록은 당시 진단용 실행 파일로 수집한 결과이다. 현재 복원한 기존 드라이버는 sdk_debug 원시 패킷 기록을 지원하지 않는다.
