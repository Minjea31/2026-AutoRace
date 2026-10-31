# LIMO Pro AutoRace 미션 시뮬레이터 · Panda3D / ROS 2

**설치부터 시작하려면 [Conda·pip·시스템 패키지 설치 및 사용 가이드](docs/SETUP.md)를 보세요.** 최초 설치, 기존 환경 사용, 터미널별 실행·센서 수신·직접 제어와 선택 패키지를 순서대로 정리했습니다.

`assets`의 실제 CAD 도면을 추출한 **10 × 6 m 트랙**에서 LIMO Pro로 미션 코드를 개발합니다. `docs/architecture.md`의 **압축 카메라 영상·라이다·차량 제어 토픽**과 `sensor_ws`의 기본 센서 설정을 사용합니다. 주차장 정차 차량 5대, 사용자 그림과 같은 하단 3차선 회피 차량 7대, 두 흰 정지선 사이에 **S자 두 개가 연속되는 통로의 콘 48개**, 미션별 표지판, 방지턱, 신호등·광고판을 배치했습니다. 표지판은 판 포함 전체 높이 20 cm, 원형 판의 지름 9 cm입니다. 가운데 앞쪽 주차 칸은 비워 두었습니다. 점수·미션 성공/실패 판정은 넣지 않았습니다.

![Panda3D 전체 미션 배치](docs/missions-check/overview.png)

[전체 미션 장면 모음](docs/missions-check/all-missions.jpg) · [콘 S자·7대 차량 회피 검증 영상](docs/missions-check/motion.mp4) · [미션별 배치와 조작](docs/MISSIONS.md)

직접 렌더링하며 전진·후진·좌우 조향·충돌 정지·후진 탈출·90° 주차 칸 진입을 확인했습니다. [검증 영상 MP4](docs/render-check/motion.mp4), [움직임 GIF](docs/render-check/motion.gif), [장면 모음](docs/render-check/contact-sheet.jpg), [동작 검증 결과](docs/render-check/report.json)를 볼 수 있습니다. 이 영상의 주차 진입은 시뮬레이터 동작 검증용 정해진 경로이며, 카메라·라이다 기반 자율주차 알고리즘은 본인의 `parking_ws`에서 연결합니다.

## 1. Conda 준비

Ubuntu 22.04 / ROS 2 Humble 기준입니다. **Conda Python 3.10**을 사용합니다. base 환경과 시스템 ROS 설치는 유지합니다. 아래 설치는 사용자가 실행하며, 실행 스크립트는 패키지를 자동 설치하지 않습니다.

```bash
cd /home/a/2026-AutoRace
conda env create -f simul/environment.yml
conda activate autorace-simul
source simul/env.sh
python simul/tools/check_environment.py
```

앞서 안내한 명령으로 환경을 이미 만들었다면 `conda env create`는 생략하고 활성화하면 됩니다. **YAML은 Panda3D의 pip 설치까지 자동으로 처리하므로 기본 실행에 별도 pip 설치는 필요 없습니다.** NumPy·PyYAML·Pillow·setuptools와 검증용 pytest도 포함됩니다. 수동 Conda/pip 설치와 ffmpeg·colcon·Ackermann·맵 생성 도구의 선택 설치는 [설치 가이드](docs/SETUP.md)에 있습니다. 환경 이름은 달라도 되며, 스크립트는 현재 활성화한 `$CONDA_PREFIX/bin/python`을 사용합니다.

`source simul/env.sh`는 시스템 ROS 환경과 Python 경로를 연결하고, 따로 설정하지 않은 경우 `ROS_DOMAIN_ID=42`를 사용합니다. **`ROS_LOCALHOST_ONLY=1`은 항상 적용**하여 같은 PC 안에서 통신합니다. 부모 터미널에 `0`이 설정되어 있어도 `1`로 바꿉니다. **시뮬레이터와 토픽을 받는 터미널마다 같은 환경을 적용**합니다. 실차 드라이버와 시뮬레이터가 같은 토픽에 동시에 발행되지 않도록 실차와 다른 도메인을 사용하세요.

ROS 명령만 쓰는 터미널은 **base 그대로 `./simul/ros2.sh ...`를 사용해도 됩니다.** 이 파일은 시뮬레이터와 같은 `ros_env.sh`를 읽고 시스템 ROS CLI를 실행합니다. 이미 지정한 도메인은 유지하고, localhost 설정은 `1`로 맞춥니다. 시뮬레이터 시작 로그와 화면 왼쪽에 실제 `ROS domain`을 표시하며, `--no-ros`로 실행하면 화면에 `ROS OFF`로 표시합니다. 설정 변경 전에 켜 둔 프로세스에는 새 환경이 적용되지 않으므로 시뮬레이터를 재실행합니다.

## 2. 실행

빌드 없이 바로 실행할 수 있습니다.

```bash
conda activate autorace-simul
cd /home/a/2026-AutoRace
./simul/run.sh
```

기본 시작 위치는 주차 구역에 접근하는 `(x=5.50, y=4.84, yaw=0)`입니다. 단위는 m/rad이고 x축이 차량의 첫 진행 방향입니다.

```bash
# 창 없이 EGL로 렌더링. 카메라·라이다 토픽은 그대로 발행
./simul/run.sh --headless

# ROS 없이 Panda3D와 키보드만 사용
./simul/run.sh --no-ros

# 전체 트랙 보기
./simul/run.sh --mission track --view overview

# 미션 시작 위치에서 시작. view follow는 차량을 따라가는 관찰 화면
./simul/run.sh --mission cone --view follow
./simul/run.sh --mission obstacle --view follow
./simul/run.sh --mission fork --view follow
./simul/run.sh --mission signal --view follow

# 시작 위치 변경
./simul/run.sh --spawn 5.8 4.84 0

# 3초 구동 후 화면/RGB/깊이 PNG 저장
./simul/run.sh --headless --duration 3 --snapshot simul/captures/check
```

사용자 그림의 색상 구역·격자·번호·붉은 S 경로선은 표시하지 않습니다. 원래 흰색 차선과 실제 시설물만 배치합니다. 주차 목표의 초록색 테두리도 기본 숨김입니다. 미션별 표지판은 흰색 기준선 옆에 배치했으며, 정차 차량·콘·표지판 지주·신호등·광고판은 카메라와 라이다에 함께 반영됩니다. 노면의 한글·횡단보도·마름모는 서로 뭉개지지 않게 별도로 복원했습니다.

콘 코스는 **오른쪽→왼쪽→오른쪽→왼쪽으로 휘는 S자 두 개**가 연속됩니다. 콘 줄의 좌우 이동 폭 **40 cm**를 유지하고, 오른쪽을 포함한 양쪽 흰 차선 밖까지 통로를 넓혔습니다. 두 정지선의 위치는 유지하며, 콘은 양쪽 24개씩 총 **48개**로 촘촘히 배치했습니다. LIMO Pro가 콘 사이의 여유 공간을 사용해 조향 한계 안에서 통과할 수 있는지도 검사합니다.

| 키 | 동작 |
| --- | --- |
| K | 키보드 제어 켜기/끄기. 기본은 ROS 제어 |
| W 또는 ↑ | 전진, 0.25 m/s |
| ↓ | 후진, 0.25 m/s |
| A / D 또는 ← / → | 좌 / 우 조향 |
| Space | 정지 |
| 마우스 왼쪽 드래그 | 관찰 화면 위치 이동 |
| 마우스 오른쪽 드래그 | 바라보는 지점을 중심으로 화면 회전 |
| 마우스 휠 | 확대 / 축소 |
| 1 / 2 / 3 / 4 | 주차장 / 전체 트랙 / 차량 추적 / 전방 시점 |
| L / R | 갈림길의 왼쪽 A / 오른쪽 B 표지판 표시. 차량 위치 유지 |
| S | 라이다 광선 표시 / 숨김. 기본 숨김 |
| V | 오른쪽 위 카메라 영상 표시 / 숨김. 기본 숨김 |
| P | 일시정지 / 계속 |
| T / G / Y | 신호등 적색 / 녹색 / 황색 |
| F5 | 선택한 미션의 시작 위치로 차량 리셋 |
| Esc | 종료 |

키보드 제어를 켠 동안에는 ROS 명령을 적용하지 않습니다. 주차 노드로 제어할 때 K를 끄세요. 키보드 제어는 시뮬레이터 내부에서 처리합니다.

`R`/`L`은 실제 3D 갈림길 표지판의 그림을 바꾸며, 카메라 토픽에도 바뀐 화살표가 보입니다. `S`와 후진 키가 겹치지 않도록 후진은 `↓`, 기존 리셋은 `F5`로 옮겼습니다. `T/G/Y`도 실제 신호등의 켜진 색을 바꿉니다. 인지 Bool은 사용자 YOLO 노드에서 계산합니다.

마우스는 관찰용 카메라를 움직입니다. 드래그나 휠을 사용하면 자유 시점으로 바뀌고, `1`~`4`를 누르면 해당 기본 시점으로 돌아갑니다. 차량의 센서 장착 위치·차량 자세·ROS 카메라 영상은 마우스로 바뀌지 않습니다. 라이다 광선과 카메라 영상 표시를 숨겨도 `/scan`과 카메라 토픽은 계속 발행합니다.

[마우스 이동·회전·확대 화면 예시](docs/mouse-controls.jpg)

## 3. 실제 제공하는 토픽

| 방향 | 토픽 | ROS 2 타입 | 기본 주기 / 내용 |
| --- | --- | --- | --- |
| 구독 | `/cmd_vel` | `geometry_msgs/msg/Twist` | `linear.x` m/s, `angular.z` rad/s |
| 발행 | **`/camera/color/image_raw/compressed`** | **`sensor_msgs/msg/CompressedImage`** | **640×480, JPEG, 30 Hz. docs의 미션 입력** |
| 발행 | **`/scan`** | **`sensor_msgs/msg/LaserScan`** | **10 Hz. docs의 미션 입력** |
| 발행 | `/camera/color/image_raw` | `sensor_msgs/msg/Image` | 640×480, `rgb8`, 30 Hz |
| 발행 | `/camera/color/camera_info` | `sensor_msgs/msg/CameraInfo` | RGB와 같은 시각·해상도 |
| 발행 | `/camera/depth/image_raw` | `sensor_msgs/msg/Image` | 640×400, `16UC1`, mm, 30 Hz. 무효 값 0 |
| 발행 | `/camera/depth/camera_info` | `sensor_msgs/msg/CameraInfo` | Depth와 같은 시각·해상도 |
| 발행 | `/imu` | `sensor_msgs/msg/Imu` | 50 Hz, 이상적 가속도·각속도·방향 |
| 발행 | `/odom` | `nav_msgs/msg/Odometry` | 50 Hz, 시작 위치 기준 |
| 발행 | `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage` | 차체·센서·광학 좌표계 |
| 발행 | `/clock` | `rosgraph_msgs/msg/Clock` | 120 Hz, 시뮬레이션 시각 |
| 발행 | `/sim/ground_truth` | `nav_msgs/msg/Odometry` | 50 Hz, 지도 기준 실제 시뮬레이션 위치. 디버깅용 |
| 발행 | `/sim/parking_status` | `std_msgs/msg/String` | 5 Hz, 위치·속도·충돌·명령 만료 JSON. 점수 없음 |

센서 주기는 **시뮬레이션 시간 기준**입니다. GPU/CPU 성능이 부족하면 실제 시간에서는 느려질 수 있고, `/clock`도 함께 느려집니다. 카메라는 Reliable, 라이다는 Best Effort의 SensorDataQoS로 발행하여 현재 Orbbec/YDLIDAR 기본값에 맞췄습니다. 구독자는 `qos_profile_sensor_data`를 사용하면 두 종류 모두 받을 수 있습니다.

`/mission/parking/command_03`, `/yolo/parking_sign`은 각각 **주차 노드와 YOLO 노드가 발행**하는 토픽입니다. 센서 시뮬레이터가 인지 결과를 대신 만들지 않습니다. `/odometry/filtered`는 EKF 출력이므로 필요하면 기존 EKF를 `use_sim_time:=true`로 별도 실행합니다. `/limo_status`는 실차 전용 커스텀 상태 메시지이므로 여기서는 합성하지 않습니다.

## 4. 토픽 수신 확인

시뮬레이터를 ROS 모드로 실행해 둔 뒤, **새 터미널에서 아래 명령을 실행**합니다. 이 터미널에는 Conda 활성화가 필요 없습니다.

```bash
cd /home/a/2026-AutoRace
./simul/ros2.sh topic list -t
./simul/ros2.sh topic echo /scan --once --field ranges --qos-reliability best_effort
./simul/ros2.sh topic echo /camera/color/image_raw/compressed --once --field header
./simul/ros2.sh topic hz /camera/color/image_raw/compressed
```

일반 `ros2` 명령이나 본인의 노드를 실행할 터미널에서는:

```bash
conda activate autorace-simul
cd /home/a/2026-AutoRace
source simul/env.sh

ros2 topic list
ros2 topic hz /camera/color/image_raw/compressed
ros2 topic echo /scan --qos-reliability best_effort --once
```

**센서 토픽이 안 보일 때:** 실행 중인 시뮬레이터의 시작 로그 또는 화면에서 도메인을 확인하고 터미널의 `echo ${ROS_DOMAIN_ID:-0}`과 비교하세요. 서로 다른 도메인은 연결되지 않습니다. 실제 사용자 세션에서도 시뮬레이터가 도메인 42, base의 `ros2 topic pub`는 도메인 0이어서 센서 토픽과 제어가 연결되지 않았습니다. 두 터미널의 `ROS_LOCALHOST_ONLY` 값도 `1`로 맞추고, 이전에 `0`으로 켠 시뮬레이터는 재실행하세요. 이미 지정한 `ROS_DOMAIN_ID`는 `env.sh`가 덮어쓰지 않습니다.

일반 ROS 터미널에서도 같은 로컬 설정을 사용하려면:

```bash
source simul/ros_env.sh
export ROS_DOMAIN_ID=42
ros2 topic list --no-daemon --spin-time 2 -t
```

`--no-ros` 실행은 토픽을 발행하지 않습니다. `V`로 카메라 미리보기를 숨기거나 `S`로 라이다 광선을 숨기는 것은 센서 발행에 영향을 주지 않습니다. 토픽은 보이지만 `/scan` 값이 안 들어오면 위의 Best Effort 옵션을 사용하세요.

압축 영상은 Python에서 다음처럼 받습니다. OpenCV를 이미 쓰는 노드라면 `cv2.imdecode`로 디코딩해도 됩니다.

```python
import io
import rclpy
from PIL import Image
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, LaserScan

class ParkingInputs(Node):
    def __init__(self):
        super().__init__('parking_inputs')
        self.create_subscription(CompressedImage,
            '/camera/color/image_raw/compressed', self.image, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan', self.scan, qos_profile_sensor_data)

    def image(self, msg):
        rgb = Image.open(io.BytesIO(bytes(msg.data))).convert('RGB')
        # 영상과 원본 header.stamp를 주차 알고리즘에 전달

    def scan(self, msg):
        # i번 거리의 각도 = msg.angle_min + i * msg.angle_increment
        # 현재 설정에서는 0이 무효 거리. inf가 들어오는 설정도 함께 처리
        pass

rclpy.init()
node = ParkingInputs()
rclpy.spin(node)
```

RViz에서는 Fixed Frame을 `map`, LaserScan 토픽을 `/scan` 및 Best Effort, Image 토픽을 `/camera/color/image_raw`로 설정합니다.

```bash
rviz2 --ros-args -p use_sim_time:=true
```

실차 QoS에 맞춘 기본 화면으로 바로 실행하려면 `./simul/rviz.sh`를 사용합니다. `/scan` Best Effort와 카메라 Reliable 설정이 저장되어 있습니다. 토픽별 큐 깊이와 실제 드라이버 근거는 [센서 기본 QoS](docs/SENSOR_DEFAULTS.md)를 참고하세요.

## 5. 내 주차 코드 연결

현재 `docs`의 인터페이스를 그대로 사용합니다.

```text
시뮬레이터 ─ /camera/color/image_raw/compressed ─> parking_ws / yolo_ws
시뮬레이터 ─ /scan ──────────────────────────> parking_ws
yolo_ws ─── /yolo/parking_sign (Bool) ────────> parking_ws
parking_ws ─ /mission/parking/command_03
             (AckermannDriveStamped) ────────> mux_ws
mux_ws ──── /cmd_vel (Twist) ────────────────> 시뮬레이터
```

미션·YOLO·mux를 실행할 터미널에서도 `source simul/env.sh`를 적용하고 **각 노드의 `use_sim_time=true`**를 설정합니다. 이때 실제 센서 launch는 실행하지 않습니다. `sensor_ws` 코드는 설정 참고 대상으로 사용했고 수정하지 않았습니다.

`/cmd_vel`의 `angular.z`는 조향각이 아닙니다. 문서대로 `v=drive.speed`, `w=v*tan(drive.steering_angle)/0.2`로 변환합니다. 속도가 0이면 제자리 회전하지 않습니다. 후진도 지원합니다. 명령이 0.5초 동안 오지 않으면 정지합니다.

센서와 무관하게 임의의 `/cmd_vel`을 보내려면 아래 명령을 실행합니다. 화면의 `control: ROS /cmd_vel`을 확인하세요. `KEYBOARD`라면 **K를 한 번 눌러 끄고**, `PAUSED`라면 P로 재개합니다. 실행 중인 다른 제어 노드가 있다면 먼저 그 노드의 발행을 중단하세요.

```bash
cd /home/a/2026-AutoRace
# 전진. Ctrl+C로 발행을 중단하면 watchdog 정지
./simul/ros2.sh topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.2}, angular: {z: 0.0}}'

# 후진
./simul/ros2.sh topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: -0.15}, angular: {z: 0.0}}'

# 좌회전하면서 전진. angular.z를 음수로 바꾸면 우회전
./simul/ros2.sh topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.2}, angular: {z: 0.3}}'

# 기존 발행 명령을 Ctrl+C로 끝낸 뒤 정지 명령
./simul/ros2.sh topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.0}, angular: {z: 0.0}}'
```

`-r 20`은 초당 20회 발행합니다. 움직임 명령을 `--once`로 한 번만 보내면 0.5초의 **시뮬레이션 시간** 뒤 watchdog으로 정지하므로 계속 주행하려면 반복 발행해야 합니다. `angular.z`는 rad/s 단위의 회전 속도이며, LIMO Pro는 속도 0에서 제자리 회전을 하지 않습니다.

실제 ROS 송수신과 CLI 제어를 별도 도메인에서 다시 검사하려면 시뮬레이터용 Conda 터미널에서 실행합니다. 이 검사는 새 오프스크린 시뮬레이터를 따로 실행하며, 현재 화면의 차량에는 명령을 보내지 않습니다.

```bash
conda activate autorace-simul
source simul/env.sh
ROS_DOMAIN_ID=144 ROS_LOCALHOST_ONLY=1 python simul/tools/check_ros_topics.py --cli-command
```

**mux_ws 구현 전의 주차 단독 연결:** 선택적으로 `parking_adapter`를 실행할 수 있습니다. 이 어댑터는 `/mission/parking/command_03`만 받아 30 Hz의 `/cmd_vel`로 바꾸며, 헤더와 수신 시각의 0.25초 만료를 검사합니다. 전체 미션 우선순위 mux는 아닙니다. 기존 mux와 동시에 실행하지 않습니다.

```bash
# 선택 기능. ackermann_msgs가 없다면 사용자가 설치
sudo apt install ros-humble-ackermann-msgs

# 별도 터미널: conda 활성화와 source simul/env.sh 후
python -m limo_parking_sim.parking_adapter
```

## 6. 센서와 치수 변경

파일: [차량·센서·토픽 설정](src/limo_parking_sim/config/limo_pro.yaml), [주차장 설정](src/limo_parking_sim/config/parking_map.yaml).

| 항목 | 반영한 값 / 근거 |
| --- | --- |
| 트랙 | 10,000×6,000 mm. PDF CAD 경로 276개를 추출. 그림의 픽셀을 눈대중으로 배치한 맵이 아님 |
| 주차장 | 전체 1,580×1,240 mm. 3열×2행, 내부 구획 500×600 mm, 선 20 mm |
| 차량 외곽 | 322×220×251 mm, 축간 200 mm, 제조사 윤거 175 mm |
| 조향 한계 | `sensor_ws/limo_driver.h`의 안쪽 바퀴 28°와 변환 윤거 172 mm 반영. 차체 중심 조향 한계 약 23.4° |
| 라이다 장착 | base_link 기준 `(0.103,0,-0.034)`. 지면에서 116 mm |
| 카메라 장착 | mount `(0.103,0,0.06)` + link `(0.03,-0.05,0)`. pitch=0에서 지면 기준 `(0.133,-0.05,0.210)` |
| 카메라 해상도·주기 | 현재 `astra_stereo_u3.launch.py`의 RGB 640×480, Depth 640×400, 각 30 Hz |
| 라이다 출력 | 현재 YAML과 동일한 −110°~+110°, 10 Hz, 약 245샘플. 물리적 최대 감지 12 m |

라이다의 메시지 `range_min/max=0.01/64.0`은 기존 드라이버 설정을 유지하지만, 실제 광선은 T-mini Pro의 `0.02~12 m`로 제한합니다. 전방 마스킹을 없애려면 `angle_min_deg=-180`, `angle_max_deg=180`으로 변경합니다. `inverted`는 드라이버가 원시 회전 순서를 ROS 각도로 정리하는 옵션이므로, 이미 ROS 좌표로 계산하는 시뮬레이션에서 다시 반전하지 않습니다.

카메라와 라이다는 같은 3D 배치와 차량 위치를 참조합니다. RGB는 Panda3D 렌더링, Depth는 GPU Z-buffer를 **광학 z축 거리 mm**로 환산한 값, 라이다는 센서 높이의 장애물 첫 교차 거리입니다. 차선은 반사 벽이 아니며, 실제 도면에 없는 외곽 벽도 라이다에 만들지 않았습니다. 시뮬레이션 안전 범위상 매트 밖으로 차량이 나가면 멈춥니다.

### 실차와 완전히 같게 하려면 남는 보정

카메라 **장치별 K/D·RGB/Depth 외부 보정은 웹 사양에 없습니다.** 기본 모델은 명목 pinhole 카메라이며, RGB 71°×43.7°는 제조사가 1920×1080 모드에 제공한 값입니다. 현재 640×480 모드의 정확한 화각을 확인한 값은 아닙니다. 내부 센서 오프셋은 기본 0으로 두고, 실제 토픽과 구별해 설정에 표시했습니다.

실차에서 다음 정보를 읽어 보정합니다.

```bash
ros2 topic echo /camera/color/camera_info --once
ros2 topic echo /camera/depth/camera_info --once
ros2 run tf2_ros tf2_echo camera_link camera_color_frame
ros2 run tf2_ros tf2_echo camera_link camera_depth_frame
```

각 카메라의 `intrinsics: [fx, fy, cx, cy]`, `xyz`, `rpy`를 설정하면 렌더링 투영과 CameraInfo/TF에 함께 적용됩니다. 렌즈 왜곡은 재현하지 않으므로 시뮬레이터의 D는 0입니다. 실제 raw 영상의 왜곡 보정은 실차 쪽에서 필요합니다. Depth는 등록되지 않은 별도 카메라이며 RGB에 정렬된 영상이 아닙니다.

차체·바퀴 외형은 `sensor_ws/limo_description`의 LIMO 메시를 가볍게 변환하고 LIMO Pro 외곽·축간·윤거에 맞췄습니다. 메시의 센서·스크린 세부 외형은 기존 자산을 따르며, 장치별 Pro 장착 상태를 새로 실측한 모델은 아닙니다. 재생성 도구는 `tools/build_limo_model.py`, 원본 해시와 변환 정보는 `assets/limo_pro_model.json`에 있습니다. 운동은 기존 드라이버의 속도·곡률 적분을 따르는 평면 Ackermann 모델이며, 서스펜션·타이어 미끄럼·모터 지연·충돌 후 물체 이동·실제 카메라 노출·구조광 결측은 모델링하지 않습니다. 가속도·조향 속도·장애물 세부 위치·표지판 위치는 자료에서 확정할 수 없어 조정 가능한 가정값으로 분리했습니다. 점수 판정은 없습니다.

## 7. ROS 패키지로 빌드하기 (선택)

바로 실행하는 `run.sh`에는 colcon 설치가 필요 없습니다. `ros2 launch`를 쓰려면 활성 Conda 환경에 colcon도 설치하고 **해당 환경의 Python으로 빌드**합니다.

```bash
conda activate autorace-simul
python -m pip install colcon-common-extensions
cd /home/a/2026-AutoRace
source simul/env.sh
cd simul
python -m colcon build --symlink-install
source install/setup.bash
ros2 launch limo_parking_sim parking.launch.py

# GUI 없이 실행
ros2 launch limo_parking_sim parking.launch.py headless:=true

# 주차 단독 어댑터도 실행할 경우에만
ros2 launch limo_parking_sim parking.launch.py parking_adapter:=true
```

기존에 시스템 Python으로 빌드한 결과가 있다면 별도 빌드 경로를 사용하세요. 실행 파일의 Python 경로가 빌드 당시 환경을 따릅니다.

## 8. 검증과 문제 해결

```bash
conda activate autorace-simul
cd /home/a/2026-AutoRace
source simul/env.sh

# 모델·토픽 계약·오프스크린 RGB/깊이 검증
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q simul/src/limo_parking_sim/test

# 시뮬레이터를 자체 실행하고 실제 DDS 메시지·제어·서비스 검증 후 종료
# 별도로 띄운 시뮬레이터가 없는 도메인 144에서 실행
ROS_DOMAIN_ID=144 python simul/tools/check_ros_topics.py

# 전진/후진/좌우 조향/충돌/90도 진입을 렌더링하고 영상·결과 다시 만들기
python simul/tools/render_validation.py --output simul/captures/motion-check

# S자 콘 및 7대 장애물 구간의 실제 운동·충돌·렌더링, L/R·신호등 카메라 검증
python simul/tools/render_missions.py --output simul/captures/missions-check

# 리셋 / 일시정지 / 계속
ros2 service call /sim/reset std_srvs/srv/Empty '{}'
ros2 service call /sim/pause std_srvs/srv/SetBool '{data: true}'
ros2 service call /sim/pause std_srvs/srv/SetBool '{data: false}'
```

`No module named rclpy`이면 Python 3.10 Conda 환경인지와 `source simul/env.sh` 적용 여부를 확인합니다. `_rclpy_pybind11` 또는 GLIBCXX 오류는 Conda와 시스템 ROS의 ABI/라이브러리 경로를 확인해야 합니다. `check_environment.py`가 읽기 전용으로 모듈 상태를 출력합니다.

검증 명령의 `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`은 이 검사와 무관한 시스템 ROS pytest 플러그인을 자동으로 불러오지 않게 합니다. Conda에 `lark` 같은 ROS launch 테스트 전용 패키지를 추가 설치할 필요가 없습니다.

창을 열 수 없으면 `--headless`를 사용합니다. EGL OpenGL 또는 Mesa 소프트웨어 렌더러가 필요합니다. 화면 없이도 실제 센서 렌더링은 계속하므로 카메라를 더미 이미지로 대체하지 않습니다. `eglGraphicsPipe` 뒤에 GPU/Mesa 렌더러가 잡혔다면 기본 EGL display 경고가 있더라도 동작할 수 있습니다.

토픽은 보이는데 `/scan`이 안 들어오면 구독 QoS를 Best Effort로 바꾸세요. 현재 실행 스크립트는 사용자 요청에 따라 `ROS_LOCALHOST_ONLY=1`로 고정하며, 같은 PC의 노드와 통신합니다.

리셋은 차량·속도·충돌 상태를 초기화하고 `/clock`은 계속 증가시킵니다. 깊이 무효 픽셀은 0, 기본 라이다 무효 거리도 0입니다. 토픽명·frame·장착 좌표의 근거와 검증 범위는 [SOURCES.md](docs/SOURCES.md), [VALIDATION.md](docs/VALIDATION.md)를 참고하세요.
