# Conda 설치부터 ROS 2 제어까지

이 문서는 `~/2026-AutoRace`에서 Bash를 사용하는 경우를 기준으로 합니다. 검증 환경은 **Ubuntu 22.04 / ROS 2 Humble / Conda Python 3.10**입니다. Conda와 시스템 ROS 설치 후에는 `source simul/env.sh`가 Conda 환경 준비를 자동 수행합니다.

현재 사용자 PC에는 `autorace-simul`과 ROS 2 Humble이 이미 있고 필수 모듈 검사도 통과했습니다. **이 PC에서 바로 사용할 때는 3번부터 시작**하면 됩니다.

## 1. 시스템 준비 — 처음 설치하는 PC만

Conda와 시스템 ROS 2를 사용합니다. 시뮬레이터의 Python 패키지는 `autorace-simul`에 설치하고, ROS 라이브러리는 `/opt/ros/humble`에서 가져옵니다.

```bash
conda --version
test -f /opt/ros/humble/setup.bash && echo 'ROS 2 Humble 설치됨'
```

ROS 2가 없는 새 Ubuntu 22.04 PC는 [ROS 2 Humble 공식 설치 안내](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)의 로케일·저장소 설정부터 완료합니다. 공식 안내에는 Desktop 및 ROS Base 설치가 구분되어 있습니다. ([공식 문서 원본](https://github.com/ros2/ros2_documentation/blob/humble/source/Installation/Ubuntu-Install-Debs.rst))

ROS apt 저장소 설정이 끝난 PC에서, RViz 등 GUI 도구까지 사용하려면:

```bash
sudo apt update
sudo apt install ros-humble-desktop
```

이미 Humble이 설치되어 있고 환경 검사에 통과했다면 위 설치는 생략합니다. ROS Base만 설치한 PC에서 메시지·TF 모듈이 빠져 있다면 해당 시스템 패키지를 추가합니다.

```bash
sudo apt install ros-humble-rclpy ros-humble-geometry-msgs \
  ros-humble-sensor-msgs ros-humble-nav-msgs ros-humble-rosgraph-msgs \
  ros-humble-std-msgs ros-humble-std-srvs ros-humble-tf2-ros-py
```

그래픽 라이브러리가 없는 최소 Ubuntu 환경에서만 다음을 추가합니다. 현재 PC에는 설치되어 있습니다.

```bash
sudo apt install libgl1 libegl1 libegl-mesa0 libgl1-mesa-dri
```

`rclpy`, `sensor_msgs`, `geometry_msgs`, `tf2_ros`는 이 가이드에서 **시스템 ROS 패키지로 공급**합니다. 이들을 `pip install`로 따로 설치할 필요가 없습니다.

## 2. Conda 환경 자동 준비

### 권장: env.sh 한 번으로 준비

```bash
cd ~/2026-AutoRace
source simul/env.sh
```

Conda 셸 초기화와 `autorace-simul` 환경 확인을 수행합니다. 환경이 없으면 `environment.yml`로 아래 항목을 설치하고 활성화하며, 기존 환경은 활성화 후 필수 패키지가 누락되거나 호환되지 않을 때만 보완합니다. 다른 Conda 환경에서 실행해도 `autorace-simul`로 전환합니다. 최초 설치·보완에는 인터넷이 필요하며, 설치 실패 시 오류를 반환합니다. **Panda3D도 YAML의 `pip:` 항목으로 자동 설치되므로 별도의 pip 명령은 필요 없습니다.**

`./simul/env.sh`는 환경 생성·검사를 수행할 수 있지만, 실행이 끝나면 현재 터미널의 환경은 그대로입니다. 현재 터미널에서 시뮬레이터·ROS 명령을 이어서 쓰려면 **`source simul/env.sh`**를 사용합니다. `simul` 폴더 안에서는 `source env.sh`입니다.

Conda 실행 파일은 `CONDA_EXE`, `PATH`, 홈 폴더의 일반적인 Anaconda/Miniconda/Miniforge/Mambaforge 경로에서 찾습니다. 별도 경로라면 `export CONDA_EXE=/설치/경로/bin/conda`를 먼저 설정하세요. Conda 자체와 시스템 ROS·그래픽 라이브러리 설치는 1번의 사전 준비입니다.

| 패키지 | 설치 위치/방법 | 용도 |
| --- | --- | --- |
| Python 3.10 | Conda | 시스템 ROS 2 Humble의 Python 모듈 연결 |
| NumPy 1.26.x | Conda | 차량 운동·라이다·영상 배열 |
| PyYAML | Conda | 차량·맵 설정 읽기 |
| Pillow | Conda | 이미지·JPEG 처리 |
| Tk | Conda | 장애물 편집 창 |
| PyQt5 5.15 | Conda (또는 시스템 PyQt5) | 미션 command 토글 창·화면 없는 렌더링 |
| pip, setuptools < 81 | Conda | 패키지 설치 및 빌드 호환 |
| pytest | Conda | 자동 검사. 일반 실행에는 사용하지 않음 |
| Panda3D 1.10.16 | 위 환경 안의 pip | 3D 화면과 RGB·Depth 렌더링 |

수동으로 환경을 조회·활성화하려면:

```bash
conda env list
conda activate autorace-simul
```

기존 환경에 YAML의 의존성을 반영해야 하는 경우에만:

```bash
cd ~/2026-AutoRace
conda env update -n autorace-simul -f simul/environment.yml
```

### 대안: 직접 Conda와 pip로 설치

YAML 대신 수동으로 만드는 경우입니다. **위 권장 방법을 실행했다면 이 단계는 생략**합니다.

```bash
conda create -n autorace-simul -c conda-forge \
  python=3.10 numpy=1.26 pyyaml pillow tk pyqt=5.15 pip 'setuptools<81' pytest
conda activate autorace-simul
python -m pip install 'panda3d==1.10.16'
```

pip 명령은 `conda activate autorace-simul` 후 `python -m pip`로 실행합니다. 시뮬레이터 기본 실행에 OpenCV, PyTorch, YOLO, colcon은 필요하지 않습니다. 맵 PNG·차량 BAM·표지판 이미지는 저장소에 포함되어 있습니다.

## 3. 환경 연결과 확인

시뮬레이터나 자신의 Python 노드를 실행할 새 터미널에서:

```bash
cd ~/2026-AutoRace
source simul/env.sh

python --version
python simul/tools/check_environment.py
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY"
```

Python은 `3.10.x`, 검사 항목은 모두 `OK`, ROS 설정은 `42 / 1`이면 됩니다. `run.sh`도 `env.sh`를 읽으므로 시뮬레이터 실행 때 필요한 연결은 자동 적용됩니다.

| 파일 | 역할 |
| --- | --- |
| `simul/env.sh` | autorace-simul 생성·활성화·필수 패키지 보완 + ROS 환경 + 시뮬레이터 Python 경로 |
| `simul/ros_env.sh` | ROS 명령용 환경 + 일반 ros2 topic 조회를 데몬 없이 실행하는 Bash 함수. Conda 활성화 없이도 사용 가능 |
| `simul/run.sh` | env.sh로 환경 준비 후 시뮬레이터 실행 |
| `simul/ros2.sh` | ROS 환경을 적용한 뒤 시스템 `ros2` CLI 실행 |

`ROS_LOCALHOST_ONLY=1`은 스크립트가 항상 적용합니다. 같은 PC의 터미널끼리 통신할 수 있습니다. `ROS_DOMAIN_ID`가 없거나 기본값 `0`이면 시뮬레이터용 `42`로 맞춥니다. `144`처럼 0이 아닌 사용자 지정 도메인은 유지합니다. 의도적으로 도메인 0을 쓰려면 `AUTORACE_SIM_DOMAIN_ID=0`을 지정하세요. 모든 통신 터미널에서 같은 도메인을 사용합니다.

## 4. 터미널 A — 시뮬레이터 실행

```bash
cd ~/2026-AutoRace
conda activate autorace-simul
export ROS_DOMAIN_ID=42
./simul/run.sh --mission track --view overview
```

이 터미널과 시뮬레이터 창을 켜 둔 채 다른 터미널에서 센서·제어 명령을 사용합니다. 기본 제어는 ROS이고 차량은 명령을 받기 전까지 정지합니다.

원하는 시작 위치를 고르려면 아래 중 하나를 실행합니다.

```bash
# 주차 접근 위치
./simul/run.sh --mission parking --view parking

# 연속 S자 두 개의 콘 미션 입구
./simul/run.sh --mission cone --view follow

# 7대 정적 장애물 회피 입구
./simul/run.sh --mission obstacle --view follow

# 갈림길 / 신호등
./simul/run.sh --mission fork --view follow
./simul/run.sh --mission signal --view follow

# 창 없이 센서를 렌더링하고 ROS 토픽 발행
./simul/run.sh --headless --mission track
```

`--no-ros`를 붙이면 화면과 키보드만 사용하며 센서 토픽은 발행하지 않습니다. `--headless`는 화면 창만 없애므로 센서 토픽은 발행합니다.

## 5. 터미널 B — 센서 수신

이 터미널은 base 상태여도 됩니다.

```bash
cd ~/2026-AutoRace
source simul/ros_env.sh
export ROS_DOMAIN_ID=42

ros2 topic list -t
ros2 topic echo /scan --once --qos-reliability best_effort
ros2 topic echo /camera/color/image_raw/compressed --once --field header
ros2 topic hz /camera/color/image_raw/compressed
```

`ros2 topic hz`는 계속 실행됩니다. 확인 후 Ctrl+C로 끝냅니다. 영상의 `--field header`는 큰 JPEG 바이트 배열 대신 시각·프레임만 보여줍니다.

매번 `source`하지 않고 한 명령씩 실행하려면 다음처럼 사용할 수도 있습니다.

```bash
cd ~/2026-AutoRace
ROS_DOMAIN_ID=42 ./simul/ros2.sh topic list -t
```

| 입력/출력 | 토픽 | 내용 |
| --- | --- | --- |
| 제어 입력 | `/cmd_vel` | Twist, `linear.x` m/s, `angular.z` rad/s |
| 카메라 출력 | `/camera/color/image_raw/compressed` | 640×480 JPEG |
| 카메라 출력 | `/camera/color/image_raw` | 640×480 `rgb8` |
| Depth 출력 | `/camera/depth/image_raw` | 640×400 `16UC1`, mm, 무효 값 0 |
| 카메라 정보 | `/camera/color/camera_info`, `/camera/depth/camera_info` | 해상도·투영 |
| 라이다 출력 | `/scan` | LaserScan, 245샘플, Best Effort |
| 상태 출력 | `/imu`, `/odom` | IMU·시작 위치 기준 odometry |
| 좌표·시간 | `/tf`, `/tf_static`, `/clock` | 센서 좌표계·시뮬레이션 시간 |

주기·프레임과 디버깅 토픽 전체는 [README의 토픽 표](../README.md#3-실제-제공하는-토픽)를 참고하세요. 주기는 시뮬레이션 시간 기준이며 실제 처리 속도는 PC 성능에 따라 달라집니다.

RViz에서 영상·라이다·TF를 보려면 `./simul/rviz.sh`를 실행합니다. 실차와 같은 `/scan` Best Effort 설정이 저장되어 있습니다. 기본 파일과 토픽별 큐 깊이는 [센서 기본 QoS](SENSOR_DEFAULTS.md)에 정리했습니다.

## 6. 터미널 B — 임의의 cmd_vel 보내기

위 ROS 환경을 읽은 터미널에서 실행합니다. 화면이 `KEYBOARD`라면 **K로 끄고**, `PAUSED`라면 P로 재개합니다. 다른 제어 노드가 `/cmd_vel`을 발행 중이면 그 노드의 발행을 먼저 중단합니다.

```bash
# 전진
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.2}, angular: {z: 0.0}}'
```

Ctrl+C로 위 발행을 끝낸 후 다른 명령을 실행합니다.

```bash
# 후진
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: -0.15}, angular: {z: 0.0}}'

# 전진하며 좌회전. angular.z가 음수이면 우회전
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.2}, angular: {z: 0.3}}'

# 기존 발행을 끝낸 뒤 정지 명령
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.0}, angular: {z: 0.0}}'
```

`-r 20`은 초당 20회 발행입니다. 발행을 중단하면 시뮬레이션 시간 0.5초 뒤 자동 정지합니다. 움직임을 `--once`로 한 번만 보내면 잠깐 움직이고 정지하므로 지속 주행에는 반복 발행을 사용합니다. LIMO Pro의 `angular.z`는 조향각이 아닌 회전 속도이며, 속도 0에서는 제자리 회전을 하지 않습니다.

## 7. 화면·마우스 조작

| 조작 | 동작 |
| --- | --- |
| 마우스 왼쪽 드래그 / 오른쪽 드래그 | 화면 위치 이동 / 회전 |
| 휠 | 확대·축소 |
| K | 키보드 제어 켜기·끄기 |
| W 또는 ↑ / ↓ | 키보드 전진 / 후진 |
| A·D 또는 ←·→ | 키보드 좌·우 조향 |
| Space | 키보드 주행 정지 |
| 1 / 2 / 3 / 4 | 주차장 / 전체 트랙 / 차량 추적 / 전방 시점 |
| S / V | 라이다 광선 / 카메라 미리보기 표시·숨김. 둘 다 기본 숨김 |
| L / R | 갈림길 왼쪽 / 오른쪽 표지판 |
| T / G / Y | 신호등 적색 / 녹색 / 황색 |
| P / F5 / Esc | 일시정지·재개 / 차량 리셋 / 종료 |

마우스는 관찰 화면만 움직입니다. S/V로 화면 표시를 숨겨도 ROS 센서 발행은 계속됩니다. 리셋은 차량·속도·충돌 상태를 초기화하고 시뮬레이션 시각은 계속 증가합니다.

## 8. 자신의 미션 코드 연결

자신의 Python 노드를 `autorace-simul`에서 실행할 때:

```bash
cd ~/2026-AutoRace
conda activate autorace-simul
source simul/env.sh
export ROS_DOMAIN_ID=42

# 예: 자신의 노드 파일 경로로 변경
python /path/to/my_parking_node.py --ros-args -p use_sim_time:=true
```

다른 Conda 환경이나 기존 ROS workspace의 노드를 사용할 때는 **그 환경의 의존성과 workspace setup을 적용한 후** `source simul/ros_env.sh`와 도메인 42 설정을 적용합니다. 그 환경에 필요한 OpenCV·PyTorch 등은 자신의 노드 의존성에 따라 설치합니다.

노드는 카메라·라이다를 구독하고 최종 `/cmd_vel`을 발행하면 됩니다. `/mission/parking/command_03`과 `/yolo/parking_sign`은 각각 자신의 주차·인지 노드가 만드는 출력입니다. 미션 노드의 `use_sim_time`을 켜서 `/clock`을 사용합니다. 시뮬레이터 센서를 사용할 때는 실제 `sensor_ws` 드라이버 launch를 함께 켤 필요가 없습니다.

설정 파일은 [차량·센서·토픽](../src/limo_parking_sim/config/limo_pro.yaml)과 [맵·시설물·미션 시작 위치](../src/limo_parking_sim/config/parking_map.yaml)입니다. 변경 후 시뮬레이터를 재실행합니다. 미션 알고리즘은 사용자가 작성하며, 시뮬레이터는 환경과 센서·제어 인터페이스를 제공합니다.

## 9. 선택 설치 — 필요한 기능만

### MP4 검증 영상 생성

화면 실행·센서 발행에는 필요 없습니다. 검증 스크립트의 MP4 저장에만 사용하며 현재 PC에는 설치되어 있습니다.

```bash
sudo apt install ffmpeg
```

### ROS 패키지로 빌드하고 ros2 launch 사용

`run.sh`로 실행할 때는 필요 없습니다. 빌드를 사용할 경우 활성 Conda Python으로 빌드합니다.

```bash
cd ~/2026-AutoRace
conda activate autorace-simul
python -m pip install 'setuptools<81' colcon-common-extensions
source simul/env.sh
export ROS_DOMAIN_ID=42

cd simul
python -m colcon build --base-paths src --symlink-install
source install/setup.bash
ros2 launch limo_parking_sim parking.launch.py mission:=cone view:=follow
```

다른 Python으로 빌드한 산출물이 있다면 새 build/install 경로를 사용합니다. 노드 실행 파일은 빌드할 때 사용한 Python 경로를 따릅니다.

### 미션 토픽을 선택해 cmd_vel로 바꾸는 어댑터

자신의 노드가 Twist를 직접 발행하거나 기존 mux를 사용한다면 생략합니다. 주차·콘·장애물 회피·차선·횡단보도·신호등·서행 미션의 `AckermannDriveStamped` 입력을 ON/OFF 토글로 연결합니다.

```bash
sudo apt install ros-humble-ackermann-msgs

cd ~/2026-AutoRace
./simul/adapter.sh
```

각 미션의 토글을 켜면 즉시 연결됩니다. 여러 입력을 켜면 docs의 번호 우선순위로 최신 유효 입력을 하나 선택하고, 번호 입력이 없으면 ON 상태인 lane 입력으로 복귀합니다. 모든 입력이 만료되면 정지합니다. **토픽 재선택**은 정지 후 모든 토글을 OFF로 되돌립니다. 사용자 지정 토픽은 **입력 추가**로 등록합니다. 미션 노드는 별도로 실행하고 `use_sim_time:=true`를 적용합니다. 기존 mux나 다른 어댑터는 먼저 종료합니다.

GUI는 PyQt5를 사용하며 새 환경 YAML에 포함됩니다. 기존 환경은 시스템 PyQt5를 재사용할 수 있습니다. 화면 없이 실제 GUI를 검증하려면 `source simul/env.sh` 후 `python simul/tools/check_adapter_window.py`를 실행합니다. 기본·최소 크기와 토글·우선순위·재선택·사용자 지정 입력의 렌더링 이미지를 저장합니다.

화면 없이 콘 미션만 연결하려면:

```bash
./simul/adapter.sh --no-gui --topic /mission/cone/command_04
```

토픽 목록과 상세 동작은 [미션 선택 창 사용법](../README.md#선택-창으로-모든-미션-직접-연결-mux-없이)을 참고하세요. 기존 주차 전용 `python -m limo_parking_sim.parking_adapter`도 유지합니다.

### 원본 PDF에서 맵 자산 다시 생성

포함된 맵을 그대로 사용할 때는 생략합니다. 맵 생성 도구에만 OpenCV·PDF 변환 도구·한글 폰트가 필요합니다.

```bash
sudo apt install poppler-utils fonts-nanum
conda activate autorace-simul
python -m pip install 'numpy==1.26.4' 'opencv-python-headless==4.11.0.86'

cd ~/2026-AutoRace
python simul/tools/extract_track.py
```

위 OpenCV 버전은 Python 3.10에서 NumPy 1.26과 사용할 수 있는 의존성을 제공합니다. ([패키지 정보](https://pypi.org/project/opencv-python-headless/4.11.0.86/)) 이 명령은 저장소의 맵 PNG·좌표 JSON을 다시 만듭니다. 차량 메시 변환과 표지판 이미지 준비 도구는 기본 환경의 NumPy·Pillow·Panda3D를 사용합니다.

## 10. 동작 검사와 문제 해결

```bash
cd ~/2026-AutoRace
conda activate autorace-simul
source simul/env.sh

python simul/tools/check_environment.py
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q simul/src/limo_parking_sim/test

# 별도 도메인의 새 오프스크린 시뮬레이터로 센서·CLI 제어 확인 후 종료
ROS_DOMAIN_ID=144 python simul/tools/check_ros_topics.py --cli-command

# 실제 움직임·연속 S자 두 개·7대 차량 회피를 영상으로 저장
python simul/tools/render_missions.py --output simul/captures/missions-check
```

도메인 144의 검증기는 현재 화면의 도메인 42 차량에 명령을 보내지 않습니다. 검사 시 도메인 144에는 다른 시뮬레이터나 제어 노드를 실행하지 않습니다. 검증용 경로 추종은 시뮬레이터의 동작 검사에만 사용하며 사용자의 자율주행 알고리즘을 대신 실행하지 않습니다.

| 증상 | 확인·해결 |
| --- | --- |
| `EnvironmentNameNotFound` | `conda env list`로 확인하고, 없다면 2번의 환경 생성 명령 실행 |
| `conda activate`가 동작하지 않음 | 현재 PC는 `source ~/anaconda3/etc/profile.d/conda.sh` 후 활성화. 다른 Conda 설치 경로라면 변경 |
| Python 3.10 오류 | base 대신 `conda activate autorace-simul` 후 `python --version` 확인 |
| `No module named panda3d` | 해당 환경 활성화 후 `python -m pip install 'panda3d==1.10.16'` |
| `No module named rclpy` | Python 3.10 및 `source simul/env.sh` 적용 여부 확인. ROS 시스템 설치도 확인 |
| 토픽이 안 보임 | 시뮬레이터 실행 여부, `--no-ros` 사용 여부, 양쪽 도메인 42·localhost 1 확인 |
| 토픽 목록이 오래된 것 같음 | 같은 ROS 환경에서 `ros2 daemon stop` 후 `ros2 topic list --no-daemon --spin-time 2` |
| `/scan` 값이 안 들어옴 | `--qos-reliability best_effort` 적용 |
| `/cmd_vel`을 보내도 정지 | K 키보드 모드·P 일시정지·충돌 상태·다른 발행자를 확인. 반복 발행에 `-r 20` 사용 |
| 창을 열 수 없음 | `--headless`로 실행하고 EGL/Mesa 또는 GPU OpenGL 설치 상태 확인 |
| pytest가 `lark` 등 ROS 플러그인 오류를 냄 | 위의 `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` 검사 명령 사용 |
| `_rclpy_pybind11`, `GLIBCXX` 오류 | 환경 검사 결과와 실제 Python 경로 확인. 현재 검증 환경인 Ubuntu 22.04 / Humble / Python 3.10 조합과 비교 |

상세 검증 결과는 [VALIDATION.md](VALIDATION.md), 시설물·치수 근거는 [SOURCES.md](SOURCES.md), 미션 배치는 [MISSIONS.md](MISSIONS.md)에 있습니다.
