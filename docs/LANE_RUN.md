# 차선 주행 실행 방법 (HSV / 딥 라이트 / 퓨전)

차선 인식은 세 가지 중 **하나만** 띄웁니다. 셋 다 `/mission/lane`을 발행하고, mux가 이것을 `/cmd_vel`로 보냅니다.

| 방식 | 워크스페이스 | 인식 | 언제 |
| --- | --- | --- | --- |
| ① HSV | `lane_ws` | `autorace_lane_lite` (색상) | 딥 모델 없이 |
| ② 딥 라이트 | `lane_fusion_ws` | `autorace_lane_deep_lite` (딥 마스크만) | 보관용 실험. 비교할 때만 |
| ③ 퓨전 (권장) | `lane_fusion_ws` | `autorace_lane_fusion` (딥 우선 + 필요할 때만 HSV) | 기본 주행 |

경로는 저장소를 `~/2026-AutoRace`(차), `~/dev/2026-AutoRace`(노트북)에 clone했다고 가정합니다.

---

## 0. 처음 한 번 / 저장소 업데이트 후

### 0-1. 통신 설정 (차·노트북 모두, 처음 한 번)

차와 노트북이 같은 공유기·같은 `ROS_DOMAIN_ID`를 써야 노트북 튜너가 차의 토픽을 봅니다. 공유기가 멀티캐스트를 막아서 IP를 적은 `fastdds_peers.xml`을 씁니다(파일 안의 IP가 바뀌면 두 PC 모두 고칩니다).

```bash
cp ~/2026-AutoRace/lane_ws/fastdds_peers.xml ~/
# ~/.bashrc 맨 끝 (도메인은 차마다 다름. 06번 차 = 96)
export ROS_DOMAIN_ID=96
export ROS_LOCALHOST_ONLY=0
export FASTRTPS_DEFAULT_PROFILES_FILE=~/fastdds_peers.xml
```
새 터미널에서 `printenv | grep -E "ROS_DOMAIN_ID|FASTRTPS"`로 확인합니다.

### 0-2. 차에서 업데이트 + 빌드 (main을 받은 뒤 매번)

```bash
cd ~/2026-AutoRace
# 예전에 git이 추적하던 mux 빌드 파일(PR #22에서 추적 제외)이 수정돼 있으면 pull이 막히므로 먼저 되돌림
git checkout -- mux_ws/build mux_ws/install mux_ws/log sensor_ws/Log 2>/dev/null
git checkout main && git pull

source /opt/ros/humble/setup.bash
# 빌드는 반드시 각 워크스페이스 폴더 안에서 (홈에서 colcon build 하면 실패)
cd ~/2026-AutoRace/sensor_ws && colcon build --packages-select limo_base        # 조향 상한(안쪽 바퀴 0.80 rad)
cd ~/2026-AutoRace/mux_ws    && colcon build                                    # 속도·조향 상한(0.8 m/s, 0.65 rad)
cd ~/2026-AutoRace/lane_ws   && PYTHONNOUSERSITE=1 colcon build --packages-select autorace_lane_lite autorace_lane_pid --cmake-args -DCMAKE_BUILD_TYPE=Release
cd ~/2026-AutoRace/lane_fusion_ws && PYTHONNOUSERSITE=1 colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### 0-3. 딥 모델 엔진 (차에서 처음 한 번, 모델이 바뀌면 다시)

TensorRT 엔진은 Jetson마다 직접 만들어야 하고 git에 올리지 않습니다. 만든 뒤 `lane_fusion_ws`를 다시 빌드해야 설치됩니다.

```bash
cd ~/2026-AutoRace/lane_fusion_ws
python3 tools/build_tensorrt.py --onnx src/autorace_lane_fusion_deep/model/lane_640_mask.onnx \
  --output src/autorace_lane_fusion_deep/model/lane_640_mask.engine --workspace-mib 1024
PYTHONNOUSERSITE=1 colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```
이미 만든 엔진이 있으면(예: `~/lane_640_mask.engine`) `src/autorace_lane_fusion_deep/model/`에 복사하고 빌드해도 됩니다. TensorRT 파이썬은 `~/.bashrc`의 `PYTHONPATH`·`LD_LIBRARY_PATH` 설정이 있어야 import됩니다(일반 ssh 터미널은 자동). 딥 노드 첫 로그가 `tensorrt (mask output) 640x480 on cuda`면 정상입니다.

---

## 1. 실행 순서 (차 ssh 터미널 3개)

1. **센서** → 2. **차선(①/②/③ 중 하나)** → 노트북 PID 튜너에서 **■ 주행 정지**, `speed` 0.3 → 3. **mux** → **▶ 주행 시작**

**터미널 1: 센서 (공통)**
```bash
cd ~/2026-AutoRace/sensor_ws && source install/setup.bash
ros2 launch autorace_sensor_bringup sensors.launch.py
```

**터미널 2: 차선 — 아래 ①/②/③ 중 하나**

**터미널 3: mux (공통, 반드시 마지막에. 켜는 순간 차선을 잡고 있으면 출발)**
```bash
cd ~/2026-AutoRace/mux_ws && source install/setup.bash
ros2 launch autorace_mux mux.launch.py
```

**종료**: mux부터 `Ctrl+C`, 그다음 차선, 센서. 차선 방식을 바꿀 때는 이전 차선 launch를 꼭 끕니다(두 개가 동시에 `/mission/lane`을 내면 안 됨).

---

## ① HSV (lane_ws)

**차 터미널 2**
```bash
source /opt/ros/humble/setup.bash
source ~/2026-AutoRace/lane_ws/install/setup.bash
ros2 launch autorace_lane_pid lane_pid.launch.py profile:=real
```
- 인식(`lane_lite_node` → `/lane_lite/target`)과 PID(`/mission/lane`)가 함께 뜹니다.
- 미리보기만: `enable_vehicle_output:=false`. 인식만: `ros2 launch autorace_lane_lite lane_lite.launch.py profile:=real`

**노트북 튜너** (0-1 설정한 터미널)
```bash
source /opt/ros/humble/setup.bash && source ~/dev/2026-AutoRace/lane_ws/install/setup.bash
ros2 run autorace_lane_pid lane_pid_tuner --profile real      # PID·속도 (차의 제어기에 바로 적용)
ros2 run autorace_lane_lite lane_lite_tuner --profile real    # 차선 (차 카메라 실시간)
```

## ② 딥 라이트 (lane_fusion_ws, 보관용)

**차 터미널 2** (딥 마스크 노드 + 딥 라이트 인식 + 퓨전 PID)
```bash
source /opt/ros/humble/setup.bash
source ~/2026-AutoRace/lane_fusion_ws/install/setup.bash
ros2 launch autorace_lane_deep_lite lane_deep_lite.launch.py profile:=real
```
- 인식 출력은 `/lane_deep/target`, PID는 `autorace_lane_fusion_pid`를 그대로 씁니다. 미리보기만: `enable_vehicle_output:=false`, 인식만: `pid:=false`, BEV 그림: `debug:=true` → `/lane_deep/debug`

**노트북 튜너**
```bash
source /opt/ros/humble/setup.bash && source ~/dev/2026-AutoRace/lane_fusion_ws/install/setup.bash
ros2 run autorace_lane_fusion_pid lane_fusion_pid_tuner --profile real   # PID·속도
ros2 run autorace_lane_deep_lite lane_deep_lite_tuner --profile real     # 차선 (딥 마스크 실시간)
```

## ③ 퓨전 (lane_fusion_ws, 권장)

**차 터미널 2** (딥 마스크 노드 + 퓨전 인식 + 퓨전 PID)
```bash
source /opt/ros/humble/setup.bash
source ~/2026-AutoRace/lane_fusion_ws/install/setup.bash
ros2 launch autorace_lane_fusion_pid lane_fusion_pid.launch.py profile:=real
```
- 미리보기만: `enable_vehicle_output:=false` (`/lane_fusion_pid/command`만, 차 안 움직임)
- 딥 없이(HSV만): `deep:=false`. 딥 노드가 죽거나 마스크가 0.12 s보다 오래되면 자동으로 HSV만으로 계속 주행합니다.
- 로그(5초마다)에서 확인:
  - `lane_fusion_deep_mask: ~29 masks/s, ~30 ms/frame`
  - `lane_fusion: ~30 Hz | valid xx% | mask own / older (mean age) / none | HSV xx% | JPEG decoded xx% | ~10 ms`
    (`none`이 많으면 딥 노드 확인, HSV %가 높으면 딥이 한쪽을 자주 놓치는 구간)

**노트북 튜너**
```bash
source /opt/ros/humble/setup.bash && source ~/dev/2026-AutoRace/lane_fusion_ws/install/setup.bash
ros2 run autorace_lane_fusion_pid lane_fusion_pid_tuner --profile real   # PID·속도
ros2 run autorace_lane_fusion lane_fusion_tuner --profile real           # 차선 (Live)
```
차선 튜너 보는 법:
- **BEV "후보" 보기**: 보라 = 딥 후보, 주황 점선 = HSV 단독 후보, 초록/하늘 = 선택된 왼쪽/오른쪽, 회색 점선 = 이전 추적
- **경계·중앙 경로 창**: 빨강 = 중앙 경로(실선 = 양쪽 관측), 초록 점 = 목표점
- **상세창 첫 줄**: 딥 마스크 상태(이 프레임 것 / n ms 전 / 너무 오래됨), HSV 계산 여부, 경계 출처(deep / deep+hsv / hsv). "출력 보류"는 목표점 급변·차로 이탈로 직전 값을 유지한 프레임
- 슬라이더 → **Apply to running lane_fusion_node**(차에 즉시 적용) → 계속 쓰려면 **Save YAML**(노트북 소스 `config/real/lane.yaml`) 후 커밋·차에서 pull·빌드

---

## 공통 팁

- 노트북 튜너 영상은 와이파이로 받아서 초당 몇 장 수준입니다. 차의 인식·제어는 차 안에서 30 Hz로 돌아서 영향이 없습니다.
- 차선을 잡는지 바로 확인: `ros2 topic echo /lane_fusion/target --once`(①은 `/lane_lite/target`, ②는 `/lane_deep/target`). 첫 값 1 = 유효
- `/mission/lane` 발행자가 하나인지: `ros2 topic info /mission/lane -v`
- 문제 장면은 bag으로 남기면 오프라인으로 프레임 단위 분석이 됩니다.
  ```bash
  ros2 bag record -o ~/lane_run_$(date +%H%M) /camera/color/image_raw/compressed /lane_fusion/deep_mask \
    /lane_fusion/target /lane_fusion_pid/state /mission/lane /cmd_vel /odom
  ```
- 오프라인 비교(노트북): `lane_fusion_ws/README.md`의 "튜너·오프라인 비교" 참고 (`lane_fusion_eval`, `--no-hsv` / 마스크 없이 = 딥만 / HSV만)
- 자세한 구조: [lane_ws/README.md](../lane_ws/README.md), [lane_ws/RUN.md](../lane_ws/RUN.md), [lane_fusion_ws/README.md](../lane_fusion_ws/README.md)
