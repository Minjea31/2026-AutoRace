# Lane 640×480 FPS 최적화 및 재적용 방법

작성일: 2026-11-05  
프로젝트: `/home/wego/2026-AutoRace`  
변경 전 기준 커밋: `aba2a1190ece02d56813a41223215d3d3a099865`

## 1. 결과와 측정 조건

YOLO와 Lane을 같은 Jetson에서 동시에 실행하고, 준비 구간이 지난 뒤 60초 동안 측정했다.

| 조건 | Lane 평균 처리 FPS | YOLO 실제 추론 FPS |
| --- | ---: | ---: |
| 최적화 전: 두 모델 TensorRT, YOLO 10Hz | 11.77 | 9.97 |
| 최적화 후: 같은 설정 | **19.24** | **10.00** |

- Lane 처리량 약 **63% 증가**, 최적화 후 구간별 범위 **16.63~20.99FPS**.
- Lane 입력: **640×480**, TensorRT FP16, OpenCV fallback OFF, 처리 타이머 30Hz.
- YOLO 입력: **640×480**, `test_in_e87.engine`, FP16 최적화, 기존 **10Hz 제한 유지**.
- 카메라: `/camera/color/image_raw/compressed`, 최종 측정 평균 **29.06Hz**.
- JetPack R36.4.7, TensorRT 10.3.0. YOLO engine은 내부 FP16 최적화를 켰고 입출력은 FP32이다.
- 전원 모드나 GPU 클록을 강제로 변경하지 않았다. 데스크톱 부하와 동적 클록이 있으므로 매번 같은 수치는 보장되지 않는다.

**이 수치는 처리 FPS이다.** Lane 모델은 ImageNet 사전학습 MobileNetV3-Small encoder와 미학습 decoder를 사용했다. 실제 차선 검출이 유효하지 않아 `valid_lane_fps`, `neural_lane_fps`는 0이다. 학습한 차선 모델의 정확도나 유효 차선 20FPS를 달성했다는 뜻은 아니다. 실제 학습 모델로 다시 측정해야 한다.

최종 기록:

- `lane_ws/train/runs/concurrent_both_trt_640_optimized_10hz.json`
- `lane_ws/train/runs/concurrent_both_trt_640_optimized_10hz_tegrastats.log`
- `lane_ws/train/runs/lane_cpu_profile_before.json`
- `lane_ws/train/runs/lane_cpu_profile_after.json`

## 2. 어떤 병목을 개선했는가

### A. 영상 전처리의 메모리 복사와 연산 축소

파일: `lane_ws/src/autorace_lane_lite/autorace_lane_lite/runtime.py`, `preprocess()`.

기존 흐름은 BGR→RGB 변환, HWC float 변환, 255로 나누기, mean 빼기, std로 나누기, CHW 변환 및 연속 메모리 복사였다. 프레임마다 큰 중간 배열이 여러 개 만들어졌다.

변경 후에는 처음부터 `(1, 3, H, W)` FP32 연속 버퍼를 만들고, BGR의 채널을 역순으로 읽어 RGB 각 평면에 직접 정규화한다.

```text
원래: (pixel / 255 - mean) / std
변경: pixel × (1 / (255 × std)) - mean / std
```

모델 입력 크기, RGB 순서, mean/std는 유지한다. 부동소수점 연산 순서에 따른 작은 오차는 테스트에서 허용 범위 내임을 확인했다.

### B. 전체 softmax 확률 텐서 대신 필요한 마스크만 계산

파일: 같은 `runtime.py`, `decode()`.

기존에는 3개 클래스 전체의 softmax를 구하고, 각 픽셀의 argmax와 최대 확률을 다시 계산했다. CHW 배열에서의 argmax, 전체 확률 정규화, 중간 배열 생성이 비쌌다.

변경 후:

1. OpenCV의 배열 비교로 가장 높은 클래스 ID를 구한다.
2. 같은 값이면 앞쪽 클래스가 선택되도록 엄격한 `>` 비교를 사용한다.
3. 안정화된 softmax의 최대 분자는 항상 1이라는 성질을 이용한다.

```text
m = max(logit0, logit1, logit2)
최대 확률 = 1 / (exp(logit0-m) + exp(logit1-m) + exp(logit2-m))
```

최대 확률만 구해 기존 confidence 기준을 적용한다. OpenCV의 벡터 연산을 사용하고 전체 3채널 확률 정규화와 strided argmax를 피한다. 배경 ID 0, 주황 차선 ID 1, 흰 점선 ID 2 및 nearest resize는 유지한다. FP32/FP16 입력, 클래스 동률, confidence 경계, 극단적인 logits, NaN/Inf를 테스트했다.

프로파일링에서 변경 전 전처리 약 12ms, 마스크 후처리 약 27ms가 관찰됐다. 변경 후에는 두 단계 비용이 감소했다. 개별 단계 시간은 CPU/GPU 부하에 따라 달라지므로 최종 판단은 ROS의 실제 처리 FPS로 한다.

### C. 카메라 수신과 추론 콜백 분리

파일: `lane_ws/src/autorace_lane_lite/autorace_lane_lite/node.py`.

기존에는 단일 스레드 executor와 같은 callback group을 사용했다. 추론 중 카메라 메시지 수신 콜백이 실행되지 못해 다음 타이머 호출이 이전 프레임을 보거나, 새 영상 처리가 늦어졌다.

변경 후:

- 카메라 구독에 별도 `MutuallyExclusiveCallbackGroup`을 부여한다.
- `main()`에서 `MultiThreadedExecutor(num_threads=2)`를 사용한다.
- 카메라 콜백은 최신 메시지와 수신 시각만 갱신한다.
- 추론 타이머와 제어 관련 콜백은 기본 callback group에 남겨 **추론·추적은 한 번에 하나만 실행**한다.
- 종료 시 executor를 종료한 뒤 node와 backend 자원을 정리한다.

입력 depth=1, 최신 프레임 우선, 같은 영상 중복 처리 방지, 영상 만료 검사, 처리 후 재검사 및 실패 시 정지 동작을 유지한다.

### D. 적용하지 않은 실험

OpenCV 스레드를 1개로 제한하는 실험에서는 개선이 없어 채택하지 않았다. 현재 최종 코드는 기존 OpenCV 스레드 설정을 사용한다. 해상도 축소, 타이머 값만 올리기, 차선 confidence 완화, 오래된 프레임 재사용으로 수치를 높인 것은 아니다.

## 3. 변경 파일과 재적용 패치

소스 변경:

- `lane_ws/src/autorace_lane_lite/autorace_lane_lite/runtime.py`
- `lane_ws/src/autorace_lane_lite/autorace_lane_lite/node.py`

검증 및 설명:

- `lane_ws/src/autorace_lane_lite/test/test_runtime.py` — 새 테스트
- `lane_ws/src/autorace_lane_lite/test/test_ros_safety.py` — 추론 중 카메라 수신 테스트 추가
- `lane_ws/README.md` — 최적화 및 실제 측정 조건 기록

이 변경만 담은 패치를 함께 저장했다:

**`/home/wego/lane_fps_optimization.patch`**

문서와 패치를 함께 보관하면 원래 코드에 다시 적용할 수 있다. 패치에는 모델 파일, 센서 로그, 실행 환경 변수는 포함하지 않았다.

### 원래 소스에 다시 적용

```bash
cd /home/wego/2026-AutoRace
git apply --check /home/wego/lane_fps_optimization.patch
git apply /home/wego/lane_fps_optimization.patch
```

현재 작업 폴더에는 이미 적용되어 있으므로 위 명령을 다시 실행할 필요가 없다. 현재 적용 상태는 다음 명령으로 확인할 수 있다.

```bash
cd /home/wego/2026-AutoRace
git apply --reverse --check /home/wego/lane_fps_optimization.patch
```

출력 없이 성공하면 패치의 변경이 현재 파일에 들어 있다는 뜻이다. 이는 확인 명령이며 변경을 되돌리지 않는다. 다른 버전에서 `--check`가 실패하면 기존 변경을 보존한 채 해당 파일 차이를 확인해 수동으로 적용한다.

## 4. 반드시 다시 빌드

당시 실행 환경은 소스가 아닌 install 디렉터리의 복사본을 사용하고 있었다. 소스만 수정하고 실행하면 이전 코드가 계속 실행될 수 있다.

```bash
cd /home/wego/2026-AutoRace/lane_ws
source env.sh
colcon build --symlink-install --packages-select autorace_lane_lite
source env.sh
```

빌드 후 실행 중이던 Lane 노드도 종료하고 다시 시작해야 한다. 현재는 위 빌드와 재시작까지 완료했다.

필요하면 실제 import 경로와 변경 반영을 확인한다.

```bash
python3 - <<'PY'
import inspect
from autorace_lane_lite import runtime
print(runtime.__file__)
print(inspect.getsource(runtime.preprocess))
PY
```

`preprocess()`에 CHW 버퍼를 만들고 채널별로 직접 쓰는 코드가 보여야 한다.

## 5. 실행 방법: 같은 성능 측정 조건 재현

같은 카메라와 `ROS_DOMAIN_ID`를 사용하고 Lane은 하나만 실행한다. 아래 명령은 미학습 모델의 제어 출력이 차량 제어로 들어가지 않도록 `/mission/lane`을 측정용 토픽으로 보낸다.

### 터미널 1: 카메라

프로젝트의 기존 센서 실행 방법으로 카메라를 켠다. 입력 토픽은 다음과 같다.

```text
/camera/color/image_raw/compressed
```

### 터미널 2: YOLO TensorRT, 10Hz

기존 ROS/PyTorch/Ultralytics/TensorRT가 동작하던 Python 환경에서 실행한다. 현재 YOLO engine은 고정 입력 `(1,3,480,640)`이므로 기본 정사각형 입력 크기를 그대로 넘기지 않고 `(480,640)`으로 맞춘다.

```bash
cd /home/wego/2026-AutoRace
source /opt/ros/humble/setup.bash
python3 - <<'PY'
import sys
sys.path.insert(0, '/home/wego/2026-AutoRace/yolo_ws/src')
import yolo_detect

yolo_detect.IMAGE_SIZE = (480, 640)  # (height, width), engine 입력과 일치
yolo_detect.PUBLISH_HZ = 10.0
yolo_detect.main(args=[
    '--ros-args',
    '-p', 'model_path:=/home/wego/2026-AutoRace/yolo_ws/model/test_in_e87.engine',
    '-p', 'image_transport:=compressed',
    '-p', 'image_topic:=/camera/color/image_raw/compressed',
])
PY
```

### 터미널 3: Lane TensorRT, 640×480

```bash
cd /home/wego/2026-AutoRace/lane_ws
source env.sh
ros2 run autorace_lane_lite lane_node --ros-args \
  -p backend:=tensorrt \
  -p model_path:=/home/wego/2026-AutoRace/lane_ws/models/lane_640_pretrained_encoder_benchmark.engine \
  -p input_width:=640 \
  -p input_height:=480 \
  -p fallback_enabled:=false \
  -r /mission/lane:=/benchmark/mission/lane
```

### 터미널 4: FPS 확인 및 60초 기록

```bash
cd /home/wego/2026-AutoRace/lane_ws
source env.sh
ros2 topic echo /lane/status --once --full-length
python3 train/monitor_ros.py --seconds 60 \
  --output "train/runs/lane_640_optimized_$(date +%Y%m%d_%H%M%S).json"
```

확인할 필드:

- `backend`: `tensorrt`인지 확인한다.
- `processed_fps`: 실제 처리량. 이번 개선 목표와 19.24FPS 기록에 해당한다.
- `valid_lane_fps`, `neural_lane_fps`: 유효 차선 검출이 나온 처리량. 현재 미학습 모델은 0이다.
- `pipeline_ms`, `total_ms`: 처리 지연.
- `error`: 모델 로딩이나 추론 오류가 없는지 확인한다.

이 문서 작성 시 실행 중인 측정 노드는 status도 `/benchmark/lane/status`로 분리해두었다. **현재 실행 중인 노드**를 확인하려면 다음 명령을 사용한다.

```bash
source /home/wego/2026-AutoRace/lane_ws/env.sh
ros2 topic echo /benchmark/lane/status --once --full-length
```

위의 터미널 3 명령으로 새로 실행하면 status는 기본 `/lane/status`이므로 `monitor_ros.py`를 그대로 사용할 수 있다. `ros2 topic hz /lane/status`는 상태 보고 주기이며 Lane 영상 처리 FPS가 아니므로 `processed_fps` 값을 봐야 한다.

## 6. 검증 재실행

차량과 분리된 전용 DDS 도메인에서 실행한다.

```bash
cd /home/wego/2026-AutoRace/lane_ws
source env.sh
ROS_DOMAIN_ID=93 ROS_LOCALHOST_ONLY=1 PYTHONDONTWRITEBYTECODE=1 \
  python3 -m unittest discover -s src/autorace_lane_lite/test -v
```

최종 코드 기준 **15개 테스트 통과**:

- RGB 정규화, 두 입력 해상도, dtype 및 연속 메모리 배치
- 기존 softmax 기준과 마스크 결과 비교, FP16/FP32, 동률·confidence 경계
- NaN/Inf 및 잘못된 logits 거부
- 추론이 대기 중이어도 새 카메라 프레임 수신
- 영상 만료·미래 시각·손상 영상의 정지 동작
- 추적 및 조향, fallback 동작

## 7. 모델 파일도 별도로 보관

패치와 Git 소스만으로 engine이 복원되지는 않는다. 특히 Lane 모델은 Git ignore 대상이다.

```text
yolo_ws/model/test_in_e87.pt
yolo_ws/model/test_in_e87.onnx
yolo_ws/model/test_in_e87.engine
lane_ws/models/lane_512_pretrained_encoder_benchmark.pt
lane_ws/train/exports/lane_640_pretrained_encoder_benchmark.onnx
lane_ws/models/lane_640_pretrained_encoder_benchmark.engine
lane_ws/models/lane_640_pretrained_encoder_benchmark.json
```

같은 장치에 기존 파일이 남아 있으면 그대로 사용한다. GPU/JetPack/TensorRT가 달라지면 ONNX 또는 checkpoint를 가져가 **대상 장치에서 engine을 다시 빌드**한다.

Lane engine 재생성 예:

```bash
cd /home/wego/2026-AutoRace/lane_ws
python3 train/build_tensorrt.py \
  --onnx train/exports/lane_640_pretrained_encoder_benchmark.onnx \
  --output models/lane_640_pretrained_encoder_benchmark_rebuilt.engine \
  --workspace-mib 512
```

출력은 새 이름을 사용했다. 실행할 때 `model_path`도 해당 경로로 바꾼다.

YOLO engine 재생성 예:

```bash
cd /home/wego/2026-AutoRace
python3 - <<'PY'
from ultralytics import YOLO
YOLO('yolo_ws/model/test_in_e87.pt', task='detect').export(
    format='engine', imgsz=(480, 640), half=True, batch=1,
    dynamic=False, workspace=0.5, simplify=False, opset=17,
    device=0, nms=False,
)
PY
```

이 명령은 같은 이름의 ONNX/engine을 생성하므로 기존 산출물을 보관하려면 실행 전에 다른 경로로 복사해둔다. 첫 engine 빌드는 이 장치에서 Lane 약 6분, YOLO 약 9분이 걸렸다. 빌드 중 메모리 확보를 위해 추론 노드는 잠시 종료하고, 빌드 완료 후 함께 다시 실행했다.
