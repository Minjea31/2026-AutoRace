# lanedeep_ws — Lightweight U-Net 차선 인식

MobileNetV3-Small encoder + U-Net decoder로 **배경 / 주황 차선 / 흰색 점선**을 분할하고, 같은 BEV·차선 추적·조향 모듈로 주행 명령을 만듭니다. 대상은 WeGo LIMO Pro의 Jetson Orin Nano 8GB입니다. 카메라는 640×480 RGB이며 ROS 압축 영상은 OpenCV로 디코딩한 뒤 RGB로 변환합니다.

```text
/camera/color/image_raw/compressed (640×480)
  → RGB·정규화·리사이즈 → MobileNetV3-Small U-Net → 3-class Mask
  → BEV → 좌/우 경계 추출·추적 → 중심·헤딩 → Ackermann 조향
  → /mission/lane → 기존 mux 및 속도 변경 미션
```

전체 모델은 **1,033,707 parameters**입니다. encoder는 [torchvision MobileNetV3-Small](https://docs.pytorch.org/vision/stable/models/mobilenetv3.html)의 `features`만 사용하고, decoder는 작은 채널과 depthwise separable convolution을 사용합니다. 학습 출력은 `[N,3,H,W]` logits, 실행 시 mask는 `0=background`, `1=orange_lane`, `2=white_dashed_lane`입니다.

공식 encoder 초기 가중치는 `models/torch_cache/hub/checkpoints/mobilenet_v3_small-047dcff4.pth`에 다운로드했습니다. 차선용 decoder와 전체 모델은 아래 데이터셋으로 학습해야 합니다.

**YOLO26n 동시 실행 시 유효한 차선 출력 20FPS 이상이 목표**입니다. Jetson에서 달성 여부는 아직 측정하지 않았습니다. 실제 라벨 학습, 카메라 BEV 보정, YOLO와 동시 측정 후 운용 설정을 정해야 합니다. 검증용 합성 데이터의 학습 결과는 차선 인식 정확도를 뜻하지 않습니다.

## 코드 구성과 기존 코드 재사용 범위

| 위치 | 역할 |
| --- | --- |
| `src/autorace_lane_deep/autorace_lane_deep/model.py` | MobileNetV3-Small encoder, U-Net decoder |
| `runtime.py` | PyTorch / ONNX Runtime / TensorRT / OpenCV backend |
| `tracking.py` | 모든 backend가 공유하는 BEV·sliding window·곡선 추적·조향 |
| `pipeline.py` | neural 추론 및 OpenCV fallback 선택 |
| `node.py` | 최신 영상 처리, 입력 유효시간, 기존 ROS 인터페이스 |
| `src/autorace_lane_deep/config/lane.yaml` | 카메라·BEV·속도·조향·추론 설정 |
| `train/` | Polygon 변환, 학습·평가, export, TensorRT 빌드, 추론·벤치마크 |

딥러닝 차선 구현은 `lanedeep_ws`의 `autorace_lane_deep` 패키지에서 관리하며, 실행 파일과 ROS 노드 이름은 `lane_deep_node`입니다. 기존 `lane_ws`는 원래 상태로 유지합니다. 공통 BEV·추적 모듈은 새로 구성했으며, 별도로 보관 중인 기존 BEV 코드가 있다면 `LaneTracker` 경계에 연결할 수 있습니다.

## ROS 인터페이스

| 방향 | 토픽 | 타입 | 의미 |
| --- | --- | --- | --- |
| 구독 | `/camera/color/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | 기본 카메라, BEST_EFFORT / depth 1 |
| 구독 | `/current_lane` | `std_msgs/msg/Bool` | false=좌측 경계 우선, true=우측 경계 우선, 기본 좌측 |
| 구독 | `/yolo/LR_sign` | `std_msgs/msg/Bool` | false=좌/A, true=우/B. 수신한 적 없는 false는 선택에 사용하지 않음 |
| 발행 | `/mission/lane` | `ackermann_msgs/msg/AckermannDriveStamped` | speed=m/s, steering_angle=rad, 좌회전 양수. RELIABLE / VOLATILE / depth 1 |
| 발행 | `/lane/status` | `std_msgs/msg/String` | JSON: backend, 유효 여부, 처리 지연, 처리·유효 차선·neural 차선 FPS |
| 선택 발행 | `/lane/mask`, `/lane/bev_mask` | `sensor_msgs/msg/Image` | `debug_masks: true`일 때 mono8 클래스 ID |

`mux_ws`, `yolo_ws`, 기존 미션 코드는 변경하지 않았습니다. `/mission/lane`을 mux와 slow/crosswalk/signal이 계속 구독하며 이 노드는 `/cmd_vel`을 직접 발행하지 않습니다. 전체 연결은 [architecture.md](../docs/architecture.md)를 참고하세요.

양쪽 경계가 있으면 중심을 계산하고, 한쪽만 있으면 `lane_width_ratio`로 중심을 추정합니다. `/current_lane`은 경계가 여러 개일 때 선호하는 측면을 정하는 입력이며, 차선 변경 명령은 아닙니다. 여러 개의 인접 경계 쌍이 보일 때만 유효시간 내 `/yolo/LR_sign`으로 좌/우 쌍을 선택합니다. 일반 구간에서 표지판 Bool만으로 조향을 강제하지 않습니다. 현재 구현은 영상 기하 기반 경로 선택이며, 갈림길 진입·통과 완료·재진입 방지의 전체 미션 상태 머신은 포함하지 않습니다. 실제 갈림길 데이터로 검증·확장이 필요합니다.

## 1. 환경 설정과 ROS 빌드

ROS2 Humble / Python 3.10 기준입니다. Jetson에서는 해당 JetPack의 CUDA·TensorRT를 사용합니다. PC용 CUDA/TensorRT wheel을 Jetson에 설치하지 마세요. TensorRT 추론 경로는 PyTorch를 import하지 않습니다.

```bash
cd ~/2026-AutoRace/lanedeep_ws
source env.sh
# 필요한 ROS·CPU 의존성 설치: 처음 한 번
sudo apt install ros-humble-ackermann-msgs python3-colcon-common-extensions \
  python3-opencv python3-numpy python3-yaml
colcon build --symlink-install --packages-select autorace_lane_deep
source env.sh
```

PC에서 ROS용 ONNX Runtime이 없다면 시스템 패키지를 공유하는 별도 환경에서 설치하고, **같은 Python으로 colcon을 빌드**합니다. 이후 ROS 실행 터미널에서도 이 환경을 활성화하세요. Jetson TensorRT/OpenCV 실행에는 이 과정이 필요 없습니다.

```bash
# lanedeep_ws에서
python3 -m venv --system-site-packages .ros-venv
source .ros-venv/bin/activate
python -m pip install 'numpy<2' 'onnxruntime>=1.16,<2'
source env.sh
python -m colcon build --symlink-install --packages-select autorace_lane_deep
source env.sh
```

`source env.sh`는 ROS와 현재 workspace를 적용합니다. 실제 차량의 DDS 설정을 보존합니다. simulator와 사용할 때는 같은 터미널에서 먼저 `source ../simul/env.sh`를 실행해 simulator 도메인 설정을 적용하고, 실행 시 `use_sim_time:=true`를 전달하세요. 차량은 기본 `use_sim_time:=false`입니다.

학습 PC는 ROS 터미널과 분리한 Python 환경을 권장합니다. [PyTorch 설치 안내](https://pytorch.org/get-started/locally/)에 따라 PC의 CUDA에 맞는 **torch / torchvision 호환 버전**을 먼저 설치하세요. 검증한 PC 환경은 torch 2.2.2 + torchvision 0.17.2, CUDA 12.1입니다.

```bash
cd ~/2026-AutoRace/lanedeep_ws
python3 -m venv .venv
source .venv/bin/activate
# 검증한 PC용 조합 예시. CUDA 환경에 따라 공식 설치 안내의 명령으로 변경
python -m pip install torch==2.2.2 torchvision==0.17.2 --index-url https://download.pytorch.org/whl/cu121
python -m pip install -r train/requirements.txt
# ImageNet encoder 다운로드 캐시도 lanedeep_ws의 ignored models 안에 보관
export TORCH_HOME="$PWD/models/torch_cache"
cd train
```

학습은 PC에서 수행합니다. `--pretrained`는 torchvision 공식 ImageNet encoder 가중치만 다운로드합니다. 차선용 학습 완료 가중치는 데이터셋으로 학습한 후 생성됩니다. 외부에서 받은 `.pt` 체크포인트는 신뢰할 수 있는 파일만 여세요.

## 2. Roboflow Polygon 라벨 → 학습 mask

Roboflow 프로젝트 유형을 **Instance Segmentation**으로 선택하고 클래스는 정확히 다음 두 개로 만드세요. 기존 Keypoint Detection 프로젝트의 점·skeleton 라벨은 이 파이프라인에 사용할 수 없습니다.

- `orange_lane`: 주황/노랑 차선의 실제 페인트 영역을 Polygon으로 둘러쌉니다.
- `white_dashed_lane`: 흰색 점선의 **각 페인트 조각만** Polygon으로 둘러쌉니다. 빈 간격을 채우지 않습니다. 같은 클래스의 여러 instance는 semantic mask에서 합쳐집니다.

흰색 실선, 횡단보도, 정지선, 콘·표지판은 차선 클래스로 라벨링하지 않습니다. 배경 클래스는 직접 라벨링하지 않습니다. 라벨을 생략한 차선 픽셀도 배경으로 학습되므로 학습에 사용할 이미지는 빠짐없이 검수하세요. 차선이 없는 장면은 annotation이 없는 음성 예제로 포함할 수 있습니다.

원본 `/home/a/AutoRace_bag/camera_frames_0p1s`는 유지됩니다. 인접 프레임을 무작위로 train/valid에 나누면 성능이 과장될 수 있으므로, **촬영 세션·주행 단위로 train/valid/test를 미리 분리**하세요. Roboflow에서 resize·crop·augmentation 없이 원본에 가까운 COCO export를 사용하고, 증강은 PyTorch 학습에서만 적용하세요.

검수 후 Roboflow에서 **COCO Segmentation** 형식으로 내려받아 다음과 같이 배치합니다. UI에서는 COCO로 표시될 수도 있으며 JSON의 `segmentation`이 polygon 배열이어야 합니다. [Roboflow COCO segmentation 설명](https://rfdetr.roboflow.com/latest/cookbooks/fine-tune_segmentation/)을 참고하세요.

```text
train/data/roboflow/
  train/{images..., _annotations.coco.json}
  valid/{images..., _annotations.coco.json}
  test/{images..., _annotations.coco.json}     # 선택
```

아래 명령은 **`lanedeep_ws/train` 안에서** 실행합니다. API key나 업로드는 필요하지 않습니다.

```bash
python convert_roboflow.py --source data/roboflow --output data/lane_masks
```

변환 결과는 split별 `images/*.png`, `masks/*.png`, `manifest.json`입니다. 원본 해상도를 유지하며 class ID를 기록합니다. 같은 클래스 Polygon은 합치고, 서로 다른 클래스가 겹친 픽셀은 **255(ignore)**로 둡니다. class 이름·이미지 크기·polygon 유효성·중복 이미지 SHA256을 검사하고 잘못된 입력은 실패시킵니다. 출력 디렉터리는 덮어쓰지 않습니다. SHA 중복 검사는 유사한 인접 프레임이나 증강된 동일 장면까지 발견하지는 못합니다.

## 3. 두 해상도 학습·평가

아래 명령은 모두 `lanedeep_ws/train`에서 실행합니다. 두 실험에 **같은 데이터 split, seed, epoch**를 사용합니다.

```bash
# 처리량 우선 후보
python train.py --data data/lane_masks --output runs/lane_512 \
  --size 512x288 --pretrained --epochs 80 --batch-size 4 --seed 31

# 작은/먼 차선 보존 비교
python train.py --data data/lane_masks --output runs/lane_640 \
  --size 640x480 --pretrained --epochs 80 --batch-size 4 --seed 31

# 중단 복원: epochs는 추가 횟수가 아니라 최종 epoch 번호
python train.py --data data/lane_masks --output runs/lane_512 \
  --size 512x288 --epochs 100 --batch-size 4 --resume runs/lane_512/last.pt

# 검증 또는 별도 test split 평가
python train.py --data data/lane_masks --size 512x288 \
  --weights runs/lane_512/best.pt --evaluate val
python train.py --data data/lane_masks --size 640x480 \
  --weights runs/lane_640/best.pt --evaluate test
```

loss는 **가중 Cross Entropy + foreground Dice**의 합입니다. 기본 CE 가중치는 배경/주황/흰 점선 `0.2 / 1.0 / 2.0`이며, ignore=255는 두 loss와 IoU에서 제외합니다. validation의 두 차선 클래스 평균 IoU가 가장 높은 `best.pt`, 복원용 `last.pt`, `metrics.jsonl`, `config.json`을 저장합니다. 클래스별 IoU도 확인하세요. 정답과 예측 모두 픽셀이 없는 클래스는 IoU 평균에서 제외하며, 없는 차선을 잘못 예측하면 0점으로 반영합니다.

CUDA에서는 AMP를 사용하고 `--no-amp`로 해제할 수 있습니다. CUDA 메모리가 부족하면 batch size를 줄이세요. CPU 확인은 `--device cpu --workers 0`으로 가능합니다. 다른 해상도 가중치에서 새 실험을 시작하려면 `--weights`를 사용하며, `--resume`은 동일한 해상도만 허용합니다.

| 모델 입력 (가로×세로) | 픽셀 수 | 비교할 점 |
| --- | --- | --- |
| 640×480 | 307,200 | 원본 비율, 먼 차선·작은 점선 보존 |
| 512×288 | 147,456 (48%) | 계산량 감소 후보, 얇은 차선 손실 여부 확인 |

입력은 **전체 영상을 해당 크기로 직접 리사이즈**합니다. 512×288은 원본 4:3 영상을 16:9로 늘리며 학습·추론이 같은 방식입니다. 추론 mask는 원본 영상 크기로 nearest resize한 뒤 BEV에 넣으므로 BEV 좌표계는 원본 기준입니다. 픽셀 수 감소가 곧 같은 비율의 FPS 향상을 보장하지는 않습니다.

## 4. ONNX export와 오프라인 추론

`lanedeep_ws/train`에서 실행합니다.

```bash
python export_onnx.py --weights runs/lane_512/best.pt --size 512x288 \
  --output exports/lane_512.onnx
python export_onnx.py --weights runs/lane_640/best.pt --size 640x480 \
  --output exports/lane_640.onnx

python infer.py --backend torch --model runs/lane_512/best.pt --size 512x288 \
  --image data/example.jpg --output runs/preview_torch
python infer.py --backend onnx --device cpu --model exports/lane_512.onnx --size 512x288 \
  --image data/example.jpg --output runs/preview_onnx
```

ONNX는 opset 17, batch=1, 고정 해상도, FP32 RGB 입력 `rgb`, logits 출력 `logits`입니다. export할 때 ONNX checker와 PyTorch/ONNX 수치 일치를 확인하고, 같은 이름의 JSON에 전처리·클래스·오차를 저장합니다. `--no-verify`는 검증을 명시적으로 생략할 때만 사용하세요.

추론 결과는 class ID의 `mask.png`, 색상 `overlay.jpg`, `bev.png`, 중심·헤딩·조향의 `result.json`입니다. 오프라인 도구는 선택한 모델의 실패가 드러나도록 fallback이 기본 OFF입니다. 필요하면 `--allow-fallback`을 지정하세요.

## 5. Jetson에서 TensorRT FP16 변환

PC의 ONNX를 Jetson의 `lanedeep_ws/train/exports`에 복사한 다음 **Jetson에서** 빌드합니다. engine은 GPU·TensorRT 환경에 의존하므로 PC에서 만든 engine을 그대로 Jetson에 배포하지 않습니다.

```bash
cd ~/2026-AutoRace/lanedeep_ws/train
python3 -c 'import tensorrt; print(tensorrt.__version__)'
python3 build_tensorrt.py --onnx exports/lane_512.onnx \
  --output ../models/lane_512.engine --workspace-mib 1024
python3 build_tensorrt.py --onnx exports/lane_640.onnx \
  --output ../models/lane_640.engine --workspace-mib 1024

python3 infer.py --backend tensorrt --model ../models/lane_512.engine --size 512x288 \
  --image data/example.jpg --output runs/preview_trt
```

지원 API는 JetPack 환경의 **TensorRT 8.5~10.x**입니다. FP16을 활성화하며 TensorRT가 필요한 계층을 FP32로 남길 수 있습니다. CPU 전처리·mask 복원·BEV는 FP16 변환 대상이 아닙니다. TensorRT 11부터는 정밀도 지정 방식이 달라지므로 현재 빌드 스크립트는 명시적으로 오류를 냅니다. 참고: [NVIDIA TensorRT 문서](https://docs.nvidia.com/deeplearning/tensorrt/latest/), [trtexec](https://github.com/NVIDIA/TensorRT/tree/main/samples/trtexec).

TensorRT 추론은 `tensorrt`와 CUDA runtime(`libcudart`)을 직접 사용하므로 PyTorch/PyCUDA가 필요 없습니다. JetPack 라이브러리를 사용하는 시스템 Python에서 실행하세요. workspace는 빌드 중 작업 메모리 상한이며 YOLO와 공유하는 실행 시 RAM 예약량은 아닙니다.

## 6. ROS2 실행

새 ROS 터미널에서 **`lanedeep_ws` 안에서** 실행합니다.

```bash
cd ~/2026-AutoRace/lanedeep_ws
source env.sh

# Jetson, 512 모델
ros2 launch autorace_lane_deep lane.launch.py backend:=tensorrt \
  model_path:="$PWD/models/lane_512.engine" input_width:=512 input_height:=288

# 위 프로세스를 종료하고 640 모델 비교
ros2 launch autorace_lane_deep lane.launch.py backend:=tensorrt \
  model_path:="$PWD/models/lane_640.engine" input_width:=640 input_height:=480

# OpenCV만 사용
ros2 launch autorace_lane_deep lane.launch.py backend:=opencv

# PC/시뮬레이터에서 ONNX 사용
source ../simul/env.sh
source env.sh
ros2 launch autorace_lane_deep lane.launch.py backend:=onnx device:=cpu \
  model_path:="$PWD/train/exports/lane_512.onnx" use_sim_time:=true
```

카메라, YOLO, mux는 각 workspace의 기존 방법으로 별도 실행합니다. lane 노드는 동시에 하나만 실행하세요. 토픽 확인은 다음 명령으로 통일합니다.

```bash
ros2 topic list
```

`backend:=auto`에서 모델을 지정하지 않으면 OpenCV fallback으로 동작합니다. 모델 로딩·추론 실패 또는 neural mask에서 경계 추출 실패 시에도 `fallback_enabled:=true`이면 OpenCV를 시도합니다. 로딩 오류는 로그와 status에 표시하고 5초 간격으로 재시도합니다. 모델 검증 때는 `fallback_enabled:=false`로 설정할 수 있습니다.

이미지 미수신·기한 초과·미래 시각·디코딩 실패 또는 두 인식 방식 모두 차선 검출 실패 시 **speed=0, steering=0**을 발행합니다. 기본 영상 유효시간은 0.25초이며 추론 완료 후에도 다시 검사합니다. 이미 처리한 영상을 새 인식처럼 재발행하지 않습니다. 입력 큐는 최신 한 장을 우선합니다.

OpenCV는 HSV와 형상에 기반한 대체 경로입니다. 흰색 실선 제외는 길이 휴리스틱이며 학습 모델과 같은 의미 구분을 보장하지 않습니다. 어두운 장면과 콘 등은 별도 검증해야 합니다.

## 7. BEV와 제어 보정

`src/autorace_lane_deep/config/lane.yaml`을 수정하고 재시작합니다. 기본값은 예시이며 실차 보정값이 아닙니다.

- `bev_source`: 원본 영상의 **좌상·우상·우하·좌하** x,y를 0~1로 정규화한 네 점입니다. 카메라 위치를 고정하고 실측한 바닥 기준점으로 설정하세요. 렌즈 왜곡이 크면 보정한 입력을 사용해야 합니다.
- `bev_lateral_m`, `bev_forward_m`: BEV 전체의 실제 가로 폭·앞쪽 거리입니다. `lane_width_ratio`는 차선 폭÷BEV 폭입니다.
- `lookahead_m`, `wheelbase_m`, `heading_gain`: 전방 목표점과 헤딩을 조향으로 변환합니다. 축간거리와 `max_steering_rad`는 실제 차량 값을 확인하세요.
- `speed_mps`: 기본 0.3m/s이며 커브에서 감속합니다. `tracking_alpha`는 중심 곡선 평활화, `max_center_offset_m`는 과도한 중심 오차 제외 범위입니다.
- `debug_masks: true`: mask와 BEV 토픽을 켭니다. 성능 측정에서는 OFF로 둡니다. `params_file:=/absolute/path/lane.yaml`로 설정 파일을 바꿀 수 있습니다. backend/model_path/device/입력 크기 등 launch 인자가 있는 값은 launch 인자가 우선합니다.

BEV·추적·조향은 모든 backend가 공유하므로 인식 방식만 바꿔 비교할 수 있습니다.

## 8. YOLO26n 동시 실행과 20FPS 측정

기존 YOLO26n을 별도 실행하고 Jetson의 전원 모드·냉각·카메라 FPS·YOLO 입력 조건을 고정하세요. 영상 입력도 초당 20장 이상이어야 합니다. Jetson의 `tegrastats`로 OS·YOLO·lane의 공유 메모리, GPU 부하와 온도를 함께 확인하세요.

아래는 `lanedeep_ws/train`에서 같은 대표 이미지 집합을 사용한 비교입니다. `--yolo-running`은 **사용자가 별도로 YOLO를 실행했다는 기록**이며 YOLO를 자동 실행하지 않습니다.

```bash
python3 benchmark.py --backend tensorrt --model ../models/lane_512.engine --size 512x288 \
  --images /home/a/AutoRace_bag/camera_frames_0p1s --frames 1000 --warmup 50 \
  --yolo-running --output runs/bench_512_with_yolo.json
python3 benchmark.py --backend tensorrt --model ../models/lane_640.engine --size 640x480 \
  --images /home/a/AutoRace_bag/camera_frames_0p1s --frames 1000 --warmup 50 \
  --yolo-running --output runs/bench_640_with_yolo.json
```

결과에는 decode·전처리·전송·추론·mask·BEV·추적·제어를 포함한 평균 FPS, p50/p95 지연, backend별 처리 횟수, 유효 차선 비율이 담깁니다. 디스크 읽기·카메라 전송·ROS 스케줄링은 제외되므로 **최종 판정은 ROS 실행 중 `/lane/status`의 실제 처리량**으로 확인하세요. `processed_fps`, `valid_lane_fps`, `neural_lane_fps`를 구분하며, fallback으로 실적을 채우지 않고 `neural_lane_fps >= 20`을 만족하는지 확인합니다.

카메라·lane·YOLO가 실행 중인 상태에서 별도 ROS 터미널로 60초 동안 기록할 수 있습니다. 기록기는 주행 명령을 발행하지 않습니다.

```bash
cd ~/2026-AutoRace/lanedeep_ws
source env.sh
python3 train/monitor_ros.py --seconds 60 --output train/runs/live_512_with_yolo.json
```

속도와 함께 held-out 데이터의 클래스별 IoU, mask 품질, 중심 오차, 실차 조향 안정성을 비교하세요. 512에서 정확도가 부족하면 640을 검토하고, 공유 부하가 크면 YOLO 입력 크기·주기도 함께 조정합니다. 모델 단독 `trtexec` FPS는 전체 차선 인식 FPS와 다릅니다.

## 검증

```bash
# lanedeep_ws에서 ROS를 source하고 build한 뒤, 전용 테스트 DDS 도메인에서 실행
source env.sh
ROS_DOMAIN_ID=93 ROS_LOCALHOST_ONLY=1 PYTHONDONTWRITEBYTECODE=1 \
  PYTHONPATH=src/autorace_lane_deep \
  python3 -m unittest discover -s src/autorace_lane_deep/test -v

# 학습용 Python 환경
PYTHONDONTWRITEBYTECODE=1 python -m unittest discover -s train -v
```

ROS 통합 테스트는 카메라 수신→주행 명령, 영상 만료·미래 시각·파손 시 정지를 검증합니다. ROS가 없는 학습 환경에서는 ROS 통합 테스트를 skip합니다. 테스트는 차량이 없는 전용 DDS 도메인에서 실행하세요.

구현 시 PC(RTX 3070 Laptop, torch 2.2.2, TensorRT 10.7)에서 테스트 16개, ROS 패키지 빌드·launch, GPU 학습·체크포인트 복원, 두 해상도의 ONNX 수치 검증·TensorRT FP16 빌드·실행을 확인했습니다. ONNX 최대 절대오차는 약 `4.5e-8`, FP16은 약 `1.2e-4`였으며 검증 입력에서 mask가 일치했습니다. 이는 합성 데이터 및 단일 입력으로 구현 경로를 확인한 결과로, 실데이터 정확도나 Jetson 처리량 검증은 아닙니다.

데이터·모델·변환 결과·미리보기·빌드 결과는 `.gitignore` 대상이며, 100 epoch 학습 실험의 검증 성능 기준 최적 모델 `model/best.pt`만 저장소에 포함합니다. 임시 테스트 산출물은 저장소에 남기지 않고, 테스트 소스는 재검증용으로 관리합니다.

## 로컬 자동 라벨링·검수·추가 학습

검수 웹 도구는 프로젝트 밖의 [~/seg_annotation](/home/a/seg_annotation/README.md)으로 분리했습니다.
`cd ~/seg_annotation && ./run.sh`로 실행하고 http://127.0.0.1:8765 에 접속합니다.
이미지·승인 대기·삭제 보관·학습 데이터도 해당 폴더에서 관리합니다.
