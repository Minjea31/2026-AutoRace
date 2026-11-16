# Lane 모델 최적화 스크립트 사용법

## 실행 위치와 준비

프로젝트의 `objavoid_ws` 폴더에서 실행합니다.

```bash
cd ~/2026-AutoRace/objavoid_ws
```

Python 환경에는 `torch`, `torchvision`, `opencv-python` 또는 시스템 OpenCV,
`numpy`, `onnx`, `onnxruntime`이 필요합니다. 플랫폼에 구성된 환경을 사용하는 경우:

```bash
conda activate dl_env
```

TensorRT 엔진은 사용할 Jetson에서 생성합니다. 기본 `trtexec` 탐색 경로는
`PATH`와 `/usr/src/tensorrt/bin/trtexec`이며 TensorRT 10.x 옵션을 사용합니다.

## 기본 실행

`/경로/best.pth`를 실제 모델 파일 경로로 바꿉니다. `.pt`도 같은 방법으로 사용합니다.

```bash
python tools/optimize_lane_model.py /경로/best.pth
```

변환 순서는 **체크포인트 → ONNX → TensorRT FP16 엔진**입니다.
ONNX 마스크를 원본 모델과 비교한 후 엔진을 생성하고 추론 벤치마크를 실행합니다.
원본 모델, 기존 엔진, ROS 설정은 변경하지 않으며 실행 중인 노드도 재시작하지 않습니다.

현재 프로젝트의 모델 파일을 사용하는 예시:

```bash
python tools/optimize_lane_model.py \
  ../lane_fusion_ws/src/autorace_lane_fusion_deep/model/best.pt
```

## 실행 전 명령 확인

```bash
python tools/optimize_lane_model.py /경로/best.pth --dry-run
```

출력 경로와 엔진 빌드 명령만 표시합니다. 모델을 로드하거나 파일을 생성하지 않습니다.
입력 모델 파일은 존재해야 합니다.

## ONNX까지만 생성

TensorRT가 없는 로컬 PC에서 ONNX를 준비할 때 사용합니다.

```bash
python tools/optimize_lane_model.py /경로/best.pth --export-only
```

ONNX 생성과 원본 모델 대비 마스크 비교까지 수행하며 엔진은 생성하지 않습니다.

## 실제 영상으로 결과 비교

카메라에서 저장한 PNG/JPG/JPEG 이미지 폴더를 지정합니다.

```bash
python tools/optimize_lane_model.py /경로/best.pth \
  --images /경로/검증이미지폴더 \
  --samples 20
```

최대 20장의 이미지로 ONNX와 원본 모델의 마스크를 비교합니다.
`--images`를 생략하면 검정·무작위 이미지로 기본 동작만 확인하므로 실제 차선 검출
정확도를 검증한 것은 아닙니다.

FP16 엔진의 마스크도 원본과 비교하려면:

```bash
python tools/optimize_lane_model.py /경로/best.pth \
  --images /경로/검증이미지폴더 \
  --check-engine
```

`--check-engine`에는 해당 Python 환경의 TensorRT 모듈과 CUDA 런타임이 추가로 필요합니다.
기본적으로는 엔진 빌드·벤치마크만 수행하고 엔진 마스크 일치 검사는 생략합니다.

## 출력 파일

기본 출력 폴더:

```text
입력모델이있는폴더/optimized/모델이름-실행시각/
```

직접 지정할 수도 있습니다. 기존 폴더를 덮어쓰지 않으므로 **새 폴더 이름**을 사용합니다.

```bash
python tools/optimize_lane_model.py /경로/best.pth \
  --output-dir ~/lane_optimized/run01
```

| 파일 | 내용 |
| --- | --- |
| `lane_mask.onnx` | 정규화·마스크 후처리가 포함된 ONNX |
| `lane_mask.engine` | 대상 Jetson에서 생성한 TensorRT FP16 엔진 |
| `timing.cache` | TensorRT 빌드 중 생성된 타이밍 캐시 |
| `build.log` | 엔진 생성 로그 |
| `benchmark.log` | 엔진 추론 성능 로그 |
| `timings.json` | 추론 시간 기록 |
| `report.json` | 변환·검증 결과와 실패 원인 |

`--export-only`에서는 엔진·빌드·벤치마크 관련 파일이 생성되지 않습니다.
생성한 엔진은 사용할 노드의 `model_path`에 별도로 지정해야 합니다.

## 주요 옵션

| 옵션 | 기본값 / 용도 |
| --- | --- |
| `--workspace-mib 512` | TensorRT 빌더의 workspace 한도(MiB). 전체 GPU 메모리 한도는 아님 |
| `--build-level 3` | TensorRT 빌드 최적화 수준, 0~5 |
| `--benchmark-seconds 5` | 엔진 벤치마크 실행 시간 |
| `--threshold 0.5` | 마스크 confidence 기준 |
| `--max-disagreement 0.005` | 검증 이미지별 최대 마스크 불일치 비율, 기본 0.5% |
| `--width 640 --height 480` | 입력 크기 지정. 두 옵션을 함께 사용 |
| `--trtexec /경로/trtexec` | TensorRT 실행 파일 직접 지정 |
| `--repo-root /경로/2026-AutoRace` | 프로젝트 루트 직접 지정 |
| `--factory 모듈:함수` | 다른 모델 구조의 생성 함수 지정 |

입력 크기는 기본적으로 체크포인트의 `size: [width, height]`를 사용하며,
해당 정보가 없으면 640×480을 사용합니다. 해상도를 자동으로 줄이지 않습니다.

## 지원하는 체크포인트

기본 모델 구조는 프로젝트의 **LaneUNet(MobileNetV3-Small U-Net)**입니다.
다음 형식을 지원합니다.

- `model`, `state_dict`, `model_state_dict` 키에 가중치가 들어 있는 체크포인트
- 가중치만 저장한 `state_dict`
- `module.` 접두어가 붙은 가중치

`.pth`나 `.pt` 확장자만 같다고 모든 모델 구조를 변환할 수 있는 것은 아닙니다.
다른 구조라면 원래 모델을 만드는 함수를 `--factory`로 지정해야 합니다.
해당 모듈은 현재 Python 환경에서 import할 수 있어야 합니다.

```bash
python tools/optimize_lane_model.py /경로/best.pth \
  --factory my_lane_model:create_model
```

모델 출력은 입력과 같은 해상도의 `[1, 3, H, W]` logits 또는 이를 `out` 키에 담은
딕셔너리여야 합니다. 클래스 순서는 **배경 0 / 노란·주황 차선 1 / 흰 차선 2**,
입력 정규화는 프로젝트의 RGB mean/std와 일치해야 합니다.

## 오류 확인

실패하면 출력 폴더의 `report.json`, `build.log`, `benchmark.log`를 확인합니다.
실패 단계에 따라 일부 파일은 없을 수 있습니다.

- `trtexec not found`: Jetson에서 실행하거나 `--export-only`를 사용합니다.
- `output directory already exists`: 다른 새 출력 폴더를 지정합니다.
- `checkpoint architecture differs`: 올바른 모델 파일 또는 `--factory`를 사용합니다.
- 마스크 불일치 기준 초과: 전처리·클래스 순서·모델 구조를 확인합니다.

벤치마크 수치는 엔진 추론 성능이며 카메라·전처리·ROS 통신까지 포함한 실제 처리 FPS와는 다릅니다.
