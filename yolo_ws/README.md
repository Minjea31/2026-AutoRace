# yolo_ws

기본적으로 `/camera/color/image_raw/compressed` (`sensor_msgs/msg/CompressedImage`)를 구독하고 다음 Bool 인지 결과를 발행합니다.

| 토픽 | 구독 ws | false / true |
| --- | --- | --- |
| `/yolo/bump_sign` | 방지턱 처리 노드 | 미검출 false / 방지턱 표지판 감지 |
| `/yolo/slow_sign` | 서행 처리 노드 | 미검출 false / 서행 표지판 감지 |
| `/yolo/crosswalk_sign` | `crosswalk_ws` | 미검출 false / 보행자 표지판 감지 |
| `/yolo/LR_sign` | `lane_ws` | 좌/A / 우/B |
| `/yolo/parking_sign` | `parking_ws` | 미검출 false / 주차 표지판 감지 |

모든 출력 타입은 `std_msgs/msg/Bool`입니다. 좌·우를 제외한 토픽은 미검출 시 false를 발행합니다. `/yolo/LR_sign`은 미검출 시 발행하지 않습니다. 인지 결과의 시간적 확인과 영상 시각 검사를 수행한 뒤 발행합니다. 갈림길 주행 동작은 `lane_ws`가 처리합니다.

`src/yolo_detect.py`는 `model/best.engine`를 사용하는 ROS 2 인지 노드입니다.

## YOLO 노드 실행

`src/`는 `ament_cmake` 기반 `yolo_detect` 패키지입니다. 노드와 `model/*.pt`, `model/*.onnx`, `model/*.engine`이 함께 설치됩니다. ROS 2 Humble의 Python 환경에서 아래 명령을 실행합니다.
기존 conda 환경을 사용한다면 ROS Python 모듈을 import할 수 있어야 합니다.

```bash
source /opt/ros/humble/setup.bash
cd yolo_ws
rosdep install --from-paths src --ignore-src -r -y
python3 -m pip install -e ./yolo26
colcon build --symlink-install --packages-select yolo_detect
source install/setup.bash
ros2 run yolo_detect yolo_detect
```

파라미터는 실행 시 지정할 수 있습니다.

```bash
ros2 run yolo_detect yolo_detect --ros-args -p model_path:=$HOME/2026-AutoRace/yolo_ws/model/best.pt -p device:=cpu -p use_sim_time:=true
```

기존처럼 저장소 루트에서 `python3 yolo_ws/src/yolo_detect.py`로 직접 실행할 수도 있습니다.
PyTorch와 Ultralytics는 실제 노드를 실행하는 Python 환경에 설치되어 있어야 합니다.

카메라와 같은 `ROS_DOMAIN_ID`를 사용합니다. 실차 플랫폼은 현재 94를 사용합니다. 시뮬레이션은 해당 도메인과 `use_sim_time:=true`를 사용합니다. 기본 TensorRT 엔진은 CUDA GPU가 필요합니다. CPU 실행은 `model_path`를 `best.pt`로 바꾸고 `device:=cpu`를 지정하세요. `model_path`, `image_topic`, `image_transport`는 실행 시 변경할 수 있습니다.

`yolo_detect.py` 상단의 **`CLASS_THRESHOLDS`**에서 클래스별 `confidence`와 `min_area_pixels`를 조정합니다. 크기는 **원본 영상 좌표로 복원하고 영상 경계로 제한한 bbox의 가로×세로 면적(px²)**입니다. 추론 입력 해상도 대비 비율이 아닙니다. 일반 YOLO 판정은 `CONFIRM_FRAMES=2`로 서로 다른 연속 영상 2장에서 확인합니다.

| 클래스 | 최소 면적(px²) |
| --- | ---: |
| slow | 3600 |
| bump | 3800 |
| parking | 2000 |
| crosswalk | 3200 |
| left / right | 500 (원본 영상 박스 면적, confidence 0.65 이상) |

현재 기본 모델은 제공된 `best.pt`에서 변환한 TensorRT **`best.engine`**입니다. 입력은 고정 `(1,3,640,640)`, batch 1이며 내부 FP16 최적화와 FP32 입출력을 사용합니다. 목표 출력은 10Hz입니다. `best.pt`와 `best.onnx`도 함께 보관합니다. 엔진은 Jetson Orin / TensorRT 10.3에서 생성했으며 다른 GPU·TensorRT 환경에서는 대상 장치에서 다시 빌드해야 합니다.

실차 실행 환경 예시:

```bash
source /opt/ros/humble/setup.bash
source ~/miniconda3/etc/profile.d/conda.sh
conda activate dl_env
cd ~/2026-AutoRace/yolo_ws
export PYTHONPATH="$PWD/yolo26:$PYTHONPATH"
source install/setup.bash
export ROS_DOMAIN_ID=94 ROS_LOCALHOST_ONLY=0
ros2 run yolo_detect yolo_detect
```

| 모델 클래스 | 발행 토픽 | Bool 의미 |
| --- | --- | --- |
| `bump` | `/yolo/bump_sign` | 검출 true / 미검출 false |
| `slow` | `/yolo/slow_sign` | 검출 true / 미검출 false |
| `crosswalk` | `/yolo/crosswalk_sign` | 검출 true / 미검출 false |
| `parking` | `/yolo/parking_sign` | 검출 true / 미검출 false |
| `left`, `right` | `/yolo/LR_sign` | 왼쪽 false / 오른쪽 true |

최신 영상을 0.1초 타이머로 처리하며 `/yolo/bump_sign`, `/yolo/slow_sign`, `/yolo/crosswalk_sign`,
`/yolo/parking_sign`에는 검출 true / 미검출 false를 계속 발행합니다.
연속 프레임 확인 전에는 false입니다. 같은 영상을 다시 추론하지는 않으며,
유효한 최신 영상의 결과만 유지합니다. 영상이 없거나 0.3초보다 오래되거나 처리에 실패하면
네 토픽 모두 false를 발행합니다. 시뮬레이션 시간이 멈춰도 발행 타이머는 동작합니다.
실제 주기는 추론 처리 시간에 따라 10 Hz보다 느려질 수 있습니다.
`/yolo/LR_sign`은 왼쪽 false / 오른쪽 true이며 미검출·좌우 동시 검출·영상 만료 시 발행하지 않습니다.

**방지턱과 서행은 별도 토픽으로 발행합니다.**
`bump`는 `/yolo/bump_sign`, `slow`는 `/yolo/slow_sign`로 발행하며 두 토픽은 각각 해당 클래스의 검출 결과를 발행합니다.
각 토픽은 해당 클래스만으로 독립적으로 판단하므로 두 클래스가 동시에 확인되면 둘 다 true입니다.
미검출·검출 조건 미충족·카메라 미수신·영상 만료·처리 실패 시에는 각각 false입니다.
수신 노드도 필요한 `/yolo/bump_sign` 또는 `/yolo/slow_sign`를 구독하도록 설정해야 합니다.

현재 모델에는 신호등 클래스가 없으므로 `/yolo/signal`은 발행하지 않습니다.
각 미션의 재진입·완료 처리는 미션 노드에서 관리합니다.

## Baseline 모델 학습

데이터 설정은 `train/dataset.yaml`, 데이터 폴더는 `train/dataset/{train,valid,test}/{images,labels}`입니다. `dataset/`은 Git에서 제외합니다.

```bash
conda activate yolo
python yolo_ws/train/train_baseline_model.py
```

기본 모델은 `yolo_ws/yolo26n.pt`, 결과 저장 경로는 `yolo_ws/train/checkpoints/`입니다. 기존 코드 구조와 학습 옵션은 유지했습니다. `checkout/`과 `train/checkpoints/`는 Git에서 제외합니다.

## 입력 및 진단 로그

로그는 프로세스 이름·레벨·시간 접두사 없이 메시지만 표시합니다.
`ros2 run yolo_detect yolo_detect` 또는 저장소 루트에서
`python3 yolo_ws/src/yolo_detect.py`로도 실행할 수 있습니다.

기본 입력은 `/camera/color/image_raw/compressed` (`sensor_msgs/msg/CompressedImage`)입니다. 아래 명령은 기본값을 명시한 것입니다. 원본 영상이 필요하면 `image_transport:=raw`, `image_topic:=/camera/color/image_raw`를 함께 지정합니다.

```bash
ros2 run yolo_detect yolo_detect --ros-args -p image_transport:=compressed -p image_topic:=/camera/color/image_raw/compressed
```

검출 로그는 confidence·면적·연속 검출 조건을 통과해 검출 신호를 실제 발행한 경우에만 출력합니다.
`class`, `output_topic`, `confidence`(동일 클래스의 최고값), `published` 값을 표시합니다.
일반 표지판의 미검출 `False` 상태와 미발행 BBox는 출력하지 않습니다.
좌회전은 `/yolo/LR_sign=False`가 유효한 발행이므로 로그를 출력합니다. 예:

```text
Detection: class=parking; output_topic=/yolo/parking_sign; confidence=0.950; published=True
```
시작 후 또는 마지막 영상 수신 후 `camera_timeout_sec`(기본 2초)가 지나면 ERROR를
출력하고, 미수신이 계속되면 5초 간격으로 반복합니다. 수신 재개 시 INFO를 출력합니다.
오류는 ROS 로그로 보고하며 노드를 종료하지 않습니다. 감시는 시뮬레이션 시간이 정지해도 동작합니다.

## 검출 화면

`src/yolo_detect.py` 상단의 `SHOW_IMAGE = True`이면 검출 박스, 클래스와 confidence를 표시합니다.
`False`이면 화면을 끕니다. bbox는 YOLO 결과를 그대로 표시합니다.
실제 토픽 발행에는 클래스별 confidence·면적·연속 프레임 조건이 추가 적용됩니다.
`q`, `Esc`, 창 닫기는 화면만 닫고, Ctrl+C는 노드를 종료합니다. 디스플레이가 없는 환경에서는 화면을 자동으로 끕니다.

## A/B로 좌·우 방향 결정

A/B는 YOLO 클래스로만 정합니다(OCR 없음, 2026-11-10 제거). `left`=A → `/yolo/LR_sign=False`,
`right`=B → `True`. 다른 클래스와 같이 confidence 0.65 이상, **면적 500px² 이상**, 연속 2프레임
확인을 거칩니다. 같은 프레임에 좌·우가 모두 확정되면 방향 발행을 생략합니다.
먼 거리에서 좌우 방향을 먼저 확보하도록 면적 기준을 500px²로 낮췄습니다.

`/yolo/LR_sign`을 발행할 때마다 확정한 bbox를 `/yolo/LR_detail`(`Float32MultiArray`)로 함께 보냅니다:
`[right(1)/left(0), 면적px², 가로, 세로, YOLO confidence, x0, y0, x1, y1, stamp_sec, stamp_nanosec]`
(원본 영상 픽셀). 차선 퓨전 노드의 갈림길 스냅샷이 사용합니다.


## 웹 YOLO 검출 화면

`/yolo/debug/compressed`는 원본 카메라와 별도로 bbox를 그린 JPEG입니다.
bbox 아래 작은 글씨로 최종 클래스, bbox 가로×세로, 면적(px²)을 표시합니다.
해당 bbox가 개별 confidence·면적 조건을 통과하고 실제 클래스 검출 신호가 발행된 프레임은 클래스별 색상,
조건 탈락·연속 검출 대기·방향 모호성 등 미발행 bbox는 회색입니다.
LEFT의 `/yolo/LR_sign=False`는 정상 방향 발행으로 취급합니다.
구독자가 있을 때만 그리기·JPEG 인코딩을 수행합니다. Bool 검출 신호와 터미널 로그 정책은 유지합니다.
웹 뷰어는 '카메라' 원본과 'YOLO 검출'을 별도 패널로 표시합니다.

## 8클래스 신호등 모델과 TensorRT 실행 (2026-11-12)

`model/best.pt`, `best.onnx`, `best.engine`은 YOLO26n 200 epoch 학습 결과입니다.
클래스 ID는 `0=slow, 1=bump, 2=crosswalk, 3=left, 4=right, 5=parking, 6=red, 7=green`입니다.
모델 원본은 `/home/a/annotation/bbox/models/yolo26n_red_green_200ep.pt`이며,
SHA-256은 `c2b39d49797a7d606576e7f3ff52746871731cc72e0675eb901e7941eacf8683`입니다.
기본 경로는 계속 `model/best.engine`을 사용합니다.

`/yolo/signal`은 `std_msgs/msg/Bool`이며 **빨강=False, 초록=True**입니다.
confidence 0.60 이상, 원본 영상 bbox 면적 2000px² 이상인 신호등이 서로 다른 영상에서
2회 연속 검출되어야 발행합니다. 빨강·초록 동시 검출 시에는 발행하지 않고 확인 횟수를 초기화합니다.
미검출·반복 영상·만료 영상·추론 오류 때도 신호등 메시지를 발행하지 않습니다.
미검출에 False를 발행하면 빨간불로 해석되므로 일반 표지판의 주기적 presence 발행과 구분합니다.
빨강 False의 로그와 디버그 bbox 색상도 실제 신호등 발행으로 처리합니다.
`signal_ws`의 기존 구독 코드와 같은 의미이며 실제 거리·조명에서 검출 기준은 추가 확인이 필요합니다.

엔진은 대상 Jetson의 TensorRT 10.3에서 생성한 FP16 최적화 결과를 복원했습니다.
고정 입력은 **1×3×640×640**, batch 1, workspace 512MiB, FP32 입출력입니다.
`~/LANE_FPS_OPTIMIZATION.md`의 이전 480×640 엔진 예제와 입력 크기가 다릅니다.
이 모델에는 `IMAGE_SIZE=640`, `rect=False`를 사용하며 해상도와 기존 표지판 면적 기준을 유지합니다.
카메라 수신은 별도 callback group과 2스레드 executor에서 수행합니다.
최신 영상과 수신 시각을 한 쌍으로 교체하고 추론·상태 갱신은 같은 callback group에서 순차 실행합니다.
OpenCV 창 처리는 main thread에서 수행해 Qt 창이 executor worker thread 사이를 이동하지 않게 합니다.
시작 시 backend를 미리 준비한 뒤 10Hz 추론 타이머를 생성합니다.
화면이 필요 없으면 `--ros-args -p show_image:=false`를 사용합니다.

원격 교체 전 백업, 검증 스크립트와 결과는 `/home/wego/yolo_update_20261112_215156/`에 있습니다.
소스와 모델을 바꾼 뒤 설치본에도 적용하려면 `yolo_ws`에서 ROS 환경을 source하고
`colcon build --packages-select yolo_detect`를 실행한 뒤 YOLO 노드를 재시작합니다.
