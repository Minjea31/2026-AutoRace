# yolo_ws

기본적으로 `/camera/color/image_raw` (`sensor_msgs/msg/Image`)를 구독하고 다음 Bool 인지 결과를 발행합니다.

| 토픽 | 구독 ws | false / true |
| --- | --- | --- |
| `/yolo/bump_sign` | 방지턱 처리 노드 | 미검출 false / 방지턱 표지판 감지 |
| `/yolo/slow_sign` | 서행 처리 노드 | 미검출 false / 서행 표지판 감지 |
| `/yolo/crosswalk_sign` | `crosswalk_ws` | 미검출 false / 보행자 표지판 감지 |
| `/yolo/LR_sign` | `lane_ws` | 좌/A / 우/B |
| `/yolo/parking_sign` | `parking_ws` | 미검출 false / 주차 표지판 감지 |

모든 출력 타입은 `std_msgs/msg/Bool`입니다. 좌·우를 제외한 토픽은 미검출 시 false를 발행합니다. `/yolo/LR_sign`은 미검출 시 발행하지 않습니다. 인지 결과의 시간적 확인과 영상 시각 검사를 수행한 뒤 발행합니다. 갈림길 주행 동작은 `lane_ws`가 처리합니다.

`src/yolo_detect.py`는 `model/test_in_e87.pt`를 사용하는 ROS 2 인지 노드입니다.

## YOLO 노드 실행

`src/`는 `ament_cmake` 기반 `yolo_detect` 패키지입니다. 노드와 `model/*.pt`가 함께 설치됩니다. ROS 2 Humble의 Python 환경에서 아래 명령을 실행합니다.
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
ros2 run yolo_detect yolo_detect --ros-args -p device:=cpu -p use_sim_time:=true
```

기존처럼 저장소 루트에서 `python3 yolo_ws/src/yolo_detect.py`로 직접 실행할 수도 있습니다.
PyTorch와 Ultralytics는 실제 노드를 실행하는 Python 환경에 설치되어 있어야 합니다.

카메라와 같은 `ROS_DOMAIN_ID`를 사용합니다. 시뮬레이션에서는 시뮬레이터의 도메인으로 맞추고 `--ros-args -p use_sim_time:=true`를 추가합니다. GPU가 있으면 GPU 0, 없으면 CPU를 사용합니다. 직접 지정하려면 `--ros-args -p device:=cpu` 또는 `-p device:=0`을 추가합니다. `model_path`, `image_topic`도 ROS 파라미터로 변경할 수 있습니다.

`yolo_detect.py` 상단의 **`CLASS_THRESHOLDS`**에서 6개 클래스 각각의 `confidence`와 `min_area_ratio`를 조정합니다. 크기는 원본 영상의 바운딩 박스 면적 비율이며, 예를 들어 640×480에서 `0.002`는 약 614 픽셀 면적입니다. 초기값은 실차에 맞춰 조정해야 합니다. `CONFIRM_FRAMES=2`로 서로 다른 연속 영상 2장에서 확인합니다.

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

기본 입력은 `/camera/color/image_raw` (`sensor_msgs/msg/Image`)입니다. 압축 입력은 다음처럼 선택합니다.

```bash
ros2 run yolo_detect yolo_detect --ros-args -p image_transport:=compressed -p image_topic:=/camera/color/image_raw/compressed
```

모든 유효한 YOLO bbox에 대해 매 추론마다 원본 영상 좌표를 영상 경계로 제한한
정수 픽셀 기준 가로(`width`), 세로(`height`), 면적(`area`)을 터미널에 출력합니다.
같은 클래스의 bbox가 여러 개면 각각 출력하며 confidence·면적 조건 탈락 및 OCR 거부 bbox도 포함합니다.
`class`는 OCR 변경 전 YOLO 클래스입니다. 예:

```text
BBox: class=bump; confidence=0.850; width=80px; height=60px; area=4800px^2
```

confidence·면적 기준을 통과한 클래스마다 매 추론에서 `class`, `output_topic`,
`confidence`(동일 클래스의 최고값), 실제 발행 여부를 출력합니다.
연속 프레임 확인 전, 좌우 동시 확인, 추론 중 영상 만료 시 미발행 이유를 표시합니다.
시작 후 또는 마지막 영상 수신 후 `camera_timeout_sec`(기본 2초)가 지나면 ERROR를
출력하고, 미수신이 계속되면 5초 간격으로 반복합니다. 수신 재개 시 INFO를 출력합니다.
오류는 ROS 로그로 보고하며 노드를 종료하지 않습니다. 감시는 시뮬레이션 시간이 정지해도 동작합니다.

## 검출 화면

`src/yolo_detect.py` 상단의 `SHOW_IMAGE = True`이면 검출 박스, 클래스와 confidence를 표시합니다.
`False`이면 화면을 끕니다. A/B가 읽힌 bbox는 OCR이 결정한 좌·우 방향으로 표시합니다.
다른 클래스의 bbox는 기존 YOLO 결과를 그대로 표시합니다.
실제 토픽 발행에는 클래스별 confidence·면적·연속 프레임 조건이 추가 적용됩니다.
`q`, `Esc`, 창 닫기는 화면만 닫고, Ctrl+C는 노드를 종료합니다. 디스플레이가 없는 환경에서는 화면을 자동으로 끕니다.

## A/B로 좌·우 방향 결정

`left_right_yolo` 파라미터의 기본값은 코드 상단 `LEFT_RIGHT_YOLO = True`입니다. YOLO가 만든 bbox에서 A가 읽히면
원래 클래스에 관계없이 `left`, B가 읽히면 `right`로 지정합니다.
예를 들어 YOLO가 `parking` 또는 `right`라고 예측해도 A가 읽히면
`left / A` bbox로 바뀌고 `/yolo/LR_sign`에 `false`를 바로 발행합니다.
B는 `right / B`로 표시하고 `true`를 바로 발행합니다.
A/B로 결정한 방향에는 추가 YOLO 클래스별 confidence·면적 조건이나
연속 2프레임 확인을 적용하지 않습니다. 입력은 YOLO가 반환한 bbox를 사용합니다.
영상 시각 유효성 검사는 유지하며 같은 프레임에 A/B가 모두 있으면 방향 발행을 생략합니다.

A/B가 없는 다른 클래스는 기존 YOLO bbox와 발행 조건을 유지합니다.
YOLO가 좌·우로 예측했지만 글자를 못 읽은 bbox는 방향을 발행하지 않습니다.
`left_right_yolo=false`이면 A/B OCR과 bbox 강제 지정을 건너뛰고 기존 YOLO 클래스와
연속 프레임 확인을 사용합니다. 코드 상단의 `LEFT_RIGHT_YOLO = False`로 기본값을
바꾸거나 실행 시 지정할 수 있습니다.

```bash
ros2 run yolo_detect yolo_detect --ros-args -p left_right_yolo:=false
python3 yolo_ws/src/yolo_detect.py --ros-args -p left_right_yolo:=false
```

OCR은 제공된 파란 표지판의 흰 A/B 글자와 OpenCV 글꼴을 기준으로 만든 경량 템플릿
문자 인식입니다. 추가 OCR 패키지나 모델 다운로드는 필요하지 않습니다.
`OCR_MIN_SCORE`(기본 0.80), `OCR_MIN_MARGIN`(기본 0.08)으로 유사도와 A/B 점수 차이를
조정합니다. `OCR` 점수는 글자 모양 유사도이며 확률이 아닙니다.
화면에는 `left / A YOLO:0.90 OCR:0.93` 형태로 표시합니다.
실제 주행의 기울기·흐림·조명에 맞게 기준을 조정할 수 있습니다.
