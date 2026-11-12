# autorace_lane_deep_lite

**검출은 딥, 차선 로직은 lane_ws** (2026-11-12). lanedeep U-Net 클래스 마스크(`0 배경 / 1 노란(주황) 차선 / 2 흰 점선`)가 lane_ws의 HSV 색 특징 단계를 대신하고, 그 뒤는 lane_ws `autorace_lane_lite` 코드를 그대로 씁니다(`src/lane_ws.cpp`, `include/autorace_lane_deep_lite/lane_ws.hpp` = lane_ws main `00e18a1` 복사본, 바뀐 곳은 마스크 입력 한 군데). 히스토그램/추적 시드 → 방향 추종 슬라이딩 윈도우 → 차로 폭 확인 → 추적(평활·3프레임 확인·유지) → 중앙 경로 → 목표점, 유지 경로는 `/odom`으로 이동. lane_ws 쪽 로직을 바꾸면 이 복사본도 같이 맞춰야 합니다.

- 설정 `config/<profile>/lane.yaml`(`lane_deep_lite:`) = lane_ws 같은 프로필 값에서 HSV 전용 키(색 범위·top-hat·흰 선 폭/길이·`min_area`)를 뺀 것. lane_ws YAML(`lane_lite:`)도 그대로 읽힘. 예전 셀 체인 방식 키(`cell`, `max_gap` 등)는 무시
- 출력 `/lane_deep/target`은 `/lane_lite/target`과 같은 형식 → lane_ws PID(`autorace_lane_pid`, `target_topic:=/lane_deep/target`)가 받음
- 통합 실행: `ros2 launch tensor_bringup tensor_bringup.launch.py lane_mode:=deep` (lane_ws도 source)
- bag 4개(프레임별 마스크): 5 cm 튐 234303 48 / 191317 56 / fast 74 / slow 104 (lane_ws HSV 76 / 53 / 90 / 112), CV 약 5.5 ms

## 실행 (실차)

```bash
cd ~/2026-AutoRace/lane_fusion_ws && source install/setup.bash
ros2 launch autorace_lane_deep_lite lane_deep_lite.launch.py profile:=real              # 딥 마스크 + 인식 + PID
ros2 launch autorace_lane_deep_lite lane_deep_lite.launch.py profile:=real pid:=false   # 인식만
ros2 launch autorace_lane_deep_lite lane_deep_lite.launch.py profile:=real debug:=true  # /lane_deep/debug (BEV 그림)
```

`lane_fusion_pid.launch.py`와 동시에 띄우지 않습니다(둘 다 `/mission/lane` 발행). PID 튜너는 `gui:=true` 또는 노트북에서 `ros2 run autorace_lane_fusion_pid lane_fusion_pid_tuner --profile real`.

## 튜너 GUI (노트북)

```bash
cd ~/dev/2026-AutoRace-fusionws/lane_fusion_ws && source install/setup.bash
export ROS_DOMAIN_ID=96 ROS_LOCALHOST_ONLY=0 FASTRTPS_DEFAULT_PROFILES_FILE=~/fastdds_peers.xml   # 06번 차
ros2 run autorace_lane_deep_lite lane_deep_lite_tuner --profile real                     # 차 실시간
ros2 run autorace_lane_deep_lite lane_deep_lite_tuner --profile real --bag BAG [--masks DIR] [--frame N]
```

- 왼쪽: 원본 + 딥 마스크(노랑/흰색) + BEV 사다리꼴(파랑), BEV 인식 화면. 아래: 상태·목표점·가까운 점·차로 폭 학습값·탈락 이유
- 오른쪽: 파라미터 슬라이더(바꾸면 바로 다시 계산, 추적은 초기화). **Apply to lane_deep_lite_node**: 차에서 도는 노드에 전체 값 적용. **Save YAML**: 기본 위치는 소스의 `config/<profile>/lane.yaml`(저장 후 다시 빌드해야 launch에 반영)
- 실시간 모드는 차의 `/lane_fusion/deep_mask`(차에서 lane_deep_lite.launch.py 또는 deep_mask.launch.py 실행 중)와 카메라를 받습니다. 와이파이로 마스크 640×480을 받으므로 초당 수~20장 정도입니다
- bag 모드의 마스크 폴더 기본값은 `<bag 상위>/deep_masks/<bag 이름>`

## 설정

`config/real/lane.yaml`(06번 차량, 2026-11-07 줄자 보정 BEV, 1.88 mm/px), `config/sim/lane.yaml`. `lane_lite:` 형식 YAML(lane_ws·lane_fusion)도 읽을 수 있고, 이 패키지에 있는 키(BEV·폭·거리)만 가져옵니다. 실행 중 `ros2 param set /lane_deep_lite_node <키> <값>`으로 바꿀 수 있습니다.

## 검증 (bag + 프레임별 마스크)

마스크는 `lane_fusion_ws/tools/bag_masks.py BAG MODEL - --masks DIR`로 만듭니다.

```bash
ros2 run autorace_lane_deep_lite lane_deep_lite_eval BAG CONFIG.yaml MASK_DIR OUT_DIR --every 60 [--video out.mp4]
./build/autorace_lane_deep_lite/test_deep_lane   # 합성 마스크 단위 테스트 5개
```

요약 줄은 `lane_fusion_eval`과 같은 형식(유효·양쪽·한쪽·유지 비율, 횡오차 흔들림, 5 cm 이상 튐, 처리 시간)에 곡률 흔들림이 추가됩니다.
