# autorace_lane_deep_lite

lanedeep U-Net **클래스 마스크만** 써서 차선을 인식하는 경량 패키지입니다(HSV 없음). 마스크 클래스가 `0 배경 / 1 노란(주황) 차선 / 2 흰 점선`이라 흰 실선·횡단보도·정지선·반사광은 처음부터 배경입니다. 그래서 `autorace_lane_fusion`의 HSV 노이즈 제거 단계(top-hat, 두께 필터, `white_max_*`, 가로 흰 선 제거, 방향 검사, 슬라이딩 윈도우)를 모두 빼고 아래만 합니다.

1. 클래스 마스크 → BEV (최근접 보간, 카메라 JPEG 디코딩 없음)
2. 격자 칸(`cell`)마다 클래스별 중심점 1개
3. 같은 클래스 중심점을 방향을 유지하며 체인으로 연결(`max_gap`으로 흰 점선 빈칸을 건넘, `max_turn_deg`), 끊긴 체인 잇기, 같은 띠의 중복 체인 제거
4. 체인마다 곡선 피팅 → 후보 경계
5. 좌우 선택: 차량에 가장 가까운 선(차량이 안쪽에 있는 선) + 추적 가산점, 두 선이 차로 폭이 안 맞으면 추적 중인/긴 쪽을 남김. 차가 선 위에 있어도 추적 중인 선은 좌우가 뒤집히지 않음
6. 추적(평활·3프레임 확인·유지), 중앙 경로(한 번 더 매끄럽게 피팅), 목표점. 곡률은 중앙 경로 `curvature_span_m` 구간의 원 맞춤(`max_curvature`로 제한)

출력 `/lane_deep/target`은 `/lane_fusion/target`과 같은 형식이라 `autorace_lane_fusion_pid`가 그대로 받습니다.

## 실행 (실차)

```bash
cd ~/2026-AutoRace/lane_fusion_ws && source install/setup.bash
ros2 launch autorace_lane_deep_lite lane_deep_lite.launch.py profile:=real              # 딥 마스크 + 인식 + PID
ros2 launch autorace_lane_deep_lite lane_deep_lite.launch.py profile:=real pid:=false   # 인식만
ros2 launch autorace_lane_deep_lite lane_deep_lite.launch.py profile:=real debug:=true  # /lane_deep/debug (BEV 그림)
```

`lane_fusion_pid.launch.py`와 동시에 띄우지 않습니다(둘 다 `/mission/lane` 발행). PID 튜너는 `gui:=true` 또는 노트북에서 `ros2 run autorace_lane_fusion_pid lane_fusion_pid_tuner --profile real`.

## 설정

`config/real/lane.yaml`(06번 차량, 2026-11-07 줄자 보정 BEV, 1.88 mm/px), `config/sim/lane.yaml`. `lane_lite:` 형식 YAML(lane_ws·lane_fusion)도 읽을 수 있고, 이 패키지에 있는 키(BEV·폭·거리)만 가져옵니다. 실행 중 `ros2 param set /lane_deep_lite_node <키> <값>`으로 바꿀 수 있습니다.

## 검증 (bag + 프레임별 마스크)

마스크는 `lane_fusion_ws/tools/bag_masks.py BAG MODEL - --masks DIR`로 만듭니다.

```bash
ros2 run autorace_lane_deep_lite lane_deep_lite_eval BAG CONFIG.yaml MASK_DIR OUT_DIR --every 60 [--video out.mp4]
./build/autorace_lane_deep_lite/test_deep_lane   # 합성 마스크 단위 테스트 5개
```

요약 줄은 `lane_fusion_eval`과 같은 형식(유효·양쪽·한쪽·유지 비율, 횡오차 흔들림, 5 cm 이상 튐, 처리 시간)에 곡률 흔들림이 추가됩니다.
