# 다차로 인식·튜닝 GUI 검증

2026-11-02, `lane_ws` 로컬 브랜치. C++17/OpenCV 코어, ROS 2 Humble 노드, Qt GUI, Bag 벤치마크가 동일한 `Pipeline::process()` 결과를 사용한다. 팀 토픽 `/camera/color/image_raw/compressed`, `/current_lane`, `/yolo/LR_sign`, `/mission/lane`과 메시지 타입은 그대로 유지한다.

## 결과 구조

`Result::boundaries`에는 검출된 각 경계의 지속 ID, BEV 2차 곡선, 흰색/노란색/미상 진단 색, 신뢰도, 현재 프레임의 실제 관측 여부가 있다. 경계 ID는 화면 내 순서가 아니라 프레임 간 곡선 거리로 연결한다. 추정 경계도 최대 `lane_occlusion_hold`초 동안 유지하며 신뢰도를 점차 낮춘다.

`Result::lanes`는 **가로로 인접한 두 경계**마다 한 개씩 만들어진다. 쌍의 폭을 여러 세로 위치에서 확인하여 지나치게 좁거나 넓은 조합과 급격한 폭 변화는 버린다. 각 항목은 좌우 경계 ID, 가운데 2차 경로, 차로 ID, 신뢰도, 추정 여부, 현재 차로 여부, 현재 차로에 대한 좌우 상대 순서를 가진다. 차로 ID는 경계 쌍 일치를 우선하고 가려진 경계 복원 시에는 경로 위치로도 연결한다. 새로운 바깥 경계가 들어와도 기존 경로의 ID는 밀려 바뀌지 않는다. 한 경계만 검출되고 기존 이웃이 없을 때는 제어가 선택한 차로 쪽에만 예상 폭의 가상 경계를 만든다.

현재 조향은 기존 선택 경계와 `Result::center_curve`로 계속 계산한다. 같은 결과의 `selected_lane_id`가 그 경로에 대응한다. 선택한 경계가 가려졌을 때는 보이는 한쪽 경계와 예상 폭을 사용하며, 멀리 떨어진 완전한 다른 차로 쌍으로 즉시 넘어가지 않도록 했다. 급격한 경로 변화 확인·조향 변화율·소실 후 정지·갈림길 A/B 상태 기계는 유지한다. 방향이 명시된 A/B 진입 때에는 해당 분기 경로로 Center Path를 전환한다.

GUI에서는 관측 경계 **빨간 실선**, 추정 경계 **빨간 점선**, 모든 차로의 가운데 경로 **파란 곡선**을 사용한다. 선택 경로는 더 굵고, 추정 경로는 점선이다. Lookahead Target은 초록 점, 횡방향 오차는 자홍색이다. Original·BEV·Detection Overlay에 같은 표시 이미지를 투영한다. 원본 데이터 `Result::original`은 후처리 없이 보존하고 GUI가 보여주는 `original_visual`에만 경로를 합성한다.

70개 파라미터는 한국어 설명·슬라이더·숫자 직접 입력을 함께 제공한다. 상단 그룹 선택으로 입력·영상, 색상 검출, ROI·BEV, Sliding Window, RANSAC, Tracking, Center Path, 제어 위치로 이동한다. 슬라이더가 설정 revision을 바꾸면 일시정지 영상도 즉시 재계산한다. Binary는 색상+ROI, 연결요소·행 폭 검사 후, RANSAC·형태 검사에서 승인한 최종 곡선을 별도로 보여주며 단계별 픽셀 수도 표시한다. Load/Save YAML과 실행 중 노드의 atomic 적용은 기존 공통 설정 schema를 사용하며, `lane_occlusion_hold`도 같은 방식으로 저장·적용된다. Bag 이전/다음 프레임·탐색·재생 속도·ROI/BEV 드래그를 유지한다.

## 장애물 회피 연결을 위한 경계

현재 `Result::lanes`의 차로 ID/경로/신뢰도/좌우 상대 위치는 향후 경로 선택의 **내부 자료 구조**다. `objavoid_ws`의 LiDAR 장애물 정보는 이번 구현에서 구독하지 않으며, 차선 변경 제어도 수행하지 않는다. 향후 연동은 장애물 판단 모듈이 선택한 차로 ID를 별도 입력으로 전달하고 `Result::lanes`의 해당 경로를 주행 목표로 사용하는 어댑터가 필요하다. 외부 메시지의 데이터·시간 기준·ID 유효기간과 mux 안전 조건은 팀과 합의한 뒤 설계해야 하므로 이번에는 토픽을 새로 만들지 않았다.

## 정답이 있는 4경계 Bag

`test/make_multilane_bag.py`가 생성한 180프레임 ROS 2 Bag을 공통 C++ 벤치마크로 처리했다. 흰색/노란색 경계가 섞인 직선·완만한 곡선이다. 초기 3경계→40번 프레임에 4번째 경계 등장→80–91번 프레임 내부 경계 가림→92번 복원→130–159번 외곽 경계 가림→160번 복원 순서다. `truth.csv`의 목표 x와 차로 수를 미리 정했다.

| 검증 항목 | 결과 |
| --- | ---: |
| 프레임별 기대 차로 수 일치 | 180 / 180 |
| 선택 차로 ID | 180프레임 내내 `2` |
| 4경계 프레임의 경로 수 | 3 |
| 내부 경계 가림 12프레임 | 세 차로 모두 유지, 관련 경로는 추정 표시 |
| 가려진 경계 복원 | 기존 차로 ID `1`, `2`, `3` 유지 |
| 선택 Target의 정답 대비 평균 절대 오차 | 0.291 BEV px |
| 선택 Target 오차 p95 | 0.623 BEV px |
| 선택 Center Path와 Target의 좌표 일치 | 180 / 180 |
| 계산 속도 0 / 검출 완전 소실 | 0 / 0 |

합성 영상은 경계 폭·색·가림 시간이 통제된다. 이 수치를 실제 도로 정확도로 일반화할 수 없다. `artifacts/multilane_fixture/final2/frames.csv`는 프레임별 차로 개수와 선택 ID, `lanes.csv`는 차로별 경계 ID·경로·신뢰도·추정 여부·상대 순서를 담는다. 단위 테스트는 4개 혼합색 경계→3개 경로, 파란 Center/초록 Target/빨간 경계의 Original·BEV 표시, 가상 경계 점선, 새 경계의 등장과 0.65초 이상 가림 뒤 ID 복원, 단일 경계의 가상 이웃을 확인한다.

## 실제 Bag과 실행 검증

실제 Bag: `/mnt/Ubuntu_Data/autorace_bags/lane_20261012_125620`, 3,311프레임·162.583초, 카메라 RGB8 640×480. 기본 `lane.yaml`은 실차 보정값이 아닌 시작 설정이다. Bag에는 `/current_lane`와 `/yolo/LR_sign`이 없고 정답 차로 ID·주행 경로 라벨도 없다. 직선·곡선·흰 외곽선/노란 중앙선이 동시에 보이는 장면의 영상과 경로 출력을 비교할 수 있지만, **정답이 있는 실제 3차선 도로 검증**은 이 Bag으로 완료할 수 없다. 아래 전체 처리 결과도 검출 수와 속도 통계이지 차로 정확도가 아니다.

실제 Bag을 기본 설정으로 처리하면 차로 경로 0개 598프레임, 1개 2,680프레임, 2개 33프레임이었다. 2개 경로가 보인 33프레임에는 추정 경로가 포함된다. **두 경계를 모두 실제로 관측한 차로는 114프레임에 한 개까지였고, 독립적인 세 차로를 실제 영상에서 검증할 수 있는 프레임은 없었다.** 원본 Bag의 장면·기본 BEV/차로 폭 보정 상태와 정답 라벨 부재가 제한이다.

제어가 양쪽 경계를 관측한 프레임 44개, 단일 경계 2,157개, 경계 없음 1,110개, 계산 속도 0은 694프레임이다. 차로 ID가 있는 경로는 2,713프레임에 생성됐고 선택 ID는 2,712프레임에서 유효했다. 전체 오프라인 처리량 83.05 FPS, 읽기+CV 평균 11.98ms·p95 16.72ms였다. 디스크 캐시가 따뜻한 실행이며 JPEG 구간 시각화 저장도 전체 처리 시간에 포함된다. 위 수치는 차로 정확도가 아니라 보정 전 설정에서의 경로 생성/처리량이다. 원본 Bag은 읽기 전용으로 열었다.

[실제 Bag 구간 이미지와 프레임 CSV](../artifacts/multilane_real_final3/) · [합성 Bag 차로별 CSV](../artifacts/multilane_fixture/final2/lanes.csv) · [최종 GUI 캡처](../artifacts/multilane_gui_final.png)

## 재현

```bash
cd /home/a/dev/2026-AutoRace/lane_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
PYTHONNOUSERSITE=1 /usr/bin/python3 src/autorace_lane/test/make_multilane_bag.py artifacts/my_multilane
ros2 run autorace_lane lane_benchmark artifacts/my_multilane/bag \
  artifacts/my_multilane_result artifacts/my_multilane/config.yaml
ros2 run autorace_lane lane_tuner --bag artifacts/my_multilane/bag \
  --config artifacts/my_multilane/config.yaml
```

Release `colcon build`가 통과했고 C++ 코어 21개·Qt ROI/BEV 드래그 4개, CTest 래퍼까지 27개 테스트가 오류/실패/skip 없이 통과했다. 갈림길 A/B, 지그재그, 단일 차선, 곡선, ROI/BEV, 반사광과 짧은 차선 픽셀이 중간 단계에 남아도 최종 경로는 승인되지 않는 사례를 포함한다. 최신 결과는 `artifacts/structure_build.txt`, `artifacts/structure_test_result.txt`에 기록했다. ROS 노드와 GUI 실시간/Bag 검증은 아래에서 구분한다.

최종 실행본으로 실제 Bag의 Qt GUI를 15초 재생했다. 렌더링 415프레임·28.73 FPS, 결과 준비→화면 표시 평균 8.12ms, Qt 타이머 최대 간격 16.97ms였다. 이전 프레임·seek·일시정지 중 튜닝·ROI/BEV 드래그·필터 슬라이더 변경이 통과했다. 합성 Bag GUI 캡처에서도 세 차로 경로와 색상 범례를 확인했다. `artifacts/multilane_gui_real_final.csv`, `artifacts/multilane_gui_final.png`에 기록했다. 실제 모니터 주사율이나 GPU 환경의 성능을 뜻하지는 않는다.

Binary 단계 분리 후 같은 실제 Bag의 GUI를 다시 15초 재생했다. 렌더링 425프레임·28.48 FPS, 결과 준비→표시 평균 7.86ms, Qt 타이머 최대 간격 17.04ms였고, 이전 프레임·seek·일시정지 중 튜닝·ROI/BEV 드래그·슬라이더 변경이 통과했다. `artifacts/structure_gui.csv`에 기록했다. 1959번 프레임의 기본 설정에서는 색상+ROI 34,545px, 연결·폭 검사 후 13,778px, 최종 승인 곡선 0px이었다. `filter_enabled=0`으로도 최종 승인 곡선은 0px이었다. 따라서 이전 GUI의 검은 “After structure filter” 화면은 실제 구조 검사 결과가 아니라 모든 곡선 검증 뒤의 마스크를 잘못 표시한 것이었다.

최종 `lane_node` ROS 통합 검증은 `/mission/lane` 20 Hz, 정상 영상의 주행, 입력 소실·과거/미래 stamp·손상 JPEG·멈춘 `/clock`에서 정지, 잘못된 원자적 파라미터 요청 거부와 `lane_occlusion_hold`를 포함한 정상 설정 적용을 확인했다. 기존 토픽 이름과 메시지 타입은 바꾸지 않았다. 결과는 `artifacts/multilane_ros_final.txt`에 기록했다.
