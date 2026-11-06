# cone_ws: LiDAR ROI 전처리와 라바콘 후보 클러스터링

`cone_lane_follower`는 `/scan`을 차량 좌표계로 변환하고 ROI 내부 포인트를
연속 스캔 인덱스와 XY 거리로 클러스터링한다. 크기 필터를 통과한 클러스터는
**라바콘 후보**이며 확정된 라바콘이 아니다. 좌우 분류·추적·경로 생성·주행 제어는 포함하지 않는다.
차량 명령 토픽(`/mission/objavoid/command_05`, `/cmd_vel` 등)에 발행하지 않는다.

## 파일 구성

- `src/cone_lane_follower/cone_lane_follower/node.py`: 스캔 유효성 검사, TF2 변환, ROI,
  인덱스 기반 클러스터링, 후보 필터, 관측 중심점 계산, Marker 발행.
- `src/cone_lane_follower/launch/roi.launch.py`: ROI·시뮬레이션 시간·설정 파일 인자, 선택적 RViz 실행.
- `src/cone_lane_follower/config/clustering.yaml`: 클러스터링·후보·시각화 파라미터.
- `src/cone_lane_follower/rviz/roi.rviz`: base_link 기준 원본 스캔(청록), ROI 경계(노랑),
  ROI 포인트(초록), 전체 클러스터(각기 다른 색), 후보 중심점(빨강)과 번호, TF.
- `src/cone_lane_follower/package.xml`, `setup.py`, `setup.cfg`, `resource/cone_lane_follower`,
  `cone_lane_follower/__init__.py`: ament_python 의존성·패키지·실행 파일·리소스 등록.
- `src/cone_lane_follower/test/`: 계산, 파라미터, DDS/TF/Marker 통합 테스트.
- `.gitignore`: 이 워크스페이스의 빌드·설치·로그·Python 캐시 제외.

## 처리 과정

1. 센서용 QoS(Best Effort, Volatile)로 `/scan`을 구독한다.
2. NaN/Inf 및 메시지의 `range_min <= r <= range_max` 밖 거리값을 제외한다.
   `angle_min + i * angle_increment`와 거리로 `(x_laser, y_laser, 0)`을 계산한다.
3. 스캔 시각의 `header.frame_id → base_link` TF를 조회해 회전과 이동을 한 번 적용한다.
   기본 장착이면 `(x + 0.103, y, -0.034)`가 되지만 이 수치를 코드에 고정하지 않는다.
   TF가 없으면 경고(최대 초당 한 번) 후 해당 스캔을 중단하고 이전 포인트를 지운다.
   스탬프 0은 TF2의 최신 변환 조회 규칙을 따른다. 한 스캔을 하나의 시각으로 처리하며
   회전 스캔의 포인트별 시간 보정(deskew)은 하지 않는다.
4. base_link 기준으로 `roi_x_min < x < roi_x_max`, `roi_y_min < y < roi_y_max`를 적용한다.
5. `/cone_lane/roi_points`에 초록색 POINTS(0.04 m)를 발행한다.
   동일 namespace/id로 갱신하며, 0개면 DELETE를 발행한다. 수신이 끊기면 0.5초의
   ROS 시간 후 만료된다. 시뮬레이션 일시 정지 시 ROS 시간과 만료도 함께 정지한다.
6. `/cone_lane/roi_marker`에 노란 LINE_STRIP(0.03 m) 사각형을 발행한다.
   닫힌 5개 꼭짓점, base_link의 z=0 평면이며 0.2초마다 현재 파라미터를 반영한다.
   실제 포인트의 z는 TF 결과를 그대로 유지한다(현재 -0.034 m). 시각화를 위한 높이 보정은 없다.
7. 전체 유효/ROI 내부/ROI 외부 포인트 수를 벽시계 기준 최대 초당 한 번 출력한다.
   전체 클러스터 수, 각 클러스터 포인트 수, 후보 개수·중심 좌표·포인트 수도 함께 출력한다.

ROI 이후 추가 처리:

1. 유효 스캔 추출부터 ROI 필터까지 원본 LaserScan 인덱스를 좌표와 함께 보존한다.
   기존 좌표 전용 함수 호출도 지원한다(`scan_to_points(..., with_indices=True)`,
   `filter_roi(points, indices)`로 인덱스 포함 결과를 얻는다). TF는 기존처럼 한 번만 적용한다.
2. 원본 인덱스가 정확히 1 증가하고, 직전 포인트와의 XY 유클리드 거리가
   `cluster_threshold` **미만**일 때만 같은 클러스터로 연결한다. 나머지는 새 클러스터다.
   NaN/Inf/ROI 바깥 포인트의 빈 구간을 연결하지 않는다. 스캔 순서를 유지하며 정렬하거나
   스캔 양 끝을 연결하지 않는다. 인접 비교는 포인트 수에 비례하는 O(N) 처리다.
3. 모든 클러스터는 `all_clusters`에 보관한다. 포인트 수가 `min_cluster_points` 이상이고,
   X와 Y의 최대값-최소값이 **각각** `max_cluster_width` 이하인 클러스터만
   `candidate_clusters`에 별도로 보관한다.
4. 후보의 관측 X/Y 좌표를 산술 평균해 중심점을 구한다. Z도 실제 변환된 포인트의 평균이다.
   실제 라바콘 기하학적 중심으로 보정하지 않는다. 벽·다른 장애물도 후보가 될 수 있다.
5. `/cone_lane/clusters`(MarkerArray)에 필터 전 모든 클러스터를 색깔별 POINTS로 표시한다.
   `/cone_lane/cone_centers`(MarkerArray)에는 후보별 빨간 SPHERE와 흰색
   `Cone Candidate N` TEXT_VIEW_FACING을 표시한다. 번호는 프레임 내 임시 번호다.
   텍스트만 중심점보다 0.12 m 위에 놓으며 구의 실제 위치는 변경하지 않는다.
6. 두 MarkerArray는 매 프레임 DELETEALL로 시작하므로 개수가 줄거나 0개여도 이전
   Marker가 남지 않는다. TF 실패 시에도 두 배열을 지운다. 서로 다른 토픽과 namespace를
   사용하며 ROI Marker에는 영향을 주지 않는다. 수명이 0.5초인 ROS 시간 Marker이며
   스캔 스탬프, base_link 좌표계를 사용한다.

두 Marker의 `frame_id`는 `base_link`다. ROI 경계는 Transient Local로 늦게 연결한
RViz에도 전달되며, 이동하는 차량의 프레임에 고정된다. ROI 파라미터 변경은 사각형에
최대 0.2초 후, 포인트에는 다음 스캔부터 반영된다. 최소값 이상 최대값 미만이 아니라
**최소값·최대값 모두 제외하는 엄격한 경계**다.

## 빌드

Conda가 아닌 시스템 ROS Python(`/usr/bin/python3`) 환경에서 빌드한다.
추가 시스템 의존성은 `package.xml`에 선언되어 있다. 설치가 필요하면 저장소 루트에서
`rosdep install --from-paths cone_ws/src --ignore-src -r -y --rosdistro humble`을 사용한다.

```bash
cd ~/2026-AutoRace/cone_ws
env -i HOME="$HOME" USER="$USER" LANG=C.UTF-8 PATH=/usr/bin:/bin /bin/bash --noprofile --norc -c '
  source /opt/ros/humble/setup.bash
  colcon build --symlink-install --packages-select cone_lane_follower
'
```

## 시뮬레이션과 실행

기존 `simul` 시뮬레이터를 먼저 실행한다. 시뮬레이터의 기본 DDS 도메인은 42이며
노드·RViz도 같은 도메인과 `/clock`을 사용해야 한다. 시뮬레이터 실행 방법은
[`../simul/README.md`](../simul/README.md)를 참고한다.

별도 터미널에서:

```bash
cd ~/2026-AutoRace
source simul/ros_env.sh
source cone_ws/install/setup.bash
ros2 launch cone_lane_follower roi.launch.py rviz:=true
```

RViz 없이 노드만 실행하려면 `rviz:=true`를 생략한다. 단독 노드 실행:

```bash
ros2 run cone_lane_follower roi_node --ros-args -p use_sim_time:=true \
  --params-file ~/2026-AutoRace/cone_ws/src/cone_lane_follower/config/clustering.yaml
```

RViz를 따로 실행하려면 같은 환경을 source한 터미널에서:

```bash
rviz2 -d ~/2026-AutoRace/cone_ws/src/cone_lane_follower/rviz/roi.rviz \
  --ros-args -p use_sim_time:=true
```

실차는 `/opt/ros/humble/setup.bash`, `cone_ws/install/setup.bash`를 source하고,
실차 센서와 동일한 DDS 도메인을 선택한 뒤 `use_sim_time:=false`로 실행한다.
이 노드는 센서 드라이버나 TF 발행자를 시작하지 않는다.

## ROI 변경

현재 `node.py`에서 사용자가 조정한 기본값(m): X `(0.05, 1.5)`, Y `(-0.5, 0.5)`.
이 값은 유지했다. 원래 실험 범위인 X `(0.1, 3.0)`, Y `(-1.5, 1.5)`로 실행하려면:

```bash
ros2 launch cone_lane_follower roi.launch.py rviz:=true \
  roi_x_max:=3.0 roi_y_min:=-1.5 roi_y_max:=1.5
```

```bash
# 시작 시 launch 인자
ros2 launch cone_lane_follower roi.launch.py roi_x_max:=4.0 roi_y_min:=-1.0 roi_y_max:=1.0

# 실행 중: double 타입이므로 소수점을 포함한다.
ros2 param set /cone_lane_roi roi_x_max 4.0
ros2 param set /cone_lane_roi roi_y_min -1.0
ros2 param set /cone_lane_roi roi_y_max 1.0
```

유한한 숫자만 허용하며 각 축의 최소값은 최대값보다 작아야 한다.
기존 범위 밖으로 최소·최대값을 함께 이동하려면 `set_parameters_atomically` 서비스로
동시에 변경하거나, 최대값을 먼저 확장한 뒤 최소값을 변경한다.

## 클러스터링 파라미터

| 파라미터 | 기본값 | 의미 |
|---|---:|---|
| `cluster_threshold` | 0.10 | 연속 인덱스 사이 XY 거리 임계값(m), 미만만 연결 |
| `min_cluster_points` | 2 | 후보 최소 포인트 수(정수, 1 이상) |
| `max_cluster_width` | 0.28 | X·Y 축 각각의 후보 최대 크기(m), 이하 허용 |
| `cluster_point_size` | 0.05 | 전체 클러스터 포인트 표시 크기(m) |
| `cone_center_size` | 0.08 | 후보 중심점 구 지름(m) |

`config/clustering.yaml`을 수정하고 노드를 재실행하면 반영된다. YAML은 클러스터링
파라미터만 포함해 사용자가 수정한 `ROI_DEFAULTS`를 덮어쓰지 않는다.
다른 YAML은 `ros2 launch cone_lane_follower roi.launch.py params_file:=/절대경로/설정.yaml`로 지정한다.
ROI launch 인자는 YAML보다 우선한다. `ros2 run`으로 실행할 때에는 위의 `--params-file`을
명시하거나 `-p cluster_threshold:=0.12` 등으로 지정한다.
새 설정 파일 설치 항목이 추가되었으므로 이번 업데이트 후에는 한 번 빌드한다.

실행 중에도 `ros2 param set /cone_lane_roi cluster_threshold 0.12`처럼 변경할 수 있다.
`min_cluster_points`는 정수, 나머지는 양의 유한한 실수만 허용한다. 파라미터 변경은
다음 스캔에 적용된다. 파라미터 값은 재실행 시 YAML/기본값으로 돌아간다.

RViz에서 `All scan clusters (individual colors)`와 `Cone candidate centers (red)`를
켜서 확인한다. ROI 초록 포인트가 클러스터 색과 겹치므로 색 구분이 필요하면
`ROI points (green)` 표시만 끌 수 있다. Fixed Frame은 `base_link`를 유지한다.

## 검증

```bash
cd ~/2026-AutoRace/cone_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
/usr/bin/python3 -m pytest src/cone_lane_follower/test -q
```

테스트는 별도 DDS 도메인에서 가상 스캔과 정적 TF를 사용한다. 거리·각도 메타데이터,
잘못된 거리, 3차원 회전/이동, ROI 경계, 파라미터 검증, Marker 속성·갱신·삭제,
TF 부재, 차량 명령 발행 부재를 확인한다.

클러스터링 테스트는 단일/복수 클러스터, 거리 임계값 미만·동일·초과, 인덱스 불연속,
NaN/Inf와 ROI 제외 후 인덱스 유지, 스캔 끝 미연결, 크기·개수 필터, 알려진 중심점,
잘못된 파라미터 거부, DDS MarkerArray 3→1→0 갱신·TF 실패 삭제를 포함한다.

```bash
colcon test --packages-select cone_lane_follower --event-handlers console_direct+
colcon test-result --verbose
```

구현 시 시스템 ROS Humble에서 빌드와 위 테스트를 실행했다. 실행 중인 시뮬레이터의
실제 `/scan`과 TF로 base_link Marker 수신도 확인했다. 클러스터링 추가 후 18개 테스트를
통과했고, 기존 노드와 충돌하지 않는 별도 검증 출력 토픽에서 실제 시뮬레이션 스캔의
클러스터·후보 중심점 MarkerArray도 수신했다. 원래 ROI 범위로 시험한 한 프레임에서
전체 클러스터 35개, 후보 23개였다. 이는 검출 정확도를 의미하지 않는다.
실차 센서 연결·주행, 실제 라바콘과 후보의 일치율은 검증하지 않았다.


`RoiNode.mission_cone`은 초기값이 False인 내부 bool 변수다. 매 스캔에서
ROI 내 후보 필터를 통과한 클러스터가 7개 이상이면 True, 6개 이하이면 False로 갱신한다.
TF 실패 시 False로 초기화한다. 값은 기존 초당 1회 통계 로그에도 표시한다.
mission_cone 자체는 내부 변수로 유지하며, 제어 명령 발행은 아래 오차 제어 절을 따른다.


`mission_cone=True`일 때 y>0 left(빨강), y<0 right(파랑)로 경계를 생성한다.
각 측 원점에 가장 가까운 점부터 마지막 점의 0.3m 이내 미사용 후보 중
거리와 진행 방향 변화의 점수를 평가해 미사용 후보를 반복 연결한다. 연결할 후보가 없으면 종료한다.
`/cone_lane/boundaries` MarkerArray를 사용하며 TF 실패나 mission_cone=False면 지운다.
이전 프레임 추적은 사용하지 않는다.


## 단면 대응과 중앙 경로

기존 경계 연결 방식과 좌우 명칭(y>0 left, y<0 right)을 유지한다.
left 경계를 누적 XY 거리로 샘플링한 뒤 주변 샘플의 접선과 직교하는 내부 방향 법선을 구한다.
법선과 right 경계 선분의 교점 중 레인 폭, right 진행 순서, 같은 진행 방향 조건을 통과한
유일한 교점만 대응점으로 사용한다. 같은 인덱스·가까운 라바콘끼리 짝짓지 않는다.
교점이 없거나 모호한 단면은 제외하고 끝점 밖으로 외삽하지 않는다.

각 단면에서 `center_x=(left_x+right_x)/2`, `center_y=(left_y+right_y)/2`를 계산한다.
중앙점은 단면 탐색 순서 그대로 연결하며 원점 거리나 최근접 순서로 재정렬하지 않는다.
큰 공백에서는 경로를 끊고 가장 긴 연속 구간만 사용한다. 이웃 중앙점의 가중 평균으로
평활화하되 단순한 레인 다각형 내부에 있고 경계를 가로지르지 않는 변경만 수용한다.
경계 다각형 교차, 중앙 경로 자기 교차, 평활화 후 과도한 꺾임은 경로를 무효화한다.

추가 설정(config/clustering.yaml, 실행 중 ROS parameter 변경 가능):

| 파라미터 | 기본값 | 의미 |
|---|---:|---|
| center_sample_step | 0.05 | 기준 경계 누적 거리 샘플 간격(m) |
| center_min_width | 0.10 | 허용 최소 단면 폭(m) |
| center_max_width | 1.50 | 허용 최대 단면 폭(m) |
| center_max_gap | 0.30 | 중앙점 연결 최대 간격(m) |
| center_max_turn | 75.0 | 평활화 후 허용 방향 변화(도) |
| center_smoothing_weight | 0.25 | 이웃 평균 반영 비율(0 초과, 0.5 이하) |

`/cone_lane/center_path`는 **visualization_msgs/MarkerArray**이다.
단면은 노랑, 원래 중앙점은 자홍, 평활화한 경로는 초록으로 표시한다.
RViz Fixed Frame base_link에서 `Center path and cross sections`를 켠다.
노드 내부 `center_pairs`, `center_points`, `center_path`에 결과를 보관한다.
mission_cone=False, TF 실패, 또는 유효한 중앙 경로를 생성하지 못하면 경로를 비우며 DELETEALL과 수명 0.5초로
이전 표시가 남지 않게 한다. 경계가 있어도 유효 경로를 만들지 못하면 center_path는 빈 목록이다.
중앙 경로는 MarkerArray로 표시한다. Path 메시지 발행·이전 프레임 추적은 추가하지 않았다.

합성 직선(엇갈린 배치), 곡선, 무효 입력, mission 종료 시 Marker 삭제를 포함해
pytest 29개가 통과했다. 실제 RViz GUI·시뮬레이터 주행·실차는 검증하지 않았다.
경계 연결의 방향 연속성을 검사하더라도 오검출이나 잘못된 초기 시드는 경로 무효화로
이어질 수 있다. 복잡한 헤어핀/S자에서의 단면 대응과 파라미터 튜닝은 추가 검증이 필요하다.
빌드·실행 방법은 위와 동일하며 노드와 RViz를 재실행하면 적용된다.


## 경계 방향 연속성

첫 경계점은 원점 XY 거리로 선택하고 이후에는 원점 거리 우선순위를 사용하지 않는다.
최근 선분들의 단위 방향을 평균해 진행 방향을 추정한다. 후보 방향과 평균 진행 방향의
각도 차이, 그리고 직전 선분 대비 각도 차이가 각각 한계를 넘으면 제외한다.
평균으로 인해 직전 방향의 급격한 역전이 숨겨지는 것을 방지한다.
첫 연결은 차량 +X 방향을 기준으로 별도의 초기 각도 한계를 적용한다.
각도 조건은 X 증가/감소 제한이 아니므로 자연스럽게 돌아가는 곡선을 연결할 수 있다.

허용 후보의 점수는 다음과 같으며 최소값을 선택한다.
`거리 가중치 × 연결 거리/최대 거리 + 방향 가중치 × 각도 차이/각도 상한`.
동일 점 재사용·자기 교차를 제외하며 right 경계 생성 시 이미 만든 left 경계와의
교차도 제외한다. 적합한 후보가 없으면 현재 지점에서 종료한다.
좌우 명칭(y>0 left, y<0 right), mission_cone 기준, 중앙 경로 생성은 유지한다.
이전 프레임 추적이나 시드 조합의 다단계 탐색은 추가하지 않았다.

| 파라미터 | 기본값 | 의미 |
|---|---:|---|
| boundary_connect_distance | 0.30 | 후보 최대 연결 거리(m) |
| boundary_max_turn | 80.0 | 평균 및 직전 방향 대비 최대 변화(도) |
| boundary_initial_max_turn | 110.0 | 첫 연결의 +X 대비 최대 각도(도) |
| boundary_direction_window | 3 | 평균에 사용할 최근 선분 수 |
| boundary_distance_weight | 1.0 | 거리 점수 가중치 |
| boundary_direction_weight | 2.0 | 방향 점수 가중치 |

config/clustering.yaml 또는 실행 중 `ros2 param set /cone_lane_roi boundary_max_turn 90.0`으로
조정한다. 최대 연결 거리와 가중치는 양수, 평균 구간은 1 이상의 정수,
최대 방향 변화는 180도 미만, 초기 각도는 180도 이하로 검증한다.
평균 구간이 길거나 각도 상한이 작으면 급커브에서 연결이 끊길 수 있으므로 실제 배치에 맞춘다.
초기 +X 가정은 차량이 커브 중간에서 시작할 때 부정확할 수 있으며,
y 부호 기반 좌우 분류의 한계도 남는다.

방향 우선 선택, 첫 연결 각도, X가 감소하는 곡선, 반대 경계 교차 거부와 파라미터
검증을 포함해 pytest 37개가 통과했다. 실제 RViz GUI·주행 검증은 수행하지 않았다.

좌우 정의는 ROS base_link 기준 +Y left(빨강), -Y right(파랑)이다.
중앙점 탐색은 left 접선의 오른쪽 법선 `(dy, -dx)`을 사용하며 진행 기준 +X는 유지한다.


## 중앙 경로 오차 제어

노드 내부의 검증된 center_path를 사용해 20Hz로
`/mission/cone/command_04`에 `ackermann_msgs/msg/AckermannDriveStamped`를 발행한다.
별도 패키지·파일·중앙 경로 구독자는 추가하지 않았다. ackermann_msgs 의존성을 선언했다.

차량 전방 기준점 `(control_reference_x, 0)`을 중앙 경로의 전방 진행 선분에 투영한다.
횡오차는 선분의 왼쪽 법선 방향 부호를 사용하고, 방향 오차는 선분 접선의 atan2(dy, dx)다.
`steering = control_lateral_gain * lateral_error + control_heading_gain * heading_error`로
조향을 계산한 뒤 각도와 변화율을 제한한다. 이는 횡오차·방향 오차의 P 피드백이며
적분·미분 또는 실제 속도 피드백은 사용하지 않는다. 조향 양수는 좌회전, 단위는 rad다.
조향 요구가 클수록 속도를 기본 속도의 30%까지 낮춘다. header는 현재 ROS 시간과 base_link다.
프로젝트 mux와 시뮬레이터 mission_adapter의 Ackermann 단위·cone 토픽을 확인했다.

| 파라미터 | 기본값 | 의미 |
|---|---:|---|
| control_speed | 0.15 | 직진 기준 명령 속도(m/s) |
| control_reference_x | 0.25 | 차량 전방 오차 평가 기준점(m) |
| control_lateral_gain | 2.0 | 횡오차 조향 이득(rad/m) |
| control_heading_gain | 1.0 | 방향 오차 조향 이득 |
| control_max_steering | 0.48 | 조향각 절댓값 상한(rad), mux 28도 한계 이내 |
| control_max_steering_rate | 1.0 | 조향 변화율 상한(rad/s) |
| control_timeout | 0.20 | 스캔의 ROS 시간·수신 경과 시간 상한(s) |
| control_max_path_distance | 0.50 | 전방 기준점에서 사용 가능한 경로까지 최대 거리(m) |

숫자 파라미터는 양의 유한수이며 최대 조향은 pi/2 미만이다. YAML 또는 실행 중
`ros2 param set /cone_lane_roi control_speed 0.1` 등으로 조정한다.

미션 비활성 시 명령은 발행하지 않는다. mission_cone=True가 되면 제어를 시작하고,
활성 중 TF 실패·스캔 중단·오래된/미래/0 timestamp·전방 경로 부족 시
speed=0, steering_angle=0으로 정지하며 정지에는 조향 변화율 제한을 적용하지 않는다.
중앙 경로 소실 시에는 아래의 제한된 제어 유지 조건을 먼저 확인한다.
TF 실패나 스캔 중단 동안에는 활성 상태를 유지하며 정지 명령을 반복한다.
정상 스캔에서 mission_cone=False가 확인되면 정지 1회를 발행하고 이후 발행을 중단한다.
이는 mux에서 speed=0도 활성 명령이므로 다른 미션을 계속 차단하지 않기 위한 처리다.
따라서 후보 6개 이하에 의한 미션 해제 후 mux가 다른 소스로 전환할 수 있다.
장기적인 미션 종료 판단은 별도 상태 관리가 필요하다.

```bash
ros2 topic echo /mission/cone/command_04
```

시뮬레이터에서 cone 미션 토픽을 선택하거나 기존 autorace_mux를 연결해야 차량에 전달된다.
노드는 /cmd_vel이나 /mission/objavoid/command_05를 발행하지 않는다.
현재 벽 오검출과 프레임별 좌우 소속 변경 문제가 제어로 해결되지는 않는다.
실제 조향 부호·게인·속도·주행 안정성은 실차 및 주행 시뮬레이션에서 아직 검증하지 않았다.
프로세스 종료/강제 종료 시에는 기존 mux/시뮬레이터의 입력 timeout 정지에 의존한다.

제어 추가 후 빌드 성공 및 pytest **48개 통과**를 확인했다. 횡오차·방향 부호,
조향 제한·변화율, 무효/오래된 경로 정지, 미션 해제 후 무발행, TF 실패 유지 정지,
파라미터 거부 및 실제 DDS Ackermann 메시지 수신을 테스트했다.
이는 가상 입력/메시지 검증이며 폐루프 실제 주행 성능 검증은 아니다.

## 중앙 경로 생성 실패 시 최근접 선분 투영

기존 법선 교점 방식의 중앙 경로가 검증을 통과하지 못하면 왼쪽 경계를
`center_sample_step`(기본 0.05m) 간격으로 샘플링한다. 각 샘플을 오른쪽의
모든 유효 선분에 투영하여 선분 내부 후보를 우선하고, 그중 거리가 가까운
유효 후보를 선택한다. 폭은 기존 `center_min_width`~`center_max_width`를 사용한다.

외삽하지 않으며 선분 밖의 투영은 끝점으로 고정한다. 끝점을 벗어난 거리 제한은
적용하지 않으며 동일 끝점의 반복 대응을 허용한다. 양쪽 진행 방향 일치, 안쪽 연결 방향, 오른쪽 누적 거리의
역행 방지, 대응선 간 교차를 검사한다. 중앙점은 대응점의 평균으로 계산하며 순서를
유지한다. 기존 평활화·레인 내부·교차·방향 변화 검증을 통과한 경로만 사용한다.
저장 폭이나 이전 경로는 사용하지 않는다. 최종 경로가 없으면 아래의 짧은 제어 유지 후 정지한다.

기존 RViz 대응선·중앙점·초록색 경로 표시를 재사용한다. 일반 로그의 `mode`는
`Normal Sections`, `Nearest Projection`, `Invalid`로 생성 방식을 표시한다.

## 경로 소실 시 짧은 제어 유지

두 경로 생성 방식 모두 실패하여 중앙 경로가 비었을 때 마지막 유효 조향각을
`control_hold_timeout`(0.30초) 이내에서 유지한다. 속도는 마지막 유효 명령 속도와
`control_hold_speed`(0.15m/s) 중 작은 값이다. 시간은 마지막 유효 제어 시점부터
steady 시간과 ROS 시간 모두로 검사하며 유지 명령은 이 시점을 갱신하지 않는다.
유효 경로가 복구되면 즉시 정상 오차 제어로 복귀한다.

이력 부재·유지 시간 만료·TF 오류·스캔 지연·미션 해제·기존 경로의 제어 오차 계산
실패 시에는 정지하고 이력을 삭제한다. 로그의 제어 mode는 `Path Feedback`,
`Hold Last Steering`, `Stopped`, `Idle`로 구분한다. 실제 폐루프 주행은 미검증이다.
