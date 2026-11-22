# cone_ws — LiDAR 콘 경로 생성·추종

LiDAR로 콘 후보와 좌우 경계를 찾고 중앙 경로를 따라 주행합니다. 현재 통합 실행의 기준은 아래 C++ 알고리즘과 토픽입니다.

## 현재 알고리즘

LiDAR 점을 군집화해 콘 후보를 찾고 좌우 경계를 연결합니다. 양 경계의 중앙 또는 한 경계의 폭 오프셋으로 경로를 만들고, 횡오차·방향 피드백으로 추종합니다. 콘 구간이 끝나면 lane으로 복귀하는 자체 구현입니다.

![cone_ws 알고리즘 개념 그림](../docs/logic/cone_ws.svg)

## 입력·출력 토픽

| 방향 | 토픽 | 타입 | 내용 / 연결 |
| --- | --- | --- | --- |
| 입력 | `/scan` | `sensor_msgs/msg/LaserScan` | 콘 후보 거리값 |
| 입력 · TF | `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage` | 센서 변환·경로 메모리 좌표 변환 |
| 입력 | `/mission/lane` | `ackermann_msgs/msg/AckermannDriveStamped` | 탈출 시 차선 명령 |
| 입력 | `/mux/active_source` | `std_msgs/msg/String` | 실제 제어 주체 |
| 출력 | `/mission/cone/command_06` | `ackermann_msgs/msg/AckermannDriveStamped` | 콘 경로 명령 → mux |
| 출력 | `/cone_lane/exit_lane_active` | `std_msgs/msg/Bool` | 탈출 상태 → fusion 모드의 차선 인식 |
| 출력 | `/cone_lane/control_status` | `visualization_msgs/msg/Marker` | 제어 상태 표시·bringup 시작 검사 |
| 출력 · 관측 | `/cone_lane/roi_marker`, `/cone_lane/roi_points` | `visualization_msgs/msg/Marker` | ROI 경계·점 |
| 출력 · 관측 | `/cone_lane/clusters`, `/cone_lane/cone_centers` | `visualization_msgs/msg/MarkerArray` | 군집·후보 중심 |
| 출력 · 관측 | `/cone_lane/boundaries`, `/cone_lane/control_points` | `visualization_msgs/msg/MarkerArray` | 좌우 경계·제어 기준점 |
| 출력 · 관측 | `/cone_lane/center_path`, `/cone_lane/center_spot`, `/cone_lane/backup_path` | `visualization_msgs/msg/MarkerArray` | 중앙 경로·단면·보조 경로 |

현재 deep 모드의 deep_lite는 `/cone_lane/exit_lane_active`를 구독하지 않습니다. 이 연결은 fusion 모드에만 있습니다. `/cone_lane/control_status`는 Marker 타입이지만 표시 전용으로만 쓰이지는 않습니다.

## 상세 구현·개발 기록

아래는 초기 Python ROI 구현부터의 기록입니다. 현재 실차의 실행 파일·입출력은 위 C++ 요약을 기준으로 봅니다.

`cone_lane_follower`는 `/scan`을 차량 좌표계로 변환하고 ROI 내부 포인트를
연속 스캔 인덱스와 XY 거리로 클러스터링한다. 크기 필터를 통과한 클러스터는
**라바콘 후보**이며 확정된 라바콘이 아니다. 이 초기 ROI 단계에 좌우 분류·경로·제어가 추가되어 현재 C++ 노드로 사용된다. 최종 콘 명령은 `/mission/cone/command_06`이다.

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
`/mission/cone/command_06`에 `ackermann_msgs/msg/AckermannDriveStamped`를 발행한다.
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

실차 YAML의 `planner_async: true`는 커브 경로 생성을 별도 프로세스로 실행한다.
제어 노드는 결과를 기다리지 않고 최신 측정 경로 또는 유효한 기억 경로를 사용한다.
계산 중에는 다음 입력을 최신 1개로 교체하며, 스캔 시간·수신 시간이
`control_timeout`을 넘거나 경로 파라미터가 바뀐 결과는 사용하지 않는다.
결과는 계산 당시 odom 변환을 보존해 현재 스캔의 base_link 좌표계로 옮긴다.
`planner_budget_sec: 0.06`은 후보 탐색의 시간 예산이며, 초과하면 유효한
측정 경로를 반환한다. 두 파라미터는 시작 시에만 설정하며,
`planner_async: false`로 기존 동기 계산을 선택할 수 있다.

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
ros2 topic echo /mission/cone/command_06
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
