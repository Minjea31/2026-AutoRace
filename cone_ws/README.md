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

현재 `node.py`에서 사용자가 조정한 기본값(m): X `(0.1, 0.5)`, Y `(-0.5, 0.5)`.
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
