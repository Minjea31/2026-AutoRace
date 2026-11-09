# objavoid_ws

ROS 2 Humble 기반 LiDAR 장애물 회피 노드. 차선 유도는 `/lane_deep/target`을 사용하고 장애물은 LiDAR 클러스터로 판단한다.

## 실행

```bash
cd ~/2026-AutoRace/objavoid_ws
source /opt/ros/humble/setup.bash
PYTHONNOUSERSITE=1 /usr/bin/python3 /usr/bin/colcon build --symlink-install --packages-select autorace_objavoid
source install/setup.bash
ros2 run autorace_objavoid objavoid_node
```

카메라·LiDAR·lane_deep 입력을 먼저 실행한다. 노드 이름은 `objavoid_node`, 출력은 `/mission/objavoid/command_05` (`AckermannDriveStamped`)다. 기본 설정 파일은 `config/objavoid.yaml`이다. 실행 인자로 지정한 파라미터는 기본 설정을 덮어쓴다.

## 입력 및 제어

- `/scan`: 점과 클러스터로 차량 통과 폭의 충돌을 검사한다. 색상은 사용하지 않는다. 큰 클러스터도 장애물 박스로 검사한다.
- `/lane_deep/target`: 유효한 중심 위치·방향을 유도 및 복귀 기준으로 사용한다. 근거리 기준점과 목표점의 전방 거리가 같아도 정상 입력으로 수용한다.
- `/lane_fusion/deep_mask`: 웹 표시용이다. objavoid에서 노란 픽셀을 도로 경계로 재판정하지 않는다.
- `/camera/color/image_raw/compressed`: 웹 영상과 시작 흰선 ROI 판정에 사용한다.

시작 ROI에서 흰선과 compact 클러스터 2개 이상이 함께 확인되면 회피가 활성화된다. 시작 전과 `manual_active=false`에서는 주행 명령을 발행하지 않는다.

스캔 수신은 별도 콜백 그룹과 2스레드 executor에서 처리한다. 입력은 최신 한 개로 교체하고, 경로 계산은 고정된 스냅샷을 사용한다. 기본 `scan_timeout: 0.0`에서는 마지막 정상 스캔을 시간 만료시키지 않는다. 잘못된 프레임은 마지막 정상 데이터를 지우지 않는다.

## 경로 설정

| 항목 | 기본값 |
| --- | --- |
| 센서 ROI | 전방 2.5m, 좌우 각 0.75m |
| 우선 장애물 각도 | 정면 좌우 각 35° |
| 기본 / 최대 검사 길이 | 0.5m / 1.0m |
| 차량 길이 / 폭 | 0.322m / 0.22m |
| 차체 여유 | 전후·좌우 각 0.05m |
| 최대 회피 조향 | ±40° |
| 제어 주기 | 20Hz |

장애물 중심이 정면 ±35°를 벗어나면 다음 전방 위협으로 우선 대상을 전환한다. 우선순위에서 제외된 물체도 충돌 검사에는 남는다. 초기 조향과 후속 조향을 연결한 후보를 검사하고, 통과 여유가 비슷한 후보는 희망 조향과 이전 유효 조향의 차이를 비교한다. 후속 경로는 lane_deep 기준선의 위치·방향에 가까운 후보를 우선한다.

통과 후보가 없으면 검사 길이를 75%·50%·25%로 줄인다. 5cm 여유로도 후보가 없으면 동일 조건에서 차체 실치수로 재검사하고, 통과하면 `BODY_WIDTH` 상태로 0.2m/s를 발행한다. 이동·회전 중 샘플 사이 보정은 유지한다.

흰 점선을 넘어 옆 차로로 회피할 수 있다. 현재 차로 폭 0.52m를 도로 외곽으로 강제하지 않는다. `/lane_deep/target`에는 노란 도로 외곽 좌표가 없으므로 이 입력만으로 외곽 침범을 판정하지 못한다. 한쪽 경계에서 반대쪽 도로 폭을 추정하지 않는다.

검사를 모두 실패하거나 차선·스캔을 확인하지 못하면 유효 패스는 비우고 0.2m/s·조향 0 명령을 유지한다. 이 상태는 통과가 검증된 경로가 아니다.

## 웹 및 RViz

웹 주소는 `http://127.0.0.1:8085`다. 다른 PC에서 접속하려면 `-p web_host:=0.0.0.0`을 지정한다.

- 하늘색: 현재 유효 패스
- 흰색: 딥 마스크 검출 차선
- 빨강: 장애물 외곽
- 초록: 시작 및 흰선 검출 ROI

카메라의 패스와 장애물은 바닥 평면 투영이다. 투영 보정은 `config/camera_projection.yaml`을 사용하며 `web_lane_config`로 변경할 수 있다. 카메라 외삽 구간은 실제 영상과 오차가 생길 수 있다.

`/state`에는 `path_valid`, `scan_receiving`과 계획 시간·입력 경과·수신 개수·건너뛴 스캔 수가 포함된다. 건너뛴 스캔 수는 통신 유실 개수가 아니다. 경로 검사 실패 시 웹 패스와 `/objavoid/path`를 비우고 RViz 패스 마커를 삭제한다.

```bash
rviz2 -d src/autorace_objavoid/config/objavoid.rviz
```

## 검증

저장소 루트에서 ROS 환경을 적용한 뒤 실행한다.

```bash
PYTHONPATH="objavoid_ws/src/autorace_objavoid:$PYTHONPATH" /usr/bin/python3 -m pytest objavoid_ws/tests -q
```

설정·코드 변경은 objavoid 재시작 후 적용된다.
