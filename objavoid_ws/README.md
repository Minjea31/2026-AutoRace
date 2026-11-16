# objavoid_ws — 장애물 회피 V3 (Gap 탐색 + 5차 다항식 Lattice + PD 추종)

3차로에 엇갈려 놓인 정차 LIMO 7대를 하나의 S자 경로로 연속 회피하고, 마지막 장애물을 지난 뒤 가운데 차로로 복귀하면 `DONE`을 내고 명령 발행을 멈추는 ROS 2 Humble 패키지 `autorace_objavoid_v3`.

- 기존 `objavoid_ws`(팀원), `objavoid_test_ws`(비교 실험), `lane_ws`, `lane_fusion_ws`, `mux_ws`, `simul/` 원본 코드·맵은 **수정하지 않았다.** (비교 실험에서 `objavoid_ws`는 소스 그대로, `objavoid_test`는 worktree 설치본 그대로 실행만 했다.)
- 로컬 PC 시뮬레이터에서만 검증했다. **실차 검증은 하지 않았다.**
- 흰 점선 자동 시작, 갈림길 주행은 구현하지 않았다(수동 활성화, DONE 후 다음 미션에 제어권 반환).

## 1. 파이프라인

```
/scan(10 Hz) ─┐   [작업 스레드: 최신 스캔만, 밀린 스캔은 버림]
/odom(50 Hz) ─┼─> 장애물 검출·추적 ──> Local Map ──> Free-Space/Gap ──> 경로 후보(5차) ──> 충돌·운동학 검사 ──> DP 선택
              │   perception.py        관측 격자       planner.py          planner.py          planner.py          planner.py
              │                                          │ 필요할 때만(재계획 정책: mission.py)
              │                                          ▼
              └──────────────────────────────> 저장 경로(odom 고정) ──> PD 제어 25 Hz(controller.py) ──> AckermannDriveStamped
                                                [제어 타이머: 계획 중에도 끊기지 않음]
```

| 파일 | 역할 |
| --- | --- |
| `road.py` | 도로 좌표(s 전방·d 왼쪽, 활성화 후 노란 실선으로 방향·중심 정렬), 사전 경계 `PriorBoundary.limits(s)` |
| `lane_boundary.py` | 카메라 노란 실선: 퓨전 딥 마스크 노란 클래스 ∧ 원본 HSV → 바닥 투영(퓨전 `lane.yaml` BEV) → odom 노란선 격자 → 경계 `limits(s)`·도로 정렬·갈림길 섬(구간 끝) |
| `perception.py` | LiDAR 클러스터링, 최소 크기 보정, odom 고정 추적, 통과 후 제거, 관측 격자(Local Map) |
| `planner.py` | Gap 탐색, 통로 노드, 5차 다항식 간선, 회전 차체 충돌 검사, 운동학 검사, DP |
| `controller.py` | `steering = atan(L·κ_ff) − K_y·e_y − K_ψ·e_ψ (− K_i∫e_y)`, 조향각·조향 변화율 제한, 속도 계획 |
| `mission.py` | ROS 독립 코어: 상태(IDLE/RUN/STOP/DONE), 재계획 정책, 안전 정지, DONE 판정, 측정 |
| `node.py`, `markers.py` | ROS 2 노드(스레드 분리), RViz 마커 |
| `params.py` → `config/objavoid_v3.yaml` | 모든 파라미터(YAML은 `tools/gen_config.py`로 생성, 단위 테스트가 일치 검사) |

## 2. 알고리즘

### 장애물 검출·추적 (`perception.py`)
- 스캔 시각의 odom 자세(50 Hz 이력 보간)로 점을 도로 좌표에 고정한다. 스캔이 늦게 와도 위치가 어긋나지 않는다.
- 도로 경계 + 6 cm 안, 구간 끝 + 0.6 m 이내 점만 각도 순서로 묶는다(인접 간격 12 cm, 3점 이상). 구간 끝 너머 갈림길 표지판도 실제 물체로 회피하되 DONE·통과 판정에서는 뺀다.
- 박스는 도로 축 정렬 박스(AABB, 비스듬한 차량도 포함하는 보수적 형태). **최소 크기 보정(0.33 × 0.23 m, LIMO 0.322 × 0.22)**은 센서가 장애물 앞에 있을 때만 한다. 길이는 앞면이 실제로 보일 때(같은 s에 폭 8 cm 이상 점들)만 뒤로 늘리고, 폭은 가려진 쪽(인접 빔이 더 가까운 쪽) 또는 센서 반대쪽으로 늘린다.
  - 시험 중 발견한 문제 두 가지를 고쳤다. ① 옆을 지날 때 대칭 확장을 하면 박스가 뒤로 계속 늘어나 0.32 m 차가 0.61 m가 됐다. ② 앞 차에 가려 뒷부분만 보인 조각을 앞면으로 착각해 0.33 m를 확장했다.
- 추적: 겹치거나 중심 거리 0.2 m 이내만 같은 장애물로 본다. 0.47 m 간격으로 붙어 있는 연속 차량을 합치지 않는다. 2회 관측 시 확정하고, 관측된 범위를 누적(합집합)한다. 시야 밖(후방 ±110° 밖)이나 가림 중에도 유지한다.
- 제거: ① 차량 뒤끝이 장애물 뒤끝 + 8 cm를 지남(=통과, 카운트) ② 미확정 0.5 s 미관측 ③ 시야 안인데 레이저가 박스를 1 s 관통(유령).
- Local Map: 구간 크기(약 7 × 2 m, 5 cm) 불리언 격자. **레이저가 실제로 지나간 칸만 '관측된 빈 공간'**이다. 무반사 빔과 장애물 뒤 그림자는 미관측으로 남는다.

### Free-Space / Gap 탐색과 경로 후보 (`planner.py`)
- 정거장(층) = 지평 2.5 m 안 장애물 중심 s(그 옆을 지나는 위치, 15 cm 이내는 병합) + 지평 끝. 0.9 m보다 벌어진 곳에는 중간 층을 둔다. 최대 6층.
- 지평 끝 상한: 경로 끝에서 차 앞끝이 구간 끝 + 0.1 m. 계획기는 갈림길 표지판을 비켜 가지 않고 가운데 차로로 복귀한다. 처음에는 표지판을 장애물로 피하느라 7대 배치에서 d≈−0.2에 머물렀다. 단, 구간 장애물이 구간 끝에 붙어 있으면(1열) 그 장애물을 다 지날 때까지 연장하고, 이때는 표지판도 피한다.
- 각 층에서 차체 길이 구간에 걸치는 장애물로 차량 중심이 갈 수 있는 빈 구간을 계산한다. 막힌 구간은 `장애물 d범위 ± (차폭/2 + 2.5 cm)`, 도로 구간은 `경계 ∓ (차폭/2 + 2 cm)`다. 그래서 **차폭 + 양쪽 여유보다 좁은 틈은 자동으로 제외**된다.
- 노드(층당 최대 10개): 빈 구간 중심, 장애물에서 편안한 여유(12 cm) 위치, 좁은 통로용 양 끝, 구간 안 차로 중심, 가운데 차로, 기존 경로 위치, 현재 진행 방향 직진 연장 위치. 3개 차로 중심만 쓰지 않는다.
- 간선: `d(s)` 5차 다항식. 시작은 현재 경로(또는 차량)의 (d, d′, d″)와 연속이고, 끝은 (d, 0, 0)이다. **층을 최대 3개 건너뛸 수 있다.** 처음에는 모든 층에서 d′=0을 강제했는데, 차로 변경을 계단 모양으로 쪼개 곡률 한계를 넘는 문제가 있어 이렇게 바꿨다.
- 검사(층 쌍마다 numpy 한 번):
  - 곡률 ≤ 1/0.55 m (차량 최소 회전반경 0.462 m)
  - 조향 변화율 `|dδ/ds|·0.15 m/s ≤ 1.0 rad/s`. 더 빨리 달릴 구간은 제어기가 `v ≤ 0.8·δ̇max/|dδ/ds|`로 감속해 실제 조향 변화율 한계(1.2, 차량 1.5 rad/s)를 지킨다. 처음엔 v_max 기준으로 검사했는데, 정지 상태에서 남은 조향을 짧은 거리에서 풀어야 할 때 모든 경로가 탈락해 바꿨다.
  - 헤딩 ≤ 0.9 rad
  - **회전 차체 직사각형 ↔ 장애물 박스 분리축(SAT) 간격 ≥ 2.5 cm**
  - 차체 네 모서리 ↔ 노란 경계 ≥ 2 cm (출발 직후 0.3 m는 여유를 완화)
- 비용: 가운데 차로 이탈², 곡률², 곡률 변화², 12 cm 미만 근접 페널티, 앞 1 m 구간의 기존 경로 이탈²(유지 압력), 미관측 노드 페널티, 종단 이탈². DP로 최소 비용 경로를 고른다. 지평 끝에 도달하는 경로가 없으면 갈 수 있는 가장 먼 층까지의 **부분 경로**를 내고, 차량은 그 끝에서 정지한다.
- 관측 검증 길이: 경로를 따라 차체 중심·양옆이 모두 관측된 빈 공간인 마지막 지점까지만 '검증됨'이다. 속도는 그 지점 앞에 멈출 수 있도록 `v ≤ √(2·a·(남은 검증 길이 − 0.12))`로 제한한다. 미관측 영역을 비어 있다고 가정하지 않는다.

### 경로 유지·재계획 (`mission.py`)
경로는 도로 좌표(odom 고정 변환)에 저장한다. 스캔마다 저장 경로를 다시 검사하고(수 µs), 아래 조건일 때만 계획한다.

| 원인 | 조건 | 유지 압력 |
| --- | --- | --- |
| `no_path` / `blocked` / `deviation` | 경로 없음 / 저장 경로가 현재 장애물·경계와 1.5 cm 이상 침범 / 추종 오차 > 12 cm | 없음(새로 계획) |
| `extend` | 남은 경로 < 1.4 m (부분 경로면 0.5 s 간격) | 있음 |
| `new_obstacle` | 확정 장애물 집합 변경(0.3 s 간격) | 있음 |
| `incomplete` / `unverified` | 부분 경로(0.5 s) / 검증 길이 < 1 m(1 s) | 있음 |

- 유지 압력은 앞 1 m에서 기존 경로와의 차이²에 가중치 8을 곱한 비용이다. 새 경로가 조금 낫다는 이유로 경로를 바꾸지 않는다. 지평 밖에서 계획된 꼬리 부분은 묶지 않는다(처음엔 묶었다가 복귀가 늦어져 수정).
- 새 경로는 추종 오차가 5 cm 이내면 기존 경로의 현재 지점 상태 (d, d′, d″)에서, 아니면 차량 자세와 현재 조향 곡률에서 시작한다. 그래서 조향이 튀지 않는다.
- 장애물 옆을 다 지나기 전 복귀: 경로 전 구간을 차체로 검사하므로 뒤끝이 걸리는 복귀 경로는 자동으로 탈락한다. 12 cm 근접 비용으로 아슬아슬한 복귀도 피한다.

### PD 제어 (`controller.py`, 25 Hz 고정 주기)
- 기준점은 base_link를 경로 선분에 수직 투영한 점이다. `e_y`는 경로 법선 방향 거리(왼쪽 +), `e_ψ = ψ − ψ_ref`다.
- `δ = atan(L·κ(s_ref + v·0.12 s)) − 1.3·e_y − 1.0·e_ψ`
  - 되먹임 합은 ±0.30 rad로 제한하고, 적분항은 기본으로 끈다(`k_i: 0.0`).
  - 선형화 결과(자전거 모델) 공간 고유주파수는 약 2.55 /m, 감쇠비는 약 0.98이다.
- `|δ| ≤ 0.40 rad`(시뮬 등가 한계 0.4086), `|δ̇| ≤ 1.2 rad/s`(차량 1.5)
- 속도:
  - 기본 `v_max`(0.3 m/s)
  - 앞 0.5 m 최대 곡률에 비례해 최대 50%까지 감속
  - 장애물 간격이 7 cm 미만이거나 추종 오차가 크면 0.2 m/s
  - 검증 길이 기반 정지 가능 속도
  - 가속 0.5, 감속 0.8 m/s²

### 안전 정지 (속도 0을 계속 발행해 MUX/어댑터가 차선 입력으로 넘어가지 않게 함)
`scan_stale`(0.4 s), `odom_stale`(0.25 s; 그 이내는 odom 속도로 외삽), `emergency`(차체 앞 6 cm + 0.35 s·v 띠에 LiDAR 점 3개 이상 — 경로·추적과 무관한 마지막 방어선), `no_path`, `path_blocked_ahead`(부분 경로 끝), `path_deviation`(20 cm). 조건이 풀리면 자동 재개.

### 시작·종료
- 시작: `/objavoid_v3/activate`(Bool true) 수동. 그 순간 odom 자세로 도로 좌표를 고정한다. 동작 중 재활성화는 무시한다(false 후 true로 재시작).
- DONE 조건은 아래를 모두 만족해야 한다.
  - 구간 장애물 1대 이상 통과
  - 구간 안에 남은 확정 장애물 없음(=마지막 장애물 뒤끝 통과)
  - 차량 앞 0.8 m(구간 끝까지) 현재 차선 폭이 관측된 빈 공간
  - 그리고 다음 중 하나:
    - `centered`: 가운데 차로 ±8 cm, 헤딩 ±0.12 rad를 0.3 s 유지
    - `section_end`: 차량 앞끝이 구간 끝(활성화 위치 + 4.25 m, 갈림길 섬 끝 바로 앞) − 0.1 m에 도달. 마지막 장애물이 구간 끝에 가까워 복귀 거리가 부족한 경우다. 결과 표에 그 시점 횡위치를 함께 적는다.
- DONE이면 `/objavoid_v3/done`=true(latched)를 내고 명령 발행을 멈춘다. MUX/어댑터는 다음 입력(차선)으로 넘어간다. 갈림길 표지판을 정면으로 볼 수 있는 가운데 차로 자세로 넘겨주는 것이 목표다.

### 맵 정보 사용 범위
- 장애물 위치는 맵에서 가져오지 않는다. `/scan` + `/odom`만 쓴다. `/sim/ground_truth`와 맵 장애물 좌표는 `tools/evaluator.py`, `tools/metrics.py`, `tools/offline_sim.py`의 **평가에만** 쓴다.
- 사전 설정: 도로 경계(노란 실선 안쪽 y=0.171~1.708 → d=±0.768, 입구 깔때기 s=0~0.5에서 ±0.26→±0.768), 구간 길이 `section_end=4.25`. 활성화 위치가 3차로 입구 가운데 차로(시뮬 `--mission obstacle` 시작점)라고 가정한다. 실차에서는 `road.PriorBoundary`를 카메라 노란 실선 모듈로 바꾸면 된다(같은 `limits(s)` 인터페이스).

## 2-1. 2026-11-14 영상(경로 사라짐) 원인과 수정

영상은 `./scripts/node.sh rviz:=true`로 V3를 켜고 시뮬레이터에서 **수동 조종**한 장면이다. 배치는 편집기로 바꾼 `scenarios/user7.obstacles.yaml`이다(8열 가운데+아래, 6열 위+가운데, 4·3열 아래, 3열 가운데). 같은 배치·자세로 재현했고(`tools/offline_sim.py`의 수동 운전자 모드, 계획기 탈락 사유 집계), 원인은 다음과 같았다.

| # | 원인 | 근거(재현) | 수정 |
| --- | --- | --- | --- |
| 1 | **경로를 벗어난 뒤 재계획의 시작 곡률을 V3 자신의 마지막 조향 명령에서 가져옴.** 수동 조종처럼 명령이 차에 적용되지 않거나 크게 벗어나면 −5.3 1/m(반경 0.19 m) 같은 불가능한 값이 됐고, 회전반경 한계로 자르지도 않아 모든 후보가 첫 샘플에서 탈락 → `path=None` → RViz 경로 사라짐, `no_path` 정지 | 수동 주행 중 경로 유지율 16~24 %, 탈락 집계 curvature 2,919건 | 시작 곡률 = odom 각속도/속도(정지 중엔 직접 몰던 경우만 명령 조향, 아니면 0), 회전반경 한계로 제한 (`mission._start_state`) |
| 2 | **도로 좌표·노란 경계를 활성화 순간의 차 자세에 고정.** 곡선 위나 비스듬한 자세에서 켜면 경계·ROI·경로가 실제 도로에서 회전 | 0.5 rad 틀어진 활성화 → 조향 0으로 도로 밖 직진, 엉뚱한 곳에서 `section_end` DONE | 카메라 노란 실선으로 도로 방향·중심 정렬(RANSAC 직선 → 평행 묶음 → 선 쌍 중점 또는 한 선+사전 모델). 정렬 전에는 출발하지 않음(`lane_unaligned`) |
| 3 | 재정렬 때 회전된 장애물 박스를 축정렬 박스로 감싸 폭이 L·sin(Δψ)만큼 부풀어(0.19 rad → +6 cm) 0.29 m 틈이 닫힘 | 편집 배치에서 위 차로 막다른 길로 진입 | 큰 재정렬(>0.05 rad)이면 박스를 버리고 새 좌표계에서 재검출 |
| 4 | 기본 시작점(8.55)에서 편집 배치는 **기하적으로 통과 불가**(8열 두 차로가 0.6 m 앞에서 막힘, 남은 위 차로로 0.25 m 옮기기에 0.41 m뿐 → 반경 0.462 m로 최대 0.10 m) | — | 수정 대상 아님: 안전 경로가 없으므로 정지가 맞다. 흰 선 앞(0.3~0.5 m 뒤)에서 활성화하면 통과 |

경로 계획 → 저장 → 25 Hz PD → `/objavoid_v3/cmd` 발행 연결 자체는 정상이었다. RViz 표시 문제가 아니라 실제 계획 실패(`path=None`)였고, 그동안 V3는 속도 0을 계속 냈다(어댑터·MUX는 차선 입력으로 넘어가지 않음).

## 2-2. 차선(노란 실선) 통합

- 입력은 퓨전 차선 인식 결과를 재사용한다(새 검출기를 만들지 않음).
  - `/lane_fusion/deep_mask`(퓨전 딥 마스크 노드)의 노란 클래스를 쓴다.
  - **같은 촬영 시각의 원본 영상**에서 퓨전 `lane.yaml`의 HSV 노란 범위(H 15~40, S·V ≥ 80)로 한 번 더 확인한다. 딥 모델이 흰 점선을 노랑으로 오분류하는 경우를 거른다. 원본이 없으면 그 마스크는 쓰지 않는다.
  - 바닥 투영은 퓨전 `lane.yaml`의 BEV 보정(사다리꼴 → 미터)과 같은 식이다. 다만 BEV 직사각형(좌우 ±0.73 m)만 쓰면 시작 위치에서 외곽선(±0.77 m)이 안 보여서, 영상 ROI 전체(전방 2.5 m, 좌우 1.5 m)를 쓴다. 실차는 `lane_config:=.../config/real/lane.yaml`.
- 누적: odom 고정 3 cm 격자에 "카메라가 본 횟수 / 노란 횟수"를 쌓는다. 2회 이상이면서 30 % 이상인 칸만 노란선으로 확정한다. 한쪽 선만 보이거나 잠깐 사라져도 기억을 쓴다.
- 경계: s 5 cm 칸마다 가운데에서 가장 안쪽 노란선(선 반폭 안쪽)을 좌/우 경계로 쓴다(넘을 수 없음). 흰 점선은 쓰지 않으므로 회피 중에 넘을 수 있다. 관측 안 된 곳은 사전 경계를 쓰되, 차체가 그 경계에 15 cm 안으로 다가가는 구간은 **미검증**으로 보고 감속·정지한다(확인 못 한 경계를 안전하다고 보지 않음).
- 도로 정렬: 입구(활성화 후 1 m)에서만 한다. 첫 정렬은 입구 차로 쌍(0.52 m)·도로 쌍(1.537 m)·한 선+사전 모델을 허용하고, 이후는 도로 쌍으로만, 두 번 연속 일관될 때만 한다. 갈림길 근처 선으로 중심이 틀어지는 것을 막기 위해서다.
- 구간 끝: 입구 이후 가운데(|d|<0.3)에 처음 나타나는 노란선(갈림길 섬 끝)으로 정한다. 한 번 찾으면 앞당기기만 한다. 섬 선은 이 미션의 외곽 경계에서 뺀다. 경로는 섬 끝 + 0.1 m(차 앞끝 기준)까지만 만들어, 표지판을 정면에 둔 가운데 차로 자세로 DONE한다.

## 3. 빌드와 실행 (ROS 2 명령)

```bash
cd ~/dev/2026-AutoRace/objavoid_ws
source /opt/ros/humble/setup.bash
PYTHONNOUSERSITE=1 colcon build --symlink-install     # 심볼릭 설치 필수(launch가 저장소의 lane_fusion_ws BEV 파일을 찾음)
source install/setup.bash
```

### 시뮬레이터 (터미널마다 `export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1 PYTHONNOUSERSITE=1`)

```bash
# 1) 시뮬레이터(conda 환경: simul/env.sh)
source ~/dev/2026-AutoRace/simul/env.sh
python -m limo_parking_sim.app --mission obstacle --view follow --map ~/dev/2026-AutoRace/objavoid_ws/scenarios/user7.yaml --spawn 9.05 1.0 3.1416
# 2) 어댑터: V3 전용 토픽 연결(같은 도메인에 기존 objavoid 노드가 없어야 함)
source ~/dev/2026-AutoRace/simul/env.sh
python -m limo_parking_sim.mission_adapter --no-gui --topic /objavoid_v3/cmd
# 3) 퓨전 딥 마스크(lane_fusion_ws 소스 그대로, onnxruntime이 ~/.local이라 PYTHONNOUSERSITE 해제)
unset PYTHONNOUSERSITE; source /opt/ros/humble/setup.bash
PYTHONPATH=~/dev/2026-AutoRace/lane_fusion_ws/src/autorace_lane_fusion_deep python3 -c 'from autorace_lane_fusion_deep.mask_node import main; main()' \
  --ros-args -p use_sim_time:=true -p model_path:=$HOME/dev/2026-AutoRace/lane_fusion_ws/src/autorace_lane_fusion_deep/model/lane_640_mask.onnx
# 4) V3 + RViz
ros2 launch autorace_objavoid_v3 objavoid_v3.launch.py rviz:=true
# 5) 활성화 / 중지(다시 시험: 시뮬 F5 -> false -> true)
ros2 topic pub --once /objavoid_v3/activate std_msgs/msg/Bool "{data: true}"
```

### 실차 (센서·`tensor_bringup` 퓨전 모드가 `/scan`·`/odom`·`/lane_fusion/deep_mask`를 낼 때)

```bash
ros2 launch autorace_objavoid_v3 objavoid_v3.launch.py profile:=real rviz:=true
ros2 topic pub --once /objavoid_v3/activate std_msgs/msg/Bool "{data: true}"
```

- `profile:=real`: 시스템 시간, 바닥 투영은 `lane_fusion_ws/.../config/real/lane.yaml`(06번 차 BEV 실측값). 다른 차는 `lane_config:=<그 차 lane.yaml>`.
- 출력 기본값 `/objavoid_v3/cmd`는 MUX에 연결돼 있지 않다. 그래서 차는 움직이지 않고 수동 조종 중에 경로만 RViz로 볼 수 있다. 수동 조종으로 경로를 벗어나면 `path_deviation` 상태가 되고 현재 자세에서 다시 계획한다. 주행에 쓰려면 기존 `objavoid_node`를 끈 뒤 `command_topic:=/mission/objavoid/command_05`.
- 상태: `ros2 topic echo /objavoid_v3/state`(JSON: 상태·정지 원인·재계획 수·계획 실패 사유 `last_fail`/`fail_checks`·차선 정렬 `lane`), 완료: `ros2 topic echo /objavoid_v3/done`.

### RViz (`/objavoid_v3/markers`, 고정 프레임 `odom`)

| namespace | 내용 |
| --- | --- |
| `obstacles` / `obstacle_label` | 추적 장애물 박스(빨강=확정, 주황=미확정), ID와 추정 크기 |
| `gaps` / `gap_nodes` | 정거장별 통과 가능한 실제 통로(초록 선분), 경로 노드(파랑 점) |
| `candidates` | 충돌·운동학 검사를 통과한 후보 간선(회색, 최대 40) |
| `path` / `verified_end` | 최종 경로(청록), 관측 검증 끝(노랑 구) |
| `lane_boundary_observed` / `lane_boundary_prior` / `lane_yellow_cells` | 카메라로 관측된 노란 경계(노랑 굵은 선) / 관측 안 된 사전 경계(회색) / 확정 노란 칸(주황 점) |
| `ego` / `target` | 차체 외곽, 제어 기준점 |
| `boundary` / `section_end` | 사전 도로 경계, 구간 끝 |
| `state` | `RUN/STOP/DONE: 원인`, 재계획 수, 통과 수 |

## 4. 테스트 도구

| 명령 | 내용 |
| --- | --- |
| `PYTHONNOUSERSITE=1 PYTHONPATH=src/autorace_objavoid_v3 python3 -m pytest -q src/autorace_objavoid_v3/test` | 단위 테스트 13개(5차식 경계조건, SAT 간격, Gap 폭 필터, 최소 크기 보정, 연속 차량 분리, 관측 격자, PD 수렴, YAML 일치) |
| `PYTHONNOUSERSITE=1 bash -c 'source ../simul/env.sh >/dev/null && python tools/offline_sim.py scenarios/*.yaml'` | 개발용 빠른 폐루프(시뮬 운동·LiDAR 모델 import, ROS 없음, 시나리오당 3~8 s) |
| `python3 tools/run_sim_tests.py --suite all` | **최종 검증**: headless Panda3D + 어댑터 + 실제 노드 + 평가기, 시험마다 격리 도메인(83~87) |
| `python3 tools/run_sim_tests.py --suite compare --impl test\|team` | 같은 시나리오·평가기로 `objavoid_test`(worktree 설치본) / `objavoid_ws`(소스) 실행 |
| `PYTHONNOUSERSITE=1 bash -c 'source ../simul/env.sh >/dev/null && python tools/bench.py'` | 계산 시간 벤치마크(무작위 7대 1000회, 최악값 포함) |
| `python3 tools/make_scenarios.py` / `python3 tools/gen_config.py` | 시나리오 맵 생성(원본 맵 복사 후 장애물만 교체) / YAML 재생성 |

- 고장 주입: `tools/fault_injector.py`. 스캔 누락 확률, 스캔·odom 차단 구간, 지연, 특정 영역 LiDAR 숨김(새 장애물 등장)을 넣는다.
- 평가: `tools/evaluator.py`. 정답 자세로 차체-장애물 최소 거리, 노란선 여유, 충돌, 조향·조향률·진동, 명령 주기를 잰다.


## 중앙 섬 매핑 및 표지판 관측 회전 (C++ / car04)

`fork_shape: true`는 노란 마스크+원본 HSV 관측으로 길이 1.687m, 폭 0.500m의 대칭 섬을 맞춥니다.
R40 모서리는 보수적인 육각형 충돌 외곽 안에 포함합니다. 도로 폭은 1.540m입니다.
서로 다른 방향의 변이 없는 단일 직선은 확정하지 않습니다. 임시 추정과 3프레임 확정을 구분합니다.
보라선 근처에서 표지판을 못 보면 먼저 섬 앞부분을 더 볼 방향을 선택하고,
그 방향으로 곡선 회전하며 목표 선속도 0.2m/s로 움직입니다. 제자리 회전이 아닙니다.
관측 경로는 차량 전체의 섬·도로·장애물 여유와 라이다 관측 여부를 검사합니다.
유효 경로가 없거나 센서가 끊기면 정지합니다. 표지판은 앞끝 주변에 있을 수 있으며 정확한 위치를 고정하지 않습니다.
표지판 방향이 확정되면 선택 통로의 lattice로 연결하고, 진입 안정 및 신선한 `/mission/lane`을 확인한 뒤 인계합니다.
`impl:=py`는 기존 갈림길 방식을 사용합니다. 새 섬 모드는 C++ 전용입니다.

### bag을 반복 재생하며 새 결과 보기

```bash
cd ~/2026-AutoRace/objavoid_ws
bash scripts/replay_view.sh ~/AutoRace_bag/rosbag2_2026_11_16-08_49_37
```

브라우저: **http://localhost:8086/**. bag·lane 마스크·새 C++ objavoid·읽기 전용 웹뷰어를 함께 실행합니다.
bag은 기본 0.5배속으로 `--clock --loop` 재생됩니다. 끝나면 처음부터 반복하며,
시계가 되감길 때 이전 회차의 섬 지도·IMU 적분·시작 플래그를 초기화합니다.
Ctrl+C로 이 실행에서 만든 프로세스만 종료합니다. 한 번만 재생하려면 `REPLAY_LOOP=false`를 지정합니다.
`RATE=1`로 속도, `PORT`로 웹 포트, `REPLAY_DOMAIN`으로 ROS 도메인(기본 109)을 변경할 수 있습니다.
새 마스크·경로·마커·상태는 `/objavoid_eval`로 분리해 녹화된 옛 결과와 섞이지 않습니다.
웹뷰어는 새 계산 결과만 구독합니다. 차량 명령은 `/objavoid_eval/command`로만 출력합니다.
센서 노드·MUX·차량 드라이버는 실행하지 않습니다.
로그는 `objavoid_ws/.replay/{bag,mask,planner,viewer}.log`에 저장됩니다.
주황 경로는 관측 회전, 노란 사각형은 표지판 예상 영역, 육각형은 섬 모델입니다.

섬은 매 프레임의 실제 관측으로 재평가합니다. 이전 누적 지도만 반복해서 확정하지 않습니다.
외곽선 후보를 제외하고 차량과 겹치는 가설은 거부합니다. 관측이 계속 맞지 않으면 확정을 풀고 재탐색합니다.
일관된 새 관측은 위치·방향에 점진적으로 반영하며 짧은 가림에서는 기존 위치를 유지합니다.
bag의 차량은 새 명령을 따라 움직이지 않으므로 인식·계획 검증에 사용합니다.

### 빌드·검증

```bash
bash scripts/build.sh
source /opt/ros/humble/setup.bash
colcon test --packages-select autorace_objavoid_v3_cpp
colcon test-result --verbose
/usr/bin/python3 tools/validate_fork_replay.py ~/AutoRace_bag/rosbag2_2026_11_16-*/
```

재생용 Python은 `.replay-venv`의 시스템 Python 3.10과 ONNX Runtime을 사용합니다.
모델은 기존 `lane_fusion_ws` ONNX를 사용하며 기존 전역 Python 환경은 변경하지 않습니다.

도면 기준 형상은 obj 활성화부터 표시하고, 기존 외곽선·점선 정렬을 시작부터 사용합니다.
섬 맞춤도 초반부터 시도하지만 시야 밖의 섬을 관측했다고 간주하지 않습니다.
회색 섬은 도면상 초기 기준, 실제 관측점에 맞춘 육각형은 별도의 추정 결과입니다.
평행선만으로 앞뒤 위치는 확정할 수 없으며 odom으로 이어가고 꼭짓점·사선 관측으로 보정합니다.
저장소의 `simul/src/limo_parking_sim/assets/track_geometry.json`은 전체 CAD 참조 자료이며,
현 구현은 전체 트랙의 절대 좌표를 복구하는 SLAM이 아니라 obj 활성화 기준의 국소 정합입니다.


### 노란 진입로 형상으로 자동 시작

04번 C++ 설정의 `start_entrance: true`는 좁은 입구(0.50m)에서 사선으로 넓어져
넓은 도로(1.54m)의 직선 외곽선으로 이어지는 모양을 기존 자동 시작 조건에 추가합니다.
마스크+HSV의 기존 바닥 관측점을 재사용하므로 별도 추론은 없습니다.
좌측 또는 우측 한쪽 경계만으로도 진입 형상을 확인합니다. 이 새 조건에는 흰 점선 쌍이 필요 없습니다.
기본 3개의 서로 다른 마스크 프레임에서 형상이 일관되면 후보 플래그를 5초간 켭니다.
그 시간 안에 전방에서 장애물 2개 이상을 관측하면 시작합니다. 후보를 연속해서 봐도 최초의 5초를 무한 연장하지 않습니다.
노란 사선 하나, 평행 직선, 좁아지는 출구 모양은 이 조건으로 시작하지 않습니다.
최종 조건은 **기존 시작 조건 OR (노란 진입 후보의 5초 유효 시간 내 장애물 2개 이상)** 입니다.
후보를 본 뒤 움직여도 검출 당시의 odom 입구 좌표를 보존해 활성화 원점을 계산합니다.

- `/objavoid_v3/entrance_armed` (`Bool`, transient local): 노란 진입 형상을 보고 열린 5초 후보 상태.
- `/objavoid_v3/active` (`Bool`, transient local): obj 미션이 활성화돼 명령을 담당하려는 상태.
- `/objavoid_v3/has_control` (`Bool`, transient local): 실제 `/mux/active_source`가 `objavoid`이고 최신 상태인 경우.

우선 미션이 활성화되면 obj가 active여도 MUX 제어권은 다를 수 있습니다.
bag 평가 스크립트는 MUX를 실행하지 않으므로 has_control은 false가 정상입니다.
현재 명령은 `/mission/objavoid/command_05`이며 MUX 순서는 횡단보도 → 신호 → 주차 → 서행 → obj → 콘 → 차선입니다.

노란 진입 형상으로 시작하면 검출한 좁은 입구 쪽 모서리의 위치·방향을 도로 좌표 원점으로 사용합니다.
따라서 활성화 당시 차량이 좌우로 치우치거나 비스듬해도 도면 기준과 정렬해 시작합니다.
재생 스크립트는 기본적으로 자동 시작 조건을 그대로 검증합니다.
시작 조건을 건너뛰고 첫 odom 자세에서 경로를 관찰하려면 `REPLAY_ACTIVATE=true`를 지정합니다.


### bag에서 진입 형상만 독립 검사

```bash
source /opt/ros/humble/setup.bash
.replay-venv/bin/python3 tools/check_entrance_bags.py ~/AutoRace_bag/rosbag2_2026_11_16-*/
```

기존 lane ONNX 추론 후 실제 C++ GroundProjection과 진입 검출기를 호출합니다.
카메라 원본·마스크·바닥 좌표 비교 이미지와 프레임별 결과는 `.replay/entrance_validation/`에 저장합니다.
실제 자동 시작을 새 조건만으로 확인하려면 `REPLAY_LEGACY_START=false bash scripts/replay_view.sh`를 사용합니다.
일반 실행의 기본값은 기존 조건 OR 새 조건입니다.


### 도로 외곽 / 섬 독립 매칭 (2026-11-16)

04번 C++ 설정의 `road_outline: true`는 도면의 도로 외곽(입구 확장, 양쪽 직선,
출구 축소)을 매칭하고 중앙 섬 후보점은 정렬에서 제외한다. 도면 템플릿 길이는
`road_outline_length: 6.665`, 넓은 직선 길이는 `road_outline_straight: 5.529` m이다.
원점은 입구가 넓어지기 시작하는 지점이다. 섬은 1.687 × 0.5 m 별도 모델로
관측 위치를 찾으며 `section_end` 보라선과의 간격을 고정하지 않는다.
직선만 보이면 앞뒤 위치는 보정하지 않는다. 노란 외곽선이 부족할 때에는 섬 점을 제외한
외곽 직선과 흰 점선으로 초기 방향·좌우 중심을 보정한다. 도면은 완전한 트랙 전체가 아니라
장애물 회피 구간의 양쪽 외곽만 포함한다.

`lane_full_image: true`는 기존 아래쪽 ROI 제한을 해제한다. 딥 마스크와 같은 시각의
HSV 교집합을 가로/세로 **2픽셀 간격(약 25%)**으로 균등 추출한다. 중심 기준
최근접 차선 선택은 하지 않는다. `lane_max_range: 2.0`으로 **전방 2m 초과는 제외**하고
좌우는 `lane_max_lateral: 1.5` m까지 사용한다. 원본 이미지는 절반 해상도로 디코딩한다.
마커는 추출된 점을 추가로 절반씩 생략하지 않는다. 누적 확정점과 현재 딥∩HSV 점은
별도 레이어이며 LiDAR 가림 필터 전 관측점도 확인할 수 있다.

매칭에는 최근 0.5초를 odom 정렬하고 2cm 격자로 중복 제거한 점을 사용한다.
도로 최적화는 최대 240개(3cm 격자), 섬은 최대 96개로 제한한다. 새로운 이미지에
실제 관측점이 있어야 확정 횟수를 늘린다. 확정된 섬 앞 모서리가 카메라 관측 영역을
벗어나면 odom 위치를 유지한다. 카메라가 보는 영역에서 매칭이 실패하면 기존의
미검출 해제 절차를 따른다.

웹 뷰어: 하늘색 도로 외곽/매칭점, 보라색 섬 후보점/확정 윤곽, 회색 미분류점.
`road_mapping.ms`는 매칭 및 경계 갱신 시간, `island_mapping.rejection`은 탈락 이유,
`section_offset_cm`는 구간 끝 기준 섬 앞 꼭짓점의 부호 있는 거리다.
전체 윤곽 매칭은 2m 이내의 보이는 부분으로 수행하며 한 프레임에 전체 코스가 보일
필요는 없다. 실측 기준이 없는 bag 재생은 위치 정확도나 실제 통과 성공을 입증하지 않는다.


로컬 재생 확인:

```bash
cd ~/2026-AutoRace/objavoid_ws
bash scripts/build.sh
bash scripts/replay_view.sh
```

브라우저에서 http://localhost:8086 를 연다. 실행한 터미널에 포커스를 두고
**Space**로 bag 일시정지/재개, **Ctrl+C**로 전체 재생 세션을 종료한다.
`REPLAY_START_PAUSED=true bash scripts/replay_view.sh`로 정지 상태에서 시작할 수 있다.
기본 bag은 `~/AutoRace_bag/rosbag2_2026_11_16-08_49_37`, 기본 속도는 0.5배/반복이다.
다른 bag은 첫 번째 인수로 전달한다. TTY가 없는 자동 검증에서는 키보드 제어를 끈다.
플랫폼 복사/재시작은 이 스크립트가 수행하지 않는다.


### 잡음 대응 정렬 / 빠른 섬 확인 / YOLO lattice (2026-11-16)

도로 매칭 **입력 선택에서 지도까지의 거리 제한(기존 25cm)을 제거**했다. 전방 2m
제한과 2픽셀 샘플링은 유지한다. 입력 전체를 3cm 격자로 균등화하고 고립된 점을
제거한 뒤 여러 초기 위치·방향에서 robust point-to-line 정렬을 수행한다.
큰 잔차는 가중치를 낮추고, 일치하지 않는 외곽점도 후보 점수에 반영한다.
섬 영역은 각 후보 좌표계에서 별도 분류하며 도로 정렬의 근거로 사용하지 않는다.
유효점 비율 50% 이상과 절사 RMS 4cm 미만을 요구한다. 이는 사용한 관측점의
잔차 기준이며 실제 절대 위치 오차를 보증하지 않는다. 직선만 보일 때는 앞뒤
위치를 보정하지 않는다. `road_mapping.inlier_ratio`와 `error`로 확인한다.
초기 관측 정렬 전에는 사전 지도를 사용해 계속 계획·제어한다. 관측 매칭이 성공하면
지도 좌표계를 갱신하고, 실패한 동안에는 마지막으로 맞춘 지도(없으면 사전 지도)를 유지한다.
회색 `lane_boundary_prior`는 계획용 사전 경계이고 하늘색 `road_outline`은 매칭 도면이다.

섬은 최초 후보 이후 위치·방향 근방 27개 가설을 재사용한다. 최초 탐색과 후속 확인을
분리하고, 모델 좌표계의 제곱거리와 미리 계산한 변 길이를 사용한다.
`island_mapping.evaluations`로 가설 평가 횟수를 확인한다. 3개의 새 이미지에서
일관된 모서리가 확인돼야 최초 확정한다.

기존 YOLO 인터페이스를 유지한다:

- `/yolo/LR_sign`, `std_msgs/msg/Bool`: **false = left/A, true = right/B**.
- 04번은 YOLO가 확인한 이벤트 1개로 방향을 선택한다(`fork_sign_min: 1`).
- 활성화 직전 1초 이내의 이벤트도 보존한다. 섬 미확정이면 방향을 저장하고 관측을 기다린다.
- 방향을 바꾸면 기존 경로를 버리고 선택한 측면으로 lattice를 다시 계산한다.
- 섬 1.2m 전부터 목표 횡위치를 부드럽게 이동시키며, 선택한 쪽 통로와 차체 전체의
  섬·외곽·장애물 간격을 검사한다. 유효 경로가 없으면 반대쪽으로 우회하지 않는다.
- 이미 통로에 진입해 방향이 잠겼다면 이후 반대 이벤트는 무시한다.

플랫폼 전송은 사용자가 명시적으로 요청한 경우에만 수행한다.


관측 매칭 대기로 경로를 정지시키지 않는다. `road_mapping.source`는 `prior`(최초 사전 지도)
또는 `matched`(한 번 이상 관측 정렬한 지도), `using_saved_map`은 새 관측 정렬 대신
저장된 지도를 사용하는지 나타낸다. 섬 매칭도 도로 관측 정렬을 기다리지 않고 수행하며,
도로 좌표계가 갱신되면 섬과 누적 관측은 같은 odom 위치를 유지한다.


### 시작 트리거 로그

활성화 성공 시 `[OBJ 시작] trigger=...`를 한 번 출력한다.
`yellow_entrance`는 노란 입구 윤곽 매칭 후 관측 창 안에서 장애물 조건을 만족한 시작,
`dashes_obstacles`는 기존 흰 점선·장애물 조건으로 시작한 경우다.
두 조건이 동시에 만족하면 실제 우선 적용하는 `yellow_entrance`로 기록하고
`entrance_ready`, `legacy_ready`를 모두 표시한다. 점선 좌우 개수·장애물 개수·원점 이동량도 기록한다.
수동 활성화는 `manual_activation`, `auto_activate` 설정은 `auto_activate`로 구분한다.
현재 시작 원인은 상태 토픽의 `start_gate.trigger`에도 남는다.


### 목표 도착 / 장애물 재등장 / 조기 표지판

- 갈림길 DONE은 표시된 초록색 목표점과 차량 기준점 사이 거리가
  `fork_goal_tolerance: 0.15` m 이내일 때 판정한다. 차선 명령 수신 여부와 관계없이
  도착을 판정하며, DONE 이후 obj 제어 명령과 주행 Path를 발행하지 않는다.
  DONE·상태·디버그 마커는 계속 확인할 수 있다.
- 완료 전에, 한 번 관측했던 장애물이 유효 스캔에서
  `obstacle_reacquire_absence_sec: 1.0`초 이상 사라졌다가
  `obstacle_reacquire_confirm_scans: 2`회 연속 재등장하면 그때의 차량 자세를 새 시작점으로
  지도 관측·섬 매칭·추적·경로를 초기화한다. YOLO 좌우 방향은 유지한다.
  센서 중단·무효 스캔과 짧은 누락은 재시작 조건으로 세지 않는다.
  `obstacle_reacquire_reset: false`로 이 동작을 끌 수 있으며,
  상태 JSON의 `road_mapping.restarts`에서 횟수를 확인한다.
- 활성화 전 YOLO 방향은 `fork_sign_cache_sec: 30.0`초까지 보존한다.
  활성화 후에는 미션 동안 유지하고, 섬 위치가 확정되면 보라선 도착 전에도
  해당 측면 lattice를 계산한다. 보라선 통과는 방향 저장·경로 생성 조건이 아니다.
- YOLO 좌·우 표지판 면적 기준은 500px², confidence는 0.65를 유지한다.


### 입구 재진입과 저장된 섬 지도 유지

- 매칭에 성공해 확정한 섬은 새 영상의 점이 부족해도 odom 기준 지도에 유지한다.
  `island_mapping.source: saved`는 마지막 확정 지도를 사용하는 상태다.
  지도 좌표계 보정 시 저장된 섬도 같은 월드 위치를 유지한다.
- 섬 접근 판단은 실제 매칭된 섬 위치를 기준으로 한다. 탐색 구간을 벗어나면
  탐색 시간·이동거리 누적값을 초기화하므로 과거 탐색의 한도로 path를 지우지 않는다.
- 입구 윤곽이 1초 이상 관측되지 않았다가 다시 연속 3회 확인되면 지도·섬·장애물 추적·
  path를 초기화하고 입구 기준으로 다시 매칭한다. 좌우 표지판 방향과 투표는 보존한다.
  같은 입구를 계속 보는 동안은 반복하지 않으며 재초기화 사이에는 최소 5초를 둔다.
- 수동 초기화도 가능하다(새 실행 파일로 노드를 시작한 뒤, 해당 ROS 도메인에서):

```bash
ros2 topic pub --once /objavoid_v3/remap std_msgs/msg/Bool '{data: true}'
```

위 명령은 활성 obj의 지도·path를 재시작한다. 표지판 방향을 제외한 지도와 추적을
지우므로 실제 차량의 주행 경로에 영향을 준다.

장애물 여유는 `preferred_obstacle_clearance: 0.10` m를 우선 적용한다(차체 외곽 기준).
10cm 조건으로 진행 가능한 경로가 없으면 기존 `margin_obstacle`로 재계산한다.
04번의 기존 최소값은 3cm이며 차체·곡률·조향·외곽 충돌 검사는 그대로 유지한다.
`clearance_policy.selected`와 `fallback`에서 선택한 최소 여유 조건을 확인한다.
이미 장애물에 가까운 시작 자세는 기존의 점진적 회복 구간을 사용한다.


### 경로 소실 후 20cm 되짚기

실차 `real_car04.yaml`은 OBJ 후진을 활성화한다. 경로 소실/재계획 실패가
`reverse_wait`(기본 1초) 지속되고 정지하면, odom에 저장한 실제 전진 궤적을
역순으로 따라 `reverse_dist`(0.20m)를 물러난다. 후방 LiDAR 관측은 요구하지 않는다.
20cm의 연속 주행 기록이 없거나 odom이 오래되면 후진하지 않는다.
실패한 전방 경로의 주변 통로는 가상 제외 영역으로 기억해 비슷한 후보들도 거부한다.
이 기록은 코스 지도 좌표로 유지하므로 시작 위치 복귀/지도 재초기화 후에도 같은 구간의 실패 통로를 다시 선택하지 않는다. YOLO 좌우 표지판 방향은 바꾸지 않는다. 기록은 현재 OBJ 프로세스가 유지되는 동안 보존된다.
MUX는 `objavoid`와 `parking`의 음수 속도만 허용한다. 노드/MUX 재시작 후 적용된다.
