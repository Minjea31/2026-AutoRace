# objavoid_v3_ws — 장애물 회피 V3 (Gap 탐색 + 5차 다항식 Lattice + PD 추종)

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
cd ~/dev/2026-AutoRace/objavoid_v3_ws
source /opt/ros/humble/setup.bash
PYTHONNOUSERSITE=1 colcon build --symlink-install     # 심볼릭 설치 필수(launch가 저장소의 lane_fusion_ws BEV 파일을 찾음)
source install/setup.bash
```

### 시뮬레이터 (터미널마다 `export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1 PYTHONNOUSERSITE=1`)

```bash
# 1) 시뮬레이터(conda 환경: simul/env.sh)
source ~/dev/2026-AutoRace/simul/env.sh
python -m limo_parking_sim.app --mission obstacle --view follow --map ~/dev/2026-AutoRace/objavoid_v3_ws/scenarios/user7.yaml --spawn 9.05 1.0 3.1416
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
