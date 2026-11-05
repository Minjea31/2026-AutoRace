# autorace_lane_pid — 시뮬용 오차 기반 PID 차로 유지

`autorace_lane_lite`가 발행하는 `/lane_lite/target`(차선 인식 결과)에서 오차를 만들고 PID로 조향합니다. **인식과 완전히 분리된 패키지**이며, 제어 파라미터는 전용 튜너 창 `lane_pid_tuner`에서만 조절합니다(차선 튜너 `lane_lite_tuner`에는 나오지 않음). 설정은 시뮬 전용(`config/sim/pid.yaml`)입니다.

## 실행

```bash
cd ~/2026-AutoRace/lane_ws
source /opt/ros/humble/setup.bash
PYTHONNOUSERSITE=1 colcon build --packages-select autorace_lane_lite autorace_lane_pid --cmake-args -DCMAKE_BUILD_TYPE=Release

# 터미널 1: 시뮬
cd ~/2026-AutoRace && ./simul.sh
# 터미널 2: 인식 + PID 제어(/mission/lane 발행) + PID 튜너 창
cd ~/2026-AutoRace && ./simul/lane.sh
#   차선 튜너 창도 띄우기:   ./simul/lane.sh lane_gui:=true
#   PID 창 없이:             ./simul/lane.sh gui:=false
# 터미널 3: /mission/lane -> /cmd_vel
cd ~/2026-AutoRace && ./simul/adapter.sh --no-gui --topic /mission/lane
```

직접 launch할 때는 `ros2 launch autorace_lane_pid lane_pid.launch.py enable_vehicle_output:=true`입니다(기본 `use_sim_time:=true`, `enable_vehicle_output:=false`이면 `/lane_pid/command`만 발행). 튜너만 따로: `ros2 run autorace_lane_pid lane_pid_tuner`.

## 제어 방식

```text
e = lateral_weight·가까운 점 횡오차 + heading_weight·가까운 점 헤딩 + lookahead_weight·목표점 횡오차   (좌측 +)
δ = kp·e + ki·∫e dt + kd·(저역통과 de/dt) + curvature_ff·atan(L·κ)
```

- **가까운 점**은 중앙 경로의 BEV 맨 아래 점(시뮬 기본 카메라에서 약 0.67 m 앞), **목표점**은 `lookahead_row`의 점(약 1.2 m 앞)입니다. `lane_lite`가 `/lane_lite/target` 뒤쪽에 `near_lateral_m, near_heading_rad, near_m`을 추가로 발행합니다.
- 각 측정값은 `lateral_rate_limit`/`heading_rate_limit`보다 빠르게 변하지 못합니다. 정지선·횡단보도 줄무늬를 한 프레임 경로로 오인해 헤딩이 ±1.6 rad로 튀는 경우를 막습니다.
- D·I는 **새 인식 결과가 들어올 때만**, 그 사이 실제 시간(dt)으로 갱신합니다. 적분은 출력이 포화된 방향으로 쌓이지 않고(조건부 적분) `integral_limit`로 제한합니다. 차선을 잃으면 I·D 기록을 지웁니다.
- 출력은 `max_steering`, `steering_rate`로 제한합니다. 속도는 `speed / (1+steer_slowdown·|δ|) / (1+curvature_slowdown·|κ|)`에 신뢰도 비율을 곱하고 `min_speed` 이상으로 유지합니다. 가속만 `acceleration`으로 제한하고 감속·정지는 즉시 적용합니다.
- 인식 무효·신뢰도 ≤ `confidence_stop`·`timeout` 초과·`active=0`이면 속도 0입니다.

## PID 튜너 창 (`lane_pid_tuner`)

- 왼쪽: 파라미터마다 **이름 [단위] / 한국어 설명 / 슬라이더 + 숫자 입력 + 범위**. 그룹: 오차 정의, PID 게인, 조향 제한, 속도, 안전.
- 오른쪽: `/lane_pid/state` 실시간 표시(결합 오차, P/I/D/FF 항, 목표·출력 조향, 속도, 인식 횡오차·헤딩·곡률·신뢰도, 데이터 나이)와 조향 기여도 막대.
- 값을 바꾸면 실행 중인 `/lane_pid_controller`에 **바로 적용**합니다(드래그 중에는 50 ms마다 묶어서 전송). 노드가 거부한 값(예: `confidence_stop ≥ confidence_full`)은 노드 값으로 되돌립니다. 노드가 없을 때 바꾼 값은 노드가 뜨면 전송합니다.
- **■ 주행 정지 / ▶ 주행 시작** 버튼은 `active` 파라미터를 바꿉니다(정지 중에도 조향 계산은 계속).
- **Save YAML** 기본 위치는 소스 트리의 `config/sim/pid.yaml`입니다. launch 기본값에 반영하려면 저장 후 다시 빌드합니다.

## 토픽

| 방향 | 토픽 | 내용 |
| --- | --- | --- |
| 구독 | `/lane_lite/target` | `Float32MultiArray`, `autorace_lane_lite` README 참고 |
| 발행 | `/lane_pid/command` | `AckermannDriveStamped` 미리보기, 50 Hz |
| 발행 | `/mission/lane` | `enable_vehicle_output:=true`일 때만, 같은 명령 |
| 발행 | `/lane_pid/state` | `[tracking, e, P, I, D, FF, 목표 조향, 출력 조향, 속도, 가까운 횡오차, 가까운 헤딩, 곡률, 신뢰도, 데이터 나이, active, 목표점 횡오차]` |

CLI로도 바꿀 수 있습니다: `ros2 param set /lane_pid_controller kp 0.5` (모두 double).

## 검증 (2026-11-05, 이 PC, headless 시뮬·별도 DDS 도메인)

- 단위 테스트 12개 통과(`./build/autorace_lane_pid/test_pid`): 부호, 결합 오차, 측정 시간 기반 D, 적분 제한·포화 시 미적분, 소실·timeout 정지, 조향 변화율·가속 제한, 측정 튐 제한, YAML 왕복.
- 기본값(0.25 m/s) 폐루프: `track` 출발 → 상단 직선(실제 위치 기준 차로 중심 오차 RMS 약 1 mm) → 우상단 코너 → 우측 직선 → 우하단 코너 통과. `signal`·`crosswalk` 출발도 같은 구간 통과. 세 경우 모두 하단 **장애물 미션 구간의 정차 차량(`obstacle_007`)** 앞에서 충돌로 정지합니다(장애물 회피는 범위 밖).
- 실패·한계:
  - 처음 시도한 고게인(kp 1.6, 가까운 점 무시) 설정은 우상단 코너 안쪽을 깎아 코너 안쪽 차선 위의 `zigzag_slow_sign`에 충돌했습니다. 기본값이 목표점 오차 중심·낮은 kp인 이유입니다.
  - 0.35 m/s에서는 `track`은 통과하지만 `signal` 출발 시 횡단보도(≈9.1, 2.8)에서 인식이 줄무늬를 경로로 오인해 `pedestrian_sign`에 충돌했습니다.
  - `fork` 출발은 출발 직후 갈림길 섬에 막힙니다(갈림길은 인식·제어 범위 밖).
  - 실차 설정·검증은 없습니다.
