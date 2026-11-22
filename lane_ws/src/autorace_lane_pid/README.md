# autorace_lane_pid

`/lane_deep/target`의 횡오차·헤딩·곡률·신뢰도로 속도와 조향을 계산합니다.
통합 실행은 `tensor_bringup lane_mode:=deep lane_speed:=0.8`입니다.
차선 단독 실행은 `autorace_lane_deep_lite/lane_deep_lite.launch.py`를 사용합니다.

기본 속도·PID는 deep 패키지의 `config/<profile>/pid.yaml`에서 읽습니다.
목표 소실·시간 초과 시 정지하고, 곡률·신뢰도에 따라 감속합니다.
주차 표지판 감속, 주차 시작/준비 이벤트와 출차 후 표지판 무시 조건도 처리합니다.

| 방향 | 토픽 | 내용 |
| --- | --- | --- |
| 구독 | `/lane_deep/target` | 차선 목표와 신뢰도 |
| 발행 | `/mission/lane` | MUX 입력 (`enable_vehicle_output:=true`) |
| 발행 | `/lane_pid/command` | PID 명령 |
| 발행 | `/lane_pid/state` | 오차·제어항·조향·속도·데이터 나이 |

`target_topic`으로 입력 토픽을 지정할 수 있습니다.
PID GUI는 `ros2 run autorace_lane_pid lane_pid_tuner --config <pid.yaml>`로 실행합니다.
실차 deep 설정을 수정할 때는 deep 패키지의 YAML을 지정하세요.
