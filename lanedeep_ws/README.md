# 현재 deep 차선 구성

이 브랜치는 현재 main의 `lane_mode:=deep lane_speed:=0.8`에서 사용하는
차선 분할·기하·추적·PID 코드와 모델을 반영합니다.

```text
카메라 → lane_fusion_deep_mask → /lane_fusion/deep_mask
      → lane_deep_lite_node → /lane_deep/target
      → lane_pid_controller → /mission/lane
```

| 역할 | 현재 코드 위치 |
| --- | --- |
| 딥 모델 추론·마스크 | `lane_fusion_ws/src/autorace_lane_fusion_deep` |
| BEV·차선 추적·목표점·지그재그 | `lane_fusion_ws/src/autorace_lane_deep_lite` |
| PID·속도·조향·주차 연계 | `lane_ws/src/autorace_lane_pid` |

예전 `lanedeep_ws/src/autorace_lane_deep` Python 제어 구현은 교체했습니다.
코드 위치와 토픽 이름은 main에서 쓰는 경로를 유지합니다.
인식 설정은 deep-lite 패키지의 `config/real/lane.yaml`,
PID 설정은 같은 폴더의 `pid.yaml`입니다. 기본 속도는 해당 YAML의 0.8 m/s입니다.
공용 미션 HSV·장애물 투영 설정은 `mission_vision.yaml`입니다.

## 빌드

ROS Humble 시스템 Python 환경에서 실행합니다.

```bash
source /opt/ros/humble/setup.bash
cd ~/2026-AutoRace/lane_ws
colcon build --symlink-install --packages-select autorace_lane_pid
source install/local_setup.bash
cd ../lane_fusion_ws
colcon build --symlink-install
source install/local_setup.bash
```

## 차선 단독 실행

센서와 필요한 MUX는 별도로 실행합니다.

```bash
cd ~/2026-AutoRace
./lane_fusion_ws/deep.sh real
# 또는 ./lanedeep_ws/lane.sh real
```

차량 출력 없이 확인하려면 `enable_vehicle_output:=false`를 추가합니다.
`profile:=real` 단독 launch는 위 YAML을 사용합니다.
main의 전체 미션 launch는 속도·주차 연계 파라미터를 추가로 지정합니다.
전체 미션과 함께 사용할 때는 main의
`tensor_bringup.launch.py lane_mode:=deep lane_speed:=0.8` 구성을 사용하세요.

인식 튜너는 `ros2 run autorace_lane_deep_lite lane_deep_lite_tuner --profile real`,
PID 튜너는 `ros2 run autorace_lane_pid lane_pid_tuner --config <deep PID YAML>`입니다.
모델 엔진은 장치에 맞춰 `lane_fusion_ws/tools/build_tensorrt.py`로 다시 만들 수 있습니다.
