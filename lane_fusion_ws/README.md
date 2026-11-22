# Deep 차선 주행

현재 실차 차선은 deep 한 경로로 실행합니다.

```text
카메라 → lane_fusion_deep_mask → /lane_fusion/deep_mask
      → lane_deep_lite_node → /lane_deep/target
      → lane_pid_controller → /mission/lane → MUX
```

실차 전체 미션 실행:

```bash
source ~/2026-AutoRace/source_all_ws.bash
ros2 launch tensor_bringup tensor_bringup.launch.py lane_mode:=deep lane_speed:=0.8
```

차선만 실행하려면 `~/2026-AutoRace/lane_fusion_ws/deep.sh real`을 사용합니다.
시뮬레이터는 `simul/lane.sh` 또는 `lane_fusion_ws/deep.sh sim`으로 연결합니다.
`lane_ws/lane.sh sim|real`도 같은 deep 실행 경로를 호출합니다.
다른 차선 실행과 동시에 띄우면 `/mission/lane` 발행이 중복됩니다.

- `lane_fusion_ws/src/autorace_lane_fusion_deep`: 카메라 딥 분할, 모델·TensorRT 변환 도구
- `lane_fusion_ws/src/autorace_lane_deep_lite`: 마스크 기반 차선 기하·추적·목표점·BEV 디버그
- `lane_ws/src/autorace_lane_pid`: PID 제어와 PID GUI 튜너

인식 설정은 `autorace_lane_deep_lite/config/<real|sim>/lane.yaml`,
주행 PID 설정은 같은 폴더의 `pid.yaml`입니다.
`mission_vision.yaml`은 서행·횡단보도·신호등·주차의 HSV와 장애물 BEV 투영용입니다.
기존 실차 수치를 옮긴 파일이며 차선 인식 설정과 구분합니다.

## 빌드

시스템 Python을 사용하는 ROS Humble 환경에서 빌드합니다.

```bash
source /opt/ros/humble/setup.bash
cd ~/2026-AutoRace/lane_ws
colcon build --symlink-install --packages-select autorace_lane_pid
source install/local_setup.bash
cd ../lane_fusion_ws
colcon build --symlink-install
source install/local_setup.bash
```

모델 엔진은 차량에서 `lane_fusion_ws/tools/build_tensorrt.py`로 만듭니다.
전체 명령은 [deep 인식 패키지](src/autorace_lane_deep_lite/README.md)를 참고하세요.

## 시각화와 튜닝

- 통합 웹뷰어의 `/camera`: `/lane_deep/debug` BEV와 `/lane_fusion/deep_mask` 마스크
- `ros2 run autorace_lane_deep_lite lane_deep_lite_tuner --profile real`: deep 인식 튜너
- `ros2 run autorace_lane_pid lane_pid_tuner --config <deep 패키지의 config/real/pid.yaml>`: PID 튜너
- `lane_deep_lite.launch.py profile:=real gui:=true`: 차선과 PID 튜너 함께 실행

기존 OpenCV/fusion 차선 인식 패키지와 fusion PID, `fusion.sh`는 정리했습니다.
