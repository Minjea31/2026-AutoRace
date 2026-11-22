# lane_ws

현재 `autorace_lane_pid` 패키지만 포함합니다. Deep 인식 결과 `/lane_deep/target`을 받아
`/mission/lane`과 `/lane_pid/command`, `/lane_pid/state`를 발행합니다.

인식·PID 구성과 빌드는 [deep 차선 안내](../lane_fusion_ws/README.md)를 참고하세요.
`./lane.sh sim|real`은 `lane_fusion_ws/deep.sh`를 호출합니다.
OpenCV 전용 `autorace_lane_lite`와 이전 `lane_pid.launch.py`는 삭제했습니다.
