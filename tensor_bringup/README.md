# tensor_bringup
Sensors must already be running. Stop individual mission launches first.

```bash
source /opt/ros/humble/setup.bash
for ws in sensor_ws mux_ws lane_fusion_ws parking_ws slow_ws yolo_ws; do source ~/2026-AutoRace/$ws/install/setup.bash; done
cd ~/2026-AutoRace/tensor_bringup
colcon build --base-paths .
source install/setup.bash
ros2 launch tensor_bringup tensor_bringup.launch.py
# Override lane speed when needed (default: 0.5 m/s).
ros2 launch tensor_bringup tensor_bringup.launch.py lane_speed:=0.5
```
Mux starts after inputs are ready and the car can move immediately.
After mux starts, readiness checks all seven mission nodes and fresh input topics for two seconds.
The final startup line is `[bringup 결과] 전체 노드 7/7 실행 확인 완료` on success;
timeouts list missing nodes and missing/stale input topics before reporting failure.
`startup_timeout` (default 60 seconds) applies to each startup check.
`lane_speed` overrides the lane PID base speed only; the default is 0.5 m/s.
Curve/confidence slowdowns still apply, and mux retains its 0.8 m/s forward limit.
RViz is separate: `rviz2 -d ~/2026-AutoRace/tensor_bringup/rviz/tensor_bringup.rviz`.
