# tensor_bringup

## Shared mission RViz

```bash
cd ~/2026-AutoRace
./view_missions.sh
# Equivalent:
ros2 launch tensor_bringup missions_rviz.launch.py
```

`autorace_missions.rviz` at the repository root links to the shared configuration.
The viewer starts visualization only; start sensors and mission bringup separately.
It uses the terminal's ROS domain and display and is independent of vehicle IP/number.

The main view follows `base_link` with the same top-down camera as `parking_real.rviz`.
Robot, 20 cm grid, red LiDAR and camera are visible throughout all missions.
The cyan curve depicts 1.5 seconds of the latest actual `/cmd_vel` request, from whichever
mission currently controls the mux; it is not a collision-checked route or measured trajectory.

Parking ROI, observed/remembered obstacles, slot occupancy, targets, lookahead and green
parking/reverse path appear automatically while `/parking/status.active` is true.
They clear when inactive or when parking status is missing for two seconds. A viewer started
mid-mission uses the recurring status message. Original parking topics are unchanged.
There is no text overlay; text markers from parking are filtered from the viewer topics.

Camera runs on its existing raw image topic. The optional Lane BEV display is off by default
to avoid requesting extra debugging work during driving. Enable it in Displays when needed.
Rendering is limited to 15 fps. TF and fused odometry displays are optional and initially off.
The helper only publishes `/missions_viz/*`; it never publishes motion, mission starts or TF.
Sensors must already be running. Stop individual mission launches first.

Startup is staggered by default. Parking starts immediately, slow and crosswalk at 2 s,
lane/TensorRT at 4 s, YOLO at 8 s. Input readiness monitoring starts at 8 s, so its
`startup_timeout` still measures the readiness phase rather than the initial stagger.
Mux starts only after the required input checks pass; output topics, speed settings
and arbitration rules are unchanged.

Mission processes use all allowed CPUs except the final localization CPU (on hosts with
at least four available CPUs), nice 5, and one OpenMP/BLAS math thread. Sensor bringup
uses matching CPU placement for EKF/IMU. This is process affinity, not exclusive
CPU isolation; kernel threads and unrelated applications may still use that CPU.
There is no global clock, power-mode or real-time scheduler change. EKF remains 30 Hz.
There are no additional launch arguments.

```bash
source /opt/ros/humble/setup.bash
for ws in sensor_ws mux_ws lane_fusion_ws parking_ws slow_ws crosswalk_ws yolo_ws; do source ~/2026-AutoRace/$ws/install/setup.bash; done
cd ~/2026-AutoRace/tensor_bringup
colcon build --base-paths .
source install/setup.bash
ros2 launch tensor_bringup tensor_bringup.launch.py
# Override lane speed when needed (default: 0.5 m/s).
ros2 launch tensor_bringup tensor_bringup.launch.py lane_speed:=0.5
# Lane mode (default fusion). fusion/deep run lane_fusion_node + fusion PID, opencv runs lane_ws:
ros2 launch tensor_bringup tensor_bringup.launch.py lane_mode:=fusion   # deep mask + HSV
source ~/2026-AutoRace/lane_ws/install/setup.bash                       # opencv needs lane_ws too
ros2 launch tensor_bringup tensor_bringup.launch.py lane_mode:=opencv   # lane_ws: lane_lite + lane PID
ros2 launch tensor_bringup tensor_bringup.launch.py lane_mode:=deep     # deep mask only (HSV detector off)
```
`lane_mode` picks the lane stack; readiness checks that mode's nodes
(fusion/deep: `lane_fusion_deep_mask`, `lane_fusion_node`, `lane_fusion_pid_controller` + `/lane_fusion/deep_mask`,
opencv: `lane_lite_node`, `lane_pid_controller`). fusion/deep PID: `autorace_lane_fusion_pid/config/real/pid.yaml`;
opencv uses lane_ws as-is (`autorace_lane_lite/config/real/lane.yaml`, `autorace_lane_pid/config/real/pid.yaml`,
lane_lite debug on, tuners off). `deep_lite` is no longer launched here.
In deep mode the lane tuner's Apply sends `yellow_enabled`/`white_enabled` too: keep them 0 there.
`lane_speed` applies to all three.
After mux starts, readiness checks all mission nodes (8 with fusion/deep, 7 with opencv) and fresh input topics for two seconds.
The final startup line is `[bringup 결과] 전체 노드 8/8 실행 확인 완료` on success;
timeouts list missing nodes and missing/stale input topics before reporting failure.
`startup_timeout` (default 60 seconds) applies to each startup check.
`lane_speed` overrides the lane PID base speed only; the default is 0.5 m/s.
Curve/confidence slowdowns still apply, and mux retains its 0.8 m/s forward limit.
RViz is separate: `rviz2 -d ~/2026-AutoRace/tensor_bringup/rviz/tensor_bringup.rviz`.

Crosswalk starts automatically with `use_sim_time:=false`, using its installed
`crosswalk.yaml` and the same process affinity/nice/thread settings as the other missions.
Readiness requires `/crosswalk_node` and fresh `/yolo/crosswalk_sign` messages before
starting mux. It does not wait for `/start_crosswalk` or `/mission/crosswalk/command_01`;
those appear only when an actual crosswalk stop begins. The existing crosswalk policy
(two sign confirmations, fresh white ROI, three-second stop) and mux priority are unchanged.

Build the crosswalk overlay once before rebuilding bringup:

```bash
cd ~/2026-AutoRace
source /opt/ros/humble/setup.bash
cd crosswalk_ws
colcon build --symlink-install --packages-select autorace_crosswalk
source install/setup.bash
# Source the other mission overlays listed above, then rebuild tensor_bringup.
```
