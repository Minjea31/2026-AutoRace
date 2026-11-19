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

Startup is staggered by default. Parking starts immediately, slow, crosswalk, signal and cone at 2 s,
lane/TensorRT at 4 s, YOLO at 8 s. Input readiness monitoring starts at 8 s, so its
`startup_timeout` still measures the readiness phase rather than the initial stagger.
Mux starts only after the required input checks pass; output topics, speed settings
and arbitration rules are unchanged.

Mission processes use all allowed CPUs except the final localization CPU (on hosts with
at least four available CPUs), nice 5, and one OpenMP/BLAS math thread. Sensor bringup
uses matching CPU placement for EKF/IMU. This is process affinity, not exclusive
CPU isolation; kernel threads and unrelated applications may still use that CPU.
There is no global clock, power-mode or real-time scheduler change. EKF remains 30 Hz.
CPU placement needs no additional launch arguments.

```bash
source /opt/ros/humble/setup.bash
for ws in sensor_ws mux_ws lane_ws lane_fusion_ws parking_ws slow_ws crosswalk_ws signal_ws cone_ws yolo_ws; do source ~/2026-AutoRace/$ws/install/setup.bash; done
cd ~/2026-AutoRace/tensor_bringup
colcon build --base-paths .
source install/setup.bash
ros2 launch tensor_bringup tensor_bringup.launch.py
# Override lane speed when needed (default: 0.5 m/s).
ros2 launch tensor_bringup tensor_bringup.launch.py lane_speed:=0.5
# Lane mode (default fusion). fusion runs lane_fusion_node + fusion PID; opencv and deep run the lane_ws logic + lane_ws PID:
ros2 launch tensor_bringup tensor_bringup.launch.py lane_mode:=fusion   # deep mask + HSV
source ~/2026-AutoRace/lane_ws/install/setup.bash                       # opencv and deep need lane_ws too
ros2 launch tensor_bringup tensor_bringup.launch.py lane_mode:=opencv   # lane_ws: lane_lite + lane PID
ros2 launch tensor_bringup tensor_bringup.launch.py lane_mode:=deep     # deep mask -> lane_deep_lite (lane_ws logic) + lane PID
```
Obstacle recovery has two explicit modes. `objavoid_mode:=normal` is the default
and uses forward driving and stops only. `objavoid_mode:=reverse` enables the
optional 20 cm reverse retrace when a route is blocked. It overrides the vehicle
configuration and is forwarded as `recovery_mode` to the obstacle launch.

```bash
ros2 launch tensor_bringup tensor_bringup.launch.py lane_speed:=0.8 objavoid_mode:=normal
ros2 launch tensor_bringup tensor_bringup.launch.py lane_speed:=0.8 objavoid_mode:=reverse
```

The C++ state topic reports the selection in `recovery.policy`. Restart the
integrated launch to change modes.

`lane_mode` picks the lane stack; readiness checks that mode's nodes
(fusion: `lane_fusion_deep_mask`, `lane_fusion_node`, `lane_fusion_pid_controller` + `/lane_fusion/deep_mask`,
deep: `lane_fusion_deep_mask`, `lane_deep_lite_node`, `lane_pid_controller` + `/lane_fusion/deep_mask`,
opencv: `lane_lite_node`, `lane_pid_controller`). fusion PID: `autorace_lane_fusion_pid/config/real/pid.yaml`;
opencv uses lane_ws as-is (`autorace_lane_lite/config/real/lane.yaml`, `autorace_lane_pid/config/real/pid.yaml`,
lane_lite debug on, tuners off). deep uses `autorace_lane_deep_lite/config/real/lane.yaml` (lane_ws values) and the same lane_ws PID with `target_topic:=/lane_deep/target`.
`lane_speed` applies to all three.
After mux starts, readiness checks all mission nodes (10 with fusion/deep, 9 with opencv) and fresh input topics for two seconds.
The final startup line is `[bringup 결과] 전체 노드 10/10 실행 확인 완료` on success;
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

Traffic light (2026-11-12): `signal_ws` `signal_node` starts with crosswalk (`use_sim_time:=false`).
Two red `/yolo/signal` (False) messages stop the car on `/mission/signal/command_02` (mux priority 2,
after crosswalk) until two green or 5 s after the last red; see [signal_ws](../signal_ws/README.md).
Readiness requires `/signal_node` but not `/yolo/signal`: YOLO publishes it only while a light is seen.

Build the crosswalk overlay once before rebuilding bringup:

```bash
cd ~/2026-AutoRace
source /opt/ros/humble/setup.bash
cd crosswalk_ws
colcon build --symlink-install --packages-select autorace_crosswalk
source install/setup.bash
# Source the other mission overlays listed above, then rebuild tensor_bringup.
```


Cone starts automatically at 2 s through `cone_lane_follower/roi.launch.py`, with
`use_sim_time:=false`, `rviz:=false` and the installed cone2 `clustering.yaml`.
Override its tuning with `cone_params:=/absolute/path/to/clustering.yaml`.
Readiness requires `/cone_lane_roi` and fresh `/cone_lane/control_status` messages,
alongside the existing `/scan` and `/odometry/filtered` checks. It never waits for
`/mission/cone/command_06`: commands begin only after LiDAR cone entry detection.
The existing mux already handles that topic at cone priority 4.

Build and source the cone overlay before rebuilding bringup:

```bash
cd ~/2026-AutoRace/cone_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select cone_lane_follower
source install/setup.bash
# Source the other overlays above, then rebuild tensor_bringup.
```

## Mission web on the same Wi-Fi

Start only the read-only web viewer:

```bash
source ~/2026-AutoRace/source_all_ws.bash
ros2 launch tensor_bringup mission_web.launch.py web_host:=0.0.0.0
```

Open `http://192.168.0.106:8080` from another device on the same Wi-Fi
(use the platform's current IP if it changes). `0.0.0.0` is the listening address;
enter the platform IP in the browser. The ROS domain and current LAN URLs appear
in the launch output. `web_port:=8081` selects another port.

To start the viewer with the mission bringup, opt in explicitly:

```bash
ros2 launch tensor_bringup tensor_bringup.launch.py web:=true web_host:=0.0.0.0
```

`web` defaults to false, preserving manual viewer startup. Ctrl+C stops a viewer
started by that launch. If the standalone `web_viewer/start.sh` viewer already owns
port 8080, stop it with `~/2026-AutoRace/web_viewer/stop.sh` before starting the launch,
or choose another `web_port`. This viewer does not start sensors or vehicle control
when launched on its own. Network/hotspot client isolation can still block peers.
