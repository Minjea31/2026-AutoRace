# Native C++ port

`roi_node` is a C++17 / rclcpp executable built with ament_cmake. The package,
node name, executable name, launch arguments, configuration, input/output topics,
QoS and marker namespaces are retained. Launch files remain normal ROS Python
launch descriptions; they do not load the old Python node.

The Python implementation and its tests are preserved under `reference/python`
solely as an equivalence oracle. They are not installed or run by `roi_node`.
The pre-migration remote package is also archived under `cone_ws/backups`.

- `include/cone_lane_follower/core.hpp`: clustering, ordered boundary selection,
  cross sections, geometry checks, offsets, quintic joins and error projection.
- `path_memory.hpp`: ordered odom path storage, refresh and recovery.
- `async_planner.hpp`: one C++ worker thread, one running job, one replaceable
  pending snapshot, generation invalidation and the existing planning budget.
  A native worker replaces Python's process worker without a Python interpreter.
- `cpp/roi_node.cpp`: TF, ROS-clock timers, mission/control state, publishers.

No thresholds or control branches were tuned during the port. In particular,
the original unreachable recovery `elif` in `control_tick` is preserved rather
than repaired. Floating-point contraction is disabled for closer Python parity.
Double-precision results are compared with numerical tolerances; wall-clock
planning durations and diagnostic string formatting are not bit-identical.

Build and run as before:

```bash
cd ~/2026-AutoRace/cone_ws
colcon build --symlink-install --packages-select cone_lane_follower
source install/setup.bash
ros2 launch cone_lane_follower roi.launch.py
```

With testing enabled, CTest runs worker reset/budget checks, 133 differential
geometry/memory cases (when pybind11 development headers are present), and actual
Python/C++ ROS output comparisons in isolated DDS domains 181 and 182. Tests must
not be run on the vehicle's DDS domain 96. `ros_compare.py` compares both synchronous
and asynchronous planners: mission entry, valid-path commands, TF-error stops,
recovery and mission exit after path/TF loss. No test publishes on domain 96.

Current YAML configuration, including `control_reference_x: 0.28`, is preserved.
The ROI defaults remain x=-0.3..1.0 m, y=-0.8..0.8 m in base_link. There is no
camera/white-region gate.

Post-migration tuning: mission entry now requires >= 8 ROI cone candidates.
The archived Python reference retains the original >= 6 entry threshold.

Post-migration tuning: no-path mission exit now requires <= 4 ROI candidates
for the existing 1.5 second timeout; > 4 resets the exit timer.
