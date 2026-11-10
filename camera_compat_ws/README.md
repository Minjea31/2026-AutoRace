# Camera OpenCV compatibility overlay
The camera uses system OpenCV 4.8; ROS binaries used OpenCV 4.5, causing mixed libraries and camera aborts. This workspace rebuilds cv_bridge and image transport plugins against OpenCV 4.8, without replacing system packages.

Sources: ros-perception/vision_opencv and ros-perception/image_transport_plugins, humble branch. Git checkouts retain exact revisions.

```bash
source /opt/ros/humble/setup.bash
cd ~/2026-AutoRace/camera_compat_ws
colcon build --packages-select cv_bridge compressed_image_transport compressed_depth_image_transport theora_image_transport --cmake-args -DOpenCV_DIR=/usr/lib/cmake/opencv4 -DCMAKE_BUILD_TYPE=Release --parallel-workers 2
source install/setup.bash
```
Sensor and tensor_bringup setup files were regenerated with this overlay as an underlay. Re-source setup in existing terminals before restarting sensors.
