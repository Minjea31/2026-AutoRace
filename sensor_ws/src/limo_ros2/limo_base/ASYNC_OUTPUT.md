# Serial receiver and ROS output

Serial decoding, IMU rate selection, motion-mode updates and odometry integration
remain on the receiver thread. Every valid motion frame is integrated before a
complete immutable odometry/TF message is handed to its output worker.

Odometry/TF, IMU, status and diagnostics have independent workers. Each keeps one
in-flight message and at most one pending message. When DDS is blocked, the pending
message is replaced with the newest complete message instead of accumulating old
samples. Message types, topics, QoS, steering encoding, covariance and source
timestamps are unchanged. Normal IMU scheduling is unchanged.
Transient hardware error bits are accumulated separately so replacing queued
status/diagnostic snapshots cannot hide an alarm that subsequently clears.

Shutdown first stops and joins serial reception, then drops pending output and
joins the publishers before their ROS objects are destroyed. An in-flight DDS
write cannot be forcibly cancelled; transport shutdown still has to return.

`latest_worker_test` checks blocked handlers, newest-only replacement, revision
ordering, stream independence, exception recovery and shutdown. The steering
command model regression remains enabled. Hardware-free pseudo-terminal testing
also verifies that a 3-second IMU/odom publish delay does not stop the other stream
or drop motion samples before position integration.
