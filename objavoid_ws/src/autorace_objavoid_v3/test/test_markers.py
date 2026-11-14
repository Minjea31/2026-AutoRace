"""RViz 마커 생성이 실제 ROS 메시지로 만들어지는지(필드 타입 오류 시 배열 전체가 실패하던 버그 회귀 방지)."""
import numpy as np
import pytest

rclpy = pytest.importorskip('rclpy')
from builtin_interfaces.msg import Time  # noqa: E402

from autorace_objavoid_v3.markers import build_markers, to_base  # noqa: E402
from autorace_objavoid_v3.mission import ObjAvoidCore  # noqa: E402
from autorace_objavoid_v3.params import Params  # noqa: E402
from autorace_objavoid_v3.perception import Box  # noqa: E402


def test_build_markers_with_everything():
    core = ObjAvoidCore(Params())
    for k in range(5):
        core.on_odom(k * 0.02, 0.0, 0.0, 0.0, 0.0, 0.0)
    assert core.activate(0.08)
    core.tracker.update([Box(1.0, 1.33, -0.115, 0.115)] * 1, 0.1, -0.2)
    core.tracker.update([Box(1.0, 1.33, -0.115, 0.115)], 0.2, -0.2)
    core.last_points = np.array([[1.0, 0.0], [1.0, 0.05]])
    core.last_boxes = [Box(1.0, 1.33, -0.115, 0.115)]
    core.aligned = True
    core.ymap.update(np.array([[1.0, 0.78], [1.03, 0.78]]), np.array([[1.0, 0.78], [1.03, 0.78]]), (0.0, 0.0, 0.0))
    core.ymap.update(np.array([[1.0, 0.78], [1.03, 0.78]]), np.array([[1.0, 0.78], [1.03, 0.78]]), (0.0, 0.0, 0.0))
    core.path = core.planner.plan((0.0, 0.0, 0.0, 0.0), [tr.box for tr in core.tracker.confirmed()], core.grid)
    assert core.path is not None
    arr = build_markers(core, Time(), 0.08)
    to_base(arr, (0.0, 0.0, 0.0))
    names = {m.ns for m in arr.markers}
    for ns in ('obstacles', 'scan_detections', 'lidar_roi_points', 'path', 'candidates', 'lane_yellow_cells', 'gaps'):
        assert ns in names
