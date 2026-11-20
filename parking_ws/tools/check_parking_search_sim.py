"""Exercise ParkingNode with simulated odometry, IMU and LiDAR on localhost.

Planning and scan matching run synchronously on a logical clock. Startup
calibration and steering readiness are bypassed; this does not test hardware.
"""

import argparse
import json
import math
import os
import sys
from pathlib import Path
from concurrent.futures import Future
from types import SimpleNamespace
import yaml
import rclpy
from rclpy.parameter import Parameter
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Imu
from std_msgs.msg import String

ROOT = Path(__file__).resolve().parents[2]
parser = argparse.ArgumentParser()
parser.add_argument("--profile", choices=["real", "default", "legacy"], default="real")
parser.add_argument("--output", required=True, type=Path)
parser.add_argument("--limited-view", action="store_true")
parser.add_argument("--fail-planning-for", type=float, default=0.0)
parser.add_argument(
    "--package", type=Path, default=ROOT / "parking_ws/src/autorace_parking"
)
parser.add_argument("--duration", type=float, default=35.0)
args = parser.parse_args()
assert os.environ.get("ROS_LOCALHOST_ONLY") == "1" and int(
    os.environ.get("ROS_DOMAIN_ID", "0")
) in range(160, 200)
sys.path.insert(0, str(args.package))
sys.path.insert(0, str(ROOT / "simul/src/limo_parking_sim"))
import autorace_parking.node as module
from autorace_parking.core import advance
from autorace_parking.tracking import compose
from limo_parking_sim.config import load_config
from limo_parking_sim.model import Simulation

params = {}
if args.profile in ("real", "default"):
    config_name = "parking_real.yaml" if args.profile == "real" else "parking.yaml"
    params = yaml.safe_load((args.package / "config" / config_name).read_text())[
        "parking_node"
    ]["ros__parameters"]
else:
    params.update(
        goal_tolerance=0.035,
        goal_yaw_tolerance=0.12,
        parking_position_tolerance=0.035,
        parking_speed=0.4,
        search_enabled=True,
        search_speed=0.1,
    )
params.update(
    localization_source="fused",
    async_scan_matching=False,
    planner_prewarm=False,
    white_roi_enabled=False,
    parking_calibration_stop=False,
    localization_max_stddev=0.0,
    localization_max_yaw_stddev=0.0,
)
rclpy.init()
n = module.ParkingNode(
    parameter_overrides=[Parameter(k, value=v) for k, v in params.items()]
)
n.worker.shutdown(wait=True)
clock = {"t": 1000.0}
module.time = SimpleNamespace(monotonic=lambda: clock["t"])
n.now_seconds = lambda: clock["t"]
n.poll_steering_model = lambda: None
n.steering_model_ready = lambda: True
n.get_logger = lambda: SimpleNamespace(
    info=lambda *a: None, warning=lambda *a: None, error=lambda *a: None
)
plans = []


class Worker:
    def submit(self, fn, *values):
        f = Future()
        try:
            result = fn(*values)
            if (
                isinstance(result, dict)
                and "path" in result
                and clock["t"] - 1000 < args.fail_planning_for
            ):
                result = dict(result, path=[], final=False)
                result["diagnostics"] = dict(
                    result["diagnostics"], reason_code="no_observed_path"
                )
            f.set_result(result)
            if isinstance(result, dict):
                plans.append(
                    {
                        "t": clock["t"] - 1000,
                        "code": result["diagnostics"].get("reason_code"),
                        "poses": len(result.get("path", [])),
                    }
                )
        except Exception as exc:
            f.set_exception(exc)
        return f

    def shutdown(self, **kw):
        pass


n.worker = Worker()
command = [0.0, 0.0]
n.publish_command = lambda v, a: command.__setitem__(slice(None), [v, a])
for name in [
    "publish_memory",
    "publish_control_visuals",
    "publish_target",
    "publish_lookahead",
    "publish_candidate_roi",
    "publish_slot_map",
    "publish_obstacles",
]:
    setattr(n, name, lambda *a: None)
n.activation.restart(clock["t"])
n.begin_parking(False, clock["t"])
sim = Simulation(load_config())
start = (sim.x, sim.y)
wheel = (0.0, 0.0, 0.0)
trace = []
parked_inside = None


def stamp(message):
    ns = round(clock["t"] * 1e9)
    message.header.stamp.sec, message.header.stamp.nanosec = divmod(ns, 10**9)


try:
    for tick in range(round(args.duration * 20)):
        clock["t"] = 1000 + tick * 0.05
        odom = Odometry()
        stamp(odom)
        odom.header.frame_id = "parking_odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x, odom.pose.pose.position.y = wheel[:2]
        odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = math.sin(
            wheel[2] / 2
        ), math.cos(wheel[2] / 2)
        odom.twist.twist.linear.x, odom.twist.twist.angular.z = (
            sim.speed * 1.02,
            sim.yaw_rate,
        )
        for i in [0, 7, 35]:
            odom.pose.covariance[i] = 0.001
        n.on_localization(odom)
        imu = Imu()
        stamp(imu)
        imu.header.frame_id = "base_link"
        imu.angular_velocity.z = sim.yaw_rate
        n.on_departure_imu(imu)
        n.on_measured_wheel(odom)
        n.on_fusion_health(
            String(data=json.dumps({"stamp": clock["t"], "ready": True}))
        )
        if tick % 2 == 0:
            msg = LaserScan()
            stamp(msg)
            msg.header.frame_id = "laser_link"
            msg.angle_min, msg.angle_increment = float(sim.angles[0]), float(
                sim.angles[1] - sim.angles[0]
            )
            msg.range_min = 0.01
            msg.range_max = (
                0.8 if args.limited_view and sim.x - start[0] < 0.12 else 3.0
            )
            msg.ranges = [
                float(r) if r <= msg.range_max else math.inf for r in sim.scan()
            ]
            n.on_scan(msg)
        n.tick()
        v, a = command
        row = {
            "t": tick * 0.05,
            "state": n.state,
            "speed": v,
            "actual_speed": sim.speed,
            "pose": [sim.x - start[0], sim.y - start[1], sim.yaw],
            "estimate": list(n.tracker.pose),
            "reason": n.reason,
            "planning": n.planning_info.get("reason_code"),
        }
        trace.append(row)
        if n.follower.parked and parked_inside is None:
            xmin, xmax, ymin, ymax = sim.cfg["world"]["parking"]["goal"]
            parked_inside = all(
                xmin <= x <= xmax and ymin <= y <= ymax for x, y in sim.body().corners()
            )
        if len(trace) == 1 or trace[-2]["state"] != n.state:
            print("STATE", json.dumps(row), flush=True)
        if tick % 100 == 0:
            print("PROGRESS", tick, "plans", len(plans), flush=True)
        if n.follower.exited:
            break
        for _ in range(6):
            sim.command(v, v * math.tan(a) / n.cfg.wheelbase)
            sim.step(1 / 120)
            vv, ww = sim.speed * 1.02, sim.yaw_rate
            wheel = compose(
                wheel,
                (
                    advance((0.0, 0.0, 0.0), vv / 120, ww / vv)
                    if abs(vv) > 1e-8
                    else (0.0, 0.0, ww / 120)
                ),
            )
            assert not sim.collision, sim.collision_names
    result = {
        "profile": args.profile,
        "settings": {
            "require_complete_entry_path": n.cfg.require_complete_entry_path,
            "search_enabled": n.search_enabled,
            "search_speed": n.search_speed,
            "parking_speed": n.follower.speed,
        },
        "limited_view": args.limited_view,
        "parked": n.follower.parked,
        "exited": n.follower.exited,
        "parked_inside_bay": parked_inside,
        "collisions": sim.collisions,
        "duration": trace[-1]["t"],
        "states": sorted(set(r["state"] for r in trace)),
        "initial_planner_failure_s": args.fail_planning_for,
        "trace": trace,
        "plans": plans,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2))
    print(
        "RESULT",
        json.dumps({k: v for k, v in result.items() if k not in ["trace", "plans"]}),
        flush=True,
    )
    assert (
        result["parked"] and result["exited"] and result["parked_inside_bay"]
    ), result["states"]
    assert all(
        abs(row["speed"]) <= 0.100001 for row in trace if row["state"] == "SEARCHING"
    )
    if n.cfg.require_complete_entry_path:
        assert "APPROACHING" not in result["states"], "Partial entry was executed"
    if args.limited_view or args.fail_planning_for > 0:
        assert any(
            row["state"] == "SEARCHING" and row["speed"] > 0 for row in trace
        ), "Test did not exercise forward search"
finally:
    n.destroy_node()
    rclpy.shutdown()
