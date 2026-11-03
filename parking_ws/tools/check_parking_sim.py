#!/usr/bin/env python3
"""Exercise the real simulator geometry/scan/vehicle without opening a GUI.

    python3 parking_ws/tools/check_parking_sim.py --output /tmp/parking-check

Ground truth is used ONLY by this validation tool for collision/bay assertions
and drawing. The planner and controller receive only scan points.
"""
import argparse
import json
import math
import os
from pathlib import Path
import sys
from dataclasses import replace

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'parking_ws/src/autorace_parking'))
sys.path.insert(0, str(ROOT / 'simul/src/limo_parking_sim'))

from autorace_parking.core import Activation, Scan, Settings, cluster_points
from autorace_parking.control import Follower, build_plan, local_goal
from autorace_parking.tracking import ScanTracker
from limo_parking_sim.config import load_config
from limo_parking_sim.model import Simulation


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT / 'parking_ws/captures/validation')
    parser.add_argument('--exit-heading-tolerance', type=float, default=.05,
                        help='Heading tolerance for the recorded entry-turn boundary, in radians')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    sim = Simulation(load_config())
    cfg = Settings()
    tracker, follower, activation = ScanTracker(), Follower(cfg, exit_heading_tolerance=args.exit_heading_tolerance), Activation()
    for frame in range(3):
        activation.update(True, frame / 30)
    assert activation.active
    start = (sim.x, sim.y)
    trace, first_path, first_clusters, exit_path = [], [], [], []
    parked_at = exiting_at = None
    parked_corners = None
    xmin, xmax, ymin, ymax = sim.cfg['world']['parking']['goal']
    for tick in range(600):
        now, new_scan = tick / 20, tick % 2 == 0
        if new_scan:
            scan = Scan(sim.scan(), sim.angles[0], sim.angles[1] - sim.angles[0], .01, 64.)
            assert tracker.update(scan.points), f'Scan registration failed at {tick}'
            if tick == 0:
                first_clusters = cluster_points(scan.points)
        if not follower.parked and (not follower.path or tick % 30 == 0):
            goal = local_goal(follower.goal, tracker.pose) if follower.goal else None
            result = build_plan(scan, replace(cfg, row_yaw=-tracker.pose[2]), goal, tracker.pose)
            if result[1]:
                follower.accept(*result, tracker.pose)
                if not first_path:
                    first_path = follower.path.copy()
        speed, steering, state = follower.command(scan, tracker.pose, new_scan=new_scan, now=now)
        trace.append({'time': now, 'state': state,
                      'pose': [sim.x - start[0], sim.y - start[1], sim.yaw],
                      'estimate': list(tracker.pose), 'speed': speed, 'actual_speed': sim.speed,
                      'steering': steering})
        if follower.parked and parked_at is None:
            parked_at = now
            parked_corners = sim.body().corners().copy()
            assert all(xmin <= x <= xmax and ymin <= y <= ymax for x, y in parked_corners)
            exit_path = follower.path.copy()
        if state == 'PARKED':
            assert speed == 0. and abs(sim.speed) < .03
        if speed < 0. and exiting_at is None:
            exiting_at = now
            assert exiting_at - parked_at >= 3. - 1e-9
        if follower.exited:
            break
        for _ in range(6):
            sim.command(speed, speed * math.tan(steering) / cfg.wheelbase)
            sim.step(1 / 120)
            assert not sim.collision, sim.collision_names
    assert follower.exited, 'Did not complete parking and departure within 30 seconds'
    exit_goal = exit_path[-1]
    exit_error = math.hypot(sim.x - start[0] - exit_goal[0], sim.y - start[1] - exit_goal[1])
    exit_length = sum(math.hypot(b[0]-a[0], b[1]-a[1]) for a, b in zip(exit_path, exit_path[1:]))
    assert exit_error < .09 and abs(sim.speed) < .03
    assert abs(math.atan2(math.sin(sim.yaw-exit_goal[2]), math.cos(sim.yaw-exit_goal[2]))) < .2
    assert abs(exit_goal[2]) <= follower.exit_heading_tolerance
    assert exit_goal[0] > .4, 'Exit must stop at the entry turn, not at the mission start'
    assert max(t['pose'][1] for t in trace[:round(parked_at*20)+1]) < .05, 'Wide left swing on entry'
    reverse_trace = [t for t in trace if t['time'] >= exiting_at]
    reverse_distance = sum(math.hypot(b['pose'][0]-a['pose'][0], b['pose'][1]-a['pose'][1])
                           for a,b in zip(reverse_trace, reverse_trace[1:]))
    report = {
        'parked': follower.parked, 'exited': follower.exited, 'collision_count': sim.collisions,
        'whole_vehicle_inside_bay_at_parking': True, 'duration_seconds': trace[-1]['time'],
        'parking_completed_seconds': parked_at, 'hold_duration_seconds': exiting_at - parked_at,
        'configured_speed_mps': follower.speed,
        'peak_actual_speed_mps': max(abs(t['actual_speed']) for t in trace),
        'exit_mode': 'reverse_entry_turn',
        'exit_heading_tolerance_rad': follower.exit_heading_tolerance,
        'exit_goal': list(exit_goal),
        'reverse_path_length_m': exit_length,
        'actual_reverse_distance_m': reverse_distance,
        'exit_goal_error_m': exit_error,
        'distance_from_start_after_exit_m': math.hypot(sim.x-start[0], sim.y-start[1]),
        'max_left_excursion_m': max(t['pose'][1] for t in trace[:round(parked_at*20)+1]),
        'final_map_pose': [sim.x, sim.y, sim.yaw],
        'max_tracking_position_error_m': max(math.hypot(t['pose'][0] - t['estimate'][0],
                                                       t['pose'][1] - t['estimate'][1]) for t in trace),
        'trace': trace,
    }
    (args.output / 'report.json').write_text(json.dumps(report, indent=2) + '\n')
    os.environ.setdefault('MPLCONFIGDIR', '/tmp/autorace-parking-mpl')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon, Rectangle
    from matplotlib.ticker import MultipleLocator
    fig, ax = plt.subplots(figsize=(10, 7))
    for box in sim.obstacles:
        if not (start[0] - .5 < box.x < start[0] + 2.5 and start[1] - 1.8 < box.y < start[1] + .5):
            continue
        corners = box.corners() - start
        ax.add_patch(Polygon(corners, facecolor='#687480', edgecolor='#333d46', alpha=.4))
    ax.add_patch(Rectangle((xmin - start[0], ymin - start[1]), xmax - xmin, ymax - ymin,
                           fill=False, edgecolor='#179d4c', linewidth=2, label='Empty parking bay'))
    for i, cluster in enumerate(first_clusters):
        x, y = zip(*cluster.points)
        ax.scatter(x, y, s=13, label=f'Initial scan cluster {i + 1}')
    if first_path:
        x, y, _ = zip(*first_path)
        ax.plot(x, y, '--', color='#39a34a', linewidth=2, label='Initial visible approach path')
    x, y, _ = zip(*exit_path)
    ax.plot(x, y, '--', color='#e39624', linewidth=3, label='Reverse exit path')
    for departing, color, label in ((False, '#147fce', 'Driven parking trajectory'),
                                    (True, '#ce4c14', 'Driven reverse trajectory')):
        points = [t['pose'] for t in trace if (t['time'] >= exiting_at) == departing]
        ax.plot([p[0] for p in points], [p[1] for p in points], color=color, linewidth=1.8, label=label)
    ax.add_patch(Polygon(parked_corners - start, facecolor='#22cbb3', alpha=.5,
                         edgecolor='#087063', linewidth=2, label='LIMO during 3 s parking hold'))
    ax.add_patch(Polygon(sim.body().corners() - start, facecolor='#22cbb3',
                         edgecolor='#087063', linewidth=2, label='LIMO after reverse departure'))
    ax.scatter([0], [0], marker='o', color='#147fce', zorder=5)
    ax.set(xlim=(-.35, 2.3), ylim=(-1.5, 1.35), aspect='equal',
           xlabel='Initial base frame x [m]', ylabel='Initial base frame y [m]',
           title='Straight + right-turn parking, 3 s hold, reverse entry turn only: no collisions')
    ax.xaxis.set_major_locator(MultipleLocator(.2))
    ax.yaxis.set_major_locator(MultipleLocator(.2))
    ax.grid(alpha=.25)
    ax.legend(loc='upper left', fontsize=8)
    fig.tight_layout()
    fig.savefig(args.output / 'parking.png', dpi=160)
    plt.close(fig)
    print(json.dumps({k: v for k, v in report.items() if k != 'trace'}, indent=2))
    print(f'Artifacts: {args.output}')


if __name__ == '__main__':
    main()
