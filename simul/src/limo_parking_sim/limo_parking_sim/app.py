"""CLI and fixed-step simulation loop. Graphics and ROS share one main thread."""
import argparse
import json
import math
import sys
import time

from .config import load_config
from .model import Simulation


def parser():
    p = argparse.ArgumentParser(description='AutoRace LIMO Pro Panda3D parking simulator')
    p.add_argument('--config',help='Vehicle/sensor YAML')
    p.add_argument('--map',dest='world',help='World/parking YAML')
    p.add_argument('--headless',action='store_true',help='Offscreen EGL rendering; sensor topics still work')
    p.add_argument('--no-ros',action='store_true',help='Interactive Panda3D only')
    p.add_argument('--editor',action='store_true',help='Open the live cone, parking and obstacle placement window')
    p.add_argument('--duration',type=float,default=0,help='Exit after N simulated seconds; 0 means unlimited')
    p.add_argument('--fast',action='store_true',help='Disable wall-clock pacing')
    p.add_argument('--view',choices=('parking','overview','follow','camera'),default='parking')
    p.add_argument('--mission',choices=('parking','track','slow','crosswalk','obstacle','fork','cone','signal'),
                   help='Start before a mission or before the full-track start line')
    p.add_argument('--spawn',nargs=3,type=float,metavar=('X','Y','YAW'),help='Override initial map pose (m, rad)')
    p.add_argument('--snapshot',help='Directory to save scene.png, color.png and depth_mm.png on exit')
    return p


def main(args=None):
    raw = sys.argv[1:] if args is None else args
    # Accept ROS remappings supplied by ros2 launch/run without burdening --no-ros.
    try:
        from rclpy.utilities import remove_ros_args
        clean = remove_ros_args(['limo_parking_sim',*raw])[1:]
    except ImportError:
        clean = raw
    options = parser().parse_args(clean)
    if not math.isfinite(options.duration) or options.duration < 0:
        raise SystemExit('--duration must be finite and nonnegative')
    cfg = load_config(options.config,options.world)
    if options.mission:
        try:
            cfg['simulation']['spawn']=cfg['world']['spawns'][options.mission]
        except KeyError:
            raise SystemExit(f'This map has no spawn for mission: {options.mission}')
    if options.spawn:
        if not all(math.isfinite(v) for v in options.spawn):
            raise SystemExit('--spawn must be finite')
        cfg['simulation']['spawn'] = options.spawn
    model = Simulation(cfg)
    if model.collisions_at(model.x,model.y,model.yaw):
        raise SystemExit('Spawn intersects an obstacle or the mat boundary; adjust --spawn')
    from .scene import Scene
    scene = Scene(cfg,model,headless=options.headless)
    scene.set_view(options.view)
    bridge = None
    try:
        if options.editor:
            scene.toggle_editor()
        if not options.no_ros:
            try:
                import rclpy
                from .ros_bridge import RosBridge
            except ImportError as exc:
                raise RuntimeError('ROS 2 Python modules unavailable. Source /opt/ros/humble/setup.bash '
                                   'and use Conda Python 3.10, or pass --no-ros.') from exc
            rclpy.init(args=['limo_parking_sim',*raw])
            bridge = RosBridge(cfg,model,scene)
        dt = 1./cfg['simulation']['physics_hz']
        periods = {'color':1./cfg['camera']['color']['fps'],
                   'depth':1./cfg['camera']['depth']['fps'],
                   'scan':1./cfg['lidar']['frequency'],
                   'telemetry':1./cfg['simulation']['telemetry_hz'],
                   'status':1./cfg['simulation']['status_hz']}
        due = dict.fromkeys(periods,0.)
        wall_start = deadline = time.perf_counter()
        ticks = 0
        while not scene.closed and (options.duration == 0 or model.time < options.duration-1e-9):
            if bridge:
                if not rclpy.ok():
                    break
                rclpy.spin_once(bridge,timeout_sec=0)
            if scene.paused:
                scene.render_frame()
                time.sleep(.02)
                deadline = time.perf_counter()
                continue
            scene.keyboard_command()
            model.step(dt)
            ticks += 1
            ready = {name for name in periods if model.time+1e-9 >= due[name]}
            for name in ready:
                due[name] += periods[name]
                if due[name] < model.time-1e-9:
                    due[name] = model.time+periods[name]
            scene.render_frame(ready & {'color','depth'})
            if bridge:
                bridge.publish_clock()
                for stream in ('color','depth'):
                    if stream in ready:
                        bridge.publish_camera(stream)
                if 'telemetry' in ready:
                    bridge.publish_telemetry()
                if 'status' in ready:
                    bridge.publish_status()
            if 'scan' in ready:
                ranges = model.scan()
                scene.update_scan(ranges)
                if bridge:
                    bridge.publish_scan(ranges)
            if not options.fast:
                deadline += dt/cfg['simulation']['real_time_factor']
                now = time.perf_counter()
                if deadline > now:
                    time.sleep(deadline-now)
                elif now-deadline > .25:
                    deadline = now  # never skip physics to catch up
        if options.snapshot:
            scene.snapshot(options.snapshot)
        print(json.dumps({'final':model.status(),'physics_ticks':ticks,
                          'wall_seconds':round(time.perf_counter()-wall_start,3)},ensure_ascii=False))
    except KeyboardInterrupt:
        pass
    finally:
        if bridge:
            bridge.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        scene.close()


if __name__ == '__main__':
    main()
