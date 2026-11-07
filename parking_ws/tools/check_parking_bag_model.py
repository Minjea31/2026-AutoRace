#!/usr/bin/env python3
"""Closed-loop Ackermann model against bag-derived static obstacle snapshots.

No hardware topics are published. Sensor replay cannot close the loop because
recorded poses do not respond to newly computed commands; this model does.
"""
import argparse
from concurrent.futures import Future
from dataclasses import replace
import json
import math
import os
from pathlib import Path
import sys
from types import SimpleNamespace


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source',type=Path,required=True)
    p.add_argument('--replay',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--cases',default='early,backed_up,late,late_radius,late_dropout')
    p.add_argument('--planning-latency',type=float,default=.2)
    p.add_argument('--response-scale',type=float,default=1.)
    p.add_argument('--actuator-delay',type=float,default=0.)
    p.add_argument('--steering-time-constant',type=float,default=.08)
    p.add_argument('--map-time',type=float)
    p.add_argument('--evolving-maps',action='store_true',help='Use successive recorded map snapshots as time advances')
    p.add_argument('--scan-fov-deg',type=float,default=180.)
    args=p.parse_args()
    if int(os.environ.get('ROS_DOMAIN_ID','0')) not in range(160,200) or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
        p.error('Use a private domain 160..199 with ROS_LOCALHOST_ONLY=1')
    sys.path.insert(0,str(args.source.resolve()))
    import yaml,rclpy
    from rclpy.parameter import Parameter
    from std_msgs.msg import Bool
    from autorace_parking.core import Scan,CollisionChecker,advance
    from autorace_parking.obstacle_memory import MemoryScan
    from autorace_parking.tracking import compose,inverse
    import autorace_parking.node as module
    config=yaml.safe_load((args.source/'config/parking_real.yaml').read_text())['parking_node']['ros__parameters']
    frames=json.loads(args.replay.read_text())['frames']
    specs={'early':(12.,(0.,0.,0.),1.,0.),'backed_up':(27.,None,1.,0.),
           'late':(43.,(0.,0.,0.),1.,0.),'late_radius':(43.,(0.,0.,0.),1.1,0.),
           'late_dropout':(43.,(0.,0.,0.),1.,.5),
           'passed_goal':(36.,None,1.,0.)}
    all_results=[];rclpy.init()
    try:
        for name in args.cases.split(','):
            stamp,initial,radius_scale,dropout=specs[name]
            if args.map_time is not None:stamp=args.map_time
            frame=min((f for f in frames if f.get('memory_cells')),key=lambda f:abs(f['t']-stamp))
            cells=frame['memory_cells'];pose=tuple(frame['pose']) if initial is None else initial
            node=module.ParkingNode(parameter_overrides=[Parameter(k,value=v) for k,v in config.items()])
            node.worker.shutdown(wait=True)
            node.get_logger=lambda:SimpleNamespace(info=lambda *a:None,warning=lambda *a:None,error=lambda *a:None)
            clock={'t':1000.};module.time=SimpleNamespace(monotonic=lambda:clock['t'])
            node.now_seconds=lambda:clock['t']
            node.localization_ready=lambda now: not (dropout and 5.<now-1000.<5.+dropout)
            node.publish_memory=lambda:None;node.publish_target=lambda:None;node.publish_lookahead=lambda:None
            node.tracker.current_pose=lambda now:pose
            commands=[];plans=[];trace=[];accepted_paths=[];previous_path=None;path_id=None
            node.publish_command=lambda v,a:commands.append((v,a,node.state))
            class TimedFuture(Future):
                def __init__(self):
                    super().__init__();self.ready=clock['t']+args.planning_latency
                def done(self):return super().done() and clock['t']>=self.ready
            class Worker:
                def submit(self,fn,*values):
                    result=fn(*values);future=TimedFuture();future.set_result(result)
                    plans.append({'t':clock['t']-1000.,'code':result['diagnostics'].get('reason_code'),
                                  'path':[compose(pose,q) for q in result['path']],
                                  'trigger':result['diagnostics'].get('recovery_trigger')})
                    return future
                def shutdown(self,**kwargs):pass
            node.worker=Worker()
            for _ in range(node.required_detections):node.on_sign(Bool(data=True))
            if name=='passed_goal':
                node.follower.entry_trace=[tuple(f['pose']) for f in frames if f['t']<=stamp]
            velocity=steering=0.;collision=False;path_lost=0;had_route=False;steering_history=[]
            def snapshot():
                transform=inverse(pose)
                points=[(compose(transform,(x,y,0.))[:2],owner) for x,y,owner in cells]
                count=round(args.scan_fov_deg)+1
                half_angle=math.radians(args.scan_fov_deg)/2
                ranges=[float('inf')]*count
                for (x,y),_ in points:
                    angle=math.atan2(y,x-.103)
                    index=round((angle+half_angle)*180/math.pi)
                    if 0<=index<count:ranges[index]=min(ranges[index],math.hypot(x-.103,y))
                raw=Scan(ranges,-half_angle,math.pi/180,.05,3.,(.103,0.,0.))
                scan=MemoryScan.__new__(MemoryScan);scan.__dict__=raw.__dict__.copy()
                scan.points=[q for q,_ in points if math.hypot(*q)<=3.5]
                scan.point_owners={q:owner for q,owner in points}
                return scan
            try:
                for i in range(900):
                    clock['t']=1000.+i*.05
                    if args.evolving_maps:
                        available=[f for f in frames if f.get('memory_cells') and f['t']<=frame['t']+i*.05]
                        cells=max(available,key=lambda f:f['t'])['memory_cells'] if available else frame['memory_cells']
                    node.tracker.pose=pose;node.tracker.valid=True
                    scan=snapshot();node.scan=scan;node.scan_time=clock['t'];node.scan_sequence=i//2
                    node.planning_scan=lambda:scan
                    before=len(commands);node.tick()
                    if node.follower.path is not previous_path and not node.follower.parked:
                        previous_path=node.follower.path
                        if node.follower.path:
                            path_id=len(accepted_paths)
                            accepted_paths.append({'id':path_id,'t':i*.05,'path':list(node.follower.path),
                                                   'recovery':node.follower.recovering})
                        else:path_id=None
                    aim=node.follower.lookahead_point
                    aim_world=None if aim is None else compose(pose,aim)
                    v,a=(commands[-1][:2] if len(commands)>before else (0.,0.))
                    velocity+=(v-velocity)*min(1.,.05/.12)
                    steering_history.append(a)
                    delayed=i-round(args.actuator_delay/.05)
                    a_delayed=steering_history[delayed] if delayed>=0 else 0.
                    steering+=(a_delayed-steering)*min(1.,.05/max(.05,args.steering_time_constant))
                    next_pose=advance(pose,velocity*.05,args.response_scale*math.tan(steering)/(node.cfg.wheelbase*radius_scale))
                    # Sweep actual motion with the same body dimensions/margins.
                    if not CollisionChecker(scan,node.cfg).free(compose(inverse(pose),next_pose),allow_unknown=True):
                        collision=True;break
                    pose=next_pose
                    if had_route and not node.follower.path and node.state not in ('REOBSERVE','RECOVERY_BLOCKED'):
                        path_lost+=1
                    had_route=had_route or bool(node.follower.path)
                    trace.append({'t':i*.05,'pose':pose,'v':v,'a':a,'state':node.state,
                                  'goal':None if node.tracked_goal is None else (node.tracked_goal.x,node.tracked_goal.y),
                                  'path_poses':len(node.follower.path),'path_id':path_id,
                                  'lookahead':aim_world,'ldf':node.follower.lookahead_distance})
                    if node.follower.parked:break
                goal=node.tracked_goal or node.follower.goal
                result={'case':name,'map_t':frame['t'],'parked':node.follower.parked,'collision':collision,
                        'seconds':i*.05,'state':node.state,'path_lost_ticks':path_lost,
                        'goal_error_m':None if goal is None else math.hypot(goal.x-pose[0],goal.y-pose[1]),
                        'recovery_attempts':node.recovery_attempts,'recovery_remaining':node.recovery_remaining,
                        'response_scale':args.response_scale,'actuator_delay_s':args.actuator_delay,
                        'evolving_maps':args.evolving_maps,
                        'pose':pose,'plans':plans,'trace':trace,'accepted_paths':accepted_paths}
                all_results.append(result)
                args.output.parent.mkdir(parents=True,exist_ok=True)
                args.output.write_text(json.dumps(all_results))
                print(json.dumps({k:v for k,v in result.items() if k not in ('plans','trace','accepted_paths')}),flush=True)
            finally:node.destroy_node()
    finally:rclpy.shutdown()
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(all_results))
    return 0 if all(r['parked'] and not r['collision'] and not r['path_lost_ticks'] for r in all_results) else 1


if __name__=='__main__':raise SystemExit(main())
