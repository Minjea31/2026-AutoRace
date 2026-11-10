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
    p.add_argument('--first-planning-latency',type=float,help='Cold first job delay; later jobs use planning-latency')
    p.add_argument('--response-scale',type=float,default=1.)
    p.add_argument('--actuator-delay',type=float,default=0.)
    p.add_argument('--steering-time-constant',type=float,default=.08)
    p.add_argument('--map-time',type=float)
    p.add_argument('--evolving-maps',action='store_true',help='Use successive recorded map snapshots as time advances')
    p.add_argument('--scan-fov-deg',type=float,default=180.)
    p.add_argument('--scan-age',type=float,default=0.,help='Acquisition age of the completed scan, seconds')
    p.add_argument('--raw-scan-age',type=float,help='Latest received acquisition age, independent of processing delay')
    p.add_argument('--through-exit',action='store_true')
    p.add_argument('--lane-observation',choices=('none','fresh','lost_during_stop'),default='none',
                   help='Synthetic lane observations for isolated handoff validation')
    p.add_argument('--mux-source',type=Path,help='Exercise actual arbitration and parking watchdog instead of assuming missing commands mean zero')
    p.add_argument('--clearance',type=float,help='Explicit validation-only margin override')
    p.add_argument('--start-pose',type=float,nargs=3,help='Explicit common mission-frame start')
    p.add_argument('--scene-start',action='store_true',help='Use the selected scene pose as the common start')
    p.add_argument('--locked-goal',action='store_true',help='Retain the selected scene target for before/after comparison')
    args=p.parse_args()
    if int(os.environ.get('ROS_DOMAIN_ID','0')) not in range(160,200) or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
        p.error('Use a private domain 160..199 with ROS_LOCALHOST_ONLY=1')
    sys.path.insert(0,str(args.source.resolve()))
    if args.mux_source:sys.path.insert(0,str(args.mux_source.resolve()))
    import yaml,rclpy
    from rclpy.parameter import Parameter
    from std_msgs.msg import Bool
    from autorace_parking.core import Scan,CollisionChecker,advance
    from autorace_parking.obstacle_memory import MemoryScan
    from autorace_parking.tracking import compose,inverse
    import autorace_parking.node as module
    config=yaml.safe_load((args.source/'config/parking_real.yaml').read_text())['parking_node']['ros__parameters']
    # This harness starts at a recorded mission scene. Approach calibration
    # and the camera trigger are exercised by validate_approach_pipeline.py.
    config.update(white_roi_enabled=False,parking_calibration_stop=False,manual_mode_guard=False,async_scan_matching=False,planner_prewarm=False)
    if args.clearance is not None:config['clearance']=args.clearance
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
            if args.scene_start:pose=tuple(frame['pose'])
            if args.start_pose is not None:pose=tuple(args.start_pose)
            node=module.ParkingNode(parameter_overrides=[Parameter(k,value=v) for k,v in config.items()])
            node.worker.shutdown(wait=True)
            node.get_logger=lambda:SimpleNamespace(info=lambda *a:None,warning=lambda *a:None,error=lambda *a:None)
            clock={'t':1000.};module.time=SimpleNamespace(monotonic=lambda:clock['t'])
            node.now_seconds=lambda:clock['t']
            node.localization_ready=lambda now: not (dropout and 5.<now-1000.<5.+dropout)
            node.steering_model_ready=lambda:True
            node.publish_memory=lambda:None;node.publish_target=lambda:None;node.publish_lookahead=lambda:None
            node.tracker.current_pose=lambda now:pose
            commands=[];plans=[];trace=[];accepted_paths=[];previous_path=None;path_id=None
            handoffs=[];stop_seen=False
            def record_command(v,a):
                commands.append((v,a,node.state))
                if args.lane_observation!='none' and v<0 and node.follower.parked and node.follower.exiting:
                    node.exit_lane_gate.begin(node.get_clock().now().nanoseconds,clock['t'])
            if args.mux_source:
                from autorace_mux.core import Arbitrator,Config,TwistConverter
                from autorace_mux.parking_guard import ParkingGuard
                arb=Arbitrator(Config());guard=ParkingGuard();convert=TwistConverter(Config())
                original_record=record_command
                def publish_to_mux(v,a):
                    original_record(v,a)
                    arb.receive('parking',v,a,int(clock['t']*1e9),clock['t'],int(clock['t']*1e9))
                node.publish_command=publish_to_mux
                if hasattr(node,'guard_pub'):
                    node.guard_pub=SimpleNamespace(publish=lambda m:guard.receive(json.loads(m.data),clock['t'],clock['t']))
            else:node.publish_command=record_command
            actual_handoff=node.try_exit_lane_handoff
            def check_handoff():
                path=list(node.follower.path);index=node.follower.index
                was_handed=node.exit_lane_handed_off
                actual_handoff()
                if node.exit_lane_handed_off and not was_handed:
                    nearest=min(range(min(index,len(path)-1),len(path)),
                                key=lambda i:math.dist(path[i][:2],pose[:2])) if path else 0
                    handoffs.append({'t':clock['t']-1000.,'pose':pose,'path':path,
                        'remaining_path_m':(math.dist(pose[:2],path[nearest][:2])+
                            sum(math.dist(a[:2],b[:2]) for a,b in zip(path[nearest:],path[nearest+1:]))) if path else 0.,
                        'measured_speed':node.follower.measured_speed})
            node.try_exit_lane_handoff=check_handoff
            class TimedFuture(Future):
                def __init__(self):
                    super().__init__();self.ready=clock['t']+(args.first_planning_latency if args.first_planning_latency is not None and not plans else args.planning_latency)
                def done(self):return super().done() and clock['t']>=self.ready
            class Worker:
                def submit(self,fn,*values):
                    result=fn(*values);future=TimedFuture();future.set_result(result)
                    if isinstance(result,dict):
                        plans.append({'t':clock['t']-1000.,'code':result['diagnostics'].get('reason_code'),
                                      'duration_s':result['diagnostics'].get('duration_s',0.),
                                      'path':[compose(pose,q) for q in result['path']],
                                      'trigger':result['diagnostics'].get('recovery_trigger')})
                    return future
                def shutdown(self,**kwargs):pass
            node.worker=Worker()
            for _ in range(node.required_detections):node.on_sign(Bool(data=True))
            if args.locked_goal and frame.get('goal') is not None:
                from autorace_parking.core import Gap
                node.slot_map=frame.get('layout')
                node.tracked_goal=Gap(*frame['goal'],.5,0,1,confirmed=True,kind=frame.get('goal_kind','enclosed_center'),
                    slot_id=frame.get('slot_id',-1),support_ids=tuple(frame.get('support_ids',())),
                    heading_options=tuple(frame.get('heading_options',())),stop_tolerance=frame.get('stop_tolerance',.03))
            if name=='passed_goal':
                node.follower.entry_trace=[tuple(f['pose']) for f in frames if f['t']<=stamp]
            velocity=steering=0.;collision=False;physical_collision=False;unsafe_observation=False
            path_lost=0;had_route=False;steering_history=[]
            parking_error=None;parked_pose=None
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
                for i in range(2000 if args.through_exit else 900):
                    clock['t']=1000.+i*.05
                    if args.evolving_maps:
                        available=[f for f in frames if f.get('memory_cells') and f['t']<=frame['t']+i*.05]
                        cells=max(available,key=lambda f:f['t'])['memory_cells'] if available else frame['memory_cells']
                    node.tracker.pose=pose;node.tracker.valid=True
                    scan=snapshot();node.scan=scan;node.scan_time=clock['t']-args.scan_age;node.scan_sequence=i//2
                    node.last_scan_received_stamp=clock['t']-(args.scan_age if args.raw_scan_age is None else args.raw_scan_age)
                    # The injected fixture is a valid remembered world map.
                    # Populate the node's memory too: leaving its stamp unset
                    # silently disabled every memory-checked recovery in models.
                    node.memory.stamp=node.scan_time
                    node.memory.cells={(j,0):[x,y,owner,0] for j,(x,y,owner) in enumerate(cells)}
                    node.planning_scan=lambda:scan
                    stop_seen=stop_seen or getattr(node.follower,'exit_handoff_hold',False)
                    if (args.lane_observation!='none' and node.follower.exiting and
                            not (args.lane_observation=='lost_during_stop' and stop_seen)):
                        node.exit_lane_gate.observe_target([1.,0.,0.,1.,0.,.9,2.,0.],clock['t'])
                        node.exit_lane_gate.observe_command(.3,0.,node.get_clock().now().nanoseconds,clock['t'])
                        node.try_exit_lane_handoff()
                    node.fusion_health={'ready':True,'stamp':clock['t']}
                    node.fusion_health_received=clock['t']
                    before=len(commands);node.tick()
                    if node.follower.path is not previous_path:
                        previous_path=node.follower.path
                        if node.follower.path:
                            path_id=len(accepted_paths)
                            accepted_paths.append({'id':path_id,'t':i*.05,'path':list(node.follower.path),
                                                   'recovery':node.follower.recovering,'exit':node.follower.exiting or node.follower.parked})
                        else:path_id=None
                    aim=node.follower.lookahead_point
                    aim_world=None if aim is None else compose(pose,aim)
                    v,a=(commands[-1][:2] if len(commands)>before else (0.,0.))
                    if args.mux_source:
                        arb.receive('lane',.3,0.,int(clock['t']*1e9),clock['t'],int(clock['t']*1e9))
                        candidate=arb.select(clock['t'],int(clock['t']*1e9))
                        if hasattr(node,'guard_pub'):candidate=guard.apply(candidate,clock['t'],clock['t'])
                        # Steering remains an Ackermann angle; converter tests cover wire conversion.
                        v,a=(candidate.speed,candidate.steering_angle) if candidate else (0.,0.)
                    velocity+=(v-velocity)*min(1.,.05/.12)
                    steering_history.append(a)
                    delayed=i-round(args.actuator_delay/.05)
                    a_delayed=steering_history[delayed] if delayed>=0 else 0.
                    steering+=(a_delayed-steering)*min(1.,.05/max(.05,args.steering_time_constant))
                    response=args.response_scale*(node.cfg.reverse_steering_response_scale if velocity<0 else 1.)
                    next_pose=advance(pose,velocity*.05,response*math.tan(steering)/(node.cfg.wheelbase*radius_scale))
                    # Sweep actual motion with the same body dimensions/margins.
                    if not CollisionChecker(scan,node.cfg).free(compose(inverse(pose),next_pose),allow_unknown=True):
                        collision=True
                        unsafe_observation=not CollisionChecker(scan,node.cfg).free((0.,0.,0.),allow_unknown=True)
                        physical_collision=not CollisionChecker(scan,replace(node.cfg,clearance=0.)).free(
                            compose(inverse(pose),next_pose),allow_unknown=True)
                        break
                    pose=next_pose
                    if had_route and not node.follower.path and node.state not in ('REOBSERVE','RECOVERY_BLOCKED','EXITED'):
                        path_lost+=1
                    had_route=had_route or bool(node.follower.path)
                    trace.append({'t':i*.05,'pose':pose,'v':v,'a':a,'state':node.state,
                                  'goal':None if node.tracked_goal is None else (node.tracked_goal.x,node.tracked_goal.y),
                                  'path_poses':len(node.follower.path),'path_id':path_id,
                                  'lookahead':aim_world,'ldf':node.follower.lookahead_distance})
                    if node.follower.parked and parked_pose is None:
                        parked_pose=pose
                        target=node.tracked_goal
                        parking_error=math.hypot(target.x-pose[0],target.y-pose[1]) if target is not None else None
                    if node.follower.exited or node.state=='SEARCH_LIMIT' or (node.follower.parked and not args.through_exit):break
                goal=node.tracked_goal or node.follower.goal
                result={'case':name,'map_t':frame['t'],'parked':node.follower.parked,'exited':node.follower.exited,'collision':collision,
                        'seconds':i*.05,'state':node.state,'path_lost_ticks':path_lost,
                        'physical_collision':physical_collision,'new_observation_already_inside_margin':unsafe_observation,
                        'parking_error_m':parking_error,'parked_pose':parked_pose,
                        'exit_goal_error_m':None if node.follower.goal is None else math.hypot(node.follower.goal.x-pose[0],node.follower.goal.y-pose[1]),
                        'recovery_attempts':node.recovery_attempts,'recovery_remaining':node.recovery_remaining,
                        'response_scale':args.response_scale,'actuator_delay_s':args.actuator_delay,
                        'clearance_m':node.cfg.clearance,'exit_reason':node.follower.exit_route_reason,'exit_plan_attempts':node.follower.exit_plan_attempts if hasattr(node.follower,'exit_plan_attempts') else None,
                        'evolving_maps':args.evolving_maps,'scan_age_s':args.scan_age,'raw_scan_age_s':args.raw_scan_age,
                        'fixed_slot_id':node.cfg.fixed_parking_slot_id,
                        'selected_slot_id':goal.slot_id if goal else None,
                        'goal_metadata':vars(node.follower.goal) if node.follower.goal is not None else None,
                        'pose':pose,'plans':plans,'trace':trace,'accepted_paths':accepted_paths}
                result.update(lane_observation=args.lane_observation,lane_handoff=node.exit_lane_handed_off,handoffs=handoffs)
                all_results.append(result)
                args.output.parent.mkdir(parents=True,exist_ok=True)
                args.output.write_text(json.dumps(all_results))
                print(json.dumps({k:v for k,v in result.items() if k not in ('plans','trace','accepted_paths')}),flush=True)
            finally:node.destroy_node()
    finally:rclpy.shutdown()
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(all_results))
    return 0 if all(r['parked'] and (r['exited'] or not args.through_exit) and not r['collision'] and not r['path_lost_ticks'] for r in all_results) else 1


if __name__=='__main__':raise SystemExit(main())
