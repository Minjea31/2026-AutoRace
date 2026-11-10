#!/usr/bin/env python3
"""Deterministic sensor replay in a private ROS domain; never plays hardware topics."""
import argparse
from collections import Counter
from concurrent.futures import Future
import json
import math
import os
from pathlib import Path
import sqlite3
import sys
import time
from types import SimpleNamespace


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--bag',required=True,type=Path)
    parser.add_argument('--source',required=True,type=Path)
    parser.add_argument('--output',required=True,type=Path)
    parser.add_argument('--start',type=float,default=0.)
    args=parser.parse_args()
    domain=int(os.environ.get('ROS_DOMAIN_ID','0'))
    if domain not in range(160,200) or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
        parser.error('Use ROS_DOMAIN_ID in 160..199 and ROS_LOCALHOST_ONLY=1')
    sys.path.insert(0,str(args.source.resolve()))
    import yaml,rclpy
    from rclpy.parameter import Parameter
    from rclpy.serialization import deserialize_message
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String,Bool
    import autorace_parking.node as node_module
    from autorace_parking.tracking import compose
    parameters=yaml.safe_load((args.source/'config/parking_real.yaml').read_text())['parking_node']['ros__parameters']
    rclpy.init();node=node_module.ParkingNode(parameter_overrides=[Parameter(k,value=v) for k,v in parameters.items()])
    node.worker.shutdown(wait=True)
    node.get_logger=lambda:SimpleNamespace(info=lambda *a:None,warning=lambda *a:None,error=lambda *a:None)
    state={'wall':0.,'now':0.};node_module.time=SimpleNamespace(monotonic=lambda:state['wall'])
    node.now_seconds=lambda:state['now']
    commands=[];plans=[];frames=[]
    node.publish_command=lambda v,a:commands.append({'t':state['wall'],'speed':v,'steering':a,'state':node.state,'poses':len(node.follower.path)})
    node.publish_memory=lambda:None;node.publish_target=lambda:None;node.publish_lookahead=lambda:None
    class Worker:
        def submit(self,fn,*values):
            start=time.monotonic();result=fn(*values);f=Future();f.set_result(result)
            goal=result['goal'];origin=values[3]
            plans.append({'t':state['wall'],'duration_s':time.monotonic()-start,'diagnostics':result['diagnostics'],
                          'goal':None if goal is None else compose(origin,(goal.x,goal.y,goal.yaw)),
                          'path':[compose(origin,p) for p in result['path']]})
            f.analysis_plan_index=len(plans)-1
            return f
        def shutdown(self,**kwargs):pass
    node.worker=Worker()
    original_poll=node.poll_plan
    def poll():
        pending=node.pending
        previous=node.follower.path
        original_poll()
        if pending and pending[0].done() and hasattr(pending[0],'analysis_plan_index'):
            record=plans[pending[0].analysis_plan_index]
            record['accepted']=node.follower.path is not previous
            record['retained_existing_path']=bool(node.planning_info.get('retained_valid_path'))
            record['state_after']=node.state
    node.poll_plan=poll
    types={'/scan':LaserScan,'/odometry/filtered':Odometry,'/localization/health':String}
    events=[]
    for db in sorted(args.bag.glob('*.db3')):
        conn=sqlite3.connect('file:'+str(db.resolve())+'?mode=ro',uri=True)
        topics={i:name for i,name in conn.execute('select id,name from topics') if name in types}
        ids=list(topics)
        for tid,stamp,blob in conn.execute('select topic_id,timestamp,data from messages where topic_id in ('+','.join('?' for _ in ids)+') order by timestamp',ids):
            events.append((stamp,topics[tid],blob))
        conn.close()
    events.sort(key=lambda e:e[0]);base=min(e[0] for e in events);end=max(e[0] for e in events)
    index=0;activated=False;start=time.monotonic()
    def frame():
        tracker=node.tracker;g=getattr(node,'tracked_goal',None) or node.follower.goal
        geometry={}
        if abs(round(state['wall'])-state['wall'])<.025:
            geometry={'obstacles':node.memory.local_points((0.,0.,0.)),
                      'memory_cells':[cell[:3] for cell in node.memory.cells.values()],
                      'boxes':node.memory.boxes((0.,0.,0.))}
        frames.append({'t':round(state['wall'],3),'pose':tracker.pose,'state':node.state,
                       'goal':None if g is None else [g.x,g.y,g.yaw],'confirmed':bool(g and g.confirmed),
                       'supports':len(g.support_ids) if g else 0,'path_poses':len(node.follower.path),
                       'reason':node.reason,'planning':node.planning_info,'memory':node.memory.diagnostics(),**geometry})
    try:
        tick=base
        while tick<=end:
            state['now']=tick/1e9;state['wall']=(tick-base)/1e9
            if not activated and state['wall']>=args.start:
                for _ in range(node.required_detections):node.on_sign(Bool(data=True))
                activated=True
            while index<len(events) and events[index][0]<=tick:
                _,topic,blob=events[index];index+=1;message=deserialize_message(blob,types[topic])
                if topic=='/scan':node.on_scan(message)
                elif topic=='/odometry/filtered':node.on_localization(message)
                else:node.on_fusion_health(message)
            node.tick()
            if len(frames)==0 or state['wall']-frames[-1]['t']>=.24:frame()
            tick+=50000000
        frame()
        counts=Counter(p['diagnostics'].get('reason_code') for p in plans)
        summary={'bag_seconds':(end-base)/1e9,'wall_seconds':time.monotonic()-start,'plans':len(plans),
                 'plan_codes':dict(counts),'commands':len(commands),'nonzero_commands':sum(abs(c['speed'])>.001 for c in commands),
                 'states':dict(Counter(f['state'] for f in frames)),
                 'final_goal':frames[-1]['goal'],'final_state':node.state,
                 'goal_updates':sum(p['diagnostics'].get('goal_updated',False) for p in plans),
                 'accepted_plans':sum(p.get('accepted',False) for p in plans),
                 'retained_plans':sum(p.get('retained_existing_path',False) for p in plans),
                 'rejected_nonempty_plans':sum(bool(p['path']) and not p.get('accepted',False) and not p.get('retained_existing_path',False) for p in plans),
                 'confirmed_frames':sum(f['confirmed'] for f in frames),
                 'max_planning_s':max((p['duration_s'] for p in plans),default=0.)}
        args.output.mkdir(parents=True,exist_ok=True)
        (args.output/'replay.json').write_text(json.dumps({'summary':summary,'frames':frames,'plans':plans,'commands':commands},ensure_ascii=False))
        (args.output/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2))
        print(json.dumps(summary,ensure_ascii=False,indent=2))
    finally:node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
