"""ROS-independent telemetry, leases and RViz marker lifetime/transform semantics."""
import math
import threading
import time
from collections import deque

MISSIONS = [('overview','전체 현황'),('bump','방지턱'),('slow','서행'),('crosswalk','횡단보도'),
            ('objavoid','OBJ 회피'),('fork','갈림길'),('cone','콘 주행'),('signal','신호등'),('parking','주차'),('lane','차선 · YOLO')]

def xyz(p): return [float(p.x), float(p.y), float(p.z)]
def rgba(c): return [float(c.r), float(c.g), float(c.b), float(c.a)]
def quaternion(q): return [q.x,q.y,q.z,q.w]
def rotate(q,p):
    x,y,z,w=q; n=x*x+y*y+z*z+w*w
    if n < 1e-12: return list(p)
    u=[x,y,z]; dot=sum(a*b for a,b in zip(u,p)); cross=[y*p[2]-z*p[1],z*p[0]-x*p[2],x*p[1]-y*p[0]]
    return [((w*w-x*x-y*y-z*z)*p[i]+2*u[i]*dot+2*w*cross[i])/n for i in range(3)]
def apply(tf,p):
    t,q=tf; r=rotate(q,p); return [r[i]+t[i] for i in range(3)]
IDENTITY=([0.,0.,0.],[0.,0.,0.,1.])
def sanitize(value):
    if isinstance(value,float) and not math.isfinite(value):return None
    if isinstance(value,dict):return {k:sanitize(v) for k,v in value.items()}
    if isinstance(value,(list,tuple)):return [sanitize(v) for v in value]
    return value

def finite_points(points): return [p for p in points if all(math.isfinite(v) for v in p)]

class Leases:
    def __init__(self, clock=time.monotonic): self.clock=clock; self.clients={}
    def touch(self, client, tab, images, frame="base_link"):
        if frame not in ("base_link","odom"):raise ValueError("알 수 없는 좌표계")
        now=self.clock(); self.clients={k:v for k,v in self.clients.items() if now-v[0]<3}
        if client not in self.clients and len(self.clients)>=32: raise ValueError('최대 접속 수를 초과했습니다.')
        self.clients[client]=(now,tab,tuple(images[:2]),frame)
    def demand(self):
        now=self.clock(); active=[v for v in self.clients.values() if now-v[0]<3]
        return {v[1] for v in active}, {s for v in active for s in v[2]}

class MarkerStore:
    """Keep actions at input rate. Throttle rendering, never DELETE/DELETEALL."""
    def __init__(self,clock=time.monotonic): self.clock=clock; self.topics={}; self.revision=0
    def clear(self,topic=None):
        if topic is None: self.topics.clear()
        else: self.topics.pop(topic,None)
        self.revision+=1
    def receive(self,topic,markers):
        now=self.clock(); items=self.topics.setdefault(topic,{})
        for m in markers:
            if m.action==3: items.clear()
            elif m.action==2: items.pop((m.ns,m.id),None)
            elif m.action==0:
                duration=m.lifetime.sec+m.lifetime.nanosec*1e-9
                items[(m.ns,m.id)]={'message':m,'received':now,'expires':now+duration if duration>0 else None,'anchor':None}
        self.revision+=1
    def records(self,topic):
        now=self.clock(); items=self.topics.get(topic,{})
        for key in list(items):
            if items[key]['expires'] is not None and now>=items[key]['expires']: del items[key]
        return list(items.values())


def marker_geometry(m):
    """Marker pose applies to points too; full quaternion, XYZ, scale and RGBA."""
    pose=(xyz(m.pose.position),quaternion(m.pose.orientation)); scale=xyz(m.scale)
    points=[xyz(p) for p in m.points]; kind=int(m.type)
    if kind in (1,3):  # cube/cylinder outline; project a cylinder ellipse later via points
        if kind==1: points=[[-scale[0]/2,-scale[1]/2,0],[scale[0]/2,-scale[1]/2,0],[scale[0]/2,scale[1]/2,0],[-scale[0]/2,scale[1]/2,0],[-scale[0]/2,-scale[1]/2,0]]
        else: points=[[math.cos(a*math.pi/24)*scale[0]/2,math.sin(a*math.pi/24)*scale[1]/2,0] for a in range(49)]
    elif kind==0 and not points: points=[[0,0,0],[scale[0],0,0]]
    elif kind==2 and not points: points=[[math.cos(a*math.pi/24)*scale[0]/2,math.sin(a*math.pi/24)*scale[1]/2,0] for a in range(49)]
    return dict(type=kind,ns=m.ns,id=m.id,text=m.text,scale=scale,color=rgba(m.color),colors=[rgba(c) for c in m.colors],
                points=finite_points([apply(pose,p) for p in points]),center=pose[0],frame=m.header.frame_id,stamp=[m.header.stamp.sec,m.header.stamp.nanosec],frame_locked=m.frame_locked)

class Telemetry:
    def __init__(self,clock=time.monotonic):
        self.clock=clock; self.lock=threading.RLock(); self.values={}; self.events=deque(maxlen=300)
        self.markers=MarkerStore(clock); self.geometry={}; self.received={}; self.errors={}; self.generations={}; self.graph={}
        self.leases=Leases(clock); self.sequence=0; self.started=clock(); self.map_cache={}; self.previous_ros_time=None
    def set(self,key,value,latched=False):
        with self.lock:
            value=sanitize(value); previous=self.values.get(key); self.values[key]=(value,self.clock(),latched)
            if key=='parking_status' and isinstance(value,dict):
                gen=(value.get('mission_attempt'),value.get('synchronization',{}).get('localization_restart_count'))
                if 'parking' in self.generations and self.generations['parking']!=gen: self.clear_group('parking')
                self.generations['parking']=gen
            event_value=lambda v: {k:v.get(k) for k in ('state','reason','fork_stage','mapping_epoch')} if key=='obj_state' and isinstance(v,dict) else v
            if key not in ('cmd','parking_status','white_ratio','signal_ratio','crosswalk_ratio','lane_target','lr_detail') and (previous is None or event_value(previous[0])!=event_value(value)):
                self.sequence+=1; self.events.append(dict(id=self.sequence,time=time.strftime('%H:%M:%S'),key=key,value=event_value(value)))
    def clear_group(self,group):
        for topic in list(self.markers.topics):
            if topic.startswith('/'+group) or group=='objavoid' and topic.startswith('/objavoid_v3'):self.markers.clear(topic)
        for topic in list(self.geometry):
            if topic.startswith('/'+group):self.geometry.pop(topic,None)
        self.map_cache.clear()
    def reset(self):
        with self.lock:
            self.markers.clear(); self.geometry.clear(); self.values.clear(); self.received.clear(); self.map_cache.clear(); self.generations.clear()
            self.sequence+=1;self.events.append(dict(id=self.sequence,time=time.strftime('%H:%M:%S'),key='session',value='ROS 시간 재시작 · 캐시 초기화'))
    def get(self,key):
        item=self.values.get(key)
        return item[0] if item and (item[2] or self.clock()-item[1]<2) else None
    def state(self):
        with self.lock:
            owner=self.get('owner'); p=self.get('parking_status') or {}; rows=[]
            for key,label in MISSIONS[1:]:
                detected=self.get({'slow':'slow_sign','crosswalk':'crosswalk_sign','parking':'parking_sign','fork':'lr_sign','signal':'signal'}.get(key,key+'_sign'))
                if key=='fork' and detected is not None: detected='오른쪽' if detected else '왼쪽'
                if key=='signal' and detected is not None: detected='초록' if detected else '빨강'
                active={'slow':self.get('slow_active'),'objavoid':self.get('obj_active'),'parking':p.get('active',self.get('parking_active')),'cone':self.get('cone_exit')}.get(key)
                if key=='cone': active=None  # exit_lane_active describes exit stage, not mission activity
                done=self.get('obj_done') if key=='objavoid' else (p.get('state')=='EXITED' if key=='parking' and p else None)
                obj=self.get('obj_state') or {}
                stage={'parking':p.get('state_text') or p.get('state'),'objavoid':obj.get('state') if isinstance(obj,dict) else obj,'fork':obj.get('fork_stage') if isinstance(obj,dict) else None,'cone':self.get('cone_status'),'lane':self.get('lane_state')}.get(key)
                rows.append(dict(id=key,label=label,detected=detected,active=active,control=None if owner is None else owner==key,done=done,stage=stage))
            return dict(mode='live',owner=owner,command=self.get('cmd'),missions=rows,
                        values={k:dict(value=self.get(k),age_ms=round((self.clock()-v[1])*1000),latched=v[2]) for k,v in self.values.items()},
                        events=list(self.events),errors=dict(self.errors),uptime_s=round(self.clock()-self.started),graph=dict(self.graph))


def render_markers(store,topics,target,lookup):
    """Freeze non-frame-locked markers in odom at header time, then follow ego now."""
    layers=[];errors={}
    for topic in topics:
        for record in store.records(topic):
            m=record['message']
            if 'geometry' not in record:record['geometry']=marker_geometry(m)
            g=record['geometry']
            try:
                source=g['frame']
                if not source:raise ValueError('frame_id 없음')
                if not m.frame_locked and record['anchor'] is not None:
                    anchor=record['anchor']
                else:
                    try:
                        tf=lookup('odom',source,g['stamp'] if not m.frame_locked else None)
                        anchor=dict(g,points=[apply(tf,p) for p in g['points']],center=apply(tf,g['center']),anchor_frame='odom')
                        if not m.frame_locked:record['anchor']=anchor
                    except Exception:
                        if source!=target:raise
                        anchor=dict(g,anchor_frame=source)
                tf=lookup(target,anchor['anchor_frame'],None)
                layers.append(dict(anchor,topic=topic,key=f'{topic}|{m.ns}|{m.id}',age_ms=round((store.clock()-record['received'])*1000),
                                   points=[apply(tf,p) for p in anchor['points']],center=apply(tf,anchor['center'])))
            except Exception as e:errors[topic+' / '+g['frame']]=str(e)[:180]
    return layers,errors
