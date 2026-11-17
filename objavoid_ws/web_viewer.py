#!/usr/bin/env python3
"""Browser view of the camera, V3 planned path, scan and MarkerArray.

Run after sourcing ROS and the V3 workspace:
    python3 ~/2026-AutoRace/objavoid_ws/web_viewer.py
Open http://<vehicle-ip>:8085/ . Launch controls manage child ROS launch processes.
"""

import argparse
import json
import math
import os
import re
import shutil
import signal
import subprocess
from collections import deque
from pathlib import Path
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from sensor_msgs.msg import CompressedImage, LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String, Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rcl_interfaces.srv import GetParameters, SetParametersAtomically
from rcl_interfaces.msg import Parameter as ParameterMsg, ParameterValue, ParameterType


PAGE = r"""<!doctype html><html lang="ko"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover"><title>2026 AutoRace</title>
<style>
:root{color-scheme:dark}*{box-sizing:border-box}body{margin:0;background:#121820;color:#e9f0f7;font:14px system-ui,sans-serif}
header{display:flex;align-items:center;gap:14px;flex-wrap:wrap;padding:10px 16px;background:#202b37}
h1{font-size:17px;margin:0}.muted{color:#9baaba}#status{margin-left:auto}main{display:flex;height:calc(100vh - 58px)}
.views{flex:1;min-width:0;display:grid;grid-template-columns:1fr 1fr;gap:2px}.panel{min-width:0;min-height:0;display:flex;flex-direction:column}.panel h2{font-size:14px;margin:0;padding:8px;background:#202b37}.camera-body{flex:1;min-height:0;display:flex;align-items:center;justify-content:center;position:relative;background:#111}#camera{width:100%;height:100%;object-fit:contain}#camera-wait{position:absolute;background:#202b37;padding:8px}canvas{width:100%;flex:1;min-width:0;touch-action:none;background:#17212b}aside{width:230px;padding:13px;background:#202b37;overflow:auto}
label{display:block;margin:6px 0;cursor:pointer}input{accent-color:#4bd8e9}.swatch{display:inline-block;width:11px;height:11px;border-radius:2px;margin-right:7px}
button{background:#34495d;color:white;border:1px solid #58718a;border-radius:5px;padding:10px 12px;min-height:44px;cursor:pointer}
pre{white-space:pre-wrap;overflow-wrap:anywhere;font-size:12px}#launch-log{max-height:260px;overflow:auto}#detail{white-space:pre-wrap;overflow-wrap:anywhere;color:#c8d7e6;font-size:12px;margin-top:16px}
body{min-height:100vh;min-height:100svh}header{padding-left:max(12px,env(safe-area-inset-left));padding-right:max(12px,env(safe-area-inset-right));padding-top:max(8px,env(safe-area-inset-top))}main{height:calc(100svh - 76px)}.path-panel{order:-1}#controls>div{display:flex;align-items:center;gap:8px;margin-bottom:8px}#controls>div button:first-of-type{margin-left:auto}button.active[data-action="start"]{background:#137b47;border-color:#57e49a}button.active[data-action="stop"]{background:#a4323c;border-color:#ff929a}button#bag-start.active{background:#137b47;border-color:#57e49a}button.pending{background:#9b7315;border-color:#ffce59}button.failed{background:#a4323c;border-color:#ff929a}button:disabled{opacity:.7}#status{font-size:11px}#mux{overflow-wrap:anywhere}
@media(max-width:1000px){main{height:auto;flex-direction:column}.views{flex:none;grid-template-columns:1fr;grid-template-rows:56svh 34svh}.panel h2{font-size:12px}.path-panel{grid-row:1}aside{width:100%;overflow:visible;padding-bottom:max(16px,env(safe-area-inset-bottom))}header{gap:8px}h1{font-size:15px}#status{width:100%;margin:0}#mux{font-size:13px}}
@media(max-width:1000px) and (orientation:landscape){.views{grid-template-columns:3fr 2fr;grid-template-rows:minmax(240px,calc(100svh - 100px))}.path-panel{grid-row:auto}#controls{display:flex;flex-wrap:wrap;gap:8px 24px;align-items:center}#controls h3{margin:8px 0}#controls>div{margin:0}#controls>div button:first-of-type{margin-left:0}}
input[type=text]{width:100%;min-height:44px;background:#17212b;color:white;border:1px solid #58718a;border-radius:5px;padding:8px}.bag-row{display:flex;align-items:center;gap:8px;margin:8px 0;overflow-wrap:anywhere}.bag-row span{flex:1;min-width:0}
@media(orientation:landscape){main{width:min(100%,calc(100svh * 16 / 9));margin-inline:auto}.views{min-width:0}header{max-width:calc(100svh * 16 / 9);margin-inline:auto}}
header{display:grid;grid-template-columns:minmax(0,1fr) auto;grid-template-areas:"title reset" "mux mux" "status status";gap:6px 12px}header h1{grid-area:title}#reset{grid-area:reset}#mux{grid-area:mux;display:block;min-width:0;width:100%;height:24px;line-height:24px;white-space:nowrap;overflow:hidden;text-overflow:ellipsis;font-variant-numeric:tabular-nums}#status{grid-area:status;min-width:0;width:100%;height:18px;line-height:18px;margin:0;white-space:nowrap;overflow:hidden;text-overflow:ellipsis;font-variant-numeric:tabular-nums}
@media(orientation:landscape){main{height:auto;flex-direction:column}.views{flex:none;width:100%;aspect-ratio:16/9;grid-template-columns:3fr 2fr;grid-template-rows:minmax(0,1fr)}.path-panel{grid-row:auto}aside{width:100%;overflow:visible}}
#topic-list{max-height:320px;overflow:auto}#topic-list label{display:flex;align-items:center;gap:8px;min-height:44px;overflow-wrap:anywhere;font-size:12px}#topic-list input{width:20px;height:20px;flex-shrink:0}#topic-list small{color:#9baaba}
</style></head><body><header><h1>2026 AutoRace</h1>
<button id="reset">화면 맞춤</button><strong id="mux">MUX: 수신 대기</strong><span id="status">연결 중…</span></header><main><div class="views"><section class="panel"><h2>카메라</h2><div class="camera-body"><img id="camera" alt="카메라 영상"><span id="camera-wait">카메라 수신 대기</span></div></section><section class="panel path-panel"><h2>생성 경로 · 하늘색: 후보 / 분홍색: 선택</h2><canvas id="view"></canvas></section></div><aside>
<section id="controls"><h3>실행 제어</h3><div>Sensor <button data-service="sensor" data-action="start">시작</button> <button data-service="sensor" data-action="stop">종료</button></div><div>Tensor <button data-service="tensor" data-action="start">시작</button> <button data-service="tensor" data-action="stop">종료</button></div></section><section><h3>속도 · m/s</h3><p id="speed">측정 대기</p><div><button id="speed-down">▼ −0.1</button> <button id="speed-up">▲ +0.1</button></div><p id="speed-error"></p></section><section><h3>Bag 녹화</h3><input id="bag-name" type="text" maxlength="80" placeholder="저장할 이름" aria-label="Bag 이름"><details id="topic-picker"><summary>녹화할 토픽 선택 <span id="topic-count">0개</span></summary><p><button id="topics-refresh">목록 갱신</button> <button id="topics-all">전체 선택</button> <button id="topics-none">선택 해제</button></p><p id="topic-error"></p><div id="topic-list"></div></details><p><button id="bag-start">녹화</button> <button id="bag-stop">종료</button></p><p id="bag-status"></p><div id="bag-list"></div></section><div id="detail"></div></aside></main>
<script>
const groups=[
 ['scan','LiDAR','#e8edf2'],['scan_detections','이번 검출','#33ff66'],
 ['obstacles','추적 장애물','#ff4b4b'],['gaps','통과 간격','#29e83a'],
 ['gap_nodes','간격 노드','#4774ff'],['candidates','후보 경로(통과 간선)','#66d9ff'],['path','섬 확정 경로','#ff3399'],['approach_path','보라선 접근 경로','#a6b3bf'],
 ['verified_end','관측 확인 끝','#ffff00'],['ego','차량','#5799ff'],['target','목표점','#4166ff'],['fork_island','갈림길 섬','#9955ff'],['road_outline','도로 외곽 도면','#33ccff'],['road_match_points','도로 매칭점','#33ccff'],['island_match_points','섬 후보점','#9955ff'],['unmatched_points','미분류점','#888888'],['fork_sign','표지판 받침','#00ccff'],['fork_observation','관측 회전','#ffa600'],['fork_goal','통로 진입 목표','#4dff80'],
 ['lane_boundary_observed','관측 경계','#ffd900'],['lane_boundary_prior','사전 경계','#888888'],
 ['yellow_observed','노란 딥∩HSV 전체','#ffcc00'],['white_observed','흰 딥∩HSV 전체','#ffffff'],['lane_white_cells','누적 흰 차선','#edf1ff'],['lane_yellow_cells','노란 차선','#ff8500'],
 ['section_end','구간 끝','#ff4dff']];
const enabled=Object.fromEntries(groups.map(([key])=>[key,true]));
const canvas=document.getElementById('view'),ctx=canvas.getContext('2d');let data={},scale=120,cx=0,cy=0,rotation=Math.PI/2;
function resize(){const r=canvas.getBoundingClientRect(),d=window.devicePixelRatio||1;canvas.width=Math.round(r.width*d);canvas.height=Math.round(r.height*d);ctx.setTransform(d,0,0,d,0,0);draw()}
new ResizeObserver(resize).observe(canvas);
function fit(){scale=Math.max(35,Math.min(canvas.clientWidth/7,canvas.clientHeight/4));cx=0;cy=0;draw()}
document.getElementById('reset').onclick=fit;
function xy(x,y){const c=Math.cos(rotation),s=Math.sin(rotation),dx=x-cx,dy=y-cy;return [canvas.clientWidth/2+(c*dx-s*dy)*scale,canvas.clientHeight/2-(s*dx+c*dy)*scale]}
function line(points,color,width,closed=false){if(points.length<2)return;ctx.beginPath();points.forEach((p,i)=>{const q=xy(p[0],p[1]);i?ctx.lineTo(...q):ctx.moveTo(...q)});if(closed)ctx.closePath();ctx.strokeStyle=color;ctx.lineWidth=width;ctx.stroke()}
function dots(points,color,r){ctx.fillStyle=color;for(const p of points){const q=xy(p[0],p[1]);ctx.beginPath();ctx.arc(q[0],q[1],r,0,2*Math.PI);ctx.fill()}}
function draw(){cx=0;cy=0;rotation=Math.PI/2;const w=canvas.clientWidth,h=canvas.clientHeight;ctx.clearRect(0,0,w,h);ctx.fillStyle='#17212b';ctx.fillRect(0,0,w,h);
 const step=scale>90?.5:1,r=Math.hypot(w,h)/(2*scale);
 for(let x=Math.ceil((cx-r)/step)*step;x<=cx+r;x+=step)line([[x,cy-r],[x,cy+r]],'#293946',1);
 for(let y=Math.ceil((cy-r)/step)*step;y<=cy+r;y+=step)line([[cx-r,y],[cx+r,y]],'#293946',1);
 line([[0,cy-r],[0,cy+r]],'#5b7183',1);line([[cx-r,0],[cx+r,0]],'#5b7183',1);
 if(enabled.scan&&data.scan) dots(data.scan,'#dce8f0',1.5);
 for(const m of data.markers||[]){if(!enabled[m.ns]||m.ns==='ego')continue;const candidate=m.ns==='candidates';const col=candidate?'rgba(102,217,255,0.75)':`rgba(${Math.round(m.color[0]*255)},${Math.round(m.color[1]*255)},${Math.round(m.color[2]*255)},${m.color[3]})`;
  if(m.type===8) dots(m.points,col,Math.max(1.5,m.scale[0]*scale/2));
  else if(m.type===4) line(m.points,col,Math.max(1,m.scale[0]*scale));
  else if(m.type===5){for(let i=0;i+1<m.points.length;i+=2)line(m.points.slice(i,i+2),col,Math.max(candidate?2:1,m.scale[0]*scale))}
  else if(m.type===2) dots([m.position],col,Math.max(3,m.scale[0]*scale/2));
  else if(m.type===9&&m.text){const q=xy(...m.position);ctx.fillStyle=col;ctx.font='12px system-ui';ctx.fillText(m.text,q[0]+5,q[1]-5)}
 }
 line([[0.161,0.11],[0.161,-0.11],[-0.161,-0.11],[-0.161,0.11]],'#5799ff',2,true);
 ctx.fillStyle='#91a9bd';ctx.font='12px system-ui';ctx.fillText(`1 m = ${Math.round(scale)} px · 차량 중심 추적 · 전방 위쪽 고정 · 두 손가락/휠 확대`,12,h-12);
}
const pointers=new Map();
function gesture(){const pts=[...pointers.values()];if(!pts.length)return null;if(pts.length===1)return {x:pts[0][0],y:pts[0][1],distance:0,angle:0};const [a,b]=pts;return {x:(a[0]+b[0])/2,y:(a[1]+b[1])/2,distance:Math.hypot(b[0]-a[0],b[1]-a[1]),angle:Math.atan2(b[1]-a[1],b[0]-a[0])}}
function worldAt(x,y){const r=canvas.getBoundingClientRect(),dx=(x-r.left-canvas.clientWidth/2)/scale,dy=-(y-r.top-canvas.clientHeight/2)/scale,c=Math.cos(rotation),s=Math.sin(rotation);return [cx+c*dx+s*dy,cy-s*dx+c*dy]}
canvas.onpointerdown=e=>{e.preventDefault();pointers.set(e.pointerId,[e.clientX,e.clientY]);canvas.setPointerCapture(e.pointerId)};
canvas.onpointermove=e=>{if(!pointers.has(e.pointerId))return;e.preventDefault();const before=gesture();pointers.set(e.pointerId,[e.clientX,e.clientY]);const after=gesture();if(before.distance>1&&after.distance>1){scale=Math.max(20,Math.min(700,scale*after.distance/before.distance))}draw()};
function releasePointer(e){pointers.delete(e.pointerId)}
canvas.onpointerup=releasePointer;canvas.onpointercancel=releasePointer;canvas.onlostpointercapture=releasePointer;
canvas.onwheel=e=>{e.preventDefault();scale=Math.max(20,Math.min(700,scale*(e.deltaY>0?.85:1.18)));draw()};
const camera=document.getElementById('camera'),cameraWait=document.getElementById('camera-wait');
let cameraSeq=-1,cameraBusy=false,cameraUrl=null;
async function updateCamera(seq){cameraBusy=true;const controller=new AbortController(),timer=setTimeout(()=>controller.abort(),3000);try{const r=await fetch('/camera.jpg?seq='+seq,{cache:'no-store',signal:controller.signal});if(!r.ok)throw Error(r.status);const blob=await r.blob(),next=URL.createObjectURL(blob),previous=cameraUrl;camera.src=next;cameraUrl=next;cameraSeq=seq;if(previous)URL.revokeObjectURL(previous)}catch(e){cameraWait.hidden=false;cameraWait.textContent='영상 전송 지연 · 재시도 중'}finally{clearTimeout(timer);cameraBusy=false}}

async function refresh(){try{const r=await fetch('/data',{cache:'no-store'});if(!r.ok)throw Error(r.status);data=await r.json();
 cameraWait.hidden=data.camera_age!=null&&data.camera_age<2;
 cameraWait.textContent=data.camera_age==null?'카메라 수신 대기':`카메라 ${data.camera_age.toFixed(1)}초 전`;
 if(data.camera_seq!==cameraSeq&&data.camera_age!=null&&!cameraBusy){updateCamera(data.camera_seq);}
 document.getElementById('status').textContent=`${data.frame||'기다리는 중'} · 마커 ${data.marker_age==null?'없음':data.marker_age.toFixed(1)+'초 전'} · 스캔 ${data.scan_age==null?'없음':data.scan_age.toFixed(1)+'초 전'}`;
 document.getElementById('mux').textContent='MUX: '+(data.mux_source??'수신 대기');
 document.getElementById('speed').textContent=laneSpeed==null?'—':laneSpeed.toFixed(1);
 const candidateSegments=(data.markers||[]).filter(m=>m.ns==='candidates'&&m.type===5).reduce((n,m)=>n+Math.floor(m.points.length/2),0);
 for(const button of document.querySelectorAll('[data-service]')){const process=data.processes?.[button.dataset.service],state=process?.state;button.classList.toggle('active',button.dataset.action==='start'?state==='실행 중':state==='종료됨'||state==='미실행');button.classList.toggle('pending',state==='종료 중')}
 const mission=data.missions?.['/objavoid_v3/state']?.value; const island=mission?.island_mapping;document.getElementById('detail').textContent=`지도 초기화 번호: ${mission?.mapping_epoch??'—'} · 단계: ${({approach:'보라선 접근',await_island:'섬 확정 대기',target_selected:'목표 선택',path_locked:'경로 유지',done:'완료'})[mission?.fork_stage]||'—'}\n후보 표시 선분: ${candidateSegments}개\n`+`진입 후보: ${data.missions?.['/objavoid_v3/entrance_armed']?.value===true?'ON (5초)':'OFF'}\nobj 활성: ${data.missions?.['/objavoid_v3/active']?.value===true?'ON':'OFF'} · MUX 제어: ${data.missions?.['/objavoid_v3/has_control']?.value===true?'ON':'OFF'}\n`+(island?`섬: ${island.confirmed?'확정':island.valid?'추정 중':'미관측'} · ${island.searching?'표지판 관측 회전':'경로 추종'}\n관측 이동: ${(island.search_distance||0).toFixed(2)} m\n구간 끝→섬: ${island.section_offset_cm==null?"미확정":island.section_offset_cm.toFixed(1)+" cm"}\n섬 판정: ${island.rejection||"-"}\n도로/섬 후보점: ${data.missions?.["/objavoid_v3/state"]?.value?.road_mapping?.road_points||0} / ${data.missions?.["/objavoid_v3/state"]?.value?.road_mapping?.island_points||0}\n도로 정렬: ${island.road_aligned?"저장된 정렬 지도 사용":"사전 지도 사용 · 관측 매칭 대기"} · 잔차 ${data.missions?.["/objavoid_v3/state"]?.value?.road_mapping?.error==null?"—":(100*data.missions["/objavoid_v3/state"].value.road_mapping.error).toFixed(1)+" cm"}\nYOLO 방향: ${data.missions?.["/objavoid_v3/state"]?.value?.fork?.side||"none"} · lattice`: '');draw()}
 catch(e){document.getElementById('status').textContent='연결 끊김';}setTimeout(refresh,200)}
document.querySelectorAll('[data-service]').forEach(button=>button.onclick=async()=>{const peers=[...document.querySelectorAll('[data-service]')].filter(b=>b.dataset.service===button.dataset.service);peers.forEach(b=>{b.disabled=true;b.classList.remove('failed')});button.classList.add('pending');try{const r=await fetch('/control',{method:'POST',headers:{'Content-Type':'application/json','X-Viewer-Control':'1'},body:JSON.stringify({service:button.dataset.service,action:button.dataset.action})});const result=await r.json();if(!r.ok||result.error){button.classList.add('failed');button.title=result.message||'요청 실패'}}catch(e){button.classList.add('failed');button.title='연결 실패'}finally{peers.forEach(b=>b.disabled=false);button.classList.remove('pending')}});
let laneSpeed=null;
async function action(service,action,extra={}){const r=await fetch('/control',{method:'POST',headers:{'Content-Type':'application/json','X-Viewer-Control':'1'},body:JSON.stringify({service,action,...extra})});const data=await r.json();if(!r.ok||data.error)throw Error(data.message||'요청 실패');return data}
async function speedStep(delta){const buttons=[document.getElementById('speed-down'),document.getElementById('speed-up')];buttons.forEach(b=>b.disabled=true);try{const result=await action('speed','step',{delta});laneSpeed=result.speed;document.getElementById('speed-error').textContent=''}catch(e){document.getElementById('speed-error').textContent=e.message}finally{buttons.forEach(b=>b.disabled=false)}}
document.getElementById('speed-down').onclick=()=>speedStep(-0.1);document.getElementById('speed-up').onclick=()=>speedStep(0.1);
async function bagAction(kind,name){try{await action('bag',kind,{name,...(kind==='start'?{topics:[...selectedTopics]}:{})});document.getElementById('bag-status').textContent='';await loadBags()}catch(e){document.getElementById('bag-status').textContent=e.message}}
document.getElementById('bag-start').onclick=()=>bagAction('start',document.getElementById('bag-name').value.trim());document.getElementById('bag-stop').onclick=()=>bagAction('stop','');
const selectedTopics=new Set();let knownTopics=[],recording=false,topicsLoaded=false;
const preferredTopics=new Set(['/camera/color/image_raw/compressed','/scan','/odom','/tf','/tf_static','/objavoid_v3/state','/objavoid_v3/markers','/objavoid_v3/done','/mission/objavoid/command_05','/mux/active_source','/cmd_vel','/rosout']);
function renderTopics(){const list=document.getElementById('topic-list');list.replaceChildren();for(const item of knownTopics){const label=document.createElement('label'),check=document.createElement('input'),text=document.createElement('span');check.type='checkbox';check.checked=selectedTopics.has(item.name);check.disabled=recording;check.onchange=()=>{check.checked?selectedTopics.add(item.name):selectedTopics.delete(item.name);updateTopicCount()};text.textContent=item.name;label.title=item.types.join(', ');label.append(check,text);list.append(label)}updateTopicCount();for(const id of ['topics-refresh','topics-all','topics-none'])document.getElementById(id).disabled=recording}
function updateTopicCount(){document.getElementById('topic-count').textContent=selectedTopics.size+'개';document.getElementById('bag-start').disabled=recording||selectedTopics.size===0}
async function loadTopics(){try{const r=await fetch('/topics',{cache:'no-store'});if(!r.ok)throw Error('토픽 목록 수신 실패');const result=await r.json();knownTopics=result.topics;const available=new Set(knownTopics.map(t=>t.name));for(const topic of selectedTopics)if(!available.has(topic))selectedTopics.delete(topic);if(!topicsLoaded&&knownTopics.length){for(const topic of preferredTopics)if(available.has(topic))selectedTopics.add(topic);topicsLoaded=true}renderTopics();document.getElementById('topic-error').textContent=knownTopics.length?'':'Sensor·Tensor 시작 후 목록을 갱신하세요.'}catch(e){document.getElementById('topic-error').textContent=e.message}}
document.getElementById('topics-refresh').onclick=loadTopics;document.getElementById('topics-all').onclick=()=>{knownTopics.forEach(t=>selectedTopics.add(t.name));renderTopics()};document.getElementById('topics-none').onclick=()=>{selectedTopics.clear();renderTopics()};document.getElementById('topic-picker').ontoggle=e=>{if(e.target.open&&!recording)loadTopics()};
loadTopics();
async function loadBags(){const r=await fetch('/bags',{cache:'no-store'});if(!r.ok)throw Error('목록 수신 실패');const result=await r.json();const changed=recording!==result.recording;recording=result.recording;if(changed)renderTopics();document.getElementById('bag-name').disabled=recording;const box=document.getElementById('bag-list');box.replaceChildren();for(const bag of result.bags){const row=document.createElement('div');row.className='bag-row';const text=document.createElement('span');text.textContent=bag.name+(bag.recording?' · 녹화/저장 중':bag.complete?' · 저장됨':' · 미완료');const button=document.createElement('button');button.textContent='삭제';button.disabled=bag.recording;button.onclick=()=>{if(confirm(bag.name+' bag을 삭제할까요?'))bagAction('delete',bag.name)};row.append(text,button);box.append(row)}document.getElementById('bag-start').classList.toggle('active',result.recording);updateTopicCount();document.getElementById('bag-stop').disabled=!result.recording;document.getElementById('bag-status').textContent=result.recording?'녹화/저장 중: '+result.current:result.error||''}
async function slowRefresh(){try{await loadBags();const r=await fetch('/speed');if(r.ok){const value=(await r.json()).speed;if(value!=null)laneSpeed=value}}catch(e){}setTimeout(slowRefresh,3000)}
slowRefresh();
fit();refresh();
</script></body></html>"""


class LaunchControls:
    COMMANDS = {'sensor': ['ros2', 'launch', 'autorace_sensor_bringup', 'sensors.launch.py', 'vehicle:=04'],
                'tensor': ['ros2', 'launch', 'tensor_bringup', 'tensor_bringup.launch.py', 'objavoid_car:=04']}

    def __init__(self):
        self.lock = threading.Lock()
        self.entries = {}
        self.COMMANDS = dict(self.COMMANDS)

    def start(self, name):
        with self.lock:
            old = self.entries.get(name)
            if old and (old['process'].poll() is None or old['stopping']):
                return '이미 실행 중이거나 종료 중입니다.'
            repo = Path(__file__).resolve().parent.parent
            script = 'source /opt/ros/humble/setup.bash && source "$1/source_all_ws.bash" && exec "${@:2}"'
            process = subprocess.Popen(['bash', '-c', script, 'viewer-launch', str(repo), *self.COMMANDS[name]],
                                       cwd=repo, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                       text=True, errors='replace', start_new_session=True)
            entry = {'process': process, 'stopping': False, 'log': deque(maxlen=100)}
            self.entries[name] = entry
            threading.Thread(target=self._read, args=(entry,), daemon=True).start()
            return '시작 요청 완료. 실행 로그와 토픽 상태를 확인하세요.'

    def _read(self, entry):
        for line in entry['process'].stdout:
            with self.lock:
                entry['log'].append(line.rstrip()[-1500:])
        entry['process'].stdout.close()

    def stop(self, name):
        with self.lock:
            entry = self.entries.get(name)
            if not entry or entry['process'].poll() is not None:
                return '웹뷰어에서 실행한 프로세스가 없습니다.'
            if entry['stopping']:
                return '종료 중입니다.'
            entry['stopping'] = True
            threading.Thread(target=self._stop, args=(entry,), daemon=True).start()
            return '종료 요청 완료.'

    def _stop(self, entry):
        proc = entry['process']
        try:
            for sig, timeout in ((signal.SIGINT, 15), (signal.SIGTERM, 5), (signal.SIGKILL, 3)):
                try:
                    os.killpg(proc.pid, sig)
                except ProcessLookupError:
                    break
                try:
                    proc.wait(timeout=timeout)
                    break
                except subprocess.TimeoutExpired:
                    pass
        finally:
            with self.lock:
                entry['stopping'] = False

    def snapshot(self):
        with self.lock:
            result = {}
            for name in self.COMMANDS:
                entry = self.entries.get(name)
                code = entry['process'].poll() if entry else None
                state = ('종료 중' if entry['stopping'] else '실행 중' if code is None else '종료됨') if entry else '미실행'
                result[name] = {'state': state, 'code': code, 'log': list(entry['log']) if entry else []}
            return result

    def shutdown(self):
        # Tensor first so mission publishers stop before sensors.
        for name in ('bag', 'tensor', 'sensor'):
            self.stop(name)
            with self.lock:
                entry = self.entries.get(name)
            if entry:
                try:
                    entry['process'].wait(timeout=25)
                except subprocess.TimeoutExpired:
                    pass


class Viewer(Node):
    def __init__(self, camera_topic):
        super().__init__('objavoid_v3_web_viewer')
        self.lock = threading.Lock()
        self.controls = LaunchControls()
        self.bag_lock = threading.Lock()
        self.speed_lock = threading.Lock()
        self.bag_root = Path(__file__).resolve().parent.parent / 'bags'
        self.bag_root.mkdir(exist_ok=True)
        self.bag_name = None
        self.actual_speed = None
        self.actual_at = None
        self.command_speed = None
        self.parameter_clients = {}
        self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(Twist, '/cmd_vel', self.on_command, qos_profile_sensor_data)
        self.missions = {}
        self.status_subscriptions = []
        for topic, kind, latched in (("/objavoid_v3/state", String, False), ("/objavoid_v3/done", Bool, True),
                                     ("/objavoid_v3/active", Bool, True), ("/objavoid_v3/entrance_armed", Bool, True), ("/objavoid_v3/has_control", Bool, True),
                                     ("/parking/status", String, False), ("/parking/control_state", String, False),
                                     ("/parking/active", Bool, False), ("/start_crosswalk", Bool, False),
                                     ("/start_signal", Bool, False), ("/slow/white_roi/detected", Bool, False)):
            qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL) if latched else qos_profile_sensor_data
            self.status_subscriptions.append(self.create_subscription(kind, topic, lambda msg, t=topic: self.on_status(t, msg), qos))
        self.status_subscriptions.append(self.create_subscription(Marker, "/cone_lane/control_status", lambda msg: self.on_status("/cone_lane/control_status", msg), qos_profile_sensor_data))
        self.markers = []
        self.scan = []
        self.frame = ''
        self.marker_at = None
        self.mapping_epoch = None
        self.marker_floor_stamp_ns = 0
        self.scan_at = None
        self.camera_at = None
        self.camera_jpeg = None
        self.camera_seq = 0
        self.mux_source = None
        self.mux_at = None
        source_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, "/mux/active_source", self.on_mux, source_qos)
        self.create_subscription(CompressedImage, camera_topic, self.on_camera, qos_profile_sensor_data)
        self.create_subscription(MarkerArray, '/objavoid_v3/markers', self.on_markers, 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)

    def on_odom(self, msg):
        with self.lock:
            self.actual_speed = msg.twist.twist.linear.x
            self.actual_at = time.monotonic()

    def on_command(self, msg):
        with self.lock:
            self.command_speed = msg.linear.x

    def parameter_call(self, kind, suffix, request):
        names = {name for name, _ in self.get_node_names_and_namespaces()}
        remote = next((n for n in ('lane_fusion_pid_controller', 'lane_pid_controller') if n in names), None)
        if remote is None:
            raise ValueError('차선 제어기가 실행 중이 아닙니다.')
        key = (remote, suffix)
        if key not in self.parameter_clients:
            self.parameter_clients[key] = self.create_client(kind, '/' + remote + '/' + suffix)
        client = self.parameter_clients[key]
        if not client.wait_for_service(timeout_sec=1):
            raise ValueError('속도 파라미터 서비스가 없습니다.')
        future = client.call_async(request)
        event = threading.Event()
        future.add_done_callback(lambda _: event.set())
        if not event.wait(3):
            future.cancel()
            raise ValueError('속도 설정 응답 시간 초과')
        return future.result()

    def speed_control(self, delta=None):
        with self.speed_lock:
            request = GetParameters.Request()
            request.names = ['speed']
            response = self.parameter_call(GetParameters, 'get_parameters', request)
            if not response.values or response.values[0].type != ParameterType.PARAMETER_DOUBLE:
                raise ValueError('speed 파라미터가 없습니다.')
            speed = response.values[0].double_value
            if delta is not None:
                if delta not in (-0.1, 0.1):
                    raise ValueError('속도는 0.1 단위로 조절합니다.')
                speed = round(min(1.0, max(0.1, speed + delta)), 1)
                request = SetParametersAtomically.Request()
                request.parameters = [ParameterMsg(name='speed', value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=speed))]
                result = self.parameter_call(SetParametersAtomically, 'set_parameters_atomically', request).result
                if not result.successful:
                    raise ValueError(result.reason)
            return speed

    def bag_path(self, name):
        if not isinstance(name, str) or not re.fullmatch(r'[\w-][\w. -]{0,79}', name) or name in ('.', '..'):
            raise ValueError('이름은 문자·숫자로 시작하고 공백, _, -, .만 포함할 수 있습니다.')
        path = self.bag_root / name
        if path.is_symlink() or path.resolve().parent != self.bag_root.resolve():
            raise ValueError('잘못된 저장 경로')
        return path

    def bag_active(self):
        with self.controls.lock:
            entry = self.controls.entries.get('bag')
            return bool(entry and (entry['stopping'] or entry['process'].poll() is None))

    def bags(self):
        with self.bag_lock:
            active = self.bag_active()
            items = [{'name': p.name, 'complete': (p / 'metadata.yaml').is_file(),
                      'recording': active and p.name == self.bag_name}
                     for p in sorted(self.bag_root.iterdir(), reverse=True) if p.is_dir() and not p.is_symlink()]
            with self.controls.lock:
                entry = self.controls.entries.get('bag')
                error = '\n'.join(list(entry['log'])[-3:]) if entry and entry['process'].poll() not in (None, 0) else ''
            return {'bags': items, 'recording': active, 'current': self.bag_name, 'error': error}

    def bag_control(self, action, name, topics=None):
        with self.bag_lock:
            if action == 'stop':
                return self.controls.stop('bag')
            path = self.bag_path(name)
            if action == 'start':
                if self.bag_active():
                    raise ValueError('이미 녹화 중입니다.')
                if path.exists():
                    raise ValueError('같은 이름의 항목이 있습니다.')
                if not isinstance(topics, list) or not topics or len(topics) > 512 or any(not isinstance(t, str) for t in topics):
                    raise ValueError('녹화할 토픽을 선택하세요.')
                available = {topic for topic, _ in self.get_topic_names_and_types()}
                selected = list(dict.fromkeys(topics))
                if any(not t.startswith('/') or t not in available for t in selected):
                    raise ValueError('선택한 토픽이 현재 목록에 없습니다. 목록을 갱신하세요.')
                self.controls.COMMANDS['bag'] = ['ros2', 'bag', 'record', '-o', str(path), *selected]
                self.bag_name = name
                return self.controls.start('bag')
            if action == 'delete':
                if self.bag_active() and name == self.bag_name:
                    raise ValueError('녹화를 종료하고 저장이 끝난 뒤 삭제하세요.')
                if not path.is_dir():
                    raise ValueError('저장 항목이 없습니다.')
                shutil.rmtree(path)
                return '삭제됨'
            raise ValueError('지원하지 않는 명령')

    def on_status(self, topic, msg):
        value = msg.text if isinstance(msg, Marker) else msg.data
        state = None
        if topic == '/objavoid_v3/state':
            try:
                state = json.loads(value)
                value = {k: state[k] for k in ('state', 'reason', 'start_gate', 'road_mapping', 'island_mapping', 'fork', 'clearance_policy', 'mapping_epoch', 'fork_stage', 'startup') if k in state}
            except (ValueError, TypeError):
                pass
        with self.lock:
            if isinstance(state,dict) and 'mapping_epoch' in state:
                epoch=state['mapping_epoch']
                if epoch!=self.mapping_epoch:
                    self.mapping_epoch=epoch
                    self.markers=[];self.frame='';self.marker_at=None
                    stamp=state.get('t',0) if state.get('state')=='IDLE' else state.get('mapping_started_at',0)
                    self.marker_floor_stamp_ns=round(stamp*1e9)
            self.missions[topic] = {'value': value, 'at': time.monotonic()}

    def on_mux(self, msg):
        with self.lock:
            self.mux_source = msg.data
            self.mux_at = time.monotonic()

    def on_camera(self, msg):
        # Relay JPEG bytes directly; PNG/other compressed transports need decoding.
        payload = bytes(msg.data)
        if not payload.startswith(b'\xff\xd8'):
            self.get_logger().warning('Camera topic must provide JPEG CompressedImage', throttle_duration_sec=5.)
            return
        with self.lock:
            self.camera_jpeg = payload
            self.camera_at = time.monotonic()
            self.camera_seq += 1

    def on_markers(self, msg):
        result = []
        frame = ''
        for m in msg.markers:
            if m.action == Marker.DELETEALL:
                result.clear()
                continue
            if m.action != Marker.ADD:
                continue
            frame = m.header.frame_id or frame
            if m.type not in (Marker.POINTS, Marker.LINE_STRIP, Marker.LINE_LIST,
                              Marker.SPHERE, Marker.TEXT_VIEW_FACING):
                continue
            result.append({'ns': m.ns, 'type': m.type,
                           'points': [[p.x, p.y] for p in m.points],
                           'position': [m.pose.position.x, m.pose.position.y],
                           'scale': [m.scale.x, m.scale.y],
                           'color': [m.color.r, m.color.g, m.color.b, m.color.a],
                           'text': m.text})
        stamp=max((m.header.stamp.sec*1_000_000_000+m.header.stamp.nanosec for m in msg.markers),default=0)
        with self.lock:
            if stamp<self.marker_floor_stamp_ns:
                return
            self.markers = result
            self.frame = frame
            self.marker_at = time.monotonic()

    def on_scan(self, msg):
        # The real car's LiDAR is 0.103 m ahead of base_link (objavoid_v3.yaml).
        # This displays the same raw scan as RViz in the car's local frame.
        step = max(1, len(msg.ranges) // 1500)
        points = []
        for i in range(0, len(msg.ranges), step):
            r = msg.ranges[i]
            if math.isfinite(r) and msg.range_min <= r <= msg.range_max:
                a = msg.angle_min + i * msg.angle_increment
                points.append([round(0.103 + r * math.cos(a), 3), round(r * math.sin(a), 3)])
        with self.lock:
            self.scan = points
            self.scan_at = time.monotonic()

    def snapshot(self):
        now = time.monotonic()
        with self.lock:
            return {'speed': {'actual': self.actual_speed, 'age': now-self.actual_at if self.actual_at is not None else None, 'command': self.command_speed},
                    'processes': self.controls.snapshot(),
                    'missions': {k: {'value': v['value'], 'age': now-v['at']} for k, v in self.missions.items()},
                    'mux_source': self.mux_source,
                    'mux_age': round(now - self.mux_at, 2) if self.mux_at is not None else None,
                    'camera_seq': self.camera_seq,
                    'camera_age': round(now - self.camera_at, 2) if self.camera_at else None,
                    'markers': self.markers, 'scan': self.scan, 'frame': self.frame,
                    'marker_age': round(now - self.marker_at, 2) if self.marker_at else None,
                    'scan_age': round(now - self.scan_at, 2) if self.scan_at else None}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--read-only', action='store_true', help='Disable all launch, bag and speed controls')
    parser.add_argument('--host', default='0.0.0.0')
    parser.add_argument('--port', type=int, default=8085)
    parser.add_argument('--camera-topic', default='/camera/color/image_raw/compressed')
    args = parser.parse_args(rclpy.utilities.remove_ros_args()[1:])
    rclpy.init()
    node = Viewer(args.camera_topic)

    class Handler(BaseHTTPRequestHandler):
        def do_POST(self):
            if args.read_only:
                self.send_error(403, 'Replay viewer is read-only')
                return
            if self.path != '/control' or self.headers.get('X-Viewer-Control') != '1':
                self.send_error(403)
                return
            try:
                size = int(self.headers.get('Content-Length', '0'))
                if not 0 < size <= 131072:
                    raise ValueError('잘못된 요청 크기')
                request = json.loads(self.rfile.read(size))
                name, action = request['service'], request['action']
                result = {}
                if name == 'speed' and action == 'step':
                    result['speed'] = node.speed_control(request.get('delta'))
                    message = ''
                elif name == 'bag':
                    message = node.bag_control(action, request.get('name'), request.get('topics'))
                else:
                    if name not in LaunchControls.COMMANDS or action not in ('start', 'stop'):
                        raise ValueError('지원하지 않는 명령')
                    if action == 'start':
                        # Avoid launching over a stack already started in a terminal.
                        topics = ('/scan', '/camera/color/image_raw/compressed') if name == 'sensor' else ('/mux/active_source', '/objavoid_v3/state')
                        if any(node.count_publishers(t) for t in topics):
                            message = '이미 관련 노드가 실행 중입니다. 기존 실행을 종료한 뒤 시작하세요.'
                        else:
                            message = node.controls.start(name)
                    else:
                        message = node.controls.stop(name)
                body = json.dumps({'message': message, **result}, ensure_ascii=False).encode()
            except (ValueError, KeyError, TypeError, OSError) as error:
                body = json.dumps({'error': True, 'message': str(error)}, ensure_ascii=False).encode()
                self.send_response(400)
                self.send_header('Content-Type', 'application/json; charset=utf-8')
                self.send_header('Content-Length', str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return
            self.send_response(200)
            self.send_header('Content-Type', 'application/json; charset=utf-8')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path.split('?')[0] == '/':
                page = PAGE
                if args.read_only:
                    page = page.replace('<body>', '<body><style>#controls,aside>section{display:none}</style>')
                body, kind = page.encode(), 'text/html; charset=utf-8'
            elif self.path.split('?')[0] == '/camera.jpg':
                with node.lock:
                    body = node.camera_jpeg
                if body is None:
                    self.send_error(503, 'Waiting for camera JPEG')
                    return
                kind = 'image/jpeg'
            elif self.path == '/topics':
                body, kind = json.dumps({'topics': [{'name': name, 'types': types} for name, types in sorted(node.get_topic_names_and_types())]}, ensure_ascii=False).encode(), 'application/json'
            elif self.path == '/bags':
                body, kind = json.dumps(node.bags(), ensure_ascii=False).encode(), 'application/json'
            elif self.path == '/speed':
                try:
                    value = node.speed_control()
                except (ValueError, RuntimeError):
                    value = None
                body, kind = json.dumps({'speed': value}).encode(), 'application/json'
            elif self.path.split('?')[0] == '/data':
                body, kind = json.dumps(node.snapshot(), allow_nan=False).encode(), 'application/json'
            else:
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header('Content-Type', kind)
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            try:
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def log_message(self, *_args):
            pass

    try:
        server = ThreadingHTTPServer((args.host, args.port), Handler)
    except OSError:
        node.destroy_node()
        rclpy.try_shutdown()
        raise
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    node.get_logger().info(f'Web viewer: http://{args.host}:{args.port}/')
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.controls.shutdown()
        server.shutdown()
        server.server_close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
