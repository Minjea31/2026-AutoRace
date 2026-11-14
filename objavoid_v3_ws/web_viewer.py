#!/usr/bin/env python3
"""Standalone browser view of the V3 RViz scan and MarkerArray.

Run after sourcing ROS and the V3 workspace:
    python3 ~/2026-AutoRace/objavoid_v3_ws/web_viewer.py
Open http://<vehicle-ip>:8085/ . This process only subscribes to ROS topics.
"""

import argparse
import json
import math
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray


PAGE = r"""<!doctype html><html lang="ko"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>ObjAvoid V3</title>
<style>
:root{color-scheme:dark}*{box-sizing:border-box}body{margin:0;background:#121820;color:#e9f0f7;font:14px system-ui,sans-serif}
header{display:flex;align-items:center;gap:14px;flex-wrap:wrap;padding:10px 16px;background:#202b37}
h1{font-size:17px;margin:0}.muted{color:#9baaba}#status{margin-left:auto}main{display:flex;height:calc(100vh - 58px)}
canvas{flex:1;min-width:0;touch-action:none;background:#17212b}aside{width:230px;padding:13px;background:#202b37;overflow:auto}
label{display:block;margin:6px 0;cursor:pointer}input{accent-color:#4bd8e9}.swatch{display:inline-block;width:11px;height:11px;border-radius:2px;margin-right:7px}
button{background:#34495d;color:white;border:1px solid #58718a;border-radius:5px;padding:5px 9px;cursor:pointer}
#detail{white-space:pre-wrap;overflow-wrap:anywhere;color:#c8d7e6;font-size:12px;margin-top:16px}
@media(max-width:700px){main{flex-direction:column}aside{width:100%;height:155px;order:-1}canvas{min-height:300px}}
</style></head><body><header><h1>ObjAvoid V3</h1><span class="muted">RViz /scan · /objavoid_v3/markers</span>
<button id="reset">화면 맞춤</button><span id="status">연결 중…</span></header><main><canvas id="view"></canvas><aside>
<div id="legend"></div><div id="detail"></div></aside></main>
<script>
const groups=[
 ['scan','LiDAR','#e8edf2'],['lidar_roi_points','검출 관심 점','#ff4dcc'],['scan_detections','이번 검출','#33ff66'],
 ['obstacles','추적 장애물','#ff4b4b'],['obstacle_label','장애물 크기','#ffffff'],['gaps','통과 간격','#29e83a'],
 ['gap_nodes','간격 노드','#4774ff'],['candidates','후보 경로','#999999'],['path','선택 경로','#ff3399'],
 ['verified_end','검증 끝','#ffff00'],['ego','차량','#5799ff'],['target','목표점','#4166ff'],['lookahead','LFD(곡률 미리보기)','#ff9900'],['fork_island','갈림길 섬','#9955ff'],['fork_sign','표지판 받침','#00ccff'],
 ['lane_boundary_observed','관측 경계','#ffd900'],['lane_boundary_prior','사전 경계','#888888'],
 ['lane_white_cells','흰 차선','#edf1ff'],['lane_yellow_cells','노란 차선','#ff8500'],
 ['section_end','구간 끝','#ff4dff'],['state','상태','#ffffff']];
const enabled={};document.getElementById('legend').innerHTML=groups.map(([k,n,c])=>{enabled[k]=true;return `<label><input type="checkbox" data-key="${k}" checked> <span class="swatch" style="background:${c}"></span>${n}</label>`}).join('');
document.getElementById('legend').onchange=e=>{if(e.target.dataset.key)enabled[e.target.dataset.key]=e.target.checked};
const canvas=document.getElementById('view'),ctx=canvas.getContext('2d');let data={},scale=120,cx=0,cy=0,drag=false,last=null;
function resize(){const r=canvas.getBoundingClientRect(),d=window.devicePixelRatio||1;canvas.width=Math.round(r.width*d);canvas.height=Math.round(r.height*d);ctx.setTransform(d,0,0,d,0,0);draw()}
new ResizeObserver(resize).observe(canvas);
function fit(){scale=Math.max(35,Math.min(canvas.clientWidth/7,canvas.clientHeight/4));cx=0;cy=0;draw()}
document.getElementById('reset').onclick=fit;
function xy(x,y){return [canvas.clientWidth/2+(x-cx)*scale,canvas.clientHeight/2-(y-cy)*scale]}
function line(points,color,width,closed=false){if(points.length<2)return;ctx.beginPath();points.forEach((p,i)=>{const q=xy(p[0],p[1]);i?ctx.lineTo(...q):ctx.moveTo(...q)});if(closed)ctx.closePath();ctx.strokeStyle=color;ctx.lineWidth=width;ctx.stroke()}
function dots(points,color,r){ctx.fillStyle=color;for(const p of points){const q=xy(p[0],p[1]);ctx.beginPath();ctx.arc(q[0],q[1],r,0,2*Math.PI);ctx.fill()}}
function draw(){const w=canvas.clientWidth,h=canvas.clientHeight;ctx.clearRect(0,0,w,h);ctx.fillStyle='#17212b';ctx.fillRect(0,0,w,h);
 const step=scale>90?.5:1;let x0=cx-w/(2*scale),x1=cx+w/(2*scale),y0=cy-h/(2*scale),y1=cy+h/(2*scale);
 ctx.strokeStyle='#293946';ctx.lineWidth=1;for(let x=Math.ceil(x0/step)*step;x<x1;x+=step){let q=xy(x,0);ctx.beginPath();ctx.moveTo(q[0],0);ctx.lineTo(q[0],h);ctx.stroke()}
 for(let y=Math.ceil(y0/step)*step;y<y1;y+=step){let q=xy(0,y);ctx.beginPath();ctx.moveTo(0,q[1]);ctx.lineTo(w,q[1]);ctx.stroke()}
 const o=xy(0,0);ctx.strokeStyle='#5b7183';ctx.beginPath();ctx.moveTo(o[0],0);ctx.lineTo(o[0],h);ctx.moveTo(0,o[1]);ctx.lineTo(w,o[1]);ctx.stroke();
 if(enabled.scan&&data.scan) dots(data.scan,'#dce8f0',1.5);
 for(const m of data.markers||[]){if(!enabled[m.ns])continue;const col=`rgba(${Math.round(m.color[0]*255)},${Math.round(m.color[1]*255)},${Math.round(m.color[2]*255)},${m.color[3]})`;
  if(m.type===8) dots(m.points,col,Math.max(1.5,m.scale[0]*scale/2));
  else if(m.type===5) line(m.points,col,Math.max(1,m.scale[0]*scale));
  else if(m.type===6){for(let i=0;i+1<m.points.length;i+=2)line(m.points.slice(i,i+2),col,Math.max(1,m.scale[0]*scale))}
  else if(m.type===2) dots([m.position],col,Math.max(3,m.scale[0]*scale/2));
  else if(m.type===9&&m.text){const q=xy(...m.position);ctx.fillStyle=col;ctx.font='12px system-ui';ctx.fillText(m.text,q[0]+5,q[1]-5)}
 }
 ctx.fillStyle='#91a9bd';ctx.font='12px system-ui';ctx.fillText(`1 m = ${Math.round(scale)} px · 좌클릭 드래그 이동 · 휠 확대`,12,h-12);
}
canvas.onpointerdown=e=>{drag=true;last=[e.clientX,e.clientY];canvas.setPointerCapture(e.pointerId)};
canvas.onpointerup=()=>drag=false;canvas.onpointermove=e=>{if(!drag)return;cx-=(e.clientX-last[0])/scale;cy+=(e.clientY-last[1])/scale;last=[e.clientX,e.clientY];draw()};
canvas.onwheel=e=>{e.preventDefault();scale=Math.max(20,Math.min(700,scale*(e.deltaY>0?.85:1.18)));draw()};
async function refresh(){try{const r=await fetch('/data',{cache:'no-store'});if(!r.ok)throw Error(r.status);data=await r.json();
 document.getElementById('status').textContent=`${data.frame||'기다리는 중'} · 마커 ${data.marker_age==null?'없음':data.marker_age.toFixed(1)+'초 전'} · 스캔 ${data.scan_age==null?'없음':data.scan_age.toFixed(1)+'초 전'}`;
 document.getElementById('detail').textContent=(data.markers||[]).filter(m=>m.ns==='state').map(m=>m.text).join('\n') || 'V3 마커를 기다리는 중…';draw()}
 catch(e){document.getElementById('status').textContent='연결 끊김';}setTimeout(refresh,200)}
fit();refresh();
</script></body></html>"""


class Viewer(Node):
    def __init__(self):
        super().__init__('objavoid_v3_web_viewer')
        self.lock = threading.Lock()
        self.markers = []
        self.scan = []
        self.frame = ''
        self.marker_at = None
        self.scan_at = None
        self.create_subscription(MarkerArray, '/objavoid_v3/markers', self.on_markers, 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)

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
        with self.lock:
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
            return {'markers': self.markers, 'scan': self.scan, 'frame': self.frame,
                    'marker_age': round(now - self.marker_at, 2) if self.marker_at else None,
                    'scan_age': round(now - self.scan_at, 2) if self.scan_at else None}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='0.0.0.0')
    parser.add_argument('--port', type=int, default=8085)
    args = parser.parse_args()
    rclpy.init()
    node = Viewer()

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path.split('?')[0] == '/':
                body, kind = PAGE.encode(), 'text/html; charset=utf-8'
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
        rclpy.shutdown()
        raise
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    node.get_logger().info(f'Web viewer: http://{args.host}:{args.port}/')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        server.server_close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
