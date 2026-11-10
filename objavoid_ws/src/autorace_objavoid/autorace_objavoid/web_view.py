"""Read-only local HTTP viewer; rendering never changes controller inputs."""
import json
import math
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import cv2
import numpy as np
import yaml


class GroundProjection:
    """Inverse of deep_lane.cpp's ground-plane BEV, normalized to camera size."""
    def __init__(self, config):
        c = config
        w, h = int(c['width']), int(c['height'])
        if (w < 2 or h < 2 or c['bev_far_m'] <= c['bev_near_m']
                or min(c['lane_width_m'], c['lane_width_px']) <= 0
                or not all(math.isfinite(float(v)) for v in c.values())):
            raise ValueError('invalid ground projection configuration')
        src = np.float32([[0, 0], [w-1, 0], [w-1, h-1], [0, h-1]])
        dst = np.float32([[c['bev_top_left'], c['bev_top_y']],
                          [c['bev_top_right'], c['bev_top_y']],
                          [c['bev_bottom_right'], c['bev_bottom_y']],
                          [c['bev_bottom_left'], c['bev_bottom_y']]])
        self.matrix = cv2.getPerspectiveTransform(src, dst)
        self.w, self.h, self.c = w, h, c

    @classmethod
    def load(cls, filename):
        root = yaml.safe_load(Path(filename).read_text())
        return cls(root.get('lane_deep_lite', root))

    def point(self, x, y, width, height):
        c = self.c
        u = c['vehicle_center_u']*(self.w-1) - y*c['lane_width_px']/c['lane_width_m']
        v = (self.h-1)*(c['bev_far_m']-x)/(c['bev_far_m']-c['bev_near_m'])
        q = self.matrix @ np.array([u, v, 1.])
        # Behind the ground horizon must not be connected across infinity.
        if not np.all(np.isfinite(q)) or q[2] <= 1e-8:
            return None
        p = q[:2]/q[2]*[width-1, height-1]
        if not np.all(np.isfinite(p)) or np.max(np.abs(p)) > 1e7:
            return None
        return tuple(int(round(a)) for a in p)


def box_corners(box):
    x, y, hx, hy, yaw = box
    co, si = math.cos(yaw), math.sin(yaw)
    return [[x+co*dx-si*dy, y+si*dx+co*dy]
            for dx, dy in ((-hx,-hy), (hx,-hy), (hx,hy), (-hx,hy), (-hx,-hy))]


def snapshot(state, speed, angle, c, gate, scan, road, lane_points,
             nearest, scan_ok, lane_ok, white_ok, count, path, steering_side=0):
    from .node import avoidance_target, avoidance_side
    steering_reference = []
    if scan_ok and lane_ok and nearest:
        side_hint = steering_side or avoidance_side(nearest)
        ahead,lateral = avoidance_target(nearest[0],c,side_hint)
        steering_reference = [(ahead,0.),(ahead,lateral)]
    boxes = list(scan.boxes) if scan_ok else []
    footprints = []
    from .node import preview_poses
    poses = preview_poses(angle,c)
    for i in range(0, len(path), 4):
        x, y = path[i]
        footprints.append(box_corners((x, y, c.vehicle_length/2+c.clearance_margin,
                                      c.vehicle_width/2+c.clearance_margin,
                                      poses[i][2])))
    return dict(state=state, speed=speed, steering=angle, active=gate.active,
                scan_ok=scan_ok, lane_ok=lane_ok, white_ok=white_ok, count=count,
                roi=[[gate.x_min,gate.y_min], [gate.x_max,gate.y_min],
                     [gate.x_max,gate.y_max], [gate.x_min,gate.y_max], [gate.x_min,gate.y_min]],
                points=list(scan.points) if scan_ok else [],
                obstacles=[dict(outline=box_corners(b), center=list(b[:2]),
                    rank=next((i+1 for i,n in enumerate(nearest) if n == b), 0)) for b in boxes],
                lane=list(lane_points),
                road=([[[x,-(nx*x+offset)/ny] for x in (road.ranges[i] if road.ranges else (0.,c.visual_road_distance))]
                       for i,(nx,ny,offset) in enumerate(road.edges)] if hasattr(road,'edges') else
                      [[road.point(x,side*road.half_width) for x in (0.,c.visual_road_distance)]
                       for side in (-1,1)] if road else []),
                road_estimated=False if hasattr(road,'edges') else bool(road and road.half_width > c.lane_width) or not lane_ok,
                road_source='yellow_edges' if hasattr(road,'edges') else 'camera_edges' if road and road.half_width > c.lane_width else 'camera' if road else 'none',
                road_dividers=[[road.point(x,side*c.lane_width/2) for x in (0.,c.visual_road_distance)]
                               for side in (-1,1)] if road and not hasattr(road,'edges') and road.half_width > c.lane_width else [],
                body=box_corners((0.,0.,c.vehicle_length/2,c.vehicle_width/2,0.)),
                path=list(path), footprints=footprints, steering_reference=steering_reference,
                updated=time.monotonic())


def detected_lane_points(mask, projection):
    """Actual segmentation pixels in ground coordinates; no inferred boundaries."""
    if projection is None:
        return []
    h,w = mask.shape
    if min(h,w) < 2:
        return []
    rows,cols = np.nonzero((mask == 1)|(mask == 2))
    stride = max(1,math.ceil(len(rows)/1200))
    coords = np.stack((cols[::stride]/(w-1),rows[::stride]/(h-1),np.ones(len(rows[::stride]))))
    q = np.linalg.inv(projection.matrix) @ coords
    valid = q[2] > 1e-8
    q = q[:,valid]
    uv = q[:2]/q[2]
    c=projection.c
    x=c['bev_far_m']-uv[1]*(c['bev_far_m']-c['bev_near_m'])/(projection.h-1)
    y=(c['vehicle_center_u']*(projection.w-1)-uv[0])*c['lane_width_m']/c['lane_width_px']
    keep=np.isfinite(x)&np.isfinite(y)&(x>=0)&(x<=3.)&(np.abs(y)<=2.)
    return np.stack((x[keep],y[keep]),axis=1).tolist()


def render_camera(image, data, projection, white_roi, lane_mask=None):
    out = image.copy()
    h,w = out.shape[:2]
    if lane_mask is not None:
        mask=cv2.resize(lane_mask,(w,h),interpolation=cv2.INTER_NEAREST)
        detected=(mask == 1)|(mask == 2)
        out[detected]=(out[detected]*.2+np.array([245,245,245])*.8).astype(np.uint8)
    def line(points,color,thickness=2):
        previous=None
        for x,y in points:
            point=projection.point(x,y,w,h) if projection else None
            if point is not None and previous is not None:
                visible,a,b=cv2.clipLine((0,0,w,h),previous,point)
                if visible:
                    cv2.line(out,a,b,color,thickness,cv2.LINE_AA)
            previous=point
    if projection and data:
        line(data.get('roi',[]),(80,210,80))
        for obj in data.get('obstacles',[]):
            rank=obj.get('rank',0)
            color=(255,180,60) if rank==1 else (80,180,255) if rank==2 else (200,100,220) if rank==3 else (80,100,255)
            line(obj['outline'],color,3 if rank else 2)
            if rank in (1,2,3):
                point=projection.point(*obj['center'],w,h)
                if point is not None and 0<=point[0]<w and 0<=point[1]<h:
                    label=f'#{rank}'
                    cv2.putText(out,label,(point[0]+5,point[1]-5),cv2.FONT_HERSHEY_SIMPLEX,.7,(0,0,0),4,cv2.LINE_AA)
                    cv2.putText(out,label,(point[0]+5,point[1]-5),cv2.FONT_HERSHEY_SIMPLEX,.7,color,2,cv2.LINE_AA)
        # Dashes distinguish extrapolated yellow boundaries from measured pixels.
        for edge in data.get('road',[]):
            if data.get('road_source') != 'yellow_edges' or len(edge) != 2:
                continue
            a, b = np.asarray(edge[0]), np.asarray(edge[1])
            for t in np.arange(0., 1., .08):
                line([a+(b-a)*t, a+(b-a)*min(t+.04,1.)],(0,220,255))
        line(data.get('path',[]),(255,220,0),3)
        reference=data.get('steering_reference',[])
        if len(reference)==2:
            a,b=np.asarray(reference[0]),np.asarray(reference[1])
            # Sampling avoids projecting a segment across the camera horizon.
            line([a+(b-a)*t for t in np.linspace(0.,1.,80)],(255,90,230),2)
    left,right,top,bottom=white_roi
    cv2.rectangle(out,(int(left*w),int(top*h)),(int(right*w),int(bottom*h)),(80,210,80),1)
    return out


class ViewerServer(ThreadingHTTPServer):
    daemon_threads = True
    block_on_close = False


class WebViewer:
    def __init__(self, host, port, projection=None, white_roi=(.2,.8,.86,.92)):
        self.lock = threading.Lock()
        self.data = {}
        self.camera = None
        self.lane_mask = None
        self.projection, self.white_roi = projection, white_roi
        self.cached_key, self.cached_jpeg = None, None
        self.render_lock = threading.Lock()
        owner = self
        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                route = self.path.split('?')[0]
                if route == '/':
                    self.send(200,'text/html; charset=utf-8',HTML.encode())
                elif route == '/state':
                    with owner.lock:
                        data = dict(owner.data)
                        camera = owner.camera
                        lane = owner.lane_mask
                    data['lane_detection'] = lane[2] if lane and time.monotonic()-lane[1] < .5 else []
                    data['age'] = time.monotonic()-data.pop('updated',time.monotonic())
                    data['camera_age'] = time.monotonic()-camera[1] if camera else None
                    self.send(200,'application/json',json.dumps(data,allow_nan=False).encode())
                elif route == '/camera.jpg':
                    jpeg = owner.camera_jpeg()
                    self.send(200 if jpeg else 503,'image/jpeg' if jpeg else 'text/plain',jpeg or b'Waiting for camera')
                else:
                    self.send(404,'text/plain',b'Not found')
            def send(self, status, kind, body):
                self.send_response(status)
                self.send_header('Content-Type',kind)
                self.send_header('Cache-Control','no-store')
                self.send_header('Content-Length',str(len(body)))
                self.end_headers()
                try:
                    self.wfile.write(body)
                except (BrokenPipeError,ConnectionResetError):
                    pass
            def log_message(self,*args):
                pass
        self.server = ViewerServer((host,port),Handler)
        self.thread = threading.Thread(target=self.server.serve_forever,daemon=True)
        self.thread.start()

    def publish(self, data):
        with self.lock:
            self.data = data

    def set_camera(self, image, received):
        with self.lock:
            self.camera = (image, received)

    def set_lane_mask(self, mask, received):
        mask = mask.copy()
        points = detected_lane_points(mask,self.projection)
        with self.lock:
            self.lane_mask = (mask,received,points)

    def camera_jpeg(self):
        # Heavy JPEG/overlay work stays in HTTP threads; never in the control timer.
        with self.render_lock:
            with self.lock:
                camera, data, lane = self.camera, self.data, self.lane_mask
            if camera is None:
                return None
            lane_fresh = lane is not None and time.monotonic()-lane[1] < .5
            key = (camera[1],int(data.get('updated',0)*10),lane[1] if lane_fresh else None)
            if key != self.cached_key:
                image = render_camera(camera[0],data,self.projection,self.white_roi,lane[0] if lane_fresh else None)
                ok, encoded = cv2.imencode('.jpg',image,[cv2.IMWRITE_JPEG_QUALITY,80])
                if ok:
                    self.cached_jpeg, self.cached_key = encoded.tobytes(), key
            return self.cached_jpeg

    def close(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=1.)


HTML = r'''<!doctype html><html lang="ko"><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>Objavoid · Live viewer</title><style>
*{box-sizing:border-box}body{margin:0;background:#10151e;color:#e8eef6;font:15px system-ui,sans-serif}header{padding:18px 24px;border-bottom:1px solid #303b4b}h1{font-size:22px;margin:0 0 6px}small,.muted{color:#a5b5c9}main{padding:18px;display:grid;grid-template-columns:1.2fr 1fr;gap:16px}.panel{background:#192230;border:1px solid #354154;border-radius:12px;padding:14px}h2{font-size:17px;margin:0 0 10px}img,canvas{width:100%;display:block;background:#0c1119;border-radius:6px}img{aspect-ratio:4/3;object-fit:contain}canvas{aspect-ratio:1}#stats{display:flex;gap:10px;flex-wrap:wrap;margin-top:12px}.chip{background:#283448;padding:8px 12px;border-radius:7px}.ok{color:#78e6aa}.bad{color:#ffad83}label{display:inline-block;margin:8px 12px 8px 0}footer{grid-column:1/-1;line-height:1.7;color:#b1bfd0}#status{font-size:17px}button{background:#344a68;color:white;border:0;border-radius:6px;padding:8px 15px;cursor:pointer}@media(max-width:850px){main{grid-template-columns:1fr}}
</style><header><h1>Objavoid · Live viewer</h1><span id="status">연결 중…</span><div id="stats"></div></header><main>
<section class="panel"><h2>실제 카메라 · 바닥 좌표 오버레이</h2><img id="camera" alt="카메라 입력 대기"><p id="cameraStatus" class="muted">카메라 입력 대기</p><button id="pause">화면 일시정지</button></section>
<section class="panel"><h2>LiDAR · 위에서 본 모습 (전방 ↑ / 좌측 ←)</h2><canvas id="map" width="700" height="700"></canvas></section>
<footer>분홍 가로선: 중심축 ↔ 회피 목표점의 좌우 오차 · 하늘색: 현재 패스 · 흰색: 검출 차선 · 노란 점선: 연장한 도로 경계 · 파랑 #1: 첫 번째 우선 장애물 · 주황 #2: 두 번째 · 보라 #3: 세 번째 우선 장애물 · 빨강: 그 외 장애물 · 초록: ROI<br>분홍선은 1번 장애물 회피 목표점의 좌우 오차이며, 조향 유지·다음 장애물 반영·충돌 검사로 최종 조향은 달라질 수 있습니다.<br>카메라의 패스·장애물·시작 ROI는 바닥 평면 투영입니다.</footer></main>
<script>
const $=id=>document.getElementById(id),canvas=$('map'),ctx=canvas.getContext('2d');let paused=false,last=null,refreshing=false,imageLoading=false;
const pt=([x,y])=>[350-y*160,595-x*160];
function line(points,color,width=2,dashed=false){if(!points?.length)return;ctx.beginPath();ctx.setLineDash(dashed?[7,6]:[]);points.forEach((p,i)=>{let q=pt(p);i?ctx.lineTo(...q):ctx.moveTo(...q)});ctx.strokeStyle=color;ctx.lineWidth=width;ctx.stroke();ctx.setLineDash([])}
function draw(d){ctx.clearRect(0,0,700,700);line(d.roi,'#6cdd8c',2);ctx.fillStyle='#f5f5f5';for(let p of d.lane_detection||[]){let q=pt(p);ctx.fillRect(q[0]-1,q[1]-1,2,2)}if(d.road_source==='yellow_edges')for(let edge of d.road||[])line(edge,'#ffdc00',2,true);for(let o of d.obstacles||[]){const color=o.rank===1?'#3cb4ff':o.rank===2?'#ffb450':o.rank===3?'#dc64c8':'#ff6450';line(o.outline,color,o.rank?3:2);if(o.rank>=1&&o.rank<=3){let q=pt(o.center);ctx.font='bold 18px sans-serif';ctx.lineWidth=4;ctx.strokeStyle='#10151e';ctx.strokeText('#'+o.rank,q[0]+6,q[1]-6);ctx.fillStyle=color;ctx.fillText('#'+o.rank,q[0]+6,q[1]-6);}}line(d.path,'#55dcff',4);line(d.steering_reference,'#e65aff',2,true);if(d.steering_reference?.length===2){let q=pt(d.steering_reference[1]);ctx.fillStyle='#e65aff';ctx.beginPath();ctx.arc(...q,4,0,2*Math.PI);ctx.fill();}}

function chip(text,ok){let el=document.createElement('span');el.className='chip '+(ok===undefined?'':ok?'ok':'bad');el.textContent=text;return el}
async function refresh(){if(paused||refreshing)return;refreshing=true;try{const r=await fetch('/state',{cache:'no-store'});if(!r.ok)throw Error();const d=await r.json();last=d;$('status').textContent=(d.age>1?'연결 지연 · ':'')+(d.state?.startsWith('LANE_BLOCKED')?'차선 내 통과 경로 없음 · 저속':d.state?.startsWith('LANE_UNAVAILABLE')?'차선 확인 불가 · 저속':d.state?.startsWith('ROAD_UNAVAILABLE')?'노란 경계 미확인':d.state?.startsWith('NO_CLEAR_PATH')?'통과 경로 없음':d.state||'센서 입력 대기');$('stats').replaceChildren(chip('속도 '+(d.speed||0).toFixed(2)+' m/s'),chip('조향 '+((d.steering||0)*180/Math.PI).toFixed(1)+'°'),chip('LiDAR '+(d.scan_receiving?'수신 중':d.scan_ok?'마지막 스캔 사용':'첫 입력 대기'),d.scan_receiving),chip('차선 '+(d.lane_detection?.length?'검출':'미검출'),!!d.lane_detection?.length));$('cameraStatus').textContent=d.camera_age==null?'카메라 입력 대기':'카메라 수신 '+d.camera_age.toFixed(1)+'초 전'+(d.camera_age>.5?' · 영상 지연':'');draw(d);if(d.camera_age!=null&&!imageLoading){imageLoading=true;const next=new Image();next.id='camera';next.alt='카메라 오버레이';next.onload=()=>{$('camera').replaceWith(next);imageLoading=false};next.onerror=()=>{imageLoading=false};next.src='/camera.jpg?t='+Date.now()}}catch(e){$('status').textContent='뷰어 연결 끊김 · 마지막 화면 유지'}finally{refreshing=false}}
$('pause').onclick=()=>{paused=!paused;$('pause').textContent=paused?'화면 재개':'화면 일시정지';if(!paused)refresh()};setInterval(refresh,250);refresh();
</script></html>'''
