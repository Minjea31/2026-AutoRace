const $ = id => document.getElementById(id);
const canvas = $('map'), ctx = canvas.getContext('2d');
const view = {scale: 125, panX: 0, panY: 0, width: 800, height: 650};
const visible = {scan: true, memory: true, slots: true, path: true, roi: true};
const colors = {scan: '#ff9575', memory: '#b59edc', empty: '#63d7e8',
  occupied: '#f5957c', target: '#ff77b0', unknown: '#a6b6bf', path: '#85e0b4', roi: '#d5ba77'};
let snapshot = null, fitted = '', polling = false;
const slotElements = [];
for (let i = 0; i < 6; i++) {
  const el = document.createElement('div'); el.className = 'slot';
  const num = document.createElement('b'), text = document.createElement('span');
  num.textContent = String(i + 1); text.textContent = '대기';
  el.append(num, text); $('slot-grid').append(el); slotElements.push(el);
}
const healthItems = {};
for (const [key, name] of [['scan_age_s', '라이다'], ['odom_age_s', 'EKF 위치'], ['health_age_s', '센서 융합']]) {
  const el = document.createElement('div'); el.className = 'input-item';
  const title = document.createElement('span'), value = document.createElement('strong');
  title.textContent = name; value.textContent = '—'; el.append(title, value);
  $('input-health').append(el); healthItems[key] = el;
}
$('host').textContent = location.host;
const number = (v, digits = 2) => Number.isFinite(v) ? v.toFixed(digits) : '—';
const metric = (id, value, unit = '') => {
  $(id).replaceChildren(document.createTextNode(value));
  if (unit) { const small = document.createElement('small'); small.textContent = unit; $(id).append(small); }
};
function position(point) {
  return [view.width/2 - point[1]*view.scale + view.panX,
    view.height*.65 - point[0]*view.scale + view.panY];
}
function toBase(point, pose) {
  if (!pose) return point;
  const x = point[0]-pose[0], y = point[1]-pose[1], c = Math.cos(pose[2]), s = Math.sin(pose[2]);
  return [c*x+s*y, -s*x+c*y];
}
function layoutData() {
  const status = snapshot?.status;
  if (!snapshot?.active || !status?.localization?.valid || !status.slot_map) return null;
  return status.slot_map;
}
function corners(cell, layout) {
  const w = layout.cell_width/2, h = layout.cell_depth/2;
  const c = Math.cos(layout.yaw), s = Math.sin(layout.yaw);
  return [[-w,-h],[w,-h],[w,h],[-w,h]].map(([x,y]) => toBase(
    [cell.center[0]+c*x-s*y, cell.center[1]+s*x+c*y], snapshot.status.localization.pose));
}
function stroke(points, color, width = 1.5, fill = null, closed = false, dash = []) {
  if (!points?.length) return;
  ctx.beginPath();
  points.forEach((p,i) => { const xy = position(p); if (i) ctx.lineTo(...xy); else ctx.moveTo(...xy); });
  if (closed) ctx.closePath();
  if (fill) { ctx.fillStyle = fill; ctx.fill(); }
  ctx.strokeStyle = color; ctx.lineWidth = width; ctx.setLineDash(dash); ctx.stroke(); ctx.setLineDash([]);
}
function label(text, point, color = '#b0c7d4', size = 11) {
  const [x,y] = position(point); ctx.font = `${size}px -apple-system, BlinkMacSystemFont, 'Segoe UI', sans-serif`;
  ctx.textAlign = 'center'; ctx.textBaseline = 'middle'; ctx.fillStyle = color;
  ctx.strokeStyle = '#111c24'; ctx.lineWidth = 4; ctx.strokeText(text,x,y); ctx.fillText(text,x,y);
}
function points(values, color, radius) {
  ctx.fillStyle = color;
  for (const point of values || []) { const [x,y] = position(point); ctx.fillRect(x-radius,y-radius,radius*2,radius*2); }
}
function grid() {
  const span = Math.ceil((view.width+view.height)/view.scale)+3;
  const centerX = view.panY/view.scale, centerY = view.panX/view.scale;
  for (let axis = 0; axis < 2; axis++) {
    const mid = axis === 0 ? centerX : centerY;
    for (let i = Math.floor((mid-span)*5); i <= Math.ceil((mid+span)*5); i++) {
      const v = i/5, major = i%5 === 0;
      const pts = axis === 0 ? [[v,centerY-span],[v,centerY+span]] : [[centerX-span,v],[centerX+span,v]];
      stroke(pts, major ? '#263b48' : '#1b2c37', major ? .8 : .5);
    }
  }
  stroke([[0,centerY-span],[0,centerY+span]], '#385563', .8);
  stroke([[centerX-span,0],[centerX+span,0]], '#385563', .8);
}
function markerLayer(markers, fallback, dashed = false) {
  for (const marker of markers || []) {
    if (marker.type === 9) continue;
    const c = marker.color || [], color = c.length ? `rgba(${c.slice(0,3).map(x=>Math.round(x*255)).join(',')},${Math.max(.4,c[3])})` : fallback;
    if (marker.points?.length) {
      if (marker.type === 5) {
        for (let i=0;i<marker.points.length-1;i+=2) stroke(marker.points.slice(i,i+2), color, 1.5);
      } else stroke(marker.points, color, 1.7, null, false, dashed ? [5,4] : []);
    } else if (marker.center) {
      const [x,y] = position(marker.center);
      if (marker.type === 1) {
        const w = (marker.scale?.[1] || .15)*view.scale, h = (marker.scale?.[0] || .15)*view.scale;
        ctx.strokeStyle = color; ctx.lineWidth = 1.6; ctx.strokeRect(x-w/2,y-h/2,w,h);
      } else {
        ctx.beginPath(); ctx.arc(x,y,Math.max(3,(marker.scale?.[0] || .04)*view.scale/2),0,Math.PI*2);
        ctx.fillStyle = color; ctx.fill();
      }
    }
  }
}
function drawVehicle() {
  const body = snapshot?.vehicle || {length:.322,width:.22}, l = body.length/2, w = body.width/2;
  for (const x of [-.10,.10]) for (const y of [-w,w])
    stroke([[x-.035,y],[x+.035,y]], '#a5bcc9', 4);
  stroke([[-l,-w],[l,-w],[l,w],[-l,w]], '#d2edf3', 1.8, '#4c819188', true);
  stroke([[l+.11,0],[l+.025,-.045],[l+.025,.045]], '#c5f2ee', 1.5, '#c5f2ee', true);
  label('83', [0,0], '#e9f9fa', Math.min(12,view.scale*.065+6));
}
function draw() {
  ctx.clearRect(0,0,view.width,view.height); grid();
  const layers = snapshot?.layers || {}, layout = layoutData();
  if (visible.roi) markerLayer(layers.roi, colors.roi, true);
  if (visible.memory) points(layers.memory, colors.memory, 1.2);
  if (visible.scan) points(layers.scan, colors.scan, 1.5);
  markerLayer(layers.obstacles, colors.occupied);
  if (layout && visible.slots) {
    const target = snapshot.status.target_slot_id;
    for (const cell of layout.cells || []) {
      const chosen = cell.id === target;
      const color = chosen ? colors.target : cell.occupied ? colors.occupied : cell.eligible_empty ? colors.empty : colors.unknown;
      const alpha = chosen ? '27' : '12';
      stroke(corners(cell,layout), color, chosen ? 2.6 : 1.8, color+alpha, true, layout.locked ? [] : [6,4]);
      const center = toBase(cell.center, snapshot.status.localization.pose);
      label(`${cell.id+1}${chosen ? ' 목표' : cell.occupied ? ' 점유' : cell.eligible_empty ? ' 빈 칸' : ' 미확정'}`,center,color,11);
    }
  }
  if (visible.path) {
    stroke(layers.path, '#111c24', 6); stroke(layers.path, colors.path, 2.8);
    markerLayer(layers.lookahead, colors.path);
    markerLayer(layers.target, colors.target);
  }
  drawVehicle(); $('scale-line').style.width = `${view.scale*.5}px`;
}
function resize() {
  const rect = canvas.getBoundingClientRect(), dpr = Math.min(2,window.devicePixelRatio || 1);
  const changed = Math.abs(view.width-rect.width) > 80;
  view.width = rect.width; view.height = rect.height;
  canvas.width = Math.round(rect.width*dpr); canvas.height = Math.round(rect.height*dpr);
  ctx.setTransform(dpr,0,0,dpr,0,0);
  if (changed && layoutData()) fit(); else draw();
}
function fit() {
  const layout = layoutData(), all = [[-.4,-.4],[.5,.4]];
  if (layout) for (const cell of layout.cells || []) all.push(...corners(cell, layout));
  if (snapshot?.layers.path) all.push(...snapshot.layers.path);
  if (!layout) all.push([1.5,-2.],[-1.5,1.5]);
  const xs = all.map(p=>p[0]), ys = all.map(p=>p[1]);
  const minX=Math.min(...xs)-.3,maxX=Math.max(...xs)+.3,minY=Math.min(...ys)-.3,maxY=Math.max(...ys)+.3;
  view.scale = Math.max(35,Math.min(400,(view.width-75)/(maxY-minY),(view.height-100)/(maxX-minX)));
  view.panX = (minY+maxY)/2*view.scale; view.panY = (minX+maxX)/2*view.scale-view.height*.15;
  draw();
}
function zoom(factor) { view.scale = Math.max(35,Math.min(500,view.scale*factor)); draw(); }
const stateText = {occupied:'점유',empty_candidate:'빈 칸 후보',unconfirmed:'미확정',outside_candidate_roi:'탐색 구역 밖',outside_initial_right:'관측 구역 밖'};
export function renderSnapshot(data) {
  snapshot = data;
  const state = data.status, active = data.active, layout = layoutData(), sync = state?.synchronization || {};
  $('connection').className = `connection ${data.connected ? 'online' : ''}`;
  $('connection').lastElementChild.textContent = data.connected ? '실시간 연결' : '주차 신호 대기';
  $('update-age').textContent = data.connected ? `상태 수신 ${number(data.age_ms,0)} ms 전` : '최신 주차 상태를 기다립니다';
  $('mission-badge').className = `mission-badge ${active ? 'active' : ''}`;
  $('mission-badge').textContent = active ? '미션 진행 중' : '대기 중';
  $('attempt').textContent = `시도 ${state?.mission_attempt ?? '—'}`;
  $('state-name').textContent = state?.state_text || (data.connected ? '주차 상태 수신 중' : '주차 노드 연결 대기');
  $('reason').textContent = state?.reason || (active ? '최신 센서와 주차 경로를 확인하고 있습니다.' : '주차 미션이 켜지면 매칭과 계획 경로가 표시됩니다.');
  metric('speed', number(state?.speed), ' m/s'); metric('steering',number(state?.steering_angle*180/Math.PI,1),' °');
  $('localization').textContent = state?.localization?.valid ? '정상' : '대기';
  $('localization').className = `small-value ${state?.localization?.valid ? 'good' : 'bad-text'}`;
  $('map-status').textContent = !data.connected ? '주차 상태 연결 대기' : !active ? '주차 미션 시작 대기 · 실시간 센서 표시' : !state.localization?.valid ? '위치 추정 복구 중 · 최신 데이터 대기' : layout ? '실제 주차칸 매칭 결과 표시 중' : '주차 미션 진행 중 · 주차칸 매칭 대기';
  $('map-status').className = `map-status ${active ? 'active' : ''}`;
  $('map-empty').hidden = Boolean(data.layers?.scan?.length || layout || data.layers?.path?.length);
  $('point-count').textContent = `라이다 ${data.layers?.scan?.length ?? 0}점 · 기억 장애물 ${data.layers?.memory?.length ?? 0}점`;
  $('fit-badge').textContent = layout ? layout.locked ? '맵 확정' : '부분 매칭' : '관측 대기';
  $('fit-badge').className = `tag ${layout?.locked ? 'locked' : ''}`;
  metric('matched', layout ? String(layout.matched_cells) : '—', ' / 6');
  metric('rmse', layout ? number(layout.fit_rmse_m == null ? null : layout.fit_rmse_m*100,1) : '—', ' cm');
  metric('hypotheses', layout ? String(layout.hypotheses ?? 1) : '—');
  const target = state?.target_slot_id;
  for (let i=0;i<6;i++) {
    const cell = layout?.cells?.find(c=>c.id === i), el = slotElements[i];
    el.className = 'slot';
    if (cell) el.classList.add(cell.id === target ? 'target' : cell.occupied ? 'occupied' : cell.eligible_empty ? 'empty' : 'unknown');
    el.lastElementChild.textContent = cell ? cell.id === target ? '선택된 목표' : stateText[cell.state] || '미확정' : '관측 대기';
  }
  $('match-note').textContent = layout ? `${layout.columns}열 × ${layout.rows}행 · 칸 ${number(layout.cell_width,2)} × ${number(layout.cell_depth,2)}m. ${layout.locked ? '확정한 지도 위에서 점유와 목표를 추적합니다.' : '현재 관측으로 맞춘 후보이며, 추가 관측으로 확인합니다.'}` : !active ? '주차 미션이 켜지면 관측 장애물로 3열 × 2행 지도를 맞춥니다.' : '장애물을 관측하고 있습니다. 매칭 결과가 나오면 실제 칸 위치와 점유 상태가 표시됩니다.';
  for (const [key,el] of Object.entries(healthItems)) {
    const age = sync[key]; el.lastElementChild.textContent = age == null ? '대기' : `${number(age*1000,0)} ms`;
    el.classList.toggle('bad', age == null || age < -.05 || age > .3);
  }
  $('input-error').textContent = Object.values(data.errors || {}).filter(Boolean).join(' · ');
  $('footer-status').textContent = `83 플랫폼 · ${data.connected ? '최신 주차 상태 수신 중' : '주차 노드 대기'} · 3열 × 2행`;
  if (data.camera) {
    const live = data.camera.status === 'live';
    $('camera-state').textContent = live ? 'LIVE' : '영상 대기';
    $('camera').parentElement.classList.toggle('has-frame', live && $('camera').naturalWidth > 0);
  }
  const signature = `${state?.mission_attempt}:${sync.localization_restart_count}`;
  if (layout && signature !== fitted) { fitted = signature; fit(); } else draw();
}
let drag = null;
canvas.addEventListener('pointerdown', e => { drag = [e.clientX,e.clientY,view.panX,view.panY]; canvas.setPointerCapture(e.pointerId); });
canvas.addEventListener('pointermove', e => { if (drag) { view.panX=drag[2]+e.clientX-drag[0];view.panY=drag[3]+e.clientY-drag[1];draw(); } });
canvas.addEventListener('pointerup', () => { drag=null; });
canvas.addEventListener('pointercancel', () => { drag=null; });
canvas.addEventListener('wheel', e => { e.preventDefault();zoom(e.deltaY < 0 ? 1.12 : 1/1.12); },{passive:false});
$('zoom-in').addEventListener('click',()=>zoom(1.25)); $('zoom-out').addEventListener('click',()=>zoom(.8));
$('fit').addEventListener('click',fit); $('reset-view').addEventListener('click',()=>{view.scale=125;view.panX=0;view.panY=0;draw();});
$('fullscreen').addEventListener('click',()=>{ $('map-stage').requestFullscreen?.().catch(()=>{}); });
document.querySelectorAll('[data-layer]').forEach(el=>el.addEventListener('change',()=>{visible[el.dataset.layer]=el.checked;draw();}));
new ResizeObserver(resize).observe(canvas);
const camera = $('camera');
function cameraConnect() { if (!document.hidden && !camera.getAttribute('src')) camera.src=`/stream/camera.mjpg?t=${Date.now()}`; }
camera.addEventListener('load',()=>camera.parentElement.classList.add('has-frame'));
camera.addEventListener('error',()=>{camera.removeAttribute('src');camera.parentElement.classList.remove('has-frame');setTimeout(cameraConnect,2500);});
document.addEventListener('visibilitychange',()=>{if(document.hidden)camera.removeAttribute('src');else {cameraConnect();refresh();}});
async function refresh() {
  if (polling || document.hidden) return;
  polling = true;
  try {
    const response=await fetch('/api/parking',{signal:AbortSignal.timeout(2500)});
    if(!response.ok)throw new Error('HTTP');
    renderSnapshot(await response.json());
  } catch {
    snapshot=null;draw();$('connection').className='connection offline';
    $('connection').lastElementChild.textContent='연결 끊김';$('update-age').textContent='자동 재연결 중';
    $('map-status').textContent='연결 끊김 · 과거 주차 화면 표시 중단';
    $('map-empty').hidden=false;
  } finally { polling=false; }
}
cameraConnect();refresh();setInterval(refresh,300);
