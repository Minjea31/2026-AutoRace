'use strict';
const $ = id => document.getElementById(id);
const canvas = $('preview'), ctx = canvas.getContext('2d');
const specs = [
  ['white_roi_left', '왼쪽 · left', 0, 1, .001, 'roi-controls', 1],
  ['white_roi_right', '오른쪽 · right', 0, 1, .001, 'roi-controls', 1],
  ['white_roi_top', '위쪽 · top', 0, 1, .001, 'roi-controls', 1],
  ['white_roi_bottom', '아래쪽 · bottom', 0, 1, .001, 'roi-controls', 1],
  ['white_roi_minimum_ratio', '최소 흰색 비율', .1, 100, .1, 'threshold-controls', 100],
  ['white_s_max', 'S 최대값', 0, 255, 1, 'hsv-controls', 1],
  ['white_v_min', 'V 최소값', 0, 255, 1, 'hsv-controls', 1],
];
let config, params = {}, source = 'bag', bag = null, frameIndex = 0, uploadImage = '', uploadName = '';
let revision = 0, busy = false, timer, last = null, previewImage = null, drag = null;
const controls = {};
function toast(text) { $('toast').textContent = text; $('toast').classList.add('visible'); clearTimeout(toast.timer); toast.timer = setTimeout(() => $('toast').classList.remove('visible'), 3000); }
function buttons(enabled) { for (const id of ['capture', 'copy', 'download']) $(id).disabled = !enabled; }
function yaml(result = last) {
  const p = result?.params || params;
  const node = $('mission').value === 'slow' ? 'slow_node' : 'crosswalk_node';
  return `${node}:\n  ros__parameters:\n` + specs.slice(0, 5).map(([key]) => `    ${key}: ${Number(p[key]).toFixed(3)}`).join('\n') +
    `\n# HSV 참고 (lane_lite 설정):\n# white_s_max: ${p.white_s_max}\n# white_v_min: ${p.white_v_min}\n# 미리보기 값 · 실제 노드에는 아직 적용하지 않음`;
}
function sync() {
  for (const [key, , , , , , scale] of specs) {
    const value = Math.round(params[key] * scale * 1000) / 1000;
    controls[key].range.value = value;
    controls[key].number.value = value;
  }
  $('parameters').textContent = yaml(null);
  draw();
}
function edit(key, value, scale) {
  if (!Number.isFinite(value)) { sync(); return; }
  const spec = specs.find(row => row[0] === key);
  params[key] = Math.round(Math.max(spec[2], Math.min(spec[3], value)) / scale * 1000) / 1000;
  if (key === 'white_roi_left') params[key] = Math.min(params[key], params.white_roi_right - .001);
  if (key === 'white_roi_right') params[key] = Math.max(params[key], params.white_roi_left + .001);
  if (key === 'white_roi_top') params[key] = Math.min(params[key], params.white_roi_bottom - .001);
  if (key === 'white_roi_bottom') params[key] = Math.max(params[key], params.white_roi_top + .001);
  sync(); change();
}
for (const [key, label, min, max, step, parent, scale] of specs) {
  const el = document.createElement('div'); el.className = 'control';
  el.innerHTML = `<div class="control-heading"><label for="${key}-range">${label}</label><div class="value-wrap"><input id="${key}-number" aria-label="${label} 숫자" type="number" min="${min}" max="${max}" step="${step}">${scale === 100 ? '<span>%</span>' : ''}</div></div><input id="${key}-range" type="range" min="${min}" max="${max}" step="${step}">`;
  $(parent).append(el); controls[key] = {number: $(`${key}-number`), range: $(`${key}-range`)};
  controls[key].range.addEventListener('input', e => edit(key, e.target.valueAsNumber, scale));
  controls[key].number.addEventListener('change', e => edit(key, e.target.valueAsNumber, scale));
}
function change() {
  revision++; buttons(false); clearTimeout(timer); timer = setTimeout(requestPreview, 70);
}
function updateBagControls() {
  $('bag-controls').hidden = source !== 'bag';
  if (!bag) return;
  $('bag-name').textContent = bag.name;
  $('bag-index').max = bag.count - 1; $('bag-index').value = frameIndex;
  $('bag-position').textContent = `${bag.times[frameIndex].toFixed(2)}s · ${frameIndex + 1} / ${bag.count}`;
  $('prev-frame').disabled = $('prev-second').disabled = frameIndex === 0;
  $('next-frame').disabled = $('next-second').disabled = frameIndex === bag.count - 1;
}
function go(index) { if (!bag) return; frameIndex = Math.max(0, Math.min(bag.count - 1, index)); updateBagControls(); change(); }
function seek(seconds) {
  if (!bag) return;
  const target = bag.times[frameIndex] + seconds;
  let lo = 0, hi = bag.count - 1;
  while (lo < hi) { const mid = Math.floor((lo + hi) / 2); if (bag.times[mid] < target) lo = mid + 1; else hi = mid; }
  if (lo > 0 && Math.abs(bag.times[lo-1] - target) < Math.abs(bag.times[lo] - target)) lo--;
  go(lo);
}
async function loadBag() {
  try {
    const response = await fetch('/api/roi/bag'); const result = await response.json();
    if (!response.ok) throw new Error(result.error);
    bag = result; frameIndex = 0; updateBagControls(); change();
  } catch (error) { $('message').textContent = error.message; $('bag-name').textContent = '카메라 bag 없음'; }
}
async function requestPreview() {
  if (busy || !config || (source === 'bag' && !bag) || (source === 'upload' && !uploadImage) || document.hidden) return;
  busy = true; const version = revision;
  const payload = {source, params: {...params}, bag_name: bag?.name, frame_index: frameIndex};
  if (source === 'upload') payload.image = uploadImage;
  try {
    const response = await fetch('/api/roi/preview', {method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify(payload), signal: AbortSignal.timeout(12000)});
    const result = await response.json(); if (!response.ok) throw new Error(result.error || '미리보기를 불러오지 못했습니다.');
    if (revision !== version) return;
    const image = new Image(); image.src = result.image; await image.decode();
    if (revision !== version) return;
    last = result; previewImage = image;
    if (source === 'upload') last.source_label = uploadName;
    canvas.width = result.width; canvas.height = result.height;
    canvas.parentElement.style.aspectRatio = `${result.width}/${result.height}`;
    draw(); $('empty').hidden = true; $('message').textContent = '';
    $('image-size').textContent = `${result.width} × ${result.height} px`;
    $('source-detail').textContent = result.source_label + (source === 'bag' ? ` · ${result.bag_time_sec.toFixed(2)}s` : source === 'live' ? ` · ${result.age_ms}ms 전 수신` : '');
    $('ratio').innerHTML = `${(result.ratio * 100).toFixed(1)}<small>%</small>`;
    $('decision').textContent = result.detected ? '흰색 조건 충족' : '흰색 기준 미달';
    $('decision').className = `pill ${result.detected ? 'hit' : 'miss'}`;
    $('pixel-count').textContent = `흰색 ${result.white_pixels.toLocaleString()} / ROI ${result.roi_pixels.toLocaleString()} px · 기준 ${(result.params.white_roi_minimum_ratio * 100).toFixed(1)}%`;
    const [x0,y0,x1,y1] = result.bounds;
    $('geometry').textContent = `영역 ${result.roi_width} × ${result.roi_height}px  |  (${x0}, ${y0}) → (${x1}, ${y1})`;
    $('parameters').textContent = yaml(result);
    $('source-tag').textContent = source === 'live' ? 'LIVE' : source === 'bag' ? 'BAG FRAME' : 'UPLOADED';
    $('source-tag').className = `source-tag ${source === 'live' ? 'live' : ''}`;
    $('connection').textContent = `ROS DOMAIN ${config.domain_id} · 미리보기 전용`;
    buttons(true);
  } catch (error) {
    if (revision !== version) return;
    last = null; buttons(false);
    $('message').textContent = error.name === 'TimeoutError' ? '처리 시간이 초과됐습니다. 다시 시도해주세요.' : error.message;
    $('decision').textContent = '영상 대기'; $('decision').className = 'pill';
    $('source-tag').textContent = source === 'live' ? '카메라 대기' : '연결 확인';
    $('ratio').innerHTML = '—<small>%</small>'; $('pixel-count').textContent = '유효한 영상을 기다립니다.';
    $('empty').textContent = source === 'live' ? '실시간 카메라 대기 중 · bag으로도 조절할 수 있습니다.' : '미리보기를 불러오지 못했습니다.';
    $('empty').hidden = false;
  } finally {
    busy = false;
    if (revision !== version) { clearTimeout(timer); timer = setTimeout(requestPreview, 0); }
  }
}
function draw() {
  if (!previewImage) return;
  ctx.drawImage(previewImage, 0, 0, canvas.width, canvas.height);
  // During dragging show the pending rectangle; the backend measures the same ROI after release.
  if (drag) {
    const x = params.white_roi_left * canvas.width, y = params.white_roi_top * canvas.height;
    const w = (params.white_roi_right - params.white_roi_left) * canvas.width;
    const h = (params.white_roi_bottom - params.white_roi_top) * canvas.height;
    ctx.strokeStyle = '#c4f284'; ctx.lineWidth = 2; ctx.setLineDash([5,4]); ctx.strokeRect(x,y,w,h); ctx.setLineDash([]);
  }
}
function point(event) { const rect = canvas.getBoundingClientRect(); return {x: Math.max(0, Math.min(1, (event.clientX-rect.left)/rect.width)), y: Math.max(0, Math.min(1, (event.clientY-rect.top)/rect.height))}; }
canvas.addEventListener('pointerdown', event => {
  if (!previewImage || !last || event.button !== 0) return;
  event.preventDefault(); const p = point(event), rect = canvas.getBoundingClientRect();
  const [l,r,t,b] = [params.white_roi_left,params.white_roi_right,params.white_roi_top,params.white_roi_bottom];
  let corner = '';
  for (const [name,x,y] of [['lt',l,t],['rt',r,t],['lb',l,b],['rb',r,b]]) if (Math.abs(p.x-x)*rect.width<15 && Math.abs(p.y-y)*rect.height<15) corner=name;
  drag = {start:p, original:{...params}, mode:corner || (p.x>=l && p.x<=r && p.y>=t && p.y<=b ? 'move' : 'new')};
  canvas.setPointerCapture(event.pointerId);
});
canvas.addEventListener('pointermove', event => {
  if (!drag) return;
  const p = point(event), o = drag.original, start = drag.start;
  if (drag.mode === 'move') {
    const dx=Math.max(-o.white_roi_left,Math.min(1-o.white_roi_right,p.x-start.x));
    const dy=Math.max(-o.white_roi_top,Math.min(1-o.white_roi_bottom,p.y-start.y));
    params.white_roi_left=o.white_roi_left+dx; params.white_roi_right=o.white_roi_right+dx;
    params.white_roi_top=o.white_roi_top+dy; params.white_roi_bottom=o.white_roi_bottom+dy;
  } else if (drag.mode === 'new') {
    params.white_roi_left=Math.min(start.x,p.x); params.white_roi_right=Math.max(start.x,p.x);
    params.white_roi_top=Math.min(start.y,p.y); params.white_roi_bottom=Math.max(start.y,p.y);
  } else {
    if (drag.mode.includes('l')) params.white_roi_left=Math.min(p.x,params.white_roi_right-.001);
    if (drag.mode.includes('r')) params.white_roi_right=Math.max(p.x,params.white_roi_left+.001);
    if (drag.mode.includes('t')) params.white_roi_top=Math.min(p.y,params.white_roi_bottom-.001);
    if (drag.mode.includes('b')) params.white_roi_bottom=Math.max(p.y,params.white_roi_top+.001);
  }
  for (const [key] of specs.slice(0,4)) params[key]=Math.round(Math.max(0,Math.min(1,params[key]))*1000)/1000;
  if (params.white_roi_right-params.white_roi_left<.001 || params.white_roi_bottom-params.white_roi_top<.001) return;
  sync(); change();
});
function endDrag() {
  if (!drag) return;
  const o=drag.original;
  if (params.white_roi_right<=params.white_roi_left || params.white_roi_bottom<=params.white_roi_top) params=o;
  drag=null; sync(); change();
}
canvas.addEventListener('pointerup',endDrag); canvas.addEventListener('pointercancel',endDrag);
async function chooseSource(next) {
  source=next; last=null; previewImage=null; buttons(false); ctx.clearRect(0,0,canvas.width,canvas.height);
  $('empty').hidden=false; $('empty').textContent='영상을 불러오는 중…';
  document.querySelectorAll('[data-source]').forEach(el=>el.classList.toggle('selected',el.dataset.source===source));
  $('upload-button').classList.toggle('selected',source==='upload');
  updateBagControls();
  if (source==='bag' && !bag) await loadBag();
  change();
}
document.querySelectorAll('[data-source]').forEach(el=>el.addEventListener('click',()=>chooseSource(el.dataset.source)));
$('upload-button').addEventListener('click',()=>$('upload').click());
$('upload').addEventListener('change',async event=>{
  const file=event.target.files[0]; if (!file) return;
  if (!['image/jpeg','image/png'].includes(file.type) || file.size>5_500_000) { toast('5.5MB 이하 JPEG 또는 PNG를 선택하세요.'); return; }
  uploadName=file.name; uploadImage=await new Promise((resolve,reject)=>{ const reader=new FileReader();reader.onload=()=>resolve(reader.result);reader.onerror=reject;reader.readAsDataURL(file); });
  chooseSource('upload');
});
$('bag-index').addEventListener('input',event=>go(event.target.valueAsNumber));
$('prev-frame').addEventListener('click',()=>go(frameIndex-1)); $('next-frame').addEventListener('click',()=>go(frameIndex+1));
$('prev-second').addEventListener('click',()=>seek(-1)); $('next-second').addEventListener('click',()=>seek(1));
$('reload-bag').addEventListener('click',loadBag);
document.addEventListener('keydown',event=>{
  if(source!=='bag' || ['INPUT','TEXTAREA','SELECT','BUTTON','SUMMARY'].includes(document.activeElement.tagName))return;
  if(event.key==='ArrowLeft'){event.preventDefault();go(frameIndex-1);} if(event.key==='ArrowRight'){event.preventDefault();go(frameIndex+1);}
});
$('mission').addEventListener('change',()=>{params={...config.missions[$('mission').value]};sync();change();});
$('reset').addEventListener('click',()=>{params={...config.missions[$('mission').value]};sync();change();toast('설정 파일의 기본값으로 복원했습니다.');});
function download(blob,name) { const url=URL.createObjectURL(blob),a=document.createElement('a');a.href=url;a.download=name;a.click();setTimeout(()=>URL.revokeObjectURL(url),1000); }
function filename(ext) { return `roi-${$('mission').value}-${new Date().toISOString().replaceAll(':','-').slice(0,19)}.${ext}`; }
$('download').addEventListener('click',()=>{if(last)download(new Blob([yaml(last)+'\n'],{type:'text/yaml'}),filename('yaml'));});
$('copy').addEventListener('click',async()=>{
  if(!last)return;const text=yaml(last);
  try { if(navigator.clipboard && window.isSecureContext) await navigator.clipboard.writeText(text); else { const input=document.createElement('textarea');input.value=text;document.body.append(input);input.select();if(!document.execCommand('copy'))throw Error();input.remove(); }toast('파라미터를 복사했습니다.'); }
  catch{$('parameters').focus();toast('아래 파라미터를 선택해 복사해주세요.');}
});
$('capture').addEventListener('click',async()=>{
  if(!last)return;const result={...last,params:{...last.params}},mission=$('mission').value;
  try {
    const image=new Image();image.src=result.image;await image.decode();await document.fonts.ready;
    const output=document.createElement('canvas');output.width=1440;output.height=1120;const c=output.getContext('2d');
    c.fillStyle='#10181d';c.fillRect(0,0,1440,1120);c.fillStyle='#edf2ee';c.font='bold 30px sans-serif';c.fillText(`AutoRace · ${mission==='slow'?'서행 / 방지턱':'횡단보도'} 흰색 ROI`,30,48);
    c.font='17px sans-serif';c.fillStyle='#a1b6bd';c.fillText(result.source_label.slice(0,95)+(result.source==='bag'?` | ${result.bag_time_sec.toFixed(3)}s | ${result.frame_index+1}/${result.frame_count}`:''),30,85);
    const scale=Math.min(960/result.width,720/result.height),w=result.width*scale,h=result.height*scale;
    c.drawImage(image,30+(960-w)/2,120+(720-h)/2,w,h);
    c.fillStyle='#c4f284';c.font='bold 36px monospace';c.fillText(`${(result.ratio*100).toFixed(2)}% WHITE`,1020,164);
    c.font='18px sans-serif';c.fillStyle='#edf2ee';
    const info=[result.detected?'흰색 조건 충족':'흰색 기준 미달',`기준: ${(result.params.white_roi_minimum_ratio*100).toFixed(1)}%`,`원본: ${result.width} × ${result.height}px`,`ROI: ${result.roi_width} × ${result.roi_height}px`,`픽셀: ${result.bounds.join(', ')}`,`흰색: ${result.white_pixels} / ${result.roi_pixels}`,`HSV S ≤ ${result.params.white_s_max}`,`HSV V ≥ ${result.params.white_v_min}`];
    info.forEach((line,i)=>c.fillText(line,1020,210+i*42));
    c.fillStyle='#91a7ad';c.font='15px sans-serif';c.fillText('미리보기 설정 · 미션 노드에는 미적용',1020,620);c.fillText('표지판 확인 조건은 별도입니다.',1020,650);
    c.fillStyle='#24343b';c.fillRect(30,865,1380,1);c.fillStyle='#edf2ee';c.font='bold 19px sans-serif';c.fillText('공유할 파라미터 (원본 영상 기준 비율)',30,903);c.font='18px monospace';
    specs.slice(0,5).forEach(([key],i)=>c.fillText(`${key}: ${Number(result.params[key]).toFixed(3)}`,30+(i%2)*660,945+Math.floor(i/2)*38));
    c.fillStyle='#91a7ad';c.font='14px sans-serif';c.fillText(`${new Date().toLocaleString('ko-KR')} · ${location.host} · 실제 노드 설정 파일은 변경하지 않았습니다.`,30,1090);
    output.toBlob(blob=>{if(blob){download(blob,filename('png'));toast('영상과 파라미터를 한 장으로 저장했습니다.');}},'image/png');
  }catch{toast('캡처 생성에 실패했습니다. 다시 시도해주세요.');}
});
async function init(){
  try{const response=await fetch('/api/roi/config');if(!response.ok)throw Error();config=await response.json();params={...config.missions.crosswalk};sync();await loadBag();requestPreview();}
  catch{$('message').textContent='서버 설정을 읽지 못했습니다. 페이지를 새로고침해주세요.';}
}
setInterval(()=>{if(source==='live'&&!document.hidden)requestPreview();},400);
document.addEventListener('visibilitychange',()=>{if(!document.hidden)requestPreview();});
init();
