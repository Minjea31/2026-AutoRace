'use strict';
const $ = id => document.getElementById(id);
const original = $('original'), binary = $('mask');
const paint = original.getContext('2d'), maskPaint = binary.getContext('2d');
const tint = document.createElement('canvas'), tintPaint = tint.getContext('2d');
tint.width = original.width; tint.height = original.height;
const controls = {};
const storageKey = 'autorace-hsv-six-bounds-v1';
const state = {info: null, bag: null, params: {}, target: 0, shown: 0, playing: false,
  anchor: 0, startTime: 0, revision: 0, request: null, lastKey: '', nextAt: 0,
  image: null, mask: null, result: null};

function fail(message) {
  $('error').hidden = !message;
  $('error').textContent = message || '';
}
function time(value) {
  const s = Math.max(0, value || 0);
  return `${String(Math.floor(s / 60)).padStart(2, '0')}:${(s % 60).toFixed(2).padStart(5, '0')}`;
}
function nearest(seconds) {
  const times = state.bag.times;
  let lo = 0, hi = times.length - 1;
  while (lo < hi) { const mid = (lo + hi) >> 1; if (times[mid] < seconds) lo = mid + 1; else hi = mid; }
  return lo > 0 && Math.abs(times[lo - 1] - seconds) < Math.abs(times[lo] - seconds) ? lo - 1 : lo;
}
function syncControls() {
  for (const [key, pair] of Object.entries(controls)) {
    pair.range.value = state.params[key]; pair.number.value = state.params[key];
  }
  $('lower').textContent = `[${state.params.h_min}, ${state.params.s_min}, ${state.params.v_min}]`;
  $('upper').textContent = `[${state.params.h_max}, ${state.params.s_max}, ${state.params.v_max}]`;
}
function invalidate(delay = 0) {
  state.revision++;
  if (state.request) state.request.abort();
  state.request = null;
  state.lastKey = '';
  state.nextAt = performance.now() + delay;
}
function edit(key, value) {
  if (!Number.isFinite(value)) return;
  const [channel, bound] = key.split('_');
  const maximum = channel === 'h' ? 179 : 255;
  state.params[key] = Math.round(Math.max(0, Math.min(maximum, value)));
  const other = `${channel}_${bound === 'min' ? 'max' : 'min'}`;
  if (state.params[`${channel}_min`] > state.params[`${channel}_max`]) state.params[other] = state.params[key];
  syncControls();
  try { localStorage.setItem(storageKey, JSON.stringify(state.params)); } catch (_) {}
  invalidate(45);
}
for (const [channel, label, maximum] of [['h', '색상', 179], ['s', '채도', 255], ['v', '밝기', 255]]) {
  const group = document.createElement('section'); group.className = `channel channel-${channel}`;
  group.innerHTML = `<div class="channel-title"><b>${channel.toUpperCase()}</b> ${label}<span>0–${maximum}</span></div>`;
  for (const bound of ['min', 'max']) {
    const key = `${channel}_${bound}`, title = `${channel.toUpperCase()} ${bound === 'min' ? '하한' : '상한'}`;
    const row = document.createElement('div'); row.className = 'slider-row';
    row.innerHTML = `<div class="slider-label"><label for="${key}">${bound === 'min' ? '하한' : '상한'}</label><input id="${key}_number" aria-label="${title} 숫자" type="number" min="0" max="${maximum}" step="1"></div><input id="${key}" aria-label="${title}" type="range" min="0" max="${maximum}" step="1">`;
    group.append(row);
    controls[key] = {range: row.querySelector('input[type=range]'), number: row.querySelector('input[type=number]')};
    for (const el of Object.values(controls[key])) el.addEventListener('input', e => edit(key, e.target.valueAsNumber));
    controls[key].number.addEventListener('blur', syncControls);
  }
  $('sliders').append(group);
}

function paused() {
  state.playing = false;
  $('play').textContent = '재생';
  $('play').setAttribute('aria-label', 'bag 재생');
}
function play() {
  if (!state.bag) return;
  if (state.playing) { paused(); return; }
  if (state.target >= state.bag.count - 1) state.target = 0;
  state.anchor = state.bag.times[state.target]; state.startTime = performance.now();
  state.playing = true;
  $('play').textContent = '일시정지'; $('play').setAttribute('aria-label', 'bag 일시정지');
}
function seek(index, stop = false) {
  if (!state.bag) return;
  if (stop) paused();
  state.target = Math.max(0, Math.min(state.bag.count - 1, index));
  if (state.playing) { state.anchor = state.bag.times[state.target]; state.startTime = performance.now(); }
  $('timeline').value = state.bag.times[state.target];
  $('position').textContent = time(state.bag.times[state.target]);
  invalidate();
}
$('play').addEventListener('click', play);
$('prev').addEventListener('click', () => seek(state.target - 1, true));
$('next').addEventListener('click', () => seek(state.target + 1, true));
$('back').addEventListener('click', () => { if (state.bag) seek(nearest(state.bag.times[state.target] - 1)); });
$('forward').addEventListener('click', () => { if (state.bag) seek(nearest(state.bag.times[state.target] + 1)); });
$('timeline').addEventListener('input', e => { if (state.bag) seek(nearest(e.target.valueAsNumber)); });
$('speed').addEventListener('change', () => {
  if (state.playing) { state.anchor = state.bag.times[state.target]; state.startTime = performance.now(); }
});
$('reset').addEventListener('click', () => {
  if (!state.info) return;
  state.params = {...state.info.defaults}; syncControls(); invalidate();
  try { localStorage.removeItem(storageKey); } catch (_) {}
});
$('overlay').addEventListener('change', draw);
document.addEventListener('keydown', e => {
  if (['INPUT', 'SELECT', 'BUTTON', 'TEXTAREA'].includes(document.activeElement.tagName)) return;
  if (e.code === 'Space') { e.preventDefault(); play(); }
  if (e.code === 'ArrowLeft') { e.preventDefault(); seek(state.target - 1, true); }
  if (e.code === 'ArrowRight') { e.preventDefault(); seek(state.target + 1, true); }
});

function draw() {
  if (!state.result || !state.image || !state.mask) return;
  const r = state.result;
  if (original.width !== r.width || original.height !== r.height) {
    original.width = binary.width = tint.width = r.width;
    original.height = binary.height = tint.height = r.height;
    original.parentElement.style.aspectRatio = binary.parentElement.style.aspectRatio = `${r.width}/${r.height}`;
  }
  paint.drawImage(state.image, 0, 0); maskPaint.drawImage(state.mask, 0, 0);
  if ($('overlay').checked) {
    // The binary PNG is opaque. Convert white coverage to a transparent green layer.
    tintPaint.clearRect(0, 0, tint.width, tint.height);
    tintPaint.drawImage(state.mask, 0, 0);
    const pixels = tintPaint.getImageData(0, 0, tint.width, tint.height);
    for (let i = 0; i < pixels.data.length; i += 4) {
      const alpha = pixels.data[i] > 0 ? 145 : 0;
      pixels.data[i] = 82; pixels.data[i + 1] = 235; pixels.data[i + 2] = 135; pixels.data[i + 3] = alpha;
    }
    tintPaint.putImageData(pixels, 0, 0); paint.drawImage(tint, 0, 0);
  }
  const [x0, y0, x1, y1] = r.bounds;
  for (const ctx of [paint, maskPaint]) {
    ctx.strokeStyle = '#f2ce69'; ctx.lineWidth = Math.max(1.5, r.width / 320);
    ctx.strokeRect(x0 + 1, y0 + 1, Math.max(0, x1 - x0 - 2), Math.max(0, y1 - y0 - 2));
    ctx.font = `${Math.max(12, r.width / 45)}px system-ui`; ctx.fillStyle = '#f2ce69';
    ctx.fillText('ROI', x0 + 3, y0 - 6);
  }
}
function showResult(r) {
  state.result = r; state.shown = r.index;
  draw();
  $('ratio').textContent = (r.ratio * 100).toFixed(3);
  $('result').classList.toggle('hit', r.detected);
  $('decision').textContent = r.detected ? '8% 이상 · 감지' : '8% 미만';
  $('detection-banner').hidden = !r.detected;
  $('meter-fill').style.width = `${Math.min(100, r.ratio * 100)}%`;
  $('pixel-count').textContent = `흰색 ${r.white_pixels.toLocaleString()} / ${r.roi_pixels.toLocaleString()} px`;
  const [x0, y0, x1, y1] = r.bounds;
  $('geometry').textContent = `ROI x ${x0}–${x1 - 1} · y ${y0}–${y1 - 1} | ${x1-x0} × ${y1-y0}px`;
  $('position').textContent = time(r.time); $('timeline').value = r.time;
  $('frame-label').textContent = `프레임 ${(r.index + 1).toLocaleString()} / ${state.bag.count.toLocaleString()}`;
  $('connection').textContent = 'bag 연결됨 · 전체 마스크 미리보기';
  $('play').disabled = false; fail('');
}
function currentKey() { return state.bag ? `${state.bag.id}:${state.target}:${JSON.stringify(state.params)}` : ''; }
async function requestFrame() {
  if (!state.bag || state.request || !state.info) return;
  const key = currentKey(); if (key === state.lastKey) return;
  const revision = state.revision, ident = state.bag.id;
  const controller = new AbortController(); state.request = controller;
  state.nextAt = performance.now() + 70;
  const timeout = setTimeout(() => controller.abort('timeout'), 15000);
  try {
    const response = await fetch('/api/frame', {method: 'POST', headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({bag: ident, index: state.target, params: {...state.params}}), signal: controller.signal});
    const data = await response.json(); if (!response.ok) throw new Error(data.error || '프레임을 불러오지 못했습니다.');
    const image = new Image(), mask = new Image(); image.src = data.image; mask.src = data.mask;
    await Promise.all([image.decode(), mask.decode()]);
    if (revision !== state.revision || state.bag?.id !== ident) return;
    state.image = image; state.mask = mask; state.lastKey = key; showResult(data);
  } catch (error) {
    if (revision !== state.revision) return;
    paused(); state.nextAt = performance.now() + 1500;
    fail(error.name === 'AbortError' || error === 'timeout' ? '프레임 응답이 지연됩니다. 잠시 후 다시 시도합니다.' : error.message);
    $('connection').textContent = '연결 재시도 중';
  } finally {
    clearTimeout(timeout);
    if (state.request === controller) state.request = null;
  }
}
let loading = 0;
async function loadBag(ident) {
  const version = ++loading; paused(); invalidate(); state.bag = null;
  state.result = state.image = state.mask = null;
  paint.clearRect(0, 0, original.width, original.height);
  maskPaint.clearRect(0, 0, binary.width, binary.height);
  $('ratio').textContent = '—'; $('decision').textContent = '영상 대기';
  $('detection-banner').hidden = true; $('result').classList.remove('hit');
  $('meter-fill').style.width = '0%';
  $('play').disabled = true; $('connection').textContent = 'bag 프레임 불러오는 중…';
  try {
    const response = await fetch(`/api/bag?id=${encodeURIComponent(ident)}`);
    const bag = await response.json(); if (!response.ok) throw new Error(bag.error);
    if (version !== loading) return;
    state.bag = bag; state.target = state.shown = 0;
    $('duration').textContent = time(bag.duration); $('position').textContent = time(0);
    $('timeline').max = bag.duration; $('timeline').value = 0;
    $('frame-count').textContent = `${bag.count.toLocaleString()} 프레임`;
    invalidate(); fail('');
  } catch (error) { if (version === loading) { fail(error.message); $('connection').textContent = 'bag을 불러오지 못했습니다.'; } }
}
async function loadInfo(refresh = false) {
  $('reload').disabled = true;
  try {
    const response = await fetch('/api/info'); const info = await response.json();
    if (!response.ok) throw new Error(info.error || '서버에 연결하지 못했습니다.');
    const previous = state.bag?.id; state.info = info;
    if (!refresh) {
      state.params = {...info.defaults};
      try {
        const saved = JSON.parse(localStorage.getItem(storageKey));
        if (saved && ['h', 's', 'v'].every(c => Number.isInteger(saved[c+'_min']) && Number.isInteger(saved[c+'_max']) && saved[c+'_min'] >= 0 && saved[c+'_min'] <= saved[c+'_max'] && saved[c+'_max'] <= (c === 'h' ? 179 : 255))) state.params = {...saved};
      } catch (_) {}
      syncControls();
    }
    $('bag').replaceChildren();
    for (const bag of info.bags) {
      const option = document.createElement('option'); option.value = bag.id;
      option.textContent = bag.name; $('bag').append(option);
    }
    if (!info.bags.length) throw new Error('카메라 영상이 있는 bag이 없습니다. 서버의 bag 경로를 확인하세요.');
    const ident = info.bags.some(b => b.id === previous) ? previous : info.bags[0].id;
    $('bag').value = ident;
    if (!refresh || ident !== previous) await loadBag(ident);
  } catch (error) { fail(error.message); $('connection').textContent = 'bag 연결 확인'; }
  finally { $('reload').disabled = false; }
}
$('bag').addEventListener('change', e => loadBag(e.target.value));
$('reload').addEventListener('click', () => loadInfo(true));
$('download').addEventListener('click', () => {
  if (!state.info) return;
  const data = {hsv_lower: ['h', 's', 'v'].map(c => state.params[c+'_min']), hsv_upper: ['h', 's', 'v'].map(c => state.params[c+'_max']),
    roi: state.info.roi, bag: state.bag?.name, bag_time_sec: state.result?.time, roi_white_percent: state.result ? state.result.ratio * 100 : null};
  const url = URL.createObjectURL(new Blob([JSON.stringify(data, null, 2)], {type: 'application/json'}));
  const link = document.createElement('a'); link.href = url; link.download = 'white_hsv_settings.json'; link.click();
  setTimeout(() => URL.revokeObjectURL(url), 1000);
});
function tick(now) {
  if (state.playing && state.bag) {
    let seconds = state.anchor + (now - state.startTime) / 1000 * Number($('speed').value);
    if (seconds >= state.bag.duration) {
      if ($('loop').checked) { state.anchor = seconds = 0; state.startTime = now; }
      else { seconds = state.bag.duration; paused(); }
    }
    state.target = nearest(seconds);
  }
  if (!document.hidden && now >= state.nextAt && currentKey() !== state.lastKey) requestFrame();
  requestAnimationFrame(tick);
}
requestAnimationFrame(tick);
loadInfo();
