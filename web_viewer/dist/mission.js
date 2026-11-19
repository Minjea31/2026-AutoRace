'use strict';

const canvas = document.querySelector('#path');
const ctx = canvas.getContext('2d');
const connection = document.querySelector('#connection');
const statusPanel = document.querySelector('#activation');
const statuses = [
  ['owner', 'MUX 제어'],
  ['slow', '서행 활성'],
  ['objavoid', '장애물 회피 활성'],
  ['parking', '주차 활성'],
];
const rows = new Map();
for (const [key, label] of statuses) {
  const title = document.createElement('dt');
  title.textContent = label;
  const value = document.createElement('dd');
  value.textContent = '수신 대기';
  statusPanel.append(title, value);
  rows.set(key, value);
}

let map = {markers: [], scan: []};
let scale = 0;

function xy([x, y]) {
  return [canvas.clientWidth / 2 - y * scale, canvas.clientHeight / 2 - x * scale];
}

function line(points, color, width, paired = false, closed = false) {
  if (points.length < 2) return;
  ctx.beginPath();
  points.forEach((point, index) => {
    const [x, y] = xy(point);
    if (index === 0 || (paired && index % 2 === 0)) ctx.moveTo(x, y);
    else ctx.lineTo(x, y);
  });
  if (closed) ctx.closePath();
  ctx.strokeStyle = color;
  ctx.lineWidth = width;
  ctx.stroke();
}

function dots(points, color, radius) {
  ctx.beginPath();
  for (const point of points) {
    const [x, y] = xy(point);
    ctx.moveTo(x + radius, y);
    ctx.arc(x, y, radius, 0, 2 * Math.PI);
  }
  ctx.fillStyle = color;
  ctx.fill();
}

function draw() {
  const width = canvas.clientWidth;
  const height = canvas.clientHeight;
  if (!width || !height || !scale) return;
  ctx.clearRect(0, 0, width, height);
  const step = scale > 90 ? 0.5 : 1;
  const range = Math.hypot(width, height) / (2 * scale);
  const grid = [];
  for (let p = Math.ceil(-range / step) * step; p <= range; p += step) {
    grid.push([p, -range], [p, range], [-range, p], [range, p]);
  }
  line(grid, '#293946', 1, true);
  line([[0, -range], [0, range], [-range, 0], [range, 0]], '#5b7183', 1, true);
  dots(map.scan, '#dce8f0', 1.5);

  for (const marker of map.markers) {
    const [r, g, b, a] = marker.color;
    const candidate = marker.ns === 'candidates';
    const color = candidate ? 'rgba(102,217,255,0.75)' :
      marker.ns === 'path' ? '#ff3399' : `rgba(${r * 255},${g * 255},${b * 255},${a})`;
    const size = marker.scale * scale;
    switch (marker.type) {
      case 8: dots(marker.points, color, Math.max(1.5, size / 2)); break;
      case 4: line(marker.points, color, Math.max(1, size)); break;
      case 5: line(marker.points, color, Math.max(candidate ? 2 : 1, size), true); break;
      case 2: dots([marker.position], color, Math.max(3, size / 2)); break;
      case 9: {
        const [x, y] = xy(marker.position);
        ctx.fillStyle = color;
        ctx.font = '12px system-ui';
        ctx.fillText(marker.text, x + 5, y - 5);
        break;
      }
    }
  }
  line([[0.161, 0.11], [0.161, -0.11], [-0.161, -0.11], [-0.161, 0.11]], '#5799ff', 2, false, true);
  ctx.fillStyle = '#aebbc8';
  ctx.font = '12px system-ui';
  ctx.fillText(`전방 ↑ · 1 m = ${Math.round(scale)} px`, 12, height - 12);
}

function fit() {
  scale = Math.max(35, Math.min(canvas.clientWidth / 7, canvas.clientHeight / 4));
  draw();
}

function zoom(factor) {
  scale = Math.max(20, Math.min(700, scale * factor));
  draw();
}

new ResizeObserver(() => {
  const ratio = Math.min(window.devicePixelRatio || 1, 1.5);
  canvas.width = Math.round(canvas.clientWidth * ratio);
  canvas.height = Math.round(canvas.clientHeight * ratio);
  ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
  if (scale) draw();
  else fit();
}).observe(canvas);

document.querySelector('#fit').onclick = fit;
canvas.addEventListener('wheel', (event) => {
  event.preventDefault();
  zoom(event.deltaY > 0 ? 0.85 : 1.18);
}, {passive: false});

const detections = document.querySelector('#yolo-detections');
const yoloEmpty = document.querySelector('#yolo-empty');
const yoloHistory = document.querySelector('#yolo-history');
const yoloHistoryEmpty = document.querySelector('#yolo-history-empty');
const historyTime = new Intl.DateTimeFormat('ko-KR', {
  dateStyle: 'short', timeStyle: 'medium', timeZone: 'Asia/Seoul'
});
let lastDetections = '';
let lastHistory = '';
let stateFailures = 0;

function showStatus({status, yolo = [], yolo_history = []}) {
  stateFailures = 0;
  yoloEmpty.textContent = '최근 감지 없음';
  yoloEmpty.hidden = yolo.length > 0;
  detections.hidden = yolo.length === 0;
  const signature = JSON.stringify(yolo);
  if (signature !== lastDetections) {
    const items = yolo.map(({topic, detail}) => {
      const item = document.createElement('li');
      const name = document.createElement('code');
      name.textContent = topic;
      item.append(name);
      if (detail) item.append(" · " + detail);
      return item;
    });
    detections.replaceChildren(...items);
    lastDetections = signature;
  }
  const historySignature = JSON.stringify(yolo_history);
  if (historySignature !== lastHistory) {
    yoloHistory.replaceChildren(...yolo_history.map(({topic, detail, timestamp}) => {
      const item = document.createElement('li');
      const name = document.createElement('code');
      const time = document.createElement('time');
      const received = new Date(timestamp * 1000);
      name.textContent = topic;
      time.dateTime = received.toISOString();
      time.textContent = historyTime.format(received);
      item.append(name, detail ? ' · ' + detail + ' · ' : ' · ', time);
      return item;
    }));
    yoloHistory.hidden = yolo_history.length === 0;
    yoloHistoryEmpty.hidden = yolo_history.length > 0;
    lastHistory = historySignature;
  }
  connection.textContent = '';
  for (const [key, cell] of rows) {
    const value = status[key];
    cell.textContent = value === true ? 'ON' : value === false ? 'OFF' : value ?? '수신 대기';
    cell.className = value === true ? 'on' : value === false ? 'off' : '';
  }
}

async function poll(path, delay, update, onError) {
  let timeout;
  try {
    if (!document.hidden) {
      const controller = new AbortController();
      timeout = setTimeout(() => controller.abort(), 5000);
      const response = await fetch(path, {signal: controller.signal});
      if (!response.ok) throw new Error(response.statusText);
      update(await response.json());
    }
  } catch {
    onError();
  } finally {
    clearTimeout(timeout);
    setTimeout(() => poll(path, delay, update, onError), delay);
  }
}

poll('/api/state', 350, showStatus, () => {
  stateFailures += 1;
  connection.textContent = stateFailures === 1 ? '응답 지연 · 다시 연결 중' : '서버 응답 없음';
  detections.hidden = true;
  yoloEmpty.hidden = false;
  yoloEmpty.textContent = '수신 대기';
  for (const cell of rows.values()) {
    cell.textContent = '—';
    cell.className = '';
  }
});
poll('/api/map', 250, (value) => {
  map = value;
  draw();
}, () => {
  map = {markers: [], scan: []};
  draw();
});
