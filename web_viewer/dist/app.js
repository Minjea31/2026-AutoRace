const cards = [...document.querySelectorAll('[data-source]')];
const connection = document.querySelector('#connection');
const grid = document.querySelector('#video-grid');
let currentView = 'all';
let failed = false;

document.querySelector('#host').textContent = location.host;

function connectImage(card) {
  const image = card.querySelector('img');
  if (card.dataset.source === 'fork') { pollFork(); return; }
  if (!image.getAttribute('src')) image.src = `/stream/${card.dataset.source}.mjpg?at=${Date.now()}`;
}

// The fork snapshot changes a few times per fork: polled as a still image, so
// it does not hold one of the browser's 6 connections like an MJPEG stream
// (a 6th stream starved /api/state and the page showed "disconnected").
function pollFork() {
  const card = cards.find(c => c.dataset.source === 'fork');
  if (!card || card.hidden) return;
  const image = card.querySelector('img');
  const next = new Image();
  next.onload = () => { image.src = next.src; card.querySelector('.stage').classList.add('has-frame'); };
  next.src = `/snapshot/fork.jpg?at=${Date.now()}`;
}
setInterval(pollFork, 2000);

function updateView(view) {
  currentView = view;
  grid.className = `video-grid ${view === 'camera' ? 'camera-only' : view === 'analysis' ? 'analysis-only' : ''}`;
  for (const card of cards) {
    const camera = ['camera', 'yolo'].includes(card.dataset.source);
    card.hidden = view === 'camera' ? !camera : view === 'analysis' && camera;
    if (card.hidden) card.querySelector('img').removeAttribute('src');
    else connectImage(card);
  }
  for (const tab of document.querySelectorAll('.tab')) {
    const active = tab.dataset.view === view;
    tab.classList.toggle('active', active);
    tab.setAttribute('aria-selected', active);
  }
}

document.querySelectorAll('.tab').forEach(tab => tab.addEventListener('click', () => updateView(tab.dataset.view)));

for (const card of cards) {
  const image = card.querySelector('img');
  image.addEventListener('load', () => card.querySelector('.stage').classList.add('has-frame'));
  image.addEventListener('error', () => {
    image.removeAttribute('src');
    setTimeout(() => { if (!card.hidden && !document.hidden) connectImage(card); }, 2500);
  });
  card.querySelector('[data-fullscreen]').addEventListener('click', async () => {
    try { await card.querySelector('.stage').requestFullscreen(); }
    catch { toast('이 브라우저에서는 전체화면을 사용할 수 없습니다.'); }
  });
  card.querySelector('a').addEventListener('click', async event => {
    event.preventDefault();
    try {
      const response = await fetch(`/snapshot/${card.dataset.source}.jpg`);
      if (!response.ok) throw new Error('영상을 수신한 뒤 저장할 수 있습니다.');
      const url = URL.createObjectURL(await response.blob());
      const download = document.createElement('a');
      download.href = url;
      download.download = `${card.dataset.source}-${new Date().toISOString().replaceAll(':', '-')}.jpg`;
      download.click();
      setTimeout(() => URL.revokeObjectURL(url), 1000);
    } catch (error) { toast(error.message || '이미지 저장에 실패했습니다.'); }
  });
}

function toast(message) {
  const element = document.querySelector('#toast');
  element.textContent = message;
  element.classList.add('visible');
  clearTimeout(toast.timer);
  toast.timer = setTimeout(() => element.classList.remove('visible'), 3500);
}

async function refresh() {
  try {
    const response = await fetch('/api/state', {signal: AbortSignal.timeout(3500)});
    if (!response.ok) throw new Error('Disconnected');
    const state = await response.json();
    connection.className = 'connection online';
    connection.lastElementChild.textContent = '플랫폼 연결됨';
    document.querySelector('#session-meta').textContent = `${state.streams.filter(s => s.status === 'live').length}개 영상 수신 중`;
    document.querySelector('#footer-status').textContent = `ROS DOMAIN ${state.domain_id} · 연결 ${Math.floor(state.uptime_s / 60)}분`;
    for (const source of state.streams) {
      const card = cards.find(c => c.dataset.source === source.id);
      if (!card) continue;
      const stage = card.querySelector('.stage');
      const badge = card.querySelector('.feed-state');
      badge.className = `feed-state ${source.status}`;
      badge.lastElementChild.textContent = source.status === 'live' ? 'LIVE' : source.status === 'stale' ? '신호 지연' : '대기 중';
      stage.classList.toggle('stale', source.status === 'stale');
      if (source.id === 'fork') { // latched: an old picture is the normal case
        badge.className = `feed-state ${source.width ? 'live' : 'waiting'}`;
        badge.lastElementChild.textContent = source.width ? '마지막 장면' : '대기 중';
        stage.classList.remove('stale');
        card.querySelector('.dimensions').textContent = source.width ? `${source.width} × ${source.height}` : '—';
        card.querySelector('.fps').textContent = source.age_ms != null ? `${Math.round(source.age_ms / 1000)} s 전` : '—';
        card.querySelector('a').setAttribute('aria-disabled', !source.width);
        continue;
      }
      if (source.status === 'live' && card.querySelector('img').naturalWidth > 0) stage.classList.add('has-frame');
      card.querySelector('.dimensions').textContent = source.width ? `${source.width} × ${source.height}` : '—';
      card.querySelector('.fps').textContent = source.status === 'live' ? `${source.fps.toFixed(1)} FPS` : '— FPS';
      card.querySelector('a').setAttribute('aria-disabled', source.status !== 'live');
      const placeholder = card.querySelector('.placeholder');
      if (source.status === 'stale') {
        placeholder.querySelector('strong').textContent = '영상 신호를 기다리고 있습니다';
        placeholder.lastElementChild.textContent = '플랫폼의 영상 노드가 실행 중인지 확인하세요.';
      }
    }
    if (failed) updateView(currentView);
    failed = false;
  } catch {
    failed = true;
    connection.className = 'connection offline';
    connection.lastElementChild.textContent = '플랫폼 연결 끊김';
    document.querySelector('#session-meta').textContent = '다시 연결하는 중';
    for (const card of cards) {
      card.querySelector('.stage').classList.add('stale');
      const badge = card.querySelector('.feed-state');
      badge.className = 'feed-state stale';
      badge.lastElementChild.textContent = '연결 끊김';
      card.querySelector('a').setAttribute('aria-disabled', 'true');
    }
    document.querySelector('#footer-status').textContent = '네트워크 연결을 확인하세요. 자동으로 다시 연결합니다.';
  }
}

document.addEventListener('visibilitychange', () => {
  if (document.hidden) cards.forEach(card => card.querySelector('img').removeAttribute('src'));
  else { updateView(currentView); refresh(); }
});
updateView('all');
refresh();
setInterval(() => { if (!document.hidden) refresh(); }, 1500);
