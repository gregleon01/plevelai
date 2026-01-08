const statsGrid = document.getElementById('stats-grid');
const eventsList = document.getElementById('events-list');
const serialPill = document.getElementById('serial-pill');
const heartbeat = document.getElementById('heartbeat');

const fmt = new Intl.DateTimeFormat([], {
  hour: '2-digit',
  minute: '2-digit',
  second: '2-digit'
});

function fmtNumber(value, decimals = 2) {
  if (value == null || Number.isNaN(Number(value))) return '—';
  return Number(value).toFixed(decimals);
}

function classByVariant(value, thresholds) {
  if (value == null || Number.isNaN(value)) return '';
  if (value >= thresholds.good) return 'success';
  if (value <= thresholds.bad) return 'danger';
  return '';
}

function renderStats(status) {
  statsGrid.innerHTML = '';

  const fps = Number(status.fps || 0);
  const score = fmtNumber(status.score, 2);
  const pan = status.joints?.pan != null ? `${fmtNumber(status.joints.pan, 1)}°` : '—';
  const tilt = status.joints?.tilt != null ? `${fmtNumber(status.joints.tilt, 1)}°` : '—';
  const pixel = status.pixel ? `(${fmtNumber(status.pixel.u, 1)}, ${fmtNumber(status.pixel.v, 1)})` : '—';
  const ground = status.target ? `(${fmtNumber(status.target[0], 2)}, ${fmtNumber(status.target[1], 2)}, ${fmtNumber(status.target[2], 2)})` : '—';

  const items = [
    { label: 'FPS', value: fps.toFixed(1), variant: classByVariant(fps, { good: 12, bad: 4 }) },
    { label: 'Last Score', value: score, variant: classByVariant(Number(status.score ?? 0), { good: 0.75, bad: 0.4 }) },
    { label: 'Pan', value: pan, variant: '' },
    { label: 'Tilt', value: tilt, variant: '' },
    { label: 'Pixel', value: pixel, variant: '' },
    { label: 'Ground XYZ', value: ground, variant: '' },
  ];

  for (const item of items) {
    const card = document.createElement('div');
    card.className = 'stat-card';

    const label = document.createElement('div');
    label.className = 'stat-label';
    label.textContent = item.label;

    const value = document.createElement('div');
    value.className = 'stat-value';
    if (item.variant) value.classList.add(item.variant);
    value.textContent = item.value;

    card.appendChild(label);
    card.appendChild(value);
    statsGrid.appendChild(card);
  }
}

function renderEvents(events) {
  eventsList.innerHTML = '';
  if (!Array.isArray(events) || events.length === 0) {
    const empty = document.createElement('div');
    empty.className = 'event-card';
    empty.innerHTML = '<div class="title">No events yet</div><div class="meta">Waiting for detections…</div>';
    eventsList.appendChild(empty);
    return;
  }

  for (const event of events) {
    const card = document.createElement('div');
    card.className = 'event-card';

    const title = document.createElement('div');
    title.className = 'title';
    if (event.message === 'target') {
      const u = fmtNumber(event.pixel?.u, 0);
      const v = fmtNumber(event.pixel?.v, 0);
      title.textContent = `Target @ ${u}, ${v}`;
    } else if (event.message === 'no_target') {
      title.textContent = 'No target';
    } else if (event.message === 'ik_unavailable') {
      title.textContent = 'IK unavailable';
    } else {
      title.textContent = event.message || 'event';
    }

    const meta = document.createElement('div');
    meta.className = 'meta';
    if (event.timestamp) {
      const dt = new Date(event.timestamp * 1000);
      meta.textContent = `${fmt.format(dt)} · conf ${fmtNumber(event.score, 2)}`;
    } else {
      meta.textContent = '—';
    }

    const badge = document.createElement('span');
    badge.className = 'badge';
    if (event.serial_sent) {
      badge.textContent = 'dispatched';
      badge.classList.add('success');
    } else if (event.has_target) {
      badge.textContent = 'holding';
      badge.classList.add('warning');
    } else {
      badge.textContent = 'idle';
      badge.classList.add('danger');
    }

    card.appendChild(title);
    card.appendChild(badge);
    card.appendChild(meta);
    eventsList.appendChild(card);
  }
}

async function refreshLoop() {
  try {
    const statusRes = await fetch('/api/status');
    const status = await statusRes.json();
    renderStats(status);

    const connected = status.serial_connected;
    serialPill.textContent = connected ? `Serial: ${status.serial_port || 'connected'}` : 'Serial: offline';
    serialPill.className = 'status-pill';
    if (connected) {
      serialPill.classList.add('success');
    } else {
      serialPill.classList.add('danger');
    }

    if (status.last_update) {
      const dt = new Date(status.last_update * 1000);
      heartbeat.textContent = fmt.format(dt);
    }
  } catch (err) {
    console.error('status poll failed', err);
  }

  try {
    const eventsRes = await fetch('/api/events?limit=30');
    const events = await eventsRes.json();
    renderEvents(events.events || []);
  } catch (err) {
    console.error('events poll failed', err);
  }
}

refreshLoop();
setInterval(refreshLoop, 1000);
