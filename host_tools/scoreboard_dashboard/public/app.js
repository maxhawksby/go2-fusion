import { sortRuns } from './run_sort.mjs';


const elements = Object.fromEntries(
  [...document.querySelectorAll('[id]')].map((element) => [element.id, element]),
);

const state = {
  runs: [],
  selectedPath: null,
  selectedUpdatedAt: null,
  run: null,
  refreshing: false,
  chartPoints: [],
  chartShowsCompleteness: false,
  sortOrder: localStorage.getItem('oak-scoreboard-sort') || 'newest',
};


function escapeText(value) {
  return String(value ?? '');
}


function number(value, digits = 1) {
  if (value === null || value === undefined || value === '' || !Number.isFinite(Number(value))) {
    return '—';
  }
  return Number(value).toFixed(digits);
}


function duration(seconds) {
  if (!Number.isFinite(Number(seconds))) return '—';
  const total = Math.max(0, Math.round(Number(seconds)));
  const minutes = Math.floor(total / 60);
  const remainder = total % 60;
  return minutes ? `${minutes}m ${String(remainder).padStart(2, '0')}s` : `${remainder}s`;
}


function titleCase(value) {
  return String(value || 'unknown')
    .toLowerCase()
    .replaceAll('_', ' ')
    .replace(/\b\w/g, (character) => character.toUpperCase());
}


function shortRunName(run) {
  return run.run_label || run.run_id || 'Mapping run';
}


function isInterruptedRecording(run, updatedAt = run.updated_at) {
  if (run.state !== 'RECORDING' || !updatedAt) return false;
  const checkpointTime = new Date(updatedAt).getTime();
  return Number.isFinite(checkpointTime) && Date.now() - checkpointTime > 15_000;
}


function displayState(run, updatedAt = run.updated_at) {
  return isInterruptedRecording(run, updatedAt) ? 'INTERRUPTED' : (run.state || 'UNKNOWN');
}


function timestamp(value) {
  if (!value) return '—';
  return new Intl.DateTimeFormat(undefined, {
    month: 'short', day: 'numeric', hour: 'numeric', minute: '2-digit', second: '2-digit',
  }).format(new Date(value));
}


async function fetchJson(url) {
  const response = await fetch(url, { cache: 'no-store' });
  if (!response.ok) {
    const payload = await response.json().catch(() => ({ error: response.statusText }));
    throw new Error(payload.error || response.statusText);
  }
  return response.json();
}


function setConnection(connected, label) {
  elements['connection-label'].textContent = label;
  elements['connection-label'].parentElement.classList.toggle('connected', connected);
  elements['connection-label'].parentElement.classList.toggle('disconnected', !connected);
}


function renderRunList() {
  elements['run-list'].replaceChildren();
  for (const run of state.runs) {
    const button = document.createElement('button');
    button.type = 'button';
    button.className = `run-item${run.path === state.selectedPath ? ' active' : ''}`;
    const top = document.createElement('span');
    top.className = 'run-item-top';
    const name = document.createElement('strong');
    name.textContent = shortRunName(run);
    const badge = document.createElement('span');
    const shownState = displayState(run);
    badge.className = `mini-state ${shownState.toLowerCase()}`;
    badge.textContent = shownState;
    top.append(name, badge);
    const details = document.createElement('small');
    const coverage = run.completeness_pct === null || run.completeness_pct === undefined
      ? `${number(run.known_area_m2)} m² known`
      : `${number(run.completeness_pct)}% mapped`;
    details.textContent = `${coverage} · ${duration(run.duration_s)}`;
    button.append(top, details);
    button.addEventListener('click', () => selectRun(run.path, true));
    elements['run-list'].append(button);
  }
}


async function refreshRuns(forceSelection = false) {
  if (state.refreshing) return;
  state.refreshing = true;
  try {
    const payload = await fetchJson('/api/runs');
    state.runs = sortRuns(payload.runs, state.sortOrder);
    setConnection(true, state.runs.length ? 'Scoreboard data found' : 'Waiting for a run');
    if (!state.runs.length) {
      state.selectedPath = null;
      elements.dashboard.hidden = true;
      elements['empty-state'].hidden = false;
      renderRunList();
      return;
    }
    const selectionExists = state.runs.some((run) => run.path === state.selectedPath);
    if (!selectionExists || forceSelection) {
      state.selectedPath = state.runs[0].path;
      state.selectedUpdatedAt = null;
    }
    renderRunList();
    const selected = state.runs.find((run) => run.path === state.selectedPath);
    if (selected && selected.updated_at !== state.selectedUpdatedAt) {
      await selectRun(selected.path, false);
    }
  } catch (error) {
    setConnection(false, 'Scoreboard server unavailable');
    console.error(error);
  } finally {
    state.refreshing = false;
  }
}


async function selectRun(runPath, scroll) {
  state.selectedPath = runPath;
  renderRunList();
  try {
    const payload = await fetchJson(`/api/run?path=${encodeURIComponent(runPath)}`);
    state.run = payload;
    const listEntry = state.runs.find((run) => run.path === runPath);
    state.selectedUpdatedAt = listEntry?.updated_at || new Date().toISOString();
    renderDashboard(payload, listEntry);
    if (scroll && window.innerWidth < 850) {
      document.getElementById('main').scrollIntoView({ behavior: 'smooth' });
    }
  } catch (error) {
    setConnection(false, error.message);
  }
}


function renderDashboard(payload, listEntry) {
  const run = payload.summary;
  const shownState = displayState(run, listEntry?.updated_at || state.selectedUpdatedAt);
  const interrupted = shownState === 'INTERRUPTED';
  elements['empty-state'].hidden = true;
  elements.dashboard.hidden = false;
  elements['run-title'].textContent = shortRunName(run);
  elements['mission-subtitle'].textContent = `${titleCase(run.source)} run · ${escapeText(run.run_id)}`;
  elements['state-badge'].textContent = shownState;
  elements['state-badge'].className = `state-badge ${shownState.toLowerCase()}`;
  elements['source-badge'].textContent = run.source === 'sim' ? 'Gazebo simulation' : titleCase(run.source);
  elements['updated-at'].textContent = timestamp(listEntry?.updated_at || state.selectedUpdatedAt);

  const rawCompleteness = run.completeness_pct;
  const completeness = Number(rawCompleteness);
  const hasCompleteness = rawCompleteness !== null
    && rawCompleteness !== undefined
    && rawCompleteness !== ''
    && Number.isFinite(completeness);
  elements['completeness-value'].textContent = hasCompleteness ? `${number(completeness)}%` : 'N/A';
  elements['progress-ring'].style.setProperty('--progress', `${hasCompleteness ? Math.min(100, Math.max(0, completeness)) * 3.6 : 0}deg`);
  elements['progress-ring'].setAttribute('aria-label', hasCompleteness
    ? `${number(completeness)} percent mapped`
    : 'Completeness unavailable without an answer key');
  elements['coverage-heading'].textContent = hasCompleteness ? 'Map completeness' : 'Known map area';
  elements['coverage-detail'].textContent = hasCompleteness
    ? 'Compared with the supplied ground-truth floor map.'
    : 'No answer key was supplied, so no percentage is invented.';
  elements['known-area'].textContent = `${number(run.known_area_m2)} m²`;

  elements['duration-value'].textContent = duration(run.duration_s);
  elements['distance-value'].textContent = `${number(run.distance_m, 2)} m`;
  elements['goals-value'].textContent = `${run.goals_reached ?? 0}/${run.goals_sent ?? 0}`;
  elements['goals-detail'].textContent = `${run.goals_failed ?? 0} failed · ${run.goals_cancelled ?? 0} cancelled`;
  elements['recoveries-value'].textContent = run.recoveries ?? 0;
  elements['collisions-value'].textContent = run.collisions ?? 0;
  elements['collisions-value'].parentElement.classList.toggle('alert', Number(run.collisions) > 0);
  elements['touches-value'].textContent = run.human_touches ?? 0;
  elements['cpu-value'].textContent = `${number(run.cpu_peak_pct)}%`;
  elements['memory-value'].textContent = `${number(run.memory_peak_mb, 0)} MB`;
  elements['safety-value'].textContent = run.safety_stops ?? 0;

  const warnings = Array.isArray(run.warnings) ? [...run.warnings] : [];
  if (interrupted) {
    warnings.unshift('No recent checkpoint was received. This is a preserved interrupted run, not a live mission.');
  }
  elements['warning-heading'].textContent = warnings.length
    ? `${warnings.length} warning${warnings.length === 1 ? '' : 's'}`
    : 'No warnings recorded';
  elements['warning-box'].classList.toggle('has-warnings', Boolean(warnings.length));
  elements['warning-list'].replaceChildren(...warnings.map((warning) => {
    const item = document.createElement('li');
    item.textContent = warning;
    return item;
  }));

  elements['end-reason'].textContent = run.end_reason
    || (interrupted ? 'Recording stopped before finalization' : (run.state === 'RECORDING' ? 'Mission in progress' : '—'));
  elements['algorithm-value'].textContent = run.configuration?.parameters?.algorithm || run.run_label || '—';
  elements['code-version'].textContent = run.code_version || 'unknown';
  elements['source-hash'].textContent = run.source_hash || 'unknown';

  renderDownloads(payload.files);
  renderEvents(payload.timeline);
  renderChart(payload.timeline, hasCompleteness);
  renderMap(payload.files.map);
}


function renderDownloads(files) {
  const options = [
    ['Run JSON', 'run.json', true],
    ['Scorecard CSV', 'scorecard.csv', files.scorecard],
    ['Timeline CSV', 'timeline.csv', true],
    ['Report', 'report.md', files.report],
  ];
  elements['download-row'].replaceChildren(...options
    .filter(([, , available]) => available)
    .map(([label, file]) => {
      const link = document.createElement('a');
      link.href = `/api/download?path=${encodeURIComponent(state.selectedPath)}&file=${encodeURIComponent(file)}`;
      link.textContent = label;
      return link;
    }));
}


function renderEvents(timeline) {
  const events = timeline.filter((row) => row.kind === 'event').slice(-8).reverse();
  if (!events.length) {
    const item = document.createElement('li');
    item.innerHTML = '<strong>No mission events yet</strong><span>The recorder is waiting for the automapper.</span>';
    elements['event-list'].replaceChildren(item);
    return;
  }
  elements['event-list'].replaceChildren(...events.map((event) => {
    const item = document.createElement('li');
    const heading = document.createElement('strong');
    heading.textContent = titleCase(event.event_type);
    const detail = document.createElement('span');
    detail.textContent = `${duration(event.time_s)} · ${event.detail || `Goal ${event.goal_sequence || '—'}`}`;
    item.append(heading, detail);
    return item;
  }));
}


function setupCanvas(canvas, height) {
  const ratio = window.devicePixelRatio || 1;
  const width = Math.max(300, canvas.clientWidth);
  canvas.width = Math.floor(width * ratio);
  canvas.height = Math.floor(height * ratio);
  const context = canvas.getContext('2d');
  context.setTransform(ratio, 0, 0, ratio, 0, 0);
  return { context, width, height };
}


function renderChart(timeline, hasCompleteness) {
  elements['chart-tooltip'].hidden = true;
  elements['chart-hover-marker'].hidden = true;
  state.chartPoints = [];
  state.chartShowsCompleteness = hasCompleteness;
  const points = timeline
    .map((row) => ({
      x: Number(row.time_s),
      y: Number(hasCompleteness ? row.completeness_pct : row.known_area_m2),
    }))
    .filter((point) => Number.isFinite(point.x) && Number.isFinite(point.y));
  const { context, width, height } = setupCanvas(elements['timeline-chart'], 230);
  context.clearRect(0, 0, width, height);
  const padding = { left: 42, right: 16, top: 17, bottom: 30 };
  const plotWidth = width - padding.left - padding.right;
  const plotHeight = height - padding.top - padding.bottom;
  context.strokeStyle = '#dfe5ed';
  context.lineWidth = 1;
  for (let index = 0; index <= 4; index += 1) {
    const y = padding.top + (plotHeight * index / 4);
    context.beginPath();
    context.moveTo(padding.left, y);
    context.lineTo(width - padding.right, y);
    context.stroke();
  }
  if (points.length < 2) {
    context.fillStyle = '#748198';
    context.font = '12px system-ui';
    context.fillText('More samples are needed for a trace.', padding.left, padding.top + 24);
    elements['chart-label'].textContent = 'Waiting for samples';
    return;
  }
  const maxX = Math.max(...points.map((point) => point.x), 1);
  const maxY = hasCompleteness ? 100 : Math.max(...points.map((point) => point.y), 1);
  const pointAt = (point) => ({
    x: padding.left + point.x / maxX * plotWidth,
    y: padding.top + plotHeight - point.y / maxY * plotHeight,
  });
  state.chartPoints = points.map((point) => ({
    ...pointAt(point),
    time: point.x,
    value: point.y,
  }));
  const gradient = context.createLinearGradient(0, padding.top, 0, padding.top + plotHeight);
  gradient.addColorStop(0, 'rgba(22,74,138,0.25)');
  gradient.addColorStop(1, 'rgba(22,74,138,0.01)');
  context.beginPath();
  points.forEach((point, index) => {
    const location = pointAt(point);
    if (index === 0) context.moveTo(location.x, location.y);
    else context.lineTo(location.x, location.y);
  });
  const last = pointAt(points.at(-1));
  const first = pointAt(points[0]);
  context.lineTo(last.x, padding.top + plotHeight);
  context.lineTo(first.x, padding.top + plotHeight);
  context.closePath();
  context.fillStyle = gradient;
  context.fill();
  context.beginPath();
  points.forEach((point, index) => {
    const location = pointAt(point);
    if (index === 0) context.moveTo(location.x, location.y);
    else context.lineTo(location.x, location.y);
  });
  context.strokeStyle = '#164a8a';
  context.lineWidth = 3;
  context.stroke();
  context.fillStyle = '#66728a';
  context.font = '11px system-ui';
  context.fillText('0', padding.left - 5, height - 9);
  context.textAlign = 'right';
  context.fillText(duration(maxX), width - padding.right, height - 9);
  context.textAlign = 'left';
  elements['chart-label'].textContent = hasCompleteness
    ? `${number(points.at(-1).y)}% mapped`
    : `${number(points.at(-1).y)} m² known`;
}


function showChartValue(event) {
  if (!state.chartPoints.length) return;
  const canvas = elements['timeline-chart'];
  const bounds = canvas.getBoundingClientRect();
  const pointerX = event.clientX - bounds.left;
  const nearest = state.chartPoints.reduce((best, point) => (
    Math.abs(point.x - pointerX) < Math.abs(best.x - pointerX) ? point : best
  ));
  const marker = elements['chart-hover-marker'];
  marker.style.left = `${nearest.x}px`;
  marker.style.top = `${nearest.y}px`;
  marker.hidden = false;
  const tooltip = elements['chart-tooltip'];
  const value = state.chartShowsCompleteness
    ? `${number(nearest.value)}% mapped`
    : `${number(nearest.value)} m² known`;
  tooltip.textContent = `${duration(nearest.time)} · ${value}`;
  tooltip.style.left = `${Math.max(4, Math.min(nearest.x + 12, bounds.width - 145))}px`;
  tooltip.style.top = `${Math.max(4, nearest.y - 44)}px`;
  tooltip.hidden = false;
}


function hideChartValue() {
  elements['chart-tooltip'].hidden = true;
  elements['chart-hover-marker'].hidden = true;
}


function pgmToken(bytes, cursor) {
  while (cursor.index < bytes.length) {
    const byte = bytes[cursor.index];
    if (byte === 35) {
      while (cursor.index < bytes.length && bytes[cursor.index] !== 10) cursor.index += 1;
    } else if (byte <= 32) {
      cursor.index += 1;
    } else {
      break;
    }
  }
  const start = cursor.index;
  while (cursor.index < bytes.length && bytes[cursor.index] > 32 && bytes[cursor.index] !== 35) {
    cursor.index += 1;
  }
  return new TextDecoder().decode(bytes.slice(start, cursor.index));
}


async function renderMap(hasMap) {
  elements['map-placeholder'].hidden = false;
  elements['map-canvas'].hidden = true;
  if (!hasMap) {
    elements['map-placeholder'].textContent = 'The map appears after the first scoreboard checkpoint.';
    return;
  }
  try {
    const response = await fetch(`/api/map?path=${encodeURIComponent(state.selectedPath)}`, { cache: 'no-store' });
    if (!response.ok) throw new Error('Map is not available yet.');
    const bytes = new Uint8Array(await response.arrayBuffer());
    const cursor = { index: 0 };
    const magic = pgmToken(bytes, cursor);
    const width = Number(pgmToken(bytes, cursor));
    const height = Number(pgmToken(bytes, cursor));
    const maximum = Number(pgmToken(bytes, cursor));
    while (cursor.index < bytes.length && bytes[cursor.index] <= 32) cursor.index += 1;
    if (magic !== 'P5' || !width || !height || maximum > 255) {
      throw new Error('Unsupported map image format.');
    }
    const source = document.createElement('canvas');
    source.width = width;
    source.height = height;
    const sourceContext = source.getContext('2d');
    const image = sourceContext.createImageData(width, height);
    for (let index = 0; index < width * height; index += 1) {
      const value = bytes[cursor.index + index] ?? 205;
      const output = index * 4;
      image.data[output] = value;
      image.data[output + 1] = value;
      image.data[output + 2] = value;
      image.data[output + 3] = 255;
    }
    sourceContext.putImageData(image, 0, 0);
    const canvas = elements['map-canvas'];
    const { context, width: canvasWidth, height: canvasHeight } = setupCanvas(canvas, 270);
    const scale = Math.min(canvasWidth / width, canvasHeight / height) * 0.92;
    const drawWidth = width * scale;
    const drawHeight = height * scale;
    context.fillStyle = '#f5f7fa';
    context.fillRect(0, 0, canvasWidth, canvasHeight);
    context.imageSmoothingEnabled = false;
    context.drawImage(source, (canvasWidth - drawWidth) / 2, (canvasHeight - drawHeight) / 2, drawWidth, drawHeight);
    canvas.hidden = false;
    elements['map-placeholder'].hidden = true;
  } catch (error) {
    elements['map-placeholder'].textContent = error.message;
  }
}


elements['refresh-button'].addEventListener('click', () => refreshRuns(true));
elements['run-sort'].value = state.sortOrder;
elements['run-sort'].addEventListener('change', (event) => {
  state.sortOrder = event.target.value;
  localStorage.setItem('oak-scoreboard-sort', state.sortOrder);
  state.runs = sortRuns(state.runs, state.sortOrder);
  renderRunList();
});
elements['timeline-chart'].addEventListener('pointermove', showChartValue);
elements['timeline-chart'].addEventListener('pointerleave', hideChartValue);
window.addEventListener('resize', () => {
  if (state.run) {
    const rawCompleteness = state.run.summary.completeness_pct;
    const hasCompleteness = rawCompleteness !== null
      && rawCompleteness !== undefined
      && rawCompleteness !== ''
      && Number.isFinite(Number(rawCompleteness));
    renderChart(state.run.timeline, hasCompleteness);
    renderMap(state.run.files.map);
  }
});

await refreshRuns();
setInterval(() => refreshRuns(), 1000);
