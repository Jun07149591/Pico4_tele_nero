const $ = id => document.getElementById(id);
let state = null, episodes = [], selected = null, activeView = 'capture', playing = false, frameIndex = 0;
let busy = false, seenEpisode = null, cameraRoles = [], playLast = 0;
let firstPoll = true;
let fpsEditing = false;
let datasetNameRoot = null, renderedExportRoot = null;
const exportSelection = new Set();
const episodeSelection = new Set();
let deleteCandidates = [];
const roleNames = {front: '顶部相机', left_wrist: '左腕相机', right_wrist: '右腕相机'};
const outcomes = {success: '成功', failure: '失败', discarded: '已丢弃', interrupted: '中断'};
const states = {idle: '待机', recording: '录制中', saving: '保存中', error: '采集异常'};
function icons() { if (window.lucide) lucide.createIcons(); }
function message(text = '') { $('message').textContent = text; $('message').hidden = !text; }
async function api(path, body) {
  const response = await fetch(path, body === undefined ? {} : {method: 'POST', headers: {'Content-Type': 'application/json', 'X-Nero-Request': '1'}, body: JSON.stringify(body)});
  const data = await response.json();
  if (!response.ok) throw new Error(data.error || response.statusText);
  return data;
}
function textNode(tag, text, className) { const node = document.createElement(tag); node.textContent = text; if (className) node.className = className; return node; }
function formatTime(seconds) { return `${String(Math.floor(seconds / 60)).padStart(2, '0')}:${(seconds % 60).toFixed(1).padStart(4, '0')}`; }
function readinessText(reason) {
  if (!reason) return '';
  const messages = {
    'telemetry missing or stale': '未收到最新遥操数据',
    'teleop arm mode differs from capture mode': '遥操与采集的单臂 / 双臂模式不一致',
    'offline review': '离线质检',
    'returning home': '正在回位', 'teleop not ready': '遥操尚未就绪',
    'input lost': 'PICO 追踪无效或中断', 'gripper unavailable': '夹爪未启用',
    'feedback_monotonic missing/stale': '关节反馈缺失或过期',
    'gripper_feedback_monotonic missing/stale': '夹爪反馈缺失或过期',
    'invalid command timestamp': '控制目标时间无效',
    'output FROZEN': '遥操已停止', 'output FAULT': '机械臂输出故障',
    'capture deadline missed': '采集处理超时',
    'feedback does not bracket sample time': '等待关节反馈对齐',
    'feedback interpolation gap too large': '关节反馈间隔过大',
    'writer queue full': '等待磁盘写入',
    'nonempty task instruction is required': '任务指令未填写'
  };
  if (messages[reason]) return messages[reason];
  for (const [key, label] of [['right_arm', '右臂'], ['left_arm', '左臂']]) {
    if (reason.startsWith(`${key}: `)) return `${label}：${messages[reason.slice(key.length + 2)] || reason.slice(key.length + 2)}`;
  }
  for (const [role, label] of Object.entries(roleNames)) {
    if (reason.startsWith(`camera ${role}`)) return `${label}画面缺失或过期`;
  }
  return reason;
}
function showView(view) {
  activeView = view; playing = false; updatePlayIcon();
  document.querySelectorAll('.view').forEach(node => node.hidden = node.id !== `${view}-view`);
  document.querySelectorAll('.tab').forEach(node => node.classList.toggle('selected', node.dataset.view === view));
  message(); if (view !== 'capture') refreshEpisodes();
}
function makeCameras(target, roles, prefix) {
  $(target).replaceChildren();
  roles.forEach(role => {
    const figure = document.createElement('figure'); figure.className = 'camera unavailable';
    const image = document.createElement('img'); image.id = `${prefix}-${role}`; image.alt = roleNames[role];
    image.addEventListener('load', () => figure.classList.remove('unavailable'));
    image.addEventListener('error', () => figure.classList.add('unavailable'));
    const caption = textNode('figcaption', roleNames[role]); caption.append(textNode('span', role));
    figure.append(image, caption); $(target).append(figure);
  });
}
function verdict(item) { const v = item.review.verdict; return textNode('span', v === 'unreviewed' ? '未检查' : v.toUpperCase(), `verdict ${v}`); }
function exportable(item) { return item.outcome === 'success' && item.review.verdict === 'pass'; }
function orderedEpisodes() { return [...episodes].reverse(); }
function chosenEpisodes() { return orderedEpisodes().filter(item => exportSelection.has(item.id)); }
function visibleExportEpisodes() {
  const query = $('export-search').value.trim().toLowerCase(), rate = $('export-rate-filter').value;
  return orderedEpisodes().filter(item => (rate === 'all' || String(item.fps) === rate) && `${item.id} ${item.task}`.toLowerCase().includes(query));
}
function exportLocked() { return busy || !state || ['recording', 'saving'].includes(state.state) || state.export.state === 'running'; }
function visibleEpisodes() { return episodes.filter(e => $('filter').value === 'all' || e.review.verdict === $('filter').value); }
function renderDeleteSelection() {
  const visible = visibleEpisodes(), locked = exportLocked();
  $('delete-count').textContent = `已选 ${episodeSelection.size} 个片段`;
  $('select-episodes').checked = visible.length > 0 && visible.every(item => episodeSelection.has(item.id));
  $('select-episodes').indeterminate = visible.some(item => episodeSelection.has(item.id)) && !$('select-episodes').checked;
  $('select-episodes').disabled = locked || !visible.length;
  $('delete-episodes').disabled = locked || !episodeSelection.size;
  document.querySelectorAll('#episode-list input').forEach(input => { input.disabled = locked; });
}
function renderExportEpisodes() {
  const eligible = new Set(episodes.filter(exportable).map(item => item.id));
  for (const id of exportSelection) if (!eligible.has(id)) exportSelection.delete(id);
  const rateFilter = $('export-rate-filter'), previousRate = rateFilter.value;
  const rates = [...new Set(episodes.map(item => item.fps))].sort((a, b) => a - b);
  rateFilter.replaceChildren(new Option('全部频率', 'all'), ...rates.map(rate => new Option(`${rate} Hz`, String(rate))));
  rateFilter.value = rates.includes(Number(previousRate)) ? previousRate : 'all';
  const chosen = chosenEpisodes(), rows = visibleExportEpisodes(), container = $('export-episode-list');
  container.replaceChildren();
  if (!rows.length) container.append(textNode('div', '暂无匹配片段', 'empty'));
  const order = new Map(chosen.map((item, index) => [item.id, `episode_${String(index).padStart(6, '0')}`]));
  rows.forEach(item => {
    const row = document.createElement('label'); row.className = `export-row${exportSelection.has(item.id) ? ' chosen' : ''}${exportable(item) ? '' : ' unselectable'}`;
    const checkbox = document.createElement('input'); checkbox.type = 'checkbox'; checkbox.dataset.episodeId = item.id;
    checkbox.setAttribute('aria-label', `导出片段 ${item.id}`); checkbox.checked = exportSelection.has(item.id);
    checkbox.disabled = exportLocked() || !exportable(item);
    checkbox.onchange = () => { if (checkbox.checked) exportSelection.add(item.id); else exportSelection.delete(item.id); renderExportEpisodes(); renderStatus(); };
    const task = textNode('span', '', 'export-task'); task.append(textNode('strong', item.task), textNode('small', item.id));
    const review = verdict(item); if (item.outcome !== 'success') review.textContent = outcomes[item.outcome];
    row.append(checkbox, textNode('span', order.get(item.id) || '--', 'export-name'), task,
      textNode('span', `${item.frames} 帧`, 'export-frames'), textNode('span', `${item.fps} Hz`, 'export-rate'), review);
    container.append(row);
  });
  renderExportSelection();
}
function renderExportSelection() {
  const chosen = chosenEpisodes(), visible = visibleExportEpisodes().filter(exportable), locked = exportLocked();
  const mixedRates = new Set(chosen.map(item => item.fps)).size > 1;
  $('selection-count').textContent = `已选 ${chosen.length} 个片段 · ${chosen.reduce((sum, item) => sum + item.frames, 0)} 帧`;
  $('export-fps').textContent = mixedRates ? '混合' : chosen.length ? `${chosen[0].fps} Hz` : '--';
  $('selection-error').textContent = mixedRates ? '所选片段采集频率不一致' : '';
  $('selection-error').hidden = !mixedRates;
  $('select-all').checked = visible.length > 0 && visible.every(item => exportSelection.has(item.id));
  $('select-all').indeterminate = visible.some(item => exportSelection.has(item.id)) && !$('select-all').checked;
  $('select-all').disabled = locked || !visible.length;
  $('clear-selection').disabled = locked || !chosen.length;
  $('export-button').disabled = locked || !chosen.length || mixedRates || !$('repo-id').value.trim() || (state?.synthetic && !$('allow-synthetic').checked);
  $('repo-id').disabled = locked; $('allow-synthetic').disabled = locked;
  document.querySelectorAll('#export-episode-list input').forEach(checkbox => {
    checkbox.disabled = locked || !episodes.some(item => item.id === checkbox.dataset.episodeId && exportable(item));
  });
}
function renderEpisodes() {
  for (const id of episodeSelection) if (!episodes.some(item => item.id === id)) episodeSelection.delete(id);
  if (selected && !episodes.some(item => item.id === selected.id)) {
    selected = null; playing = false; updatePlayIcon();
    $('replay').hidden = true; $('empty-replay').hidden = false; $('replay-cameras').replaceChildren();
  }
  $('episode-count').textContent = episodes.length;
  $('accepted').textContent = episodes.filter(e => e.outcome === 'success' && e.review.verdict === 'pass').length;
  $('pending').textContent = episodes.filter(e => e.review.verdict === 'unreviewed').length;
  $('recent-list').replaceChildren();
  if (!episodes.length) $('recent-list').append(textNode('div', '暂无片段', 'empty'));
  episodes.slice(0, 5).forEach(item => {
    const row = document.createElement('button'); row.className = 'episode-row';
    row.append(textNode('span', item.task), textNode('span', outcomes[item.outcome]), textNode('span', `${item.seconds.toFixed(1)} s`), verdict(item));
    row.onclick = () => { showView('episodes'); openEpisode(item.id); }; $('recent-list').append(row);
  });
  $('episode-list').replaceChildren();
  visibleEpisodes().forEach(item => {
    const choice = document.createElement('div'); choice.className = 'episode-choice';
    const checkbox = document.createElement('input'); checkbox.type = 'checkbox';
    checkbox.setAttribute('aria-label', `选择片段 ${item.id}`); checkbox.checked = episodeSelection.has(item.id);
    checkbox.onchange = () => { if (checkbox.checked) episodeSelection.add(item.id); else episodeSelection.delete(item.id); renderDeleteSelection(); };
    const row = document.createElement('button'); row.className = `episode-item${selected?.id === item.id ? ' current' : ''}`;
    row.append(textNode('strong', item.task), textNode('small', `${item.id} · ${outcomes[item.outcome]} · ${item.frames} 帧 · ${item.seconds.toFixed(1)} s`), verdict(item));
    row.onclick = () => openEpisode(item.id); choice.append(checkbox, row); $('episode-list').append(choice);
  });
  renderDeleteSelection();
  renderExportEpisodes();
  renderStatus();
}
async function refreshEpisodes() { try { episodes = await api('/api/episodes'); renderEpisodes(); } catch (error) { message(error.message); } }
async function openEpisode(id) {
  try {
    playing = false; updatePlayIcon(); selected = await api(`/api/episode?id=${encodeURIComponent(id)}`); frameIndex = 0;
    $('replay').hidden = false; $('empty-replay').hidden = true;
    $('replay-task').textContent = selected.task; $('replay-outcome').textContent = outcomes[selected.outcome];
    $('episode-id').textContent = selected.id; $('notes').value = selected.review.notes;
    $('quality').textContent = selected.quality.ok ? '数据检查通过' : `数据检查：${selected.quality.issues.join(', ')}`;
    $('pass').disabled = !selected.quality.ok;
    $('timeline').max = Math.max(0, selected.frames - 1); $('timeline').value = 0;
    makeCameras('replay-cameras', selected.cameras, 'replay'); renderEpisodes(); drawFrame();
  } catch (error) { message(error.message); }
}
function drawFrame() {
  if (!selected) return;
  $('timeline').value = frameIndex; $('play-time').textContent = `${formatTime(frameIndex / selected.fps)} / ${formatTime(selected.seconds)}`;
  selected.cameras.forEach(role => { $(`replay-${role}`).src = `/api/frame?id=${encodeURIComponent(selected.id)}&role=${role}&index=${frameIndex}`; });
  const table = document.createElement('table'); const head = document.createElement('tr');
  ['维度', '反馈', '目标'].forEach(t => head.append(textNode('th', t))); table.append(head);
  if (selected.frames) selected.vector_names.forEach((name, i) => {
    const row = document.createElement('tr'); row.append(textNode('td', name), textNode('td', selected.state[frameIndex][i].toFixed(5)), textNode('td', selected.action[frameIndex][i].toFixed(5))); table.append(row);
  });
  $('joint-values').replaceChildren(table);
}
function updatePlayIcon() { $('play').innerHTML = `<i data-lucide="${playing ? 'pause' : 'play'}"></i>`; icons(); }
async function command(commandName, values = {}) {
  if (busy) return; busy = true; message(); renderStatus();
  try { const result = await api('/api/command', {command: commandName, ...values}); await refreshEpisodes(); return result; }
  catch (error) { message(error.message); }
  finally { const refreshed = await poll(); busy = false; if (refreshed) renderStatus(); }
}
function renderStatus() {
  if (!state) return;
  if (datasetNameRoot !== state.root) {
    datasetNameRoot = state.root;
    let savedName = null;
    try { savedName = localStorage.getItem(`nero.datasetName:${state.root}`); } catch (_) {}
    $('repo-id').value = savedName ?? state.export.result?.repo_id ?? 'local/nero_pick_place';
  }
  const recording = state.state === 'recording', exporting = state.export.state === 'running';
  $('connection').textContent = state.review_only ? '离线质检' : (state.ready ? '已就绪' : '未就绪'); $('connection').className = `status ${state.review_only ? '' : (state.ready ? 'good' : 'bad')}`;
  $('mode').textContent = state.mode === 'dual' ? '双臂 · 左 / 右' : '单臂 · 右臂';
  $('demo').hidden = !state.synthetic; $('synthetic-label').hidden = !state.synthetic;
  $('rate').textContent = `${state.fps} Hz`;
  if (!fpsEditing) $('capture-fps').value = state.fps;
  $('capture-fps').max = state.max_capture_fps || 30;
  const pendingFps = Number($('capture-fps').value), rateChanged = pendingFps !== state.fps;
  const rateLocked = busy || recording || state.state === 'saving' || exporting || state.review_only;
  $('capture-fps').disabled = rateLocked;
  $('apply-fps').disabled = rateLocked || !rateChanged || !$('capture-fps').checkValidity() || !Number.isInteger(pendingFps);
  $('dimensions').textContent = state.mode === 'dual' ? '16' : '8';
  $('robot-age').textContent = `反馈 ${state.robot_age_ms === null ? '--' : Math.max(0, state.robot_age_ms).toFixed(0)} ms`;
  $('elapsed').textContent = formatTime(state.elapsed_s);
  $('frame-count').textContent = `${state.frames} 帧${state.skipped_frames ? ` · 缺失 ${state.skipped_frames} 帧` : ''}`;
  const taskMissing = !$('task').value.trim();
  const blockedReason = state.capture_wait_reason || state.readiness_reason || state.error;
  $('record-state').textContent = recording && state.capture_wait_reason ? '录制中 · 等待数据' : states[state.state] || state.state;
  $('readiness').textContent = readinessText(blockedReason) || (!recording && rateChanged ? '采集频率尚未应用' : !recording && taskMissing ? '任务指令未填写' : '');
  $('readiness').title = blockedReason || '';
  $('footer-state').textContent = state.review_only ? '离线质检' : (state.synthetic ? '模拟数据源' : '本地采集'); $('root').textContent = state.root;
  $('start').disabled = busy || !state.ready || recording || state.state === 'saving' || exporting || taskMissing || rateChanged;
  ['success', 'failure', 'discard'].forEach(id => $(id).disabled = busy || !recording);
  $('task').disabled = busy || recording;
  renderDeleteSelection();
  renderExportSelection();
  if (state.export.state === 'running') {
    const progress = state.export.progress;
    $('export-status').textContent = progress ? `正在导出 ${progress.completed} / ${progress.total}${progress.episode ? ` · ${progress.episode}` : ''}` : '正在导出…';
  }
  if (state.export.state === 'error') $('export-status').textContent = `导出失败：${state.export.error}`;
  if (state.export.state === 'complete') {
    const result = state.export.result;
    $('export-status').textContent = `已导出：${result.repo_id}\n${result.episodes} 个片段 · ${result.frames} 帧 · ${result.fps} Hz\n${result.parquet_files ?? result.episodes} 个 Parquet${result.video_files !== undefined ? ` · ${result.video_files} 个 MP4` : ''}\n数据集目录：${result.root}\nOpenPI 配置：${result.openpi_spec}`;
    if (renderedExportRoot !== result.root) {
      renderedExportRoot = result.root;
      const table = document.createElement('table'), head = document.createElement('thead'), heading = document.createElement('tr');
      ['原始片段', '导出文件'].forEach(label => heading.append(textNode('th', label))); head.append(heading); table.append(head);
      const body = document.createElement('tbody');
      for (const file of result.files || []) {
        const row = document.createElement('tr'), source = document.createElement('td'), paths = document.createElement('td');
        source.append(textNode('strong', file.source), textNode('div', `${file.frames} 帧`));
        paths.append(textNode('div', file.parquet, 'path'));
        for (const [role, path] of Object.entries(file.videos)) {
          paths.append(textNode('div', roleNames[role] || role, 'export-camera-role'), textNode('div', path, 'path'));
        }
        row.append(source, paths); body.append(row);
      }
      table.append(body); $('export-files').replaceChildren(table);
    }
  } else {
    renderedExportRoot = null; $('export-files').replaceChildren();
  }
}
async function poll() {
  try {
    state = await api('/api/status'); renderStatus();
    if (firstPoll) { firstPoll = false; if (state.review_only) showView('episodes'); }
    if (JSON.stringify(cameraRoles) !== JSON.stringify(state.cameras)) { cameraRoles = state.cameras; makeCameras('live-cameras', cameraRoles, 'live'); }
    for (const role of cameraRoles) {
      const camera = state.camera_status?.[role], image = $(`live-${role}`);
      if (camera && image) image.parentElement.querySelector('figcaption span').textContent = camera.ready ? `${Math.round(camera.age_ms)} ms` : '无有效画面';
    }
    if (seenEpisode !== state.last_episode) { seenEpisode = state.last_episode; await refreshEpisodes(); }
    return true;
  } catch (error) { $('connection').textContent = '服务离线'; $('connection').className = 'status bad'; $('start').disabled = true; return false; }
}
document.querySelectorAll('[data-view]').forEach(button => button.onclick = () => showView(button.dataset.view));
$('all-episodes').onclick = () => showView('episodes'); $('filter').onchange = renderEpisodes; $('task').oninput = renderStatus;
$('select-episodes').onchange = () => {
  const checked = $('select-episodes').checked;
  visibleEpisodes().forEach(item => checked ? episodeSelection.add(item.id) : episodeSelection.delete(item.id));
  renderEpisodes();
};
$('delete-episodes').onclick = () => {
  if (exportLocked() || !episodeSelection.size) return;
  deleteCandidates = orderedEpisodes().filter(item => episodeSelection.has(item.id)).map(item => item.id);
  $('delete-title').textContent = `删除 ${deleteCandidates.length} 个片段`;
  $('delete-list').replaceChildren(...deleteCandidates.map(id => textNode('li', `${id} · ${episodes.find(item => item.id === id).task}`)));
  $('delete-dialog').showModal();
};
$('confirm-delete').onclick = async () => {
  const episode_ids = [...deleteCandidates];
  $('delete-dialog').close(); playing = false; updatePlayIcon();
  await command('delete', {episode_ids});
  await refreshEpisodes();
};
$('capture-fps').oninput = () => { fpsEditing = true; renderStatus(); };
$('apply-fps').onclick = async () => { const result = await command('configure', {fps: Number($('capture-fps').value)}); if (result) { fpsEditing = false; renderStatus(); } };
$('capture-fps').onkeydown = event => { if (event.key === 'Enter' && !$('apply-fps').disabled) $('apply-fps').click(); };
$('export-search').oninput = renderExportEpisodes; $('export-rate-filter').onchange = renderExportEpisodes;
$('select-all').onchange = () => { const checked = $('select-all').checked; visibleExportEpisodes().filter(exportable).forEach(item => checked ? exportSelection.add(item.id) : exportSelection.delete(item.id)); renderExportEpisodes(); };
$('clear-selection').onclick = () => { exportSelection.clear(); renderExportEpisodes(); };
$('repo-id').oninput = () => {
  if (state) { try { localStorage.setItem(`nero.datasetName:${state.root}`, $('repo-id').value); } catch (_) {} }
  renderExportSelection();
};
$('allow-synthetic').onchange = renderExportSelection;
$('start').onclick = () => command('start', {task: $('task').value});
['success', 'failure', 'discard'].forEach(id => $(id).onclick = () => command('finish', {outcome: id === 'discard' ? 'discarded' : id}));
document.addEventListener('keydown', event => {
  if (activeView !== 'capture' || event.isComposing || event.ctrlKey || event.metaKey || event.altKey
      || $('delete-dialog').open || event.defaultPrevented) return;
  const field = event.target.closest('input, textarea, select');
  if ((field && !field.disabled) || event.target.isContentEditable) return;
  const space = event.code === 'Space' || event.key === ' ';
  const failure = event.code === 'KeyL' || event.key.toLowerCase() === 'l';
  if (!space && !failure) return;
  event.preventDefault();
  if (event.repeat || busy || !state) return;
  const button = $(space ? (state.state === 'recording' ? 'success' : 'start') : 'failure');
  if (!button.disabled) button.click();
});
['pass', 'fail', 'unreview'].forEach(id => $(id).onclick = async () => {
  if (!selected) return; const result = await command('review', {id: selected.id, verdict: id === 'unreview' ? 'unreviewed' : id, notes: $('notes').value});
  if (result) { selected.review = result.review; renderEpisodes(); }
});
$('save-notes').onclick = () => selected && command('review', {id: selected.id, verdict: selected.review.verdict, notes: $('notes').value});
$('play').onclick = () => { if (!selected?.frames) return; playing = !playing; if (frameIndex >= selected.frames - 1) frameIndex = 0; playLast = performance.now(); updatePlayIcon(); };
$('timeline').oninput = () => { playing = false; updatePlayIcon(); frameIndex = Number($('timeline').value); drawFrame(); };
$('export-button').onclick = async () => {
  if (busy) return; busy = true; renderStatus();
  try { message(); await api('/api/export', {repo_id: $('repo-id').value, allow_synthetic: $('allow-synthetic').checked, episode_ids: chosenEpisodes().map(item => item.id)}); }
  catch (error) { message(error.message); }
  finally { busy = false; await poll(); }
};
setInterval(async () => {
  if (activeView !== 'capture' || document.hidden) return;
  for (const role of cameraRoles) {
    const image = $(`live-${role}`); if (image && image.complete) image.src = `/api/preview?role=${role}&t=${Date.now()}`;
  }
}, 200);
setInterval(() => {
  if (!playing || !selected || performance.now() - playLast < 1000 / selected.fps) return;
  const advance = Math.max(1, Math.floor((performance.now() - playLast) * selected.fps / 1000));
  playLast = performance.now(); frameIndex = Math.min(frameIndex + advance, selected.frames - 1); drawFrame();
  if (frameIndex >= selected.frames - 1) { playing = false; updatePlayIcon(); }
}, 15);
setInterval(poll, 600); icons(); refreshEpisodes(); poll();
