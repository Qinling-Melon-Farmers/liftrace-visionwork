'use strict';
const logState = {connection: {}, data: null, busy: false, checkedAt: 0, supported: false, pendingUlog: null};
const logById = id => document.getElementById(id);
function logNode(tag, text) { const n = document.createElement(tag); if (text != null) n.textContent = String(text); return n; }
function logSize(n) { return (n / 1048576).toFixed(2) + ' MiB · ' + n + ' bytes'; }
async function logAPI(path, body) {
  const response = await fetch(path, body === undefined ? {cache: 'no-store'} : {
    method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify(body)});
  const data = await response.json();
  if (!response.ok || !data.ok) throw new Error(data.error || '日志请求失败');
  return data;
}
function logButtons() {
  const offline = !logState.supported || logState.connection.state !== 'ok', active = logState.data && logState.data.jobs.some(j => j.active);
  const conflict = logState.data && logState.data.recorders.some(p => p.kind === 'bag');
  const transfer = logState.data && logState.data.recorders.some(p => p.kind === 'ulog');
  logById('bag-start').disabled = offline || logState.busy || !logState.data || active || conflict ||
    !logState.data.available[logById('bag-profile').value === 'routine' ? 'routine' : 'bag'];
  logById('ulog-fetch').disabled = offline || logState.busy || !logState.data || active || transfer || !logState.data.available.ulog ||
    !logById('ulog-id').value;
  logById('ulog-index').disabled = offline || logState.busy || active || transfer;
  logById('log-refresh').disabled = logState.busy;
  logById('log-connect').disabled = logState.busy;
  document.querySelectorAll('[data-stop]').forEach(b => { b.disabled = offline || logState.busy; });
}
function logFiles() {
  const body = logById('log-files'); body.replaceChildren();
  const filter = logById('log-filter').value.toLowerCase();
  const files = logState.data ? logState.data.files.filter(f => f.path.toLowerCase().includes(filter)) : [];
  for (const f of files) {
    const row = logNode('tr');
    row.append(logNode('td', f.path), logNode('td', logSize(f.size)), logNode('td', new Date(f.mtime * 1000).toLocaleString()));
    const cell = logNode('td');
    if (f.downloadable && logState.connection.state === 'ok') {
      const button = logNode('button', '下载到本机'); button.className = 'btn';
      button.onclick = () => logDownload(f); cell.append(button);
      if (f.path.toLowerCase().endsWith('.ulg') && typeof ulogBoardView === 'function') {
        const view = logNode('button', '查看 ULog'); view.className = 'btn';
        view.onclick = () => ulogBoardView(f); cell.append(view);
      }
    } else cell.textContent = f.downloadable ? '请先连接' : '活动或未封闭';
    row.append(cell); body.append(row);
  }
  if (!files.length) { const row = logNode('tr'), cell = logNode('td', '没有匹配文件或尚未刷新。'); cell.colSpan = 4; row.append(cell); body.append(row); }
}
function logRender() {
  const data = logState.data;
  logById('log-connection').textContent = (logState.connection.host || '未配置') + ' · ' +
    (logState.connection.board_root || '') + ' · ' + (logState.connection.state === 'ok' ? '已连接' : '请先在主工作台连接');
  logById('log-at').textContent = logState.checkedAt ? '最后查询 ' + new Date(logState.checkedAt).toLocaleTimeString() : '尚未查询';
  logById('log-recorders').textContent = data ? '检测到的 recorder/下载进程：' +
    (data.recorders.map(p => p.kind + ' PID ' + p.pid).join('、') || '无') : '尚未查询板端 recorder。';
  const jobs = logById('log-jobs'); jobs.replaceChildren();
  const labels = {starting: '启动中（尚未 READY）', recording: '录制中', transferring: '下载到板端中', complete: '文件已封闭', failed: '失败或未封闭，请查看输出'};
  for (const j of data ? data.jobs : []) {
    const item = logNode('div'); item.className = 'log-job';
    item.append(logNode('p', j.id + ' · ' + (labels[j.state] || j.state) + ' · PID ' + j.pid));
    if (j.summary && j.summary.recording_status) item.append(logNode('p', '原录制检查：' + j.summary.recording_status));
    if (j.active) {
      const stop = logNode('button', j.kind === 'bag' ? '停止此工作台 bag' : '取消此 ULog 下载');
      stop.className = 'btn'; stop.dataset.stop = j.id;
      stop.onclick = () => { if (window.confirm('停止此工作台日志操作？只停止核对过的独立进程，不影响旧任务录包。'))
        logAction('/api/recording/stop', {id: j.id, confirm: '停止日志操作'}); };
      item.append(stop);
    }
    const details = logNode('details'), summary = logNode('summary', '进程输出 / 录制结果');
    details.append(summary, logNode('pre', j.tail || JSON.stringify(j.summary || {}, null, 2))); item.append(details); jobs.append(item);
  }
  if (!jobs.childNodes.length) jobs.append(logNode('p', '没有工作台登记的日志操作。旧任务内置录包请从文件列表下载。'));
  logFiles(); logButtons();
}
async function logRefreshData() {
  const snapshot = await logAPI('/api/snapshot');
  logState.supported = !!(snapshot.capabilities && snapshot.capabilities.recording === true);
  if (!logState.supported) {
    logState.connection = snapshot.connection || {}; logState.data = null; logRender();
    throw new Error('当前后端尚未支持日志操作。请使用独立 --logs-only 8792 页面，保留原设备会话。');
  }
  logById('log-login').hidden = !snapshot.logs_only;
  if (snapshot.logs_only && !logById('log-host').value) logById('log-host').value = snapshot.connection.host || '';
  const before = JSON.stringify([logState.connection.host, logState.connection.board_root]);
  logState.connection = snapshot.connection;
  if (before !== JSON.stringify([snapshot.connection.host, snapshot.connection.board_root])) {
    logState.data = null; logState.checkedAt = 0;
    logState.pendingUlog = null;
    logById('ulog-select').replaceChildren(logNode('option', '请查询或手填已核实 ID')); logById('ulog-id').value = '';
  }
  if (snapshot.connection.state !== 'ok') { logState.data = null; logRender(); return; }
  logState.data = await logAPI('/api/recording/status', {});
  logState.checkedAt = Date.now(); logRender();
  if (logState.pendingUlog) {
    const job = logState.data.jobs.find(j => j.id === logState.pendingUlog);
    if (job && job.state === 'complete') {
      const file = logState.data.files.find(f => f.path === 'flight_workbench_recordings/' + job.id + '/flight.ulg' && f.downloadable);
      if (file && typeof ulogBoardView === 'function') {
        logState.pendingUlog = null; await ulogBoardView(file);
      }
    } else if (job && !job.active) {
      logState.pendingUlog = null;
      throw new Error('飞控日志下载失败或未封闭；请检查该日志操作输出');
    }
  }
}
async function logAction(path, body) {
  if (logState.busy) return;
  logState.busy = true; logButtons(); logById('log-message').textContent = '正在操作…';
  try {
    if (path) {
      const result = await logAPI(path, body);
      if (path === '/api/recording/start' && body.kind === 'ulog') logState.pendingUlog = result.id;
    }
    await logRefreshData(); logById('log-message').textContent = '状态已刷新。';
  } catch (error) {
    logState.data = null; logById('log-message').textContent = error.message + '；请刷新核实，避免重复启动。';
  } finally { logState.busy = false; logRender(); }
}
async function logIndex() {
  if (logState.busy || !window.confirm('查询飞控日志索引？仅在未解锁时执行，会发送 LOG_REQUEST_LIST 并在结束时 END。')) return;
  logState.busy = true; logButtons(); logById('log-message').textContent = '正在查询飞控索引…';
  logById('ulog-id').value = '';
  const placeholder = logNode('option', '请选择要匹配的飞控日志'); placeholder.value = '';
  logById('ulog-select').replaceChildren(placeholder);
  try {
    const data = await logAPI('/api/recording/index', {confirm: '查询飞控日志索引'}), select = logById('ulog-select');
    for (const entry of data.entries.slice().reverse()) {
      const option = logNode('option', 'ID ' + entry.log_id + ' · ' + (entry.time_beijing || '飞控时间未知') + ' · ' + logSize(entry.bytes));
      option.value = entry.log_id; select.append(option);
    }
    const partial = data.entries.some(entry => entry.index_complete === false);
    logById('log-message').textContent = (partial ? '仅收到部分索引，可刷新重查。' : '索引已查询。') +
      '请选择日志后点击下载并查看，ID自动带入；按解锁时间和运动匹配试飞，不自动选择最大编号。';
  } catch (error) { logById('log-message').textContent = error.message; }
  finally { logState.busy = false; logButtons(); }
}
async function logDownload(file) {
  logById('log-message').textContent = '正在下载 ' + file.path + '…';
  try {
    const response = await fetch('/api/recording/download?path=' + encodeURIComponent(file.path), {cache: 'no-store'});
    if (!response.ok) { const error = await response.json(); throw new Error(error.error || '下载失败'); }
    const blob = await response.blob();
    if (blob.size !== file.size) throw new Error('文件大小与查询时不一致，请刷新后重试');
    const url = URL.createObjectURL(blob), a = logNode('a');
    a.href = url; a.download = file.path.split('/').pop(); document.body.append(a); a.click(); a.remove();
    setTimeout(() => URL.revokeObjectURL(url), 30000);
    logById('log-message').textContent = '已交给浏览器保存：' + a.download;
  } catch (error) { logById('log-message').textContent = error.message; }
}
function logInit() {
  logById('log-connect').onclick = async () => {
    if (logState.busy) return;
    const host = logById('log-host').value.trim(), password = logById('log-password').value;
    logState.busy = true; logButtons(); logById('log-message').textContent = '正在验证日志连接（不启动节点）…';
    try {
      if (host && host !== logState.connection.host) await logAPI('/api/config', {host});
      const body = {auto_password: true, save_password: logById('log-save-password').checked};
      if (password) body.password = password;
      await logAPI('/api/connect', body); await logRefreshData();
      logById('log-message').textContent = '日志连接已验证；现有设备节点未重启。';
    } catch (error) { logById('log-message').textContent = error.message; }
    finally { logById('log-password').value = ''; logState.busy = false; logButtons(); }
  };
  logById('log-refresh').onclick = () => logAction();
  logById('bag-profile').onchange = logButtons;
  logById('bag-start').onclick = () => {
    const duration = Number(logById('bag-duration').value);
    if (!Number.isInteger(duration) || duration < 1 || duration > 900) { logById('log-message').textContent = '请输入 1–900 秒整数。'; return; }
    if (window.confirm('开始独立轻量 bag？已有任务内置录包时会拒绝，不启动设备或任务。'))
      logAction('/api/recording/start', {kind: 'bag', profile: logById('bag-profile').value, duration, confirm: '开始轻量录制'});
  };
  logById('ulog-index').onclick = logIndex;
  logById('ulog-select').onchange = () => { logById('ulog-id').value = logById('ulog-select').value; logButtons(); };
  logById('ulog-id').oninput = logButtons;
  logById('ulog-fetch').onclick = () => {
    const raw = logById('ulog-id').value, log_id = Number(raw);
    if (!raw || !Number.isInteger(log_id) || log_id < 0 || log_id > 65534) { logById('log-message').textContent = '请输入已核实的日志 ID。'; return; }
    if (window.confirm('将飞控已封闭日志 ID ' + log_id + ' 下载到板端？仅连接且未解锁允许；不会启动 FC 日志。'))
      logAction('/api/recording/start', {kind: 'ulog', log_id, confirm: '取回飞控日志'});
  };
  logById('log-filter').oninput = logFiles;
  logAction();
  setInterval(() => { if (!logState.busy && logState.data && logState.data.jobs.some(j => j.active)) logAction(); }, 5000);
}
if (typeof document !== 'undefined') document.addEventListener('DOMContentLoaded', logInit);
