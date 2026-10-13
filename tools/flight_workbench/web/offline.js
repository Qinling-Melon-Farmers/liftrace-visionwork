'use strict';
const replayState = {library: null, jobs: [], busy: false, mode: 'video', reportWindows: new Map()};
const replayById = id => document.getElementById(id);
function replayNode(tag, text, cls) {
  const node = document.createElement(tag);
  if (text != null) node.textContent = String(text);
  if (cls) node.className = cls;
  return node;
}
function replayMessage(text, error = false) {
  const node = replayById('replay-message'); node.textContent = text;
  node.classList.toggle('error', error);
}
async function replayAPI(path, body) {
  const response = await fetch(path, body === undefined ? {cache: 'no-store'} : {
    method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify(body)});
  const data = await response.json();
  if (!response.ok || !data.ok) throw new Error(data.error || '请求失败，请刷新后重试');
  return data;
}
function replayOptions(id, entries, empty, value, label, requested) {
  const select = replayById(id), chosen = requested === undefined ? select.value : requested;
  select.replaceChildren();
  const first = replayNode('option', empty); first.value = ''; select.append(first);
  for (const entry of entries) {
    const option = replayNode('option', label(entry)); option.value = value(entry); select.append(option);
  }
  select.value = entries.some(e => value(e) === chosen) ? chosen : '';
}
function replayButtons() {
  const active = replayState.jobs.some(j => ['queued', 'running'].includes(j.state));
  const capabilities = replayState.library?.capabilities;
  replayById('replay-start').disabled = replayState.busy || active || !capabilities?.generate || !replayById('replay-bag').value;
  replayById('analysis-start').disabled = replayState.busy || active || !capabilities?.analysis || !replayById('replay-bag').value;
  replayById('replay-stop').disabled = replayState.busy || !active;
  replayById('replay-refresh').disabled = replayState.busy;
  replayById('replay-pick-bag').hidden = !capabilities?.native_picker;
  replayById('replay-pick-bag').disabled = replayState.busy;
  replayById('replay-upload').disabled = replayState.busy;
  document.querySelectorAll('.pick-output').forEach(button => {
    button.hidden = !capabilities?.native_picker; button.disabled = replayState.busy;
  });
  const link = replayById('replay-open'), url = replayById('replay-results').value;
  link.hidden = !url; if (url) link.href = url;
}
function replaySource() {
  const selected = replayState.library?.bags.find(b => JSON.stringify([b.root, b.path]) === replayById('replay-bag').value);
  replayById('replay-source-detail').textContent = selected ? `${selected.path}\n${(selected.size / 1048576).toFixed(1)} MiB` : '选择已封闭的 ROS1 .bag 文件。导入会保留原文件。';
  replayButtons();
}
function replayMode(mode) {
  replayState.mode = mode;
  for (const kind of ['video', 'analysis']) {
    const active = kind === mode;
    replayById('tab-' + kind).setAttribute('aria-selected', String(active));
    replayById('tab-' + kind).tabIndex = active ? 0 : -1;
    replayById('flow-' + kind).hidden = !active;
  }
}
function replayLink(text, url) {
  const link = replayNode('a', text, 'btn'); link.href = url; link.target = '_blank'; link.rel = 'noopener'; return link;
}
function replayJobs() {
  const list = replayById('replay-jobs'); list.replaceChildren();
  const labels = {queued: '等待启动', running: '处理中', ready: '已完成', failed: '未完成'};
  for (const job of replayState.jobs.slice(0, 10)) {
    const item = replayNode('article', null, 'job');
    const head = replayNode('div', null, 'job-header');
    const title = replayNode('div');
    title.append(replayNode('span', job.kind === 'analysis' ? '投递 / 降落分析' : '视频回放', 'job-type'));
    title.append(replayNode('div', job.path.split('/').pop(), 'job-name'));
    const stopped = job.phase === '已停止';
    head.append(title, replayNode('span', stopped ? '已停止' : labels[job.state] || job.state, 'job-state ' + (stopped ? 'stopped' : job.state))); item.append(head);
    item.append(replayNode('p', job.phase + (job.progress == null ? '' : ` ${job.progress}%`), 'job-phase'));
    if (['queued', 'running'].includes(job.state)) {
      const progress = replayNode('progress'); progress.max = 100;
      if (job.progress != null) progress.value = job.progress;
      progress.setAttribute('aria-label', '任务进度'); item.append(progress);
    }
    if (job.error && !stopped) item.append(replayNode('p', job.error, 'job-error'));
    if (job.output_directory) item.append(replayNode('p', '输出：' + job.output_directory, 'job-output'));
    if (job.state === 'ready') {
      const links = replayNode('div', null, 'job-links');
      links.append(replayLink(job.kind === 'analysis' ? '打开图表与模式报告' : '打开播放器', job.url));
      if (job.kind === 'analysis') links.append(replayLink('打开 PNG', job.url.replace(/analysis_report\.html$/, 'delivery_curves.png')));
      item.append(links);
    }
    const details = replayNode('details'); details.open = job.state === 'failed' && !stopped;
    details.append(replayNode('summary', '查看运行日志'), replayNode('pre', job.tail || '等待输出')); item.append(details);
    list.append(item);
    const report = replayState.reportWindows.get(job.id);
    if (report && job.state === 'ready') {
      if (!report.closed) report.location.replace(job.url);
      replayState.reportWindows.delete(job.id);
    } else if (report && job.state === 'failed') {
      if (!report.closed) report.close(); replayState.reportWindows.delete(job.id);
    }
  }
  if (!replayState.jobs.length) {
    const empty = replayNode('div', null, 'empty-state');
    empty.append(replayNode('h3', '还没有导出任务'), replayNode('p', '选择左侧日志，然后导出视频回放或分析曲线。任务开始后，这里显示实际阶段、进度和结果。'));
    list.append(empty);
  }
  replayButtons();
}
async function replayRefresh(selected) {
  if (replayState.busy) return;
  replayState.busy = true; replayButtons();
  try {
    const data = await replayAPI('/api/replay/library'); replayState.library = data; replayState.jobs = data.jobs;
    for (const [id, enabled, text] of [['replay-capabilities', data.capabilities.generate, data.capabilities.detail], ['analysis-capabilities', data.capabilities.analysis, data.capabilities.analysis_detail]]) {
      replayById(id).textContent = text; replayById(id).classList.toggle('unavailable', !enabled);
    }
    replayById('replay-roots').textContent = JSON.stringify(data.roots, null, 2);
    replayOptions('replay-bag', data.bags, '请选择或导入日志', b => JSON.stringify([b.root, b.path]), b => b.path.split('/').pop() + ' / ' + (b.size / 1048576).toFixed(1) + ' MiB', selected);
    const outputs = [{key: 'generated', name: '工作台本地结果库'}, ...Object.entries(data.output_locations || {}).map(([key, name]) => ({key, name}))];
    for (const id of ['video-output', 'analysis-output']) {
      const chosen = replayById(id).value || 'generated';
      replayOptions(id, outputs, '请选择导出目录', e => e.key, e => e.name, chosen);
    }
    replayOptions('replay-results', data.results, '请选择已生成结果', r => r.url, r => (r.kind === 'analysis' ? '曲线报告 / ' : '视频回放 / ') + r.path);
    if (data.truncated) replayMessage('目录扫描达到上限，可选择单份 bag 或缩小配置的数据根。');
    replayJobs(); replaySource();
  } catch (error) { replayMessage(error.message, true); }
  finally { replayState.busy = false; replayButtons(); }
}
async function replayStart(kind) {
  const button = replayById(kind === 'analysis' ? 'analysis-start' : 'replay-start');
  if (button.disabled) return;
  const [root, path] = JSON.parse(replayById('replay-bag').value);
  const body = {root, path, kind, output_key: replayById(kind === 'analysis' ? 'analysis-output' : 'video-output').value};
  if (kind === 'video') {
    body.fps = Number(replayById('replay-fps').value); body.frame = replayById('replay-frame').value.trim();
    body.encoder = replayById('replay-encoder').value; body.speed = replayById('replay-speed').value; body.video_set = replayById('replay-video-set').value;
    if (!Number.isInteger(body.fps) || body.fps < 1 || body.fps > 30 || !body.frame) return replayMessage('请输入 1–30 的整数帧率及有效坐标系。', true);
  }
  let report = null;
  if (kind === 'analysis') {
    report = window.open('about:blank', '_blank');
    if (report) { report.opener = null; report.document.title = '正在分析飞行日志'; report.document.body.textContent = '正在分析日志，完成后此页自动显示报告。'; }
  }
  replayState.busy = true; replayButtons();
  try {
    const result = await replayAPI('/api/replay/start', body);
    if (report) replayState.reportWindows.set(result.id, report);
    replayMessage(kind === 'analysis' ? '分析任务已开始，完成后自动显示报告。' : '回放任务已开始，视频校验完成后可打开播放器。');
    replayState.jobs = (await replayAPI('/api/replay/jobs')).jobs; replayJobs();
  } catch (error) { if (report && !report.closed) report.close(); replayMessage(error.message, true); }
  finally { replayState.busy = false; replayButtons(); }
}
async function replayPick(kind, selectId) {
  if (replayState.busy) return;
  replayState.busy = true; replayButtons(); replayMessage('请在本机文件对话框中完成选择。');
  try {
    const result = await replayAPI('/api/replay/select-local', {kind});
    replayState.busy = false;
    if (result.cancelled) { replayMessage('已取消选择。'); return; }
    await replayRefresh(kind === 'bag' ? JSON.stringify([result.root, result.path]) : undefined);
    if (selectId) replayById(selectId).value = result.root;
    replayMessage(kind === 'bag' ? '本地日志已选中。' : '导出文件夹已选择：' + result.directory);
  } catch (error) { replayMessage(error.message, true); }
  finally { replayState.busy = false; replayButtons(); }
}
async function replayUpload() {
  const input = replayById('replay-upload'), file = input.files[0];
  if (!file) return;
  if (!file.name.toLowerCase().endsWith('.bag') || file.size > 4 * 1024 ** 3) { input.value = ''; return replayMessage('请选择 4 GiB 以内的 .bag 文件。', true); }
  replayState.busy = true; replayButtons();
  try {
    const result = await new Promise((resolve, reject) => {
      const xhr = new XMLHttpRequest(); xhr.open('POST', '/api/replay/upload?name=' + encodeURIComponent(file.name));
      xhr.setRequestHeader('Content-Type', 'application/octet-stream');
      xhr.upload.onprogress = event => replayMessage('正在导入 ' + file.name + (event.lengthComputable ? ` ${Math.round(event.loaded / event.total * 100)}%` : ''));
      xhr.onerror = () => reject(new Error('导入中断，请重新导入日志。'));
      xhr.onload = () => { try { const d = JSON.parse(xhr.responseText); if (xhr.status >= 400 || !d.ok) throw new Error(d.error || '导入失败'); resolve(d); } catch (e) { reject(e); } };
      xhr.send(file);
    });
    replayState.busy = false; await replayRefresh(JSON.stringify([result.root, result.path])); replayMessage('已导入日志，原文件保留。');
  } catch (error) { replayMessage(error.message, true); }
  finally { input.value = ''; replayState.busy = false; replayButtons(); }
}
async function replayPoll() {
  if (replayState.busy) return;
  try {
    const active = replayState.jobs.some(j => ['queued', 'running'].includes(j.state));
    replayState.jobs = (await replayAPI('/api/replay/jobs')).jobs; replayJobs();
    if (active && !replayState.jobs.some(j => ['queued', 'running'].includes(j.state))) await replayRefresh();
  } catch (error) { replayMessage(error.message, true); }
}
function replayInit() {
  replayById('replay-refresh').onclick = () => replayRefresh();
  replayById('replay-start').onclick = () => replayStart('video');
  replayById('analysis-start').onclick = () => replayStart('analysis');
  replayById('replay-pick-bag').onclick = () => replayPick('bag');
  replayById('replay-upload').onchange = replayUpload;
  replayById('replay-bag').onchange = replaySource;
  replayById('replay-results').onchange = replayButtons;
  replayById('replay-stop').onclick = async () => {
    if (!confirm('停止当前离线导出？已生成的部分文件会保留。')) return;
    try { await replayAPI('/api/replay/stop', {}); await replayPoll(); replayMessage('已请求停止当前导出。'); } catch (e) { replayMessage(e.message, true); }
  };
  document.querySelectorAll('.pick-output').forEach(button => button.onclick = () => replayPick('folder', button.dataset.for));
  document.querySelectorAll('.flow-tab').forEach(button => {
    button.onclick = () => replayMode(button.dataset.mode);
    button.onkeydown = event => {
      if (!['ArrowLeft', 'ArrowRight', 'Home', 'End'].includes(event.key)) return;
      event.preventDefault(); const mode = event.key === 'Home' ? 'video' : event.key === 'End' ? 'analysis' : replayState.mode === 'video' ? 'analysis' : 'video';
      replayMode(mode); replayById('tab-' + mode).focus();
    };
  });
  replayRefresh(); setInterval(replayPoll, 2000);
}
if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', replayInit);
else replayInit();
