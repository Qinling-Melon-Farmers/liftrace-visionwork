'use strict';
const replayState = {library: null, jobs: [], busy: false};
const replayById = id => document.getElementById(id);
function replayNode(tag, text) {
  const node = document.createElement(tag);
  if (text != null) node.textContent = String(text);
  return node;
}
async function replayAPI(path, body) {
  const response = await fetch(path, body === undefined ? {cache: 'no-store'} : {
    method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify(body)});
  const data = await response.json();
  if (!response.ok || !data.ok) throw new Error(data.error || '回放请求失败');
  return data;
}
function replayOptions(id, entries, empty, value, label) {
  const select = replayById(id), chosen = select.value;
  select.replaceChildren();
  select.append(replayNode('option', empty));
  select.firstChild.value = '';
  for (const entry of entries) {
    const option = replayNode('option', label(entry)); option.value = value(entry); select.append(option);
  }
  select.value = entries.some(e => value(e) === chosen) ? chosen : '';
}
function replayButtons() {
  replayById('replay-start').disabled = replayState.busy || !replayState.library ||
    !replayState.library.capabilities.generate || !replayById('replay-bag').value ||
    replayState.jobs.some(j => j.state === 'queued' || j.state === 'running');
  replayById('replay-refresh').disabled = replayState.busy;
  const link = replayById('replay-open'), url = replayById('replay-results').value;
  link.hidden = !url;
  if (url) link.href = url;
}
function replayJobs() {
  const list = replayById('replay-jobs'); list.replaceChildren();
  for (const job of replayState.jobs) {
    const item = replayNode('div'); item.className = 'log-job';
    item.append(replayNode('p', job.path + ' · ' + job.state + ' · ' + job.phase +
      (job.progress == null ? '' : ' · ' + job.progress + '%') + ' · encoder=' + (job.encoder || 'cpu')));
    if (job.error) item.append(replayNode('p', job.error));
    if (job.state === 'ready') {
      const link = replayNode('a', '打开播放器 ↗'); link.href = job.url;
      link.className = 'btn'; link.target = '_blank'; link.rel = 'noopener'; item.append(link);
    }
    const details = replayNode('details'); details.open = job.state === 'running' || job.state === 'failed';
    details.append(replayNode('summary', 'operation.log（最近12KB）'), replayNode('pre', job.tail || '等待输出'));
    item.append(details); list.append(item);
  }
  if (!replayState.jobs.length) list.append(replayNode('p', '尚无工作台离线回放任务。'));
  replayButtons();
}
async function replayRefresh() {
  if (replayState.busy) return;
  replayState.busy = true; replayButtons();
  try {
    const data = await replayAPI('/api/replay/library'); replayState.library = data; replayState.jobs = data.jobs;
    replayById('replay-capabilities').textContent = data.capabilities.detail;
    replayById('replay-roots').textContent = JSON.stringify(data.roots, null, 2);
    replayOptions('replay-bag', data.bags, '请选择 bag', b => JSON.stringify([b.root, b.path]),
      b => b.root + ' / ' + b.path + ' · ' + (b.size / 1048576).toFixed(1) + ' MiB');
    replayOptions('replay-results', data.results, '请选择已生成结果', r => r.url, r => r.root + ' / ' + r.path);
    replayById('replay-message').textContent = data.truncated ? '目录扫描达到上限；请将 data_roots 配置为更小的目录。' : '本地库已刷新';
    replayJobs();
  } catch (error) { replayById('replay-message').textContent = error.message; }
  finally { replayState.busy = false; replayButtons(); }
}
async function replayStart() {
  if (replayById('replay-start').disabled) return;
  replayState.busy = true; replayButtons();
  try {
    const [root, path] = JSON.parse(replayById('replay-bag').value);
    await replayAPI('/api/replay/start', {root, path, fps: Number(replayById('replay-fps').value),
      frame: replayById('replay-frame').value.trim(), encoder: replayById('replay-encoder').value});
    replayById('replay-message').textContent = '回放任务已登记；刷新或关闭页面不停止生成。';
    replayState.jobs = (await replayAPI('/api/replay/jobs')).jobs; replayJobs();
  } catch (error) { replayById('replay-message').textContent = error.message; }
  finally { replayState.busy = false; replayButtons(); }
}
async function replayPoll() {
  if (replayState.busy) return;
  try {
    const previouslyActive = replayState.jobs.some(j => ['queued', 'running'].includes(j.state));
    replayState.jobs = (await replayAPI('/api/replay/jobs')).jobs; replayJobs();
    if (previouslyActive && !replayState.jobs.some(j => ['queued', 'running'].includes(j.state))) await replayRefresh();
  } catch (error) { replayById('replay-message').textContent = error.message; }
}
function replayInit() {
  replayById('replay-refresh').onclick = replayRefresh;
  replayById('replay-start').onclick = replayStart;
  replayById('replay-bag').onchange = replayButtons;
  replayById('replay-results').onchange = replayButtons;
  replayRefresh(); setInterval(replayPoll, 2000);
}
if (typeof document !== 'undefined') {
  if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', replayInit);
  else replayInit();
}
