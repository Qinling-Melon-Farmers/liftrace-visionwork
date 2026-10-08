'use strict';
const ulogState = {records: [], selected: '', busy: false};
const ulogById = id => document.getElementById(id);
function ulogNode(tag, text) { const node = document.createElement(tag); if (text != null) node.textContent = String(text); return node; }
async function ulogRequest(path, options) {
  const response = await fetch(path, options || {cache: 'no-store'});
  const data = await response.json();
  if (!response.ok || !data.ok) throw new Error(data.error || 'ULog请求失败');
  return data;
}
function ulogURL(id, name) { return '/api/ulog/file?id=' + encodeURIComponent(id) + '&name=' + encodeURIComponent(name); }
function ulogRender() {
  const select = ulogById('ulog-library'); select.replaceChildren();
  const labels = {queued: '等待分析', analyzing: '分析中', ready: '可查看', failed: '失败'};
  for (const record of ulogState.records) {
    const option = ulogNode('option', record.name + ' · ' + labels[record.state] + ' · ' + new Date(record.created_at * 1000).toLocaleString());
    option.value = record.id; select.append(option);
  }
  if (!ulogState.records.length) { const option = ulogNode('option', '尚未导入'); option.value = ''; select.append(option); }
  select.value = ulogState.selected;
  const record = ulogState.records.find(r => r.id === ulogState.selected);
  ulogById('ulog-result').hidden = !record || record.state !== 'ready';
  ulogById('ulog-local-add').disabled = ulogState.busy;
  if (!record) return;
  if (record.state === 'failed') { ulogById('ulog-local-message').textContent = record.error; return; }
  if (record.state !== 'ready') { ulogById('ulog-local-message').textContent = record.name + ' · ' + labels[record.state] + '（仅本机离线分析）'; return; }
  ulogById('ulog-local-message').textContent = '分析完成；可选择历史文件、查看图表或下载CSV/摘要。';
  ulogById('ulog-title').textContent = record.name;
  ulogById('ulog-summary').textContent = JSON.stringify(record.summary, null, 2);
  const exports = ulogById('ulog-exports'); exports.replaceChildren();
  for (const [name, title] of [['analysis.zip', '全部图表/CSV/摘要 ZIP'], ['input.ulg', '保存原始 ULog'],
      ...record.files.filter(n => !n.endsWith('.png')).map(n => [n, n])]) {
    const link = ulogNode('a', title); link.className = 'btn'; link.href = ulogURL(record.id, name);
    link.download = name === 'input.ulg' ? record.name : name; exports.append(link);
  }
  const images = ulogById('ulog-images'); images.replaceChildren();
  for (const name of ['trajectory.png', 'overview.png']) {
    if (!record.files.includes(name)) continue;
    const link = ulogNode('a'); link.href = ulogURL(record.id, name); link.target = '_blank'; link.rel = 'noopener';
    const img = ulogNode('img'); img.src = link.href; img.alt = name === 'trajectory.png' ? '局部轨迹与位置高度估计' : '电机命令、姿态、RC、电池与EKF诊断';
    img.style.width = '100%'; img.loading = 'lazy'; link.append(img); images.append(link);
  }
}
async function ulogRefresh(selected) {
  const data = await ulogRequest('/api/ulog/library'); ulogState.records = data.records;
  ulogState.selected = selected || ulogState.selected || (data.records[0] && data.records[0].id) || '';
  ulogRender();
}
async function ulogBoardView(file) {
  if (ulogState.busy) return;
  ulogState.busy = true; ulogRender(); ulogById('ulog-local-message').textContent = '取回已封闭 ULog 到本地查看器…';
  try {
    const result = await ulogRequest('/api/ulog/board', {method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify({path: file.path})});
    await ulogRefresh(result.id);
  } catch (error) { ulogById('ulog-local-message').textContent = error.message; }
  finally { ulogState.busy = false; ulogById('ulog-local-add').disabled = false; }
}
function ulogInit() {
  ulogById('ulog-library').onchange = () => { ulogState.selected = ulogById('ulog-library').value; ulogRender(); };
  ulogById('ulog-library-refresh').onclick = () => ulogRefresh().catch(e => { ulogById('ulog-local-message').textContent = e.message; });
  ulogById('ulog-local-add').onclick = async () => {
    if (ulogState.busy) return;
    const files = Array.from(ulogById('ulog-local-files').files || []);
    if (!files.length) { ulogById('ulog-local-message').textContent = '请先选择本地 .ulg 文件。'; return; }
    ulogState.busy = true; ulogRender();
    const errors = [];
    try {
      for (const file of files) {
        try {
          if (!file.name.toLowerCase().endsWith('.ulg') || file.size < 16 || file.size > 128 * 1048576) throw new Error('文件扩展名或大小无效');
          const data = await ulogRequest('/api/ulog/upload?name=' + encodeURIComponent(file.name),
              {method: 'POST', headers: {'Content-Type': 'application/octet-stream'}, body: file});
          await ulogRefresh(data.id);
        } catch (error) { errors.push(file.name + '：' + error.message); }
      }
    } finally {
      ulogState.busy = false; ulogById('ulog-local-add').disabled = false;
      if (errors.length) ulogById('ulog-local-message').textContent = errors.join('\n');
    }
  };
  ulogRefresh().catch(e => { ulogById('ulog-local-message').textContent = e.message; });
  setInterval(() => { if (!ulogState.busy && ulogState.records.some(r => ['queued', 'analyzing'].includes(r.state)))
    ulogRefresh().catch(e => { ulogById('ulog-local-message').textContent = e.message; }); }, 2000);
}
if (typeof document !== 'undefined') document.addEventListener('DOMContentLoaded', ulogInit);
