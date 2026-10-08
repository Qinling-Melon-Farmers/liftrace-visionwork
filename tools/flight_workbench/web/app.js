/* Liftrace 试飞验证看板 · 纯前端（无构建 / 无依赖）
 * 结构：util → state → api → AnsiTerm → bus(SSE) → render* → actions → init
 * 安全约定：任何会下发到板端的动作都必须先把完整命令展示给人看；
 *          本文件不含解锁 / 起飞 / 投递下发，只调用现场既有入口。
 */
'use strict';

/* ==========================================================================
 * 1. 工具
 * ========================================================================== */

var $ = function (sel, root) { return (root || document).querySelector(sel); };

function el(tag, cls, text) {
  var n = document.createElement(tag);
  if (cls) n.className = cls;
  if (text !== undefined && text !== null) n.textContent = String(text);
  return n;
}
function clear(node) { while (node.firstChild) node.removeChild(node.firstChild); }
function txt(v) { return (v === undefined || v === null) ? '' : String(v); }
function pad2(n) { return (n < 10 ? '0' : '') + n; }
function hhmmss(sec) {
  if (!sec || !isFinite(sec)) return '—';
  var d = new Date(sec * 1000);
  return pad2(d.getHours()) + ':' + pad2(d.getMinutes()) + ':' + pad2(d.getSeconds());
}
function dur(sec) {
  if (sec === undefined || sec === null || !isFinite(sec) || sec < 0) return '—';
  var s = Math.floor(sec);
  return pad2(Math.floor(s / 60)) + ':' + pad2(s % 60);
}
function fmtSize(n) {
  if (n === undefined || n === null || !isFinite(n)) return '—';
  if (n < 1024) return n + ' B';
  if (n < 1024 * 1024) return (n / 1024).toFixed(1) + ' KB';
  return (n / 1048576).toFixed(2) + ' MB';
}

/* 只允许保存 UI 偏好；绝不写口令 / 密钥 */
var PREF_KEY = 'liftrace.flight_workbench.prefs.v1';
var prefs = { sound: false, autoscroll: true, group_id: null };
function loadPrefs() {
  try {
    var raw = window.localStorage.getItem(PREF_KEY);
    if (raw) {
      var o = JSON.parse(raw);
      if (o && typeof o === 'object') {
        prefs.sound = !!o.sound;
        prefs.autoscroll = o.autoscroll === undefined ? true : !!o.autoscroll;
        prefs.group_id = typeof o.group_id === 'string' ? o.group_id : null;
      }
    }
  } catch (e) { /* localStorage 不可用时忽略 */ }
}
function savePrefs() {
  try { window.localStorage.setItem(PREF_KEY, JSON.stringify(prefs)); } catch (e) { /* ignore */ }
}

/* 复制到剪贴板（失败时退回 prompt 展示，便于手工复制） */
function copyText(text, label) {
  var s = String(text === undefined || text === null ? '' : text);
  var done = function () { toast('ok', (label || '内容') + '已复制到剪贴板'); };
  if (navigator.clipboard && navigator.clipboard.writeText) {
    navigator.clipboard.writeText(s).then(done, function () { fallbackCopy(s, label); });
  } else fallbackCopy(s, label);
}
function fallbackCopy(s, label) {
  try {
    var ta = el('textarea');
    ta.value = s;
    ta.style.position = 'fixed'; ta.style.opacity = '0';
    document.body.appendChild(ta); ta.select();
    var okc = document.execCommand('copy');
    document.body.removeChild(ta);
    toast(okc ? 'ok' : 'warn', okc ? ((label || '内容') + '已复制') : '复制失败，请手工选择文本');
  } catch (e) { toast('warn', '复制失败：' + (label || '') + ' 请手工选择'); }
}
function copyBtn(getText, label) {
  var b = el('button', 'btn btn-sm btn-ghost', '复制');
  b.addEventListener('click', function () { copyText(getText(), label || '命令'); });
  return b;
}

/* toast：右上角，4 秒消失 */
var TOAST_MAX = 6;
function toast(level, text) {
  var box = $('#toasts');
  if (!box) return;
  var lv = (level === 'ok' || level === 'warn' || level === 'error') ? level : 'info';
  var t = el('div', 'toast t-' + lv);
  t.appendChild(el('span', 'tiny muted', hhmmss(Date.now() / 1000) + '  '));
  t.appendChild(document.createTextNode(txt(text)));
  box.appendChild(t);
  while (box.children.length > TOAST_MAX) box.removeChild(box.firstChild);
  window.setTimeout(function () { if (t.parentNode) t.parentNode.removeChild(t); }, 4000);
}

/* 提示音：WebAudio 现场生成，无音频文件 */
var _ac = null;
function audioCtx() {
  if (_ac) return _ac;
  try {
    var C = window.AudioContext || window.webkitAudioContext;
    if (!C) return null;
    _ac = new C();
  } catch (e) { _ac = null; }
  return _ac;
}
function beep(kind) {
  if (!prefs.sound) return;
  var c = audioCtx();
  if (!c) return;
  try { if (c.state === 'suspended' && c.resume) c.resume(); } catch (e) { /* ignore */ }
  var notes = (kind === 'error') ? [[400, 0], [300, 0.18]] : [[880, 0], [1320, 0.14]];
  notes.forEach(function (n) {
    var osc = c.createOscillator(), g = c.createGain();
    osc.type = 'sine';
    osc.frequency.value = n[0];
    var t0 = c.currentTime + n[1];
    g.gain.setValueAtTime(0.0001, t0);
    g.gain.exponentialRampToValueAtTime(0.18, t0 + 0.02);
    g.gain.exponentialRampToValueAtTime(0.0001, t0 + 0.16);
    osc.connect(g); g.connect(c.destination);
    osc.start(t0); osc.stop(t0 + 0.2);
  });
}

/* ==========================================================================
 * 2. 全局状态
 * ========================================================================== */

var state = {
  snapshot: null,
  connection: { host: '', state: 'unknown', detail: '', board_root: '', site_dir: '', password_available: false },
  profile: { path: '', auto_password: false, saved_password: false },
  groups: [],
  terminals: [],
  sessions: {},
  trial: {},
  stage: null,
  telemetry: null,
  probe: {},
  orchestration: null,
  alerts: [],
  timeline: [],
  board: { logs: [], preflight: null },
  report: null,
  toast: null,
  sse: { state: 'idle' },
  tabs: {},              // 终端面板 UI 状态（per terminal id）
  activeTerm: null,
  terms: {},             // id -> AnsiTerm
  logTerm: null,         // independent read-only trial mirror (a DOM node has one parent)
  drawer: 'run',
  drawerCollapsed: false,
  reportText: ''
};
var ALERT_KEEP = 200;
var TL_KEEP = 300;

// Connection events are patches; keep host options and configuration from the snapshot.
function mergeConnection(patch) {
  if (patch) Object.assign(state.connection, patch);
}

function hostChoices() {
  var c = state.connection || {};
  var choices = (c.host_options || []).map(function (o) {
    return { value: o.host, label: o.host + (o.label ? (' — ' + o.label) : '') };
  });
  if (c.host && !choices.some(function (o) { return o.value === c.host; })) {
    choices.push({ value: c.host, label: c.host + ' — 当前自定义地址' });
  }
  choices.push({ value: '', label: '自定义地址…' });
  return choices;
}

function connected() { return state.connection && state.connection.state === 'ok'; }

/* ==========================================================================
 * 3. API 封装
 * ========================================================================== */

function qs(obj) {
  var parts = [];
  Object.keys(obj || {}).forEach(function (k) {
    var v = obj[k];
    if (v === undefined || v === null) return;
    parts.push(encodeURIComponent(k) + '=' + encodeURIComponent(v));
  });
  return parts.join('&');
}

function postJSON(path, body) {
  return window.fetch(path, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(body || {})
  }).then(function (r) {
    return r.json().catch(function () { return { ok: false, error: 'HTTP ' + r.status + ' 响应不是 JSON' }; })
      .then(function (j) {
        if (!j || typeof j !== 'object') j = { ok: false, error: 'HTTP ' + r.status + ' 空响应' };
        if (j.ok === undefined) j.ok = r.ok;
        return j;
      });
  }, function (err) {
    return { ok: false, error: '请求失败：' + (err && err.message ? err.message : String(err)) };
  });
}
function getJSON(path) {
  return window.fetch(path).then(function (r) { return r.json(); },
    function (err) { return { ok: false, error: String(err && err.message ? err.message : err) }; });
}

var api = {
  snapshot: function () { return getJSON('/api/snapshot'); },
  config: function (body) { return postJSON('/api/config', body); },
  connect: function (body) { return postJSON('/api/connect', body || {}); },
  disconnect: function () { return postJSON('/api/disconnect', {}); },
  preflight: function () { return postJSON('/api/action/preflight', {}); },
  probeReconnect: function () { return postJSON('/api/action/probe_reconnect', {}); },
  startAll: function (includeServo, groupId) { return postJSON('/api/action/start_all', { include_servo: !!includeServo, group_id: groupId, confirm: '启动设备' }); },
  stopAll: function () { return postJSON('/api/action/stop_all', {}); },
  missionStart: function () {
    // 后端要求确认词「启动任务」（仅在 READY 且飞手完成解锁/悬停后调用一次）
    return postJSON('/api/action/mission_start', { confirm: '启动任务' });
  },
  report: function () { return postJSON('/api/action/report', {}); },
  sessionOpen: function (id, confirmText) {
    var b = { id: id };
    if (confirmText !== undefined && confirmText !== null) b.confirm = confirmText;
    return postJSON('/api/session/open', b);
  },
  sessionInput: function (id, data) { return postJSON('/api/session/input', { id: id, data: data }); },
  sessionClose: function (id) { return postJSON('/api/session/close', { id: id }); },
  sessionClear: function (id) { return postJSON('/api/session/clear', { id: id }); },
  sessionKey: function (id, key) { return postJSON('/api/session/key', { id: id, key: key || 'C-c' }); },
  sessionResize: function (id, rows, cols) { return postJSON('/api/session/resize', { id: id, rows: rows, cols: cols }); },
  trialStart: function (body) { return postJSON('/api/trial/start', body); },
  trialStop: function () { return postJSON('/api/trial/stop', {}); },
  logsRefresh: function () { return postJSON('/api/logs/refresh', {}); },
  logsTail: function (run, file, lines) { return postJSON('/api/logs/tail', { run: run, file: file, lines: lines || 200 }); }
};

/* 只读数据请求 + 错误 toast 的统一封装 */
function act(promise, failPrefix) {
  return promise.then(function (res) {
    if (!res || res.ok !== true) toast('error', (failPrefix || '请求失败') + '：' + txt(res && res.error ? res.error : '未知错误'));
    return res;
  });
}

/* ==========================================================================
 * 4. ANSI 终端渲染（增量追加，绝不整体重绘历史行）
 * ========================================================================== */

var MAX_LINES = 5000;      // 每会话最多保留行数
var RENDER_TAIL = 200;     // 增量更新时最多重建的尾部行数
/* 注：文本一律通过 textContent 写入 DOM，不需要 HTML 转义；
 *     终端文本的转义序列在 AnsiTerm._parse / 下方颜色表中统一处理。 */
var ANSI_FG = ['#3f4753', '#ff6b6b', '#7ee787', '#ffd866', '#6fb3ff', '#d2a8ff', '#5fd7d7', '#d7dde5'];
var ANSI_FG_BRIGHT = ['#6b7787', '#ff9d97', '#b3f0b8', '#ffe9a3', '#9ccbff', '#e2ccff', '#9ff0f0', '#ffffff'];
function ansiColor(code) {
  if (code >= 30 && code <= 37) return ANSI_FG[code - 30];
  if (code >= 90 && code <= 97) return ANSI_FG_BRIGHT[code - 90];
  return null;
}
/* 解析一段文本 -> [{t:文本, fg,bold}...]；SGR 之外的所有转义一律吞掉 */

function AnsiTerm(opts) {
  this.id = opts.id;
  this.maxLines = opts.maxLines || MAX_LINES;
  this.scrollEl = el('div', 'term-out');
  this.scrollEl.setAttribute('role', 'log');
  this.onTrim = opts.onTrim || null;
  this.onAppend = opts.onAppend || null;
  this.buf = [];           // 行 = {chars:[{c,fg,bold}]}
  this.col = 0;            // 当前光标列（\r 覆盖依赖它）
  this.style = { fg: null, bold: false };
  this._queue = '';
  this._pending = false;
  this._dirtyFrom = -1;
  this._rendered = 0;
  this._staleTail = false;
  this._droppedLines = 0;
  this._screen = false;
  this._filter = '';
  this._filterMode = false;
  this.lines = 0;          // 收到过的行数（含新行）
  this.autoScroll = !!prefs.autoscroll;
  this.followTail = true;
  this.savedScrollTop = 0;
  var self = this;
  this.scrollEl.addEventListener('scroll', function () {
    if (!self.scrollEl.isConnected) return;
    self.savedScrollTop = self.scrollEl.scrollTop;
    self.followTail = self.scrollEl.scrollHeight - self.scrollEl.clientHeight - self.scrollEl.scrollTop < 24;
  });
}
AnsiTerm.prototype.saveScroll = function () {
  if (this.scrollEl.isConnected) this.savedScrollTop = this.scrollEl.scrollTop;
};
AnsiTerm.prototype.stick = function () {
  this.scrollEl.scrollTop = (this.autoScroll && this.followTail)
    ? this.scrollEl.scrollHeight : this.savedScrollTop;
};
AnsiTerm.prototype.append = function (data) {
  if (data === undefined || data === null) return;
  this._queue += String(data);
  if (this._pending) return;
  this._pending = true;
  var self = this;
  window.requestAnimationFrame(function () {
    self._pending = false;      // 必须先清标记再取队列，避免 rAF 同步回调时丢数据
    var s = self._queue;
    self._queue = '';
    if (!s) return;
    self._parse(s);
    self._flush();
  });
};
AnsiTerm.prototype._parse = function (s) {
  for (var i = 0; i < s.length; i++) {
    var ch = s.charAt(i);
    if (ch === '\x1b') {
      // 注意：s.slice(i) 仍带 ESC 本身，正则必须锚定 \x1b
      var rest = s.slice(i);
      var m = /^\x1b\[([0-9;?]*)([ -\/]*)([@-~])/.exec(rest);
      if (m) {
        if (m[3] === 'm') {
          var parts = m[1].split(';');
          for (var k = 0; k < parts.length; k++) {
            var n = parts[k] === '' ? 0 : parseInt(parts[k], 10);
            if (isNaN(n)) continue;
            if (n === 0) { this.style = { fg: null, bold: false }; }
            else if (n === 1) { this.style = { fg: this.style.fg, bold: true }; }
            else if (n === 22) { this.style = { fg: this.style.fg, bold: false }; }
            else if (n === 39) { this.style = { fg: null, bold: this.style.bold }; }
            else { var c = ansiColor(n); if (c) this.style = { fg: c, bold: this.style.bold }; }
          }
        } else if (m[3] === 'J' || m[3] === 'K') {
          // 清屏 / 清行：只在“清整屏”时重置缓冲，清行忽略（ROS 进度输出不依赖它）
          var pm = (m[1].charAt(0) === '2');
          if (m[3] === 'J' && pm) this._screenClear();
        }
        // m[0] 含 ESC，循环末尾还会 +1，正好补齐
        i += m[0].length - 1;
        continue;
      }
      var m2 = /^\x1b\][^\x07\x1b]*(?:\x07|\x1b\\)?/.exec(rest);
      if (m2) { i += m2[0].length - 1; continue; }
      var m3 = /^\x1b[@-Z\\-_]/.exec(rest);
      if (m3) { i += m3[0].length - 1; continue; }
      continue; // 孤立 ESC，吞掉
    }
    if (ch === '\n') { this._newLine(); continue; }
    if (ch === '\r') { this._setLineDirty(); this.col = 0; continue; }
    if (ch === '\b') { if (this.col > 0) this.col--; continue; }
    if (ch === '\x00' || ch === '\x07') continue;
    if (ch < ' ') continue; // 其余控制字符吞掉
    this._putChar(ch);
  }
};
AnsiTerm.prototype._setLineDirty = function () {
  var last = this.buf.length - 1;
  if (this._dirtyFrom < 0 || this._dirtyFrom > last) this._dirtyFrom = last;
};
AnsiTerm.prototype._cur = function () {
  if (!this.buf.length) this.buf.push({ chars: [] });
  return this.buf[this.buf.length - 1];
};
AnsiTerm.prototype._putChar = function (ch) {
  var line = this._cur();
  var w = /[\u1100-\u115f\u2e80-\ua4cf\ua960-\ua97f\uac00-\ud7ff\uf900-\ufaff\ufe10-\ufe19\ufe30-\ufe6f\uff00-\uff60\uffe0-\uffe6]/.test(ch) ? 2 : 1; // 仅用于光标推进
  var idx = this.col;
  line.chars[idx] = { c: ch, fg: this.style.fg, bold: this.style.bold };
  // 宽字符占两格：第二格记为空白占位，避免列错位
  if (w === 2) {
    var nxt = line.chars[idx + 1];
    if (!nxt || nxt.c === '') line.chars[idx + 1] = { c: '', fg: this.style.fg, bold: this.style.bold };
  }
  this.col = idx + w;
  if (this._dirtyFrom < 0 || this._dirtyFrom > this.buf.length - 1) this._dirtyFrom = this.buf.length - 1;
};
AnsiTerm.prototype._newLine = function () {
  this.buf.push({ chars: [] });
  this.col = 0;
  this.lines++;
  this._trim();
  var last = this.buf.length - 1;
  if (this._dirtyFrom < 0 || this._dirtyFrom > last) this._dirtyFrom = last;
};
AnsiTerm.prototype._trim = function () {
  if (this.buf.length <= this.maxLines) return;
  var drop = this.buf.length - this.maxLines;
  var firstDroppedWasRendered = (this._rendered > 0); // DOM 首行是否已被丢掉
  this.buf.splice(0, drop);
  this._droppedLines += drop;
  if (firstDroppedWasRendered) this._staleTail = true;
  if (this._dirtyFrom >= 0) {
    this._dirtyFrom = this._dirtyFrom - drop;
    if (this._dirtyFrom < 0) this._dirtyFrom = 0;
  }
  this._rendered = Math.max(0, this._rendered - drop);
  if (this.onTrim) this.onTrim(this);
};
AnsiTerm.prototype._screenClear = function () {
  this.buf = [{ chars: [] }];
  this.col = 0;
  this._dirtyFrom = 0;
  this._staleTail = true; // 整屏清空：下次统一重绘
  this._screen = true;
};
AnsiTerm.prototype._flush = function () {
  if (this._screen) {
    this._screen = false;
    this._renderAll(true);
    return;
  }
  if (this._filterMode) { this._renderAll(false); return; }
  if (this._staleTail) { this._renderAll(false); return; } // 首行被丢弃过：DOM 与行号已错位
  if (this._dirtyFrom < 0) return;
  var start = this._dirtyFrom;
  this._dirtyFrom = -1;
  if (start > this._rendered || (this._rendered - start) > RENDER_TAIL) {
    this._renderAll(false);
    return;
  }
  this._renderTail(start);
};
AnsiTerm.prototype._lineNode = function (line) {
  var d = el('div', 'tl');
  var chars = line.chars || [];
  var frag = document.createDocumentFragment();
  var runText = '', runFg = null, runBold = false;
  var flushRun = function () {
    if (!runText) return;
    var sp = el('span');
    if (runFg) sp.style.color = runFg;
    if (runBold) sp.style.fontWeight = '700';
    sp.textContent = runText;
    frag.appendChild(sp);
    runText = '';
  };
  for (var i = 0; i < chars.length; i++) {
    var c = chars[i];
    if (!c) continue;
    if (c.c === '') continue; // 宽字符占位
    if (c.fg !== runFg || c.bold !== runBold) { flushRun(); runFg = c.fg; runBold = c.bold; }
    runText += c.c;
  }
  flushRun();
  d.appendChild(frag);
  return d;
};
AnsiTerm.prototype._renderTail = function (start) {
  this.saveScroll();
  // DOM indices match buf indices, including line zero and pure append batches.
  while (this.scrollEl.children.length > start) this.scrollEl.removeChild(this.scrollEl.lastChild);
  var frag = document.createDocumentFragment();
  for (var i = start; i < this.buf.length; i++) frag.appendChild(this._lineNode(this.buf[i]));
  this.scrollEl.appendChild(frag);
  this._rendered = this.buf.length;
  this._staleTail = false;
  this.stick();
  if (this.onAppend) this.onAppend(this);
};
AnsiTerm.prototype._renderAll = function () {
  this.saveScroll();
  clear(this.scrollEl);
  var frag = document.createDocumentFragment();
  if (this._filterMode) {
    this._filterRows().forEach(function (row) { frag.appendChild(row); });
  } else {
    // buf is already bounded by maxLines; do not drop all but 200 DOM rows and
    // then address that shortened DOM using absolute buffer indices.
    for (var i = 0; i < this.buf.length; i++) frag.appendChild(this._lineNode(this.buf[i]));
  }
  this.scrollEl.appendChild(frag);
  this._rendered = this.buf.length;
  this._staleTail = false;
  this._dirtyFrom = -1;
  this.stick();
  if (this.onAppend) this.onAppend(this);
};
AnsiTerm.prototype._lineText = function (line) {
  var s = '', chars = line.chars || [];
  for (var j = 0; j < chars.length; j++) if (chars[j] && chars[j].c) s += chars[j].c;
  return s;
};
AnsiTerm.prototype._filterRows = function () {
  var kw = this._filter.toLowerCase();
  var out = [];
  for (var i = 0; i < this.buf.length; i++) {
    var s = this._lineText(this.buf[i]);
    if (s.toLowerCase().indexOf(kw) >= 0) out.push(this._lineNode(this.buf[i]));
  }
  return out;
};
AnsiTerm.prototype.setLineFilter = function (kw) {
  var next = txt(kw);
  if (next === this._filter) return; // 避免抽屉重绘时整屏重建
  this._filter = next;
  this._filterMode = !!next;
  this._renderAll(false);
};
AnsiTerm.prototype.setAutoScroll = function (on) {
  this.autoScroll = !!on;
  if (on) this.followTail = true;
  this.stick();
};
AnsiTerm.prototype.clearView = function () {
  this.buf = [{ chars: [] }];
  this.col = 0;
  this.lines = 0;
  this._droppedLines = 0;
  this._screen = false;
  this._dirtyFrom = 0;
  clear(this.scrollEl);
  this._rendered = 0;
  this._staleTail = false;
  this._renderAll(false);
};
AnsiTerm.prototype.stats = function () {
  return { lines: this.buf.length, total: this.lines, dropped: this._droppedLines };
};
AnsiTerm.prototype.plainLines = function () {
  var out = [];
  for (var i = 0; i < this.buf.length; i++) {
    // 转义序列在 _parse 阶段已被消费，这里只做尾部空白清理，绝不删除正常文本
    out.push(this._lineText(this.buf[i]).replace(/\s+$/, ''));
  }
  return out;
};

/* ==========================================================================
 * 5. SSE 事件总线
 * ========================================================================== */

function setSse(text, cls) {
  state.sse = { state: text, cls: cls || '' };
  renderTerminals();
}

function startSSE() {
  if (state._es) return;
  var es = new EventSource('/api/events');
  state._es = es;
  es.onopen = function () { setSse('已连接 /api/events', 'ok'); };
  es.onerror = function () { setSse('事件流断开，浏览器将自动重连…', 'bad'); };
  es.onmessage = function (ev) {
    var msg = null;
    try { msg = JSON.parse(ev.data); } catch (e) { return; }
    if (!msg) return;
    bus.dispatch(msg);
  };
}

var saveScheduled = false;
function scheduleRender() {
  if (saveScheduled) return;
  saveScheduled = true;
  window.requestAnimationFrame(function () {
    saveScheduled = false;
    renderTopbar();
    // Telemetry arrives every second; preserve input focus and IME composition.
    var active = document.activeElement;
    if (!(active && active.closest && active.closest('#groups-body'))) renderGroups();
    if (!(active && active.closest && active.closest('#term-body'))) renderTerminals();
    renderMonitor(); renderDrawer();
  });
}

var bus = {
  dispatch: function (msg) {
    var t = msg.t;
    switch (t) {
      case 'hello':
        applySnapshot(msg.snapshot || {});
        toast('info', '事件流已连接，面板缓冲已重置');
        break;
      case 'out':
        this.onOut(msg.s || 'trial', msg.d || '', msg.n);
        break;
      case 'session':
        if (msg.s && msg.session) state.sessions[msg.s] = msg.session;
        if (msg.session && !msg.s) state.sessions[msg.session.id] = msg.session;
        setSse('事件流正常', 'ok');
        scheduleRender();
        break;
      case 'stage':
        if (msg.stage) { onStage(msg.stage); scheduleRender(); }
        break;
      case 'telemetry':
        state.telemetry = msg.telemetry || null;
        scheduleRender();
        break;
      case 'probe':
        state.probe = msg.probe || {};
        if (msg.telemetry) state.telemetry = msg.telemetry;
        scheduleRender();
        break;
      case 'alert':
        if (msg.alert) { state.alerts.unshift(msg.alert); if (state.alerts.length > ALERT_KEEP) state.alerts.pop(); renderMonitor(); }
        break;
      case 'timeline':
        if (msg.item) { state.timeline.push(msg.item); if (state.timeline.length > TL_KEEP) state.timeline.shift(); renderDrawer(); }
        break;
      case 'trial':
        state.trial = msg.trial || {};
        scheduleRender();
        break;
      case 'orchestration':
        state.orchestration = msg.orchestration || null;
        renderMonitor();
        break;
      case 'board':
        state.board = msg.board || state.board || { logs: [], preflight: null };
        renderMonitor(); renderDrawer();
        break;
      case 'report':
        state.report = msg.report || null;
        state.reportText = (msg.report && msg.report.markdown) || '';
        renderMonitor();
        break;
      case 'connection':
        mergeConnection(msg.connection);
        scheduleRender();
        break;
      case 'toast':
        toast(msg.level, msg.text);
        break;
      default:
        break;
    }
  },
  onOut: function (sid, data) {
    if (!data) return;
    var term = state.terms[sid];
    if (term) term.append(data);
    if (sid === 'trial' && state.logTerm) state.logTerm.append(data);
  }
};

function onStage(stage) {
  var prevName = state.stage ? state.stage.name : null;
  state.stage = stage;
  if (prevName !== stage.name) {
    if (stage.name === 'READY') beep('ok');
    else if (stage.name === 'FAILED') beep('error');
  }
  renderMonitor();
  scheduleRender();
}

function applySnapshot(snap) {
  state.snapshot = snap;
  if (snap.connection) state.connection = snap.connection;
  if (snap.profile) state.profile = snap.profile;
  state.groups = snap.groups || [];
  state.terminals = snap.terminals || [];
  state.sessions = snap.sessions || {};
  state.trial = snap.trial || {};
  state.stage = snap.stage || null;
  state.telemetry = snap.telemetry || null;
  state.probe = snap.probe || {};
  state.orchestration = snap.orchestration || null;
  state.alerts = (snap.alerts || []).slice(0, ALERT_KEEP);
  state.timeline = (snap.timeline || []).slice(0, TL_KEEP);
  state.board = snap.board || { logs: [], preflight: null };
  state.report = snap.report || null;
  state.reportText = (snap.report && snap.report.markdown) || '';
  renderCapabilities();

  // 选择最近的组（记忆优先）
  var ids = state.groups.map(function (g) { return g.id; });
  var pick = null;
  if (prefs.group_id && ids.indexOf(prefs.group_id) >= 0) pick = prefs.group_id;
  else if (snap.trial && snap.trial.group_id && ids.indexOf(snap.trial.group_id) >= 0) pick = snap.trial.group_id;
  else if (ids.length) pick = ids[0];
  state.selectedGroup = pick;

  initTermsFromSnapshot();
  scheduleRender();
}

/* hello / 重连时重建终端与标签，清空面板缓冲（旧缓冲按协议丢弃） */
function initTermsFromSnapshot() {
  state.logTerm = new AnsiTerm({ id: 'trial-log' });
  state.terms = { trial: new AnsiTerm({ id: 'trial' }) };
  state.terms.trial.autoScroll = !!prefs.autoscroll;
  state.terms.trial.onAppend = function () { /* 状态条由 render 刷新 */ };
  state.terms.trial.onTrim = function () { };

  var tabs = { _log: state.tabs._log || { filter: '' } };
  state.terminals.forEach(function (t) {
    var term = new AnsiTerm({ id: t.id });
    term.autoScroll = !!prefs.autoscroll;
    state.terms[t.id] = term;
    var prev = state.tabs[t.id] || {};
    tabs[t.id] = {
      input: prev.input || '',
      filter: prev.filter || '',
      history: prev.history || [],
      histIdx: -1,
      autoScroll: prev.autoScroll === undefined ? !!prefs.autoscroll : !!prev.autoScroll
    };
  });
  state.tabs = tabs;

  var active = null;
  if (state.activeTerm && state.terms[state.activeTerm]) active = state.activeTerm;
  else if (state.terminals.length) {
    var running = state.terminals.filter(function (t) {
      var s = state.sessions[t.id];
      return s && s.state === 'running';
    });
    active = (running.length ? running[0] : state.terminals[0]).id;
  }
  state.activeTerm = active;
}

/* ==========================================================================
 * 6. 渲染 · 顶栏
 * ========================================================================== */

var STAGE_LABELS = {
  IDLE: '空闲（未起飞流程）',
  STARTING: '启动中（节点拉起）',
  INITIALIZING: '初始化（等待飞控与建图）',
  MAPPING_READY: '建图就绪（等待应用就绪）',
  READY: '就绪（READY，可人工解锁）',
  IN_FLIGHT: '飞行中（人工接管）',
  DISARMED: '已锁定（未解锁）',
  STOPPED: '已停止',
  FAILED: '失败（需人工排查）',
  UNKNOWN: '未知'
};

/* 板端地址下拉：列出 memoir/现场部署记录里出现过的历史 SSH 地址 */
function renderHostSelect() {
  var sel = $('#host-select');
  if (!sel) return;
  var c = state.connection || {};
  var options = hostChoices();
  var current = c.host || '';
  var key = current + '|' + JSON.stringify(options);
  if (sel.getAttribute('data-key') !== key) {
    sel.setAttribute('data-key', key);
    clear(sel);
    options.forEach(function (o) {
      var opt = el('option', null, o.label);
      opt.value = o.value;
      sel.appendChild(opt);
    });
    sel.value = current;
  }
  sel.title = '板端 SSH 地址（历史地址来自现场部署记录与项目 memoir）。切换后记得点「连接」。';
}

function renderTopbar() {
  var c = state.connection || {};
  var chip = $('#conn-chip');
  var st = c.state || 'unknown';
  chip.className = 'chip chip-' + st;
  renderHostSelect();
  var hostEl = $('#conn-host');
  hostEl.textContent = (c.host || '未配置') + (c.port ? (':' + c.port) : '') + ' · ' + ({
    unknown: '未连接', checking: '检查中', ok: '正常', failed: '失败'
  }[st] || st);
  $('#conn-detail').textContent = c.detail ? ('· ' + c.detail) : '';

  var off = !connected();
  $('#offline-banner').classList.toggle('hidden', !off);
  ['#btn-preflight', '#btn-start-all', '#btn-stop-all', '#btn-report'].forEach(function (sel) {
    var b = $(sel); if (b) b.disabled = off;
  });

  var view = probeView();
  var tel = view.usable ? state.telemetry || {} : {};
  var tAt = (state.telemetry || {}).at || (state.telemetry || {}).t;
  $('#topbar-probe').textContent = 'master ' + (tel.master === true ? 'OK' : (tel.master === false ? '无' : '—'))
    + ' · 节点 ' + (tel.nodes ? tel.nodes.length : '—')
    + ' · 遥测 ' + hhmmss(tAt) + ' · ' + view.detail
    + (tel.probe && tel.probe.node ? (' · 探针 ' + (tel.probe.host || '') + (tel.probe.pid ? ('#' + tel.probe.pid) : '')) : '');
  var reconnect = $('#btn-probe-reconnect');
  if (reconnect) reconnect.disabled = !supportsCapability('probe_reconnect') || !connected() || !!(state.sessions.probe && ['running','starting'].indexOf(state.sessions.probe.state) >= 0);
}

function supportsCapability(name) {
  return !!(state.snapshot && state.snapshot.capabilities && state.snapshot.capabilities[name] === true);
}

function renderCapabilities() {
  [['#link-recording', 'recording'], ['#btn-probe-reconnect', 'probe_reconnect']].forEach(function (item) {
    var control = $(item[0]);
    if (control) control.style.display = supportsCapability(item[1]) ? '' : 'none';
  });
}

function probeView() {
  var tel = state.telemetry || {}, link = tel.probe_link || (state.probe || {}).link || {};
  var session = state.sessions.probe || {}, at = Number(tel.at || tel.t || 0);
  var age = at ? nowSec() - at : null;
  var fresh = age !== null && age >= 0 && age <= 3;
  var terminal = ['failed','exited'].indexOf(session.state) >= 0;
  var usable = fresh && tel.master === true && !terminal && session.exit_code !== 130 &&
    (!link.status || link.usable === true);
  var detail = link.detail || (usable ? '探针遥测新鲜' : '当前设备状态未观测');
  if (!fresh && link.status === 'live') detail = '探针遥测已过期；当前设备状态未观测';
  if (link.retrying) detail += ' · 仅probe退避恢复（第' + link.retry_count + '次）';
  return {usable:usable, age:age, detail:detail};
}

function servoInitBlocked() {
  var session = state.sessions.servo || {}, tel = state.telemetry || {};
  return ['running','starting'].indexOf(session.state) >= 0 ||
    (probeView().usable && !!(tel.services || {})['/legacy/Servo_raw']);
}

/* ==========================================================================
 * 7. 渲染 · 任务组
 * ========================================================================== */

function releaseBadge(release) {
  if (release === 'real') return { cls: 'badge badge-real', text: '实投' };
  if (release === 'mock') return { cls: 'badge badge-mock', text: '模拟投递' };
  return { cls: 'badge badge-none', text: '无投递' };
}

/* 依据 group 复原后端将要执行的完整命令。
 * 该逻辑与后端 wb_board.build_group_command / terminal_wrapped_command 一致，
 * 只用于“把命令给人看”，不参与任何下发。 */
/* 与后端 wb_board.REAL_RELEASE_FOLDERS 保持一致（只有这些模块目录有 start_real.sh） */
var REAL_RELEASE_FOLDERS = ['01_visual_interrupt', '02_high_view_revisit', '05_low_multi',
  '06_high_priority', '08_full_mission'];
function shQuote(s) { return "'" + String(s).replace(/'/g, "'\\''") + "'"; }
function shellArgQuote(s) {
  s = String(s);
  return s && /^[A-Za-z0-9_@%+=:,./-]+$/.test(s) ? s : "'" + s.replace(/'/g, "'\"'\"'") + "'";
}

function groupCommandBody(g, mode, realRelease, checkConfig, speed, options) {
  var c = state.connection || {};
  var folder = g.folder || '';
  var siteDir = c.site_dir || 'deployment/site_20260928';
  var siteConfig = (g.site_config || siteDir + '/test_area.yaml').replace(/\{site_dir\}/g, siteDir);
  var route = g.channel || 'module';
  var moduleBase = 'deployment/board_trials_4x4/' + folder;
  var extra = '';
  options = options || {};
  if (route === 'competition') {
    var field = options.competition_config == null ? g.site_config : options.competition_config;
    if (typeof field !== 'string' || !field.trim() || /[\r\n\x00]/.test(field) ||
        (mode === 'flight' && !checkConfig && !realRelease)) return {body:null,note:'独立正赛需要场地配置，flight仅支持实投'};
    for (var pair of [['motion-optimization',options.motion_optimization],['resume-survey',options.resume_survey],['obstacle-columns',options.obstacle_columns]]) {
      if (pair[1] != null) {
        if (['on','off'].indexOf(pair[1]) < 0) return {body:null,note:'开关仅支持on/off，省略继承YAML'};
        extra += ' --' + pair[0] + ' ' + pair[1];
      }
    }
    return {body:'bash ' + shellArgQuote(g.entry || 'deployment/competition/start.sh') + ' ' +
      (checkConfig ? 'preview' : mode) + ' --site-config ' + shellArgQuote(field.trim()) + extra + (checkConfig ? ' --check-config' : ''),
      note:'所选YAML为权威；继承不传覆盖，on/off显式覆盖。先配置检查核对有效参数；测试场地成功不代表10×10正赛。'};
  }
  if (g.channel === 'low_observation') {
    if (['hover','forward','square'].indexOf(g.profile) < 0 || realRelease || speed != null ||
        options.capture_lighting != null || options.motion_optimized || options.survey_pattern != null ||
        options.resume_survey != null || options.site_geometry != null || options.geometry_revision != null || options.speed_profile != null) {
      return {body:null,note:'低空观察继承原 profile，不支持投递或专项参数'};
    }
    return {body:'bash deployment/low_hover_observation/start.sh ' + (checkConfig ? 'preview' : mode) + ' ' + g.profile,
      note:'preview 为离线配置展开（无 ROS）；flight 仅观察动作和诊断录包，飞手人工解锁并重新拨入 OFFBOARD'};
  }
  if (options.site_geometry !== undefined) {
    var plans=groupUI(g).geometryPlans || {};
    return plans[geometryPlanKey(g,mode,realRelease,checkConfig,speed,options)] ||
      {body:null,note:'手动坐标或参数尚未生成匹配命令，请点击“生成并预览坐标命令”'};
  }
  if (speed !== null && speed !== undefined) extra += ' --capture-speed ' + Number(speed).toFixed(1);
  if (options.capture_lighting) extra += ' --capture-lighting ' + options.capture_lighting;
  if (options.speed_profile != null) {
    if (folder !== '08_full_mission' || route !== 'module' || ['limited','competition'].indexOf(options.speed_profile) < 0)
      return {body:null,note:'08速度版本只支持limited/competition'};
    extra += ' --speed-profile ' + options.speed_profile;
  }
  if (options.motion_optimized) extra += ' --motion-optimized';
  if (options.survey_pattern) extra += ' --survey-pattern ' + options.survey_pattern;
  if (options.resume_survey) extra += ' --resume-survey ' + options.resume_survey;

  // check_config 优先于 channel：后端只在“启动 ROS 节点”时才区分现场/模块入口
  if (checkConfig) {
    return { body: 'bash ' + moduleBase + '/start.sh preview --site-config ' + shellArgQuote(siteConfig) + extra + ' --check-config',
             note: '只做配置检查：按模块入口展开参数，不启动任何 ROS 节点' };
  }
  if (route === 'site') {
    if (extra && folder !== '09_high_speed_capture') {
      return { body: null, note: '现场快捷入口不支持速度参数（后端会拒绝：现场快捷入口只有拍摄组支持附加参数）' };
    }
    return { body: 'bash ' + siteDir + '/start_test.sh ' + (g.key || '') + ' ' + mode + extra,
             note: '现场快捷入口：flight 对投递组自动走 start_real.sh（真实舵机），记忆组走 start.sh' };
  }
  if (realRelease && mode === 'flight') {
    var okFolder = REAL_RELEASE_FOLDERS.indexOf(folder) >= 0;
    return {
      body: okFolder ? ('bash ' + moduleBase + '/start_real.sh --site-config ' + shellArgQuote(siteConfig) + extra) : null,
      note: okFolder
        ? '模块实投入口：经释放许可代理调用现场 /legacy/Servo_raw（真实舵机）'
        : (folder + ' 没有 start_real.sh，不能走实投入口（后端会拒绝该组合）')
    };
  }
  return { body: 'bash ' + moduleBase + '/start.sh ' + mode + ' --site-config ' + shellArgQuote(siteConfig) + extra,
           note: '模块入口：默认模拟投递（mock 舵机），不接 PWM' };
}

function buildTrialCommand(g, mode, realRelease, checkConfig, speed, options) {
  var c = state.connection || {};
  var root = c.board_root || '<board_root>';
  var env = c.env_script || '<env_script>';
  var built = groupCommandBody(g, mode, realRelease, checkConfig, speed, options);
  if (!built.body) {
    return '（该参数组合后端会拒绝启动：' + built.note + '）';
  }
  return 'cd ' + shQuote(root) + ' && source ' + shQuote(root.replace(/\/$/, '') + '/' + env)
    + ' && ' + built.body;
}

function renderGroups() {
  var body = $('#groups-body');
  if (!body) return;
  var scrollTop = body.scrollTop;
  clear(body);
  var groups = state.groups || [];
  var hint = $('#groups-hint');
  var tSess = state.sessions.trial;
  hint.textContent = (tSess && (tSess.state === 'running' || tSess.state === 'starting'))
    ? ('trial 会话 ' + tSess.state + '：' + (state.trial && state.trial.name ? state.trial.name : ''))
    : 'trial 会话未运行';

  if (!groups.length) { body.appendChild(el('p', 'empty', '等待快照…（/api/snapshot 的 groups 为空）')); return; }

  appendGroupSection(body, '独立正赛（先实测并确认场地）', groups.filter(function (g) { return g.channel === 'competition'; }));
  appendGroupSection(body, '现场组号（1–6）', groups.filter(function (g) { return (g.section || g.channel) === 'site'; }));
  appendGroupSection(body, '专项模块与对照', groups.filter(function (g) { return (g.section || g.channel) === 'module'; }));
  appendGroupSection(body, '独立低空观察（顺序：悬停 → 前移 → 矩形）', groups.filter(function (g) { return g.section === 'observation'; }));

  var others = groups.filter(function (g) { return g.channel && ['site','module','low_observation','competition'].indexOf(g.channel) < 0; });
  if (others.length) appendGroupSection(body, '其他任务组', others);
  body.scrollTop = scrollTop;
}

function appendGroupSection(body, title, list) {
  if (!list.length) return;
  var h = el('div', 'grp-section-title');
  h.appendChild(el('span', null, title));
  h.appendChild(el('span', 'muted tiny', list.length + ' 组'));
  body.appendChild(h);
  list.forEach(function (g) { body.appendChild(groupCard(g)); });
}

function competitionEffectiveConfig(g,options,stage,trial) {
  var actual=(stage || {}).effective_config;
  var matches=trial && trial.group_id===g.id &&
    (trial.competition_config || g.site_config)===(options.competition_config || g.site_config) &&
    (trial.motion_optimization || null)===(options.motion_optimization || null) &&
    (trial.resume_survey || null)===(options.resume_survey || null) &&
    (trial.obstacle_columns || null)===(options.obstacle_columns || null) &&
    (trial.speed_profile || null)===(options.speed_profile || null) &&
    (trial.survey_pattern || null)===(options.survey_pattern || null) &&
    !!trial.motion_optimized===!!options.motion_optimized;
  return actual && matches ? effectiveConfigText(actual) : '当前所选配置的有效值尚未确认：先运行配置检查。复选框和选项只表示请求。';
}

function effectiveConfigText(actual) {
  var source=actual.source==='generated_runtime' ? '已生成配置' : '离线配置检查';
  var speeds=actual.planning ? '；规划上限=' + actual.planning.max_vel + 'm/s / ' + actual.planning.max_acc + 'm/s²' : '';
  if (actual.following) speeds += '；前视=' + actual.following.cruise_lead_m + '/' + actual.following.precision_lead_m + '/' + actual.following.boundary_lead_m + 'm';
  if (actual.initial_distances) speeds += '；初始化限幅/轨迹前视/规划启动=' + actual.initial_distances.controller_limit_m + '/' + actual.initial_distances.traj_target_dist_m + '/' + actual.initial_distances.planner_start_max_distance_m + 'm';
  if (actual.corridor) speeds += '；走廊快/慢=' + actual.corridor.open_lead_m + '/' + actual.corridor.door_lead_m + 'm';
  if (actual.drop_agl != null) speeds += '；投递FC AGL=' + actual.drop_agl + 'm';
  return source + speeds + '：运动优化=' + (actual.motion_optimization ? '开启' : '关闭') + '；高位续扫=' + (actual.resume_survey ? '开启' : '关闭') + (actual.generation_ready===false ? '；实测场地/墙面未确认，尚不可生成飞行配置' : '') + (actual.runtime_path ? '；' + actual.runtime_path : '');
}

function groupUI(g) {
  var U = state.tabs[g.id] || (state.tabs[g.id] = {});
  if (!U.mode) U.mode = 'preview';
  if (!U.release) U.release = g.release || 'none';
  if (g.speed_options && g.speed_options.length && U.speed == null) U.speed = g.speed_options[0];
  return U;
}

function trialBody(g, mode, checkConfig) {
  var U = groupUI(g);
  var body = { group_id: g.id, mode: checkConfig ? 'preview' : mode,
    real_release: !checkConfig && mode === 'flight' && U.release === 'real' };
  if (checkConfig) body.check_config = true;
  if (g.competition_options_supported) {
    if (U.motionOptimization) body.motion_optimization = U.motionOptimization;
    if (U.obstacleColumns) body.obstacle_columns = U.obstacleColumns;
    if (U.competitionConfig != null) body.competition_config = U.competitionConfig;
  }
  if (g.speed_profile_options && U.speedProfile) body.speed_profile = U.speedProfile;
  if (g.speed_options && g.speed_options.length && U.speed != null) body.capture_speed = U.speed;
  if (g.lighting_options && U.lighting) body.capture_lighting = U.lighting;
  if (g.motion_optimization_supported && U.motionOptimized) body.motion_optimized = true;
  if (g.survey_patterns && U.pattern) body.survey_pattern = U.pattern;
  if (g.resume_survey_supported && U.resume) body.resume_survey = U.resume;
  if ((g.needs_waypoints || g.survey_patterns || g.folder==='09_high_speed_capture') && U.geometryEnabled) {body.site_geometry=U.geometryValues || {};body.geometry_revision=U.geometryRevision || '';}
  if (!checkConfig) body.confirm = body.real_release ? '实投' : '启动试飞';
  var built = groupCommandBody(g, body.mode, body.real_release, !!body.check_config, body.capture_speed, body);
  if (built.body) body.expected_body = built.body;
  return body;
}

function geometryPlanKey(g, mode, realRelease, checkConfig, speed, options) {
  var c = state.connection || {};
  return JSON.stringify([g.id, c.host, c.board_root, c.site_dir, g.site_config,
    mode, !!realRelease, !!checkConfig, speed || null, options.capture_lighting || null,
    !!options.motion_optimized, options.survey_pattern || null, options.resume_survey || null, options.speed_profile || null,
    options.geometry_revision, options.site_geometry]);
}

function geometryFromDraft(U,g) {
  function row(text, count, label) {
    var raw = text == null ? '' : String(text).trim();
    var parts = raw ? raw.split(/[\s,，]+/) : [];
    if (!raw || (count && parts.length !== count)) {
      throw Error(label + '：' + (count ? '需要' + count + '个数，实际' + parts.length + '个' : '请填写数值') + '（逗号或空格分隔，不带括号）');
    }
    var values = parts.map(Number);
    if (values.some(function(v) { return !Number.isFinite(v); })) throw Error(label + '：请输入有限数值，不带括号或单位');
    return values;
  }
  var result = {}, points = [];
  if (!g || g.needs_waypoints) {
    String(U.geometryPoints || '').split(/\r?\n/).forEach(function(line,index) {
      if (!line.trim()) return;
      var p = row(line,3,'走廊航点第' + (index+1) + '行');
      points.push({x:p[0],y:p[1],agl:p[2]});
    });
    if (points.length < 2) throw Error('至少填写两个走廊航点，每行x, y, agl');
    result.corridor_waypoints=points;result.landing_xy=row(U.geometryLanding,2,'H中心');
  }
  if (U.surveyEnabled) {
    function value(key,label) { return row(U[key],1,label)[0]; }
    result.survey_plan={bounds:row(U.surveyBounds,4,'搜索区范围'),inset:value('surveyInset','航线内收距离'),pattern:U.pattern || 'rectangle',camera_agl:value('surveyCameraHeight','镜头离地高度'),fc_to_camera_z:value('surveyOffset','镜头相对FC的Z偏移'),fov_x_deg:value('surveyFovX','任务X方向视场角'),fov_y_deg:value('surveyFovY','任务Y方向视场角')};
  }
  if ((!g || g.needs_waypoints) && String(U.geometryWalls || '').trim()) {
    var entry=Number(U.geometryEntry);
    if (!Number.isInteger(entry) || entry<1 || entry>points.length) throw Error('入口序号从1开始且不能超过航点数');
    result.corridor_geometry={wall_axis:Number(U.geometryAxis || 0),wall_coordinates:row(U.geometryWalls,null,'墙面位置'),entry_waypoints:entry};
  }
  if (String(U.geometryArea || '').trim()) {
    try { result.flight_area=JSON.parse(U.geometryArea); }
    catch(e) { throw Error('flight_area：JSON格式错误，请使用完整JSON对象'); }
  }
  return result;
}

function renderSurveyCoverage(box,p) {
  box.replaceChildren();
  box.appendChild(el('p','tiny','镜头 '+p.camera_agl.toFixed(2)+'m → FC '+p.high_agl.toFixed(2)+'m；单帧视场X×Y '+p.footprint_xy.map(function(v){return v.toFixed(2);}).join('×')+'m；理想覆盖 '+p.covered_m2.toFixed(2)+' / '+p.area_m2.toFixed(2)+'m² ('+p.coverage_percent.toFixed(1)+'%)；航长 '+p.route_length_m.toFixed(1)+'m'));
  var ns='http://www.w3.org/2000/svg',svg=document.createElementNS(ns,'svg'),b=p.bounds,pad=.25,scale=Math.max(b[1]-b[0],b[3]-b[2]);
  svg.setAttribute('viewBox',[b[0]-pad,-b[3]-pad,b[1]-b[0]+2*pad,b[3]-b[2]+2*pad].join(' '));svg.setAttribute('role','img');svg.setAttribute('aria-label','搜索航线与理想视场覆盖');svg.style.width='100%';svg.style.maxHeight='300px';
  function rect(r,fill){var e=document.createElementNS(ns,'rect');[['x',r[0]],['y',-r[3]],['width',r[1]-r[0]],['height',r[3]-r[2]],['fill',fill]].forEach(function(v){e.setAttribute(v[0],v[1]);});svg.appendChild(e);}
  rect(b,'#2b4560');p.footprints.forEach(function(r){rect(r,'#346f96');});p.blind_rectangles.forEach(function(r){rect(r,'#a66b27');});
  var line=document.createElementNS(ns,'polyline');line.setAttribute('points',p.route.map(function(v){return v[0]+','+(-v[1]);}).join(' '));line.setAttribute('fill','none');line.setAttribute('stroke','#f3f7ff');line.setAttribute('stroke-width',scale*.008);svg.appendChild(line);
  var start=document.createElementNS(ns,'circle');start.setAttribute('cx',p.route[0][0]);start.setAttribute('cy',-p.route[0][1]);start.setAttribute('r',scale*.018);start.setAttribute('fill','#64ef9b');svg.appendChild(start);box.appendChild(svg);
  box.appendChild(el('div','tiny muted','白线=设计航线，绿点=起点，蓝色=理想覆盖，橙色=盲区；图中向右为+X，向上为+Y。'));
  p.warnings.forEach(function(w){box.appendChild(el('div','tiny muted',w));});
}

function geometryEditor(g,U,onChange) {
  var box=el('div','geometry-editor');
  var label=el('label','chk'), enabled=el('input');enabled.type='checkbox';enabled.checked=!!U.geometryEnabled;
  enabled.setAttribute('data-option','geometryEnabled');
  label.appendChild(enabled);label.appendChild(el('span',null,'使用实测坐标／自动搜索航线（保留原现场YAML）'));box.appendChild(label);
  enabled.addEventListener('change',function(){U.geometryEnabled=enabled.checked;U.geometryPlans={};renderGroups();});
  if (!U.geometryEnabled) return box;
  box.appendChild(el('div','tiny muted','单位m；相对起飞点：+X朝场内、+Y向左。走廊点agl为FC中心离地高度；自动搜索填写镜头离地高度，再按安装外参换算FC高度。'));
  function input(key,title,placeholder,multiline) {
    var wrap=el('label','geometry-field');wrap.appendChild(el('span',null,title));
    var field=el(multiline?'textarea':'input');field.value=U[key] || '';field.placeholder=placeholder;
    if(multiline)field.rows=key==='geometryPoints'?5:3;
    field.setAttribute('data-geometry',key);
    field.addEventListener('input',function(){U[key]=field.value;U.geometryPlans={};U.geometryRevision=null;U.surveyPreview=null;coverage.replaceChildren();status.textContent='坐标已修改，请重新生成命令';onChange();});
    wrap.appendChild(field);box.appendChild(wrap);return field;
  }
  var coverage=el('div','survey-coverage');
  if (g.survey_patterns || g.folder==='09_high_speed_capture') {
    var surveyLabel=el('label','chk'),survey=el('input');survey.type='checkbox';survey.checked=!!U.surveyEnabled;survey.setAttribute('data-option','surveyEnabled');
    surveyLabel.appendChild(survey);surveyLabel.appendChild(el('span',null,'按搜索区自动生成，并计算理想FOV覆盖'));box.appendChild(surveyLabel);
    survey.addEventListener('change',function(){U.surveyEnabled=survey.checked;U.geometryPlans={};U.surveyPreview=null;U.geometryRevision=null;renderGroups();});
    if(U.surveyEnabled) {
      var shape=el('select');shape.setAttribute('data-geometry','surveyPattern');
      [['rectangle','矩形'],['snake2','两条扫描线'],['snake3','三条扫描线']].forEach(function(v){var o=el('option',null,v[1]);o.value=v[0];shape.appendChild(o);});shape.value=U.pattern || 'rectangle';
      shape.addEventListener('change',function(){U.pattern=shape.value;U.geometryPlans={};U.surveyPreview=null;U.geometryRevision=null;renderGroups();});box.appendChild(shape);
      input('surveyBounds','搜索区：xmin,xmax,ymin,ymax（不含走廊）','填写实测搜索矩形',false);
      input('surveyInset','航线相对搜索边界内收（m）','填写机体中心内收距离',false);
      input('surveyCameraHeight','镜头实际离地高度（m）','不是FC高度',false);
      input('surveyOffset','镜头相对FC的Z偏移（m，向上为正）','核对known_rig；镜头在FC下方应为负',false);
      input('surveyFovX','任务X方向有效视场角（度）','根据实测覆盖或标定填写',false);
      input('surveyFovY','任务Y方向有效视场角（度）','按相机安装方向核对',false);
      box.appendChild(el('div','tiny muted','FC高度=镜头高度−安装Z偏移；例如偏移−0.16时镜头2m需要FC 2.16m，原限高不会自动放宽。FOV可由2×atan(实测宽度÷2÷镜头高度)换算为度。'));
    }
  }
  if (g.needs_waypoints) {
  input('geometryPoints','有序走廊航点：每行 x, y, agl','填写现场实测值，每行三个数，不含H观察爬升点',true);
  input('geometryLanding','H中心：x, y','填写终点H中心',false);
  var detail=el('div','tiny muted','可选：填写墙面几何后，现有运动优化才会尝试合并直线中继点/入口升降。坐标点本身不会自动启用这些条件。');box.appendChild(detail);
  var axis=el('select');[['0','墙面常量轴X'],['1','墙面常量轴Y']].forEach(function(item){var o=el('option',null,item[1]);o.value=item[0];axis.appendChild(o);});axis.value=U.geometryAxis || '0';
  axis.addEventListener('change',function(){U.geometryAxis=axis.value;U.geometryPlans={};U.geometryRevision=null;onChange();});box.appendChild(axis);
  input('geometryWalls','墙面位置（沿所选轴，逗号分隔；可留空）','实测墙平面位置',false);
  input('geometryEntry','低入口在有序航点中的序号（从1开始）','填写对应序号',false);
  }
  input('geometryArea','可选flight_area坐标覆盖（JSON对象；留空继承现场范围）','可填写center_bounds/target_bounds/search_bounds/staging_xy/survey_xy/map_size',true);
  box.appendChild(el('div','tiny muted','不扩大原场地时无需填写flight_area。扩大时需同时核对搜索区、规划地图尺寸和航线。所有点仍需通过原工程校验。'));
  box.appendChild(coverage);
  if(U.surveyPreview && U.surveyPreview.pattern===(U.pattern || 'rectangle'))renderSurveyCoverage(coverage,U.surveyPreview);
  var apply=el('button','btn btn-sm','生成并预览坐标命令（不连接飞机）');box.appendChild(apply);
  var status=el('div','tiny muted',U.geometryPlans && Object.keys(U.geometryPlans).length?'命令已生成；点击配置检查在板端核对合并结果。':'草稿尚未生成命令。');box.appendChild(status);
  apply.addEventListener('click',function(){
    try { U.geometryValues=geometryFromDraft(U,g); } catch(e) {status.textContent=e.message;return;}
    U.geometryRevision=Date.now().toString(36);U.geometryPlans={};apply.disabled=true;status.textContent='正在本机生成命令…';
    var revision=U.geometryRevision;
    var requests=[trialBody(g,'preview',true),trialBody(g,'preview',false),trialBody(g,'flight',false)];
    var coverageRequest=U.geometryValues.survey_plan ? fetch('/api/survey/plan',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(U.geometryValues.survey_plan)}).then(function(r){return r.json().then(function(v){if(!r.ok || !v.ok)throw Error(v.error || '覆盖计算失败');return v.plan;});}) : Promise.resolve(null);
    Promise.all(requests.map(function(body){
      delete body.confirm;delete body.expected_body;
      return fetch('/api/trial/command',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)})
        .then(function(r){return r.json().then(function(v){if(!r.ok || !v.ok)throw Error(v.error || '命令生成失败');return v;});})
        .then(function(plan){return [geometryPlanKey(g,body.mode,body.real_release,!!body.check_config,body.capture_speed,body),plan];});
    }).concat([coverageRequest])).then(function(plans){
      if(U.geometryRevision!==revision)return;
      U.surveyPreview=plans.pop();
      plans.forEach(function(pair){U.geometryPlans[pair[0]]=pair[1];});renderGroups();
    }).catch(function(e){status.textContent=e.message;U.geometryPlans={};}).finally(function(){apply.disabled=false;onChange();});
  });
  return box;
}
function groupCard(g) {
  var U = groupUI(g);
  var sel = state.selectedGroup === g.id;
  var card = el('div', 'grp-card' + (sel ? ' sel' : ''));

  // 标题
  var top = el('div', 'grp-top');
  top.appendChild(el('span', 'badge badge-info', g.key || g.id));
  top.appendChild(el('span', 'grp-title', g.name || g.id));
  var selectBtn = el('button', 'btn btn-sm grp-select', sel ? '已选择' : '选择此组');
  selectBtn.disabled = sel;
  selectBtn.title = '选择要查看和操作的任务组；不会启动任务或替换正在运行的试飞';
  selectBtn.addEventListener('click', selectGroup);
  top.appendChild(selectBtn);
  card.appendChild(top);
  if (g.plan) card.appendChild(el('div', 'grp-plan', 'plan：' + g.plan));
  if (g.ending) card.appendChild(el('div', 'grp-ending', 'ending：' + g.ending));

  // 徽标
  var badges = el('div', 'badges');
  var rb = releaseBadge(U.release);
  badges.appendChild(el('span', rb.cls, rb.text));
  if (U.release === 'real') badges.appendChild(el('span', 'badge', '实投需舵机'));
  if (g.needs_waypoints) badges.appendChild(el('span', 'badge', '需实测航点'));
  if (g.manual_mission_start) badges.appendChild(el('span', 'badge', '手动启动任务'));
  card.appendChild(badges);

  var ops = el('div', 'grp-ops');
  function choiceRow(label, field, choices) {
    var row = el('div', 'grp-row');
    row.appendChild(el('span', 'lbl', label));
    var select = el('select', 'grp-choice');
    select.setAttribute('data-option', field);
    choices.forEach(function (choice) {
      var option = el('option', null, choice[1]); option.value = choice[0];
      select.appendChild(option);
    });
    select.value = U[field] || '';
    select.addEventListener('change', function () {
      U[field] = select.value;
      if (field === 'release') { U.realConfirm = ''; U.armedOk = false; }
      renderGroups();
    });
    row.appendChild(select); ops.appendChild(row);
  }
  if (g.release_options && g.release_options.length) {
    choiceRow('投递', 'release', g.release_options.map(function (v) { return [v, v === 'real' ? '真实投递 · 需双重确认' : '模拟投递 · mock']; }));
  }
  if (g.competition_options_supported) {
    var presetRow=el('div','grp-row');presetRow.appendChild(el('span','lbl','配置来源'));
    var preset=el('select','grp-choice');preset.setAttribute('data-option','competitionPreset');
    var custom=el('option',null,'自定义路径（在下方填写）');custom.value='';preset.appendChild(custom);
    (g.competition_config_options || []).forEach(function(item){var option=el('option',null,item.label);option.value=item.path;preset.appendChild(option);});
    preset.value=U.competitionConfig == null ? g.site_config : U.competitionConfig;
    preset.addEventListener('change',function(){if(preset.value){U.competitionConfig=preset.value;U.armedOk=false;U.realConfirm='';renderGroups();}});
    presetRow.appendChild(preset);ops.appendChild(presetRow);
    var configRow = el('div','grp-row');
    configRow.appendChild(el('span','lbl','场地YAML'));
    var field = el('input'); field.type='text'; field.setAttribute('data-option','competitionConfig');
    field.value = U.competitionConfig == null ? g.site_config : U.competitionConfig;
    field.addEventListener('change',function(){U.competitionConfig=field.value;renderGroups();});
    configRow.appendChild(field);ops.appendChild(configRow);
    if (U.competitionConfig === 'deployment/competition/field_20261007_validated.yaml') {
      ops.appendChild(el('div','warn-line','当前选择测试场地复现：不是正赛默认参数，不能宣称10×10比赛验收；实际高度/速度以前述文件的配置检查为准。'));
    }
    choiceRow('运动优化','motionOptimization',[['','继承所选YAML'],['on','显式开启'],['off','显式关闭']]);
    ops.appendChild(el('div','tiny muted','模板默认开启运动优化与高位续扫；两项独立。继承的实际值由所选YAML和配置检查决定。'));
    choiceRow('障碍柱','obstacleColumns',[['','继承所选YAML'],['on','显式开启'],['off','显式关闭']]);
    ops.appendChild(el('div','warn-line','必填正赛实测场地与确认项；默认模板未确认。运动优化/障碍柱继承时不发送覆盖参数，当前有效值以配置检查输出为准；关闭障碍柱移除配置柱，不会关闭真实点云避障。'));
  }
  if (g.speed_profile_options) {
    choiceRow('08速度版本','speedProfile',g.speed_profile_options.map(function(item){return [item.value,item.label];}));
    ops.appendChild(el('div','warn-line','沿用2026-10-07最后221730现场几何与FC高2m；不是10×10正赛场地。1.2/1.0为配置上限，尚未实飞验收。距离单位m，不是实测速度。'));
    ops.appendChild(el('div','tiny muted',competitionEffectiveConfig(g,trialBody(g,'preview',true),state.stage,state.trial)));
  }
  if (g.motion_optimization_supported) {
    var motionRow = el('div', 'grp-row');
    var motionLab = el('label', 'chk');
    var motion = el('input'); motion.type = 'checkbox'; motion.checked = !!U.motionOptimized;
    motion.setAttribute('data-option', 'motionOptimized');
    motion.addEventListener('change', function () { U.motionOptimized = motion.checked; renderGroups(); });
    motionLab.appendChild(motion); motionLab.appendChild(el('span', null, '显式开启运动优化（不勾选继承配置）'));
    motionRow.appendChild(motionLab); ops.appendChild(motionRow);
  }
  if (g.survey_patterns && g.survey_patterns.length) {
    choiceRow('搜索路线', 'pattern', [['', '继承现场配置']].concat(g.survey_patterns.map(function (v) { return [v, v]; })));
  }
  if (g.resume_survey_supported) {
    choiceRow('高位续扫', 'resume', [['', '继承现场配置'], ['on', '开启续扫'], ['off', '关闭续扫']]);
  }
  if (g.lighting_options && g.lighting_options.length) {
    choiceRow('光照标签', 'lighting', [['', '继承现场配置']].concat(g.lighting_options.map(function (v) {
      return [v, { normal: '正常 normal', dim: '较暗 dim', unspecified: '未指定 unspecified' }[v] || v];
    })));
    ops.appendChild(el('div', 'tiny muted', '光照标签用于采集记录，不改变相机曝光。'));
  }

  if (g.competition_options_supported) {
    var selectedOptions=trialBody(g,'preview',true);
    ops.appendChild(el('div','tiny muted','所选配置：' + (selectedOptions.competition_config || g.site_config) + '；运动优化=' + (selectedOptions.motion_optimization || '继承') + '；高位续扫=' + (selectedOptions.resume_survey || '继承')));
    ops.appendChild(el('div','tiny muted',competitionEffectiveConfig(g,selectedOptions,state.stage,state.trial)));
  }

  // 模式
  var modeRow = el('div', 'grp-row');
  modeRow.appendChild(el('span', 'lbl', '模式'));
  [['preview', '预览 preview'], ['flight', '飞行 flight']].forEach(function (m) {
    var b = el('button', 'btn btn-sm' + (U.mode === m[0] ? ' active' : ''), m[1]);
    b.addEventListener('click', function () { U.mode = m[0]; renderGroups(); });
    modeRow.appendChild(b);
  });
  ops.appendChild(modeRow);

  // flight 二次确认
  if (U.mode === 'flight') {
    var armRow = el('div', 'grp-row');
    var lab = el('label', 'chk');
    var cb = el('input'); cb.type = 'checkbox'; cb.checked = !!U.armedOk;
    cb.addEventListener('change', function () { U.armedOk = cb.checked; renderGroups(); });
    lab.appendChild(cb);
    lab.appendChild(el('span', null, '已确认：飞机回到起飞点、未解锁、机头朝场内'));
    armRow.appendChild(lab);
    ops.appendChild(armRow);
  }

  // 实投确认词
  if (U.release === 'real' && U.mode === 'flight') {
    var realRow = el('div', 'grp-row');
    realRow.appendChild(el('span', 'lbl', '实投确认词'));
    var inp = el('input'); inp.type = 'text'; inp.value = U.realConfirm || '';
    inp.placeholder = '输入 实投'; inp.size = 10;
    inp.addEventListener('input', function () { U.realConfirm = inp.value; updateFlightBtn(); });
    realRow.appendChild(inp);
    ops.appendChild(realRow);
    ops.appendChild(el('div', 'err-line', '真实投递：先输入「实投」，再确认完整命令后才会下发。'));
  }

  // 速度
  if (g.speed_options && g.speed_options.length) {
    var spRow = el('div', 'grp-row');
    spRow.appendChild(el('span', 'lbl', '速度'));
    g.speed_options.forEach(function (s) {
      var b = el('button', 'btn btn-sm' + (U.speed === s ? ' active' : ''), String(s));
      b.addEventListener('click', function () { U.speed = s; renderGroups(); });
      spRow.appendChild(b);
    });
    ops.appendChild(spRow);
    ops.appendChild(el('div', 'warn-line', '速度选项属于「仅采集不投递」路径：只跑视觉采集，不下发投递。'));
  }

  if (g.channel !== 'competition' && (g.needs_waypoints || g.survey_patterns || g.folder==='09_high_speed_capture')) {
    ops.appendChild(geometryEditor(g,U,function(){if(flightBtn)updateFlightBtn();}));
    if(g.needs_waypoints)ops.appendChild(el('div','warn-line','走廊/H航点仍需实测；自动搜索不会代填门口。'));
  }

  // 按钮
  var btnRow = el('div', 'grp-row');
  var previewBtn = el('button', 'btn btn-sm', '预览（preview）');
  previewBtn.disabled = !connected() || tSessRunning() || !!state.trialPending;
  previewBtn.addEventListener('click', function () { startTrial(g, 'preview'); });
  var checkBtn = el('button', 'btn btn-sm', '配置检查（不启动节点）');
  checkBtn.disabled = previewBtn.disabled;
  checkBtn.title = '独立展开配置：mode=preview, check_config=true，不经过飞行或实投确认';
  checkBtn.addEventListener('click', function () { startTrial(g, 'preview', true); });

  var flightBtn = el('button', 'btn btn-sm btn-danger', '飞行（flight）');
  btnRow.appendChild(previewBtn);
  btnRow.appendChild(flightBtn);
  btnRow.appendChild(checkBtn);
  if (tSessRunning()) {
    var stopBtn = el('button', 'btn btn-sm', '停止（Ctrl+C）');
    stopBtn.title = 'POST /api/trial/stop {} → 向 trial 会话发送 INT（等同 Ctrl+C），不做任何自动降落';
    stopBtn.addEventListener('click', function () { doTrialStop(); });
    btnRow.appendChild(stopBtn);
  }
  ops.appendChild(btnRow);

  // 手动启动任务（仅 manual_mission_start 的组）：与右栏同一动作、同一确认
  if (g.manual_mission_start) {
    var msRow = el('div', 'grp-row');
    var msBtn = el('button', 'btn btn-sm', '启动任务（仅 READY 后）');
    var stageName = (state.stage || {}).name || 'IDLE';
    var stOk = MISSION_STAGES.indexOf(stageName) >= 0;
    msBtn.disabled = !connected() || !stOk;
    msBtn.title = !connected() ? '未连接板端，先连接'
      : (!stOk ? ('当前阶段是 ' + stageName + '，只有 READY 之后才允许启动任务（后端会返回 400）')
        : 'POST /api/action/mission_start {confirm:"启动任务"} → rosservice call /navigation/start_mission "{}"');
    msBtn.addEventListener('click', doMissionStart);
    msRow.appendChild(msBtn);
    msRow.appendChild(el('span', 'tiny muted', '该组 manual_mission_start=true：只在下发一次，不能为催促重复调用'));
    ops.appendChild(msRow);
    if (state.lastMissionOutput) {
      ops.appendChild(el('div', 'tiny muted', '最近一次 rosservice 返回（必须 success: true）'));
      ops.appendChild(el('pre', 'cmd-pre', state.lastMissionOutput));
    }
  }

  var reqEl = el('div', 'tiny muted');
  ops.appendChild(reqEl);
  var cmdNote = el('div', 'cmd-note', 'POST /api/trial/start');
  ops.appendChild(cmdNote);

  // flight 按钮启用条件
  function updateFlightBtn() {
    var needReal = (U.release === 'real');
    var realOk = !needReal || (U.realConfirm || '').trim() === '实投';
    var readyBody=trialBody(g,'flight',false);
    var geometryReady=(!U.geometryEnabled && g.channel !== 'competition') || !!readyBody.expected_body;
    flightBtn.disabled = !connected() || tSessRunning() || !!state.trialPending || !U.armedOk || !realOk || !geometryReady;
    flightBtn.title = !connected() ? '未连接板端'
      : (tSessRunning() || state.trialPending ? '专项正在运行或准备下发，先停止当前任务'
        : (!U.armedOk ? '请先勾选飞行前确认（已回到起飞点、未解锁、机头朝场内）'
          : (!realOk ? '真实投递需输入确认词「实投」' : 'POST /api/trial/start {group_id,mode:"flight",...}')));
    reqEl.textContent = '请求体：' + JSON.stringify(trialBody(g, U.mode, false));
  }
  flightBtn.addEventListener('click', function () { startTrial(g, 'flight'); });
  updateFlightBtn();

  // 命令预览（始终可见）
  var request = trialBody(g, U.mode, false);
  var cmd = buildTrialCommand(g, U.mode, request.real_release, false, request.capture_speed, request);
  var det = el('details', 'cmd-box');
  if (U.cmdOpen) det.open = true;
  det.addEventListener('toggle', function () { U.cmdOpen = det.open; });
  var sum = el('summary');
  sum.appendChild(el('span', null, '命令预览（将要执行的完整命令）'));
  det.appendChild(sum);
  var cb2 = el('div', 'cmd-box-body');
  var pre = el('pre', 'cmd-pre', cmd);
  cb2.appendChild(pre);
  cb2.appendChild(el('div', 'tiny muted wrap-any', '入口说明：' + groupCommandBody(g, U.mode, request.real_release, false, request.capture_speed, request).note));
  cb2.appendChild(el('div', 'tiny muted', '独立配置检查命令（不启动节点）：'));
  var checkRequest = trialBody(g, 'preview', true);
  cb2.appendChild(el('pre', 'cmd-pre', buildTrialCommand(g, 'preview', false, true, checkRequest.capture_speed, checkRequest)));
  var row = el('div', 'grp-row');
  row.appendChild(copyBtn(function () { return cmd; }, '试飞命令'));
  if (g.command) {
    var b2 = el('button', 'btn btn-sm btn-ghost', '后端登记的原始命令');
    b2.addEventListener('click', function () { openCommandModal('后端登记的原始命令', g.command, g.name); });
    row.appendChild(b2);
  }
  cb2.appendChild(row);
  det.appendChild(cb2);
  ops.appendChild(det);

  if (g.notes) ops.appendChild(el('div', 'tiny muted wrap-any', '备注：' + g.notes));

  card.appendChild(ops);
  card.addEventListener('click', function (ev) {
    if (ev.target && ev.target.closest('button,input,select,textarea,label,a,details')) return;
    selectGroup();
  });
  function selectGroup() {
    if (state.selectedGroup === g.id) return;
    state.selectedGroup = g.id;
    prefs.group_id = g.id;
    savePrefs();
    renderGroups();
    renderMonitor();
  }
  return card;
}

function tSessRunning() {
  var s = state.sessions.trial;
  return !!(s && (s.state === 'running' || s.state === 'starting'));
}

/* ==========================================================================
 * 8. 渲染 · 终端
 * ========================================================================== */

function termStatus(tid) {
  var s = state.sessions[tid] || {};
  var st = s.state || 'idle';
  var cls = (st === 'running' || st === 'starting' || st === 'exited' || st === 'failed') ? st : 'idle';
  var label = { idle: '未启动', starting: '启动中', running: '运行中', exited: '已退出', failed: '失败' }[st] || st;
  // 5a is a one-shot initialization, whereas 5b is a persistent service.
  if (tid === 'servo_init') {
    if (st === 'exited' && s.exit_code === 0) { cls = 'succeeded'; label = '初始化成功'; }
    else if (st === 'failed' || (st === 'exited' && typeof s.exit_code === 'number')) {
      cls = 'failed'; label = '初始化失败';
    } else if (st === 'running' || st === 'starting') { cls = 'starting'; label = '初始化中'; }
    else if (st === 'idle') label = '未初始化';
    else if (st === 'exited') label = '已退出（初始化结果未知）';
  }
  return { state: st, cls: cls, label: label, sess: s };
}

function renderTerminals() {
  var tabsBox = $('#term-tabs');
  var bodyBox = $('#term-body');
  if (!tabsBox || !bodyBox) return;

  var list = state.terminals || [];
  $('#terminals-hint').textContent = state.sse.state ? state.sse.state : '';

  // 标签栏
  clear(tabsBox);
  list.forEach(function (t) {
    var st = termStatus(t.id);
    var b = el('button', 'ttab' + (state.activeTerm === t.id ? ' active' : ''));
    b.appendChild(el('span', 'dot ' + st.cls));
    b.appendChild(el('span', null, t.title || t.id));
    b.title = (t.desc || '') + '\n命令：' + (t.command || '');
    b.addEventListener('click', function () { state.activeTerm = t.id; renderTerminals(); });
    tabsBox.appendChild(b);
  });
  if (!list.length) tabsBox.appendChild(el('span', 'tiny muted', '等待快照…'));

  // Preserve terminal history position when telemetry redraws its controls.
  Object.keys(state.terms).forEach(function (id) { state.terms[id].saveScroll(); });
  // 内容区
  clear(bodyBox);
  var t = null;
  for (var i = 0; i < list.length; i++) if (list[i].id === state.activeTerm) t = list[i];
  if (!t && list.length) { t = list[0]; state.activeTerm = t.id; }
  if (!t) { bodyBox.appendChild(el('p', 'empty', '等待快照…（/api/snapshot 的 terminals 为空）')); return; }

  var st = termStatus(t.id);
  var term = state.terms[t.id];
  if (!term) return;

  // 工具条
  var tool = el('div', 'term-tool');
  var startBtn = el('button', 'btn btn-sm btn-primary', '启动');
  startBtn.disabled = !connected() || t.id === 'trial' || st.state === 'running' || st.state === 'starting';
  if (t.id === 'servo_init' && servoInitBlocked()) startBtn.disabled = true;
  if (t.id === 'trial') startBtn.textContent = '从任务组启动';
  startBtn.title = 'POST /api/session/open {id:"' + t.id + '"}' + (t.confirm ? '，需带 confirm:"确认"' : '') + '\n' + (t.command || '');
  if (t.id === 'servo_init' && servoInitBlocked()) startBtn.title = '5b服务仍在运行，禁止重复5a初始化互踩；按现场流程先退出旧服务，不自动杀进程。';
  startBtn.addEventListener('click', function () { openTerminal(t); });
  var stopBtn = el('button', 'btn btn-sm', '停止');
  stopBtn.disabled = st.state === 'idle';
  stopBtn.title = 'POST /api/session/close {id:"' + t.id + '"}';
  stopBtn.addEventListener('click', function () { act(api.sessionClose(t.id), '停止 ' + t.title); });
  var clearBtn = el('button', 'btn btn-sm', '清屏');
  clearBtn.title = 'POST /api/session/clear {id:"' + t.id + '"}（清板端缓冲与本地视图）';
  clearBtn.addEventListener('click', function () {
    term.clearView();
    if (t.id === 'trial' && state.logTerm) state.logTerm.clearView();
    act(api.sessionClear(t.id), '清屏 ' + t.title);
  });
  var ctrlCBtn = el('button', 'btn btn-sm', 'Ctrl+C');
  ctrlCBtn.title = 'POST /api/session/key {id:"' + t.id + '", key:"C-c"}';
  ctrlCBtn.addEventListener('click', function () { act(api.sessionKey(t.id, 'C-c'), '发送 Ctrl+C'); });
  var cmdBtn = el('button', 'btn btn-sm btn-ghost', '命令');
  cmdBtn.title = '查看该终端将要执行的完整命令';
  cmdBtn.addEventListener('click', function () {
    openCommandModal(t.title || t.id, t.command || '（后端未登记命令）', (t.desc || '') + (t.confirm ? ('\n启动确认：' + t.confirm) : ''));
  });

  tool.appendChild(startBtn); tool.appendChild(stopBtn); tool.appendChild(clearBtn);
  tool.appendChild(ctrlCBtn); tool.appendChild(cmdBtn);
  var sw = el('label', 'switch');
  var swi = el('input'); swi.type = 'checkbox'; swi.checked = !!(state.tabs[t.id] && state.tabs[t.id].autoScroll);
  swi.addEventListener('change', function () {
    if (state.tabs[t.id]) state.tabs[t.id].autoScroll = swi.checked;
    prefs.autoscroll = swi.checked;
    savePrefs();
    Object.keys(state.terms).forEach(function (k) { state.terms[k].setAutoScroll(swi.checked); });
    if (state.logTerm) state.logTerm.setAutoScroll(swi.checked);
    var sa = $('#sw-autoscroll'); if (sa) sa.checked = swi.checked;
  });
  sw.appendChild(swi); sw.appendChild(el('span', null, '自动滚动'));
  tool.appendChild(sw);

  var grow = el('span', 'grow');
  tool.appendChild(grow);
  var stats = term.stats();
  tool.appendChild(el('span', 'term-meta',
    '状态 ' + st.label
    + ' · 启动 ' + hhmmss(st.sess.started_at)
    + ' · 结束 ' + hhmmss(st.sess.ended_at)
    + ' · exit ' + (st.sess.exit_code === null || st.sess.exit_code === undefined ? '—' : st.sess.exit_code)
    + ' · 行 ' + stats.lines + (stats.dropped ? ('（丢 ' + stats.dropped + '）') : '')
    + (st.sess.chars ? (' · ' + st.sess.chars + ' 字符') : '')
  ));
  bodyBox.appendChild(tool);

  if (t.desc) bodyBox.appendChild(el('div', 'term-desc', t.desc));
  if (t.id === 'servo_init') bodyBox.appendChild(el('div', 'term-desc',
    '绿色仅表示软件初始化成功，不代表舵机物理动作反馈。'));
  if (t.confirm) bodyBox.appendChild(el('div', 'warn-line', '该终端启动前会弹确认框，确认文案：' + t.confirm));

  // 输出区（增量渲染，只追加）
  bodyBox.appendChild(term.scrollEl);
  term.stick();

  // Each view has its own filter; filtering the drawer must not hide flight output.
  var U = state.tabs[t.id] || (state.tabs[t.id] = { input: '', history: [], histIdx: -1 });
  var filterRow = el('div', 'term-line-filter');
  var filter = el('input'); filter.type = 'text'; filter.value = U.filter || '';
  filter.placeholder = '筛选本终端关键字（不区分大小写）';
  filter.addEventListener('input', function () { U.filter = filter.value; term.setLineFilter(U.filter); });
  var resetFilter = el('button', 'btn btn-sm', '清除筛选');
  resetFilter.addEventListener('click', function () { U.filter = ''; filter.value = ''; term.setLineFilter(''); });
  filterRow.appendChild(filter); filterRow.appendChild(resetFilter); bodyBox.appendChild(filterRow);
  term.setLineFilter(U.filter || '');

  // 输入行
  var inRow = el('div', 'term-input');
  var inp = el('input');
  inp.type = 'text';
  inp.placeholder = '输入后回车发送到该会话（交互式 shell / sudo 提示用）';
  inp.value = U.input || '';
  inp.spellcheck = false;
  inp.addEventListener('input', function () { U.input = inp.value; });
  inp.addEventListener('keydown', function (ev) {
    if (ev.key === 'Enter' && !ev.shiftKey) {
      var data = inp.value;
      if (data === '' || data === null || data === undefined) return;
      U.history = U.history || [];
      if (U.history[U.history.length - 1] !== data) U.history.push(data);
      U.histIdx = -1;
      inp.value = ''; U.input = '';
      act(api.sessionInput(t.id, data + '\n'), '发送到 ' + t.title);
    } else if (ev.key === 'ArrowUp') {
      var h = U.history || [];
      if (!h.length) return;
      ev.preventDefault();
      U.histIdx = (U.histIdx < 0) ? h.length - 1 : Math.max(0, U.histIdx - 1);
      inp.value = h[U.histIdx]; U.input = inp.value;
    } else if (ev.key === 'ArrowDown') {
      var h2 = U.history || [];
      if (!h2.length) return;
      ev.preventDefault();
      U.histIdx = (U.histIdx < 0) ? -1 : Math.min(h2.length - 1, U.histIdx + 1);
      inp.value = (U.histIdx < 0) ? '' : h2[U.histIdx];
      U.input = inp.value;
    }
  });
  var sendBtn = el('button', 'btn btn-sm', '发送');
  sendBtn.disabled = st.state === 'idle';
  sendBtn.addEventListener('click', function () {
    var d = inp.value; if (!d) return;
    inp.value = ''; U.input = '';
    act(api.sessionInput(t.id, d + '\n'), '发送到 ' + t.title);
  });
  inRow.appendChild(inp); inRow.appendChild(sendBtn);
  bodyBox.appendChild(inRow);
  bodyBox.appendChild(el('div', 'term-hint', '会话状态 ' + st.label + (st.state === 'running' ? '（ssh 会话保持打开）' : '') + ' · 发送内容是原始文本，直接进入远端 stdin'));
}

/* ==========================================================================
 * 9. 渲染 · 右栏监视
 * ========================================================================== */

var DETAIL_LABELS = {
  pose_samples: '位姿样本 pose_samples',
  camera_info: '相机内参 camera_info',
  image_seen: '图像到达 image_seen',
  compressed_fresh: '压缩图新鲜 compressed_fresh',
  distinct_clouds: '点云簇 distinct_clouds',
  clouds: '点云 clouds',
  odom: '里程计 odom',
  lidar: '雷达 lidar',
  detections: '检测 detections'
};
var TOPIC_WATCH = [
  '/mavros/state',
  '/camera/image_raw',
  '/camera/image_raw/compressed',
  '/camera/camera_info',
  '/livox/lidar',
  '/freedom/static_pointcloud',
  '/navigation/setpoint_mission',
  '/uav_vision/detections'
];

function kvRow(box, k, v, cls) {
  box.appendChild(el('div', 'k', k));
  box.appendChild(el('div', 'v ' + (cls || ''), v === undefined || v === null || v === '' ? '—' : String(v)));
}

function renderMonitor() {
  var body = $('#monitor-body');
  if (!body) return;
  var scrollTop = body.scrollTop;
  var oldReport = body.querySelector('.report-pre');
  var reportTop = oldReport ? oldReport.scrollTop : 0;
  var keepReport = state.reportText;
  clear(body);
  var rawTelemetry = state.telemetry || {}, view = probeView();
  var telemetryAt = Number(rawTelemetry.at || rawTelemetry.t || 0);
  var telemetryAge = telemetryAt ? nowSec() - telemetryAt : null;
  var telemetryFresh = view.usable;
  state._lastProbeUsable = telemetryFresh;
  var tel = telemetryFresh ? rawTelemetry : {};
  var unobserved = telemetryAt ? '未观测（遥测已过期）' : '未观测';

  var stage = state.stage || {};
  var runningGroup = (state.groups || []).find(function(g) { return g.id === state.trial.group_id; });
  var selectedGroup = (state.groups || []).find(function(g) { return g.id === state.selectedGroup; });
  var observationMode = (runningGroup || selectedGroup || {}).channel === 'low_observation';
  var sname = stage.name || 'IDLE';
  var card = el('div', 'stage-card c-' + (/^(IDLE|STARTING|INITIALIZING|MAPPING_READY|READY|IN_FLIGHT|DISARMED|STOPPED|FAILED)$/.test(sname) ? sname : 'unknown'));
  var head = el('div');
  var observationLabel = {INITIALIZING:'低空观察：等待地面参考 / 预发保持', READY:'低空观察 READY：等待人工解锁与重新拨入 OFFBOARD', IN_FLIGHT:'低空观察动作 / 保持 / 接管'};
  if (observationMode && stage.phase === 'HOLD_FOR_PILOT') observationLabel[sname] = '低空观察：READY已撤销，保持等待飞手接管';
  if (observationMode && stage.phase === 'FINISHED_HOVER') observationLabel[sname] = '低空观察路线结束：继续悬停，等待飞手落地';
  if (observationMode && stage.phase === 'TAKEN_OVER') observationLabel[sname] = '低空观察已接管：等待落地上锁与入口收尾';
  head.appendChild(el('div', 'stage-name', (observationMode && observationLabel[sname]) || stage.label || STAGE_LABELS[sname] || sname));
  head.appendChild(el('div', 'stage-sub', '阶段码 ' + sname + (stage.reason ? (' · 原因：' + stage.reason) : '')));
  card.appendChild(head);
  var elapsed = el('div', 'stage-elapsed', '已持续 ' + dur(nowSec() - (stage.since || nowSec())));
  elapsed.setAttribute('data-elapsed-since', String(stage.since || nowSec()));
  card.appendChild(elapsed);
  body.appendChild(card);

  // 启动任务按钮紧贴大状态卡
  body.appendChild(missionStartSection(stage));

  // 指标网格
  var sec = el('div', 'sec');
  sec.appendChild(secHead('关键状态'));
  var sb = el('div', 'sec-body');
  var kv = el('div', 'kv');
  kvRow(kv, 'armed', telemetryFresh ? (stage.armed === true ? '已解锁' : (stage.armed === false ? '未解锁' : '未观测')) : unobserved,
    telemetryFresh && stage.armed === false ? 'ok' : 'warn');
  kvRow(kv, 'ever_armed', stage.ever_armed ? '本次已解锁过' : '从未解锁');
  kvRow(kv, 'mode', stage.mode || '—');
  kvRow(kv, 'phase', stage.phase || '—');
  kvRow(kv, 'reason', stage.reason || '—');
  kvRow(kv, observationMode ? '观察状态' : '任务结果', observationMode ? (stage.phase || '等待入口输出；不采用常规Mission门槛') : stage.outcome === 'complete' ? '任务完成' : (stage.outcome === 'aborted' ? '任务中止' : '未确认'),
    stage.outcome === 'aborted' ? 'warn' : '');
  if (stage.pilot_action) kvRow(kv, '飞手操作', stage.pilot_action, 'warn');
  var align = stage.alignment || '—';
  if (!observationMode) kvRow(kv, '定位一致性', align + (stage.alignment_hint ? ('（' + stage.alignment_hint + '）') : ''),
    align === 'stable' ? 'ok' : (align === 'unstable' || align === 'bad' ? 'bad' : 'warn'));
  kvRow(kv, 'READY 时刻', stage.ready_at ? hhmmss(stage.ready_at) : '—');
  kvRow(kv, '开始时刻', stage.started_at ? hhmmss(stage.started_at) : '—');
  sb.appendChild(kv);

  var d = stage.detail || {};
  var keys = Object.keys(d);
  if (keys.length) {
    sb.appendChild(el('div', 'tiny muted', '关键量'));
    var kv2 = el('div', 'kv');
    keys.forEach(function (k) {
      var v = d[k];
      if (v !== null && typeof v === 'object') { try { v = JSON.stringify(v); } catch (e) { v = String(v); } }
      kvRow(kv2, DETAIL_LABELS[k] || k, v);
    });
    sb.appendChild(kv2);
  }

  var runDir = stage.run_dir || (state.trial && state.trial.run_dir) || '';
  if (runDir) {
    var rr = el('div', 'grp-row');
    rr.appendChild(el('span', 'tiny muted', '产物目录'));
    rr.appendChild(el('code', 'tiny mono wrap-any', runDir));
    rr.appendChild(copyBtn(function () { return runDir; }, '产物目录'));
    sb.appendChild(rr);
  }
  sec.appendChild(sb);
  body.appendChild(sec);

  // orchestration 进度
  var orch = state.orchestration;
  if (orch && (orch.running || (orch.steps && orch.steps.length))) {
    var so = el('div', 'sec');
    so.appendChild(secHead('一键启动进度' + (orch.running ? '（进行中）' : '')));
    var sob = el('div', 'sec-body');
    var steps = el('div', 'steps');
    (orch.steps || []).forEach(function (s) {
      var initStatus = s.id === 'servo_init' ? termStatus(s.id) : null;
      var row = el('div', 'step ' + (initStatus ? initStatus.cls : 's-' + (s.state || 'pending')));
      row.appendChild(el('span', 'sdot'));
      row.appendChild(el('span', null, s.title || s.id));
      if (initStatus) row.appendChild(el('span', 'sdetail', '· ' + initStatus.label));
      if (s.detail) row.appendChild(el('span', 'sdetail', '· ' + s.detail));
      steps.appendChild(row);
    });
    sob.appendChild(steps);
    if (orch.step) sob.appendChild(el('div', 'tiny muted', '当前步骤：' + orch.step + ' · 开始 ' + hhmmss(orch.started_at)));
    so.appendChild(sob);
    body.appendChild(so);
  }

  // 设备与遥测
  var sd = el('div', 'sec');
  sd.appendChild(secHead('设备与遥测', 'master ' + (tel.master === true ? 'OK' : (tel.master === false ? '无' : '—'))));
  var sdb = el('div', 'sec-body');
  var kv3 = el('div', 'kv');
  kvRow(kv3, '探针链路', view.detail, telemetryFresh ? 'ok' : 'warn');
  kvRow(kv3, 'ROS master', tel.master === true ? '已就绪' : (tel.master === false ? '未就绪' : '—'), tel.master ? 'ok' : 'warn');
  kvRow(kv3, '节点数', (tel.nodes || []).length + (tel.nodes && tel.nodes.length ? ('（' + tel.nodes.slice(0, 4).join(', ') + (tel.nodes.length > 4 ? ' …' : '') + '）') : ''));
  kvRow(kv3, '最后遥测', telemetryAt ? hhmmss(telemetryAt) + (telemetryFresh ? '' : ' · 已过期') : '未观测', telemetryFresh ? '' : 'warn');
  var mst = tel.state || {};
  function observedBool(value) { return typeof value === 'boolean' ? String(value) : '未观测'; }
  kvRow(kv3, '飞控', telemetryFresh
    ? 'connected=' + observedBool(mst.connected) + ' armed=' + observedBool(mst.armed) + ' mode=' + txt(mst.mode || '未观测')
    : unobserved, telemetryFresh ? '' : 'warn');
  if (tel.mission && !observationMode) kvRow(kv3, 'mission', txt(tel.mission.phase || '') + (tel.mission.reason ? (' · ' + tel.mission.reason) : ''));
  if (tel.probe_status) kvRow(kv3, '探针阶段', txt(tel.probe_status.scope || '') + ' / ' + txt(tel.probe_status.stage || ''));
  if (tel.extended && tel.extended.landed_state !== undefined) kvRow(kv3, 'landed_state', String(tel.extended.landed_state));
  var hover = telemetryFresh ? tel.terminal_hover : null;
  if (typeof hover === 'string') {
    try { hover = JSON.parse(hover); } catch (e) { hover = { status: hover }; }
  }
  var hoverStage = hover && (hover.stage || hover.status || hover.state || hover.phase || '');
  var hoverLabel = { DESCEND_TO_HOVER: '下降至收尾悬停', PILOT_HANDOFF: '交给飞手落地' }[hoverStage] || hoverStage;
  if (!observationMode) kvRow(kv3, '收尾悬停', hover ? [hoverLabel, hover.reason || hover.message || ''].filter(Boolean).join(' · ') || JSON.stringify(hover) : (telemetryFresh ? '未观测' : unobserved), hover ? '' : 'warn');
  if (observationMode) {
    var obs = telemetryFresh && tel.observe && tel.observe.low_hover;
    kvRow(kv3, '低空status话题', obs ? JSON.stringify(obs) : unobserved);
    kvRow(kv3, '就绪依据', '仅动作终端 READY_FOR_MANUAL_ARM_AND_OFFBOARD；status READY不替代预发完成');
  }
  var lio = telemetryFresh ? tel.lio_realtime || [] : [];
  if (!lio.length) kvRow(kv3, 'LIO实时状态', telemetryFresh ? '未观测' : unobserved, 'warn');
  lio.forEach(function (diag) {
    var values = diag.values || {};
    var metric = ['output_age_sec', 'lidar_queue', 'imu_queue'].filter(function (key) { return values[key] != null; })
      .map(function (key) { return key + '=' + values[key]; }).join(' · ');
    kvRow(kv3, 'LIO ' + (diag.name || '实时状态'), [diag.message, metric].filter(Boolean).join(' · ') || '未观测',
      Number(diag.level) > 0 ? 'warn' : '');
  });
  sdb.appendChild(kv3);

  var topics = tel.topics || {};
  var tb = el('table', 'mini');
  var th = el('tr');
  ['话题', 'hz', 'age', '样本'].forEach(function (h, i) {
    var c = el('th', i > 0 ? 'num' : null, h); th.appendChild(c);
  });
  tb.appendChild(el('thead')).appendChild(th);
  var tbody = el('tbody');
  (observationMode ? ['/mavros/state','/mavros/local_position/pose','/Odometry','/mavros/vision_pose/pose','/low_hover_observation/status'] : TOPIC_WATCH).forEach(function (name) {
    var info = topics[name];
    var tr = el('tr');
    if (!info) { tr.className = 'miss'; tr.appendChild(el('td', 'mono', name)); tr.appendChild(el('td', 'num', '—')); tr.appendChild(el('td', 'num', '—')); tr.appendChild(el('td', 'num', '—')); }
    else {
      var age = info.age;
      if (age !== undefined && age !== null && age > 2) tr.className = 'stale';
      tr.appendChild(el('td', 'mono', name));
      tr.appendChild(el('td', 'num', info.hz === undefined || info.hz === null ? '—' : Number(info.hz).toFixed(1)));
      tr.appendChild(el('td', 'num', age === undefined || age === null ? '—' : Number(age).toFixed(2) + 's'));
      tr.appendChild(el('td', 'num', info.count === undefined ? '—' : String(info.count)));
    }
    tbody.appendChild(tr);
  });
  tb.appendChild(tbody);
  sdb.appendChild(tb);
  sdb.appendChild(el('div', 'tiny muted', 'age > 2s 标黄；未出现的话题按“无数据”处理。'));
  sd.appendChild(sdb);
  body.appendChild(sd);

  // 飞控冲突
  var conflicts = (state.board && state.board.preflight && state.board.preflight.conflicts) || [];
  if (conflicts.length) {
    var sc = el('div', 'sec');
    sc.appendChild(secHead('飞控冲突', conflicts.length + ' 个'));
    var scb = el('div', 'sec-body');
    var cb = el('div', 'conflict-box');
    cb.appendChild(el('div', null, '必须先退出的旧应用（否则会抢飞控 / 抢串口）：'));
    var ul = el('ul');
    conflicts.forEach(function (c) { var li = el('li'); li.appendChild(el('code', null, String(c))); ul.appendChild(li); });
    cb.appendChild(ul);
    scb.appendChild(cb);
    sc.appendChild(scb);
    body.appendChild(sc);
  }

  // 阶段时间线
  var hist = (stage.history || []).slice().reverse();
  var sh = el('div', 'sec');
  sh.appendChild(secHead('阶段时间线', hist.length + ' 条'));
  var shb = el('div', 'sec-body');
  if (!hist.length) shb.appendChild(el('p', 'empty', '暂无阶段变化'));
  else {
    var ul2 = el('ul', 'hist-list');
    hist.slice(0, 40).forEach(function (h) {
      var li = el('li');
      li.appendChild(el('span', 'tl-time', hhmmss(h.at)));
      li.appendChild(el('span', null, (STAGE_LABELS[h.name] || h.name) + '（' + h.name + '）'));
      ul2.appendChild(li);
    });
    shb.appendChild(ul2);
  }
  sh.appendChild(shb);
  body.appendChild(sh);

  // 告警
  var sa = el('div', 'sec');
  sa.appendChild(secHead('告警', state.alerts.length + ' 条'));
  var sab = el('div', 'sec-body');
  if (!state.alerts.length) sab.appendChild(el('p', 'empty', '暂无告警'));
  else {
    var ul3 = el('ul', 'alert-list');
    state.alerts.slice(0, 40).forEach(function (a) {
      var lv = a.level || 'info';
      var li = el('li', 'a-' + lv);
      li.appendChild(el('span', 'alert-time', hhmmss(a.at) + ' '));
      li.appendChild(el('span', null, txt(a.text)));
      if (a.hint) { li.appendChild(el('div', 'alert-hint', txt(a.hint))); }
      ul3.appendChild(li);
    });
    sab.appendChild(ul3);
  }
  sa.appendChild(sab);
  body.appendChild(sa);

  // 回报
  var sr = el('div', 'sec');
  sr.appendChild(secHead('回报 markdown'));
  var srb = el('div', 'sec-body');
  if (!keepReport) srb.appendChild(el('p', 'empty', '点顶栏「生成回报」后在此显示 markdown。'));
  else {
    var pre = el('pre', 'report-pre', keepReport);
    srb.appendChild(pre);
    var row = el('div', 'grp-row');
    row.appendChild(copyBtn(function () { return state.reportText; }, '回报 markdown'));
    var dl = el('a', 'btn btn-sm', '下载 .md');
    dl.href = 'data:text/markdown;charset=utf-8,' + encodeURIComponent(state.reportText);
    var rp = (state.report && state.report.path) || '';
    var fname = rp ? rp.split('/').pop() : 'flight_report.md';
    dl.setAttribute('download', fname);
    row.appendChild(dl);
    if (rp) row.appendChild(el('span', 'tiny muted wrap-any', rp));
    srb.appendChild(row);
  }
  sr.appendChild(srb);
  body.appendChild(sr);
  body.scrollTop = scrollTop;
  var newReport = body.querySelector('.report-pre');
  if (newReport) newReport.scrollTop = reportTop;
}

function secHead(title, right) {
  var h = el('div', 'sec-head');
  h.appendChild(el('span', null, title));
  if (right) h.appendChild(el('span', 'tiny muted', right));
  return h;
}

/* 启动任务：只在 READY / IN_FLIGHT 允许，且必须人工点一次 + 二次确认。
 * 后端 mission_start 的允许阶段与这里保持一致，并会再校验一次。 */
var MISSION_STAGES = ['READY', 'IN_FLIGHT'];
function missionStartSection(stage) {
  var st = (stage && stage.name) || 'IDLE';
  var stageOk = MISSION_STAGES.indexOf(st) >= 0;
  var cur = null;
  (state.groups || []).forEach(function (gg) {
    if (gg.id === state.trial.group_id) cur = gg;
  });
  var manual = !!(cur && cur.manual_mission_start);
  if (cur && cur.channel === 'low_observation') {
    var observationSection = el('div','sec');
    observationSection.appendChild(secHead('独立低空观察', cur.profile));
    observationSection.appendChild(el('div','sec-body small', '无任务管理器。看到 READY_FOR_MANUAL_ARM_AND_OFFBOARD 后飞手人工解锁并重新拨入 OFFBOARD；路线结束继续悬停，飞手落地上锁。'));
    return observationSection;
  }
  var tSess = state.sessions.trial || {};
  var trialRun = (tSess.state === 'running' || tSess.state === 'starting');

  var sec = el('div', 'sec');
  sec.appendChild(secHead(manual ? '启动任务（仅 READY 后）' : '自动任务', manual ? '板端下发一次' : '飞手操作后由入口启动'));
  var sb = el('div', 'sec-body');
  sb.appendChild(el('div', 'tiny muted',
    manual ? '板端执行一次：rosservice call /navigation/start_mission "{}"（请求体 {confirm:"启动任务"}）'
      : 'READY 后人工解锁、人工拨入 OFFBOARD；稳定条件满足后入口自动起飞并开始任务，接管会取消自动时序。'));

  var row = el('div', 'grp-row');
  var label = '启动任务（仅 READY 后）' + (manual && cur ? (' · ' + (cur.key || cur.id)) : '');
  var btn = el('button', 'btn btn-sm', label);
  var reason = '';
  if (!connected()) reason = '未连接板端，先连接';
  else if (!manual) reason = '所选运行入口使用自动任务时序，无需手动下发';
  else if (!stageOk) reason = '当前阶段是 ' + st + '，只有 READY 之后才允许启动任务（后端会返回 400）';
  else if (!trialRun) reason = '专项入口未在运行：先启动对应任务组';
  else if (state.trial.mode !== 'flight' || state.trial.check_config) reason = 'preview/配置检查不能启动任务';
  else if (stage.auto_sequence) reason = '板端自动时序负责启动任务，请勿重复下发';
  btn.disabled = !!reason;
  btn.title = reason ? reason
    : 'POST /api/action/mission_start {confirm:"启动任务"} → 板端 rosservice call /navigation/start_mission "{}"\n'
      + '只在 READY、飞手已解锁并进入 OFFBOARD、低空悬停稳定后调用一次；不要为催促重复调用。';
  btn.addEventListener('click', doMissionStart);
  row.appendChild(btn);
  if (reason) row.appendChild(el('span', 'tiny muted', reason));
  else row.appendChild(el('span', 'tiny muted', '只在飞手完成解锁与稳定悬停后调用一次'));
  if (manual) row.appendChild(el('span', 'badge badge-info', '该组 manual_mission_start=true'));
  sb.appendChild(row);

  if (state.lastMissionOutput) {
    sb.appendChild(el('div', 'tiny muted', '最近一次 rosservice 返回（必须 success: true）'));
    sb.appendChild(el('pre', 'cmd-pre', state.lastMissionOutput));
  }
  sec.appendChild(sb);
  return sec;
}

/* 兼容显式 manual_mission_start 的旧入口；当前自动任务无需下发。 */
function doMissionStart() {
  var stage = state.stage || {};
  var cmdText = 'POST /api/action/mission_start\n' + JSON.stringify({ confirm: '启动任务' })
    + '\n→ 板端执行：rosservice call /navigation/start_mission "{}"';
  confirmModal('启动任务（仅 READY 后）', cmdText,
    '只在 READY、飞手已解锁并进入 OFFBOARD、低空悬停稳定后调用一次；不要为催促重复调用。\n'
    + '当前阶段：' + txt(stage.name || '—') + ' · armed=' + (stage.armed ? 'true' : 'false')
    + ' · trial 会话=' + txt((state.sessions.trial || {}).state || '—') + '\n'
    + '飞手人工解锁并人工拨入 OFFBOARD；实际起飞方式以当前入口配置为准。', '确认下发一次').then(function (res) {
    if (!res || !res.confirmed) return;
    act(api.missionStart(), '启动任务').then(function (r) {
      if (!r) return;
      var out = txt(r.output || '').trim();
      state.lastMissionOutput = out || ('（服务未返回文本）ok=' + txt(r.ok));
      renderMonitor();
      var good = (r.ok === true) && /success:\s*true/i.test(out);
      toast(good ? 'ok' : 'warn', good
        ? '启动任务返回 success: true'
        : ('启动任务返回未确认成功，必须人工核对：' + (out || txt(r.error || '无输出'))));
    });
  });
}

function nowSec() { return Date.now() / 1000; }

/* 每秒刷新阶段持续时间（本地时钟，不用服务端时间） */
function tickElapsed() {
  if (probeView().usable !== state._lastProbeUsable) scheduleRender();
  var nodes = document.querySelectorAll('[data-elapsed-since]');
  for (var i = 0; i < nodes.length; i++) {
    var since = parseFloat(nodes[i].getAttribute('data-elapsed-since'));
    if (!isFinite(since)) continue;
    nodes[i].textContent = '已持续 ' + dur(nowSec() - since);
  }
}

/* ==========================================================================
 * 10. 渲染 · 底部抽屉
 * ========================================================================== */

function renderDrawer() {
  var body = $('#drawer-body');
  var tools = $('#drawer-tools');
  if (!body || !tools) return;
  // Keep the filter DOM/focus (including IME) and scroll on periodic updates.
  if (state.drawer === 'run' && body.getAttribute('data-drawer') === 'run'
      && state.logTerm && state.logTerm.scrollEl.parentNode === body) return;
  var sameDrawer = body.getAttribute('data-drawer') === state.drawer;
  var scrollTop = sameDrawer ? body.scrollTop : 0;
  if (state.logTerm) state.logTerm.saveScroll();
  body.setAttribute('data-drawer', state.drawer);
  clear(tools);
  clear(body);

  var tabs = document.querySelectorAll('.dtab');
  for (var i = 0; i < tabs.length; i++) tabs[i].classList.toggle('active', tabs[i].getAttribute('data-drawer') === state.drawer);

  if (state.drawer === 'run') {
    var U = state.tabs._log || (state.tabs._log = { filter: '' });
    var f = el('input'); f.type = 'text'; f.value = U.filter;
    f.placeholder = '关键字过滤（INITIALIZING / MAPPING_READY / READY / FLIGHT_STATUS / ERROR）';
    f.size = 40;
    f.addEventListener('input', function () {
      U.filter = f.value;
      if (state.logTerm) state.logTerm.setLineFilter(f.value);
    });
    f.addEventListener('keydown', function (ev) { if (ev.key === 'Enter') ev.preventDefault(); });
    var clr = el('button', 'btn btn-sm btn-ghost', '清除过滤');
    clr.addEventListener('click', function () { U.filter = ''; f.value = ''; if (state.logTerm) state.logTerm.setLineFilter(''); });
    tools.appendChild(el('span', 'tiny muted', 'trial 会话实时输出（只读）'));
    tools.appendChild(f); tools.appendChild(clr);
    var t = state.logTerm;
    if (t) {
      body.appendChild(t.scrollEl);
      t.setLineFilter(U.filter || '');
      t.stick();
    }
  } else if (state.drawer === 'board') {
    var ref = el('button', 'btn btn-sm', '刷新');
    ref.title = 'POST /api/logs/refresh {}';
    ref.addEventListener('click', function () {
      act(api.logsRefresh(), '刷新板端产物').then(function (r) {
        if (r && r.ok && r.board) { state.board = r.board; renderDrawer(); }
      });
    });
    tools.appendChild(ref);
    var runs = (state.board && state.board.logs) || [];
    tools.appendChild(el('span', 'tiny muted', runs.length + ' 个 run 目录'));
    var pf = state.board && state.board.preflight;
    if (pf) {
      var box = el('div', 'sec');
      box.appendChild(secHead('预检查', hhmmss(pf.at) + (pf.conflicts && pf.conflicts.length ? (' · 冲突 ' + pf.conflicts.length) : '')));
      var pb = el('div', 'sec-body');
      (pf.checks || []).forEach(function (c) {
        var row = el('div', 'grp-row');
        row.appendChild(el('span', 'badge ' + (c.ok ? 'badge-info' : 'badge-real'), c.ok ? 'OK' : 'FAIL'));
        row.appendChild(el('span', null, c.name));
        if (c.detail) row.appendChild(el('span', 'tiny muted wrap-any', c.detail));
        pb.appendChild(row);
      });
      if (pf.conflicts && pf.conflicts.length) {
        var cbox = el('div', 'conflict-box');
        cbox.appendChild(el('div', null, '必须先退出的旧应用：'));
        var ul = el('ul');
        pf.conflicts.forEach(function (c) { var li = el('li'); li.appendChild(el('code', null, String(c))); ul.appendChild(li); });
        cbox.appendChild(ul);
        pb.appendChild(cbox);
      }
      box.appendChild(pb);
      body.appendChild(box);
    }
    if (!runs.length) body.appendChild(el('p', 'empty', '暂无板端产物（点「刷新」拉取 /api/logs/refresh）'));
    runs.forEach(function (run) {
      body.appendChild(el('div', 'run-title', txt(run.run) + '  ' + (run.trial ? ('· ' + run.trial) : '') + '  ' + hhmmss(run.mtime)));
      (run.files || []).forEach(function (fl) {
        var row = el('div', 'file-row');
        row.appendChild(el('span', 'fname', fl.name));
        row.appendChild(el('span', 'fsize', fmtSize(fl.size)));
        var v = el('button', 'btn btn-sm btn-ghost', '预览');
        v.addEventListener('click', function () {
          act(api.logsTail(run.run, fl.name, 200), '预览 ' + fl.name).then(function (r) {
            if (!r || !r.ok) return;
            openTextModal(fl.name + ' · ' + run.run, r.text || '');
          });
        });
        row.appendChild(v);
        var dl = el('a', 'btn btn-sm btn-ghost', '下载');
        dl.href = '/api/logs/download?' + qs({ run: run.run, file: fl.name });
        dl.setAttribute('download', fl.name);
        row.appendChild(dl);
        body.appendChild(row);
      });
    });
  } else {
    tools.appendChild(el('span', 'tiny muted', (state.timeline || []).length + ' 条'));
    var tb = el('ul', 'tl-list');
    (state.timeline || []).slice().reverse().forEach(function (it) {
      var li = el('li');
      li.appendChild(el('span', 'tl-time', hhmmss(it.at)));
      li.appendChild(el('span', 'badge a-' + (it.level || 'info'), it.level || 'info'));
      li.appendChild(el('span', null, txt(it.text)));
      tb.appendChild(li);
    });
    if (!state.timeline.length) body.appendChild(el('p', 'empty', '暂无操作时间线'));
    else body.appendChild(tb);
  }
  body.scrollTop = scrollTop;
}

/* ==========================================================================
 * 11. 模态框
 * ========================================================================== */

function closeModal() {
  var root = $('#modal-root');
  clear(root);
  root.classList.add('hidden');
}
function openModal(title, opts) {
  var root = $('#modal-root');
  clear(root);
  root.classList.remove('hidden');
  var m = el('div', 'modal');
  m.appendChild(el('div', 'modal-head', title));
  var body = el('div', 'modal-body');
  m.appendChild(body);
  var foot = el('div', 'modal-foot');
  m.appendChild(foot);
  root.appendChild(m);
  root.onclick = function (ev) { if (ev.target === root) closeModal(); };
  return { body: body, foot: foot, close: closeModal };
}
/* 命令确认模态：展示准确命令 + 备注，确认才返回 true */
function openCommandModal(title, command, note) {
  var mo = openModal(title);
  mo.body.appendChild(el('div', 'tiny muted', '本工作台将要执行的准确命令（只读展示）：'));
  mo.body.appendChild(el('pre', 'cmd-pre', command || '（空）'));
  if (note) mo.body.appendChild(el('div', 'tiny muted wrap-any', note));
  mo.foot.appendChild(copyBtn(function () { return command || ''; }, '命令'));
  var ok = el('button', 'btn btn-primary', '知道了');
  ok.addEventListener('click', closeModal);
  mo.foot.appendChild(ok);
}
function openTextModal(title, text) {
  var mo = openModal(title);
  mo.body.appendChild(el('pre', 'cmd-pre', String(text || '')));
  mo.foot.appendChild(copyBtn(function () { return String(text || ''); }, '日志内容'));
  var ok = el('button', 'btn btn-primary', '关闭');
  ok.addEventListener('click', closeModal);
  mo.foot.appendChild(ok);
}
/* Promise 风格确认框；extra 用于附加表单内容（如口令输入） */
function confirmModal(title, command, note, okLabel) {
  return new Promise(function (resolve) {
    var mo = openModal(title);
    if (command) {
      mo.body.appendChild(el('div', 'tiny muted', '将要执行的准确命令 / 请求：'));
      mo.body.appendChild(el('pre', 'cmd-pre', command));
    }
    if (note) mo.body.appendChild(el('div', 'small wrap-any', note));
    var extra = el('div');
    mo.body.appendChild(extra);
    var cancel = el('button', 'btn', '取消');
    cancel.addEventListener('click', function () { closeModal(); resolve(false); });
    var ok = el('button', 'btn btn-primary', okLabel || '确认执行');
    ok.addEventListener('click', function () {
      var vals = {};
      var inputs = extra.querySelectorAll('[data-field]');
      for (var i = 0; i < inputs.length; i++) vals[inputs[i].getAttribute('data-field')] = inputs[i].value;
      closeModal();
      resolve({ confirmed: true, values: vals });
    });
    mo.foot.appendChild(cancel); mo.foot.appendChild(ok);
    return extra;
  });
}
/* 带字段的确认框 */
function formModal(title, command, note, fields, okLabel) {
  return new Promise(function (resolve) {
    var mo = openModal(title);
    if (command) {
      mo.body.appendChild(el('div', 'tiny muted', '将要执行的准确命令 / 请求：'));
      mo.body.appendChild(el('pre', 'cmd-pre', command));
    }
    if (note) mo.body.appendChild(el('div', 'small wrap-any', note));
    var grid = el('div', 'field-grid');
    fields.forEach(function (f) {
      grid.appendChild(el('label', null, f.label));
      var inp;
      if (f.type === 'select') {
        inp = el('select');
        (f.options || []).forEach(function (o) {
          var opt = el('option', null, o.label === undefined ? o.value : o.label);
          opt.value = o.value;
          inp.appendChild(opt);
        });
        inp.value = f.value === undefined || f.value === null ? '' : String(f.value);
      } else {
        inp = el('input');
        inp.type = f.type || 'text';
        inp.value = f.value === undefined || f.value === null ? '' : String(f.value);
        if (f.placeholder) inp.placeholder = f.placeholder;
        if (f.type === 'checkbox') { inp.checked = !!f.value; inp.style.width = 'auto'; }
      }
      inp.setAttribute('data-field', f.name);
      grid.appendChild(inp);
    });
    mo.body.appendChild(grid);
    var cancel = el('button', 'btn', '取消');
    cancel.addEventListener('click', function () { closeModal(); resolve(null); });
    var ok = el('button', 'btn btn-primary', okLabel || '确认');
    ok.addEventListener('click', function () {
      var vals = {};
      var inputs = grid.querySelectorAll('[data-field]');
      for (var i = 0; i < inputs.length; i++) {
        var k = inputs[i].getAttribute('data-field');
        vals[k] = inputs[i].type === 'checkbox' ? inputs[i].checked : inputs[i].value;
      }
      closeModal();
      resolve(vals);
    });
    mo.foot.appendChild(cancel); mo.foot.appendChild(ok);
  });
}

/* ==========================================================================
 * 12. 动作
 * ========================================================================== */

function openTerminal(t) {
  var doOpen = function (confirmText) {
    act(api.sessionOpen(t.id, confirmText), '启动 ' + (t.title || t.id)).then(function (r) {
      if (r && r.ok) {
        if (r.session) state.sessions[t.id] = r.session;
        toast('ok', '已启动：' + (t.title || t.id));
        renderTerminals();
      }
    });
  };
  var cmdText = 'POST /api/session/open\n' + JSON.stringify(t.confirm ? { id: t.id, confirm: '确认' } : { id: t.id });
  if (t.confirm) {
    confirmModal('启动确认 · ' + (t.title || t.id), cmdText, t.confirm + '\n\n该终端涉及执行机构 / 硬件，确认后才会启动：\n' + (t.command || ''), '确认启动')
      .then(function (res) { if (res && res.confirmed) doOpen('确认'); });
  } else {
    doOpen(undefined);
  }
}

function doConnect(custom) {
  var c = state.connection || {};
  var fields = [
    { name: 'host', label: 'SSH 地址（历史）', type: 'select', value: custom === true ? '' : c.host,
      options: hostChoices() },
    { name: 'host_custom', label: '自定义地址（优先）', placeholder: 'IP / 主机名，也支持 user@host' },
    { name: 'user', label: 'SSH 用户名', value: c.user || 'orangepi' },
    { name: 'port', label: 'SSH 端口', type: 'number', value: c.port || 22 },
    { name: 'password', label: 'SSH 密码（可选）', type: 'password', placeholder: '留空使用当前口令 / 密钥' }
  ];
  formModal('连接板端', '先保存所选地址，再检查 SSH 连接；不启动飞行或设备。',
    '自定义地址留空时使用下拉选项。密码只留在当前工作台进程内存，不写入文件或浏览器存储。',
    fields, '连接').then(function (vals) {
    if (!vals) return;
    var customHost = String(vals.host_custom || '').trim();
    var host = customHost || String(vals.host || '').trim();
    var user = String(vals.user || '').trim();
    if (host.indexOf('@') >= 0) {
      if (customHost) user = host.slice(0, host.lastIndexOf('@'));
      host = host.slice(host.lastIndexOf('@') + 1);
    }
    var port = Number(vals.port);
    if (!host || !/^[a-zA-Z0-9.:[\]_-]+$/.test(host) || host.charAt(0) === '-' ||
        !/^[a-zA-Z0-9_][a-zA-Z0-9_.-]*$/.test(user) || !Number.isInteger(port) || port < 1 || port > 65535) {
      toast('warn', '请填写有效的地址、用户名和 1–65535 范围内的端口');
      return;
    }
    if (c.transport === 'local') {
      toast('info', '当前是离线预览。要连接板端，请用默认 SSH 模式启动工作台；本次未连接。');
      return;
    }
    act(api.config({ host: user + '@' + host, port: port }), '保存连接地址').then(function (cfg) {
      if (!cfg || !cfg.ok) return;
      mergeConnection(cfg.connection);
      state.connection.state = 'checking';
      renderTopbar();
      var body = {};
      if (vals.password) body.password = vals.password;
      act(api.connect(body), '连接').then(function (r) {
        if (r && r.connection) mergeConnection(r.connection);
        scheduleRender();
        if (r && r.ok) toast('ok', '已连接 ' + (state.connection.host || ''));
      });
    });
  });
}

function doConfig() {
  var c = state.connection || {};
  var cmdText = 'POST /api/config\n' + JSON.stringify({
    host: '<host>', user: '<user>', port: '<port>', board_root: '<board_root>',
    site_dir: '<site_dir>', env_script: '<env_script>', model: '<model>', metadata: '<metadata>',
    auto_password: '<bool>', save_password: '<bool>'
  }, null, 2);
  formModal('连接参数（engineer only）', cmdText,
    '这些参数决定 SSH 目标与板端工程根目录；修改后需要重新连接。口令不会写入浏览器 localStorage。\n'
    + 'host 下拉是现场用过的历史地址；要用清单外的地址，在「host（自定义）」里直接填（填了就覆盖下拉选择）。',
    [
      { name: 'host', label: 'host（历史地址）', type: 'select', value: c.host || '', options: hostChoices() },
      { name: 'host_custom', label: 'host（自定义，可留空）', value: '', placeholder: 'orangepi@192.168.43.59' },

      { name: 'port', label: 'port', value: c.port || 22 },
      { name: 'board_root', label: 'board_root', value: c.board_root || '' },
      { name: 'site_dir', label: 'site_dir', value: c.site_dir || '' },
      { name: 'env_script', label: 'env_script', value: c.env_script || '' },
      { name: 'model', label: 'model', value: c.model || '' },
      { name: 'metadata', label: 'metadata', value: c.metadata || '' },
      { name: 'password', label: '口令（可选）', type: 'password' },
      { name: 'save_password', label: '记住口令（仅本机，仓库外）', type: 'checkbox', value: false }
    ], '保存').then(function (vals) {
    if (!vals) return;
    vals.port = parseInt(vals.port, 10) || 22;
    if (vals.host_custom && String(vals.host_custom).trim()) vals.host = String(vals.host_custom).trim();
    delete vals.host_custom;
    if (!vals.host) {
      toast('warn', 'host 为空：请从下拉选一个历史地址，或填写自定义地址');
      return;
    }
    act(api.config(vals), '保存连接参数').then(function (r) {
      if (r && r.connection) mergeConnection(r.connection);
      if (r && r.profile) state.profile = r.profile;
      scheduleRender();
    });
  });
}

function doPreflight() {
  var cmdText = 'POST /api/action/preflight {}';
  confirmModal('单实例检查', cmdText,
    '检查板端工程目录、ROS/MAVROS 进程与飞控串口占用；不启动任何节点。', '开始检查').then(function (res) {
    if (!res || !res.confirmed) return;
    act(api.preflight(), '单实例检查').then(function (r) {
      if (r && r.board) state.board = r.board;
      renderMonitor(); renderDrawer();
    });
  });
}

function doStartAll() {
  var selected = (state.groups || []).find(function(g) { return g.id === state.selectedGroup; });
  if (selected && selected.channel === 'low_observation') {
    var devices = (state.terminals || []).filter(function(t) { return ['roscore','mavros','lidar','observation_localization'].indexOf(t.id) >= 0; });
    return confirmModal('启动低空观察设备', devices.map(function(t) { return t.title + ' → ' + t.command; }).join('\n'),
      '仅 ROS、MAVROS、雷达、定位/EV；已有新鲜设备跳过。本操作不启动观察 flight，随后点击卡片配置检查和飞行。', '确认启动设备')
      .then(function(res) { if (res && res.confirmed) return act(api.startAll(false, selected.id), '启动低空观察设备'); });
  }
  var servo = (state.terminals || []).filter(function (t) { return t.id !== 'roscore' && t.needs_servo; });
  var cmdText = 'POST /api/action/start_all\n' + JSON.stringify({ include_servo: '<bool>', confirm: '启动设备' });
  var note = '按顺序启动设备终端并逐项等待就绪：\n'
    + (state.terminals || []).map(function (t) { return t.seq + '. ' + t.title + '  →  ' + (t.command || ''); }).join('\n')
    + '\n\n不包含解锁、起飞、投递动作；舵机相关终端是否包含由下面选项决定。';
  formModal('一键启动设备', cmdText, note, [
    { name: 'include_servo', label: '包含舵机端子（需逐个确认）', type: 'checkbox', value: false }
  ], '确认启动').then(function (vals) {
    if (!vals) return;
    act(api.startAll(vals.include_servo, selected && selected.id), '一键启动设备').then(function (r) {
      if (r && r.orchestration) state.orchestration = r.orchestration;
      renderMonitor();
    });
  });
}

function doStopAll() {
  var cmdText = 'POST /api/action/stop_all {}';
  var list = (state.terminals || []).map(function (t) { return t.title; }).join(' / ');
  confirmModal('全部停止', cmdText,
    '会依次关闭所有常驻终端（' + list + '）与 trial 会话。\n'
    + '注意：本工作台不自动降落、不自动上锁；请在飞控链路仍可用时确认飞机安全状态。', '停止全部').then(function (res) {
    if (!res || !res.confirmed) return;
    act(api.stopAll(), '全部停止');
  });
}

function doReport() {
  act(api.report(), '生成回报').then(function (r) {
    if (r && r.report) { state.report = r.report; state.reportText = r.report.markdown || ''; }
    renderMonitor();
  });
}

function startTrial(g, mode, checkConfig) {
  if (!connected()) { toast('warn', '先连接板端'); return; }
  if (tSessRunning() || state.trialPending) { toast('warn', '当前专项正在运行或准备下发，请先停止当前任务'); return; }
  var U = groupUI(g);
  var body = trialBody(g, mode, checkConfig);
  if (body.mode === 'flight') {
    if (!U.armedOk) { toast('warn', '飞行前必须先勾选「已确认飞机回到起飞点、未解锁、机头朝场内」'); return; }
    if (body.real_release && (U.realConfirm || '').trim() !== '实投') {
      toast('warn', '真实投递必须输入确认词「实投」');
      return;
    }
  }
  var built = groupCommandBody(g, body.mode, body.real_release, !!body.check_config, body.capture_speed, body);
  if (!built.body) { toast('warn', built.note); return; }
  var cmd = buildTrialCommand(g, body.mode, body.real_release, !!body.check_config, body.capture_speed, body);
  var note = '请求体：' + JSON.stringify(body) + '\n'
    + '入口说明：' + built.note + '\n'
    + '模式：' + (mode === 'flight' ? 'flight（飞行）' : 'preview（只采集，不下发）') + '\n'
    + 'release=' + txt(U.release || 'none') + ' · needs_servo=' + (body.real_release ? 'true' : 'false')
    + ' · needs_waypoints=' + (g.needs_waypoints ? 'true' : 'false') + '\n'
    + (g.ending ? ('ending：' + g.ending + '\n') : '')
    + (mode === 'flight'
      ? 'READY 后由飞手人工解锁、人工拨入 OFFBOARD；入口在稳定条件满足后自动起飞并开始任务。飞手接管后自动时序取消。'
      : '预览模式只启动视觉采集与配置检查。');
  if (g.channel === 'low_observation') {
    note = built.note + '\n' + g.plan + '\n' + g.ending + '\n'
      + '配置检查与preview均离线展开原profiles.yaml，无节点/设定点。flight输出在专项flight终端。\n'
      + '空中Ctrl+C只请求保持等待接管，定位/MAVROS继续运行；飞手落地上锁并等待OBSERVATION_CLOSED后才能重跑。';
  }

  state.trialPending = true;
  scheduleRender();
  var approval = checkConfig ? Promise.resolve({ confirmed: true })
    : confirmModal((mode === 'flight' ? '确认启动飞行试飞 · ' : '确认启动预览 · ') + g.name, cmd, note,
      mode === 'flight' ? '确认启动 flight' : '确认启动 preview');
  return approval.then(function (res) {
    if (!res || !res.confirmed) return;
    // Recheck after the dialog: a new snapshot may report a trial started elsewhere.
    if (!connected() || tSessRunning()) { toast('warn', '连接或运行状态已变化，未下发新任务'); return; }
    return act(api.trialStart(body), checkConfig ? '配置检查' : '启动试飞').then(function (r) {
      if (r && r.trial) state.trial = r.trial;
      if (r && r.ok) state.activeTerm = 'trial';
      if (r && r.ok && g.channel === 'low_observation' && body.mode === 'flight' && !checkConfig) U.armedOk = false;
      scheduleRender();
      if (r && r.ok) toast('ok', (checkConfig ? '配置检查已下发：' : '试飞入口已下发：') + g.name);
    });
  }).finally(function () {
    state.trialPending = false;
    scheduleRender();
  });
}

function doTrialStop() {
  var cmdText = 'POST /api/trial/stop {}\n→ 向 trial 会话发送 SIGINT（等同 Ctrl+C），由现场入口自行收尾';
  confirmModal('停止试飞入口', cmdText,
    state.trial.route === 'low_observation'
      ? '空中只请求保持等待飞手接管，不强杀。飞手切手动、落地上锁后等待 OBSERVATION_CLOSED；期间保留定位和MAVROS，不能重跑。'
      : '只是结束入口进程，不发送降落 / 上锁指令。飞机状态以飞控与飞手判断为准。', '停止试飞').then(function (res) {
    if (!res || !res.confirmed) return;
    act(api.trialStop(), '停止试飞');
  });
}

/* ==========================================================================
 * 13. 初始化
 * ========================================================================== */

function bindUI() {
  $('#btn-connect').addEventListener('click', function () { doConnect(false); });
  $('#btn-config').addEventListener('click', doConfig);
  var probeReconnect = $('#btn-probe-reconnect');
  if (probeReconnect) probeReconnect.addEventListener('click', function () {
    if (!supportsCapability('probe_reconnect')) return;
    confirmModal('只重连 probe', 'POST /api/action/probe_reconnect {}',
      '只连接只读遥测探针。设备/任务/舵机常驻服务不会重新启动；75锁冲突时保留已有实例。', '只重连 probe').then(function (res) {
        if (res && res.confirmed) act(api.probeReconnect(), '探针重连');
      });
  });
  var hostSel = $('#host-select');
  if (hostSel) {
    hostSel.addEventListener('change', function () {
      var host = hostSel.value;
      if (!host) { hostSel.value = state.connection.host || ''; doConnect(true); return; }
      act(api.config({ host: host }), '切换板端地址').then(function (r) {
        if (r && r.connection) mergeConnection(r.connection);
        if (r && r.profile) state.profile = r.profile;
        scheduleRender();
        if (r && r.ok) toast('info', '板端地址已切到 ' + host + '，点「连接」开始检查');
      });
    });
  }
  $('#btn-disconnect').addEventListener('click', function () {
    confirmModal('断开板端', 'POST /api/disconnect {}', '先落地上锁；断开会关闭本工作台启动的应用和设备会话。外部终端启动的设备需另行收尾。', '断开').then(function (res) {
      if (!res || !res.confirmed) return;
      act(api.disconnect(), '断开连接');
    });
  });
  $('#btn-preflight').addEventListener('click', doPreflight);
  $('#btn-start-all').addEventListener('click', doStartAll);
  $('#btn-stop-all').addEventListener('click', doStopAll);
  $('#btn-report').addEventListener('click', doReport);
  $('#btn-report').addEventListener('contextmenu', function (ev) { ev.preventDefault(); doConfig(); });
  // 双击标题打开连接参数（不显眼入口，避免误触）
  $('.brand-title').addEventListener('dblclick', doConfig);

  var swSound = $('#sw-sound');
  swSound.checked = !!prefs.sound;
  swSound.addEventListener('change', function () {
    prefs.sound = swSound.checked; savePrefs();
    if (prefs.sound) { audioCtx(); beep('ok'); }
  });
  var swAuto = $('#sw-autoscroll');
  swAuto.checked = !!prefs.autoscroll;
  swAuto.addEventListener('change', function () {
    prefs.autoscroll = swAuto.checked; savePrefs();
    Object.keys(state.terms).forEach(function (k) { state.terms[k].setAutoScroll(swAuto.checked); });
    if (state.logTerm) state.logTerm.setAutoScroll(swAuto.checked);
    Object.keys(state.tabs).forEach(function (k) { if (state.tabs[k]) state.tabs[k].autoScroll = swAuto.checked; });
    renderTerminals();
  });

  var dtabs = document.querySelectorAll('.dtab');
  for (var i = 0; i < dtabs.length; i++) {
    dtabs[i].addEventListener('click', function (ev) {
      state.drawer = ev.currentTarget.getAttribute('data-drawer');
      state.drawerCollapsed = false;
      $('#drawer').classList.remove('collapsed');
      $('#btn-drawer').textContent = '收起';
      renderDrawer();
    });
  }
  $('#btn-drawer').addEventListener('click', function () {
    state.drawerCollapsed = !state.drawerCollapsed;
    $('#drawer').classList.toggle('collapsed', state.drawerCollapsed);
    $('#btn-drawer').textContent = state.drawerCollapsed ? '展开' : '收起';
    if (!state.drawerCollapsed) renderDrawer();
  });

  window.addEventListener('beforeunload', function () {
    if (state._es) { try { state._es.close(); } catch (e) { /* ignore */ } }
  });
}

function init() {
  loadPrefs();
  bindUI();
  // 首次进入先用一次快照对齐（SSE hello 也会重置）
  api.snapshot().then(function (snap) {
    if (snap && snap.ok) applySnapshot(snap);
    else toast('warn', '读取 /api/snapshot 失败：' + txt(snap && snap.error ? snap.error : '未知错误'));
  });
  startSSE();
  window.setInterval(tickElapsed, 1000);
}

if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', init);
else init();
