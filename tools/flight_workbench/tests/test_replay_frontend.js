/* Concatenate replay.js before this fixture; no browser or packages required. */
const assert = require('assert');
class ReplayElement {
  constructor(tag) { this.tag = tag; this.children = []; this.value = ''; this.disabled = false; this.hidden = false; }
  get firstChild() { return this.children[0]; }
  append(...nodes) { this.children.push(...nodes); }
  replaceChildren(...nodes) { this.children = nodes; }
}
const elements = {};
for (const id of ['replay-message', 'replay-capabilities', 'replay-refresh', 'replay-roots', 'replay-bag',
  'replay-fps', 'replay-frame', 'replay-encoder', 'replay-start', 'replay-jobs', 'replay-results', 'replay-open']) elements[id] = new ReplayElement('div');
elements['replay-fps'].value = '10'; elements['replay-frame'].value = 'camera_init'; elements['replay-encoder'].value = 'cpu';
global.document = {getElementById: id => elements[id], createElement: tag => new ReplayElement(tag)};
const calls = [], intervals = [];
global.setInterval = fn => intervals.push(fn);
let fixture = {ok: true, roots: {data: '/fixture'}, bags: [{root: 'data', path: '<bag>.bag', size: 100}],
  results: [{root: 'data', path: '1008/analysis', url: '/replay/result/data/1008/analysis/'}], jobs: [],
  capabilities: {generate: false, detail: 'Windows 原生仅查看'}};
global.fetch = async (path, options) => {
  calls.push({path, body: options?.body && JSON.parse(options.body)});
  return {ok: true, json: async () => path.endsWith('/jobs') ? {ok: true, jobs: fixture.jobs} : fixture};
};
const settle = () => new Promise(resolve => setImmediate(resolve));
(async () => {
  replayInit(); await settle();
  assert.deepStrictEqual(calls.map(c => c.path), ['/api/replay/library']);
  assert(elements['replay-start'].disabled);
  elements['replay-bag'].value = JSON.stringify(['data', '<bag>.bag']); replayButtons();
  assert(elements['replay-start'].disabled, 'Windows remains view-only even with a selected bag');
  elements['replay-results'].value = fixture.results[0].url; replayButtons();
  assert(!elements['replay-open'].hidden); assert.equal(elements['replay-open'].href, fixture.results[0].url);
  fixture.capabilities.generate = true; replayButtons();
  assert(!elements['replay-start'].disabled);
  fixture.jobs = [{path: '<bag>.bag', state: 'running', phase: 'render', tail: '<script>unsafe</script>', progress: 30}];
  await intervals[0]();
  assert(elements['replay-start'].disabled);
  assert(!calls.some(c => c.path.endsWith('/start')), 'poll and page load never start a replay');
  fixture.jobs = []; replayState.jobs = []; replayButtons();
  await replayStart();
  const started = calls.filter(c => c.path.endsWith('/start'));
  assert.equal(started.length, 1);
  assert.deepStrictEqual(started[0].body, {root: 'data', path: '<bag>.bag', fps: 10, frame: 'camera_init', encoder: 'cpu'});
  assert(!calls.some(c => /connect|session|recording/.test(c.path)), 'local replay never invokes board APIs');
  console.log('PASS: local replay view-only, selection, polling, CPU default and explicit start');
})().catch(error => { console.error(error); process.exitCode = 1; });
