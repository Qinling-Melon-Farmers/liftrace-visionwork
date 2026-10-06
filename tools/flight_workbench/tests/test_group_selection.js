// Run the real card handlers with a small DOM, without SSH or flight actions.
const fs = require('fs');
const vm = require('vm');
const path = require('path');
const assert = require('assert');

class Element {
  constructor(tag) { this.tagName = tag.toUpperCase(); this.children = []; this.handlers = {}; }
  appendChild(child) { this.children.push(child); child.parentNode = this; return child; }
  addEventListener(name, fn) { this.handlers[name] = fn; }
  setAttribute(name, value) { this[name] = value; }
  closest(selector) {
    const tags = selector.split(',').map(tag => tag.toUpperCase());
    for (let node = this; node; node = node.parentNode) {
      if (tags.includes(node.tagName)) return node;
    }
    return null;
  }
}
const storage = {};
const ctx = {
  console,
  document: { readyState: 'loading', addEventListener() {}, createElement: tag => new Element(tag) },
  window: { localStorage: {
    getItem: key => storage[key] || null,
    setItem: (key, value) => { storage[key] = value; }
  } }
};
vm.createContext(ctx);
vm.runInContext(fs.readFileSync(path.join(__dirname, '../web/app.js'), 'utf8'), ctx);
const contract = JSON.parse(fs.readFileSync(process.argv[2], 'utf8'));
const groups = [...new Map(contract.cases.map(c => [c.group.id, c.group])).values()];
ctx.state.connection = contract.connection;
ctx.state.selectedGroup = groups[0].id;
ctx.state.trial = { group_id: groups[0].id, mode: 'flight' };
ctx.state.sessions.trial = { state: 'running' };
let renders = 0;
ctx.renderGroups = () => { renders++; };
ctx.renderMonitor = () => {};
ctx.scheduleRender = () => {};
ctx.initTermsFromSnapshot = () => {};
ctx.startTrial = () => { throw new Error('Selecting a card must never start a trial'); };

for (const group of groups) {
  const card = ctx.groupCard(group);
  const descendants = node => [node,...node.children.flatMap(descendants)];
  const controls = descendants(card).filter(node=>node['data-option']);
  for (const [field,capability] of [['motionOptimized','motion_optimization_supported'],
    ['pattern','survey_patterns'],['resume','resume_survey_supported'],['lighting','lighting_options'],['release','release_options']]) {
    assert.strictEqual(controls.some(node=>node['data-option']===field),!!group[capability],`${group.id} ${field} capability`);
  }
  const release = controls.find(node=>node['data-option']==='release');
  if(release && group.release_options.includes('real')) {
    ctx.groupUI(group).realConfirm='实投';ctx.groupUI(group).armedOk=true;
    release.value='mock';release.handlers.change();
    assert.strictEqual(ctx.groupUI(group).realConfirm,'','Changing release resets the typed real confirmation');
    assert.strictEqual(ctx.groupUI(group).armedOk,false,'Changing release resets the flight confirmation');
  }
  const title = card.children[0].children[1];
  card.handlers.click({ target: title });
  assert.strictEqual(ctx.state.selectedGroup, group.id);
  assert.strictEqual(ctx.state.trial.group_id, groups[0].id, 'Running group stays unchanged');
  if (group.id !== groups[0].id) {
    assert.strictEqual(JSON.parse(storage[ctx.PREF_KEY]).group_id, group.id);
  }
}
assert(renders >= groups.length - 1);
const last = groups[groups.length - 1];
ctx.applySnapshot({ groups, trial: { group_id: groups[0].id } });
assert.strictEqual(ctx.state.selectedGroup, last.id, 'SSE snapshot must preserve the choice');
ctx.prefs.group_id = null;
ctx.loadPrefs();
assert.strictEqual(ctx.prefs.group_id, last.id, 'Browser reload must restore the choice');

const card = ctx.groupCard(groups[0]);
const selectButton = card.children[0].children[2];
selectButton.handlers.click();
assert.strictEqual(ctx.state.selectedGroup, groups[0].id, 'Explicit selection button works');

for (const tag of ['button', 'label', 'input', 'details', 'a', 'select', 'textarea']) {
  const control = new Element(tag);
  const nested = new Element('span');
  control.appendChild(nested);
  card.appendChild(control);
  ctx.state.selectedGroup = last.id;
  card.handlers.click({ target: nested });
  assert.strictEqual(ctx.state.selectedGroup, last.id, `Nested ${tag} click must not select a group`);
}
ctx.state.groups = [
  { id: 'running', key: 'RUNNING', manual_mission_start: true },
  { id: 'selected', key: 'SELECTED', manual_mission_start: true }
];
ctx.state.trial = { group_id: 'running', mode: 'flight' };
ctx.state.selectedGroup = 'selected';
const missionSection = ctx.missionStartSection({ name: 'READY' });
const missionButton = missionSection.children[1].children[1].children[0];
assert(missionButton.textContent.includes('RUNNING'));
assert(!missionButton.textContent.includes('SELECTED'));
console.log(`PASS ${groups.length} group selections, persisted snapshots/reload, explicit button, 7 nested controls, running trial unchanged and correct mission label`);
