// Execute the real app and the main agent's wrapper, with no network or DOM startup.
const assert = require('node:assert/strict'), fs = require('node:fs'), vm = require('node:vm');
const data = JSON.parse(fs.readFileSync(0, 'utf8'));
const sandbox = {window: {addEventListener() {}}, document: {readyState: 'loading', addEventListener() {}},
  console, setTimeout, clearTimeout};
vm.createContext(sandbox);
vm.runInContext(fs.readFileSync('web/app.js', 'utf8'), sandbox);
// app.js defines groupCard, required by the latest low-coverage wrapper.
vm.runInContext(fs.readFileSync('web/low-coverage.js', 'utf8'), sandbox);
const g = data.group;
assert.equal(sandbox.groupUI(g).release, 'mock');
assert.equal(sandbox.groupUI(g).slot, '1');
assert.equal(sandbox.usesObservationFlow(g), true);
for (const {body, expected} of data.cases) {
  const ui = sandbox.groupUI(g);
  ui.slot = String(body.slot); ui.release = body.real_release ? 'real' : 'mock';
  const built = sandbox.groupCommandBody(g, body.mode, body.real_release, body.check_config, null, body);
  assert.equal(built.body, expected);
  const request = sandbox.trialBody(g, body.mode, body.check_config);
  assert.equal(request.expected_body, expected);
  assert.equal(request.slot, body.slot);
  assert.equal(request.real_release, body.mode === 'flight' && !body.check_config && body.real_release);
}
for (const slot of [true, '1', 0, 4, '1;bad']) {
  assert.equal(sandbox.groupCommandBody(g, 'flight', false, false, null, {slot}).body, null);
}
for (const options of [{route:'site'}, {site_geometry:{}}, {flight_settings:{}}, {coverage_agl:1.8}]) {
  assert.equal(sandbox.groupCommandBody(g, 'flight', false, false, null, options).body, null);
}
assert.equal(sandbox.groupCommandBody({...g,folder:'08_full_mission'},'flight',false,false,null,{}).body,null);
assert.equal(sandbox.groupCommandBody({channel:'module',folder:'08_full_mission'},'flight',false,false,null,{slot:1}).body,null);
console.log('PASS: 24 slot/mode/release/check combinations; real app + low-coverage wrapper; invalid inputs');
