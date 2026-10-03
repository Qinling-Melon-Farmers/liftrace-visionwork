// No browser dependencies; run the actual frontend command and action functions.
const fs = require('fs');
const vm = require('vm');
const assert = require('assert');
const path = require('path');
const contract = JSON.parse(fs.readFileSync(process.argv[2], 'utf8'));
const ctx = {document: {readyState: 'loading', addEventListener() {}}, window: {}, console};
vm.createContext(ctx);
vm.runInContext(fs.readFileSync(path.join(__dirname, '../web/app.js'), 'utf8'), ctx);
ctx.state.connection = contract.connection;
for (const c of contract.cases) {
  assert.strictEqual(ctx.groupCommandBody(c.group,c.mode,c.real,c.check,c.speed).body,c.command);
}
(async () => {
  const sent = [];
  ctx.toast = () => {};
  ctx.scheduleRender = () => {};
  ctx.confirmModal = () => Promise.resolve({confirmed: true});
  ctx.act = p => p;
  ctx.api.trialStart = body => {sent.push(body); return Promise.resolve({ok:true});};
  for (const c of contract.cases.filter(c => !c.check)) {
    ctx.state.tabs[c.group.id] = {armedOk:true,realConfirm:'实投',speed:c.speed};
    ctx.startTrial(c.group,c.mode);
    await new Promise(resolve => setImmediate(resolve));
    const request = sent[sent.length-1];
    assert.strictEqual(request.expected_body,c.command);
    assert.strictEqual(request.confirm,c.real ? '实投' : '启动试飞');
    assert.strictEqual(request.real_release,c.real);
  }
  const renders = [];
  let focused = true;
  ctx.document.activeElement = {closest: sel => focused && sel === '#groups-body'};
  ctx.window.requestAnimationFrame = fn => fn();
  for (const name of ['renderTopbar','renderGroups','renderTerminals','renderMonitor','renderDrawer']) {
    ctx[name] = () => renders.push(name);
  }
  // Reload only the declared render function to exercise its focus protection.
  const source=fs.readFileSync(path.join(__dirname,'../web/app.js'),'utf8');
  vm.runInContext(source.slice(source.indexOf('function scheduleRender()'),source.indexOf('\nvar bus =')),ctx);
  ctx.scheduleRender();
  assert(!renders.includes('renderGroups'));
  focused=false; ctx.scheduleRender();
  assert(renders.includes('renderGroups'));
  console.log(`PASS ${contract.cases.length} command parity cases, ${sent.length} actual UI requests, IME focus protection`);
})().catch(error => {console.error(error); process.exitCode=1;});
