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
  assert.strictEqual(ctx.groupCommandBody(c.group,c.mode,c.real,c.check,c.speed,c.options).body,c.command, JSON.stringify(c));
}
(async () => {
  const sent = [];
  ctx.toast = () => {};
  ctx.scheduleRender = () => {};
  let modalCount = 0;
  ctx.confirmModal = () => {modalCount++;return Promise.resolve({confirmed: true});};
  ctx.act = p => p;
  ctx.api.trialStart = body => {sent.push(body); return Promise.resolve({ok:true});};
  ctx.state.connection.state = 'ok';
  for (const c of contract.cases) {
    const options = c.options || {};
    ctx.state.tabs[c.group.id] = {armedOk:!c.check,realConfirm:c.check?'':'实投',speed:c.speed,
      release:c.real?'real':'mock',motionOptimized:!!options.motion_optimized,
      pattern:options.survey_pattern,resume:options.resume_survey,lighting:options.capture_lighting,
      motionOptimization:options.motion_optimization,obstacleColumns:options.obstacle_columns,
      competitionConfig:options.competition_config};
    const before = modalCount;
    await ctx.startTrial(c.group,c.mode,c.check);
    const request = sent[sent.length-1];
    assert.strictEqual(request.expected_body,c.command);
    const realFlight=c.real && !c.check && c.mode==='flight';
    assert.strictEqual(request.confirm,c.check?undefined:(realFlight ? '实投' : '启动试飞'));
    assert.strictEqual(request.real_release,realFlight);
    assert.strictEqual(request.mode,c.check?'preview':c.mode);
    for (const field of ['capture_lighting','motion_optimized','survey_pattern','resume_survey',
                         'motion_optimization','obstacle_columns','competition_config']) {
      assert.strictEqual(request[field],options[field] || undefined);
    }
    assert.strictEqual(modalCount-before,c.check?0:1,'Configuration check bypasses flight confirmation');
  }
  const c=contract.cases.find(c=>c.mode==='flight' && !c.check && c.real);
  const count=sent.length;
  ctx.state.tabs[c.group.id]={armedOk:true,realConfirm:'实投',release:'real'};
  ctx.state.sessions.trial={state:'running'};
  await ctx.startTrial(c.group,'flight');
  await ctx.startTrial(c.group,'preview');
  await ctx.startTrial(c.group,'preview',true);
  assert.strictEqual(sent.length,count,'Running trial blocks every new entry');
  ctx.state.sessions.trial={state:'stopped'};
  ctx.state.tabs[c.group.id].realConfirm='';
  await ctx.startTrial(c.group,'flight');
  assert.strictEqual(sent.length,count,'Real release requires the typed confirmation');
  ctx.state.tabs[c.group.id].realConfirm='实投';
  ctx.confirmModal=()=>Promise.resolve({confirmed:false});
  await ctx.startTrial(c.group,'flight');
  assert.strictEqual(sent.length,count,'Cancelled second confirmation issues no request');
  let approve;
  ctx.confirmModal=()=>new Promise(resolve=>{approve=resolve;});
  const pending=ctx.startTrial(c.group,'flight');
  await ctx.startTrial(c.group,'preview',true);
  assert.strictEqual(sent.length,count,'Pending confirmation blocks another entry');
  ctx.state.sessions.trial={state:'running'};
  approve({confirmed:true});await pending;
  assert.strictEqual(sent.length,count,'Trial state is rechecked after confirmation');
  ctx.state.sessions.trial={state:'stopped'};
  ctx.state.connection.state='failed';
  await ctx.startTrial(c.group,'preview',true);
  assert.strictEqual(sent.length,count,'Disconnected check issues no request');
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
  console.log(`PASS ${contract.cases.length} command parity cases, ${sent.length} mocked UI requests, independent config check, running/pending/confirmation guards, IME focus protection`);
})().catch(error => {console.error(error); process.exitCode=1;});
