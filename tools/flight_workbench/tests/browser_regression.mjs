// Real Chromium DOM regression; no packages, SSH, ROS or flight actions.
// node tests/browser_regression.mjs http://127.0.0.1:8793 [chromium-path]
import fs from 'node:fs';
import os from 'node:os';
import path from 'node:path';
import {spawn} from 'node:child_process';
import assert from 'node:assert/strict';
const url = process.argv[2] || 'http://127.0.0.1:8793';
const browser = process.argv[3] || process.env.BROWSER_BIN;
if (!browser) throw new Error('Pass BROWSER_BIN or an explicit Chromium executable');
const profile = fs.mkdtempSync(path.join(os.tmpdir(), 'liftrace-ui-regression-'));
const child = spawn(browser, ['--headless=new', '--disable-gpu', '--no-first-run',
  '--no-default-browser-check', '--remote-debugging-port=0', '--user-data-dir='+profile, 'about:blank'],
  {windowsHide: true, stdio: 'ignore'});
const sleep = ms => new Promise(r => setTimeout(r, ms));
let socket, seq=0;
const pending = new Map();
const checks=[];
try {
  let port;
  for(let i=0;i<120;i++) {
    try { port=Number(fs.readFileSync(path.join(profile,'DevToolsActivePort'),'utf8').split('\n')[0]); break; } catch {}
    if(child.exitCode!==null) throw new Error('Browser exited before CDP was ready');
    await sleep(100);
  }
  assert(port, 'CDP startup timeout');
  const pages=await (await fetch(`http://127.0.0.1:${port}/json/list`)).json();
  socket=new WebSocket(pages.find(p=>p.type==='page').webSocketDebuggerUrl);
  await new Promise((ok, fail)=>{socket.onopen=ok;socket.onerror=fail;});
  socket.onmessage=e=>{const m=JSON.parse(e.data);if(!m.id)return;const p=pending.get(m.id);if(!p)return;
    pending.delete(m.id);clearTimeout(p.timer);m.error?p.fail(new Error(JSON.stringify(m.error))):p.ok(m.result);};
  const call=(method,params={})=>new Promise((ok,fail)=>{
    const id=++seq;pending.set(id,{ok,fail,timer:setTimeout(()=>{pending.delete(id);fail(new Error(method+' timeout'));},10000)});
    socket.send(JSON.stringify({id,method,params}));
  });
  const run=async expression=>{
    const r=await call('Runtime.evaluate',{expression,returnByValue:true,awaitPromise:true});
    if(r.exceptionDetails) throw new Error(JSON.stringify(r.exceptionDetails));
    return r.result.value;
  };
  const test=async(name,expression)=>{assert.equal(await run(expression),true,name);checks.push(name);console.log('PASS '+name);};
  await call('Emulation.setDeviceMetricsOverride',{width:1600,height:1000,deviceScaleFactor:1,mobile:false});
  await call('Page.navigate',{url});
  for(let i=0;i<60;i++){if(await run('typeof state!=="undefined" && state.groups.length>=9'))break;await sleep(100);}
  await test('offline transport (no flight backend)', 'state.connection.transport === "local"');
  await test('realtime observer and logs use this origin; motor big page is removed', `
    ['/observe','/logs'].every(path=>{const a=document.querySelector('a[href="'+path+'"]');return a&&a.target==='_blank'&&a.rel.includes('noopener')&&a.origin===location.origin;}) &&
    !document.querySelector('a[href="/motor"]')`);
  await test('competition defaults to the independent template and has three-state controls', `
    state.groups.find(g=>g.id==='competition').site_config==='deployment/competition/field.example.yaml' &&
    ['motionOptimization','resume','obstacleColumns'].every(k=>{const s=document.querySelector('[data-option="'+k+'"]');return s&&s.options.length===3&&s.value==='';}) &&
    ['FC高位2.6m','走廊前视0.6/0.4m','投递FC AGL 0.35m','FC软件限高3.2m'].every(value=>
      [...document.querySelectorAll('.grp-card')].find(card=>
        card.querySelector('.grp-title').textContent===state.groups.find(g=>g.id==='competition').name).textContent.includes(value))`);
  await run(`window.cg=state.groups.find(g=>g.id==='competition');
    groupUI(cg).motionOptimization='off';groupUI(cg).obstacleColumns='on';renderGroups();`);
  await test('competition switches reach the actual check command', `
    trialBody(cg,'preview',true).expected_body.includes('--motion-optimization off --obstacle-columns on --check-config') &&
    trialBody(cg,'preview',true).motion_optimization==='off' && trialBody(cg,'preview',true).obstacle_columns==='on'`);
  await run(`groupUI(cg).motionOptimization='';groupUI(cg).obstacleColumns='';renderGroups();`);
  await test('switch inheritance omits overrides rather than displaying an enabled state', `
    !trialBody(cg,'preview',true).expected_body.includes('--motion-optimization') &&
    !trialBody(cg,'preview',true).expected_body.includes('--obstacle-columns')`);
  await run(`window.preset=document.querySelector('[data-option="competitionPreset"]');
    preset.value='deployment/competition/field_20261007_validated.yaml';preset.dispatchEvent(new Event('change'));`);
  await test('test reproduction is explicitly optional and never the default competition config', `
    trialBody(cg,'preview',true).expected_body.includes('field_20261007_validated.yaml') &&
    document.querySelector('#groups-body').textContent.includes('当前选择测试场地复现') &&
    cg.site_config==='deployment/competition/field.example.yaml' && !groupUI(cg).armedOk`);
  await run(`document.querySelector('[data-option="competitionPreset"]').value=cg.site_config;
    document.querySelector('[data-option="competitionPreset"]').dispatchEvent(new Event('change'));`);
  if(process.argv[4]) {
    const shot=await call('Page.captureScreenshot',{format:'png',captureBeyondViewport:false});
    fs.writeFileSync(process.argv[4],Buffer.from(shot.data,'base64'));
  }
  await run('state._es.close(); state._es=null; state.activeTerm="trial"; renderTerminals(); renderDrawer();');
  await run(`window.originalConnection=JSON.parse(JSON.stringify(state.connection));
    window.hostOptionCount=state.connection.host_options.length;
    bus.dispatch({t:'connection',connection:{state:'failed',detail:'network unavailable'}});`);await sleep(100);
  await test('partial connection updates retain the SSH address dropdown', `
    hostOptionCount>1 && document.querySelector('#host-select').options.length===hostOptionCount+1 &&
    state.connection.board_root===originalConnection.board_root && state.connection.transport==='local'`);
  await run(`document.querySelector('#btn-connect').click();`);
  await test('connection dialog exposes history, custom address, username, port and password', `
    ['host','host_custom','user','port','password'].every(k=>document.querySelector('[data-field="'+k+'"]')) &&
    document.querySelector('[data-field="host"]').options.length===hostOptionCount+1 &&
    document.querySelector('[data-field="password"]').type==='password'`);
  await run(`window.sentConnection=[]; window.oldConfig=api.config; window.oldConnect=api.connect;
    api.config=async b=>{sentConnection.push({kind:'config',body:b});return {ok:true,connection:{host:b.host,port:b.port}};};
    api.connect=async b=>{sentConnection.push({kind:'connect',body:b});return {ok:true,connection:{state:'failed',detail:'stub'}};};
    document.querySelector('#modal-root .btn-primary').click();`);await sleep(100);
  await test('offline preview never initiates an SSH connection', 'sentConnection.length===0');
  await run(`state.connection.transport='ssh';document.querySelector('#btn-connect').click();
    document.querySelector('[data-field="host"]').value=state.connection.host_options[1].host;
    document.querySelector('[data-field="password"]').value='ui-test-only-password';
    document.querySelector('#modal-root .btn-primary').click();`);await sleep(100);
  await test('history address and password are passed only to the mocked connection APIs', `
    sentConnection.length===2 && sentConnection[0].body.host===originalConnection.host_options[1].host &&
    sentConnection[1].body.password==='ui-test-only-password' && !sentConnection[1].body.save_password &&
    !JSON.stringify(localStorage).includes('ui-test-only-password') &&
    document.querySelector('#host-select').options.length===hostOptionCount+1`);
  await run(`document.querySelector('#host-select').value='';
    document.querySelector('#host-select').dispatchEvent(new Event('change'));
    document.querySelector('[data-field="host_custom"]').value='192.0.2.10';
    document.querySelector('[data-field="user"]').value='pilot';
    document.querySelector('[data-field="port"]').value='2222';
    document.querySelector('#modal-root .btn-primary').click();`);await sleep(100);
  await test('custom address uses the entered username and port without auto-saving a password', `
    sentConnection.length===4 && sentConnection[2].body.host==='pilot@192.0.2.10' &&
    sentConnection[2].body.port===2222 && !('password' in sentConnection[3].body)`);
  await run(`document.querySelector('#btn-connect').click();
    document.querySelector('[data-field="port"]').value='0';
    document.querySelector('#modal-root .btn-primary').click();`);await sleep(100);
  await test('invalid connection settings issue no request', 'sentConnection.length===4');
  await run(`api.config=oldConfig;api.connect=oldConnect;state.connection=originalConnection;renderTopbar();`);
  await test('trial terminal and log mirror are simultaneously mounted', `
    state.terms.trial.scrollEl!==state.logTerm.scrollEl &&
    document.querySelector('#term-body').contains(state.terms.trial.scrollEl) &&
    document.querySelector('#drawer-body').contains(state.logTerm.scrollEl)`);
  await run('bus.onOut("trial","FLIGHT_STATUS panzer\\nREADY bridge\\n"); bus.onOut("lidar","LIDAR_ONLY\\n");');await sleep(100);
  await test('trial output is visible in both views without unrelated device logs', `
    [state.terms.trial,state.logTerm].every(t=>t.scrollEl.textContent.includes('panzer') && !t.scrollEl.textContent.includes('LIDAR_ONLY'))`);
  await run(`window.filterInput=document.querySelector('#drawer-tools input');filterInput.focus();
    filterInput.value='panzer';filterInput.dispatchEvent(new Event('input',{bubbles:true}));scheduleRender();`);await sleep(100);
  await test('filter retains focus across telemetry render and affects only the log view', `
    document.activeElement===filterInput && document.querySelector('#drawer-tools input')===filterInput &&
    state.logTerm.scrollEl.textContent.includes('panzer') && !state.logTerm.scrollEl.textContent.includes('bridge') &&
    state.terms.trial.scrollEl.textContent.includes('bridge')`);
  await run('bus.onOut("trial","panzer new\\nNO_MATCH\\n");');await sleep(100);
  await test('new filtered output appears without losing the filter', `state.logTerm.scrollEl.textContent.includes('panzer new') && !state.logTerm.scrollEl.textContent.includes('NO_MATCH')`);
  await run(`filterInput.value='unmatched';filterInput.dispatchEvent(new Event('input',{bubbles:true}));`);
  await test('zero-match filter shows no unrelated lines', 'state.logTerm.scrollEl.textContent === ""');
  await run(`document.querySelector('#drawer-tools button').click();`);
  await test('clearing filter restores retained history', `state.logTerm.scrollEl.textContent.includes('bridge') && state.logTerm.scrollEl.textContent.includes('NO_MATCH')`);
  await run(`window.groupBody=document.querySelector('#groups-body'); groupBody.scrollTop=900;
    window.groupTop=groupBody.scrollTop; state.selectedGroup=state.groups[4].id; renderGroups();`);
  await test('group selection redraw keeps the scroll offset', 'Math.abs(groupBody.scrollTop-groupTop)<2');
  await run(`window.card=[...document.querySelectorAll('.grp-card')][4];
    [...card.querySelectorAll('button')].find(b=>b.textContent==='飞行 flight').click();`);
  await test('switching preview/flight keeps the group scroll offset', 'Math.abs(groupBody.scrollTop-groupTop)<2');
  await run(`window.confirmInput=[...document.querySelectorAll('.grp-card')][4].querySelector('input[type=text]');
    confirmInput.focus();confirmInput.value='实投';confirmInput.dispatchEvent(new Event('input',{bubbles:true}));
    window.editingTop=groupBody.scrollTop;scheduleRender();`);await sleep(100);
  await test('parameter editing retains focus, value and scroll', `document.activeElement===confirmInput && confirmInput.value==='实投' && Math.abs(groupBody.scrollTop-editingTop)<2`);
  await run(`document.activeElement.blur();window.oldTrialStart=api.trialStart;
    window.stubTrials=[];window.stubModals=[];window.oldConfirmModal=confirmModal;
    api.trialStart=async b=>{stubTrials.push(b);return {ok:true};};
    confirmModal=async(...args)=>{stubModals.push(args);return {confirmed:true};};
    state.connection.state='ok';state.sessions.trial={state:'stopped'};renderGroups();`);
  await test('original site IDs and module aliases remain clickable', `
    ['site1','site2','site3','site4','site5','site6','mod03','mod04','mod08','mod06mock'].every(id=>state.groups.some(g=>g.id===id)) &&
    document.querySelectorAll('.grp-card').length===state.groups.length`);
  await test('resume controls match independent competition and groups 06/08', `
    [...document.querySelectorAll('.grp-card')].every((card,i)=>
      !!card.querySelector('[data-option="resume"]')===(state.groups[i].id==='competition' || ['06_high_priority','08_full_mission'].includes(state.groups[i].folder)))`);
  await run(`window.captureGroup=state.groups.find(g=>g.folder==='09_high_speed_capture');
    Object.assign(groupUI(captureGroup),{speed:1.2,lighting:'dim',motionOptimized:true,pattern:'snake3'});
    renderGroups();window.captureCard=[...document.querySelectorAll('.grp-card')].find(c=>c.querySelector('.grp-title').textContent===captureGroup.name);
    window.patternSelect=captureCard.querySelector('[data-option="pattern"]');patternSelect.focus();scheduleRender();`);await sleep(100);
  await test('new route control retains focus and selected value on telemetry refresh', `
    document.activeElement===patternSelect && patternSelect.value==='snake3'`);
  await test('capture command includes speed lighting motion and route in backend order', `
    captureCard.querySelector('.cmd-pre').textContent.includes('--capture-speed 1.2 --capture-lighting dim --motion-optimized --survey-pattern snake3')`);
  await run(`startTrial(captureGroup,'flight',true)`);
  await test('independent config check skips flight confirmation and only sends preview/check', `
    stubTrials.length===1 && stubModals.length===0 && stubTrials[0].mode==='preview' &&
    stubTrials[0].check_config && !stubTrials[0].real_release && !stubTrials[0].confirm &&
    stubTrials[0].expected_body.endsWith('--check-config')`);
  await run(`(async()=>{state.sessions.trial={state:'running'};renderGroups();
    await startTrial(captureGroup,'flight');await startTrial(captureGroup,'preview');await startTrial(captureGroup,'preview',true);})()`);
  await test('running trial blocks every start action while cards remain browsable', `
    stubTrials.length===1 && [...document.querySelectorAll('.grp-card')].every(c=>
      [...c.querySelectorAll('button')].filter(b=>['预览（preview）','飞行（flight）','配置检查（不启动节点）'].includes(b.textContent)).every(b=>b.disabled)) &&
    [...document.querySelectorAll('.grp-select')].some(b=>!b.disabled)`);
  await run(`(async()=>{state.sessions.trial={state:'stopped'};window.releaseGroup=state.groups.find(g=>g.id==='site1');
    Object.assign(groupUI(releaseGroup),{mode:'flight',release:'mock',armedOk:true,realConfirm:''});
    await startTrial(releaseGroup,'flight');
    groupUI(releaseGroup).release='real';await startTrial(releaseGroup,'flight');})()`);
  await test('mock flight works and real flight cannot bypass typed confirmation', `
    stubTrials.length===2 && stubTrials[1].real_release===false && !stubTrials[1].expected_body.includes('start_real.sh')`);
  await run(`(async()=>{groupUI(releaseGroup).realConfirm='实投';await startTrial(releaseGroup,'flight');})()`);
  await test('real flight uses the real entry after both confirmations', `
    stubTrials.length===3 && stubTrials[2].real_release && stubTrials[2].confirm==='实投' &&
    stubTrials[2].expected_body.includes('start_real.sh') && stubModals.length===2`);
  await run(`state.stage=Object.assign({},state.stage,{outcome:'aborted',pilot_action:'等待人工拨入OFFBOARD'});
    state.telemetry=Object.assign({},state.telemetry,{terminal_hover:null,lio_realtime:[]});renderMonitor();`);
  await test('aborted outcome and missing LIO/hover observations are explicit', `
    document.querySelector('#monitor-body').textContent.includes('任务中止') &&
    document.querySelector('#monitor-body').textContent.includes('等待人工拨入OFFBOARD') &&
    document.querySelector('#monitor-body').textContent.includes('未观测')`);
  await run(`state.telemetry.at=Date.now()/1000;state.telemetry.master=true;state.telemetry.probe_link={status:'live',usable:true};
    state.sessions.probe={state:'running',exit_code:null};state.telemetry.terminal_hover=JSON.stringify({stage:'PILOT_HANDOFF'});
    state.telemetry.lio_realtime=[{name:'FAST-LIO',level:1,message:'queue growing',
      values:{output_age_sec:'0.31',lidar_queue:'4',imu_queue:'8'}}];renderMonitor();`);
  await test('observed terminal handoff and LIO metrics are displayed', `
    ['交给飞手落地','output_age_sec=0.31','lidar_queue=4','imu_queue=8'].every(v=>document.querySelector('#monitor-body').textContent.includes(v))`);
  await run(`state.telemetry.at=Date.now()/1000-10;state.telemetry.state={connected:true,armed:false,mode:'OFFBOARD'};renderMonitor();`);
  await test('stale telemetry suppresses cached flight state hover and LIO observations', `
    document.querySelector('#monitor-body').textContent.includes('已过期') &&
    !document.querySelector('#monitor-body').textContent.includes('connected=true') &&
    !document.querySelector('#monitor-body').textContent.includes('output_age_sec=0.31') &&
    !document.querySelector('#monitor-body').textContent.includes('交给飞手落地')`);
  await run(`state.telemetry.at=Date.now()/1000;state.telemetry.state={};renderMonitor();`);
  await test('missing flight state fields stay unobserved instead of false', `
    document.querySelector('#monitor-body').textContent.includes('connected=未观测 armed=未观测 mode=未观测')`);
  await run(`api.trialStart=oldTrialStart;confirmModal=oldConfirmModal;document.activeElement.blur();renderGroups();`);
  await run(`document.activeElement.blur(); window.monitor=document.querySelector('#monitor-body');
    monitor.scrollTop=350;window.monitorTop=monitor.scrollTop;renderMonitor();scheduleRender();`);await sleep(100);
  await test('status panel refresh does not scroll back to the top', 'monitorTop>0 && Math.abs(monitor.scrollTop-monitorTop)<2');
  await run(`state.terms.trial.clearView(); state.logTerm.clearView();
    bus.onOut('trial', Array.from({length:450},(_,i)=>'flight row '+i).join('\\n')+'\\n');`);await sleep(150);
  await run(`bus.onOut('trial','flight row 450\\n');`);await sleep(100);
  await test('output beyond 200 lines retains first and last rows without duplication', `
    state.terms.trial.scrollEl.children.length===state.terms.trial.buf.length &&
    state.terms.trial.scrollEl.children[0].textContent==='flight row 0' &&
    state.terms.trial.scrollEl.textContent.includes('flight row 450')`);
  await run('state.terms.trial.scrollEl.scrollTop=120;');await sleep(100);
  await run(`window.historyTop=state.terms.trial.scrollEl.scrollTop;
    bus.onOut('trial','appended while reading\\n'); scheduleRender();`);await sleep(150);
  await test('reading terminal history survives incoming output and telemetry', 'Math.abs(state.terms.trial.scrollEl.scrollTop-historyTop)<2');
  await run(`state.terms.trial.setAutoScroll(true);bus.onOut('trial','follow tail\\n');`);await sleep(100);
  await test('explicit auto-scroll returns to the latest output', `(()=>{let e=state.terms.trial.scrollEl;return e.scrollHeight-e.clientHeight-e.scrollTop<2})()`);
  await run(`window.limitTerm=new AnsiTerm({id:'bounded-test',maxLines:10});limitTerm.append('first');`);await sleep(60);
  await run(`limitTerm.append(' + more\\n');`);await sleep(60);
  await test('updating the first partial line does not duplicate it', `limitTerm.scrollEl.children[0].textContent==='first + more' && limitTerm.scrollEl.children.length===2`);
  await run(`limitTerm.append(Array.from({length:20},(_,i)=>'bounded '+i).join('\\n'));`);await sleep(60);
  await test('bounded terminal trimming preserves valid DOM indices', 'limitTerm.buf.length===10 && limitTerm.scrollEl.children.length===10 && limitTerm.scrollEl.lastChild.textContent==="bounded 19"');
  await run(`state.activeTerm='servo_init';
    state.sessions.servo_init={id:'servo_init',state:'exited',exit_code:0,started_at:1791350138.4908187,ended_at:1791350143.9172504};
    window.beforeInitStage=state.stage.name;
    state.orchestration={running:false,steps:[{id:'servo_init',title:'5a fixture',state:'pending'}]};
    renderTerminals();renderMonitor();`);
  await test('one-shot 5a exit zero shows a green software initialization success', `
    document.querySelector('.ttab.active .dot').classList.contains('succeeded') &&
    (()=>{let reference=document.createElement('span');reference.style.color='var(--green)';document.body.appendChild(reference);
      let expected=getComputedStyle(reference).color;reference.remove();
      return getComputedStyle(document.querySelector('.ttab.active .dot')).backgroundColor===expected;})() &&
    document.querySelector('#term-body .term-meta').textContent.includes('初始化成功') &&
    document.querySelector('#term-body').textContent.includes('不代表舵机物理动作反馈')`);
  await test('5a orchestration step shares succeeded class without changing mission stage', `
    document.querySelector('.step.succeeded .sdot') &&
    getComputedStyle(document.querySelector('.step.succeeded .sdot')).backgroundColor===getComputedStyle(document.querySelector('.ttab.active .dot')).backgroundColor &&
    document.querySelector('.step.succeeded').textContent.includes('初始化成功') && state.stage.name===beforeInitStage`);
  await run(`bus.dispatch({t:'session',s:'servo_init',session:{id:'servo_init',state:'failed',exit_code:1}});`);await sleep(100);
  await test('nonzero 5a event shows red initialization failure', `
    document.querySelector('.ttab.active .dot').classList.contains('failed') &&
    document.querySelector('#term-body .term-meta').textContent.includes('初始化失败')`);
  await test('5a orchestration failure uses the same failed color class', `
    document.querySelector('.step.failed .sdot') &&
    getComputedStyle(document.querySelector('.step.failed .sdot')).backgroundColor===getComputedStyle(document.querySelector('.ttab.active .dot')).backgroundColor`);
  await run(`state.sessions.servo_init={state:'running',exit_code:null};renderTerminals();`);
  await test('running initialization stays pending rather than success', `
    document.querySelector('.ttab.active .dot').classList.contains('starting') &&
    document.querySelector('#term-body .term-meta').textContent.includes('初始化中')`);
  await run(`delete state.sessions.servo_init;renderTerminals();`);
  await test('never-run 5a is idle and labeled not initialized', `
    document.querySelector('.ttab.active .dot').classList.contains('idle') &&
    document.querySelector('#term-body .term-meta').textContent.includes('未初始化')`);
  await run(`state.activeTerm='servo';state.sessions.servo={state:'exited',exit_code:0};renderTerminals();`);
  await test('persistent 5b exiting zero does not get initialization success', `
    document.querySelector('.ttab.active .dot').classList.contains('exited') &&
    !document.querySelector('.ttab.active .dot').classList.contains('succeeded') &&
    document.querySelector('#term-body .term-meta').textContent.includes('已退出')`);
  console.log(`${checks.length} browser checks passed; no SSH/ROS/flight requests sent`);
} finally {
  if(socket && socket.readyState===WebSocket.OPEN) {
    socket.send(JSON.stringify({id:++seq,method:'Browser.close'}));
    await sleep(300);
    socket.close();
  }
  child.kill();
  await sleep(1000);
  // Only remove the unique temp profile created above, never an existing user profile.
  if(path.dirname(path.resolve(profile))===path.resolve(os.tmpdir()) && path.basename(profile).startsWith('liftrace-ui-regression-')) {
    try{fs.rmSync(profile,{recursive:true,force:true,maxRetries:5,retryDelay:200});}catch{}
  }
}
