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
  await test('offline transport', 'state.connection.transport === "local"');
  await run('state._es.close(); state._es=null;');
  await run(`window.g=state.groups.find(x=>x.id==='site5');window.u=groupUI(g);u.geometryEnabled=true;u.surveyEnabled=true;u.pattern='snake3';renderGroups();`);
  await test('search editor available without corridor', `document.querySelector('[data-geometry="surveyBounds"]') && !document.querySelector('[data-geometry="geometryPoints"]')`);
  await run(`const entries={surveyBounds:'0.6,5.5,-1.1,1.1',surveyInset:'0.35',surveyCameraHeight:'1.84',surveyOffset:'-0.16',surveyFovX:'75',surveyFovY:'60'};Object.entries(entries).forEach(([key,value])=>{const e=document.querySelector('[data-geometry="'+key+'"]');e.value=value;e.dispatchEvent(new Event('input',{bubbles:true}));});window.findApply=()=>Array.from(document.querySelectorAll('button')).find(b=>b.textContent.includes('生成并预览坐标命令'));findApply().click();`);
  for(let i=0;i<70;i++){if(await run('Object.keys(u.geometryPlans||{}).length===3 && !!u.surveyPreview'))break;await sleep(100);}
  await test('generates three commands and coverage', 'Object.keys(u.geometryPlans||{}).length===3 && !!u.surveyPreview');
  await test('lens height converted once', 'Math.abs(u.surveyPreview.high_agl-2)<1e-8 && u.surveyPreview.route.length===6');
  await test('flight command includes generated shape', `trialBody(g,'flight',false).expected_body.includes('--survey-pattern snake3')`);
  await test('coverage chart drawn', `!!document.querySelector('svg[aria-label="搜索航线与理想视场覆盖"] polyline') && document.querySelector('#groups-body').textContent.includes('理想覆盖')`);
  await test('no SSH or flight created', `state.connection.transport==='local' && !state.trial.group_id && !state.sessions.trial`);
  await run(`const e=document.querySelector('[data-geometry="surveyCameraHeight"]');e.value='2.0';e.dispatchEvent(new Event('input',{bubbles:true}));`);
  await test('editing clears old coverage and command', '!u.surveyPreview && !trialBody(g,"flight",false).expected_body');
  await run(`state.selectedGroup='mod08';window.g=state.groups.find(x=>x.id==='mod08');window.u=groupUI(g);u.geometryEnabled=true;u.surveyEnabled=true;renderGroups();`);
  await test('full mission retains measured corridor input', `!!document.querySelector('[data-geometry="geometryPoints"]') && !!document.querySelector('[data-geometry="surveyBounds"]')`);
  console.log(`${checks.length} survey browser checks passed; no SSH/ROS/flight`);
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
