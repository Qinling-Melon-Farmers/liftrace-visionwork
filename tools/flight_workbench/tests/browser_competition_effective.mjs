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
  await test('native local transport with empty sessions', 'state.connection.transport === "local" && Object.keys(state.sessions).length===0');
  await run('state._es.close(); state._es=null; window.cg=state.groups.find(g=>g.id==="competition");');
  await test('independent template and motion/resume three-state selectors', `
    cg.site_config==='deployment/competition/field.example.yaml' && cg.resume_survey_supported===true &&
    ['motionOptimization','resume','obstacleColumns'].every(k=>{const s=document.querySelector('[data-option="'+k+'"]');return s&&s.options.length===3&&s.value==='';})`);
  await test('selected request alone never claims confirmation', `document.querySelector('#groups-body').textContent.includes('所选配置：') && document.querySelector('#groups-body').textContent.includes('有效值尚未确认')`);
  await run(`for(const [key,value] of [['motionOptimization','off'],['resume','on']]) {const select=document.querySelector('[data-option="'+key+'"]');select.value=value;select.dispatchEvent(new Event('change'));}`);
  await test('motion off and resume on reach independent CLI flags', `trialBody(cg,'preview',true).expected_body.includes('--motion-optimization off --resume-survey on')`);
  await run(`window.options=trialBody(cg,'preview',true);state.trial={group_id:cg.id,competition_config:cg.site_config,motion_optimization:'off',resume_survey:'on'};
    state.stage=Object.assign({},state.stage,{effective_config:{source:'validated_settings',motion_optimization:false,resume_survey:true,generation_ready:false}});renderGroups();`);
  await test('checked state shows actual values and incomplete measurement', `document.querySelector('#groups-body').textContent.includes('离线配置检查：运动优化=关闭；高位续扫=开启') && document.querySelector('#groups-body').textContent.includes('尚不可生成飞行配置')`);
  if(process.argv[4]) {const s=await call('Page.captureScreenshot',{format:'png'});fs.writeFileSync(process.argv[4]+'_checked.png',Buffer.from(s.data,'base64'));}
  await run(`state.stage=Object.assign({},state.stage,{effective_config:{source:'generated_runtime',motion_optimization:false,resume_survey:true,runtime_path:'/logs/offline-fixture/runtime.yaml'}});renderGroups();`);
  await test('generated state shows actual runtime source and path', `document.querySelector('#groups-body').textContent.includes('已生成配置：运动优化=关闭；高位续扫=开启') && document.querySelector('#groups-body').textContent.includes('/logs/offline-fixture/runtime.yaml')`);
  if(process.argv[4]) {const s=await call('Page.captureScreenshot',{format:'png'});fs.writeFileSync(process.argv[4]+'_generated.png',Buffer.from(s.data,'base64'));}
  await run(`const select=document.querySelector('[data-option="resume"]');select.value='off';select.dispatchEvent(new Event('change'));`);
  await test('changing resume clears the old confirmation', `document.querySelector('#groups-body').textContent.includes('有效值尚未确认') && !document.querySelector('#groups-body').textContent.includes('已生成配置：')`);
  await run(`groupUI(cg).resume='on';groupUI(cg).motionOptimization='on';renderGroups();`);
  await test('changing motion clears the old confirmation', `document.querySelector('#groups-body').textContent.includes('有效值尚未确认')`);
  await run(`groupUI(cg).motionOptimization='off';groupUI(cg).competitionConfig='different.yaml';renderGroups();`);
  await test('changing config path clears the old confirmation', `document.querySelector('#groups-body').textContent.includes('有效值尚未确认')`);
  await test('DOM test kept backend empty and offline', `state.connection.transport==='local' && Object.keys(state.sessions).length===0`);
  console.log(JSON.stringify({result:'PASS',checks,board_connected:false,simulated_display_evidence:true}));
} finally {
  if(socket)socket.close();
  if(child.exitCode===null) {
    if(process.platform==='win32') {const {spawnSync}=await import('node:child_process');spawnSync('taskkill',['/PID',String(child.pid),'/T','/F'],{windowsHide:true});}
    else child.kill('SIGTERM');
  }
  const tempRoot=path.resolve(os.tmpdir());const target=path.resolve(profile);
  if(target.startsWith(tempRoot+path.sep)&&path.basename(target).startsWith('liftrace-ui-regression-'))fs.rmSync(target,{recursive:true,force:true});
}
