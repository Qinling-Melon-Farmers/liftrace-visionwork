// Read-only observer regression using Chromium's built-in CDP; no dependencies.
// node browser_observe.mjs http://127.0.0.1:8798 <chromium-path> [screenshot-path]
import fs from 'node:fs';import os from 'node:os';import path from 'node:path';
import {spawn} from 'node:child_process';import assert from 'node:assert/strict';
const base=process.argv[2]||'http://127.0.0.1:8798',browser=process.argv[3]||process.env.BROWSER_BIN;
if(!browser)throw new Error('Pass a Chromium executable');
const profile=fs.mkdtempSync(path.join(os.tmpdir(),'liftrace-observer-regression-'));
const child=spawn(browser,['--headless=new','--disable-gpu','--no-first-run','--no-default-browser-check','--remote-debugging-port=0','--user-data-dir='+profile,'about:blank'],{windowsHide:true,stdio:'ignore'});
const sleep=ms=>new Promise(r=>setTimeout(r,ms));const pending=new Map(),requests=[],checks=[];let socket,seq=0;
try{
  let port;
  for(let i=0;i<120;i++){try{port=Number(fs.readFileSync(path.join(profile,'DevToolsActivePort'),'utf8').split('\n')[0]);break;}catch{}
    if(child.exitCode!==null)throw new Error('Browser exited');await sleep(100);}
  assert(port,'CDP timeout');const pages=await(await fetch(`http://127.0.0.1:${port}/json/list`)).json();
  socket=new WebSocket(pages.find(p=>p.type==='page').webSocketDebuggerUrl);
  await new Promise((ok,fail)=>{socket.onopen=ok;socket.onerror=fail;});
  socket.onmessage=e=>{const m=JSON.parse(e.data);if(m.method==='Network.requestWillBeSent')requests.push(m.params.request);
    if(!m.id)return;const p=pending.get(m.id);if(!p)return;pending.delete(m.id);clearTimeout(p.timer);m.error?p.fail(new Error(JSON.stringify(m.error))):p.ok(m.result);};
  const call=(method,params={})=>new Promise((ok,fail)=>{const id=++seq;pending.set(id,{ok,fail,timer:setTimeout(()=>{pending.delete(id);fail(new Error(method+' timeout'));},10000)});socket.send(JSON.stringify({id,method,params}));});
  const run=async expression=>{const r=await call('Runtime.evaluate',{expression,returnByValue:true,awaitPromise:true});if(r.exceptionDetails)throw new Error(JSON.stringify(r.exceptionDetails));return r.result.value;};
  const test=async(name,expression)=>{assert.equal(await run(expression),true,name);checks.push(name);console.log('PASS '+name);};
  const navigate=async route=>{await call('Page.navigate',{url:base+route});for(let i=0;i<70;i++){
    if(await run('typeof observationState!=="undefined" && observationState.configuration.profiles.length===3'))return;await sleep(100);}throw new Error(route+' did not load');};
  await call('Network.enable');await call('Emulation.setDeviceMetricsOverride',{width:1680,height:1100,deviceScaleFactor:1,mobile:false});
  await navigate('/motor');
  await test('motor page is a large read-only page with three configured labels', `document.body.classList.contains('motor-page') && document.querySelector('#profile-select').options.length===3 && document.querySelector('#page-title').textContent.includes('电机')`);
  await test('default mapping is unverified and page never includes camera elements', `observationState.mapping===null && [...document.querySelectorAll('[data-motor]')].every(s=>s.value==='') && !document.querySelector('img,video')`);
  await test('disconnected observer directs the user back to the main workbench', `document.querySelector('#connection-status').textContent.includes('回主页连接') && !document.querySelector('button[id*="flight"],button[id*="connect"]')`);
  await run(`observationState.eventSource.close();observationState.stream='fixture';observationState.connection={state:'ok',host:'fixture-only'};
    window.makeTelemetry=(at,overrides={})=>({at,state:{connected:true,armed:true,mode:'OFFBOARD'},observe:{
      fc_pose:{frame:'map',stamp:at,source_age:.05,x:.1,y:.2,z:2.8,roll_deg:1,pitch_deg:2,yaw_deg:3},
      lio_pose:{frame:'camera_init',stamp:at,source_age:.1,x:1,y:2,z:3,roll_deg:4,pitch_deg:5,yaw_deg:6},
      ev_pose:{frame:'ev_frame',stamp:at,source_age:.05,x:4,y:5,z:6},setpoint:{frame:'camera_init',source_age:.1,x:1,y:2,z:3},
      battery:{voltage:16,current:2,percentage:.6},rc_out:{channels:[1100,1200,1300,1400,1500,1600]},
      esc_telemetry:{entries:[{index:null,slot:0,rpm:800,current:3,temperature:45}]},
      low_hover:{stage:'FINISHED_HOVER',reason:'route complete',profile:'hover',index:0,pose:[1,2,3],target:[1,2,3],recorder_alive:true}},
      mission:{phase:'RETURN',reason:'fixture mission'},terminal_hover:{stage:'PILOT_HANDOFF'},
      lio_realtime:[{name:'LIO',level:0,message:'fixture',values:{output_age_sec:'0.12'}}],...overrides});
    obsEvent({t:'telemetry',telemetry:makeTelemetry(Date.now()/1000)});`);
  await test('raw channels are all shown without assuming motor order', `document.querySelectorAll('.raw-channel').length===6 && document.querySelector('#mapping-status').textContent.includes('未核实') && document.querySelector('#legend-motors').children.length===0`);
  await test('ESC slot is clearly unmapped and FC/LIO/EV preserve their frames', `document.querySelector('#esc-table').textContent.includes('ESC条目 1（物理映射未核实）') && ['map','camera_init','ev_frame'].every(f=>document.querySelector('#status-grid').textContent.includes(f))`);
  await test('manual-node observation shows mission and handoff without claiming a managed trial', `
    ['RETURN','fixture mission','PILOT_HANDOFF','外部/手动节点观察'].every(v=>document.querySelector('#status-grid').textContent.includes(v))`);
  await run(`obsEvent({t:'trial',trial:{group_id:'site1',name:'fixture trial'}});obsEvent({t:'stage',stage:{name:'IN_FLIGHT'}});
    obsEvent({t:'session',s:'trial',session:{state:'running'}});`);
  await test('stage trial and session events expose the workbench-managed session', `
    ['本工作台自管会话','fixture trial','IN_FLIGHT'].every(v=>document.querySelector('#status-grid').textContent.includes(v))`);
  await run(`obsEvent({t:'session',s:'trial',session:{state:'stopped'}});`);
  await run(`for(let i=1;i<=4;i++){const s=document.querySelector('[data-motor="'+i+'"]');s.value=String([4,2,1,3][i-1]);s.dispatchEvent(new Event('change'));}
    document.querySelector('#confirm-mapping').click();`);
  await test('four explicit channel selections and confirmation enable command curves', `JSON.stringify(observationState.mapping)==='[4,2,1,3]' && document.querySelector('#legend-motors').children.length===4 && document.querySelector('#legend-motors').textContent.includes('raw4')`);
  await run(`window.focusedMapping=document.querySelector('[data-motor="1"]');focusedMapping.focus();window.savedScroll=window.scrollY;obsRender();`);await sleep(1100);
  await test('mapping focus and scroll survive periodic observer refresh', `document.activeElement===focusedMapping && document.querySelector('[data-motor="1"]')===focusedMapping && Math.abs(scrollY-savedScroll)<2`);
  await run(`document.activeElement.blur();document.querySelector('#profile-select').value='hover';document.querySelector('#segment-start').click();
    window.fixtureAt=Date.now()/1000;for(let i=0;i<5;i++)obsIngest(makeTelemetry(fixtureAt+i*.001),fixtureAt+i*.001);
    document.querySelector('#segment-end').click();`);
  await test('local segment captures samples and mapping without starting a flight', `observationState.segments.length===1 && obsSummary(observationState.segments[0]).sample_count===5 && JSON.stringify(observationState.segments[0].mapping)==='[4,2,1,3]' && document.querySelector('#segment-table').textContent.includes('相对四路均值偏差')`);
  await run(`for(const tag of ['forward','square']){document.querySelector('#profile-select').value=tag;document.querySelector('#profile-select').dispatchEvent(new Event('change'));document.querySelector('#segment-start').click();
    fixtureAt+=.01;obsIngest(makeTelemetry(fixtureAt),fixtureAt);document.querySelector('#segment-end').click();}`);
  await test('hover forward and square appear in the local comparison table', `observationState.segments.length===3 && ['hover','forward','square'].every(p=>document.querySelector('#segment-table').textContent.includes(p))`);
  if(process.argv[4]){const clip=await run(`(()=>{const m=document.querySelector('#motor-panel').getBoundingClientRect(),r=document.querySelector('#record-panel').getBoundingClientRect();return {x:0,y:m.top+scrollY,width:document.documentElement.clientWidth,height:r.bottom-m.top,scale:1};})()`);
    const shot=await call('Page.captureScreenshot',{format:'png',captureBeyondViewport:true,clip});fs.writeFileSync(process.argv[4],Buffer.from(shot.data,'base64'));}
  await test('JSON and CSV keep local Z frame and raw output meaning', `(()=>{const x=JSON.parse(obsJSON());return x.samples.every(s=>s.data.fc_pose.z===2.8 && s.data.lio_pose.frame==='camera_init') && !JSON.stringify(x).includes('height_agl') && obsCSV().includes('rc_out_raw_json') && obsCSV().includes('lio_pose_frame');})()`);
  await run(`window.oldDownload=obsDownload;window.downloads=[];obsDownload=(content,type,ext)=>downloads.push({content,type,ext});document.querySelector('#export-json').click();document.querySelector('#export-csv').click();`);
  await test('download buttons only create local exports', `downloads.length===2 && downloads[0].ext==='json' && downloads[1].ext==='csv'`);
  await run(`observationState.configuration.max_samples=3;for(let i=0;i<6;i++){fixtureAt+=.001;obsIngest(makeTelemetry(fixtureAt),fixtureAt);}obsRender();`);
  await test('history and old segment comparison show explicit truncation', `observationState.samples.length===3 && observationState.dropped>0 && document.querySelector('#record-status').textContent.includes('截断') && document.querySelector('#segment-table').textContent.includes('已截断')`);
  await run(`observationState.telemetry=makeTelemetry(Date.now()/1000);observationState.telemetry.observe.fc_pose.source_age=.4;obsRender();`);
  await test('pose older than 300ms is marked expired and excluded', `document.querySelector('#status-grid').textContent.includes('位姿已过期') && obsNormalized(observationState.telemetry,Date.now()/1000).fc_pose.z===null`);
  await run(`observationState.telemetry.observe.fc_pose.source_age=null;obsRender();`);
  await test('unknown source clock is labeled and excluded from pose data', `document.querySelector('#status-grid').textContent.includes('源时间未知') && obsNormalized(observationState.telemetry,Date.now()/1000).fc_pose.z===null`);
  await run(`observationState.telemetry=makeTelemetry(Date.now()/1000-10);obsRender();`);
  await test('SSE silence cannot make cached telemetry look current', `document.querySelector('#connection-status').textContent.includes('已过期') && document.querySelector('#raw-outputs').textContent.includes('未观测') && !document.querySelector('#status-grid').textContent.includes('已解锁')`);
  await run(`document.querySelector('#reset-local').click();`);
  await test('reset clears local history and segments without touching nodes', `observationState.samples.length===0 && observationState.segments.length===0 && observationState.mapping!==null`);
  await run(`document.querySelector('#clear-mapping').click();`);
  await test('clearing local mapping removes all motor curves', `observationState.mapping===null && document.querySelector('#legend-motors').children.length===0`);
  await run(`obsConfirmMapping([4,2,1,3]);obsStartSegment('hover',Date.now()/1000);fixtureAt=Date.now()/1000+.05;
    obsIngest(makeTelemetry(fixtureAt),fixtureAt);obsEvent({t:'connection',connection:{host:'new-fixture',board_root:'/different-root'}});`);
  await test('changing target clears mapping ends segment and keeps export sample origin', `
    observationState.mapping===null && observationState.active===null && observationState.samples.some(s=>s.source.host==='fixture-only') &&
    document.querySelector('#connection-status').textContent.includes('连接目标已改变') && !document.querySelector('#raw-outputs').textContent.includes('1100')`);
  await navigate('/observe');
  await test('observe page uses the same read-only data source with a distinct layout', `!document.body.classList.contains('motor-page') && document.querySelector('#page-title').textContent.includes('实时状态') && document.querySelectorAll('canvas').length===13`);
  await test('notice explains 1Hz limits local Z and reset evidence', `document.body.textContent.includes('约1Hz') && document.body.textContent.includes('不能用它证明没有发生reset') && document.body.textContent.includes('不显示离地高度')`);
  const apiRequests=requests.filter(r=>r.url.startsWith(base+'/api/'));
  assert(apiRequests.length>=4 && apiRequests.every(r=>r.method==='GET' && ['/api/snapshot','/api/events'].includes(new URL(r.url).pathname)),'Only GET snapshot and SSE requests');
  assert(!requests.some(r=>/jpeg|image_raw|camera|\.jpg(?:\?|$)|\.mp4(?:\?|$)/i.test(r.url)),'No camera requests');
  checks.push('GET/SSE-only transport and no camera requests');
  console.log(`${checks.length} observer browser checks passed; no flight/actions/camera requests`);
}finally{
  if(socket&&socket.readyState===WebSocket.OPEN){socket.send(JSON.stringify({id:++seq,method:'Browser.close'}));await sleep(300);socket.close();}
  child.kill();await sleep(1000);
  if(path.dirname(path.resolve(profile))===path.resolve(os.tmpdir())&&path.basename(profile).startsWith('liftrace-observer-regression-')){
    try{fs.rmSync(profile,{recursive:true,force:true,maxRetries:5,retryDelay:200});}catch{}}
}
