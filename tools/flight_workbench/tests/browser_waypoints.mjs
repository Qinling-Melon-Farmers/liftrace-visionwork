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

  const until=async expression=>{for(let i=0;i<100;i++){try{if(await run(expression))return;}catch{}await sleep(100);}throw Error('timeout: '+expression);};
  const shot=async name=>{if(process.argv[4]){await run('(()=>{const t=document.querySelector(".waypoint-dialog iframe")?.contentDocument?.getElementById("toast");if(t)t.hidden=true;})()');const r=await call('Page.captureScreenshot',{format:'png'});fs.writeFileSync(process.argv[4]+'_'+name+'.png',Buffer.from(r.data,'base64'));}};
  await test('starts offline with no sessions', 'state.connection.transport==="local" && Object.keys(state.sessions).length===0');
  await run('state.selectedGroup="competition";renderGroups();');
  await until('!!document.querySelector(".grp-card.sel .waypoint-open")');
  await run(`(async()=>{const r=await fetch('/api/waypoints/draft',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({group_id:'competition'})});const d=await r.json();if(d.draft)await fetch('/api/waypoints/disable',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({group_id:'competition',scope:d.scope})});const u=groupUI(state.groups.find(g=>g.id==='competition'));WaypointWorkbench.clearDraft(u);renderGroups();})()`);
  await shot('competition');
  await run('document.querySelector(".grp-card.sel .waypoint-open").click()');
  const inside=expr=>'document.querySelector(".waypoint-dialog iframe").contentWindow.eval('+JSON.stringify(expr)+')';
  await until('!!document.querySelector(".waypoint-dialog iframe") && '+inside('typeof site!=="undefined" && !!site'));
  await test('competition loads its own unconfirmed empty template', inside('site.site_confirmed===false && site.landing_xy===null && site.corridor_waypoints.length===0'));
  await test('opens with coordinate guide and selection-only mode',inside('currentStep===1 && mode==="select" && !document.querySelectorAll("[data-panel]")[0].hidden && document.querySelector(".coordinate-guide").textContent.includes("前3，右1")'));
  await shot('guide');
  await run(inside('document.querySelectorAll("[data-bound=search_bounds]")[0].value="0.6";document.querySelectorAll("[data-bound=search_bounds]")[0].dispatchEvent(new Event("input"));'));
  await test('named coordinate field updates numeric schema without translating origin',inside('site.flight_area.search_bounds[0]===.6 && currentStep===1 && site.high_agl===2.6'));
  await run(inside('document.querySelectorAll("[data-bound=search_bounds]")[0].value="";document.querySelectorAll("[data-bound=search_bounds]")[0].dispatchEvent(new Event("input"));'));
  await until(inside('document.getElementById("export").disabled && !document.getElementById("map-status").hidden'));
  await test('incomplete bounds block save and identify stale preview',inside('site.flight_area.search_bounds[0]===null && document.getElementById("export").disabled'));
  await run(inside('document.getElementById("undo").click();'));
  await test('undo restores complete original range',inside('site.flight_area.search_bounds[0]!==null && document.getElementById("map-status").hidden'));
  await run(inside('site.flight_area.search_bounds=[.6,7.2,-3.7,1.6];site.flight_area.coverage_bounds=[.7,7.1,-3.6,1.5];render();document.querySelectorAll("[data-step]")[1].click();'));
  await test('step navigation exposes generator without changing coordinates',inside('currentStep===2 && mode==="select" && !document.querySelectorAll("[data-panel]")[1].hidden'));

  for(const [pattern,count] of [['rectangle',5],['snake2',4],['snake3',6]]) {
    await run(inside('document.getElementById("route-pattern").value='+JSON.stringify(pattern)+';document.getElementById("route-inset").value="0.2";document.getElementById("generate-route").click();'));
    await sleep(300);
    await until(inside('!document.getElementById("generate-route").disabled && site.flight_area.survey_xy.length==='+count));
    await test(pattern+' generates inside asymmetric bounds with original height',inside('site.high_agl===2.6 && site.max_agl===3.2 && Math.abs(Math.min(...site.flight_area.survey_xy.map(p=>p[1]))+3.5)<1e-8'));
  }
  await test('map distinguishes point kinds and shows traversal arrows',inside('!!document.querySelector("[data-route-arrow=survey]") && document.getElementById("map").textContent.includes("搜1")'));
  await run(inside('document.querySelector("[data-pattern=rectangle]").click();'));
  await test('route illustration selects matching shape',inside('document.getElementById("route-pattern").value==="rectangle" && document.querySelector("[data-pattern=rectangle]").getAttribute("aria-pressed")==="true"'));
  await run(inside('document.querySelector("[data-pattern=snake3]").click();site.corridor_waypoints=[{x:8,y:2,agl:.9},{x:8,y:-2,agl:.9}];site.landing_xy=[8,-2];render();'));

  await until(inside('!document.getElementById("export").disabled'));
  await shot('map');
  await run(inside('document.querySelectorAll("[data-step]")[2].click();'));
  await test('door diagram and point editing shown together',inside('currentStep===3 && !document.getElementById("point-inspector").hidden && !!document.querySelector(".door-guide") && document.getElementById("map").textContent.includes("廊1")'));
  await shot('corridor');
  await run(inside('document.querySelector("[data-mode=landing]").click();'));
  const clickXY=await run(`(()=>{const f=document.querySelector('.waypoint-dialog iframe'),b=f.getBoundingClientRect(),p=f.contentWindow.eval('(()=>{const xy=screen(7.8,-1);const p=map.createSVGPoint();p.x=xy[0];p.y=xy[1];const q=p.matrixTransform(map.getScreenCTM());return {x:q.x,y:q.y};})()');return {x:b.left+p.x,y:b.top+p.y};})()`);
  await call('Input.dispatchMouseEvent',{type:'mousePressed',...clickXY,button:'left',clickCount:1});
  await call('Input.dispatchMouseEvent',{type:'mouseReleased',...clickXY,button:'left',clickCount:1});
  await test('actual map click preserves metric coordinates at current viewport',inside('Math.abs(site.landing_xy[0]-7.8)<.001 && site.landing_xy[1]===-1'));
  await run(inside('document.getElementById("undo").click();'));
  await test('undo restores H after graphical edit',inside('site.landing_xy[0]===8 && site.landing_xy[1]===-2'));
  await run(inside('document.querySelectorAll("[data-step]")[3].click();'));

  await test('final step explains local draft and preserves original flight values',inside('!document.querySelectorAll("[data-panel]")[3].hidden && document.getElementById("point-inspector").hidden && site.high_agl===2.6 && site.max_agl===3.2'));
  await until(inside('!document.getElementById("export").disabled'));
  await shot('save');
  await run(inside('document.getElementById("export").click()'));
  await until('!document.querySelector(".waypoint-dialog")');
  await until('Object.keys(groupUI(state.groups.find(g=>g.id==="competition")).geometryPlans||{}).length===3');
  await test('all commands use formal overlay and preserve false confirmation', `(()=>{const g=state.groups.find(g=>g.id==='competition'),u=groupUI(g);return u.geometryValues.site_confirmed===false && Object.values(u.geometryPlans).length===3 && Object.values(u.geometryPlans).every(p=>p.body.includes('site_overlays/competition_')&&!p.body.includes('board_trials'));})()`);
  await call('Page.reload');await sleep(400);await until('typeof state!=="undefined" && !!state.groups?.length');
  await run('state.selectedGroup="competition";renderGroups();');
  await until('!!groupUI(state.groups.find(g=>g.id==="competition")).geometryValues');
  await test('reload restores draft without changing origin or confirmation', `(()=>{const u=groupUI(state.groups.find(g=>g.id==='competition'));return u.geometryValues.flight_area.survey_xy.length===6&&!u.geometryValues.site_confirmed;})()`);
  await run(`const p=document.querySelector('[data-option="competitionPreset"]');p.value='deployment/competition/candidates/snake3_motion.yaml';p.dispatchEvent(new Event('change'));`);
  await until('!groupUI(state.groups.find(g=>g.id==="competition")).geometryEnabled');
  await test('switching selected source clears previous command',`!groupUI(state.groups.find(g=>g.id==='competition')).geometryRevision`);
  await run('document.querySelector(".grp-card.sel .waypoint-open").click()');
  await until('!!document.querySelector(".waypoint-dialog iframe") && '+inside('typeof site!=="undefined" && !!site'));
  await test('snake3 source retains its own height',inside('site.high_agl===2.16'));
  await run(inside('document.getElementById("cancel-map").click()'));
  await run('state.selectedGroup="mod08";renderGroups();');
  await run('document.querySelector(".grp-card.sel .waypoint-open").click()');
  await until('!!document.querySelector(".waypoint-dialog iframe") && '+inside('typeof site!=="undefined" && !!site'));
  await test('module08 retains its original geometry and height',inside('site.high_agl===2 && site.corridor_waypoints.length>=5'));
  await run(inside('document.getElementById("cancel-map").click()'));
  await call('Page.navigate',{url:url+'/observe'});await until('!!document.getElementById("pose-charts")');
  await test('motor panel removed while pose/battery/record remain', `!document.getElementById('motor-panel') && !!document.getElementById('pose-charts') && !!document.getElementById('diagnostic-charts') && !!document.getElementById('record-panel')`);
  await shot('observe');
  for(const page of ['logs','replay']){await call('Page.navigate',{url:url+'/'+page});await sleep(400);await shot(page);}
  console.log(JSON.stringify({result:'PASS',checks,board_connected:false}));
} finally {
  if(socket&&socket.readyState===WebSocket.OPEN){socket.send(JSON.stringify({id:++seq,method:'Browser.close'}));await sleep(1000);socket.close();}
  if(child.exitCode===null) {
    if(process.platform==='win32') {const {spawnSync}=await import('node:child_process');spawnSync('taskkill',['/PID',String(child.pid),'/T','/F'],{windowsHide:true});}
    else child.kill('SIGTERM');
  }
  const tempRoot=path.resolve(os.tmpdir());const target=path.resolve(profile);
  if(target.startsWith(tempRoot+path.sep)&&path.basename(target).startsWith('liftrace-ui-regression-')){
    // Let Windows deliver exit events between attempts instead of blocking its
    // message loop while Chromium releases profile files.
    for(let attempt=0;attempt<30;attempt++){
      try{fs.rmSync(target,{recursive:true,force:true});break;}
      catch(error){if(attempt===29)throw error;await sleep(100);}
    }
  }
}
