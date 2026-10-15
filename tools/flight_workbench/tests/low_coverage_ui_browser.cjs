/* Node >=22 + Chromium/Edge. Only mock APIs and static files; no backend/SSH/ROS.
 * Windows: node tests/new_start_ui_browser.cjs "C:\...\msedge.exe" [web-root]
 */
'use strict';
const fs=require('node:fs'),path=require('node:path'),os=require('node:os'),http=require('node:http'),assert=require('node:assert/strict'),{spawn}=require('node:child_process');
const root=path.resolve(process.argv[3]||path.join(__dirname,'../web')),browser=process.argv[2];
if(!browser)throw Error('Pass a Chromium/Edge executable path');
const geometry={flight_area:{center_bounds:[-1,9,-5,5],target_bounds:[0,8,-4,4],search_bounds:[0,7,-4,4],coverage_bounds:[0,7,-4,4],staging_xy:[1,0],survey_xy:[[1,0],[2,0]],map_size:[20,20,5]},corridor_waypoints:[{x:6,y:0,agl:.9},{x:7,y:0,agl:.9}],landing_xy:[8,-3],high_agl:2.6,low_agl:1.4};
const groups=[
 {id:'competition',channel:'competition',name:'competition fixture',release:'real',release_options:['real'],site_config:'deployment/competition/field_10x10_default.yaml',competition_options_supported:true,resume_survey_supported:true},
 {id:'mod08',channel:'module',folder:'08_full_mission',name:'module fixture',release:'mock',release_options:['mock','real'],needs_waypoints:true,site_config:'deployment/board_trials_4x4/08_full_mission/site_20261007_221730.yaml',resume_survey_supported:true,speed_profile_options:[{value:'',label:'limited'},{value:'competition',label:'competition'}]}
 ,{id:'mod10backup',key:'10',channel:'module',folder:'10_low_coverage_full',name:'backup fixture',release:'mock',release_options:['mock','real'],needs_waypoints:true,low_coverage_backup:true,site_config:'deployment/board_trials_4x4/10_low_coverage_full/site_waypoint_map_20261015.yaml'}
],requests=[];
const coverageFixture={flight_area:{center_bounds:[-.5,9.5,-4.8,4.8],target_bounds:[-.5,7.4,-4.8,4.8],search_bounds:[.4,6.8,-4.1,4.1],staging_xy:[.6,.05],survey_xy:[[1,-3.5],[5.5,-3.5],[5.5,3.5],[1,3.5],[1,-3.5]],map_size:[22,12,4]},corridor_waypoints:[{x:6.9,y:3.4,agl:.9},{x:8.3,y:3.4,agl:.9},{x:8.3,y:1.9,agl:.9},{x:8.3,y:-.4,agl:.9},{x:8.3,y:-3.5,agl:.9}],landing_xy:[8.3,-3.5],landing_capture_agl:.9,landing_transit_agl:1.,max_agl:2.3,high_agl:2.,low_agl:1.8};
const server=http.createServer(async(req,res)=>{
 try{
  const u=new URL(req.url,'http://localhost');
  if(u.pathname.startsWith('/api/')){
   let raw='';for await(const chunk of req)raw+=chunk;const body=raw?JSON.parse(raw):{};requests.push({url:u.pathname,body});let data={ok:true};
   if(u.pathname==='/api/snapshot')data={ok:true,connection:{state:'ok',host:'mock-board.invalid',board_root:'/mock',site_dir:'deployment/site_20260928'},groups,terminals:[],sessions:{},trial:{},stage:{name:'IDLE'}};
   else if(u.pathname==='/api/competition/config-summary')data={ok:true,source:'offline_reference',values:{geometry}};
   else if(u.pathname==='/api/trial/command')data={ok:true,body:'MOCK_REVIEWED '+JSON.stringify(body),note:'mock only'};
   else if(u.pathname==='/api/waypoints/context'&&body.group_id==='mod10backup')data={ok:true,folder:'10_low_coverage_full',group_id:'mod10backup',site:coverageFixture,scope:'backup-map-scope',source:'candidate-map.yaml',origin:'mock fixture only'};
   else if(u.pathname==='/api/waypoints/preview'&&body.group_id==='mod10backup')data={ok:true,errors:[],yaml:JSON.stringify(body.site)};
   else if(u.pathname==='/api/waypoints/import'&&body.group_id==='mod10backup')data={ok:true,site:coverageFixture,ignored_fields:[],imported_fields:['flight_area','corridor_waypoints','landing_xy']};
   else if(u.pathname==='/api/waypoints/save'&&body.group_id==='mod10backup')data={ok:true,patch:body.site,scope:'backup-map-scope',revision:'mapApplied1',source:'candidate-map.yaml'};
   else if(u.pathname==='/api/waypoints/draft')data={ok:true,draft:null};
   else if(u.pathname==='/api/trial/start')data={ok:true,trial:{mock:true}};
   res.setHeader('Content-Type','application/json');res.end(JSON.stringify(data));return;
  }
  const file=u.pathname==='/'?'index.html':u.pathname.replace(/^\/static\//,''),target=path.resolve(root,file);
  if(!target.startsWith(root+path.sep)){res.writeHead(403);res.end();return;}
  res.setHeader('Content-Type',({'.html':'text/html','.js':'text/javascript','.css':'text/css','.json':'application/json'})[path.extname(file)]||'text/plain');res.end(fs.readFileSync(target));
 }catch(e){res.writeHead(500);res.end(String(e));}
});
const sleep=ms=>new Promise(r=>setTimeout(r,ms));let child,socket,profile;
(async()=>{
 await new Promise(r=>server.listen(0,'127.0.0.1',r));const origin='http://127.0.0.1:'+server.address().port;
 profile=fs.mkdtempSync(path.join(os.tmpdir(),'workbench-mock-ui-'));
 child=spawn(browser,['--headless=new','--disable-gpu','--no-first-run','--no-default-browser-check','--disable-background-networking','--remote-debugging-port=0','--user-data-dir='+profile,'about:blank'],{windowsHide:true,stdio:'ignore'});
 let port;for(let i=0;i<100;i++){try{port=Number(fs.readFileSync(path.join(profile,'DevToolsActivePort'),'utf8').split('\n')[0]);break;}catch{}await sleep(100);}assert(port,'Browser debugger must become available');
 const pages=await(await fetch('http://127.0.0.1:'+port+'/json/list')).json();socket=new WebSocket(pages.find(x=>x.type==='page').webSocketDebuggerUrl);
 await new Promise((r,j)=>{socket.onopen=r;socket.onerror=j;});let seq=0;const pending=new Map(),pageErrors=[];
 socket.onmessage=event=>{const v=JSON.parse(event.data);if(v.method==='Runtime.exceptionThrown')pageErrors.push(v.params.exceptionDetails.text);const p=pending.get(v.id);if(p){pending.delete(v.id);clearTimeout(p.timer);v.error?p.reject(Error(JSON.stringify(v.error))):p.resolve(v.result);}};
 const call=(method,params={})=>new Promise((resolve,reject)=>{const id=++seq;pending.set(id,{resolve,reject,timer:setTimeout(()=>reject(Error(method+' timed out')),15000)});socket.send(JSON.stringify({id,method,params}));});
 const evaluate=async expression=>{const r=await call('Runtime.evaluate',{expression,awaitPromise:true,returnByValue:true});if(r.exceptionDetails)throw Error(r.exceptionDetails.exception?.description||r.exceptionDetails.text);return r.result.value;};
 await call('Runtime.enable');await call('Page.enable');
 await call('Page.addScriptToEvaluateOnNewDocument',{source:"window.EventSource=class {addEventListener(){} close(){}};const mockFetch=window.fetch.bind(window);window.fetch=(url,opts)=>{if(new URL(url,location.href).origin!==location.origin)throw Error('External requests forbidden');return mockFetch(url,opts);};window.fixtureGeometry="+JSON.stringify(geometry)+";"});
 await call('Page.navigate',{url:origin});
 for(let i=0;i<100;i++){if(await evaluate("typeof state!=='undefined'&&state.groups.length===3&&!!document.querySelector('.grp-card')"))break;await sleep(50);}
 const results=await evaluate(String.raw`(async()=>{
 const out=[],check=(v,m)=>{if(!v)throw Error(m);},wait=()=>new Promise(r=>setTimeout(r,100));
 const c=state.groups.find(g=>g.id==='competition'),m=state.groups.find(g=>g.id==='mod08');
 const card=g=>Array.from(document.querySelectorAll('.grp-card')).find(n=>n.textContent.includes(g.name));
 const flight=g=>Array.from(card(g).querySelectorAll('button')).find(n=>n.textContent==='飞行（flight）');
 window.backupOnly=true;const run=async(name,fn)=>{if(window.backupOnly && !name.startsWith('backup defaults'))return;try{await fn();out.push({name,ok:true});}catch(e){out.push({name,ok:false,error:e.message});}};
 await wait();
 await run('normal gates: preview, confirmation, disconnect, running, pending, real release',async()=>{
  const u=groupUI(c);check(flight(c).disabled,'initial confirmation gate');
  u.mode='flight';u.armedOk=true;u.realConfirm='实投';renderGroups();check(!flight(c).disabled,'plain confirmed competition is enabled');
  for(const gate of ['offline','running','pending','real']){
   if(gate==='offline')state.connection.state='unknown';if(gate==='running')state.sessions.trial={state:'running'};if(gate==='pending')state.trialPending=true;if(gate==='real')u.realConfirm='';
   renderGroups();check(flight(c).disabled,'gate '+gate);
   state.connection.state='ok';state.sessions={};state.trialPending=false;u.realConfirm='实投';
  }renderGroups();
 });
 await run('persisted competition map can regenerate after reload and option change',async()=>{
  const u=groupUI(c),key=competitionDraftScope(c);
  localStorage.setItem('competitionDraft:'+key,JSON.stringify({geometry:window.fixtureGeometry,enabled:true}));
  delete u.competitionDraft;u.geometryEnabled=false;u.geometryPlans={};renderGroups();
  u.mode='flight';u.armedOk=true;u.realConfirm='实投';renderGroups();
  check(flight(c).disabled,'restored draft must await command generation');
  const rebuild=Array.from(card(c).querySelectorAll('button')).find(n=>n.textContent.includes('重新生成坐标'));
  check(rebuild,'restored draft has no visible regenerate action');
  rebuild.click();await wait();u.armedOk=true;u.realConfirm='实投';renderGroups();
  check(!flight(c).disabled,'generated command enables flight');
  u.resume='off';renderGroups();check(flight(c).disabled,'changed switch must invalidate matching command');
  Array.from(card(c).querySelectorAll('button')).find(n=>n.textContent.includes('重新生成坐标')).click();await wait();renderGroups();
  check(!flight(c).disabled,'changed options can regenerate without saving map again');
  check(trialBody(c,'flight',false).site_geometry.landing_xy[0]===8,'saved geometry retained');
 });
 await run('regenerated competition sends the reviewed coordinates to mock start',async()=>{
  const u=groupUI(c);u.armedOk=true;u.realConfirm='实投';
  const expected=trialBody(c,'flight',false),savedConfirm=confirmModal;
  confirmModal=async()=>({confirmed:true});
  try{await startTrial(c,'flight');check(state.trial.mock===true,'mock start did not accept the reviewed request');check(expected.site_geometry.landing_xy[0]===8,'reviewed H coordinate retained');check(!state.trialPending,'pending gate released after successful mock start');}
  finally{confirmModal=savedConfirm;}
 });
 await run('module regenerated request contains edited XY and search heights',async()=>{
  const u=groupUI(m);u.mode='flight';u.release='mock';u.geometryEnabled=true;u.geometryValues={high_agl:2.6,low_agl:1.4};u.waypointDraftApplied=true;
  u.geometryPoints='6, 0, 0.9\n7, 0, 0.9';u.geometryLanding='7.6, -3';u.geometryArea=JSON.stringify(window.fixtureGeometry.flight_area);renderGroups();
  card(m).querySelector('[data-waypoint-command]').click();await wait();u.armedOk=true;renderGroups();
  const body=trialBody(m,'flight',false);check(!flight(m).disabled,'module generated command enables flight');
  check(body.expected_body.includes('7.6'),'edited H coordinate reaches reviewed command');
  check(body.site_geometry.high_agl===2.6&&body.site_geometry.low_agl===1.4,'search heights retained');
 });
 await run('effective config rejects a different geometry revision',async()=>{
  const options={site_geometry:{landing_xy:[8,-3]},geometry_revision:'new'},trial={group_id:m.id,site_geometry:{landing_xy:[7,-3]},geometry_revision:'old'};
  check(competitionEffectiveConfig(m,options,{effective_config:{motion_optimization:true,resume_survey:true}},trial).includes('尚未确认'),'stale geometry reported as current effective config');
 });
 await run('confirmation cannot send options changed while dialog is open',async()=>{
  const u=groupUI(m);u.geometryEnabled=false;u.flightSettings=null;u.mode='flight';u.release='mock';u.armedOk=true;u.resume='';
  const savedConfirm=confirmModal,savedStart=api.trialStart;let approve,calls=0;
  confirmModal=()=>new Promise(r=>approve=r);api.trialStart=async()=>{calls++;return {ok:true};};
  try{const done=startTrial(m,'flight');u.resume='off';approve({confirmed:true});await done;check(calls===0,'stale approved request was sent');check(!state.trialPending,'pending gate released');}
  finally{confirmModal=savedConfirm;api.trialStart=savedStart;state.trialPending=false;}
 });
 await run('command generation rejects a changed board scope',async()=>{
  const u=groupUI(m);u.flightSettings={drop_agl:.4};u.geometryPlans={};u.resume='';const savedFetch=fetch,savedHost=state.connection.host;let finish=[];
  window.fetch=(url,opts)=>url==='/api/trial/command'?new Promise(resolve=>finish.push(()=>resolve({ok:true,json:async()=>({ok:true,body:'STALE_BOARD_COMMAND'})}))):savedFetch(url,opts);
  let rejected=false;
  try{const task=FlightSettings.generate(m,u,document.createElement('p')).catch(()=>rejected=true);state.connection.host='another-mock.invalid';finish.forEach(f=>f());await task;check(rejected,'changed-board command generation was accepted');check(!Object.values(u.geometryPlans).some(p=>p.body==='STALE_BOARD_COMMAND'),'old board commands entered new board cache');}
  finally{state.connection.host=savedHost;window.fetch=savedFetch;}
 });
 await run('backup defaults, explicit height, regeneration and old-card isolation',async()=>{
  const b=state.groups.find(g=>g.id==='mod10backup'),u=groupUI(b);u.mode='flight';u.armedOk=true;renderGroups();
  check(u.coverageHeight==='1.8','backup must default to FC 1.8m');check(card(b).textContent.includes('3线6点约29.8m'),'final lawnmower reference');check(card(b).textContent.includes('99.42%'),'1.8m ground-area reference');
  check(!flight(b).disabled,'board default must not require manual geometry');
  for(const pattern of ['lawnmower','nested_rectangles'])for(const height of ['1.8','1.9']){
   u.coveragePattern=pattern;u.coverageHeight=height;renderGroups();
   const direct=trialBody(b,'flight',false);
   check(!('site_geometry' in direct),'default must not inject manual geometry');
   check(direct.expected_body.includes('--site-config '+b.site_config),'default uses exact board file');
   check(direct.expected_body.includes('--coverage-pattern '+pattern),'default binds shape');
   check(direct.expected_body.includes('--coverage-agl '+height),'default binds height');
   check(direct.expected_body.includes('--check-config'),'flight preserves check before flight');
   check(direct.expected_body.includes('--speed-profile competition'),'default competition speed');
  }
  u.coveragePattern='lawnmower';u.coverageHeight='1.8';renderGroups();
  u.geometryEnabled=true;u.geometryValues={};u.geometryPoints='6.9,3.4,0.9\n8.3,3.4,0.9\n8.3,1.9,0.9\n8.3,-0.4,0.9\n8.3,-3.5,0.9';u.geometryLanding='8.3,-3.5';u.geometryArea=JSON.stringify(window.fixtureGeometry.flight_area);renderGroups();
  card(b).querySelector('[data-waypoint-command]').click();await wait();u.armedOk=true;renderGroups();
  check(!flight(b).disabled,'generated backup can be reviewed');
  check(trialBody(b,'flight',false).coverage_agl===1.8,'height must enter actual request');
  const select=card(b).querySelector('[data-option=coverageHeight]');select.value='1.9';select.dispatchEvent(new Event('change'));u.armedOk=true;renderGroups();
  check(flight(b).disabled,'changed height invalidates old command');
  card(b).querySelector('[data-waypoint-command]').click();await wait();u.armedOk=true;renderGroups();
  const body=trialBody(b,'flight',false);check(!flight(b).disabled,'new height can regenerate');
  check(body.coverage_agl===1.9&&body.expected_body.includes('1.9'),'reviewed command binds selected FC height');check(card(b).textContent.includes('100%'),'1.9m ground-area reference');
  check(body.site_geometry.corridor_waypoints.length===5,'five original corridor points preserved');
  check(body.site_geometry.landing_xy[0]===8.3,'original H preserved');
  const mapBefore=JSON.stringify(body.site_geometry);
  const shape=card(b).querySelector('[data-option=coveragePattern]');check(shape,'shape selector must be visible');shape.value='nested_rectangles';shape.dispatchEvent(new Event('change'));u.armedOk=true;renderGroups();
  check(flight(b).disabled,'changed pattern must invalidate old command');
  card(b).querySelector('[data-waypoint-command]').click();await wait();u.armedOk=true;renderGroups();
  const nested=trialBody(b,'flight',false);check(!flight(b).disabled,'nested rectangles command can regenerate');
  check(nested.coverage_pattern==='nested_rectangles'&&nested.expected_body.includes('nested_rectangles'),'shape reaches reviewed command');
  check(nested.coverage_agl===1.9,'pattern switch retains FC height');check(card(b).textContent.includes('回字形参考：11点约45.5m'),'selected reference preview must match shape');check(JSON.stringify(nested.site_geometry)===mapBefore,'pattern switch retains site geometry');
  check(!('coverage_agl' in trialBody(m,'preview',true)),'08 must not inherit backup height');
 });
 await run('backup map import is applied only by Save and use for this group',async()=>{
  const b=state.groups.find(g=>g.id==='mod10backup'),u=groupUI(b);u.geometryEnabled=false;u.geometryValues=null;u.geometryPlans={};u.waypointDraftApplied=false;renderGroups();
  card(b).querySelector('.waypoint-open').click();
  for(let i=0;i<20&&!document.querySelector('.waypoint-dialog iframe');i++)await wait();
  const iframe=document.querySelector('.waypoint-dialog iframe');check(iframe,'map iframe must open');
  await new Promise(r=>setTimeout(r,500));const f=iframe.contentWindow;
  await f.importLocal(JSON.stringify(window.fixtureGeometry),'waypoint_map.yaml');
  check(!u.waypointDraftApplied,'import alone must not be presented as applied');
  await new Promise(r=>setTimeout(r,400));check(!f.document.getElementById('export').disabled,'map validation must enable apply');
  f.document.getElementById('export').click();await new Promise(r=>setTimeout(r,400));
  check(u.waypointDraftApplied&&u.geometryEnabled,'map Save must apply the draft');
  u.armedOk=true;renderGroups();const body=trialBody(b,'flight',false);
  check(body.expected_body,'save must generate actual task commands');
  check(body.site_geometry.corridor_waypoints.length===5,'saved original corridor point count');
  check(body.site_geometry.flight_area.staging_xy[0]===.6&&body.site_geometry.flight_area.staging_xy[1]===.05,'S point reaches command');
  check(body.site_geometry.flight_area.search_bounds[1]===6.8,'coverage boundary reaches command');
  check(!flight(b).disabled,'reviewed mapped backup can be started after confirmation');
 });
 return out;
})()`);
 console.log(JSON.stringify({webRoot:root,results,pageErrors,commandRequests:requests.filter(r=>r.url==='/api/trial/command').length,mockTrialStarts:requests.filter(r=>r.url==='/api/trial/start').length},null,2));
 if(results.some(r=>!r.ok)||pageErrors.length)process.exitCode=1;
 await call('Browser.close').catch(()=>{});
})().catch(e=>{console.error(e);process.exitCode=1;}).finally(()=>{socket?.close();child?.kill();server.closeAllConnections();server.close();});
