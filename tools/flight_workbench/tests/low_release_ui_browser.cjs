/* Node >=22 + Chromium/Edge. Only mock APIs and static files; no backend/SSH/ROS.
 * Generate fixture: python tests/test_low_release.py --fixture <fixture.json>
 * Run: node tests/low_release_ui_browser.cjs <browser.exe> <fixture.json>
 */
'use strict';
const fs=require('node:fs'),path=require('node:path'),os=require('node:os'),http=require('node:http'),assert=require('node:assert/strict'),{spawn}=require('node:child_process');
const root=path.resolve(path.join(__dirname,'../web')),browser=process.argv[2],fixture=JSON.parse(fs.readFileSync(process.argv[3],'utf8'));
if(!browser)throw Error('Pass a Chromium/Edge executable path');
const geometry={},groups=[fixture.group],requests=[];
const server=http.createServer(async(req,res)=>{
 try{
  const u=new URL(req.url,'http://localhost');
  if(u.pathname.startsWith('/api/')){
   let raw='';for await(const chunk of req)raw+=chunk;const body=raw?JSON.parse(raw):{};requests.push({url:u.pathname,body});let data={ok:true};
   if(u.pathname==='/api/snapshot')data={ok:true,connection:{state:'ok',host:'mock-board.invalid',board_root:'/mock',site_dir:'deployment/site_20260928'},groups,terminals:[],sessions:{},trial:{},stage:{name:'IDLE'}};
   else if(u.pathname==='/api/competition/config-summary')data={ok:true,source:'offline_reference',values:{geometry}};
   else if(u.pathname==='/api/trial/command')data={ok:true,body:'MOCK_REVIEWED '+JSON.stringify(body),note:'mock only'};
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
 for(let i=0;i<100;i++){if(await evaluate("typeof state!=='undefined'&&state.groups.length===1&&!!document.querySelector('.grp-card')"))break;await sleep(50);}
 const results=await evaluate(String.raw`(async()=>{
 const out=[],check=(v,m)=>{if(!v)throw Error(m);};
 const g=state.groups[0],u=groupUI(g);
 const card=()=>document.querySelector('.grp-card');
 const flight=()=>Array.from(card().querySelectorAll('button')).find(n=>n.textContent==='飞行（flight）');
 const select=(field,value)=>{const node=card().querySelector('[data-option="'+field+'"]');check(node,'missing '+field);node.value=value;node.dispatchEvent(new Event('change',{bubbles:true}));};
 const run=async(name,fn)=>{try{await fn();out.push({name,ok:true});}catch(e){out.push({name,ok:false,error:e.message});}};
 await run('actual card renders mock, three slots and no geometry editor',async()=>{
  check(g.low_release_test===true,'wrong fixture');
  check(u.release==='mock'&&u.slot==='1','default must be mock/rear');
  const slots=card().querySelector('[data-option="slot"]');
  check(slots.options.length===3,'three slots');
  check(slots.options[0].textContent.includes('后')&&slots.options[1].textContent.includes('右')&&slots.options[2].textContent.includes('左'),'slot labels');
  check(card().textContent.includes('0.45')&&card().textContent.includes('0.50'),'flight plan missing');
  check(card().textContent.includes('无需另点启动任务')&&card().textContent.includes('已有5b服务'),'automatic action / existing servo guidance missing');
  const previous=state.trial;state.trial={group_id:g.id};
  const mission=missionStartSection({name:'READY'});state.trial=previous;
  check(!mission.querySelector('button'),'independent automatic entry must not show start-mission button');
  check(mission.textContent.includes('0.60m后自动开始动作')&&mission.textContent.includes('不自动启动真实PWM、不复位'),'automatic mission guidance missing');
  check(!card().querySelector('[data-waypoint-command]'),'independent test must not inherit geometry');
 });
 await run('real confirmation and slot changes invalidate approval',async()=>{
  u.mode='flight';u.armedOk=true;renderGroups();check(!flight().disabled,'mock flight confirmation');
  select('release','real');check(flight().disabled,'real must require confirmation');
  u.armedOk=true;u.realConfirm='实投';renderGroups();check(!flight().disabled,'confirmed real option');
  select('slot','2');check(!u.armedOk&&u.realConfirm==='','slot change retains old confirmation');
  check(flight().disabled,'slot change must disable flight');
 });
 await run('configuration check and preview stay action free with real selected',async()=>{
  select('slot','3');
  for(const mode of ['preview','flight']) {
   const body=trialBody(g,mode,true);check(body.mode==='preview'&&body.check_config,'check mode');
   check(body.slot===3&&!body.real_release&&!body.expected_body.includes('--real-release'),'check release permission');
  }
  check(!trialBody(g,'preview',false).expected_body.includes('--real-release'),'preview release permission');
  const old=confirmModal;confirmModal=async()=>{throw Error('config check requested flight confirmation');};
  try{await startTrial(g,'preview',true);}finally{confirmModal=old;}
 });
 await run('reviewed real command sent only to mock HTTP endpoint',async()=>{
  u.mode='flight';u.release='real';u.slot='2';u.realConfirm='实投';u.armedOk=true;renderGroups();
  const old=confirmModal;
  confirmModal=async(title,command,note)=>{check(command.includes('flight --slot 2 --real-release'),'reviewed real command');check(note.includes('人工')&&note.includes('一次'),'pilot/single release note');check(note.includes('无需另点启动任务')&&note.includes('不自动启动真实PWM、不复位'),'automatic action / existing servo confirmation missing');return {confirmed:true};};
  try{await startTrial(g,'flight',false);check(!u.armedOk,'flight confirmation reset');}finally{confirmModal=old;}
 });
 await run('slot changed while modal open prevents send',async()=>{
  u.armedOk=true;u.realConfirm='实投';const old=confirmModal,oldStart=api.trialStart;let finish,calls=0;
  confirmModal=()=>new Promise(r=>finish=r);api.trialStart=async()=>{calls++;return {ok:true};};
  try{const waiting=startTrial(g,'flight',false);u.slot='1';finish({confirmed:true});await waiting;check(calls===0,'stale command sent');}
  finally{confirmModal=old;api.trialStart=oldStart;}
 });
 await run('stop dialog retains position and localization',async()=>{
  state.trial={route:'module',low_release_test:true};const old=confirmModal;let seen=false;
  confirmModal=async(title,command,note)=>{seen=true;check(note.includes('保留定位和MAVROS')&&note.includes('不强杀'),'wrong stop guidance');return {confirmed:false};};
  try{doTrialStop();check(seen,'stop dialog missing');}finally{confirmModal=old;}
 });
 return out;
})()`);
 const starts=requests.filter(r=>r.url==='/api/trial/start');
 for(const request of starts){const match=fixture.cases.find(c=>c.body.mode===request.body.mode&&c.body.slot===request.body.slot&&c.body.real_release===request.body.real_release&&c.body.check_config===!!request.body.check_config);assert(match,'unknown mock request');assert.equal(request.body.expected_body,match.expected);}
 assert.equal(starts.length,2,'config check and real-selection mock start only');
 console.log(JSON.stringify({results,pageErrors,mockStarts:starts},null,2));
 if(results.some(r=>!r.ok)||pageErrors.length)process.exitCode=1;
 await call('Browser.close').catch(()=>{});
})().catch(e=>{console.error(e);process.exitCode=1;}).finally(()=>{socket?.close();child?.kill();server.closeAllConnections();server.close();});
