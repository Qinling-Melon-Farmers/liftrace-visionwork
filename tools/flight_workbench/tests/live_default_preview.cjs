// Read-only live UI/API command preview; never invokes trial/start, devices, check-config or PWM.
const {spawn}=require('node:child_process'),fs=require('node:fs'),os=require('node:os'),path=require('node:path');
(async()=>{
 const dir=fs.mkdtempSync(path.join(os.tmpdir(),'wb-live-'));
 const child=spawn('C:\\Program Files (x86)\\Microsoft\\Edge\\Application\\msedge.exe',['--headless','--disable-gpu','--no-first-run','--remote-debugging-port=0','--user-data-dir='+dir,'about:blank']);
 let stderr='';const endpoint=await new Promise((resolve,reject)=>{child.stderr.on('data',d=>{stderr+=d;const m=stderr.match(/DevTools listening on (ws:\/\/[^\s]+)/);if(m)resolve(m[1]);});setTimeout(()=>reject(Error('browser timeout')),15000).unref();});
 const url=new URL(endpoint);const tabs=await (await fetch('http://'+url.host+'/json/list')).json();
 const ws=new WebSocket(tabs.find(t=>t.type==='page').webSocketDebuggerUrl);await new Promise(r=>ws.onopen=r);
 let id=0;const pending=new Map();ws.onmessage=e=>{const v=JSON.parse(e.data);if(v.id){const p=pending.get(v.id);if(p){pending.delete(v.id);v.error?p.reject(Error(v.error.message)):p.resolve(v.result);}}};
 const call=(method,params={})=>new Promise((resolve,reject)=>{const seq=++id;pending.set(seq,{resolve,reject});ws.send(JSON.stringify({id:seq,method,params}));});
 try{
 await call('Page.enable');await call('Emulation.setDeviceMetricsOverride',{width:1600,height:1200,deviceScaleFactor:1,mobile:false});
 await call('Page.navigate',{url:'http://127.0.0.1:8771/?ui=20261015-live2'});await new Promise(r=>setTimeout(r,3500));
 const result=await call('Runtime.evaluate',{awaitPromise:true,returnByValue:true,expression:`(async()=>{
  const out=[];const g=state.groups.find(g=>g.id==='mod10backup'),u=groupUI(g);u.geometryEnabled=false;u.geometryPlans={};
  for(const pattern of ['lawnmower','nested_rectangles'])for(const height of ['1.8','1.9']){
   u.coveragePattern=pattern;u.coverageHeight=height;
   const body=trialBody(g,'preview',false);
   const r=await fetch('/api/trial/command',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)}),data=await r.json();
   out.push({group:g.id,pattern,height,ok:r.ok&&data.ok&&body.expected_body===data.body,noManual:!body.site_geometry,speed:body.speed_profile||'inherit-board-competition',expected:body.expected_body,actual:data.body,error:data.error});
  }
  const a=state.groups.find(g=>g.id==='mod08'),au=groupUI(a);au.geometryEnabled=false;
  const ab=trialBody(a,'preview',false);const ar=await fetch('/api/trial/command',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(ab)}),ad=await ar.json();
  out.push({group:a.id,site:a.site_config,speed:ab.speed_profile,ok:ar.ok&&ad.ok&&ab.expected_body===ad.body,expected:ab.expected_body,actual:ad.body,error:ad.error});
  return {results:out,probe:state.telemetry?.probe_link,connection:state.connection.state,sessions:Object.fromEntries(Object.entries(state.sessions).map(([k,v])=>[k,v.state]))};
 })()`});
 const data=result.result.value;if(!data)throw Error(JSON.stringify(result));console.log(JSON.stringify(data,null,2));
 const shot=await call('Page.captureScreenshot',{format:'png'});fs.writeFileSync(path.join(os.tmpdir(),'workbench_8771_live_defaults.png'),Buffer.from(shot.data,'base64'));
 if(data.results.some(v=>!v.ok))process.exitCode=1;
 }finally{ws.close();child.kill();}
})().catch(e=>{console.error(e);process.exitCode=1;});