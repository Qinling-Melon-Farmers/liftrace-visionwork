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

  const g=state.groups.find(g=>g.id==='mod10backup');if(!g)throw Error('Missing backup10');
  const U=groupUI(g);U.geometryEnabled=false;U.geometryPlans={};state.selectedGroup=g.id;renderGroups();
  const out=[];
  for(const pattern of ['lawnmower','nested_rectangles'])for(const shift of [false,true]){
   let p=document.querySelector('[data-panel="low-coverage-options"]');if(!p)throw Error('Missing visible route panel');
   const shape=p.querySelector('[data-option="coveragePattern"]');shape.value=pattern;shape.dispatchEvent(new Event('change'));
   p=document.querySelector('[data-panel="low-coverage-options"]');const cb=p.querySelector('[data-option="coverageShiftback"]');cb.checked=shift;cb.dispatchEvent(new Event('change'));
   p=document.querySelector('[data-panel="low-coverage-options"]');
   const body=trialBody(g,'preview',false),r=await fetch('/api/trial/command',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)}),data=await r.json();
   out.push({pattern,shift,visible:!!p.offsetHeight,ok:r.ok&&data.ok&&data.body===body.expected_body,source:window.lowCoverageSite(g,pattern,shift),sourceShown:p.textContent.includes(window.lowCoverageSite(g,pattern,shift)),command:data.body});
  }
  document.querySelector('[data-panel="low-coverage-options"]').scrollIntoView({block:'center'});
  return {results:out,banner:document.querySelector('#workbench-build').textContent};
 })()`});
 if(result.exceptionDetails)throw Error(JSON.stringify(result.exceptionDetails)); const data=result.result.value;if(!data)throw Error(JSON.stringify(result));console.log(JSON.stringify(data,null,2));
 const shot=await call('Page.captureScreenshot',{format:'png'});fs.writeFileSync(path.join(os.tmpdir(),'workbench_route_choices_20261016.png'),Buffer.from(shot.data,'base64'));
 if(data.results.some(v=>!v.ok||!v.visible||!v.sourceShown))process.exitCode=1;
 }finally{ws.close();child.kill();}
})().catch(e=>{console.error(e);process.exitCode=1;});