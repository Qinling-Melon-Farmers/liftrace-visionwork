import fs from 'node:fs';
import os from 'node:os';
import path from 'node:path';
import {spawn} from 'node:child_process';
const [base,out]=process.argv.slice(2);
const profile=fs.mkdtempSync(path.join(os.tmpdir(),'liftrace-screenshots-'));
const child=spawn('C:\\Program Files (x86)\\Microsoft\\Edge\\Application\\msedge.exe',[
  '--headless=new','--disable-gpu','--no-first-run','--no-default-browser-check',
  '--remote-debugging-port=0','--user-data-dir='+profile,'about:blank'],{windowsHide:true,stdio:'ignore'});
const sleep=ms=>new Promise(r=>setTimeout(r,ms));
let socket,seq=0;
const pending=new Map();
const shots=[];
try {
  let port;
  for(let i=0;i<150;i++){try{port=Number(fs.readFileSync(path.join(profile,'DevToolsActivePort'),'utf8').split('\n')[0]);break;}catch{}await sleep(100);}
  if(!port)throw Error('Edge CDP startup timeout');
  const pages=await(await fetch(`http://127.0.0.1:${port}/json/list`)).json();
  socket=new WebSocket(pages.find(p=>p.type==='page').webSocketDebuggerUrl);
  await new Promise((ok,fail)=>{socket.onopen=ok;socket.onerror=fail;});
  socket.onmessage=e=>{const m=JSON.parse(e.data),p=pending.get(m.id);if(!p)return;pending.delete(m.id);clearTimeout(p.timer);m.error?p.fail(Error(JSON.stringify(m.error))):p.ok(m.result);};
  const call=(method,params={})=>new Promise((ok,fail)=>{const id=++seq;pending.set(id,{ok,fail,timer:setTimeout(()=>{pending.delete(id);fail(Error(method+' timeout'));},15000)});socket.send(JSON.stringify({id,method,params}));});
  const run=async expression=>{const r=await call('Runtime.evaluate',{expression,returnByValue:true,awaitPromise:true});if(r.exceptionDetails)throw Error(JSON.stringify(r.exceptionDetails));return r.result.value;};
  await call('Page.enable');
  await call('Emulation.setDeviceMetricsOverride',{width:1920,height:1600,deviceScaleFactor:1,mobile:false});
  const navigate=async route=>{await call('Page.navigate',{url:base+route});for(let i=0;i<100;i++){if(await run('document.readyState === "complete"'))break;await sleep(100);}await sleep(1600);};
  const shot=async(file,route,view)=>{const metrics=await call('Page.getLayoutMetrics');const size=metrics.cssContentSize||metrics.contentSize;const width=Math.ceil(size.width),height=Math.ceil(size.height);const r=await call('Page.captureScreenshot',{format:'png',captureBeyondViewport:true,clip:{x:0,y:0,width,height,scale:1}});fs.writeFileSync(path.join(out,file),Buffer.from(r.data,'base64'));shots.push({file,route,url:base+route,view,width,height});console.log(file);};
  await navigate('/');
  for(let i=0;i<100;i++){if(await run('typeof state!=="undefined" && state.groups.length>=9'))break;await sleep(100);}
  if(!await run('state.connection.transport==="local" && state.connection.state==="unknown" && Object.keys(state.sessions).length===0'))throw Error('Expected empty offline state');
  // Only scroll the actual task pane; never start, connect or inject telemetry.
  await run('document.querySelector("#groups-body").scrollTop=0');
  await shot('01_home_special_cards.png','/','Homepage and task cards, offline');
  await run(`const card=[...document.querySelectorAll('.grp-card')].find(c=>c.querySelector('[data-option="competitionPreset"]'));if(!card)throw Error('Competition card missing');for(const d of card.querySelectorAll('details'))d.open=true;const pane=document.querySelector('#groups-body');pane.scrollTop+=card.getBoundingClientRect().top-pane.getBoundingClientRect().top;`);
  await sleep(400);
  await shot('02_competition_expanded.png','/','Independent competition card, configuration and command expanded; formal defaults');
  await navigate('/observe');
  await shot('03_realtime_observe.png','/observe','Unified live observation, empty offline telemetry');
  await navigate('/logs');
  await shot('04_logs.png','/logs','Logs and recording page, offline');
  fs.writeFileSync(path.join(out,'pages.json'),JSON.stringify({screenshots:shots,telemetry:'Empty offline state; no injected values; no board or ROS connection'},null,2));
}finally{
  if(socket?.readyState===WebSocket.OPEN){socket.send(JSON.stringify({id:++seq,method:'Browser.close'}));await sleep(500);socket.close();}
  child.kill();
}
