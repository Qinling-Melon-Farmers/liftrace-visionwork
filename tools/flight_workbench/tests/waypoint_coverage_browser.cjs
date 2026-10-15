/* Live 8771 read-only browser check. Never saves a draft or starts a task.
 * node waypoint_coverage_browser.cjs <Edge path> <artifact directory> [web root]
 * Optional web root replaces only waypoint static assets through CDP.
 */
'use strict';
const fs=require('node:fs'),path=require('node:path'),os=require('node:os'),assert=require('node:assert/strict'),{spawn}=require('node:child_process');
const [edge,artifacts,webRoot]=process.argv.slice(2),sleep=ms=>new Promise(r=>setTimeout(r,ms));
let child,socket;
(async()=>{
 fs.mkdirSync(artifacts,{recursive:true});
 const profile=fs.mkdtempSync(path.join(os.tmpdir(),'waypoint-coverage-'));
 child=spawn(edge,['--headless=new','--disable-gpu','--no-first-run','--no-default-browser-check','--remote-debugging-port=0','--window-size=1600,1100','--user-data-dir='+profile,'about:blank'],{windowsHide:true,stdio:'ignore'});
 let port;for(let i=0;i<100;i++){try{port=Number(fs.readFileSync(path.join(profile,'DevToolsActivePort'),'utf8').split('\n')[0]);break;}catch{}await sleep(100);}assert(port);
 const pages=await(await fetch('http://127.0.0.1:'+port+'/json/list')).json();
 socket=new WebSocket(pages.find(p=>p.type==='page').webSocketDebuggerUrl);await new Promise((r,j)=>{socket.onopen=r;socket.onerror=j;});
 let seq=0;const pending=new Map(),errors=[];
 const call=(method,params={})=>new Promise((resolve,reject)=>{const id=++seq;pending.set(id,{resolve,reject});socket.send(JSON.stringify({id,method,params}));});
 socket.onmessage=async event=>{
  const v=JSON.parse(event.data);
  if(v.method==='Runtime.exceptionThrown')errors.push(v.params.exceptionDetails.exception?.description||v.params.exceptionDetails.text);
  if(v.method==='Fetch.requestPaused'){
   const p=v.params,u=new URL(p.request.url),name=path.basename(u.pathname);
   if(webRoot&&/^waypoint-.*\.(js|html|css)$/.test(name)){
    await call('Fetch.fulfillRequest',{requestId:p.requestId,responseCode:200,responseHeaders:[{name:'Content-Type',value:name.endsWith('.js')?'text/javascript':name.endsWith('.css')?'text/css':'text/html'}],body:fs.readFileSync(path.join(webRoot,name)).toString('base64')});
   }else await call('Fetch.continueRequest',{requestId:p.requestId});
  }
  const p=pending.get(v.id);if(p){pending.delete(v.id);v.error?p.reject(Error(JSON.stringify(v.error))):p.resolve(v.result);}
 };
 const evaluate=async expression=>{const r=await call('Runtime.evaluate',{expression,awaitPromise:true,returnByValue:true});if(r.exceptionDetails)throw Error(r.exceptionDetails.exception?.description||r.exceptionDetails.text);return r.result.value;};
 await call('Runtime.enable');await call('Page.enable');
 if(webRoot)await call('Fetch.enable',{patterns:[{urlPattern:'*/static/waypoint-*'}]});
 await call('Page.navigate',{url:'http://127.0.0.1:8771/'});
 for(let i=0;i<150;i++){if(await evaluate("typeof state!=='undefined'&&state.groups?.some(g=>g.folder==='10_low_coverage_full'&&[...document.querySelectorAll('.grp-card')].some(c=>c.textContent.includes(g.name)&&c.querySelector('.waypoint-open')&&c.querySelector('[data-option=coveragePattern]')))"))break;await sleep(100);}
 const result=[];
 for(const pattern of ['lawnmower','nested_rectangles'])for(const agl of ['1.8','1.9']){
  await evaluate(`(()=>{const g=state.groups.find(g=>g.folder==='10_low_coverage_full');const card=()=>[...document.querySelectorAll('.grp-card')].find(c=>c.textContent.includes(g.name));for(const [key,value] of [['coveragePattern',${JSON.stringify(pattern)}],['coverageHeight',${JSON.stringify(agl)}]]){const select=card().querySelector('[data-option='+key+']');if(!select)throw Error('Card selector missing: '+key);select.value=value;select.dispatchEvent(new Event('change'));}card().querySelector('[data-option=coveragePattern]').scrollIntoView({block:'center'});})()`);
  const cardImage=await call('Page.captureScreenshot',{format:'png',captureBeyondViewport:false});fs.writeFileSync(path.join(artifacts,'card_'+pattern+'_'+agl+'.png'),Buffer.from(cardImage.data,'base64'));
  await evaluate("(()=>{const g=state.groups.find(g=>g.folder==='10_low_coverage_full');[...document.querySelectorAll('.grp-card')].find(c=>c.textContent.includes(g.name)).querySelector('.waypoint-open').click();})()");
  let data;
  for(let i=0;i<200;i++){
   data=await evaluate("(()=>{const f=document.querySelector('.waypoint-dialog iframe')?.contentWindow;const o=f?.document.querySelector('[data-coverage-overlay]');return o?{points:JSON.parse(o.dataset.waypoints),agl:o.dataset.agl,summary:f.document.getElementById('coverage-route-summary').textContent,corridor:f.document.querySelectorAll('[data-kind=corridor]').length}:null;})()");
   if(data)break;await sleep(100);
  }
  assert(data,'Live map must show coverage overlay');assert.equal(data.agl,agl);assert.equal(data.points.length,pattern==='lawnmower'?6:11);assert.equal(data.corridor,5);
  const immutable=await evaluate("(()=>{const f=document.querySelector('.waypoint-dialog iframe').contentWindow;return f.eval('(()=>{const before=JSON.stringify(site);draw();return {unchanged:before===JSON.stringify(site),explicit:Object.hasOwn(site,\"search_waypoints_xy\")};})()');})()");
  assert.equal(immutable.unchanged,true,'Drawing must not modify save payload');assert.equal(immutable.explicit,false,'Generated overlay must not create explicit search_waypoints_xy');
  result.push({pattern,...data,selection_source:'actual card select change events',draw_unchanged:immutable.unchanged,generated_explicit:immutable.explicit});
  await evaluate("document.querySelector('.waypoint-dialog').scrollIntoView()");
  const image=await call('Page.captureScreenshot',{format:'png',captureBeyondViewport:false});fs.writeFileSync(path.join(artifacts,pattern+'_'+agl+'.png'),Buffer.from(image.data,'base64'));
  // Exercise changed geometry and explicit points in the iframe's JS memory only.
  const checks=await evaluate(`(()=>{const f=document.querySelector('.waypoint-dialog iframe').contentWindow;return f.eval(\`(()=>{const before=structuredClone(site);site.flight_area.search_bounds=[.8,6.4,-4,4];site.coverage_cross_inset=.4;site.coverage_lane_spacing=2;draw();const changed=JSON.parse(document.querySelector('[data-coverage-overlay]').dataset.waypoints);site.search_waypoints_xy=[[2,0],[3,1]];draw();const explicit=JSON.parse(document.querySelector('[data-coverage-overlay]').dataset.waypoints);site=before;draw();return {changed,explicit};})()\`);})()`);
  assert.deepEqual(checks.explicit,[[2,0],[3,1]]);assert.notDeepEqual(checks.changed,data.points);
  // Real import endpoint and UI handler; import remains unsaved in iframe memory.
  const imported=await evaluate(`(async()=>{const f=document.querySelector('.waypoint-dialog iframe').contentWindow;return f.eval(\`(async()=>{const imported=structuredClone(site);imported.flight_area.search_bounds=[.8,6.4,-4,4];imported.coverage_cross_inset=.4;imported.coverage_lane_spacing=2;imported.coverage_loop_inset=1;const file=new File([JSON.stringify(imported)],'coverage-browser-import.yaml',{type:'application/yaml'});await document.getElementById('import').onchange({target:{files:[file],value:''}});const overlay=document.querySelector('[data-coverage-overlay]');return {points:JSON.parse(overlay.dataset.waypoints),agl:overlay.dataset.agl};})()\`);})()`);
  assert.equal(imported.agl,agl);assert.notDeepEqual(imported.points,data.points);
  await evaluate("document.querySelector('.waypoint-dialog-head button').click()");
 }
 assert.deepEqual(errors,[]);fs.writeFileSync(path.join(artifacts,'result.json'),JSON.stringify({origin:'http://127.0.0.1:8771',webRoot:webRoot||'live fixed',result,errors},null,2));
 console.log(JSON.stringify({ok:true,artifacts,result,errors},null,2));
 await call('Browser.close').catch(()=>{});
})().catch(e=>{console.error(e);process.exitCode=1;}).finally(()=>{socket?.close();child?.kill();});
