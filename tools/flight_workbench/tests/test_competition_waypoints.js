// Run with node; all HTTP requests are mocks, no server/board/ROS is contacted.
const assert=require('assert'),fs=require('fs'),path=require('path'),vm=require('vm');
const source=name=>fs.readFileSync(path.join(__dirname,'../web',name),'utf8');
const plain=value=>JSON.parse(JSON.stringify(value));
const clone=value=>structuredClone(value);
const tick=async()=>{for(let i=0;i<12;i++)await Promise.resolve();};
class Element {
  constructor(tag){this.tagName=tag;this.children=[];this.attrs={};this.dataset={};this.events={};this.style={};this.value='';this.disabled=false;this.hidden=false;this.classList={toggle(){},add(){}};}
  append(...items){for(const item of items){this.children.push(item);if(item&&typeof item==='object')item.parentElement=this;}}
  appendChild(item){this.append(item);return item;}
  replaceChildren(...items){this.children=[];this.append(...items);}
  get firstChild(){return this.children[0];}
  removeChild(item){this.children.splice(this.children.indexOf(item),1);}
  setAttribute(key,value){this.attrs[key]=String(value);}
  getAttribute(key){return this.attrs[key];}
  removeAttribute(key){delete this.attrs[key];}
  addEventListener(key,fn){this.events[key]=fn;}
  click(){if(this.disabled)return;return this.onclick?.()??this.events.click?.();}
  showModal(){} close(){} remove(){} focus(){} select(){}
  all(){return [this,...this.children.filter(x=>x instanceof Element).flatMap(x=>x.all())];}
  querySelector(selector){return this.all().find(e=>e.attrs['data-waypoint-command']===selector.match(/data-waypoint-command="(.*?)"/)?.[1]);}
}
function environment(){
  const roots=[],ids={},listeners={},timers=[];
  const doc={readyState:'loading',addEventListener(){},body:new Element('body'),
    createElement(tag){const e=new Element(tag);if(tag==='iframe')e.contentWindow={messages:[],postMessage(message){this.messages.push(clone(message));}};return e;},
    createElementNS(ns,tag){return new Element(tag);},createTextNode(text){return text;},
    getElementById(id){return ids[id]||(ids[id]=new Element('div'));},
    querySelector(selector){for(const root of [doc.body,...roots]){const e=root.querySelector(selector);if(e)return e;}return null;},
    querySelectorAll(){return [];}};
  const ctx={document:doc,console,structuredClone:clone,location:{origin:'http://test.invalid'},
    alert(message){throw Error(message);},confirm:()=>true,
    setTimeout(fn){timers.push(fn);return timers.length;},clearTimeout(id){if(id)timers[id-1]=null;},
    addEventListener(type,fn){listeners[type]=fn;}};
  ctx.window=ctx;vm.createContext(ctx);
  return {ctx,doc,roots,ids,listeners,async timers(){const pending=timers.splice(0);for(const fn of pending)if(fn)await fn();}};
}
const emptySite={flight_area:{center_bounds:[-.5,9.5,-2,8],target_bounds:[.5,3,-1,2],search_bounds:[.2,7,-1.5,5],coverage_bounds:[.5,6,-1,4],staging_xy:[1,0],survey_xy:[[1,0],[2,1]],map_size:[21,18,5]},corridor_waypoints:[],landing_xy:null,high_agl:2.6,max_agl:3.2,site_confirmed:false};
const filledSite={...clone(emptySite),corridor_waypoints:[{x:7,y:4,agl:.9},{x:9,y:7,agl:.9}],landing_xy:[9,7]};
const patchOf=site=>Object.fromEntries(['flight_area','corridor_waypoints','landing_xy','corridor_geometry','site_confirmed'].filter(k=>k in site).map(k=>[k,clone(site[k])]));
const group={id:'competition',name:'Competition',channel:'competition',release:'real',site_config:'field-a.yaml',competition_options_supported:true,resume_survey_supported:true};

async function mainPipeline(){
  const env=environment(),{ctx,roots,doc,listeners}=env;vm.runInContext(source('app.js'),ctx);
  ctx.state.connection={host:'local',board_root:'/project',site_dir:'site',state:'ok'};
  const calls=[],drafts={};let deferDraft,delayed;
  ctx.fetch=async(url,options)=>{
    const body=JSON.parse(options.body);calls.push({url,body});
    assert(body.group_id,'Every waypoint/command request has group_id');
    if(body.group_id===group.id)assert(body.competition_config,'Competition uses actual selected path');
    let data;
    if(url.endsWith('/draft')){
      if(body.competition_config==='slow.yaml')return new Promise(resolve=>{deferDraft=()=>resolve({ok:true,json:async()=>({ok:true,draft:delayed})});});
      data={ok:true,scope:'scope-'+body.competition_config,source:body.competition_config,draft:drafts[body.competition_config]||null};
    }else if(url.endsWith('/context'))data={ok:true,editor_kind:'competition',scope:'scope-'+body.competition_config,source:body.competition_config,site:clone(filledSite)};
    else if(url.endsWith('/save')){data={ok:true,revision:'saved',scope:body.scope,source:body.competition_config,patch:patchOf(body.site)};drafts[body.competition_config]=data;}
    else if(url.endsWith('/disable')){delete drafts[body.competition_config];data={ok:true};}
    else if(url==='/api/trial/command')data={ok:true,body:['overlay',body.competition_config,body.mode,!!body.check_config,body.motion_optimization||'',body.landing_config||''].join(' '),note:'mock overlay'};
    else throw Error('Unexpected API '+url);
    return {ok:true,json:async()=>data};
  };
  vm.runInContext(source('waypoint-bridge.js'),ctx);
  ctx.renderGroups=()=>{roots.length=0;roots.push(ctx.geometryEditor(group,ctx.state.tabs[group.id],()=>{}));};
  const U=ctx.groupUI(group);await tick();ctx.renderGroups();
  assert(ctx.WaypointWorkbench.supported(group));
  for(const folder of ['04_corridor_landing','08_full_mission'])assert(ctx.WaypointWorkbench.supported({channel:'module',folder}));
  assert(!ctx.WaypointWorkbench.supported({channel:'site',folder:'08_full_mission'}));
  const entry=roots[0].all().find(e=>e.className==='waypoint-open'||e.className==='btn btn-sm waypoint-open');
  await entry.click();const frame=doc.body.all().find(e=>e.tagName==='iframe');
  await listeners.message({origin:ctx.location.origin,source:frame.contentWindow,data:{type:'waypoint-ready'}});
  const context=frame.contentWindow.messages.at(-1).context;
  assert.equal(context.competition_config,'field-a.yaml');assert.equal(context.group_id,group.id);
  U.armedOk=true;U.realConfirm='实投';U.landingConfig='h-policy.yaml';
  await listeners.message({origin:ctx.location.origin,source:frame.contentWindow,data:{type:'waypoint-apply',site:filledSite}});await tick();
  assert.equal(U.armedOk,false);assert.equal(U.realConfirm,'');assert.equal(U.landingConfig,'h-policy.yaml');
  assert.equal(U.waypointSource,'field-a.yaml');assert.equal(U.waypointServerScope,'scope-field-a.yaml');
  const commandCalls=calls.filter(c=>c.url==='/api/trial/command');assert.equal(commandCalls.length,3);
  const patch=commandCalls[0].body.site_geometry;
  assert.deepEqual(patch,patchOf(filledSite));assert.equal(patch.site_confirmed,false);assert(!('high_agl' in patch));assert(!('max_agl' in patch));
  assert.throws(()=>ctx.geometryFromDraft({...U,geometryPoints:''},group),/至少/);
  assert.throws(()=>ctx.geometryFromDraft({...U,geometryLanding:''},group),/H中心/);
  const incomplete=clone(filledSite.flight_area);delete incomplete.coverage_bounds;
  assert.throws(()=>ctx.geometryFromDraft({...U,geometryArea:JSON.stringify(incomplete)},group),/coverage_bounds/);
  for(const [mode,check]of [['preview',true],['preview',false],['flight',false]]){
    const body=plain(ctx.trialBody(group,mode,check));assert(body.expected_body.startsWith('overlay field-a.yaml'));
    assert.deepEqual(body.site_geometry,patch);assert.equal(body.geometry_revision,U.geometryRevision);assert.equal(body.landing_config,'h-policy.yaml');
  }
  const planKey=ctx.geometryPlanKey(group,'preview',false,true,null,ctx.trialBody(group,'preview',true));
  for(const change of [{competition_config:'other.yaml'},{motion_optimization:'off'},{obstacle_columns:'off'},{landing_config:'different-h.yaml'},{resume_survey:'off'}]){
    const changed={...ctx.trialBody(group,'preview',true),...change};assert.notEqual(ctx.geometryPlanKey(group,'preview',false,true,null,changed),planKey);
  }
  U.motionOptimization='off';assert(!ctx.trialBody(group,'preview',true).expected_body);U.motionOptimization='';
  const field=roots[0].all().find(e=>e.attrs['data-geometry']==='geometryLanding');
  U.armedOk=true;U.realConfirm='实投';field.value='8.5, 6';field.events.input();
  assert(!ctx.trialBody(group,'flight',false).expected_body);assert(!U.armedOk);assert.equal(U.realConfirm,'');
  assert(ctx.competitionEffectiveConfig(group,ctx.trialBody(group,'preview',true),{effective_config:{source:'generated_runtime'}},{group_id:group.id,competition_config:'field-a.yaml'}).includes('尚未确认'));
  U.competitionConfig='field-b.yaml';ctx.groupUI(group);await tick();assert(!U.geometryEnabled);assert(!U.geometryValues);
  U.competitionConfig='field-a.yaml';ctx.groupUI(group);await tick();assert(U.waypointDraftApplied);assert.equal(U.geometryValues.site_confirmed,false);
  // Late response for another source cannot carry a map into the new source.
  delayed={scope:'old',source:'slow.yaml',revision:'old',patch:patchOf(filledSite)};U.competitionConfig='slow.yaml';ctx.groupUI(group);
  U.competitionConfig='field-b.yaml';ctx.groupUI(group);deferDraft();await tick();assert(!U.geometryEnabled);assert(!U.waypointSource);
  // The module command path still accepts the inherited H strategy.
  const module={id:'m04',channel:'module',folder:'04_corridor_landing',site_config:'module.yaml',needs_waypoints:true};
  ctx.state.tabs.m04={landingConfig:'h-policy.yaml'};
  assert(ctx.trialBody(module,'preview',true).expected_body.includes('--landing-config h-policy.yaml'));
  U.speed='9';U.speedProfile='limited';U.pattern='snake3';U.motionOptimized=true;
  const polluted={...group,speed_options:[9],speed_profile_options:[{}],survey_patterns:['snake3'],motion_optimization_supported:true};
  const body=ctx.trialBody(polluted,'preview',true);for(const key of ['capture_speed','speed_profile','survey_pattern','motion_optimized'])assert(!(key in body));
  // A draft edit while the confirmation dialog is open invalidates that approval.
  U.geometryEnabled=false;U.armedOk=true;U.realConfirm='实投';ctx.toast=()=>{};ctx.scheduleRender=()=>{};let approve;
  ctx.confirmModal=()=>new Promise(resolve=>approve=resolve);let starts=0;ctx.api.trialStart=()=>{starts++;return Promise.resolve({ok:true});};
  const pending=ctx.startTrial(group,'flight');U.competitionConfig='changed.yaml';approve({confirmed:true});await pending;assert.equal(starts,0);
  console.log('PASS competition save → three command modes, source isolation/races, stale commands/approval, module04/08 and H config');
}

async function mapEditor(){
  const env=environment(),{ctx,doc,ids,listeners}=env,calls=[];
  const parent={messages:[],postMessage(message){this.messages.push(clone(message));}};ctx.parent=parent;
  for(const id of ['default-agl','map','snap','export','geometry','route-pattern','route-inset'])doc.getElementById(id);
  ids['default-agl'].parentElement=new Element('label');ids.snap.value='0.1';ids['default-agl'].value='.9';
  ctx.fetch=async(url,options)=>{
    const body=JSON.parse(options.body);calls.push({url,body});assert.equal(body.group_id,group.id);assert.equal(body.competition_config,'offset-field.yaml');assert.equal(body.scope,'offset-scope');
    let data;if(url.endsWith('/preview'))data={ok:true,yaml:'mock geometry',errors:body.site.corridor_waypoints.length<2||!body.site.landing_xy?['Need corridor/H']:[]};
    else if(url.endsWith('/route')){
      const [xmin,xmax,ymin,ymax]=body.site.flight_area.search_bounds,inset=body.inset;
      if(inset<0||2*inset>=Math.min(xmax-xmin,ymax-ymin))return {ok:false,json:async()=>({error:'inset outside range'})};
      const x0=xmin+inset,x1=xmax-inset,y0=ymin+inset,y1=ymax-inset;
      data={ok:true,route:body.pattern==='rectangle'?[[x0,y0],[x1,y0],[x1,y1],[x0,y1],[x0,y0]]:body.pattern==='snake2'?[[x0,y0],[x1,y0],[x1,y1],[x0,y1]]:[[x0,y0],[x1,y0],[x1,(y0+y1)/2],[x0,(y0+y1)/2],[x0,y1],[x1,y1]]};
    }else if(url.endsWith('/import'))data={...clone(filledSite),high_agl:99,max_agl:100};
    else throw Error('Unexpected API '+url);return {ok:true,json:async()=>data};
  };
  vm.runInContext(source('waypoint-map.js'),ctx);
  listeners.message({origin:ctx.location.origin,source:parent,data:{type:'waypoint-context',context:{editor_kind:'competition',group_id:group.id,competition_config:'offset-field.yaml',scope:'offset-scope',site:clone(emptySite)}}});
  await env.timers();assert(ids.export.disabled,'Empty template is editable but unsavable');
  const confirmed=ids.geometry.all().find(e=>e.id==='site-confirmed');assert.equal(confirmed.checked,false);
  confirmed.checked=true;confirmed.onchange();assert.equal(vm.runInContext('site.site_confirmed',ctx),true);
  const unconfirm=ids.geometry.all().find(e=>e.id==='site-confirmed');unconfirm.checked=false;unconfirm.onchange();assert.equal(vm.runInContext('site.site_confirmed',ctx),false);
  const geometryCount=ids.geometry.children.length;assert.equal(geometryCount,6,'Coverage and explicit confirmation are shown');
  // H outside tiny target bounds remains visible; moving boundaries leaves origin at zero.
  vm.runInContext('site.landing_xy=[9,7];draw()',ctx);
  const h=ids.map.children.find(e=>e.attrs['data-kind']==='landing');assert(Number(h.attrs.cx)>0&&Number(h.attrs.cx)<1000);assert(Number(h.attrs.cy)>0&&Number(h.attrs.cy)<730);
  assert.deepEqual(plain(vm.runInContext('site.landing_xy',ctx)),[9,7]);
  ids['route-inset'].value='.2';
  for(const [pattern,count]of [['rectangle',5],['snake2',4],['snake3',6]]){
    ids['route-pattern'].value=pattern;await ids['generate-route'].onclick();await env.timers();
    assert.equal(vm.runInContext('site.flight_area.survey_xy.length',ctx),count);assert.equal(vm.runInContext('site.high_agl',ctx),2.6);assert.equal(vm.runInContext('site.max_agl',ctx),3.2);
    assert.equal(vm.runInContext('site.site_confirmed',ctx),false);
    assert.deepEqual(plain(vm.runInContext('site.flight_area.survey_xy[0]',ctx)),[.4,-1.3]);
  }
  assert.equal(parent.messages.filter(m=>m.type==='waypoint-apply').length,0,'Generating never saves');
  const three=plain(vm.runInContext('site.flight_area.survey_xy',ctx));
  vm.runInContext("updatePoint('survey',0,1.25,-.75);render()",ctx);assert.deepEqual(plain(vm.runInContext('site.flight_area.survey_xy[0]',ctx)),[1.25,-.75]);
  ids['undo'].onclick();assert.equal(vm.runInContext('site.flight_area.survey_xy.length',ctx),4,'Route generation participates in undo');
  ids['route-inset'].value='9';const before=plain(vm.runInContext('site',ctx));await ids['generate-route'].onclick();assert.deepEqual(plain(vm.runInContext('site',ctx)),before,'Rejected inset leaves draft intact');
  ids['import'].onchange({target:{files:[{name:'different-height.yaml',size:1,text:async()=>''}],value:'x'}});await tick();
  assert.equal(vm.runInContext('site.high_agl',ctx),2.6);assert.equal(vm.runInContext('site.max_agl',ctx),3.2);
  await env.timers();assert.equal(ids.export.disabled,false);ids.export.onclick();
  const saved=parent.messages.at(-1);assert.equal(saved.type,'waypoint-apply');assert.equal(saved.site.site_confirmed,false);assert.deepEqual(saved.site.flight_area.coverage_bounds,filledSite.flight_area.coverage_bounds);assert.equal(saved.site.high_agl,2.6);
  assert(calls.some(c=>c.url.endsWith('/import')));assert(calls.some(c=>c.url.endsWith('/preview')));assert.equal(three.length,6);
  console.log('PASS empty template, full map bounds/H, asymmetric origin, three routes/undo/edit/rejection, scoped preview/import, source heights');
}
mainPipeline().then(mapEditor).catch(error=>{console.error(error);process.exitCode=1;});
