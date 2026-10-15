/* A map draft enters the existing geometry-command pipeline, never a flight API. */
(function(){
  'use strict';
  const supported=g=>g.channel==='competition'||(g.channel==='module'&&['10_low_coverage_full','04_corridor_landing','08_full_mission'].includes(g.folder));
  const source=(g,U)=>g.channel==='competition'?(U.competitionConfig??g.site_config):g.site_config;
  const identity=(g,U)=>({group_id:g.id,...(g.channel==='competition'?{competition_config:source(g,U)}:{})});
  const scope=(g,U)=>JSON.stringify([state.connection?.host,state.connection?.board_root,state.connection?.site_dir,g.id,source(g,U)]);
  let dialog,frame,active;
  function invalidate(U){U.geometryRevision=null;U.geometryPlans={};U.armedOk=false;U.realConfirm='';}
  function clearDraft(U){invalidate(U);U.geometryEnabled=false;U.geometryValues=null;U.waypointDraftApplied=false;U.geometryPoints=U.geometryLanding=U.geometryWalls=U.geometryArea='';U.geometrySiteConfirmed=false;U.waypointServerScope=null;U.waypointSource=null;U.waypointDraftScope=null;U.waypointLoadError=null;}
  async function request(path,body){const r=await fetch('/api/waypoints/'+path,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)});const data=await r.json();if(!r.ok||data.ok===false)throw Error(data.error||'航点读取失败');return data;}
  function populate(g,U,data){
    if(g.channel==='competition'&&U.competitionConfig==null)U.competitionConfig=g.site_config;
    const p=data.patch;invalidate(U);U.geometryEnabled=true;U.geometryValues=p;U.geometryRevision=data.revision;U.surveyEnabled=false;U.surveyPreview=null;U.pattern='';
    U.geometryPoints=(p.corridor_waypoints||[]).map(v=>[v.x,v.y,v.agl].join(', ')).join('\n');U.geometryLanding=(p.landing_xy||[]).join(', ');U.geometrySiteConfirmed=p.site_confirmed===true;
    U.geometryArea=JSON.stringify(p.flight_area,null,2);U.geometryAxis=String(p.corridor_geometry?.wall_axis??1);U.geometryWalls=(p.corridor_geometry?.wall_coordinates||[]).join(', ');U.geometryEntry=String(p.corridor_geometry?.entry_waypoints??1);
    U.waypointDraftScope=scope(g,U);U.waypointServerScope=data.scope;U.waypointSource=data.source;U.waypointDraftApplied=true;
  }
  function generate(g){const button=document.querySelector('[data-waypoint-command="'+g.id+'"]');if(button)button.click();}
  function restore(g,U){
    if(!supported(g))return;const key=scope(g,U);if(U.waypointLoadedScope===key)return;
    // Changing board/root never transfers the preceding board's map draft.
    if((U.waypointLoadedScope&&U.waypointLoadedScope!==key)||(U.waypointDraftScope&&U.waypointDraftScope!==key))clearDraft(U);
    U.waypointLoadedScope=key;
    request('draft',identity(g,U)).then(data=>{if(U.waypointLoadedScope!==key||scope(g,U)!==key)return;if(data.draft&&data.draft.enabled!==false){populate(g,U,{...data.draft,source:data.draft.source??data.source,scope:data.draft.scope??data.scope});renderGroups();generate(g);}}).catch(e=>{if(U.waypointLoadedScope===key)U.waypointLoadError=e.message;});
  }
  function close(){if(dialog){dialog.close();dialog.remove();}dialog=frame=active=null;}
  function sendContext(){if(active?.context&&frame)frame.contentWindow.postMessage({type:'waypoint-context',context:active.context},location.origin);}
  async function open(g,U){
    if(tSessRunning()||state.trialPending){alert('任务正在运行或等待确认，请结束后再调整航点。');return;}
    close();dialog=document.createElement('dialog');dialog.className='waypoint-dialog';dialog.setAttribute('aria-label','飞行前调整航点');
    const head=document.createElement('div');head.className='waypoint-dialog-head';const title=document.createElement('strong');title.textContent=g.name+' · 地图调整';const cancel=document.createElement('button');cancel.className='btn';cancel.textContent='关闭';cancel.onclick=close;head.append(title,cancel);
    const note=document.createElement('p');note.className='waypoint-dialog-note';note.textContent='正在读取所选现场配置与本机草稿…';
    frame=document.createElement('iframe');frame.title='航点地图与坐标微调';frame.src='/static/waypoint-map.html';
    dialog.append(head,note,frame);document.body.append(dialog);dialog.showModal();dialog.addEventListener('cancel',close);
    const key=scope(g,U);active={g,U,key,note};const task=active;
    try{const context=await request('context',{...identity(g,U),...(U.geometryEnabled&&U.geometryValues?{site_geometry:U.geometryValues}:{})});if(active!==task||scope(g,U)!==key)return;active.context={...context,...identity(g,U)};note.textContent=(context.origin||'所选配置')+' · '+context.source;sendContext();}
    catch(e){if(active===task){note.textContent=e.message;note.classList.add('error');}}
  }
  window.addEventListener('message',async event=>{
    if(!active||event.origin!==location.origin||event.source!==frame?.contentWindow)return;
    if(event.data?.type==='waypoint-ready')return sendContext();
    if(event.data?.type==='waypoint-cancel')return close();
    if(event.data?.type!=='waypoint-apply')return;
    const task=active;
    try{
      if(scope(task.g,task.U)!==task.key)throw Error('连接或配置来源已变化，请重新打开地图');
      if(tSessRunning()||state.trialPending)throw Error('任务正在运行或等待确认，请结束后再应用航点');
      if(!task.context)throw Error('配置尚未读取完成');
      const data=await request('save',{...identity(task.g,task.U),scope:task.context.scope,site:event.data.site});
      if(active!==task||scope(task.g,task.U)!==task.key)return;
      populate(task.g,task.U,{...data,source:data.source??task.context.source,scope:data.scope??task.context.scope});close();renderGroups();generate(task.g);
    }catch(e){frame?.contentWindow.postMessage({type:'waypoint-error',error:e.message},location.origin);}
  });
  function button(g,U){
    const wrap=document.createElement('div');wrap.className='waypoint-entry';
    const openButton=document.createElement('button');openButton.className='btn btn-sm waypoint-open';openButton.textContent='地图调整航点';openButton.disabled=tSessRunning()||!!state.trialPending;openButton.onclick=()=>open(g,U);wrap.append(openButton);
    const note=document.createElement('p');note.className='tiny muted';note.textContent=U.waypointDraftApplied&&U.geometryEnabled?'已应用本机地图草稿；请核对下方命令并运行配置检查。':'点击地图布点或输入坐标微调，保存后直接用于本组启动配置。';wrap.append(note);
    if(U.waypointSource){const from=document.createElement('p');from.className='tiny muted wrap-any';from.textContent='草稿来源：'+U.waypointSource+' · scope '+U.waypointServerScope;wrap.append(from);}
    if(U.waypointLoadError){const error=document.createElement('p');error.className='tiny error';error.textContent=U.waypointLoadError;wrap.append(error);}
    if(U.waypointDraftApplied){
      const reset=document.createElement('button');reset.className='btn btn-sm';reset.textContent='使用原现场配置';reset.disabled=tSessRunning()||!!state.trialPending;
      reset.onclick=async()=>{const key=scope(g,U);try{if(tSessRunning()||state.trialPending)throw Error('任务正在运行或等待确认');reset.disabled=true;await request('disable',{...identity(g,U),scope:U.waypointServerScope});if(scope(g,U)!==key)return;clearDraft(U);renderGroups();}catch(e){alert(e.message);}finally{reset.disabled=false;}};wrap.append(reset);
    }
    return wrap;
  }
  window.WaypointWorkbench={supported,restore,button,invalidate,clearDraft};
})();
