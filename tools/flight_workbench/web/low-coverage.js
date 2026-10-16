/* Optional low-coverage backup: existing map and command pipeline, no automatic startup. */
(function(){
  'use strict';
  const isBackup=g=>g?.folder==='10_low_coverage_full';
  window.lowCoverageSite=(g,pattern,shiftback=false)=>(shiftback?g.site_configs_shiftback_by_pattern:g.site_configs_by_pattern)?.[pattern??'lawnmower'];
  window.lowCoverageReference=U=>U.geometryEnabled?'手动几何覆盖：覆盖率需重算':('新区域76.63㎡ · 理想FOV参考 '+(U.coverageShiftback?(Number(U.coverageHeight||1.8)===1.9?'100%':'99.74%'):(Number(U.coverageHeight||1.8)===1.9?'89.91%':'88.56%'))+'，不代表实飞检出率');
  let speedAPI='explicit',speedScope=null;
  const originalSnapshot=applySnapshot;
  applySnapshot=function(snap){
    originalSnapshot(snap);
    const g=(snap.groups||[]).find(isBackup),c=snap.connection||{},scope=JSON.stringify([c.host,c.board_root]);
    if(!g||speedScope===scope)return;speedScope=scope;
    fetch('/api/trial/command',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({group_id:g.id,mode:'preview',coverage_agl:1.8,coverage_pattern:'lawnmower',speed_profile:'competition'})})
      .then(async r=>{const data=await r.json();if(speedScope!==scope)return;if(r.ok&&data.ok!==false){speedAPI='explicit';return;}
        const fallback=await fetch('/api/trial/command',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({group_id:g.id,mode:'preview',coverage_agl:1.8,coverage_pattern:'lawnmower'})});
        const inherited=await fallback.json();if(speedScope===scope&&fallback.ok&&inherited.ok!==false){speedAPI='inherit';groupUI(g).geometryPlans={};renderGroups();}
      }).catch(()=>{});
  };
  // Older running backends emit this advisory even for a confirmed default map.
  const originalToast=toast;
  toast=function(kind,message){if(typeof message==='string'&&message.includes('的走廊航点/H 坐标必须实测填写')&&(state.groups||[]).some(g=>['08_full_mission','10_low_coverage_full'].includes(g.folder)&&message.startsWith(g.name)&&!groupUI(g).geometryEnabled))return;return originalToast.apply(this,arguments);};
  const originalUI=groupUI;
  groupUI=function(g){const U=originalUI(g);if(isBackup(g)){if(U.coverageHeight==null)U.coverageHeight='1.8';if(U.coveragePattern==null)U.coveragePattern='lawnmower';}return U;};
  const originalBody=trialBody;
  trialBody=function(g,mode,check){const body=originalBody(g,mode,check);if(isBackup(g)){if(speedAPI==='inherit')delete body.speed_profile;else body.speed_profile=groupUI(g).speedProfile||'competition';body.coverage_agl=Number(groupUI(g).coverageHeight||1.8);body.coverage_pattern=groupUI(g).coveragePattern||'lawnmower';body.coverage_shiftback=!!groupUI(g).coverageShiftback;delete body.expected_body;const built=groupCommandBody(g,body.mode,body.real_release,!!body.check_config,body.capture_speed,body);if(built.body)body.expected_body=built.body;}return body;};
  const originalKey=geometryPlanKey;
  geometryPlanKey=function(g,mode,real,check,speed,options){const key=originalKey(g,mode,real,check,speed,options);return isBackup(g)?key+'|site='+window.lowCoverageSite(g,options.coverage_pattern??groupUI(g).coveragePattern,options.coverage_shiftback??!!groupUI(g).coverageShiftback)+'|coverage-agl='+Number(options.coverage_agl??groupUI(g).coverageHeight??1.8)+'|coverage-pattern='+(options.coverage_pattern??groupUI(g).coveragePattern??'lawnmower')+'|speed-profile='+(options.speed_profile??groupUI(g).speedProfile??'competition'):key;};
  const originalCommand=groupCommandBody;
  groupCommandBody=function(g,mode,real,check,speed,options){
    if(['04_corridor_landing','08_full_mission'].includes(g.folder)&&!g.exact_site_reference)return {body:null,note:'后端未加载新现场配置，请在会话结束后重载工作台'};
    if(!isBackup(g))return originalCommand(g,mode,real,check,speed,options);
    options=options||{};
    const U=groupUI(g),plans=U.geometryPlans||{};
    if(!U.geometryEnabled && options.site_geometry == null){
      const agl=Number(options.coverage_agl??U.coverageHeight??1.8),pattern=options.coverage_pattern??U.coveragePattern??'lawnmower';
      if(![1.8,1.9].includes(agl)||!['lawnmower','nested_rectangles'].includes(pattern)||!window.lowCoverageSite(g,pattern,options.coverage_shiftback??!!U.coverageShiftback))return {body:null,note:'默认地图、形状或FC高度无效'};
      const common=' --site-config '+shellArgQuote(window.lowCoverageSite(g,pattern,options.coverage_shiftback??!!U.coverageShiftback))+' --coverage-agl '+agl.toFixed(1)+' --coverage-pattern '+pattern+(speedAPI==='inherit'?'':' --speed-profile '+(options.speed_profile??U.speedProfile??'competition'))+(options.motion_optimized?' --motion-optimized':'');
      const entry='bash deployment/board_trials_4x4/10_low_coverage_full/start.sh';
      const checked=entry+' preview'+common+' --check-config';
      return {body:check?checked:mode==='flight'?checked+' && '+entry+' flight'+common+(real?' --real-release':''):entry+' preview'+common,note:'使用板端已确认的本组默认YAML和正赛档competition；旧后端省略速度覆盖并继承板端已确认的1.2/1.0。手动覆盖可选。飞行前检查同一配置。'};
    }
    return plans[geometryPlanKey(g,mode,real,check,speed,options)]||{body:null,note:'手动覆盖：保存并用于本组，再生成匹配形状与FC 1.8/1.9m的命令；未启用手动时直接使用板端默认地图。'};
  };
  const originalDraft=geometryFromDraft;
  geometryFromDraft=function(U,g){
    const result=originalDraft(U,g);
    if(!isBackup(g))return result;
    const patch={...U.geometryValues,...result};
    for(const key of Object.keys(patch))if(!['flight_area','corridor_waypoints','landing_xy','corridor_geometry','landing_transit_agl','landing_capture_agl','max_agl','search_waypoints_xy','coverage_rectangles','coverage_scan_axis','coverage_cross_inset','coverage_lane_spacing','coverage_loop_inset'].includes(key))delete patch[key];
    return patch;
  };
  const originalEditor=geometryEditor;
  geometryEditor=function(g,U,onChange){
    const box=originalEditor(g,U,onChange);if(g.folder==='08_full_mission')box.appendChild(el('div','tiny muted',U.geometryEnabled||U.surveyEnabled?'手动搜索几何：覆盖率需重算':'默认高位FC2.6m · 新区域76.63㎡ · 理想FOV参考93.46%，不代表实飞检出率'));if(!isBackup(g))return box;
    const note=el('div','warn-line','默认正赛档competition：规划上限1.2m/s、加速度1.0m/s²；H捕获/对准FC1.1m继承板端YAML；运动优化和投后移动恢复继承模块。候选未实飞 · FC离地高度（不是镜头高度） · 虚拟柱关闭，实体避障保留。地图沿用实测起点、搜索范围、走廊与H；新形状均使用所选FC高度，保留实测地图。候选默认沿Y长线、跨扫描轴两端内收0.6m；牛耕间距2.6m，回字形独立圈内收1.8m，优先使用所选现场文件显式搜索点或两框，缺省才从范围生成。仅比较FOV理论扫过的地面面积。备份投递目标FC 0.25m；释放窗口继承08（当前0.10～0.45m），两者含义不同。');
    const row=el('label','geometry-field');row.appendChild(el('span',null,'备用遍历 FC AGL'));
    const select=el('select');select.setAttribute('data-option','coverageHeight');
    for(const value of ['1.8','1.9']){const option=el('option',null,value+' m'+(value==='1.8'?'（默认）':''));option.value=value;select.appendChild(option);}
    select.value=U.coverageHeight||'1.8';select.onchange=function(){U.coverageHeight=select.value;U.geometryPlans={};U.armedOk=false;U.realConfirm='';renderGroups();};row.appendChild(select);
    const shapeRow=el('label','geometry-field');shapeRow.appendChild(el('span',null,'搜索形状'));
    const shape=el('select');shape.setAttribute('data-option','coveragePattern');
    for(const [value,label]of [['lawnmower','牛耕'],['nested_rectangles','回字形']]){const option=el('option',null,label);option.value=value;shape.appendChild(option);}
    shape.value=U.coveragePattern||'lawnmower';shape.disabled=!!U.geometryEnabled;shape.title=U.geometryEnabled?'手动覆盖优先；先使用原现场配置，再选择默认路线':'';shape.onchange=function(){U.coveragePattern=shape.value;U.geometryPlans={};U.armedOk=false;U.realConfirm='';renderGroups();};shapeRow.appendChild(shape);
    const routePreview=el('div','tiny muted',(U.coveragePattern==='nested_rectangles'?'回字形：显式两框、11点':'牛耕：显式六点')+' · '+(U.coverageShiftback?'可选后移1m':'原搜索路线（默认）')+' · FC '+(U.coverageHeight||'1.8')+'m；打开地图核对所选文件。手动覆盖优先，覆盖启用时先恢复原现场配置再切换路线。');
    const shiftRow=el('label','chk'),shift=el('input');shift.type='checkbox';shift.checked=!!U.coverageShiftback;shift.disabled=!!U.geometryEnabled;shift.title=shape.title;shift.setAttribute('data-option','coverageShiftback');shift.onchange=function(){U.coverageShiftback=shift.checked;U.geometryPlans={};U.armedOk=false;U.realConfirm='';renderGroups();};shiftRow.append(shift,el('span',null,'可选：低位搜索点整体后移1m（默认关闭，走廊/H不变）'));
    box.prepend(note,shapeRow,shiftRow,row,routePreview,el('div','tiny muted',window.lowCoverageReference(U)));
    box.appendChild(el('div','tiny muted','默认直接使用板端本组现场YAML，无须开启手动或填坐标。可选手动覆盖：仅导入或保存文件不会应用任务。请在地图内点击“保存并用于本组”，再核对生成命令中的10_low_coverage_full、--coverage-agl与--coverage-pattern。飞行命令会先执行同一份地图的--check-config。'));
    return box;
  };

  // 路线选择放在卡片最前面；不藏在手动地图编辑内部。
  const originalCard=groupCard;
  groupCard=function(g){
    const card=originalCard(g);if(!isBackup(g))return card;
    const U=groupUI(g),panel=el('div','low-coverage-options');
    panel.setAttribute('data-panel','low-coverage-options');
    panel.appendChild(el('strong',null,'备用10：选择搜索路线与现场YAML'));
    for(const field of ['coveragePattern','coverageShiftback','coverageHeight']){
      const control=card.querySelector('[data-option="'+field+'"]');
      if(control)panel.appendChild(control.closest('label'));
    }
    panel.appendChild(el('div','tiny muted wrap-any','当前YAML：'+(window.lowCoverageSite(g,U.coveragePattern,!!U.coverageShiftback)||'未登记，请更新后端配置')));
    panel.appendChild(el('div','tiny muted','后移1m只改变搜索点，走廊与H不变；这是可选路线，默认不后移。负X表示起飞点后方，仍须在目标/机体边界内。'));
    if(U.geometryEnabled)panel.appendChild(el('div','warn-line','当前有手动地图覆盖。请先关闭“使用实测坐标／自动搜索航线”以切换登记YAML，避免旧草稿覆盖所选路线。'));
    card.querySelector('.grp-ops').prepend(panel);return card;
  };
  const build=document.getElementById&&document.getElementById('workbench-build');
  if(build)build.textContent='工作台 2026-10-16 · 路线选择与临时释放版 · 当前地址 '+location.host+'（不同端口可能是另一版本）';
})();
