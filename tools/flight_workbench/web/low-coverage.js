/* Optional low-coverage backup: existing map and command pipeline, no automatic startup. */
(function(){
  'use strict';
  const isBackup=g=>g?.folder==='10_low_coverage_full';
  const originalUI=groupUI;
  groupUI=function(g){const U=originalUI(g);if(isBackup(g)){if(U.coverageHeight==null)U.coverageHeight='1.8';if(U.coveragePattern==null)U.coveragePattern='lawnmower';}return U;};
  const originalBody=trialBody;
  trialBody=function(g,mode,check){const body=originalBody(g,mode,check);if(isBackup(g)){body.coverage_agl=Number(groupUI(g).coverageHeight||1.8);body.coverage_pattern=groupUI(g).coveragePattern||'lawnmower';delete body.expected_body;const built=groupCommandBody(g,body.mode,body.real_release,!!body.check_config,body.capture_speed,body);if(built.body)body.expected_body=built.body;}return body;};
  const originalKey=geometryPlanKey;
  geometryPlanKey=function(g,mode,real,check,speed,options){const key=originalKey(g,mode,real,check,speed,options);return isBackup(g)?key+'|coverage-agl='+Number(options.coverage_agl??groupUI(g).coverageHeight??1.8)+'|coverage-pattern='+(options.coverage_pattern??groupUI(g).coveragePattern??'lawnmower'):key;};
  const originalCommand=groupCommandBody;
  groupCommandBody=function(g,mode,real,check,speed,options){
    if(!isBackup(g))return originalCommand(g,mode,real,check,speed,options);
    options=options||{};
    const U=groupUI(g),plans=U.geometryPlans||{};
    return plans[geometryPlanKey(g,mode,real,check,speed,options)]||{body:null,note:'备用候选：先导入/确认实测地图，保存并用于本组，再生成匹配形状与FC 1.8/1.9m的命令；飞行命令先做同一配置检查。'};
  };
  const originalDraft=geometryFromDraft;
  geometryFromDraft=function(U,g){
    const result=originalDraft(U,g);
    if(!isBackup(g))return result;
    const patch={...U.geometryValues,...result};
    for(const key of Object.keys(patch))if(!['flight_area','corridor_waypoints','landing_xy','corridor_geometry','landing_transit_agl','landing_capture_agl','max_agl'].includes(key))delete patch[key];
    return patch;
  };
  const originalEditor=geometryEditor;
  geometryEditor=function(g,U,onChange){
    const box=originalEditor(g,U,onChange);if(!isBackup(g))return box;
    const note=el('div','warn-line','默认limited规划上限0.5m/s、加速度0.35m/s²；运动优化和投后移动恢复继承模块。候选未实飞 · FC离地高度（不是镜头高度） · 虚拟柱关闭，实体避障保留。地图沿用实测起点、搜索范围、走廊与H；新形状均使用所选FC高度，保留实测地图。候选默认沿Y长线、跨扫描轴两端内收0.6m；牛耕间距2.6m，回字形独立圈内收1.8m，由新模块从实测范围生成。仅比较FOV理论扫过的地面面积。备份投递目标FC 0.25m；释放窗口继承08（当前0.10～0.43m），两者含义不同。');
    const row=el('label','geometry-field');row.appendChild(el('span',null,'备用遍历 FC AGL'));
    const select=el('select');select.setAttribute('data-option','coverageHeight');
    for(const value of ['1.8','1.9']){const option=el('option',null,value+' m'+(value==='1.8'?'（默认）':''));option.value=value;select.appendChild(option);}
    select.value=U.coverageHeight||'1.8';select.onchange=function(){U.coverageHeight=select.value;U.geometryPlans={};U.armedOk=false;U.realConfirm='';renderGroups();};row.appendChild(select);
    const shapeRow=el('label','geometry-field');shapeRow.appendChild(el('span',null,'搜索形状'));
    const shape=el('select');shape.setAttribute('data-option','coveragePattern');
    for(const [value,label]of [['lawnmower','牛耕'],['nested_rectangles','回字形']]){const option=el('option',null,label);option.value=value;shape.appendChild(option);}
    shape.value=U.coveragePattern||'lawnmower';shape.onchange=function(){U.coveragePattern=shape.value;U.geometryPlans={};U.armedOk=false;U.realConfirm='';renderGroups();};shapeRow.appendChild(shape);
    const routePreview=el('div','tiny muted',(U.coveragePattern==='nested_rectangles'?'回字形参考：11点约45.5m':'牛耕参考：3线6点约29.8m')+'；FC '+(U.coverageHeight||'1.8')+'m理论地面面积覆盖 '+(Number(U.coverageHeight||1.8)===1.9?'100%':'99.42%')+'。仅对应随包已打点地图和FOV模型，不代表实飞检出率；改变范围后重新核算。保留原S、门/H和所选FC高度。');
    box.prepend(note,shapeRow,row,routePreview);
    box.appendChild(el('div','tiny muted','注意：仅导入或保存文件不会应用任务。请在地图内点击“保存并用于本组”，再核对生成命令中的10_low_coverage_full、--coverage-agl与--coverage-pattern。飞行命令会先执行同一份地图的--check-config。'));
    return box;
  };
})();
