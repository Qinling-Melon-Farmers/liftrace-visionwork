/* Read-only browser observer. Network access is limited to GET snapshot and SSE.
 * Segment recording, channel mapping, reset and downloads are browser-local. */
'use strict';
var OBS_COLORS = ['#79baff','#f5b965','#7ce0b1','#ce9eff'];
var OBS_SEGMENT_LIMIT = 24;
var OBS_POSES = [['fc_pose','FC local'],['lio_pose','LIO'],['ev_pose','EV输入'],['setpoint','目标设定点']];
var observationState = {
  connection:{}, telemetry:null, configuration:{topics:{},profiles:[],max_samples:1800},
  stage:{}, trial:{}, trialSession:{}, connectionNotice:'',
  samples:[], segments:[], active:null, seq:0, dropped:0, segmentsDropped:0, lastAt:null,
  draft:[null,null,null,null], mapping:null, mappingSource:null, wiring:null, wiringSuppressed:false,
  channelCount:16, eventSource:null, stream:'等待连接'
};
function obsNumber(value) { return typeof value==='number' && Number.isFinite(value) ? value : null; }
function obsNode(tag, text, cls) {
  var node=document.createElement(tag); if(text!=null)node.textContent=String(text); if(cls)node.className=cls; return node;
}
function obsClear(node) { while(node.firstChild)node.removeChild(node.firstChild); }
function obsById(id) { return document.getElementById(id); }
function obsValue(value, unit) { return value==null ? '未观测' : (typeof value==='number' ? value.toFixed(3) : String(value)) + (unit||''); }
function obsTime(t) { return t ? new Date(t*1000).toLocaleString() : '未观测'; }
function obsFresh(tel, now) {
  var at=tel && obsNumber(tel.at || tel.t);
  return at!==null && now-at>=-1 && now-at<=2;
}
function obsScope() { return JSON.stringify([observationState.connection.host||'',observationState.connection.board_root||'']); }
function obsSetConnection(patch, replace) {
  var before=obsScope(),old=observationState.connection;
  var next=replace ? Object.assign({},patch||{}) : Object.assign({},old,patch||{});
  if((old.host||old.board_root) && before!==JSON.stringify([next.host||'',next.board_root||''])){
    obsEndSegment(Date.now()/1000);
    observationState.mapping=null;observationState.draft=[null,null,null,null];
    observationState.mappingSource=null;observationState.wiringSuppressed=false;
    observationState.telemetry=null;observationState.lastAt=null;
    observationState.connectionNotice='连接目标已改变：旧片段保留来源，图表切换至新目标；已登记目标载入接线，其他目标需核实映射。';
  }
  observationState.connection=next;
  obsApplyWiring();
}
function obsCurrentWiring() {
  var wiring=observationState.wiring,c=observationState.connection;
  return wiring && wiring.targets.some(function(t){return t.host===c.host && t.board_root===c.board_root;}) ? wiring : null;
}
function obsSetWiring(wiring) {
  var layout=wiring && wiring.rc_layout,motors=wiring && wiring.motors;
  if(!wiring || wiring.version!==1 || !wiring.confirmed_by || !Array.isArray(wiring.targets) || !wiring.targets.length ||
      !wiring.targets.every(function(t){return t && typeof t.host==='string' && t.host && typeof t.board_root==='string' && t.board_root;}) ||
      !layout || !Number.isInteger(layout.port) || layout.port<0 || ![8,16].includes(layout.channels_per_port) ||
      !Array.isArray(motors) || motors.length!==4 || !motors.every(function(m,i){return m && m.motor===i+1 &&
        typeof m.position==='string' && m.position && Number.isInteger(m.aux) && m.aux>0 && m.aux<=layout.channels_per_port;}) ||
      new Set(motors.map(function(m){return m.aux;})).size!==4)return false;
  var count=(layout.port+1)*layout.channels_per_port;
  if(count>32)return false;
  observationState.wiring=JSON.parse(JSON.stringify(wiring));
  obsApplyWiring();return true;
}
function obsApplyWiring() {
  var wiring=obsCurrentWiring();
  if(!wiring || observationState.mapping || observationState.wiringSuppressed)return;
  var layout=wiring.rc_layout;
  observationState.channelCount=Math.max(observationState.channelCount,(layout.port+1)*layout.channels_per_port);
  var channels=wiring.motors.map(function(m){return layout.port*layout.channels_per_port+m.aux;});
  observationState.draft=channels.slice();observationState.mapping=channels;
  observationState.mappingSource='user-confirmed-wiring';
}
function obsMotorLabel(index,wiring) {
  if(wiring===undefined)wiring=obsCurrentWiring();
  var motor=wiring && wiring.motors[index];
  return 'M'+(index+1)+(motor?' '+motor.position+' · AUX'+motor.aux:'');
}
function obsPayload(value) {
  if(!value || typeof value!=='object')return null;
  var age=obsNumber(value.source_age);
  return age!==null && (age>2 || age<-.1) ? null : value;
}
function obsNormalized(tel, now) {
  var raw=obsFresh(tel,now) && tel.observe || {};
  var out={};
  OBS_POSES.forEach(function(pair) {
    var pose=raw[pair[0]];
    if(!pose){out[pair[0]]=null;return;}
    var result={frame:pose.frame||'未观测',stamp:obsNumber(pose.stamp),source_age:obsNumber(pose.source_age)};
    result.unknown_age=result.source_age===null;
    result.stale=result.source_age!==null && (result.source_age>.3 || result.source_age<-.1);
    ['x','y','z','roll_deg','pitch_deg','yaw_deg'].forEach(function(k){result[k]=result.stale||result.unknown_age?null:obsNumber(pose[k]);});
    out[pair[0]]=result;
  });
  ['battery','rc_out','esc_status','esc_telemetry','low_hover'].forEach(function(k){out[k]=obsPayload(raw[k]);});
  out.fc_state=obsFresh(tel,now) ? tel.state||{} : {};
  out.lio_realtime=obsFresh(tel,now) ? tel.lio_realtime||[] : [];
  out.mission=obsFresh(tel,now) ? tel.mission||{} : {};
  out.terminal_hover=obsFresh(tel,now) ? tel.terminal_hover : null;
  if(typeof out.terminal_hover==='string'){try{out.terminal_hover=JSON.parse(out.terminal_hover);}catch(e){out.terminal_hover=null;}}
  return out;
}
function obsLimit() {
  var configured=Number(observationState.configuration.max_samples);
  return Number.isFinite(configured) && configured>=1 ? Math.min(1800,Math.floor(configured)) : 1800;
}
function obsIngest(tel, now) {
  var at=tel && obsNumber(tel.at || tel.t);
  if(at!==null && observationState.lastAt!==null && at<=observationState.lastAt)return false;
  observationState.telemetry=tel;
  if(at===null)return false;
  observationState.lastAt=at;
  if(!obsFresh(tel,now))return false;
  var data=obsNormalized(tel,now);
  observationState.samples.push({seq:++observationState.seq,at:now,source_at:at,scope:obsScope(),
    source:{host:observationState.connection.host||'',board_root:observationState.connection.board_root||''},
    mapping:observationState.mapping?observationState.mapping.slice():null,mapping_source:observationState.mappingSource,
    wiring:obsCurrentWiring(),data:data});
  var limit=obsLimit();
  while(observationState.samples.length>limit){observationState.samples.shift();observationState.dropped++;}
  var channels=data.rc_out && data.rc_out.channels;
  if(Array.isArray(channels) && channels.length>observationState.channelCount)observationState.channelCount=channels.length;
  return true;
}
function obsRaw(sample, channel) {
  if(!Number.isInteger(channel) || channel<1)return null;
  var output=sample && sample.data.rc_out;
  return output && Array.isArray(output.channels) ? obsNumber(output.channels[channel-1]) : null;
}
function obsValidMapping(channels) {
  return Array.isArray(channels) && channels.length===4 && channels.every(function(c){return Number.isInteger(c) && c>0 && c<=observationState.channelCount;})
    && new Set(channels).size===4;
}
function obsConfirmMapping(channels) {
  if(!obsValidMapping(channels))return false;
  observationState.mapping=channels.slice();observationState.mappingSource='manual';return true;
}
function obsStartSegment(profile, now) {
  if(observationState.active || !observationState.configuration.profiles.some(function(p){return p.id===profile;}))return false;
  observationState.active={profile:profile,start:now,startSeq:observationState.seq+1,
    scope:obsScope(),source:{host:observationState.connection.host||'',board_root:observationState.connection.board_root||''},
    mapping:observationState.mapping ? observationState.mapping.slice() : null,
    mapping_source:observationState.mappingSource,wiring:obsCurrentWiring()};
  return true;
}
function obsEndSegment(now) {
  if(!observationState.active)return false;
  var segment=observationState.active;segment.end=now;segment.endSeq=observationState.seq;
  observationState.segments.push(segment);observationState.active=null;
  while(observationState.segments.length>OBS_SEGMENT_LIMIT){observationState.segments.shift();observationState.segmentsDropped++;}
  return true;
}
function obsStats(values) {
  var valid=values.filter(function(v){return obsNumber(v)!==null;});
  return valid.length ? {count:valid.length,mean:valid.reduce(function(a,b){return a+b;},0)/valid.length,min:Math.min.apply(null,valid),max:Math.max.apply(null,valid)}
    : {count:0,mean:null,min:null,max:null};
}
function obsLioAge(data) {
  var values=[];
  (data.lio_realtime||[]).forEach(function(d){
    var raw=d.values && d.values.output_age_sec;
    if(raw!==undefined && raw!==null && raw!==''){
      var value=Number(raw);if(Number.isFinite(value))values.push(value);
    }
  });
  return values.length ? Math.max.apply(null,values) : null;
}
function obsSummary(segment) {
  var endSeq=segment.endSeq==null ? observationState.seq : segment.endSeq;
  var samples=observationState.samples.filter(function(s){return s.seq>=segment.startSeq && s.seq<=endSeq && s.scope===segment.scope;});
  var count=Math.max(0,endSeq-segment.startSeq+1);
  var result={profile:segment.profile,start:segment.start,end:segment.end||null,window_sec:(segment.end||Date.now()/1000)-segment.start,
    sample_count:count,retained_count:samples.length,truncated:samples.length<count,mapping:segment.mapping,
    mapping_source:segment.mapping_source,wiring:segment.wiring||null,source:segment.source,poses:{},battery:{},outputs:[],output_deviation:[],lio_age_sec:null,states:[]};
  OBS_POSES.forEach(function(pair){
    var frames=Array.from(new Set(samples.map(function(s){var p=s.data[pair[0]];return p&&p.frame;}).filter(Boolean)));
    var pose={frames:frames};
    ['x','y','z','roll_deg','pitch_deg','yaw_deg','source_age'].forEach(function(key){
      // Coordinate statistics never combine different frames.
      pose[key]=frames.length>1 ? {count:0,mean:null,min:null,max:null,reason:'坐标系发生变化，未合并'}
        : obsStats(samples.map(function(s){return s.data[pair[0]] && s.data[pair[0]][key];}));
    });result.poses[pair[0]]=pose;
  });
  ['voltage','current','percentage'].forEach(function(key){result.battery[key]=obsStats(samples.map(function(s){return s.data.battery && obsNumber(s.data.battery[key]);}));});
  result.lio_age_sec=obsStats(samples.map(function(s){return obsLioAge(s.data);}));
  result.states=Array.from(new Set(samples.map(function(s){var low=s.data.low_hover||{},fc=s.data.fc_state||{};
    return [low.stage||'未观测',fc.mode||'未观测',typeof fc.armed==='boolean'?'armed='+fc.armed:'armed未观测'].join(' / ');} )));
  if(segment.mapping){
    result.outputs=segment.mapping.map(function(channel){return {channel:channel,stats:obsStats(samples.map(function(s){return obsRaw(s,channel);} ))};});
    result.output_deviation=segment.mapping.map(function(channel){return {channel:channel,stats:obsStats(samples.map(function(s){
      var values=segment.mapping.map(function(c){return obsRaw(s,c);});
      if(values.some(function(v){return v===null;}))return null;
      return obsRaw(s,channel)-values.reduce(function(a,b){return a+b;},0)/4;
    }))};});
  }
  return result;
}
function obsCsvCell(value) { return '"'+String(value==null?'':value).replace(/"/g,'""')+'"'; }
function obsCSV() {
  var fields=['received_at','source_at','seq','host','board_root','profile','mode','armed','low_hover_stage'];
  OBS_POSES.forEach(function(pair){['frame','source_age','x','y','z','roll_deg','pitch_deg','yaw_deg'].forEach(function(k){fields.push(pair[0]+'_'+k);});});
  fields=fields.concat(['battery_voltage','battery_current','battery_percentage','lio_output_age_sec','rc_out_raw_json','confirmed_mapping_json','selected_outputs_json','esc_status_json','esc_telemetry_json','mapping_source','motor_wiring_json']);
  var rows=[fields];
  observationState.samples.forEach(function(s){
    var segment=observationState.segments.concat(observationState.active?[observationState.active]:[]).find(function(p){return s.seq>=p.startSeq && s.seq<=(p.endSeq==null?observationState.seq:p.endSeq);});
    var fc=s.data.fc_state||{},low=s.data.low_hover||{};
    var row=[s.at,s.source_at,s.seq,s.source.host,s.source.board_root,segment?segment.profile:'',fc.mode,fc.armed,low.stage];
    OBS_POSES.forEach(function(pair){var pose=s.data[pair[0]]||{};['frame','source_age','x','y','z','roll_deg','pitch_deg','yaw_deg'].forEach(function(k){row.push(pose[k]);});});
    var bat=s.data.battery||{},map=segment?segment.mapping:s.mapping;
    row=row.concat([bat.voltage,bat.current,bat.percentage,obsLioAge(s.data),JSON.stringify(s.data.rc_out),JSON.stringify(map),
      map?JSON.stringify(map.map(function(c){return obsRaw(s,c);})):'',JSON.stringify(s.data.esc_status),JSON.stringify(s.data.esc_telemetry),
      segment?segment.mapping_source:s.mapping_source,JSON.stringify(segment?segment.wiring:s.wiring)]);
    rows.push(row);
  });
  return '\uFEFF'+rows.map(function(row){return row.map(obsCsvCell).join(',');}).join('\r\n');
}
function obsJSON() {
  return JSON.stringify({format:'liftrace-browser-observation-v1',exported_at:new Date().toISOString(),
    connection:{host:observationState.connection.host},configuration:observationState.configuration,
    output_semantics:'raw RC output commands, not current/RPM; mapping is user-confirmed wiring or manual browser selection',
    mapping:observationState.mapping,mapping_source:observationState.mappingSource,wiring:obsCurrentWiring(),
    dropped_samples:observationState.dropped,dropped_segments:observationState.segmentsDropped,samples:observationState.samples,
    segments:observationState.segments.concat(observationState.active?[observationState.active]:[]).map(obsSummary)},null,2);
}
function obsDownload(content,type,extension) {
  var blob=new Blob([content],{type:type}),url=URL.createObjectURL(blob);
  var link=obsNode('a');link.href=url;link.download='liftrace-observe-'+new Date().toISOString().replace(/[:.]/g,'-')+'.'+extension;
  document.body.appendChild(link);link.click();link.remove();setTimeout(function(){URL.revokeObjectURL(url);},1000);
}
function obsConfigure(configuration) {
  observationState.configuration=Object.assign({topics:{},profiles:[],max_samples:1800},configuration||{});
  var select=obsById('profile-select'),previous=select.value;obsClear(select);
  observationState.configuration.profiles.forEach(function(profile){var option=obsNode('option',profile.name||profile.id);option.value=profile.id;select.appendChild(option);});
  if(observationState.configuration.profiles.some(function(p){return p.id===previous;}))select.value=previous;
  var topic=obsById('topic-list');obsClear(topic);
  Object.keys(observationState.configuration.topics||{}).forEach(function(k){topic.appendChild(obsNode('span',k+'：'+observationState.configuration.topics[k]));});
  if(!topic.children.length)topic.appendChild(obsNode('span','未配置观察话题'));
  var proposed=observationState.configuration.output_channels;
  obsApplyWiring();
  if(Array.isArray(proposed) && proposed.length===4 && !observationState.mapping && !observationState.wiringSuppressed)observationState.draft=proposed.slice();
  obsMappingControls();obsProfileDescription();
}
function obsProfileDescription() {
  var profile=observationState.configuration.profiles.find(function(p){return p.id===obsById('profile-select').value;});
  obsById('profile-description').textContent=profile ? (profile.description||'本地观察标签；不执行飞行。') : '没有配置片段标签。';
}
function obsMappingControls() {
  var root=obsById('mapping-controls');
  // Preserve a focused selector and its options across incoming telemetry.
  if(root.contains(document.activeElement))return;
  obsClear(root);
  observationState.draft.forEach(function(selected,i){
    var label=obsNode('label',obsMotorLabel(i));
    var select=obsNode('select');select.setAttribute('data-motor',String(i+1));
    var blank=obsNode('option','未选择 / 未核实');blank.value='';select.appendChild(blank);
    for(var channel=1;channel<=observationState.channelCount;channel++){
      var option=obsNode('option','raw channel '+channel);option.value=String(channel);select.appendChild(option);
    }
    select.value=selected||'';
    select.addEventListener('change',function(){observationState.draft[i]=select.value?Number(select.value):null;
      observationState.mapping=null;observationState.mappingSource=null;observationState.wiringSuppressed=true;obsRender();});
    label.appendChild(select);root.appendChild(label);
  });
}
function obsMetric(root,label,value,warn) { var box=obsNode('div',null,'metric');box.appendChild(obsNode('b',label));box.appendChild(obsNode('span',value,warn?'warn':''));root.appendChild(box); }
function obsPlot(id,series) {
  var canvas=obsById('chart-'+id),legend=obsById('legend-'+id);if(!canvas)return;
  var w=Math.max(280,canvas.clientWidth||400),h=Math.max(180,canvas.clientHeight||240),ratio=window.devicePixelRatio||1;
  canvas.width=Math.round(w*ratio);canvas.height=Math.round(h*ratio);
  var ctx=canvas.getContext('2d');ctx.scale(ratio,ratio);ctx.clearRect(0,0,w,h);
  var samples=observationState.samples.filter(function(s){return s.scope===obsScope();}),values=[];
  series.forEach(function(line){samples.forEach(function(s){var value=line.value(s);if(obsNumber(value)!==null)values.push(value);});});
  obsClear(legend);
  series.forEach(function(line,i){var item=obsNode('span',line.label);item.style.color=OBS_COLORS[i%4];legend.appendChild(item);});
  ctx.font='11px monospace';ctx.fillStyle='#9aaec8';
  if(!values.length){ctx.fillText('未观测 / 无有效样本',20,40);return;}
  var min=Math.min.apply(null,values),max=Math.max.apply(null,values),range=max-min||Math.max(.1,Math.abs(max)*.05);min-=range*.08;max+=range*.08;
  var left=60,right=w-12,top=14,bottom=h-30;
  var start=samples[0].at,end=samples[samples.length-1].at;
  function x(at){return left+(at-start)/Math.max(1,end-start)*(right-left);}
  function y(value){return bottom-(value-min)/(max-min)*(bottom-top);}
  for(var tick=0;tick<=4;tick++){
    var value=min+(max-min)*tick/4,yy=y(value);ctx.strokeStyle='#26364a';ctx.beginPath();ctx.moveTo(left,yy);ctx.lineTo(right,yy);ctx.stroke();ctx.fillText(value.toFixed(2),2,yy+4);
  }
  ctx.fillText(new Date(start*1000).toLocaleTimeString(),left,h-8);ctx.fillText(new Date(end*1000).toLocaleTimeString(),Math.max(left,right-70),h-8);
  series.forEach(function(line,i){
    ctx.strokeStyle=OBS_COLORS[i%4];ctx.lineWidth=1.7;ctx.beginPath();var drawing=false,previous=null;
    samples.forEach(function(s){var value=line.value(s);if(obsNumber(value)===null){drawing=false;previous=null;return;}
      if(previous && (s.at-previous.at>3 || (line.frame && line.frame(s)!==line.frame(previous))))drawing=false;
      if(drawing)ctx.lineTo(x(s.at),y(value));else ctx.moveTo(x(s.at),y(value));drawing=true;previous=s;
    });ctx.stroke();
    // A single sample is still visible.
    var last=samples[samples.length-1],latest=line.value(last);if(obsNumber(latest)!==null){ctx.fillStyle=OBS_COLORS[i%4];ctx.beginPath();ctx.arc(x(last.at),y(latest),2.5,0,Math.PI*2);ctx.fill();}
  });
}
function obsMakeChart(root,id,title) {
  var box=obsNode('div',null,'chart-box');box.appendChild(obsNode('h3',title));
  var canvas=obsNode('canvas');canvas.id='chart-'+id;canvas.setAttribute('aria-label',title);box.appendChild(canvas);
  var legend=obsNode('div',null,'legend');legend.id='legend-'+id;box.appendChild(legend);root.appendChild(box);
}
function obsCharts() {
  var pose=obsById('pose-charts'),attitude=obsById('attitude-charts'),diagnostic=obsById('diagnostic-charts');
  OBS_POSES.forEach(function(pair){obsMakeChart(pose,pair[0],pair[1]+' · x/y/z（m；各自frame）');
    obsMakeChart(attitude,pair[0]+'-attitude',pair[1]+' · roll/pitch/yaw（°）');});
  [['voltage','电压 V'],['current','电池电流 A'],['percentage','电量比例']].forEach(function(pair){obsMakeChart(diagnostic,'battery-'+pair[0],pair[1]);});
  obsMakeChart(diagnostic,'lio-age','LIO output age（s）');
}
function obsTable(headers,rows) {
  var table=obsNode('table'),head=obsNode('tr');headers.forEach(function(h){head.appendChild(obsNode('th',h));});table.appendChild(head);
  rows.forEach(function(values){var row=obsNode('tr');values.forEach(function(v){row.appendChild(obsNode('td',v));});table.appendChild(row);});return table;
}
function obsStatText(stats) { return stats && stats.count ? '均值 '+stats.mean.toFixed(3)+' / 范围 '+stats.min.toFixed(3)+'…'+stats.max.toFixed(3)+'（n='+stats.count+'）' : '未观测'; }
function obsRenderSegments() {
  var root=obsById('segment-table');obsClear(root);
  var segments=observationState.segments.concat(observationState.active?[observationState.active]:[]);
  if(!segments.length){root.appendChild(obsNode('p','暂无本地观察片段','hint'));return;}
  var rows=segments.map(function(segment){var s=obsSummary(segment),fc=s.poses.fc_pose;
    var poses=OBS_POSES.slice(0,3).map(function(pair){var p=s.poses[pair[0]];return pair[1]+' frame='+p.frames.join('/')+' local Z '+obsStatText(p.z)+'；roll '+obsStatText(p.roll_deg)+'；pitch '+obsStatText(p.pitch_deg)+'；yaw '+obsStatText(p.yaw_deg);}).join('\n');
    var outputs=s.mapping ? s.outputs.map(function(o,i){return obsMotorLabel(i,s.wiring)+' raw'+o.channel+' '+obsStatText(o.stats)+'；相对四路均值偏差 '+obsStatText(s.output_deviation[i].stats);}).join('\n') : '映射未核实，未生成电机四路统计';
    return [s.profile+' · '+(s.source.host||'来源未观测')+'\n'+obsTime(s.start)+' → '+(s.end?obsTime(s.end):'记录中'),s.window_sec.toFixed(1)+'s / '+s.retained_count+'/'+s.sample_count+'样本'+(s.truncated?' · 已截断':''),poses,
      'LIO age '+obsStatText(s.lio_age_sec)+'\n电压 '+obsStatText(s.battery.voltage)+'\n电流 '+obsStatText(s.battery.current)+'\n电量 '+obsStatText(s.battery.percentage),outputs,s.states.join('\n')||'未观测'];
  });root.appendChild(obsTable(['片段 / 日期','时间窗 / 保留样本','分帧位姿 m / 姿态 °','LIO / 电池','四路输出命令','观测状态'],rows));
}
function obsRender() {
  var now=Date.now()/1000,tel=observationState.telemetry,fresh=obsFresh(tel,now),data=obsNormalized(tel,now),connection=observationState.connection;
  var connected=connection.state==='ok';
  obsById('connection-status').textContent=(connected ? '共享工作台连接：'+(connection.host||'') : '工作台未连接：请回主页连接板端。')+' · SSE '+observationState.stream+' · '+(fresh?'遥测 '+obsTime(tel.at||tel.t):'遥测未观测或已过期')+(observationState.connectionNotice?' · '+observationState.connectionNotice:'');
  obsById('connection-status').className=fresh?'':'warn';
  var grid=obsById('status-grid');obsClear(grid);var fc=data.fc_state||{},low=data.low_hover||{};
  obsMetric(grid,'飞控模式',fc.mode||'未观测',!fc.mode);
  obsMetric(grid,'解锁状态',typeof fc.armed==='boolean'?(fc.armed?'已解锁':'未解锁'):'未观测',typeof fc.armed!=='boolean');
  obsMetric(grid,'飞控连接',typeof fc.connected==='boolean'?String(fc.connected):'未观测',fc.connected!==true);
  var managed=observationState.trial.group_id && ['running','starting'].indexOf(observationState.trialSession.state)>=0;
  obsMetric(grid,'观察来源',managed?'本工作台自管会话':'外部/手动节点观察（未记录自管试飞运行）');
  obsMetric(grid,'本工作台会话',managed?(observationState.trial.name||observationState.trial.group_id)+' · '+observationState.trialSession.state:'无运行记录');
  obsMetric(grid,'自管入口阶段',managed?observationState.stage.label||observationState.stage.name||'未观测':'未观测');
  obsMetric(grid,'通用任务phase / reason',[data.mission.phase||'未观测',data.mission.reason||'未观测'].join(' / '));
  var hover=data.terminal_hover||{};
  obsMetric(grid,'收尾悬停 / 交接',[hover.stage||'未观测',hover.reason||''].filter(Boolean).join(' / '));
  obsMetric(grid,'低空观察阶段',low.stage||'未观测',!low.stage);obsMetric(grid,'观察原因',low.reason||'未观测');
  obsMetric(grid,'实际入口profile / 点序号',(low.profile||'未观测')+' / '+(low.index==null?'未观测':low.index));
  obsMetric(grid,'板端录制器',typeof low.recorder_alive==='boolean'?(low.recorder_alive?'运行':'未运行'):'未观测',low.recorder_alive!==true);
  obsMetric(grid,'入口原始pose',low.pose ? JSON.stringify(low.pose) : '未观测');
  obsMetric(grid,'入口原始target',low.target ? JSON.stringify(low.target) : '未观测');
  OBS_POSES.forEach(function(pair){var p=data[pair[0]];obsMetric(grid,pair[1]+' frame / 数据年龄',p?p.frame+' / '+(p.unknown_age?'源时间未知':obsValue(p.source_age,'s'))+(p.stale?(p.source_age<0?' · 源时间异常':' · 位姿已过期（>0.3s）'):''):'未观测',!p||p.stale||p.unknown_age);});
  var raw=obsById('raw-outputs');obsClear(raw);var channels=data.rc_out && data.rc_out.channels;
  if(Array.isArray(channels)&&channels.length)channels.forEach(function(value,i){raw.appendChild(obsNode('span','raw '+(i+1)+' = '+obsValue(obsNumber(value)), 'raw-channel'));});
  else raw.appendChild(obsNode('span','RC OUT：未观测','warn'));
  var wiring=obsCurrentWiring();
  obsById('wiring-status').textContent=wiring ? wiring.confirmed_by+' · '+wiring.view+'；'+wiring.rc_layout.description : '此连接目标没有已登记接线，请手动核实原始通道。';
  obsMappingControls();obsById('mapping-status').textContent=observationState.mapping ?
    (observationState.mappingSource==='user-confirmed-wiring'?'已按用户接线配置：':'已本地确认：')+
    observationState.mapping.map(function(c,i){return obsMotorLabel(i)+'←raw'+c;}).join('，')+
    (!Array.isArray(channels)?' · 等待RC OUT遥测':(observationState.mapping.some(function(c){return c>channels.length;})?' · 尚未收到所选原始通道':'')) : '四路映射未核实；不显示电机曲线';
  obsById('mapping-status').className=observationState.mapping?'':'warn';
  obsPlot('motors',observationState.mapping ? observationState.mapping.map(function(channel,i){return {label:obsMotorLabel(i)+' · raw'+channel+'（输出命令）',value:function(s){return obsRaw(s,channel);}};}) : []);
  var esc=obsById('esc-table');obsClear(esc);var entries=[];
  ['esc_status','esc_telemetry'].forEach(function(key){var value=data[key];if(value && Array.isArray(value.entries))value.entries.forEach(function(entry,slot){entries.push([key,entry.index==null?'ESC条目 '+(Number.isInteger(entry.slot)?entry.slot+1:slot+1)+'（物理映射未核实）':'原始index '+entry.index,
    obsValue(obsNumber(entry.rpm)),obsValue(obsNumber(entry.voltage),'V'),obsValue(obsNumber(entry.current),'A'),obsValue(obsNumber(entry.temperature),'°C')]);});});
  esc.appendChild(entries.length?obsTable(['来源','ESC条目 / 原始index','RPM','电压','电流','温度'],entries):obsNode('p','ESC：未观测；不能从RC OUT推算电流或RPM。','hint'));
  OBS_POSES.forEach(function(pair){
    function series(keys){return keys.map(function(key){return {label:key,value:function(s){return s.data[pair[0]]&&s.data[pair[0]][key];},frame:function(s){return s.data[pair[0]]&&s.data[pair[0]].frame;}};});}
    obsPlot(pair[0],series(['x','y','z']));obsPlot(pair[0]+'-attitude',series(['roll_deg','pitch_deg','yaw_deg']));
  });
  ['voltage','current','percentage'].forEach(function(key){obsPlot('battery-'+key,[{label:key,value:function(s){return s.data.battery&&obsNumber(s.data.battery[key]);}}]);});
  obsPlot('lio-age',[{label:'output_age_sec',value:function(s){return obsLioAge(s.data);}}]);
  var diag=obsById('diagnostic-status');obsClear(diag);
  if(!data.lio_realtime.length)diag.appendChild(obsNode('p','LIO诊断：未观测','warn'));
  data.lio_realtime.forEach(function(d){diag.appendChild(obsNode('p',(d.name||'LIO')+' · level='+d.level+' · '+(d.message||'未观测')+' · '+JSON.stringify(d.values||{}),Number(d.level)>0?'warn':'hint'));});
  obsById('segment-start').disabled=!!observationState.active||!observationState.configuration.profiles.length;
  obsById('segment-end').disabled=!observationState.active;
  obsById('record-status').textContent='本页保留 '+observationState.samples.length+'/'+obsLimit()+' 点 · 最多保留 '+OBS_SEGMENT_LIMIT+' 个片段'+(observationState.dropped?' · 已截断最早 '+observationState.dropped+' 点，片段统计仅使用保留样本':'')+(observationState.segmentsDropped?' · 最早 '+observationState.segmentsDropped+' 个片段已移除':'')+(observationState.active?' · 正在本地记录 '+observationState.active.profile:'');
  obsById('record-status').className=observationState.dropped?'warn':'hint';obsRenderSegments();
}
function obsSnapshot(snapshot) {
  obsSetConnection(snapshot.connection,true);
  observationState.stage=snapshot.stage||{};observationState.trial=snapshot.trial||{};observationState.trialSession=(snapshot.sessions||{}).trial||{};
  obsConfigure(snapshot.observation);if(snapshot.telemetry)obsIngest(snapshot.telemetry,Date.now()/1000);obsRender();
}
function obsEvent(message) {
  if(message.t==='hello' || message.t==='snapshot')obsSnapshot(message.snapshot||{});
  else if(message.t==='connection'){obsSetConnection(message.connection,false);obsRender();}
  else if(message.t==='telemetry'){obsIngest(message.telemetry,Date.now()/1000);obsRender();}
  else if(message.t==='stage'){observationState.stage=message.stage||{};obsRender();}
  else if(message.t==='trial'){observationState.trial=message.trial||{};obsRender();}
  else if(message.t==='session' && message.s==='trial'){observationState.trialSession=message.session||{};obsRender();}
}
function obsInit() {
  var motor=window.location.pathname==='/motor';document.body.classList.toggle('motor-page',motor);
  document.title=motor?'Liftrace 低空电机只读观察':'Liftrace 只读实时状态';obsById('page-title').textContent=motor?'低空电机观察 · 只读大页':'实时状态曲线 · 只读';
  if(!motor){var panel=obsById('motor-panel');panel.parentNode.insertBefore(panel,obsById('record-panel').nextSibling);}
  obsCharts();obsMappingControls();
  obsById('profile-select').addEventListener('change',obsProfileDescription);
  obsById('segment-start').addEventListener('click',function(){obsStartSegment(obsById('profile-select').value,Date.now()/1000);obsRender();});
  obsById('segment-end').addEventListener('click',function(){obsEndSegment(Date.now()/1000);obsRender();});
  obsById('confirm-mapping').addEventListener('click',function(){if(!obsConfirmMapping(observationState.draft)){
    obsById('mapping-status').textContent='请选择4个不同且有效的原始通道，确认实际接线后再保存本地映射。';return;}obsRender();});
  obsById('clear-mapping').addEventListener('click',function(){observationState.mapping=null;observationState.draft=[null,null,null,null];
    observationState.mappingSource=null;observationState.wiringSuppressed=true;obsRender();});
  obsById('reset-local').addEventListener('click',function(){observationState.samples=[];observationState.segments=[];observationState.active=null;observationState.dropped=0;observationState.segmentsDropped=0;obsRender();});
  obsById('export-json').addEventListener('click',function(){obsDownload(obsJSON(),'application/json','json');});
  obsById('export-csv').addEventListener('click',function(){obsDownload(obsCSV(),'text/csv;charset=utf-8','csv');});
  window.addEventListener('resize',obsRender);
  var wiringURL=document.body.getAttribute('data-wiring-config');
  if(wiringURL)window.fetch(wiringURL,{method:'GET',cache:'no-store'}).then(function(r){if(!r.ok)throw new Error('wiring HTTP '+r.status);return r.json();})
    .then(function(wiring){if(!obsSetWiring(wiring))throw new Error('Invalid motor wiring');obsRender();})
    .catch(function(){obsById('wiring-status').textContent='接线配置读取失败，请手动核实映射。';});
  window.fetch('/api/snapshot',{method:'GET'}).then(function(r){if(!r.ok)throw new Error('snapshot HTTP '+r.status);return r.json();}).then(obsSnapshot).catch(function(){observationState.stream='快照读取失败';obsRender();});
  var events=new EventSource('/api/events');observationState.eventSource=events;
  events.onopen=function(){observationState.stream='已连接';obsRender();};
  events.onerror=function(){observationState.stream='断开 / 等待重连';obsRender();};
  events.onmessage=function(event){try{obsEvent(JSON.parse(event.data));}catch(e){observationState.stream='数据解析失败';}};
  window.setInterval(obsRender,1000);
}
if(document.readyState==='loading')document.addEventListener('DOMContentLoaded',obsInit);else obsInit();
