'use strict';
let lowCoverageContext=null;
let site, context, mode='select', selected=null, history=[], drag=null, previewTimer, previewVersion=0;
const $=id=>document.getElementById(id), map=$('map');
const colors={survey:'#1769a4',corridor:'#ca6821',staging:'#667c8e',landing:'#107c72'};
const names={survey:'搜索航点',corridor:'走廊航点',staging:'准备点',landing:'H 降落点',select:'选择 / 拖动'};
let scale=1, left=0, bottom=0, currentStep=1;
const stepNames=['坐标与范围','搜索路线','走廊与H','检查与保存'];
function showStep(step){
  currentStep=Math.max(1,Math.min(4,Number(step)));
  document.querySelectorAll('[data-step]').forEach(b=>{b.classList.toggle('active',Number(b.dataset.step)===currentStep);if(Number(b.dataset.step)===currentStep)b.setAttribute('aria-current','step');else b.removeAttribute('aria-current');});
  document.querySelectorAll('[data-panel]').forEach(p=>p.hidden=Number(p.dataset.panel)!==currentStep);
  $('point-inspector').hidden=![2,3].includes(currentStep);
  $('step-prev').disabled=currentStep===1;$('step-next').hidden=currentStep===4;
  $('step-position').textContent=currentStep+' / 4';
  $('step-next').textContent='下一步：'+(stepNames[currentStep]||'')+' →';
}
function chooseStep(step){
  showStep(step);mode='select';selected=currentStep===2?{kind:'survey',i:0}:currentStep===3?{kind:'corridor',i:0}:null;
  document.querySelectorAll('[data-mode]').forEach(b=>b.classList.toggle('active',b.dataset.mode==='select'));
  $('hint').textContent=['','先确认右侧范围；图上右方为机头前方，起飞点始终为(0,0)。','生成路线后可拖动已有点；要增加点，请明确选择上方“＋搜索点”。','先点“＋走廊点”按顺序添加；再点“设置H中心”。点击已有点可精确输入坐标。','核对连线顺序与右侧检查结果；保存只更新本机草稿。'][currentStep];
  if(site){draw();list();}
}
function selectMode(next){
  mode=next;selected=null;
  if(next!=='select')showStep(['survey','staging'].includes(next)?2:3);
  document.querySelectorAll('[data-mode]').forEach(b=>b.classList.toggle('active',b.dataset.mode===mode));
  $('hint').textContent=mode==='select'?'点击已有点选择或拖动；空白处不会添加点。':'当前操作：'+names[mode]+'。点击空白处'+(['staging','landing'].includes(mode)?'设置位置（替换原位置）':'追加一个点')+'，也可拖动已有点。';
  draw();list();
}
function patternSelection(){document.querySelectorAll('[data-pattern]').forEach(b=>{const on=b.dataset.pattern===$('route-pattern').value;b.classList.toggle('active',on);b.setAttribute('aria-pressed',String(on));});}
function draftSummary(){
  const n=v=>Number.isFinite(v)?v+'m':'未提供';
  $('height-summary').textContent='继承所选配置：高位 FC AGL '+n(site.high_agl)+'，软件上限 '+n(site.max_agl)+'。选“三线”只改图形；若需三线专用高度，请先在主卡片切换对应YAML。';
  const h=site.landing_xy;
  $('draft-summary').textContent='搜索 '+points('survey').length+' 点 · 走廊 '+points('corridor').length+' 点 · H '+(h?'('+h.join(', ')+')':'尚未设置')+'。起飞原点(0,0)，所有AGL为FC中心离地高度。';
  $('coverage-legend').hidden=!site.flight_area.coverage_bounds;
}
function checkSummary(errors){
  $('check-status').replaceChildren();
  const msg=document.createElement('span');msg.textContent=errors.length?'还有 '+errors.length+' 项坐标问题需要处理。':'坐标检查通过；保存后还需运行卡片的配置检查。';$('check-status').append(msg);
  if(errors.length){const b=document.createElement('button');b.textContent='查看问题';b.onclick=()=>chooseStep(4);$('check-status').append(b);}
}
function readableError(text){
  const fields={center_bounds:'FC飞行边界',target_bounds:'目标范围',search_bounds:'搜索范围',coverage_bounds:'低空覆盖范围',map_size:'规划地图尺寸',staging_xy:'准备点S',survey_xy:'搜索航点',search_line_x:'原配置扫描线X',site_confirmed:'实测场地确认',landing_xy:'H中心',corridor_waypoints:'走廊航点'};
  let result=String(text);for(const [key,label]of Object.entries(fields))result=result.replaceAll(key,label+'（'+key+'）');return result;
}
function toast(text){$('toast').textContent=text;$('toast').hidden=false;clearTimeout(toast.timer);toast.timer=setTimeout(()=>$('toast').hidden=true,5000);}
function remember(){history.push(JSON.stringify(site));if(history.length>80)history.shift();++previewVersion;clearTimeout(previewTimer);$('export').disabled=true;}
function competition(){return context?.editor_kind==='competition';}
function identity(){return {group_id:context.group_id,...(competition()?{competition_config:context.competition_config}:{}),scope:context.scope};}
function normalize(value){const next=structuredClone(value);next.corridor_waypoints??=[];next.flight_area.survey_xy??=[];return next;}
function coordinateSite(value){
  if(!competition())return normalize(value);
  // Imports and saves change geometry only; flight and H strategy remain at source.
  const next=structuredClone(initialSite||site);
  if(!Object.hasOwn(value,'corridor_geometry'))delete next.corridor_geometry;
  for(const key of ['corridor_waypoints','landing_xy','corridor_geometry','site_confirmed'])if(Object.hasOwn(value,key))next[key]=structuredClone(value[key]);
  for(const key of ['center_bounds','target_bounds','search_bounds','coverage_bounds','staging_xy','survey_xy','map_size'])if(Object.hasOwn(value.flight_area||{},key))next.flight_area[key]=structuredClone(value.flight_area[key]);
  return normalize(next);
}
function points(kind){if(kind==='survey'&&lowCoverageContext)return [];if(kind==='survey')return site.flight_area.survey_xy;if(kind==='corridor')return site.corridor_waypoints.map(p=>[p.x,p.y]);return [kind==='staging'?site.flight_area.staging_xy:site.landing_xy].filter(Boolean);}
function activeKind(){return mode==='select'?(selected?.kind||'corridor'):mode;}
function updatePoint(kind,i,x,y){if(kind==='survey')site.flight_area.survey_xy[i]=[x,y];else if(kind==='corridor')Object.assign(site.corridor_waypoints[i],{x,y});else site[kind==='landing'?'landing_xy':'flight_area']=kind==='landing'?[x,y]:{...site.flight_area,staging_xy:[x,y]};}
function screen(x,y){return [left+x*scale,bottom-y*scale];}
function world(event){const p=map.createSVGPoint();p.x=event.clientX;p.y=event.clientY;const q=p.matrixTransform(map.getScreenCTM().inverse());return [(q.x-left)/scale,(bottom-q.y)/scale];}
function snapped(v){const step=Number($('snap').value);return +(step?Math.round(v/step)*step:v).toFixed(3);}
function svg(tag,attrs,text){const el=document.createElementNS('http://www.w3.org/2000/svg',tag);for(const [key,value]of Object.entries(attrs))el.setAttribute(key,value);if(text!=null)el.textContent=text;map.append(el);return el;}
function draw(){
  const a=site.flight_area;
  const boxValid=box=>Array.isArray(box)&&box.length===4&&box.every(Number.isFinite)&&box[0]<box[1]&&box[2]<box[3]&&Math.max(...box.map(Math.abs))<1000;
  // Keep the last map visible while a partially edited range is invalid.
  const boxes=[a.center_bounds,a.target_bounds,a.search_bounds,...(a.coverage_bounds?[a.coverage_bounds]:[])];
  $('map-status').hidden=boxes.every(boxValid);if(!boxes.every(boxValid))return;
  map.replaceChildren();
  const width=map.clientWidth||1000,height=map.clientHeight||730;
  map.setAttribute('viewBox',`0 0 ${width} ${height}`);
  const ps=['survey','corridor','staging','landing'].flatMap(points).filter(p=>p.length===2&&p.every(Number.isFinite));ps.push([0,0]);
  const b=[Math.min(...boxes.map(v=>v[0]),...ps.map(v=>v[0])),Math.max(...boxes.map(v=>v[1]),...ps.map(v=>v[0])),Math.min(...boxes.map(v=>v[2]),...ps.map(v=>v[1])),Math.max(...boxes.map(v=>v[3]),...ps.map(v=>v[1]))];
  const x0=b[0]-.8,x1=b[1]+.8,y0=b[2]-.8,y1=b[3]+.8;
  const plotWidth=Math.max(200,width-115),plotHeight=Math.max(180,height-90);
  scale=Math.min(plotWidth/(x1-x0),plotHeight/(y1-y0));left=65+(plotWidth-(x1-x0)*scale)/2-x0*scale;bottom=height-48-(plotHeight-(y1-y0)*scale)/2+y0*scale;
  const rect=(box,attrs)=>{const [x,y]=screen(box[0],box[3]);svg('rect',{x,y,width:(box[1]-box[0])*scale,height:(box[3]-box[2])*scale,...attrs});};
  rect(a.center_bounds,{fill:'#f3f7fa',stroke:'#8d9fab','stroke-width':1.5});rect(a.search_bounds,{fill:'#dfeef9','fill-opacity':.45,stroke:'#80abc9','stroke-dasharray':'6 5'});
  rect(a.target_bounds,{fill:'none',stroke:'#107c72','stroke-dasharray':'3 4'});
  if(a.coverage_bounds)rect(a.coverage_bounds,{fill:'none',stroke:'#8763a0','stroke-dasharray':'8 4'});
  const step=scale<45?1:.5;
  for(let x=Math.ceil(x0/step)*step;x<=x1;x+=step){const p=screen(x,y0),q=screen(x,y1);svg('line',{x1:p[0],y1:p[1],x2:q[0],y2:q[1],stroke:'#d8e3ea','stroke-width':.7});svg('text',{x:p[0],y:height-28,'text-anchor':'middle',fill:'#718695','font-size':12},+x.toFixed(1));}
  for(let y=Math.ceil(y0/step)*step;y<=y1;y+=step){const p=screen(x0,y),q=screen(x1,y);svg('line',{x1:p[0],y1:p[1],x2:q[0],y2:q[1],stroke:'#d8e3ea','stroke-width':.7});svg('text',{x:49,y:p[1]+4,'text-anchor':'end',fill:'#718695','font-size':12},+y.toFixed(1));}
  svg('text',{x:width-20,y:height-7,'text-anchor':'end',fill:'#405e72','font-size':13},'X / m · 机头前方 →');svg('text',{x:18,y:28,fill:'#405e72','font-size':13},'+Y / m · 机身左侧 ↑');
  const g=site.corridor_geometry;if(g){for(const w of g.wall_coordinates||[]){if(!Number.isFinite(w))continue;const p=screen(g.wall_axis===1?b[0]:w,g.wall_axis===1?w:b[2]),q=screen(g.wall_axis===1?b[1]:w,g.wall_axis===1?w:b[3]);svg('line',{x1:p[0],y1:p[1],x2:q[0],y2:q[1],stroke:'#92a2ad','stroke-width':2,'stroke-dasharray':'9 6'});svg('text',{x:q[0]+8,y:q[1]-7,fill:'#728794','font-size':11},(g.wall_axis===1?'Y':'X')+'='+w);}}
  // Direction chevrons distinguish route order without implying a planned trajectory.
  for(const kind of ['survey','corridor']){
    const ps=points(kind);if(ps.length>1)svg('polyline',{points:ps.map(p=>screen(...p).join(',')).join(' '),fill:'none',stroke:colors[kind],'stroke-width':2.5,'stroke-linejoin':'round','stroke-opacity':.8});
    for(let i=1;i<ps.length;i++){
      const a=screen(...ps[i-1]),b=screen(...ps[i]),dx=b[0]-a[0],dy=b[1]-a[1],len=Math.hypot(dx,dy);if(len<35)continue;
      const ux=dx/len,uy=dy/len,x=(a[0]+b[0])/2,y=(a[1]+b[1])/2;
      svg('path',{d:`M${x-ux*6-uy*5},${y-uy*6+ux*5} L${x+ux*3},${y+uy*3} L${x-ux*6+uy*5},${y-uy*6-ux*5}`,fill:'none',stroke:colors[kind],'stroke-width':2.5,'data-route-arrow':kind});
    }
  }
  const origin=screen(0,0);
  svg('path',{d:`M${origin[0]-12},${origin[1]-9} L${origin[0]+13},${origin[1]} L${origin[0]-12},${origin[1]+9} L${origin[0]-7},${origin[1]}Z`,fill:'#203a50',stroke:'white','stroke-width':1.5});
  svg('text',{x:origin[0],y:origin[1]+35,fill:'#203a50','font-size':13,'text-anchor':'middle',stroke:'white','stroke-width':4,'paint-order':'stroke'},'起飞 (0,0) · 机头→');
  for(const kind of ['survey','corridor','staging','landing'])points(kind).forEach((p,i)=>{
    const equal=q=>q[0]===p[0]&&q[1]===p[1];
    if(points(kind).slice(0,i).some(equal))return;
    const indices=points(kind).map((q,j)=>equal(q)?j:-1).filter(j=>j>=0);
    const [x,y]=screen(...p);if(selected?.kind===kind&&indices.includes(selected.i))svg('circle',{cx:x,cy:y,r:24,fill:'none',stroke:colors[kind],'stroke-width':2,'stroke-opacity':.5});
    const circle=svg('circle',{cx:x,cy:y,r:['landing','staging'].includes(kind)?15:indices.length>1?25:18,fill:kind==='survey'?'white':colors[kind],stroke:colors[kind],'stroke-width':2,'data-kind':kind,'data-index':i});circle.style.cursor='grab';
    svg('text',{x,y:y+4,'text-anchor':'middle',fill:kind==='survey'?colors[kind]:'white','font-size':kind==='landing'?14:12,'font-weight':600,'pointer-events':'none'},kind==='landing'?'H':kind==='staging'?'S':(kind==='survey'?'搜':'廊')+indices.map(j=>j+1).join('/'));
    if(kind==='landing'){
      const same=points('corridor').map((q,j)=>equal(q)?j+1:0).filter(Boolean);
      if(same.length)svg('text',{x:x+23,y:y+5,fill:colors.landing,'font-size':11,stroke:'white','stroke-width':3,'paint-order':'stroke','pointer-events':'none'},'与廊'+same.join('/')+'同XY');
    }
  });
}
function list(){
  const kind=activeKind();$('list-title').textContent=names[kind];$('count').textContent=points(kind).length+' 个';$('points').replaceChildren();
  $('list-note').textContent=selected?.kind===kind?'已选中'+names[kind]+(['survey','corridor'].includes(kind)?' '+(selected.i+1):'')+'，可直接输入 X、Y'+(kind==='corridor'?' 和 AGL':'')+' 微调，地图与导出配置同步更新。':'按编号依次飞行，点击地图上的点可输入坐标微调。AGL 为飞控中心离地高度。';
  $('default-agl').parentElement.hidden=kind!=='corridor';$('clear').hidden=!['survey','corridor'].includes(kind);$('close-route').hidden=kind!=='survey';$('join-h').hidden=kind!=='corridor';
  const ps=points(kind);if(!ps.length){const p=document.createElement('p');p.className='empty';p.textContent='在左侧地图点击，按飞行顺序添加航点。';$('points').append(p);}
  ps.forEach((p,i)=>{
    const row=document.createElement('div');row.className='point'+(selected?.kind===kind&&selected.i===i?' selected':'');
    const top=document.createElement('div');top.className='point-top';const title=document.createElement('strong');title.textContent=['staging','landing'].includes(kind)?names[kind]:names[kind]+' '+(i+1);top.append(title);
    title.onclick=()=>{selected={kind,i};draw();list();};const buttons=document.createElement('div');
    if(['survey','corridor'].includes(kind))for(const [label,delta]of [['上移',-1],['下移',1],['删除',0]]){const button=document.createElement('button');button.textContent=label;button.disabled=delta!==0&&(i+delta<0||i+delta>=ps.length);button.onclick=()=>{remember();const arr=kind==='survey'?site.flight_area.survey_xy:site.corridor_waypoints;if(delta){[arr[i],arr[i+delta]]=[arr[i+delta],arr[i]];selected={kind,i:i+delta};}else{arr.splice(i,1);selected=null;}render();};buttons.append(button);}
    top.append(buttons);row.append(top);const coords=document.createElement('div');coords.className='coords';
    for(const [label,value,index]of [['X',p[0],0],['Y',p[1],1],...(kind==='corridor'?[['AGL',site.corridor_waypoints[i].agl??1.4,2]]:[])]){const lab=document.createElement('label');lab.textContent=label+' / m';const input=document.createElement('input');input.type='number';input.step=index===2?'0.05':'0.01';input.value=Number.isFinite(value)?Number(value.toFixed(3)):value;input.setAttribute('aria-label',title.textContent+' '+label);input.oninput=()=>{if(!Number.isFinite(input.valueAsNumber))return;if(!input.dataset.editStarted){remember();input.dataset.editStarted='1';}const current=points(kind)[i];if(index===2)site.corridor_waypoints[i].agl=input.valueAsNumber;else updatePoint(kind,i,index===0?input.valueAsNumber:current[0],index===1?input.valueAsNumber:current[1]);selected={kind,i};draw();preview();document.getElementById('undo').disabled=false;};input.onblur=()=>{delete input.dataset.editStarted;};lab.append(input);coords.append(lab);}
    row.append(coords);$('points').append(row);
  });
}
function geometry(){
  $('geometry').replaceChildren();
  const specs=[
    ['FC飞行边界','center_bounds','FC中心允许运动的范围，须包含起飞原点与走廊/H；不是直接照抄物理外墙。'],
    ['搜索范围 · 蓝框','search_bounds',lowCoverageContext?'低位路线使用此实测范围，按卡片形状及内收/间距生成；不使用旧高位航点。':'高位搜索航点的生成范围，通常不含走廊。'],
    ['目标范围 · 绿框','target_bounds',competition()?'用于目标位置筛选，需在飞行边界内。':'用于目标位置筛选；本试飞模板要求它包含FC飞行边界。'],
    ...(competition()?[['低空覆盖范围 · 紫框','coverage_bounds','没有足够线索时低空补搜的范围，需位于搜索范围内。']]:[]),
    ['规划地图尺寸','map_size','地图X/Y以原点为中心。不是填边界坐标；尺寸不足会在检查中提示。']
  ];
  for(const [title,key,help]of specs){
    const advanced=['target_bounds','coverage_bounds','map_size'].includes(key);
    const row=document.createElement(advanced?'details':'div');row.className='geometry-row';row.dataset.range=key;
    const heading=document.createElement(advanced?'summary':'p');heading.className='geometry-title';heading.textContent=title+' / m'+(advanced?' · 展开':'');
    const note=document.createElement('p');note.className='geometry-help';note.textContent=help;
    const fields=document.createElement('div');fields.className='bounds-inputs'+(key==='map_size'?' dimensions':'');
    const labels=key==='map_size'?['X总长','Y总宽','Z总高']:['X最小 · 后侧','X最大 · 前侧','Y最小 · 右侧','Y最大 · 左侧'];
    labels.forEach((text,i)=>{const lab=document.createElement('label');lab.textContent=text;const input=document.createElement('input');input.type='number';input.step='0.05';input.value=site.flight_area[key]?.[i]??'';input.setAttribute('aria-label',title+' '+text);input.dataset.bound=key;input.dataset.index=i;
      input.oninput=()=>{if(!input.dataset.editStarted){remember();input.dataset.editStarted='1';}site.flight_area[key]??=Array(labels.length).fill(null);site.flight_area[key][i]=Number.isFinite(input.valueAsNumber)?input.valueAsNumber:null;draw();preview();$('undo').disabled=false;};
      input.onblur=()=>{delete input.dataset.editStarted;};lab.append(input);fields.append(lab);
    });row.append(heading,note,fields);$('geometry').append(row);
  }
  if(competition()){
    const label=document.createElement('label');label.className='site-confirmation';const confirmed=document.createElement('input');confirmed.type='checkbox';confirmed.checked=site.site_confirmed===true;confirmed.id='site-confirmed';confirmed.onchange=()=>{remember();site.site_confirmed=confirmed.checked;render();};label.append(confirmed,document.createTextNode('已按实测确认正赛场地（不是仅确认这张示意图）'));$('geometry').append(label);
  }
  const g=site.corridor_geometry||{wall_axis:1,wall_coordinates:[],entry_waypoints:1};$('wall-axis').value=g.wall_axis;$('walls').value=(g.wall_coordinates||[]).join(', ');$('entry').value=g.entry_waypoints;
}
async function api(url,body){if(lowCoverageContext&&body)body.group_id=lowCoverageContext.group_id;const r=await fetch(url,body?{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)}:{});if(!r.ok){const e=await r.json();throw new Error(e.error||'读取失败');}return r;}
function preview(){const version=++previewVersion;clearTimeout(previewTimer);$('export').disabled=true;previewTimer=setTimeout(async()=>{try{const data=await(await api('/api/waypoints/preview',{...identity(),site:coordinateSite(site)})).json();if(version!==previewVersion)return;if(data.ok===false)throw Error(data.error||'坐标检查失败');const errors=(data.errors||[]).map(readableError);checkSummary(errors);$('preview').textContent=data.yaml;$('validation').className=errors.length?'invalid':'';$('validation').replaceChildren();for(const text of errors.length?errors:['坐标检查通过，可保存到工作台']){const p=document.createElement('div');p.textContent=text;$('validation').append(p);}$('export').disabled=!!errors.length;}catch(e){if(version!==previewVersion)return;$('validation').className='invalid';$('validation').textContent=e.message;checkSummary([e.message]);}},180);}
function render(){draw();list();geometry();draftSummary();patternSelection();preview();$('undo').disabled=!history.length;}
map.addEventListener('pointerdown',event=>{const target=event.target.closest('[data-kind]');if(target){selected={kind:target.dataset.kind,i:Number(target.dataset.index)};mode='select';showStep(['survey','staging'].includes(selected.kind)?2:3);document.querySelectorAll('[data-mode]').forEach(button=>button.classList.toggle('active',button.dataset.mode==='select'));$('hint').textContent='已选中'+names[selected.kind]+'，在坐标面板输入数值微调，也可继续拖动。';drag={...selected,start:world(event),moved:false};map.setPointerCapture(event.pointerId);draw();list();return;}if(mode==='select')return;const [x,y]=world(event).map(snapped);remember();if(mode==='corridor'){site.corridor_waypoints.push({x,y,agl:Number($('default-agl').value)});selected={kind:mode,i:site.corridor_waypoints.length-1};}else if(mode==='survey'){site.flight_area.survey_xy.push([x,y]);selected={kind:mode,i:site.flight_area.survey_xy.length-1};}else{updatePoint(mode,0,x,y);selected={kind:mode,i:0};}render();});
map.addEventListener('pointermove',event=>{const coords=world(event);$('cursor').textContent='X '+coords[0].toFixed(2)+' m　Y '+coords[1].toFixed(2)+' m';if(!drag)return;if(!drag.moved&&Math.hypot(coords[0]-drag.start[0],coords[1]-drag.start[1])<.02)return;if(!drag.moved){remember();drag.moved=true;}updatePoint(drag.kind,drag.i,...coords.map(snapped));draw();list();});
function endDrag(event){if(drag?.moved)render();else if(drag&&event.type==='pointerup'){const row=$('points').querySelector('.selected');if(row){row.scrollIntoView({block:'nearest',behavior:'instant'});const input=row.querySelector('input');input?.focus({preventScroll:true});input?.select();}}drag=null;}map.addEventListener('pointerup',endDrag);map.addEventListener('pointercancel',endDrag);
document.querySelectorAll('[data-mode]').forEach(button=>button.onclick=()=>selectMode(button.dataset.mode));
document.querySelectorAll('[data-step]').forEach(button=>button.onclick=()=>chooseStep(button.dataset.step));
$('step-prev').onclick=()=>chooseStep(currentStep-1);$('step-next').onclick=()=>chooseStep(currentStep+1);
document.querySelectorAll('[data-pattern]').forEach(button=>button.onclick=()=>{$('route-pattern').value=button.dataset.pattern;patternSelection();});
$('route-pattern').onchange=patternSelection;
$('undo').onclick=()=>{if(history.length){site=JSON.parse(history.pop());selected=null;render();}};
$('clear').onclick=()=>{const kind=activeKind();if(!confirm('清空'+names[kind]+'？可撤销。'))return;remember();if(kind==='survey')site.flight_area.survey_xy=[];else site.corridor_waypoints=[];selected=null;render();};
$('close-route').onclick=()=>{const ps=site.flight_area.survey_xy;if(!ps.length)return;if(ps.length>=8){toast('搜索航点最多 8 个，请先删除一个点');return;}if(JSON.stringify(ps[0])===JSON.stringify(ps.at(-1)))return;remember();ps.push([...ps[0]]);render();};
$('join-h').onclick=()=>{if(!site.landing_xy||!site.corridor_waypoints.length)return;remember();const p=site.corridor_waypoints.at(-1);[p.x,p.y]=site.landing_xy;render();};
for(const id of ['wall-axis','walls','entry'])$(id).onchange=()=>{remember();const walls=$('walls').value.split(/[,，\s]+/).filter(Boolean).map(Number);if(competition()&&!walls.length)delete site.corridor_geometry;else site.corridor_geometry={wall_axis:Number($('wall-axis').value),wall_coordinates:walls,entry_waypoints:Number($('entry').value)};render();};
$('reset').onclick=()=>{if(!initialSite)return;remember();site=structuredClone(initialSite);selected=null;render();};
$('import').onchange=async event=>{const file=event.target.files[0];if(!file||!site)return;const before=JSON.stringify(site);try{if(file.size>2*1024*1024)throw new Error('配置文件不能超过 2MiB');const next=await(await api('/api/waypoints/import',{...identity(),text:await file.text()})).json();if(next.ok===false)throw Error(next.error||'导入失败');if(JSON.stringify(site)!==before)throw Error('坐标已变化，请重新导入');remember();site=coordinateSite(next.site||next);selected=null;render();toast('已导入 '+file.name);}catch(e){toast(e.message);}event.target.value='';};
$('generate-route').onclick=async()=>{
  if(!site)return;const before=JSON.stringify(site),button=$('generate-route');const inset=Number($('route-inset').value);
  if(!$('route-inset').value.trim()||!Number.isFinite(inset)||inset<0){toast('内收距离需填写非负有限数值');return;}
  button.disabled=true;
  try{
    const data=await(await api('/api/waypoints/route',{...identity(),site:coordinateSite(site),pattern:$('route-pattern').value,inset})).json();
    if(data.ok===false)throw Error(data.error||'路线生成失败');
    if(JSON.stringify(site)!==before)throw Error('坐标已变化，请重新生成路线');
    if(!Array.isArray(data.route)||!data.route.length||data.route.some(p=>!Array.isArray(p)||p.length!==2||!p.every(Number.isFinite)))throw Error('路线接口未返回有效XY航点');
    remember();site.flight_area.survey_xy=structuredClone(data.route);selected=null;mode='survey';showStep(2);document.querySelectorAll('[data-mode]').forEach(b=>b.classList.toggle('active',b.dataset.mode==='survey'));$('hint').textContent='已生成搜索路线，可拖动或输入坐标微调；保存后才应用到工作台。';render();toast('搜索点已更新；高度继承源配置，可撤销');
  }catch(e){toast(e.message);}finally{button.disabled=false;}
};
let initialSite=null;
$('export').onclick=()=>{if(!site||$('export').disabled)return;$('export').disabled=true;window.parent.postMessage({type:'waypoint-apply',site:coordinateSite(site)},location.origin);};
$('cancel-map').onclick=()=>window.parent.postMessage({type:'waypoint-cancel'},location.origin);
window.addEventListener('message',event=>{
  if(event.origin!==location.origin||event.source!==window.parent)return;
  if(event.data?.type==='waypoint-context'){
    if(event.data.context.folder==='10_low_coverage_full'&&!lowCoverageContext){lowCoverageContext=event.data.context;document.querySelectorAll('[data-mode=survey]').forEach(b=>b.hidden=true);const generate=document.getElementById('generate-route');if(generate)generate.hidden=true;const notice=document.createElement('p');notice.className='warn-line';notice.hidden=true;notice.textContent='备用牛耕候选：搜索范围是coverage_bounds；路线由板端模块生成。卡片选择FC 1.8/1.9m（不是镜头高度）。保留现有门/H点，不表示已补齐门前门后引导。导入后请点击保存并用于本组。';document.querySelector('header').after(notice);}

    if(initialSite)return;
    context=event.data.context;site=normalize(context.site);if(competition())site.site_confirmed=site.site_confirmed===true;initialSite=structuredClone(site);history=[];selected=null;
    if(competition()){
      $('default-agl').value=site.corridor_waypoints[0]?.agl??.9;$('default-agl').removeAttribute('max');
      $('wall-note').textContent='正赛正式检查目前要求入口序号为1。墙面参考使用原任务规则；清空墙坐标会继承原YAML的墙配置，不表示取消墙面保护。高度、速度和H策略不在此修改。';
    }
    showStep(1);render();
  }else if(event.data?.type==='waypoint-error'){toast(event.data.error);preview();}
});
window.addEventListener('resize',()=>{if(site)draw();});
window.parent.postMessage({type:'waypoint-ready'},location.origin);

// Low-coverage endpoints are a read-only overlay, separate from editable survey_xy.
const drawGeometryMap=draw;
draw=function(){
  drawGeometryMap();
  if(!lowCoverageContext||!site)return;
  let note=document.getElementById('coverage-route-summary');
  if(!note){note=document.createElement('p');note.id='coverage-route-summary';note.className='warn-line';document.querySelector('header').after(note);}
  map.querySelectorAll('[data-coverage-overlay]').forEach(el=>el.remove());
  try{
    const route=WaypointCoverage.route(site,lowCoverageContext);
    const group=document.createElementNS('http://www.w3.org/2000/svg','g');group.setAttribute('data-coverage-overlay','');group.setAttribute('pointer-events','none');map.append(group);
    const add=(tag,attrs,text)=>{const el=svg(tag,attrs,text);group.append(el);return el;};
    add('polyline',{points:route.points.map(p=>screen(...p).join(',')).join(' '),fill:'none',stroke:colors.survey,'stroke-width':3,'stroke-linejoin':'round','data-coverage-route':route.source});
    const seen=new Set();
    route.points.forEach((p,i)=>{
      const key=p.join(',');if(seen.has(key))return;seen.add(key);
      const indices=route.points.flatMap((q,j)=>q[0]===p[0]&&q[1]===p[1]?[j+1]:[]);
      const [x,y]=screen(...p);
      const marker=add('circle',{cx:x,cy:y,r:indices.length>1?22:16,fill:'white',stroke:colors.survey,'stroke-width':2,'data-coverage-point':i,'data-world-x':p[0],'data-world-y':p[1]});
      const title=document.createElementNS('http://www.w3.org/2000/svg','title');title.textContent='搜'+indices.join('/')+' · ('+p.join(', ')+') · FC '+route.agl+'m';marker.append(title);
      add('text',{x,y:y+4,'text-anchor':'middle',fill:colors.survey,'font-size':11,'font-weight':600},'搜'+indices.join('/'));
    });
    for(let i=1;i<route.points.length;i++){
      const [ax,ay]=screen(...route.points[i-1]),[bx,by]=screen(...route.points[i]),dx=bx-ax,dy=by-ay,len=Math.hypot(dx,dy);if(len<30)continue;
      const ux=dx/len,uy=dy/len,x=(ax+bx)/2,y=(ay+by)/2;
      add('path',{d:'M'+(x-ux*6-uy*5)+','+(y-uy*6+ux*5)+' L'+(x+ux*3)+','+(y+uy*3)+' L'+(x-ux*6+uy*5)+','+(y-uy*6-ux*5),fill:'none',stroke:colors.survey,'stroke-width':2,'data-coverage-arrow':i});
    }
    group.dataset.waypoints=JSON.stringify(route.points);group.dataset.agl=route.agl;
    note.textContent=(route.source==='site.search_waypoints_xy'?'显式搜索点':route.pattern==='nested_rectangles'?'低位回字形':'低位牛耕')+' · '+route.points.length+'点 · FC '+route.agl+'m · '+route.source+'。蓝线按板端顺序显示（含圈间折点），只读预览；走廊/H独立保留。';
  }catch(error){note.textContent='低位搜索线未生成：'+error.message;}
  const panel=document.querySelector('[data-panel="2"]');
  if(panel){
    for(const child of panel.children)child.hidden=child.tagName!=='H2'&&child.id!=='height-summary';
    panel.querySelector('h2').textContent='② 低位搜索路线预览';
    $('height-summary').textContent=note.textContent+' 形状和FC高度在主卡片选择；可在第①步修改范围。';
    document.querySelector('[data-step="2"] small').textContent='牛耕 / 回字形 · 只读预览';
    if(currentStep===2){$('point-inspector').hidden=true;$('hint').textContent='蓝色搜索线按当前卡片选择生成；编辑范围后自动更新，走廊/H可在第③步调整。';}
    const group=map.querySelector('[data-coverage-overlay]');
    $('draft-summary').textContent='低位搜索 '+(group?JSON.parse(group.dataset.waypoints).length:'未生成')+' 点 · 走廊 '+site.corridor_waypoints.length+' 点。形状和FC高度继承主卡片；旧survey_xy不作为低位搜索线。';
  }
};
