/* Supports standalone or concatenated execution; mocked DOM/fetch only. */
if (typeof logInit === 'undefined') {
  require('vm').runInThisContext(require('fs').readFileSync(require('path').join(__dirname,'../web/logs.js'),'utf8'));
}
const assert = require('assert');
class LogElement {
  constructor(tag) { this.tagName=tag;this.childNodes=[];this.dataset={};this.value='';this.textContent='';this.disabled=false; }
  append(...nodes) { this.childNodes.push(...nodes); }
  replaceChildren(...nodes) { this.childNodes=nodes; }
  remove() {}
  click() { this.clicked=true; }
}
const ids={};
for(const id of ['log-connection','log-refresh','log-at','log-message','bag-profile','bag-duration','bag-start',
  'ulog-index','ulog-select','ulog-id','ulog-fetch','log-recorders','log-jobs','log-filter','log-files',
  'log-login','log-connect','log-host','log-password','log-save-password'])ids[id]=new LogElement('div');
ids['bag-profile'].value='routine';ids['bag-duration'].value='300';
global.document={getElementById:id=>ids[id],createElement:tag=>new LogElement(tag),body:new LogElement('body'),
  querySelectorAll:()=>{const all=[];function visit(n){if(n.dataset.stop)all.push(n);n.childNodes.forEach(visit);}visit(ids['log-jobs']);return all;}};
global.window={confirm:()=>true};
const calls=[],timers=[],saved=[];
global.setInterval=fn=>timers.push(fn);global.setTimeout=()=>{};
global.URL={createObjectURL:b=>{saved.push(b);return 'blob:fixture';},revokeObjectURL:()=>{}};
let fixtureConnection={state:'unknown',host:'orangepi@fixture',board_root:'/fixture'};
let fixtureStatus={ok:true,jobs:[],recorders:[],files:[],available:{bag:true,routine:true,ulog:true}};
let downloadSize=3;
let fixtureCapabilities={recording:true};
global.fetch=async(path,options)=>{
  const body=options&&options.body?JSON.parse(options.body):undefined;calls.push({path,body});
  let data=path==='/api/snapshot'?{ok:true,connection:fixtureConnection,capabilities:fixtureCapabilities}:fixtureStatus;
  if(path==='/api/recording/index')data={ok:true,entries:[{log_id:7,time_beijing:null,bytes:42}]};
  return {ok:true,json:async()=>data,blob:async()=>({size:downloadSize})};
};
const settle=()=>new Promise(resolve=>setImmediate(resolve));
(async()=>{
  logInit();await settle();
  assert.deepStrictEqual(calls.map(c=>c.path),['/api/snapshot']);assert(ids['bag-start'].disabled);
  assert(!calls.some(c=>/start|stop|index/.test(c.path)));
  fixtureConnection={...fixtureConnection,state:'ok'};await logAction();
  assert(!ids['bag-start'].disabled && ids['ulog-fetch'].disabled);
  ids['ulog-id'].value='7'; logButtons(); assert(!ids['ulog-fetch'].disabled);
  fixtureStatus.recorders=[{kind:'bag',pid:42}];await logAction();
  assert(ids['bag-start'].disabled);assert(!ids['ulog-fetch'].disabled);
  fixtureStatus.recorders=[];fixtureStatus.jobs=[{id:'bag_fixture',kind:'bag',pid:43,active:true,state:'recording',summary:{},tail:'<script>unsafe</script>'}];
  await logAction();assert(ids['bag-start'].disabled && ids['ulog-fetch'].disabled && ids['ulog-index'].disabled);
  const stop=document.querySelectorAll('[data-stop]')[0];assert.equal(stop.dataset.stop,'bag_fixture');
  stop.onclick();await settle();assert(calls.some(c=>c.path==='/api/recording/stop' && c.body.confirm==='停止日志操作'));
  fixtureStatus.jobs=[];await logAction();await logIndex();assert.equal(ids['ulog-id'].value,'');
  assert.equal(ids['ulog-select'].childNodes[1].value,7);
  ids['ulog-select'].value='7'; ids['ulog-select'].onchange();
  assert.equal(ids['ulog-id'].value,'7'); assert(!ids['ulog-fetch'].disabled);
  ids['ulog-id'].value='';ids['ulog-fetch'].onclick();assert(ids['log-message'].textContent.includes('请输入'));
  ids['ulog-select'].value='7';ids['ulog-select'].onchange();ids['ulog-fetch'].onclick();await settle();
  assert(calls.some(c=>c.path==='/api/recording/start'&&c.body.kind==='ulog'&&c.body.log_id===7));
  global.ulogBoardView=async file=>saved.push(file);
  logState.pendingUlog='ulog_fixture';
  fixtureStatus.jobs=[{id:'ulog_fixture',kind:'ulog',active:true,state:'transferring'}];
  fixtureStatus.files=[{path:'flight_workbench_recordings/ulog_fixture/flight.ulg',size:3,downloadable:false}];
  await logAction();assert.equal(saved.length,0);
  fixtureStatus.jobs[0].active=false;fixtureStatus.jobs[0].state='complete';fixtureStatus.files[0].downloadable=true;
  await logAction();assert.equal(saved.length,1);assert.equal(logState.pendingUlog,null);
  await logAction();assert.equal(saved.length,1);saved.length=0;fixtureStatus.jobs=[];fixtureStatus.files=[];
  ids['bag-duration'].value='0';ids['bag-start'].onclick();assert(ids['log-message'].textContent.includes('1–900'));
  ids['bag-duration'].value='60';ids['bag-profile'].value='diagnostic';ids['bag-start'].onclick();await settle();
  assert(calls.some(c=>c.path==='/api/recording/start'&&c.body.profile==='diagnostic'&&c.body.duration===60));
  await logDownload({path:'board_fixture/closed.bag',size:4});assert.equal(saved.length,0);
  assert(ids['log-message'].textContent.includes('不一致'));
  await logDownload({path:'board_fixture/closed.bag',size:3});assert.equal(saved.length,1);
  assert(calls.some(c=>c.path==='/api/recording/download?path=board_fixture%2Fclosed.bag'));
  ids['ulog-id'].value='7';fixtureConnection={...fixtureConnection,host:'orangepi@new-fixture'};await logAction();
  assert.equal(ids['ulog-id'].value,'');
  assert(!calls.some(c=>/logging_start|param\/set|ssh.*password/.test(c.path)));
  fixtureCapabilities={}; const prior=calls.length;await logAction();
  assert.deepStrictEqual(calls.slice(prior).map(c=>c.path),['/api/snapshot']);
  assert(ids['bag-start'].disabled && ids['ulog-index'].disabled);
  assert(ids['log-message'].textContent.includes('当前后端尚未支持'));
  console.log('PASS: offline, duplicate bag, owned stop, explicit index/download, input, size and target changes');
})().catch(error=>{console.error(error);process.exitCode=1;});
