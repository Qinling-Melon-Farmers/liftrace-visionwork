// Exercise actual status projection for snapshots and SSE, without HTTP/devices.
const fs=require('fs'),vm=require('vm'),path=require('path'),assert=require('assert');
const ctx={document:{readyState:'loading',addEventListener(){},querySelector(){return null;}},window:{},console};
vm.createContext(ctx);
vm.runInContext(fs.readFileSync(path.join(__dirname,'../web/app.js'),'utf8'),ctx);
ctx.initTermsFromSnapshot=()=>{};ctx.scheduleRender=()=>{};ctx.setSse=()=>{};
const cases=[
  [{},'idle','未初始化'],
  [{state:'starting',exit_code:null},'starting','初始化中'],
  [{state:'running',exit_code:null},'starting','初始化中'],
  [{state:'exited',exit_code:0},'succeeded','初始化成功'],
  [{state:'failed',exit_code:1},'failed','初始化失败'],
  [{state:'exited',exit_code:2},'failed','初始化失败'],
  [{state:'failed',exit_code:143},'failed','初始化失败'],
  [{state:'exited',exit_code:null},'exited','已退出（初始化结果未知）'],
  [{state:'running',exit_code:0},'starting','初始化中'],
];
for(const [session,cls,label] of cases){
  ctx.applySnapshot({sessions:{servo_init:session}});
  let status=ctx.termStatus('servo_init');
  assert.strictEqual(status.cls,cls);assert.strictEqual(status.label,label);
  ctx.bus.dispatch({t:'session',s:'servo_init',session});
  status=ctx.termStatus('servo_init');
  assert.strictEqual(status.cls,cls);assert.strictEqual(status.label,label);
}
for(const id of ['servo','mavros','roscore','lidar','camera']){
  ctx.state.sessions[id]={state:'exited',exit_code:0};
  assert.strictEqual(ctx.termStatus(id).cls,'exited');
  assert.strictEqual(ctx.termStatus(id).label,'已退出');
}
console.log('PASS 9 one-shot states via snapshot and SSE; 5 persistent terminals keep exited semantics');
if(process.argv[2]){
  const snapshot=JSON.parse(fs.readFileSync(process.argv[2],'utf8'));
  ctx.applySnapshot(snapshot);
  const init=ctx.termStatus('servo_init'),servo=ctx.termStatus('servo');
  assert.strictEqual(init.cls,'succeeded');assert.strictEqual(init.label,'初始化成功');
  assert.notStrictEqual(servo.cls,'succeeded');
  console.log(JSON.stringify({readonlySnapshotProjection:{servo_init:{state:init.state,cls:init.cls,label:init.label},
    servo:{state:servo.state,cls:servo.cls,label:servo.label}}}));
}
