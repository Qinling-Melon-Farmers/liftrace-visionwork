// Pure browser recording/summary regressions; no HTTP, SSH, ROS or hardware.
const fs=require('fs'),path=require('path'),vm=require('vm'),assert=require('assert');
const source=fs.readFileSync(path.join(__dirname,'../web/observe.js'),'utf8');
const ctx={console,document:{readyState:'loading',addEventListener(){}},window:{}};
vm.createContext(ctx);vm.runInContext(source,ctx);
const plain=value=>JSON.parse(JSON.stringify(value));
const state=ctx.observationState;
state.configuration={max_samples:3,topics:{},profiles:[{id:'hover'},{id:'forward'},{id:'square'}]};
function tel(at,extra={}) {
  return {at,state:{connected:true,armed:true,mode:'OFFBOARD'},observe:{
    fc_pose:{frame:'map',stamp:at,source_age:.1,x:1,y:2,z:10,roll_deg:1,pitch_deg:2,yaw_deg:3},
    lio_pose:{frame:'camera_init',stamp:at,source_age:.2,x:20,y:30,z:40},
    ev_pose:null,setpoint:null,rc_out:{channels:[1000+at,1100+at,1200+at,1300+at]},
    battery:{voltage:16,current:2,percentage:.6},low_hover:{stage:'FINISHED_HOVER',profile:'hover'}},
    lio_realtime:[{values:{output_age_sec:'0.12'}}],...extra};
}
assert(!ctx.obsStartSegment('unknown',99));
assert(ctx.obsStartSegment('hover',99));
assert(!ctx.obsStartSegment('forward',99),'Segments cannot overlap');
for(let at=100;at<=104;at++)assert(ctx.obsIngest(tel(at),at));
assert.strictEqual(state.samples.length,3);
assert.strictEqual(state.dropped,2);
assert(!ctx.obsIngest(tel(104),104),'Duplicate telemetry not counted twice');
assert(!ctx.obsIngest(tel(103),104),'Old snapshot cannot replace newer telemetry');
assert.strictEqual(state.telemetry.at,104);
assert(ctx.obsEndSegment(105));
assert(!ctx.obsEndSegment(106));
const summary=ctx.obsSummary(state.segments[0]);
assert.strictEqual(summary.sample_count,5);
assert.strictEqual(summary.retained_count,3);
assert.strictEqual(summary.truncated,true);
assert.strictEqual(summary.poses.fc_pose.z.mean,10,'Local Z is preserved and never converted to AGL');
assert.strictEqual(summary.poses.lio_pose.z.mean,40,'LIO coordinates remain in their own frame');
assert.strictEqual(summary.poses.ev_pose.z.count,0,'Missing values are not zero');
assert.strictEqual(summary.lio_age_sec.mean,.12);
const poseOld=tel(200);poseOld.observe.fc_pose.source_age=.31;
const old=ctx.obsNormalized(poseOld,200);
assert(old.fc_pose.stale && old.fc_pose.z===null && old.fc_pose.source_age===.31,'Pose age above .3s is marked stale and excluded');
const unknownTime=tel(201);unknownTime.observe.fc_pose.source_age=null;
assert(ctx.obsNormalized(unknownTime,201).fc_pose.unknown_age && ctx.obsNormalized(unknownTime,201).fc_pose.z===null,'Unknown source time never becomes fresh pose');
assert.strictEqual(ctx.obsNormalized(tel(200),203).fc_pose,null,'Telemetry older than 2s is not current');
const waiting=tel(202,{observe_status:{battery:{topic:'/mavros/battery',status:'waiting_message',publishers:['/mavros'],count:0,hz:0,age:null}}});
assert(ctx.obsTopicStatus('battery',waiting,202).includes('尚未收到消息'));
waiting.observe_status.battery.status='no_publisher';waiting.observe_status.battery.publishers=[];
assert(ctx.obsTopicStatus('battery',waiting,202).includes('没有已登记发布者'));
assert(ctx.obsTopicStatus('battery',waiting,205).includes('已过期'));
assert(ctx.obsTopicStatus('battery',tel(202),202).includes('旧探针'));
assert(!ctx.obsIngest(tel(200),203),'Expired telemetry does not enter curve samples');
assert.strictEqual(ctx.obsStats([null,undefined,NaN,Infinity]).count,0);
assert.strictEqual(ctx.obsStats([null,4,6]).mean,5);
state.samples[2].data.fc_pose.frame='other_frame';
const mixed=ctx.obsSummary(state.segments[0]);
assert.strictEqual(mixed.poses.fc_pose.z.count,0,'Statistics never average changed frames');
assert(mixed.poses.fc_pose.z.reason);
const exported=JSON.parse(ctx.obsJSON());
assert.strictEqual(exported.dropped_samples,2);
assert.strictEqual(exported.samples.length,3);
assert.strictEqual(exported.segments[0].truncated,true);
const csv=ctx.obsCSV();
assert(csv.includes('battery_voltage')&&csv.includes('lio_pose_frame')&&csv.includes('lio_output_age_sec'));
assert(!/rc_out|esc_|mapping|motor/i.test(csv));
assert(!/rc_out|esc_|mapping|motor/i.test(JSON.stringify(exported.samples)));
assert.strictEqual(summary.battery.voltage.mean,16);
assert.strictEqual(summary.poses.fc_pose.roll_deg.mean,1);
assert(csv.includes('"map"')&&csv.includes('"camera_init"'));
assert.strictEqual(ctx.obsCsvCell('OFF"BOARD'),'"OFF""BOARD"');
state.configuration.max_samples=20000;assert.strictEqual(ctx.obsLimit(),1800);
for(let i=0;i<26;i++){assert(ctx.obsStartSegment('square',300+i));assert(ctx.obsEndSegment(300+i+.5));}
assert.strictEqual(state.segments.length,24,'Segment metadata is also bounded');
assert.strictEqual(state.segmentsDropped,3);
ctx.obsSetConnection({host:'board-a',board_root:'/one'},true);
assert(ctx.obsStartSegment('hover',500));ctx.obsIngest(tel(500),500);
ctx.obsSetConnection({host:'board-b',board_root:'/two'},true);
assert.strictEqual(state.active,null,'Host change finishes the old local segment');
assert.strictEqual(state.samples[state.samples.length-1].source.host,'board-a','Original sample source is kept');
assert(state.connectionNotice);
assert(!/\/api\/(?:action|trial|session|connect|config)/.test(source),'Observer has no action API');
assert(!/method:\s*['"]POST/.test(source));
assert(!/image_raw|jpeg|toDataURL|drawImage/i.test(source),'No camera/image/JPEG processing');
const html=fs.readFileSync(path.join(__dirname,'../web/observe.html'),'utf8');
const css=fs.readFileSync(path.join(__dirname,'../web/observe.css'),'utf8');
assert(!/motor-panel|mapping-controls|raw-outputs|chart-motors|esc-table|wiring/i.test(html));
assert(!/motor-page|mapping-controls|raw-channel/.test(css));
assert(!/obsMotor|obsMapping|obsSetWiring|motor_wiring/.test(source));
console.log('PASS observer: no motor panel, pose/attitude/battery diagnostics, target scope, bounded recording, stale data, JSON/CSV and read-only contract');
