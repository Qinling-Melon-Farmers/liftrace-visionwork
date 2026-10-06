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
assert.strictEqual(state.mapping,null,'No guessed RC-to-motor mapping');
assert(!ctx.obsConfirmMapping([1,1,2,3]));
assert(!ctx.obsConfirmMapping([null,2,3,4]));
assert(!ctx.obsConfirmMapping([1,2,3,1.5]));
assert(ctx.obsConfirmMapping([4,2,1,3]));
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
assert.deepStrictEqual(plain(summary.mapping),[4,2,1,3]);
assert.strictEqual(summary.outputs[0].stats.mean,1403);
assert.strictEqual(summary.output_deviation[0].stats.mean,150,'Deviation is raw command relative to four selected outputs');
assert.strictEqual(summary.poses.fc_pose.z.mean,10,'Local Z is preserved and never converted to AGL');
assert.strictEqual(summary.poses.lio_pose.z.mean,40,'LIO coordinates remain in their own frame');
assert.strictEqual(summary.poses.ev_pose.z.count,0,'Missing values are not zero');
assert.strictEqual(summary.lio_age_sec.mean,.12);
ctx.obsConfirmMapping([1,2,3,4]);
assert.deepStrictEqual(plain(ctx.obsSummary(state.segments[0]).mapping),[4,2,1,3],'Segment mapping is captured at start');
const poseOld=tel(200);poseOld.observe.fc_pose.source_age=.31;
const old=ctx.obsNormalized(poseOld,200);
assert(old.fc_pose.stale && old.fc_pose.z===null && old.fc_pose.source_age===.31,'Pose age above .3s is marked stale and excluded');
const unknownTime=tel(201);unknownTime.observe.fc_pose.source_age=null;
assert(ctx.obsNormalized(unknownTime,201).fc_pose.unknown_age && ctx.obsNormalized(unknownTime,201).fc_pose.z===null,'Unknown source time never becomes fresh pose');
assert.strictEqual(ctx.obsNormalized(tel(200),203).fc_pose,null,'Telemetry older than 2s is not current');
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
assert(csv.includes('rc_out_raw_json')&&csv.includes('lio_pose_frame')&&csv.includes('selected_outputs_json'));
assert(csv.includes('"map"')&&csv.includes('"camera_init"'));
assert.strictEqual(ctx.obsCsvCell('OFF"BOARD'),'"OFF""BOARD"');
state.configuration.max_samples=20000;assert.strictEqual(ctx.obsLimit(),1800);
for(let i=0;i<26;i++){assert(ctx.obsStartSegment('square',300+i));assert(ctx.obsEndSegment(300+i+.5));}
assert.strictEqual(state.segments.length,24,'Segment metadata is also bounded');
assert.strictEqual(state.segmentsDropped,3);
ctx.obsSetConnection({host:'board-a',board_root:'/one'},true);
assert(ctx.obsConfirmMapping([4,2,1,3]));assert(ctx.obsStartSegment('hover',500));ctx.obsIngest(tel(500),500);
ctx.obsSetConnection({host:'board-b',board_root:'/two'},true);
assert.strictEqual(state.mapping,null,'A new host must verify the mapping again');
assert.strictEqual(state.active,null,'Host change finishes the old local segment');
assert.strictEqual(state.samples[state.samples.length-1].source.host,'board-a','Original sample source is kept');
assert(state.connectionNotice);
assert(!/\/api\/(?:action|trial|session|connect|config)/.test(source),'Observer has no action API');
assert(!/method:\s*['"]POST/.test(source));
assert(!/image_raw|jpeg|toDataURL|drawImage/i.test(source),'No camera/image/JPEG processing');
console.log('PASS observer: bounded recording, local mapping/segments, output deviations, missing/stale poses, separate frames, JSON/CSV and read-only network/image contract');
