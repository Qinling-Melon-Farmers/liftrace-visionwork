const fs=require('fs'), vm=require('vm'), assert=require('assert');
const ctx={document:{readyState:'loading',addEventListener(){}},window:{},console};
vm.createContext(ctx);vm.runInContext(fs.readFileSync(require('path').join(__dirname,'../web/app.js'),'utf8'),ctx);
const group={id:'competition',site_config:'field.yaml'};
const options={resume_survey:'on',motion_optimization:'off'};
const trial={group_id:'competition',competition_config:'field.yaml',resume_survey:'on',motion_optimization:'off'};
let stage={effective_config:{source:'validated_settings',motion_optimization:false,resume_survey:true,generation_ready:false}};
assert(ctx.competitionEffectiveConfig(group,options,{},trial).includes('尚未确认'));
assert(ctx.competitionEffectiveConfig(group,options,stage,trial).includes('离线配置检查'));
assert(ctx.competitionEffectiveConfig(group,options,stage,trial).includes('尚不可生成'));
stage={effective_config:{source:'generated_runtime',motion_optimization:false,resume_survey:true,runtime_path:'/run/runtime.yaml'}};
assert(ctx.competitionEffectiveConfig(group,options,stage,trial).includes('运行已生成配置'));
assert(ctx.competitionEffectiveConfig(group,options,stage,trial).includes('/run/runtime.yaml'));
for(const changed of [{resume_survey:'off'},{motion_optimization:'on'},{obstacle_columns:'off'},{competition_config:'different.yaml'}]) {
 assert(ctx.competitionEffectiveConfig(group,Object.assign({},options,changed),stage,trial).includes('尚未确认'));
}
assert(ctx.competitionEffectiveConfig(group,options,stage,Object.assign({},trial,{group_id:'mod08'})).includes('尚未确认'));
console.log('PASS config evidence, separate check/runtime sources, changed selections invalidate display');
