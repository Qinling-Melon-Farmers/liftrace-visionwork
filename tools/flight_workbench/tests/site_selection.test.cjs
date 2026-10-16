const assert=require('node:assert/strict'),fs=require('node:fs'),vm=require('node:vm');
const data=JSON.parse(fs.readFileSync(0,'utf8')),U={},requests=[];
const sandbox={window:{addEventListener(){}},globalThis:null,applySnapshot(){},toast(){},groupUI(){return U;},trialBody(){return {};},geometryPlanKey(){return 'base';},groupCommandBody(){return {body:'original'};},geometryFromDraft(){return {};},geometryEditor(){},groupCard(){},renderGroups(){},shellArgQuote:s=>/^[a-zA-Z0-9_./-]+$/.test(s)?s:"'"+s.replaceAll("'","'\\''")+"'",state:{tabs:{[data.group.id]:U},connection:{host:'test',board_root:'/board',site_dir:'site'}},fetch:async(path,options)=>{requests.push(JSON.parse(options.body));return {ok:true,json:async()=>({ok:true,source:'OLD-SOURCE',draft:null})};},document:{},console};
sandbox.globalThis=sandbox;vm.createContext(sandbox);
vm.runInContext(fs.readFileSync('web/low-coverage.js','utf8'),sandbox);
for(const [shiftback,height,value] of [[false,1.8,'88.56%'],[false,1.9,'89.91%'],[true,1.8,'99.74%'],[true,1.9,'100%']]){assert.ok(sandbox.window.lowCoverageReference({coverageShiftback:shiftback,coverageHeight:height}).includes(value));}
assert.ok(sandbox.window.lowCoverageReference({geometryEnabled:true}).includes('需重算'));
assert.ok(!sandbox.window.lowCoverageReference({geometryEnabled:true}).includes('%'));
for(const test of data.cases){U.coveragePattern=test.options.coverage_pattern;assert.equal(sandbox.groupCommandBody(data.group,test.mode,test.real,test.check,null,test.options).body,test.expected);}
const {route}=require('../web/waypoint-coverage.js');
for(const ctx of data.contexts){const r=route(ctx.site,ctx);assert.equal(r.points.length,ctx.coverage_pattern==='lawnmower'?6:11);assert.equal(r.points[0][0],ctx.coverage_shiftback?0:1);assert.equal(Math.max(...r.points.map(p=>p[0])),ctx.coverage_shiftback?5.2:6.2);}
U.geometryEnabled=true;U.geometryPlans={};const options={coverage_pattern:'nested_rectangles',coverage_agl:1.8,site_geometry:{}};
const key=sandbox.geometryPlanKey(data.group,'preview',false,false,null,options);U.geometryPlans[key]={body:'MANUAL-OVERLAY'};
assert.equal(sandbox.groupCommandBody(data.group,'preview',false,false,null,options).body,'MANUAL-OVERLAY');
U.geometryValues={search_waypoints_xy:[[0,1],[2,3]],coverage_rectangles:[[0,5,-4,4],[1,3,-3,3]]};
assert.equal(sandbox.geometryFromDraft(U,data.group).search_waypoints_xy.length,2);
U.geometryEnabled=false;U.geometryValues=null;U.waypointDraftScope=null;
vm.runInContext(fs.readFileSync('web/waypoint-bridge.js','utf8'),sandbox);
U.coveragePattern='lawnmower';sandbox.window.WaypointWorkbench.restore(data.group,U);const first=U.waypointLoadedScope;
U.coveragePattern='nested_rectangles';sandbox.window.WaypointWorkbench.restore(data.group,U);
assert.notEqual(U.waypointLoadedScope,first);assert.deepEqual(requests.map(r=>r.coverage_pattern),['lawnmower','nested_rectangles']);
setImmediate(()=>{assert.ok(U.waypointLoadError.includes('后端地图来源不一致'));console.log('PASS: 32 frontend/backend commands, explicit map routes, manual override, source scopes and stale backend rejection');});

// 真实groupUI会恢复草稿：scope不得通过groupUI再次进入restore。
U.waypointLoadedScope=null;
sandbox.groupUI=function(g){sandbox.window.WaypointWorkbench.restore(g,U);return U;};
assert.doesNotThrow(()=>sandbox.groupUI(data.group));
