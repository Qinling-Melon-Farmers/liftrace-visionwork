'use strict';
const assert=require('node:assert/strict');
const {route}=require('../web/waypoint-coverage.js');
const site={flight_area:{search_bounds:[.4,6.8,-4.1,4.1],survey_xy:[[99,99]]}};
const lawn=[[1,-4.1],[1,4.1],[3.6,4.1],[3.6,-4.1],[6.2,-4.1],[6.2,4.1]];
assert.deepEqual(route(site).points,lawn);
const nested=[[1,-4.1],[1,4.1],[6.2,4.1],[6.2,-4.1],[1,-4.1],[2.8,-4.1],[2.8,-3.2],[2.8,3.2],[4.4,3.2],[4.4,-3.2],[2.8,-3.2]];
assert.deepEqual(route(site,{coverage_pattern:'nested_rectangles',coverage_agl:1.9}).points,nested);
assert.equal(route(site,{coverage_agl:1.9}).agl,1.9);
assert.deepEqual(route({...site,search_waypoints_xy:[[2,0],[3,1]]},{coverage_pattern:'nested_rectangles'}).points,[[2,0],[3,1]]);
assert.deepEqual(route({...site,coverage_scan_axis:'x',coverage_cross_inset:1,coverage_lane_spacing:3}).points,[[.4,-3.1],[6.8,-3.1],[6.8,-.10000000000000009],[.4,-.10000000000000009],[.4,2.9],[6.8,2.9],[6.8,3.1],[.4,3.1]]);
assert.deepEqual(route({...site,coverage_rectangles:[[1,6,-4,4],[2,5,-3,3]]},{coverage_pattern:'nested_rectangles'}).points.at(-1),[2,-3]);
assert.equal(route({...site,_low_coverage_original:{coverage_lane_spacing:2}}).points.length,8);
assert.throws(()=>route({...site,search_waypoints_xy:[]}));
assert.throws(()=>route({...site,search_waypoints_xy:[[2,0],[9,0]]}));
assert.throws(()=>route({...site,coverage_cross_inset:4}));
assert.throws(()=>route({...site,coverage_loop_inset:4},{coverage_pattern:'nested_rectangles'}));
assert.deepEqual(site.flight_area.survey_xy,[[99,99]]);
if(process.argv[2]){
 const cases=JSON.parse(require('node:fs').readFileSync(process.argv[2],'utf8'));
 for(const test of cases){const actual=route(test.site);assert.deepEqual(actual.points,test.expected);assert.equal(actual.source,test.source);}
 console.log('PASS: '+cases.length+' exact Python coverage_waypoints reference cases');
}
console.log('PASS: coverage defaults, explicit precedence, both axes, custom spacing/insets/rectangles and rejection; no site mutation');
