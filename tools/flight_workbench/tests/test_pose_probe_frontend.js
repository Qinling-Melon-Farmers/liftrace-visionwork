// Execute only observe.js pure formatting/normalization. No HTTP or device calls.
const fs = require('fs'), path = require('path'), vm = require('vm'), assert = require('assert');
const source = fs.readFileSync(path.join(__dirname, '../web/observe.js'), 'utf8');
const ctx = {console, document: {readyState: 'loading', addEventListener() {}}, window: {}};
vm.createContext(ctx);
vm.runInContext(source, ctx);
const now = 1791360765;
const reason = 'TypeError: pose decoder regression: expected geometry_msgs/PoseStamped';
const tel = {
  at: now, master: true, observe: {fc_pose: null, battery: {voltage: 23.1}},
  observe_status: {fc_pose: {topic: '/mavros/local_position/pose', status: 'parse_error',
    publishers: ['/mavros'], count: 900, hz: 30, age: .01, error: reason}}
};
const detail = ctx.obsTopicStatus('fc_pose', tel, now);
assert(detail.includes('字段解析失败'), 'Pose parse error must be visible instead of receiving/healthy');
assert(detail.includes('30Hz'), 'A received packet rate must not hide decoder failure');
assert.strictEqual(ctx.obsNormalized(tel, now).fc_pose, null, 'Invalid pose is not synthesized');
assert.strictEqual(ctx.obsNormalized(tel, now).battery.voltage, 23.1, 'Pose error must not hide healthy battery data');
assert(!ctx.obsTopicStatus('fc_pose', tel, now+10).includes(reason), 'Old error is not presented as current telemetry');
const recovered = {...tel, observe_status: {fc_pose: {...tel.observe_status.fc_pose, status: 'receiving', error: null}}};
assert(!ctx.obsTopicStatus('fc_pose', recovered, now).includes(reason), 'Recovered messages clear the displayed error');
console.log('PASS: pose parse-error label, packet rate, missing pose, independent battery, freshness and recovery; detailed exception is tested in probe stderr');
