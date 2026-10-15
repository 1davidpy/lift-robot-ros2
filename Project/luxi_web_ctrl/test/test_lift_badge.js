// Execute the actual dashboard lift-status block against recorded/protocol cases.
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const assert = require('node:assert/strict');
const source = fs.readFileSync(path.join(__dirname, '../web/app.js'), 'utf8');
const start = source.indexOf('  const ls = s.lift_status, lift = s.lift;');
const end = source.indexOf('  const count = Object.values(s.cameras)', start);
assert(start >= 0 && end > start);
const block = source.slice(start, end);
function label({transport = 'uart', flags = 0x33, guard = 0, healthy = true,
                age = 20, health = 'ok', fault = 0} = {}) {
  const out = {};
  vm.runInNewContext(block, {
    s: {server_mono_ns: 1000000000,
      lift_status: {fresh: true, transport, feedback_healthy: healthy, health},
      lift: {receive_first_mono_ns: 1000000000 - age * 1e6, flags, guard, fault,
             current_um: 0, feedback_age_ms: 100, speed_rpm: 0, valid: true}},
    badge: (id, value) => {out[id] = value;}, text: () => {},
  });
  return out['lift-badge'];
}
assert.match(label({flags: 0x30, healthy: false}), /等待电机\/位置参考/);
assert.equal(label({guard: 4}), '已连接 · UART');
assert.match(label({transport: 'can', guard: 4, healthy: false}), /保护\/故障/);
assert.match(label({healthy: false, health: 'motor_feedback_stale'}), /电机反馈过期/);
assert.equal(label({age: 301}), '无 UART 反馈');
assert.match(label({health: 'locked_startup_preparation'}), /锁闸启动准备/);
assert.match(label({fault: 1, healthy: false}), /保护\/故障/);
console.log('7 lift dashboard classification cases passed');
