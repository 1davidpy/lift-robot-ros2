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
                age = 20, health = 'ok', fault = 0, paused = false} = {}) {
  const out = {};
  vm.runInNewContext(block, {
    s: {server_mono_ns: 1000000000,
      lift_status: {fresh: true, transport, feedback_healthy: healthy, health,
                    position_invalid_isolation: paused, position_invalid_paused: paused},
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
assert.match(label({flags: 0x30, healthy: false, paused: true}), /仅暂停升降/);
assert.match(label({fault: 1, healthy: false, paused: true}), /保护\/故障/);
const readySource = source.match(/function liftAllowsTeleop\(s\) \{[\s\S]*?\r?\n\}/)?.[0];
assert(readySource, 'actual dashboard start predicate not found');
function allows(extraLift = {}, extraControl = {}) {
  const s = {control: {fresh: true, lift_position_paused: true, ...extraControl},
    lift_status: {fresh: true, allow_motion: true, feedback_healthy: false,
      stationary: false, position_invalid_isolation: true, position_invalid_paused: true,
      position_invalid_stationary: true, flags: 0, fault: 0, ...extraLift}};
  return vm.runInNewContext(readySource+';liftAllowsTeleop(s)', {s});
}
assert.equal(allows(), true);
assert.equal(allows({}, {lift_position_paused: false}), false);
assert.equal(allows({}, {fresh: false}), false);
assert.equal(allows({position_invalid_stationary: false}), false);
assert.equal(allows({fault: 1}), false);
assert.equal(allows({flags: 128}), false);
assert.equal(allows({fresh: false}), false);
assert.equal(allows({feedback_healthy: true, stationary: true,
                     position_invalid_paused: false}, {lift_position_paused: false}), true);
console.log('9 lift badge cases and 8 teleop start cases passed');
