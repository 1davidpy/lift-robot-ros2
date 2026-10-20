const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const assert = require('node:assert/strict');
const source = fs.readFileSync(path.join(__dirname, '../web/app.js'), 'utf8');
const start = source.indexOf('  const armValid =');
const end = source.indexOf('  const jointRows =', start);
assert(start >= 0 && end > start);
function armLabel(a) {
  const out = {};
  vm.runInNewContext(source.slice(start, end), {
    a, s:{system:{mock:false}}, phases:{},
    badge:(id,label)=>out[id]=label, text:(id,value)=>out[id]=value,
  });
  return out;
}
assert.equal(armLabel(null)['arm-phase'], '采集节点离线');
assert.equal(armLabel({fresh:false,valid:Array(16).fill(true)})['arms-badge'], '未就绪');
const partial = armLabel({fresh:true,phase:'monitor_only',valid:[...Array(8).fill(true),...Array(8).fill(false)]});
assert.match(partial['arms-badge'], /部分在线 · 8\/16/);
assert.equal(partial['arms-detail'], '左臂 0/8 · 右臂 8/8 有效反馈');
assert.equal(armLabel({fresh:true,phase:'monitor_only',valid:Array(16).fill(true)})['arms-badge'], '反馈在线');
console.log('4 arm dashboard connection cases passed');
