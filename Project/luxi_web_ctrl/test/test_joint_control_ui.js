'use strict';
const fs=require('node:fs'),path=require('node:path'),vm=require('node:vm'),assert=require('node:assert/strict');
const {harness,fixture}=require('./test_manual_pose_ui');
const cfg=JSON.parse(fs.readFileSync(path.join(__dirname,'../../robot_bringup/config/arms-kh.json'),'utf8'));
const bounds=[...cfg.right.limits,...cfg.left.limits],lower=bounds.map(x=>x[0]),upper=bounds.map(x=>x[1]);
const initial=lower.map((v,i)=>v+.4*(upper[i]-v));
function model(phase='idle') {
  const s=fixture(phase==='idle'?'inactive':phase==='ready'||phase==='stopped'?'holding':'moving');
  s.arms.position=initial.slice();s.arms.enabled=phase!=='idle';
  s.arm_startup.joint_control={unit:'rad',joint_order:'right_then_left',lower,upper,phase,target:initial.slice(),speed_rad_s:.15};
  return s;
}
async function run(){
  const h=await harness(model());h.elements['joint-speed'].value='.15';
  vm.runInContext(fs.readFileSync(path.join(__dirname,'../web/joint-control.js'),'utf8'),h.context,{filename:'joint-control.js'});
  assert.equal(h.postRequests().length,0,'Page load must not enable or move');
  assert.equal(h.elements['joint-execute'].disabled,true,'Blank targets must never mean zero');
  await h.elements['joint-read-current'].emit('click');
  assert.equal(h.elements['joint-execute'].disabled,false);
  const read=vm.runInContext('jointTargets()',h.context);
  read.forEach((v,i)=>assert.ok(Math.abs(v-initial[i])<1e-12));
  assert.equal(h.postRequests().length,0,'Reading feedback only edits the form');
  const degrees=180/Math.PI;
  for(let i=0;i<16;i++){
    assert.equal(Number(h.elements['joint-angle-'+i].value),initial[i],'Radian input preserves exact feedback');
    assert.equal(Number(h.elements['joint-angle-deg-'+i].value),initial[i]*degrees);
    assert.equal(h.elements['joint-current-'+i].textContent,initial[i].toFixed(3)+' rad\n'+(initial[i]*degrees).toFixed(2)+' °');
  }
  assert.ok(h.elements['arm-joints'].innerHTML.includes('<td>'+initial[8].toFixed(3)+'</td><td>'+(initial[8]*degrees).toFixed(2)+'</td>'),'Feedback table displays radians first, then degrees');
  for(let i=0;i<16;i++){
    const printed=h.elements['joint-limit-'+i].textContent.match(/-?\d+\.\d+/g).map(Number);
    assert.ok(printed[0]>=lower[i] && printed[1]<=upper[i] && printed[2]/degrees>=lower[i] && printed[3]/degrees<=upper[i], 'Printed bounds in either unit must never invite an out-of-range input');
  }
  for(let i=0;i<16;i++){
    const input=h.elements['joint-angle-'+i],saved=input.value;
    input.value=String(upper[i]+.0001);await input.emit('input');
    assert.equal(h.elements['joint-execute'].disabled,true,'Each upper limit must reject');
    input.value=String(lower[i]-.0001);await input.emit('input');
    assert.equal(h.elements['joint-execute'].disabled,true,'Each lower limit must reject');
    input.value=saved;await input.emit('input');
    const degreeInput=h.elements['joint-angle-deg-'+i];
    for(const value of [(upper[i]+.0001)*degrees,(lower[i]-.0001)*degrees]){
      degreeInput.value=String(value);await degreeInput.emit('input');
      assert.equal(h.elements['joint-execute'].disabled,true,'Both degree limits must reject before converting');
    }
    degreeInput.value=String(initial[i]*degrees);await degreeInput.emit('input');
    assert.ok(Math.abs(Number(input.value)-initial[i])<1e-12,'Editing degrees updates the primary radians input');
    assert.equal(h.elements['joint-execute'].disabled,false);
  }
  for(const id of ['joint-angle-0','joint-angle-deg-0']){
    const input=h.elements[id];
    for(const value of ['','NaN','Infinity','bad']){
      input.value=value;await input.emit('input');assert.equal(h.elements['joint-execute'].disabled,true);
    }
    await h.elements['joint-read-current'].emit('click');
  }
  const buffer=new ArrayBuffer(8),view=new DataView(buffer);
  function nextUp(value){view.setFloat64(0,value);view.setBigUint64(0,view.getBigUint64(0)+(value>=0?1n:-1n));return view.getFloat64(0);}
  for(let i=0;i<16;i++){
    const degreeInput=h.elements['joint-angle-deg-'+i];
    degreeInput.value=String(nextUp(upper[i]*degrees));await degreeInput.emit('input');
    assert.equal(h.elements['joint-execute'].disabled,true,'A one-ULP degree violation must not round into an allowed radian value');
    h.elements['joint-angle-'+i].value=String(initial[i]);await h.elements['joint-angle-'+i].emit('input');
  }
  h.elements['joint-angle-0'].value='.1234567890123456';await h.elements['joint-angle-0'].emit('input');
  h.elements['joint-angle-deg-0'].value='999';
  assert.equal(vm.runInContext('jointTargets()[0]',h.context),.1234567890123456,'The radian field remains authoritative at submission; degree edits synchronize through their input event');
  await h.elements['joint-read-current'].emit('click');
  // Mixed unit edits remain one radian target per joint, including exact endpoints.
  for(let i=0;i<16;i++){
    const input=h.elements[(i%2?'joint-angle-deg-':'joint-angle-')+i];
    input.value=input.max;await input.emit('input');
  }
  vm.runInContext('jointControlUI()',h.context);
  assert.equal(h.elements['joint-execute'].disabled,false,'Exact original endpoints are accepted');
  assert.equal(h.elements['joint-execute'].disabled,false,'Endpoint conversion must not round beyond the limit');
  const endpoints=vm.runInContext('jointTargets()',h.context);endpoints.forEach((v,i)=>assert.equal(v,upper[i]));
  h.elements['joint-speed'].value='.201';await h.elements['joint-speed'].emit('input');assert.equal(h.elements['joint-execute'].disabled,true);
  h.elements['joint-speed'].value='.15';await h.elements['joint-speed'].emit('input');
  h.model.post=async(url,body)=>{assert.equal(body.command,'joint_move');h.model.current=model('preparing');};
  await h.elements['joint-execute'].emit('click');
  const posted=h.postRequests().at(-1);assert.equal(posted.url,'/api/arm_startup');
  const body=JSON.parse(posted.body.detail);assert.deepEqual(body.positions,upper);assert.equal(body.unit,'rad');assert.equal(body.speed_rad_s,.15);assert.ok(body.request_id);
  assert.equal(h.elements['joint-execute'].disabled,true);
  for(const command of ['pose_enter','pose_hold','pose_cancel'])assert.equal(h.command(command).disabled,true);
  for(const kind of ['arm_startup','control','episode'])assert.equal(h.commands.find(b=>b.dataset.kind===kind&&b.dataset.command==='start').disabled,true);
  assert.match(h.elements['arm-phase'].textContent,/关节角度/);
  vm.runInContext('busy=true;pendingCommands=1;',h.context);h.refresh(model('moving'));
  assert.equal(h.elements['joint-stop'].disabled,false,'Explicit stop remains available while another request is pending');
  h.model.post=async()=>{h.model.current=model('stopped');};await h.elements['joint-stop'].emit('click');
  assert.deepEqual(h.postRequests().at(-1),{url:'/api/arm_startup',body:{command:'joint_stop',detail:''}});
  assert.equal(vm.runInContext('busy',h.context),true,'Stop must preserve another pending request');
  vm.runInContext('busy=false;pendingCommands=0;',h.context);h.refresh(model('ready'));
  const count=h.postRequests().length;await h.emitWindow('blur');await h.emitWindow('pagehide');h.document.hidden=true;await h.emitDocument('visibilitychange');
  assert.equal(h.postRequests().length,count,'Finite accepted action and hold are not resubmitted or unloaded on page events');
  const old=model();delete old.arm_startup.joint_control;h.refresh(old);assert.equal(h.elements['joint-execute'].disabled,true);
  h.refresh(model());h.model.current.arms.valid[5]=false;h.refresh(h.model.current);assert.equal(h.elements['joint-execute'].disabled,true);
  const outside=model();outside.arms.position[5]=upper[5]+.001;h.refresh(outside);
  assert.equal(h.elements['joint-execute'].disabled,true);assert.match(h.elements['joint-control-status'].textContent,/当前角度超出原限位/);
  console.log('PASS: simultaneous radian/degree feedback and limits; 64 input limit checks; either-unit edits and mixed exact endpoints; empty/nonfinite/speed gates; radian API payload; moving interlocks and busy stop; no automatic commands');
}
run().catch(e=>{console.error(e);process.exitCode=1;});
