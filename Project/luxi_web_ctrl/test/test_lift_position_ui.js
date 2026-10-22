const fs=require('node:fs'),vm=require('node:vm'),assert=require('node:assert/strict'),path=require('node:path');
const source=fs.readFileSync(path.join(__dirname,'../web/app.js'),'utf8');
const part=source.slice(source.indexOf('function liftPositionUI()'),source.indexOf('function liftPositionPayload('));
function run(overrides={},owner='',age=1000){
  const elements={},out={};
  const ls={fresh:true,dashboard_position_available:true,allow_motion:true,feedback_healthy:true,stationary:true,enabled:false,phase:'disabled',...overrides};
  const context={state:{system:{services:{lift:true}},lift_status:ls},connected:true,
    liftPositionOwner:owner,liftPositionBusy:false,liftPositionArmedAt:Date.now()-age,liftPositionError:'',liftPositionMoving:false,liftPositionTask:1,
    $:id=>elements[id]??=( {} ),text:(id,v)=>out[id]=v,badge:(id,v)=>out[id]=v,Date};
  vm.runInNewContext(part+'\nliftPositionUI();',context);return {elements,out,context};
}
assert.equal(run().elements['lift-position-enable'].disabled,false);
assert.equal(run({allow_motion:false}).elements['lift-position-enable'].disabled,true);
assert.equal(run({feedback_healthy:false}).elements['lift-position-enable'].disabled,true);
assert.equal(run({},'opening',100).context.liftPositionOwner,'opening');
assert.equal(run({},'expired',1000).context.liftPositionOwner,'');
const ready={dashboard_position:true,external_owner:'mine',enabled:true,phase:'idle',ready_for_command:true,session_present:false,external_result:'idle'};
assert.equal(run(ready,'mine').elements['lift-position-move'].disabled,false);
assert.equal(run({...ready,phase:'moving',external_result:'accepted'},'mine').elements['lift-position-move'].disabled,true);
assert.equal(run({...ready,external_owner:'other'},'mine').elements['lift-position-move'].disabled,true);
const html=fs.readFileSync(path.join(__dirname,'../web/index.html'),'utf8');
assert.match(html,/id="lift-position-range"[^>]*min="-180"[^>]*max="180"/);
const targetSource=source.slice(source.indexOf('function liftPositionTarget(value)'),source.indexOf("$('lift-position-range').addEventListener"));
const values={},targetContext={liftPositionError:'',$:id=>values[id]??=({}),text:()=>{}};
vm.createContext(targetContext);vm.runInContext(targetSource,targetContext);
assert.equal(targetContext.liftPositionTarget('167.229'),true);
assert.equal(values['lift-position-number'].value,'167.229');
assert.equal(targetContext.liftPositionTarget('-180'),true);
assert.equal(targetContext.liftPositionTarget('180'),true);
assert.equal(targetContext.liftPositionTarget('180.001'),false);
assert.equal(targetContext.liftPositionTarget(''),false);
const listeners={},labels={};
const inputContext={liftPositionError:'',$:id=>values[id]??=({}),text:(id,v)=>labels[id]=v};
inputContext.$=id=>{const el=values[id]??=({});el.addEventListener=(event,fn)=>listeners[id+':'+event]=fn;return el;};
vm.createContext(inputContext);
vm.runInContext(source.slice(source.indexOf('function liftPositionTarget(value)'),source.indexOf("$('lift-position-enable').addEventListener")),inputContext);
listeners['lift-position-number:input']({target:{value:'-38.506'}});
assert.equal(values['lift-position-range'].value,'-38.506');
assert.equal(labels['lift-position-move'],'移动到目标 -38.506 mm');
console.log('PASS: 8 lift position UI states, signed endpoints and micrometre target precision');
