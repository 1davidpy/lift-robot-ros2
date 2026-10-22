const fs=require('node:fs'),vm=require('node:vm'),assert=require('node:assert/strict'),path=require('node:path');
const source=fs.readFileSync(path.join(__dirname,'../web/app.js'),'utf8');
const heartbeat=source.slice(source.indexOf('async function liftPositionHeartbeat(){'),source.indexOf('setInterval(liftPositionHeartbeat,'));
function fixture(){
  let now=1000,mode='ok',calls=0,stops=0;
  const c={liftPositionOwner:'owner',liftPositionPending:false,liftPositionHeartbeatAckAt:now,liftPositionError:'',
    state:{lift_status:{external_lease_ms:1000}},Date:{now:()=>now},
    liftPositionRequest:async op=>{assert.equal(op,'heartbeat');calls++;if(mode==='network')throw new Error('timeout');if(mode==='rejected'){let e=new Error('expired');e.status=409;throw e;}},
    liftPositionUI:()=>{},liftPositionStop:()=>{stops++;c.liftPositionOwner='';}};
  vm.createContext(c);vm.runInContext(heartbeat,c);
  return {c,tick:()=>c.liftPositionHeartbeat(),time:v=>now=v,mode:v=>mode=v,calls:()=>calls,stops:()=>stops};
}
(async()=>{
  let f=fixture();f.time(1350);f.mode('network');await f.tick();
  assert.equal(f.stops(),0);assert.equal(f.c.liftPositionOwner,'owner');assert.match(f.c.liftPositionError,/续租响应延迟/);
  f.time(1650);f.mode('ok');await f.tick();assert.equal(f.c.liftPositionHeartbeatAckAt,1650);assert.equal(f.c.liftPositionError,'');
  f.time(2700);f.mode('network');await f.tick();assert.equal(f.stops(),1);
  await f.tick();assert.equal(f.calls(),3,'no automatic re-arm or replay after stop');
  f=fixture();f.mode('rejected');await f.tick();assert.equal(f.stops(),1,'semantic rejection stops immediately');
  f=fixture();f.c.liftPositionPending=true;await f.tick();assert.equal(f.calls(),0);
  console.log('PASS: transient timeout recovery, lease expiry, permanent rejection, pending-request guard, no automatic replay');
})().catch(e=>{console.error(e);process.exitCode=1;});
