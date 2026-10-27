'use strict';
const fs = require('node:fs'), vm = require('node:vm'), assert = require('node:assert/strict'), path = require('node:path');
const source = fs.readFileSync(path.join(__dirname, '../web/app.js'), 'utf8');
const html = fs.readFileSync(path.join(__dirname, '../web/index.html'), 'utf8');
const cameraIds = ['head_rgb', 'left_wrist_rgb', 'right_wrist_rgb'];

class Element {
  constructor(dataset = {}) { this.dataset = dataset; this.disabled = false; this.hidden = false; this.value = ''; this.textContent = ''; this.listeners = {}; this.attributes = {}; this.children = []; this.classList = {add(){}, remove(){}, toggle(){}}; }
  addEventListener(event, listener) { (this.listeners[event] ||= []).push(listener); }
  async emit(event, data = {}) { for (const listener of this.listeners[event] || []) await listener({target:this, ...data}); }
  setAttribute(key, value) { this.attributes[key] = value; }
  replaceChildren(...children) { this.children = children; }
  append(...children) { this.children.push(...children); }
  querySelector(selector) { if (selector === '.service-state') return this.status; if (selector === 'span') return this.span ||= new Element(); throw new Error('Unexpected element selector: ' + selector); }
  querySelectorAll(selector) { if (selector === 'button') return this.children; throw new Error('Unexpected element selector: ' + selector); }
  closest(selector) { if (selector === '[data-device]') return this.parent; return null; }
  focus() {}
}
function fixture(phase = 'inactive') {
  return {
    system:{services:{arm_startup:true,control:true,episode:true,base:false,lift:false},free_gib:10,data_directory:'/data'},
    arm_startup:{fresh:true,configured:true,phase:'idle',reason:'waiting_for_web_button',owned:false,max_error_rad:0,
      manual_pose:{phase,reason:'',target:Array(16).fill(0),can_hold:phase === 'editing'}},
    arms:{fresh:true,enabled:false,phase:'disabled',valid:Array(16).fill(true),position:Array(16).fill(0)},
    quest:{fresh:true,rate_hz:90,left_tracking:0,right_tracking:0,grips:[0,0],sticks:[0,0]},
    control:{fresh:true,phase:'disabled',shared_enable:false,system_phase:'disabled'},
    recorder:{fresh:true,state:'idle',samples:0},cameras:Object.fromEntries(cameraIds.map(id => [id,{fresh:false,enabled:false}])),
    base_status:{fresh:false},lift_status:{fresh:false},base:{valid:false},lift:{},server_mono_ns:0,
  };
}
async function harness(initial = fixture()) {
  const elements = {}, commands = [], devices = [], documentListeners = {}, windowListeners = {}, requests = [], beacons = [];
  for (const match of html.matchAll(/\bid="([^"]+)"/g)) elements[match[1]] = new Element();
  elements.task.value = 'teleoperation'; elements.reason.value = 'Unrelated recording note'; elements.preview.checked = false;
  for (const match of html.matchAll(/<button\b([^>]*)>([^<]*)<\/button>/g)) {
    const attrs = match[1], kind = attrs.match(/data-kind="([^"]+)"/)?.[1], command = attrs.match(/data-command="([^"]+)"/)?.[1];
    if (!command) continue;
    const button = new Element({kind,command}); button.textContent = match[2]; button.disabled = attrs.includes('disabled'); commands.push(button);
  }
  for (const match of html.matchAll(/data-device="([^"]+)"/g)) {
    const box = new Element({device:match[1]}); box.status = new Element();
    box.children = ['toggle','restart'].map(serviceAction => { const button = new Element({serviceAction}); button.parent = box; return button; });
    devices.push(box);
  }
  const queryAll = selector => {
    if (selector === '[data-command]') return commands;
    if (selector === '[data-device]') return devices;
    if (selector === '[data-device] button') return devices.flatMap(box => box.children);
    if (selector === '[data-independent-enable]' || selector === '[data-shared-control]') return [];
    throw new Error('Unexpected document selector: ' + selector);
  };
  const document = {hidden:false,hasFocus:()=>true,body:new Element(),getElementById:id => elements[id],querySelectorAll:queryAll,
    createElement:()=>new Element(),addEventListener:(event,listener)=>(documentListeners[event] ||= []).push(listener),
    querySelector:selector => { const kind = selector.match(/data-kind="([^"]+)"/)?.[1], command = selector.match(/data-command="([^"]+)"/)?.[1]; return commands.find(button => button.dataset.kind === kind && button.dataset.command === command); }};
  const model = {current:initial,post:null};
  const context = vm.createContext({document,window:{addEventListener:(event,listener)=>(windowListeners[event] ||= []).push(listener)},
    navigator:{sendBeacon:(url,body)=>{beacons.push({url,body});return true;}},Blob,AbortSignal,URL,Date,Math,console,
    setTimeout(){},setInterval(){},fetch:async (url,options = {}) => {
      requests.push({url,options});
      if (options.method) { if (model.post) await model.post(url,JSON.parse(options.body)); return {ok:true,text:async()=>JSON.stringify({message:'accepted'})}; }
      const body = url === '/api/state' ? model.current : url === '/api/devices' ? {devices:Object.fromEntries(devices.map(box=>[box.dataset.device,{LoadState:'loaded',ActiveState:'active'}]))} : [];
      return {ok:true,text:async()=>JSON.stringify(body)};
    }});
  vm.runInContext(source, context, {filename:'app.js'});
  const flush = async () => { for (let i=0;i<4;i++) await new Promise(resolve => setImmediate(resolve)); };
  await flush();
  const command = name => commands.find(button => button.dataset.command === name && button.dataset.kind === 'arm_startup');
  const refresh = s => { model.current = s; context.testState = s; vm.runInContext('render(testState); deviceButtons();',context); };
  const postRequests = () => requests.filter(request => request.options.method).map(request => ({url:request.url,body:JSON.parse(request.options.body)}));
  const emitWindow = async event => { for (const listener of windowListeners[event] || []) await listener(); await flush(); };
  const emitDocument = async event => { for (const listener of documentListeners[event] || []) await listener(); await flush(); };
  return {elements,commands,devices,context,model,command,refresh,postRequests,beacons,emitWindow,emitDocument,document,flush};
}

async function run() {
  const h = await harness();
  assert.equal(h.postRequests().length,0,'Loading the page must not enter, enable, hold or cancel automatically');
  assert.equal(h.command('pose_enter').disabled,false);
  assert.equal(h.command('pose_hold').disabled,true);
  assert.equal(h.command('pose_cancel').disabled,true);

  const old = fixture(); delete old.arm_startup.manual_pose; h.refresh(old);
  for (const command of ['pose_enter','pose_hold','pose_cancel']) assert.equal(h.command(command).disabled,true);
  assert.match(h.elements['manual-pose-badge'].textContent,/待更新/);

  const editing = fixture('editing'); h.refresh(editing);
  assert.equal(h.command('pose_enter').disabled,true);
  assert.equal(h.command('pose_hold').disabled,false);
  assert.equal(h.command('pose_cancel').disabled,false);
  for (const kind of ['arm_startup','control','episode']) assert.equal(h.commands.find(button => button.dataset.kind === kind && button.dataset.command === 'start').disabled,true);
  assert.ok(h.devices.find(box => box.dataset.device === 'arms').children.every(button => button.disabled));
  editing.arm_startup.manual_pose.can_hold = false; h.refresh(editing); assert.equal(h.command('pose_hold').disabled,true);
  editing.arm_startup.manual_pose.can_hold = true; editing.arms.valid[3] = false; h.refresh(editing); assert.equal(h.command('pose_hold').disabled,true);
  editing.arms.valid[3] = true; editing.arms.enabled = true; h.refresh(editing); assert.equal(h.command('pose_hold').disabled,true);

  const preparing = fixture('preparing'); h.refresh(preparing);
  vm.runInContext('busy = true;',h.context); h.refresh(preparing);
  assert.equal(h.command('pose_cancel').disabled,false,'Preparing remains cancellable during an outstanding request');
  assert.equal(h.command('pose_hold').disabled,true);
  assert.match(h.elements['manual-pose-status'].textContent,/300 ms/);
  vm.runInContext('busy = false;',h.context);

  const holding = fixture('holding'); holding.arms.enabled = true; holding.arm_startup.owned = false; h.refresh(holding);
  assert.equal(h.command('pose_enter').disabled,false);
  assert.equal(h.command('pose_cancel').disabled,true,'Cancel must not remove torque after hold succeeds');
  for (const kind of ['arm_startup','control','episode']) assert.equal(h.commands.find(button => button.dataset.kind === kind && button.dataset.command === 'start').disabled,false);
  for (const command of ['start','stop','disable']) assert.equal(h.command(command).disabled,false,'Startup controls must take over independent manual hold without startup ownership');
  assert.match(h.elements['arm-phase'].textContent,/姿态保持/);
  assert.match(h.elements['manual-pose-status'].textContent,/无需另行解锁/);
  const faultHold = structuredClone(holding);
  faultHold.arm_startup.manual_pose.fault_latched = true;
  h.refresh(faultHold);
  assert.match(h.elements['manual-pose-badge'].textContent,/已暂停/);
  assert.doesNotMatch(h.elements['manual-pose-badge'].className,/good/);
  assert.match(h.elements['manual-pose-status'].textContent,/不会自动重发/);
  assert.equal(h.command('pose_cancel').disabled,true);
  const shared = structuredClone(holding);
  Object.assign(shared.control,{shared_enable:true,system_phase:'stopped',parking_policy:'legacy_timed_return_lift_stopped'});
  Object.assign(shared.base_status,{fresh:true}); shared.base = {valid:true,voltage:24,velocity:[0,0,0]};
  Object.assign(shared.lift_status,{fresh:true,feedback_healthy:true,stationary:true,allow_motion:true});
  h.refresh(shared);
  assert.equal(h.commands.find(button => button.dataset.kind === 'control' && button.dataset.command === 'start').disabled,false,'Existing legacy parking policy must allow shared teleoperation takeover');
  const staleHold = structuredClone(holding); staleHold.arms.fresh = false; h.refresh(staleHold);
  assert.match(h.elements['manual-pose-badge'].textContent,/待确认/); assert.doesNotMatch(h.elements['manual-pose-badge'].className,/good/);
  h.refresh(holding);
  await h.emitWindow('blur'); await h.emitWindow('pagehide'); h.document.hidden = true; await h.emitDocument('visibilitychange');
  vm.runInContext('offline();',h.context); await h.flush();
  assert.equal(h.postRequests().length,0,'Blur, page hide, backgrounding and disconnection must not remove holding torque');
  assert.equal(h.beacons.length,0);
  assert.match(h.elements['manual-pose-badge'].textContent,/待确认/);
  for (const command of ['pose_enter','pose_hold','pose_cancel']) assert.equal(h.command(command).disabled,true);

  h.refresh(fixture()); h.document.hidden = false;
  h.model.post = async (url,body) => { if (body.command === 'pose_enter') h.model.current = fixture('editing'); };
  await h.command('pose_enter').emit('click');
  assert.deepEqual(h.postRequests().at(-1),{url:'/api/arm_startup',body:{command:'pose_enter',detail:''}});
  h.model.post = async (url,body) => { if (body.command === 'pose_hold') h.model.current = holding; };
  await h.command('pose_hold').emit('click');
  assert.deepEqual(h.postRequests().at(-1),{url:'/api/arm_startup',body:{command:'pose_hold',detail:''}});
  const count = h.postRequests().length; await h.command('pose_cancel').emit('click'); assert.equal(h.postRequests().length,count);

  h.refresh(fixture('preparing')); vm.runInContext('busy = true; pendingCommands = 1;',h.context);
  h.model.post = async () => { h.model.current = fixture(); };
  await h.command('pose_cancel').emit('click');
  assert.deepEqual(h.postRequests().at(-1),{url:'/api/arm_startup',body:{command:'pose_cancel',detail:''}});
  assert.equal(vm.runInContext('busy',h.context),true,'Finishing cancel must not clear another outstanding request');
  assert.match(html,/先支撑双臂及夹爪/); assert.match(html,/移除双臂和夹爪的电机转矩/);
  console.log('PASS: manual pose page initialization, API interactions, handover, feedback gates, preparation cancellation and no automatic torque removal');
}
run().catch(error => { console.error(error); process.exitCode = 1; });
