'use strict';
const $ = id => document.getElementById(id);
const cameraIds = ['head_rgb', 'left_wrist_rgb', 'right_wrist_rgb'];
const phases = {disabled:'未使能',homing:'回起始姿态',aligning:'等待手柄对齐',teleop:'遥操作中',hold:'保持',returning:'返回中',idle:'空闲',recording:'录制中'};
const systemPhases = {disabled:'已失能',preparing:'检查底盘与升降',starting:'双臂归位 / 对齐',arming:'等待统一使能确认',active:'全部遥操作已启用',stopping_base:'底盘 / 升降停车，双臂保持',homing_lift:'旧版流程：升降返回零点',returning_arms:'双臂返回 home，升降原位停止',disabling:'等待双臂失能反馈',stopped:'已停止 · 双臂保持'};
let liftPositionArmedAt = 0, liftPositionTask = 0, liftPositionMoving = false;
let liftPositionOwner = '', liftPositionSequence = 0, liftPositionPending = false, liftPositionBusy = false, liftPositionError = '';
let liftPositionHeartbeatAckAt = 0;
let keyboardActive = false, keyboardPending = false, keyboardStarting = false;
let keyboardError = "";
const heldKeys = new Set();
let state = null, connected = false, busy = false, lastUpdate = 0;
const frames = {};
function text(id, value) { $(id).textContent = value; }
function badge(id, label, tone = '') { text(id, label); $(id).className = 'pill ' + tone; }
function log(message) { text('result', new Date().toLocaleTimeString() + '  ' + message + '\n' + $('result').textContent.slice(0, 2500)); }
async function request(path, options = {}) {
  const {timeoutMs,...fetchOptions} = options;
  const r = await fetch(path, {...fetchOptions, signal:AbortSignal.timeout(timeoutMs ?? (options.method ? 12000 : 2500)), cache:'no-store'});
  const body = await r.text();
  let data; try { data = JSON.parse(body); } catch { data = null; }
  if (!r.ok) { const error = new Error(data?.message || ({409:'设备当前状态不允许此操作',503:'对应节点未就绪',504:'指令响应超时，请检查状态后再操作'}[r.status] || 'HTTP ' + r.status)); error.status = r.status; throw error; }
  return data;
}
function buttons() {
  const s = state, services = s?.system?.services || {};
  keyboardUI();
  liftPositionUI();
  document.querySelectorAll('[data-command]').forEach(b => {
    const kind = b.dataset.kind, cmd = b.dataset.command;
    let allow = connected && services[kind] && !busy;
    if (kind === 'arm_startup') {
      const startup = s?.arm_startup;
      allow &&= startup?.fresh && startup.configured;
      if (cmd === 'start') allow &&= s.arms?.fresh && s.arms.valid.every(Boolean) &&
        !['preparing','moving'].includes(startup.phase) && (!s.arms.enabled || startup.owned);
      else allow &&= startup.owned;
      b.disabled = !allow; return;
    }
    if (cmd === 'start') {
      if (kind === 'control') {
        allow &&= s.arms?.fresh && s.arms.phase !== 'monitor_only' && s.arms.valid.every(Boolean) && s.quest?.fresh && s.quest.rate_hz >= 50 && s.control?.fresh;
        if (s.control?.shared_enable) allow &&= ['disabled','stopped'].includes(s.control.system_phase) && s.base_status?.fresh && s.base?.valid && s.lift_status?.fresh && s.lift_status.feedback_healthy && s.lift_status.stationary && s.lift_status.allow_motion && (s.control.parking_policy === 'arms_home_lift_stopped' || s.control.supported_home_confirmed) && s.quest.grips.every(v => v < 0.35) && s.quest.sticks.every(v => Math.abs(v) < 0.16);
        else allow &&= s.control?.phase === 'disabled';
      }
      if (kind === 'lift') allow &&= s.lift_status?.feedback_healthy && s.lift_status?.stationary && s.lift_status?.fresh && !s.lift_status.enabled && s.lift_status.allow_motion && s.quest?.fresh && s.quest.grips.every(v => v < 0.35);
      if (kind === 'base') allow &&= s.base?.valid && s.base_status?.fresh && !s.base_status.enabled && s.quest?.fresh && !s.base_status.receive_only;
      if ((kind === 'base' || kind === 'lift') && (s.control?.shared_enable || s.base_status?.shared_enable || s.lift_status?.shared_enable)) allow = false;
      if (kind === 'episode') allow &&= s.recorder?.fresh && s.recorder.state === 'idle' && s.arms?.fresh && s.arms.enabled && $('task').value.trim().length > 0;
    } else if (kind === 'episode') allow &&= s.recorder?.fresh && s.recorder.state === 'recording';
    b.disabled = !allow;
  });
}
function hideFrame(id, message) {
  $(id).hidden = true; $(id+'-empty').hidden = false;
  $(id+'-empty').querySelector('span').textContent = message;
}
function offline() {
  stopKeyboard();
  document.body.classList.remove('recording');
  connected = false; $('offline').hidden = false;
  badge('connection','服务断开','bad');
  $('arm-joints').textContent = '连接中断，关节反馈已失效';
  for (const id of ['quest-badge','arms-badge','base-badge','lift-badge','camera-badge','record-badge']) badge(id,'状态失效','bad');
  cameraIds.forEach(id => hideFrame(id,'连接中断，画面已隐藏'));
  buttons();
}
function render(s) {
  state = s; connected = true; lastUpdate = Date.now(); $('offline').hidden = true;
  badge('connection','● 网页服务在线','good');
  text('updated','更新于 ' + new Date().toLocaleTimeString());
  badge('mode',s.system.mock ? 'MOCK · 模拟设备' : 'REAL · 实机模式',s.system.mock ? 'warn' : 'good');
  const q = s.quest || {}, a = s.arms, b = s.base, bs = s.base_status, r = s.recorder;
  const startup = s.arm_startup;
  const startupNames = {idle:'等待手动抬臂',preparing:'准备使能',moving:'正在缓慢抬臂',ready:'已到位 · 转矩保持',hold:'已停止 · 转矩保持'};
  const startupReasons = {calibration_hold_restored:'零位已更新 · 原姿态保持中',waiting_for_web_button:'点击按钮开始',operator_start:'正在执行启动姿态',operator_stop:'操作员停止',measured_arrival:'已通过关节反馈确认到位',feedback_lost:'反馈中断，已停止轨迹，需要手动重新启动',following_error:'跟随误差过大，已停止轨迹',startup_timeout:'抬臂超时，已停止轨迹',enable_feedback_timeout:'使能前反馈未就绪',initialization_failed:'驱动初始化失败',teleop_handover:'已交给遥操作控制',operator_disable:'已撤销使能'};
  badge('arm-startup-badge', startup?.fresh && startup.configured ? startupNames[startup.phase] || startup.phase : '启动功能未就绪', startup?.phase === 'ready' ? 'good' : 'warn');
  text('arm-startup-status', startup?.fresh && startup.configured ?
    (startupReasons[startup.reason] || startup.reason) + ' · 最大位置误差 ' + Number(startup.max_error_rad).toFixed(3) + ' rad' + (startup.last_rejection ? ' · ' + startup.last_rejection : '') : '等待 OpenArm 服务及启动姿态配置');
  const shared = !!s.control?.shared_enable;
  text('shared-control-status', a?.phase === 'monitor_only' ? '当前为只读设备检查，运动控制未开启。' : shared ? (systemPhases[s.control.system_phase] || s.control.system_phase) + ' · ' + s.control.reason : '当前运行版本使用独立使能；统一控制尚未加载。');
  document.querySelector('[data-kind="control"][data-command="start"]').textContent = shared ? '启用全部遥操作' : '使能双臂';
  document.querySelector('[data-kind="control"][data-command="stop"]').textContent = shared ? '双臂归位并失能 / 底盘升降停车' : '双臂返回并失能';
  document.querySelector('[data-kind="control"][data-command="disable"]').textContent = shared ? '停止全部 / 双臂保持' : '撤销双臂使能（移除转矩）';
  document.querySelectorAll('[data-independent-enable]').forEach(e => { e.hidden = shared; });
  document.querySelectorAll('[data-shared-control]').forEach(e => { e.hidden = !shared; });
  badge('quest-badge',q.fresh ? '已接收' : '未连接',q.fresh ? 'good' : 'bad');
  text('quest-rate',q.fresh ? q.rate_hz + ' Hz' : '— Hz');
  const tracking = v => ({0:'正常',1:'过期',2:'无效'}[v] || '未知');
  text('quest-detail',q.fresh ? '追踪：左 ' + tracking(q.left_tracking) + ' / 右 ' + tracking(q.right_tracking) : '请打开 Quest 应用并向板端发送 UDP');
  const armValid = a?.fresh && a.valid?.every(Boolean);
  const armCount = a?.fresh ? a.valid.filter(Boolean).length : 0;
  badge('arms-badge',armValid ? (s.system.mock ? '模拟反馈' : '反馈在线') : armCount ? '部分在线 · ' + armCount + '/16' : '未就绪',armValid ? 'good' : armCount ? 'warn' : 'bad');
  text('arm-phase',!a?.fresh ? '采集节点离线' : a.phase === 'fault' ? '驱动通信异常' : startup?.fresh && startup.owned ? startupNames[startup.phase] || startup.phase : a?.fresh && a.phase === 'monitor_only' ? '状态采集 · 未使能' : s.control?.fresh ? (phases[s.control.phase] || s.control.phase) : a?.fresh ? (a.enabled ? '已使能' : '未使能') : '采集节点离线');
  text('arms-detail',a?.fresh ? '左臂 ' + a.valid.slice(8,16).filter(Boolean).length + '/8 · 右臂 ' + a.valid.slice(0,8).filter(Boolean).length + '/8 有效反馈' : '没有新鲜关节反馈');
  const jointRows = [];
  for (const [side, offset] of [['左臂',8],['右臂',0]]) {
    for (let j = 0; j < 8; j++) {
      const k = offset + j, valid = a?.fresh && a.valid?.[k];
      const value = field => valid && Number.isFinite(a[field]?.[k]) ? a[field][k].toFixed(3) : '—';
      const temperature = field => valid && Number.isFinite(a[field]?.[k]) ? a[field][k] : '—';
      jointRows.push(`<tr><td>${side}</td><td>${j === 7 ? '夹爪' : 'J' + (j + 1)}</td><td>${valid ? '在线' : '无反馈'}</td><td>${value('position')}</td><td>${value('velocity')}</td><td>${value('effort')}</td><td>${temperature('mos_temperature')} / ${temperature('rotor_temperature')}</td></tr>`);
    }
  }
  $('arm-joints').innerHTML = jointRows.join('');
  badge('base-badge',!s.system.base_enabled ? '未配置' : b?.valid ? (b.mock ? '模拟反馈' : '反馈在线') : '未连接',b?.valid ? 'good' : 'warn');
  text('base-speed',b?.valid ? Math.hypot(b.velocity[0],b.velocity[1]).toFixed(3) + ' m/s' : '— m/s');
  text('base-detail',b?.valid ? b.voltage.toFixed(1)+' V · '+(bs?.fresh ? (bs.enabled ? '已启用' : '未使能') : '许可未知')+' · FlagStop '+b.flag_stop+' · '+(bs?.input_source || 'quest') : '检查供电、CH3、共地和底盘固件');
  if (bs?.fresh) $('base-detail').textContent += ' · ' + (bs.interface || 'CH3') + ' · ' + (bs.receive_only ? '只读诊断，禁止运动' : bs.reason) + ' · 有效包 ' + bs.accepted_groups;
  text('quest-base-hint', bs?.shared_enable ? '底盘随 X 统一启用；Y 已预留，不再独立使能。左摇杆平移，右摇杆左右转向；回中停车。B 停止全部，不执行回位。' : '当前独立模式：Y 切换底盘；左摇杆平移，右摇杆左右转向，回中停车。');
  const ls = s.lift_status, lift = s.lift;
  // Connection tracks complete CAN telemetry, independently of the stricter
  // measurement-valid flag used by motion planning and dataset consumers.
  const liftAgeMs = (Number(s.server_mono_ns) - Number(lift?.receive_first_mono_ns)) / 1e6;
  const liftConnected = !!(ls?.fresh && Number.isFinite(liftAgeMs) && liftAgeMs >= 0 && liftAgeMs <= 300);
  const liftTransport = ls?.transport === 'uart' ? 'UART' : 'CAN';
  text('lift-bus', ls?.transport === 'uart' ? 'USB 串口' : 'CH4');
  text('lift-transport', ls?.transport === 'uart' ? 'UART · 115200 bit/s' : '经典 CAN · 500 kbit/s');
  const liftFault = liftConnected && (!!lift.fault || !!(lift.flags & 128) || (liftTransport === 'CAN' && !!(lift.guard & 4)));
  const liftLabel = !ls?.fresh ? '服务离线' : !liftConnected ? `无 ${liftTransport} 反馈` :
    liftFault ? '已连接 · 保护/故障' : !(lift.flags & 1) ? '已连接 · 等待电机/位置参考' :
    ls.health === 'locked_startup_preparation' ? '已连接 · 锁闸启动准备' :
    !ls.feedback_healthy ? '已连接 · 电机反馈过期' : `已连接 · ${liftTransport}`;
  badge('lift-badge', liftLabel, liftFault ? 'bad' : liftConnected && ls.feedback_healthy ? 'good' : 'warn');
  const liftHasPosition = liftConnected && !!(lift.flags & 1) && Number.isFinite(lift.current_um);
  text('lift-height', liftHasPosition ? (lift.current_um / 1000).toFixed(3) + ' mm' : '— mm');
  const motorAgeMs = liftConnected ? Number(lift.feedback_age_ms) + liftAgeMs : NaN;
  const liftMeasurement = !liftHasPosition ? '位置无效' : liftFault ? '最近位置读数，保护/故障中' : lift.valid ? '位置样本有效' : '位置样本过期，显示最近读数';
  text('lift-detail', ls?.fresh ? (ls.enabled ? '已启用' : '未使能') + ' · ' + ls.phase + ' · ' + ls.reason +
    (liftConnected ? ' · ' + liftMeasurement + ' · 电机反馈年龄 ' + Math.round(motorAgeMs) + ' ms · ' + lift.speed_rpm + ' rpm · 抱闸输出' + ((lift.flags & 4) ? '释放' : '锁定') : ` · 等待完整 ${liftTransport} 遥测`) : '检查升降服务、配置接口及设备供电');
  const count = Object.values(s.cameras).filter(c => c.fresh).length;
  badge('camera-badge',s.system.no_cameras ? '已关闭' : count === 3 ? '全部在线' : count ? '部分在线' : '等待图像',count ? 'good' : 'warn');
  text('camera-count',count + ' / 3 在线');
  text('disk',(s.ram_staging ? 'RAM 缓冲可用 ' : '磁盘可用 ') + (s.system.free_gib < 0 ? '未知' : s.system.free_gib.toFixed(1) + ' GiB'));
  text('camera-note',s.system.mock ? '当前显示模拟图像，不能作为实机采集数据。' : s.system.no_cameras ? 'RGB 采集已暂停，请先检查 USB 连接和供电，恢复相机服务后再开启。' : '已接入的相机自动显示 RGB 预览；未接入的相机显示等待图像。');
  for (const id of cameraIds) {
    const c = s.cameras[id];
    const problem = c.driver?.fresh && c.driver.phase === 'error' ? c.driver.reason : null;
    text(id+'-state',c.fresh ? '● 在线' : problem ? '采集异常' : c.enabled ? '等待连接' : '已关闭');
    text(id+'-meta',`SN ${c.driver?.serial || '—'} · ${Number.isFinite(c.driver?.fps) ? c.driver.fps.toFixed(1) + ' FPS · ' : ''}` + (c.fresh ? `${c.width} × ${c.height} · #${c.sequence} · ${c.age_ms} ms · ${c.timestamp_quality}` : problem || '等待有效图像'));
    if (!c.fresh || !$('preview').checked) hideFrame(id,$('preview').checked ? (problem || '暂无实时图像') : '预览已暂停');
  }
  const recording = !!(r?.fresh && r.state === 'recording');
  document.body.classList.toggle('recording',recording);
  const recorderOffline = s.system.services?.episode ? '状态过期' : '未启动';
  badge('record-badge',r?.fresh ? (recording ? '● 录制中' : '待命') : recorderOffline,recording ? 'bad' : r?.fresh ? 'good' : 'warn');
  text('samples',r?.fresh ? r.samples : '—');
  text('record-phase',r?.fresh ? (phases[r.state] || r.state) : recorderOffline);
  text('episode-id',r?.episode || '尚未开始录制');
  text('qc',(r?.qc?.length ? '质量检查：' + JSON.stringify(r.qc) + '。' : '') + '保存目录：' + s.system.data_directory);
  const receiver = s.receiver || {}, peer = receiver.peer || {}, delivered = receiver.last_verified || {};
  text('receiver-state', receiver.online ? `接收中：${peer.name} (${peer.address})` : '未连接接收电脑');
  text('receiver-directory', receiver.online ? `保存到：${peer.directory} · ${peer.episode || ''} ${peer.message || '等待新数据'}` : '在要保存数据的电脑上运行 episode_receive，填写板端地址和本机保存目录。');
  text('receiver-last', delivered.episode_id ? `最近已校验：${delivered.episode_id}${delivered.source_released ? ' · X5 RAM 副本已释放' : ''}` : '尚无已校验的回传记录');
  text('storage-note', s.ram_staging ? '录制先进入 X5 RAM；断电前未回传的数据会丢失。PC 校验并保存后自动释放 RAM；断网保留待传，空间不足会停止录制。' : '录制保存在板端磁盘。接收电脑会自动下载并校验已完成的数据。');
  text('raw',JSON.stringify(s,null,2)); buttons();
}
async function poll() {
  try { render(await request('/api/state')); } catch { offline(); }
  setTimeout(poll,500);
}
async function preview(id) {
  if (connected && $('preview').checked && state?.cameras[id]?.fresh && !document.hidden) {
    try {
      const r = await fetch('/api/cameras/' + id, {cache:'no-store',signal:AbortSignal.timeout(1800)});
      if (!r.ok) throw new Error('图像过期');
      const blob = await r.blob();
      if (connected && $('preview').checked && state?.cameras[id]?.fresh) {
        const previous = frames[id]; frames[id] = URL.createObjectURL(blob);
        $(id).src = frames[id]; $(id).hidden = false; $(id+'-empty').hidden = true;
        if (previous) URL.revokeObjectURL(previous);
      }
    } catch { hideFrame(id,'图像读取失败 / 已过期'); }
  }
  setTimeout(() => preview(id),350);
}
async function episodes() {
  try {
    const items = await request('/api/episodes'); $('episodes').replaceChildren();
    if (!items.length) text('episodes','还没有已提交的数据');
    for (const id of items.sort().reverse().slice(0,30)) {
      const row = document.createElement('div'); row.className = 'episode-row';
      const label = document.createElement('span'); label.textContent = id;
      const link = document.createElement('a'); link.href = '/api/episodes/' + encodeURIComponent(id) + '/manifest';
      link.target = '_blank'; link.rel = 'noopener'; link.textContent = '查看清单 ↗';
      row.append(label,link); $('episodes').append(row);
    }
  } catch { text('episodes','列表读取失败，请刷新重试'); }
}
document.querySelectorAll('[data-command]').forEach(b => b.addEventListener('click', async () => {
  if (busy || b.disabled) return;
  busy = true; buttons();
  try {
    const kind = b.dataset.kind, command = b.dataset.command;
    const detail = command === 'start' ? $('task').value.trim() : $('reason').value.trim();
    const result = await request('/api/' + kind,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({command,detail})});
    log(b.textContent + '：' + result.message);
    render(await request('/api/state'));
    if (kind === 'episode') await episodes();
  } catch (e) { log('操作未确认：' + e.message + '。请查看设备状态，超时指令不会自动重发。'); }
  finally { busy = false; buttons(); }
}));
$('task').addEventListener('input',buttons);
$('preview').addEventListener('change',() => { if (!$('preview').checked) cameraIds.forEach(id => hideFrame(id,'预览已暂停')); });
$('refresh-episodes').addEventListener('click',episodes);
setInterval(() => { if (connected && Date.now() - lastUpdate > 2500) offline(); },300);
poll(); episodes(); cameraIds.forEach(preview);
setInterval(episodes,5000);

function keyboardUI() {
  const s = state, bs = s?.base_status;
  let reason = '';
  if (!connected) reason = '网页服务未连接，请检查网络。';
  else if (!s.system.services.base) reason = '底盘控制服务未启动。';
  else if (!bs?.fresh) reason = '底盘控制状态已过期。';
  else if (bs.shared_enable) reason = '统一 X 模式使用 Quest 摇杆；独立键盘使能已关闭。';
  else if (bs.receive_only) reason = '底盘处于只读诊断模式，需要启动控制模式。';
  else if (!s.base?.valid || !bs.feedback_fresh) reason = '没有有效底盘反馈，请检查供电和 CAN3。';
  else if (s.base.flag_stop) reason = '底盘反馈失能（FlagStop 非零），请检查下位机状态。';
  else if (s.recorder?.failure_menu) reason = '请先关闭录制失败菜单。';
  else if (bs.enabled && !keyboardActive) reason = '已有控制许可，请先停车退出，再使能本页键盘控制。';
  $('keyboard-start').disabled = (!keyboardActive && !!reason) || keyboardStarting;
  $('keyboard-start').setAttribute('aria-checked', String(keyboardActive));
  text('keyboard-start', keyboardStarting ? '键盘控制：正在开启…' : keyboardActive ? '键盘控制：开启' : '键盘控制：关闭');
  $('keyboard-stop').disabled = !connected || !s?.system?.services?.base;
  badge('keyboard-badge', keyboardStarting ? '正在使能' : keyboardActive ? '键盘已使能' : reason ? '暂不可使能' : '待使能', keyboardActive ? 'good' : 'warn');
  text('keyboard-state', keyboardError || (keyboardStarting ? '正在等待底盘确认…' : keyboardActive ? '键盘控制已启用，按住 WASD 行驶；松键停车。' : reason || '准备就绪，打开键盘控制开关即可，无需令牌。'));
  text('keyboard-input', '当前按键：' + ([...heldKeys].map(k => k.slice(3)).join(' + ') || '无（零速）'));
  for (const key of ['w','a','s','d']) $('key-'+key).classList.toggle('pressed', heldKeys.has('Key'+key.toUpperCase()));
}
async function baseKeyboard(command, detail = '') {
  return request('/api/keyboard', {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify({command,detail})});
}
function stopKeyboard() {
  const active = keyboardActive;
  keyboardActive = false; heldKeys.clear(); buttons();
  if (active) baseKeyboard('stop').catch(e => log('停车请求失败：' + e.message));
}
async function keyboardTick() {
  if (!keyboardActive || keyboardPending) return;
  keyboardPending = true;
  try {
    await baseKeyboard('keyboard', JSON.stringify([Number(heldKeys.has('KeyW'))-Number(heldKeys.has('KeyS')), Number(heldKeys.has('KeyA'))-Number(heldKeys.has('KeyD'))]));
  } catch (e) { keyboardError = '键盘控制已撤销：' + e.message; stopKeyboard(); log(keyboardError); }
  finally { keyboardPending = false; }
}
$('keyboard-start').addEventListener('click', async () => {
  if (keyboardStarting) return;
  if (keyboardActive) { stopKeyboard(); return; }
  keyboardStarting = true; keyboardError = ''; buttons();
  try {
    await baseKeyboard('keyboard_start');
    keyboardActive = true; heldKeys.clear();
    if (document.hidden || !document.hasFocus()) { stopKeyboard(); return; }
    $('keyboard-panel').focus();
    await keyboardTick(); if (keyboardActive) log('WASD 已启用，限速 0.05 m/s');
  } catch(e) { keyboardError = '键盘启用失败：' + e.message; log(keyboardError); }
  finally { keyboardStarting = false; buttons(); }
});
window.addEventListener('keydown', e => {
  if (!keyboardActive) return;
  if (e.code === 'Space' || e.code === 'Escape') { e.preventDefault(); stopKeyboard(); return; }
  if (e.target.closest('input,textarea,select,[contenteditable]')) return;
  if (['KeyW','KeyA','KeyS','KeyD'].includes(e.code)) { e.preventDefault(); heldKeys.add(e.code); keyboardUI(); keyboardTick(); }
});
window.addEventListener('keyup', e => { if (heldKeys.delete(e.code)) { e.preventDefault(); keyboardUI(); keyboardTick(); } });
window.addEventListener('blur', stopKeyboard);
window.addEventListener('pagehide', stopKeyboard);
document.addEventListener('visibilitychange', () => { if (document.hidden) stopKeyboard(); });
document.querySelector('[data-kind="base"][data-command="stop"]').addEventListener('click', stopKeyboard);
setInterval(keyboardTick, 50);

$('keyboard-stop').addEventListener('click', async () => {
  keyboardActive = false; heldKeys.clear(); keyboardError = ''; buttons();
  try { await baseKeyboard('stop'); log('底盘已撤销控制许可'); render(await request('/api/state')); }
  catch(e) { keyboardError = '停车请求未确认：' + e.message; buttons(); log(keyboardError); }
});

// Service state and sensor freshness are deliberately displayed independently.
let deviceStates = {}, deviceBusy = false;
function deviceButtons() {
  document.querySelectorAll('[data-device]').forEach(box => {
    const id = box.dataset.device, d = deviceStates[id];
    const running = d && ['active', 'activating', 'reloading'].includes(d.ActiveState);
    const labels = {active:'服务运行中', activating:'正在启动 / 自动重试', deactivating:'正在停止', inactive:'已关闭', failed:'启动失败，可重试'};
    box.querySelector('.service-state').textContent = d ? (d.LoadState === 'not-found' ? '服务未安装' : labels[d.ActiveState] || d.ActiveState) : '服务状态不可用';
    box.querySelectorAll('button').forEach(b => {
      b.disabled = deviceBusy || !connected || !d || d.LoadState !== 'loaded' || d.ActiveState === 'deactivating';
      if (b.dataset.serviceAction === 'toggle') {
        b.textContent = (running ? '关闭' : '开启') + (id === 'program' ? '控制与录制程序' : '采集');
        b.setAttribute('role', 'switch'); b.setAttribute('aria-checked', String(!!running));
      }
    });
  });
}
async function pollDevices() {
  try { deviceStates = (await request('/api/devices')).devices || {}; }
  catch { deviceStates = {}; }
  deviceButtons();
}
document.querySelectorAll('[data-device] button').forEach(b => b.addEventListener('click', async () => {
  const id = b.closest('[data-device]').dataset.device;
  const running = ['active', 'activating', 'reloading'].includes(deviceStates[id]?.ActiveState);
  const action = b.dataset.serviceAction === 'toggle' ? (running ? 'stop' : 'start') : 'restart';
  deviceBusy = true; deviceButtons();
  try { const result = await request('/api/devices/' + id, {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify({action})}); log(result.message); }
  catch(e) { log(e.message); }
  finally { deviceBusy = false; await pollDevices(); }
}));
pollDevices(); setInterval(pollDevices, 3000); setInterval(deviceButtons, 500);

function liftPositionUI() {
  const ls = state?.lift_status;
  let reason = !connected ? '网页未连接' : !state?.system?.services?.lift ? '升降服务未就绪' :
    !ls?.fresh ? '升降状态已过期' : !ls.dashboard_position_available || !ls.allow_motion ? '升降位置控制未开启' :
    !ls.feedback_healthy ? '升降反馈异常' : state?.recorder?.failure_menu ? '请先关闭录制失败菜单' :
    state?.control?.fresh && !['disabled','stopped'].includes(state.control.system_phase) ? '请先停止统一遥操作' : '';
  const ours = !!liftPositionOwner && ls?.dashboard_position && ls.external_owner === liftPositionOwner;
  if (liftPositionOwner && ls?.fresh && !ours && !liftPositionBusy && Date.now()-liftPositionArmedAt>750) { liftPositionOwner = ''; liftPositionError = '位置控制已停止，需要重新开启'; }
  if (!reason && !liftPositionOwner && ls?.enabled) reason = '已有其他控制会话，请先停止';
  if (!reason && !ls?.stationary && !liftPositionOwner) reason = '平台未停稳';
  if (ours && ls.external_task_id === liftPositionTask && ls.external_result === 'reached') liftPositionMoving = false;
  const ready = !liftPositionMoving && ours && ls.enabled && ls.stationary && ls.phase === 'idle' && ls.ready_for_command && !ls.session_present && !['running','accepted','settling'].includes(ls.external_result);
  $('lift-position-enable').disabled = !!reason || !!liftPositionOwner || liftPositionBusy;
  $('lift-position-move').disabled = !!reason || !ready || liftPositionBusy;
  $('lift-position-range').disabled = !!reason || !liftPositionOwner || liftPositionBusy;
  $('lift-position-number').disabled = $('lift-position-range').disabled;
  $('lift-position-stop').disabled = !connected || !state?.system?.services?.lift;
  text('lift-position-current', ls?.fresh && ls.feedback_healthy ? (ls.current_um/1000).toFixed(3) : '—');
  badge('lift-position-badge', ours ? ready ? '位置控制就绪' : '位置控制中' : reason ? '暂不可开启' : '等待开启', ours ? 'good' : 'warn');
  const detail = {configuring:'读取设备配置',waiting_for_settle:'等待停稳',connecting:'准备运动',session_ready:'等待设备就绪',move_ack:'运动准备中',moving:'正在移动',stopping:'正在停车',idle:ls?.external_result === 'reached' ? '已到位并锁定，可选择下一个位置' : '可选择目标位置'};
  text('lift-position-state', liftPositionError || reason || (ours ? detail[ls.phase] || ls.phase : '开启后可独立控制升降位置，无需 Quest。'));
}
function liftPositionPayload(extra = {}) { return JSON.stringify({owner:liftPositionOwner,sequence:++liftPositionSequence,...extra}); }
async function liftPositionRequest(command, extra = {}) {
  return request('/api/lift',{method:'POST',timeoutMs:command==='heartbeat'?350:undefined,headers:{'Content-Type':'application/json'},body:JSON.stringify({command:'position_'+command,detail:liftPositionPayload(extra)})});
}
function liftPositionStop(beacon = false) {
  if (!liftPositionOwner) return;
  const body = JSON.stringify({command:'position_stop',detail:liftPositionPayload()});
  liftPositionOwner = '';
  if (beacon) navigator.sendBeacon('/api/lift',new Blob([body],{type:'application/json'}));
  else request('/api/lift',{method:'POST',headers:{'Content-Type':'application/json'},body}).catch(e=>log('升降停车未确认：'+e.message));
  liftPositionUI();
}
function liftPositionTarget(value) {
  const raw = Number(value), um = Math.round(raw*1000);
  if (!String(value).trim() || !Number.isFinite(raw) || raw < -180 || raw > 180 || Math.abs(raw*1000-um)>1e-6) return false;
  const n=um/1000;
  liftPositionError='';
  $('lift-position-range').value = String(n); $('lift-position-number').value = String(n);
  text('lift-position-move','移动到目标 '+(n>0?'+':'')+n+' mm'); return true;
}
$('lift-position-range').addEventListener('input',e=>liftPositionTarget(e.target.value));
$('lift-position-number').addEventListener('input',e=>liftPositionTarget(e.target.value));
$('lift-position-number').addEventListener('change',e=>{if(!liftPositionTarget(e.target.value)){liftPositionError='目标须在 −180 至 +180 mm 内，最多三位小数';liftPositionUI();}});
$('lift-position-enable').addEventListener('click',async()=>{
  if(liftPositionBusy||liftPositionOwner)return;
  liftPositionBusy=true;liftPositionError='';liftPositionOwner='web-'+Date.now().toString(36)+'-'+Math.random().toString(36).slice(2);liftPositionSequence=0;liftPositionArmedAt=Date.now();liftPositionHeartbeatAckAt=Date.now();liftPositionMoving=false;liftPositionUI();
  try { await liftPositionRequest('arm'); liftPositionHeartbeatAckAt=Date.now(); $('lift-position-panel').focus(); log('升降位置控制已开启'); }
  catch(e){liftPositionError=e.message;liftPositionOwner='';log('升降位置控制开启失败：'+e.message);}
  finally{liftPositionBusy=false;liftPositionUI();}
});
$('lift-position-move').addEventListener('click',async()=>{
  if(!liftPositionOwner||liftPositionBusy||!liftPositionTarget($('lift-position-number').value))return;
  liftPositionBusy=true;liftPositionError='';liftPositionUI();
  try {liftPositionTask=Date.now();liftPositionMoving=true;await liftPositionRequest('move',{task_id:liftPositionTask,target_um:Math.round(Number($('lift-position-number').value)*1000)});log('已提交升降目标 '+$('lift-position-number').value+' mm');}
  catch(e){liftPositionMoving=false;liftPositionError=e.message;log('升降目标未接受：'+e.message);}
  finally{liftPositionBusy=false;liftPositionUI();}
});
$('lift-position-stop').addEventListener('click',async()=>{
  liftPositionStop();liftPositionError='';
  try{await request('/api/lift',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({command:'stop'})});log('升降已停车并退出位置控制');}
  catch(e){liftPositionError=e.message;}liftPositionUI();
});
async function liftPositionHeartbeat(){
  if(!liftPositionOwner||liftPositionPending)return;
  liftPositionPending=true;const owner=liftPositionOwner;
  try{await liftPositionRequest('heartbeat');if(owner===liftPositionOwner){liftPositionHeartbeatAckAt=Date.now();if(liftPositionError.startsWith('续租响应延迟')){liftPositionError='';liftPositionUI();}}}
  catch(e){if(owner===liftPositionOwner){
    const leaseMs=Number(state?.lift_status?.external_lease_ms)||1000;
    if((e.status && e.status<500)||Date.now()-liftPositionHeartbeatAckAt>=leaseMs){liftPositionError='位置控制已撤销：'+e.message;liftPositionStop();}
    else {liftPositionError='续租响应延迟，正在重试；超时将停车';liftPositionUI();}
  }}
  finally{liftPositionPending=false;}
}
setInterval(liftPositionHeartbeat,75);
window.addEventListener('blur',()=>liftPositionStop(true));
window.addEventListener('pagehide',()=>liftPositionStop(true));
document.addEventListener('visibilitychange',()=>{if(document.hidden)liftPositionStop(true);});
