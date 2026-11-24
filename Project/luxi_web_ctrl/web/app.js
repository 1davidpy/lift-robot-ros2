'use strict';
const $ = id => document.getElementById(id);
const cameraIds = ['head_rgb', 'left_wrist_rgb', 'right_wrist_rgb'];
const phases = {disabled:'未使能',homing:'回起始姿态',aligning:'等待手柄对齐',teleop:'遥操作中',hold:'保持',returning:'返回中',idle:'空闲',recording:'录制中'};
const systemPhases = {disabled:'已失能',preparing:'检查底盘与升降',starting:'双臂归位 / 对齐',arming:'等待统一使能确认',active:'全部遥操作已启用',stopping_base:'底盘 / 升降停车，双臂保持',homing_lift:'旧版流程：升降返回零点',returning_arms:'双臂返回 home，升降原位停止',disabling:'等待双臂失能反馈',stopped:'已停止 · 双臂保持'};
let liftPositionArmedAt = 0, liftPositionTask = 0, liftPositionMoving = false;
let liftPositionOwner = '', liftPositionSequence = 0, liftPositionPending = false, liftPositionBusy = false, liftPositionError = '';
let liftPositionHeartbeatAckAt = 0;
let keyboardActive = false, keyboardPending = 0, keyboardStarting = false;
let keyboardError = "";
let keyboardOwner = "";
let keyboardSequence = 0;
const heldKeys = new Set();
let state = null, connected = false, busy = false, pendingCommands = 0, manualPoseCancelPending = false, lastUpdate = 0;
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

let sourceSwitchBusy = false, sourceSwitchError = '';
function consoleSourceReason() {
  if (state?.system?.control_authorized === false) return '运动控制请使用板端控制台';
  const c = state?.control;
  if (!c || c.control_source === 'legacy' || !c.control_source) return '';
  return !c.fresh ? '控制权限状态已过期' : c.control_source !== 'console' ? '请先切换为控制台控制' : '';
}
function teleopStatusMessage() {
  const c = state?.control, q = state?.quest;
  if (!q?.fresh || !c?.vr_fresh) return '遥操权限已选中，但手柄数据中断。请恢复 Quest 应用的数据发送，再启用遥操。';
  const invalid = [q.left_tracking >= 2 ? '左手柄' : '', q.right_tracking >= 2 ? '右手柄' : ''].filter(Boolean);
  if (invalid.length) return invalid.join('、') + '追踪无效。请恢复手柄追踪，再松开 X 后长按约 1.5 秒启用遥操。';
  if (state?.recorder?.failure_menu) return '录制失败菜单正在阻止启用，请先关闭该菜单。';
  if (c.system_phase === 'active') return '遥操作已启用。控制台停止按钮始终可用。';
  if (!['disabled','stopped'].includes(c.system_phase)) return '遥操正在启动：' + (systemPhases[c.system_phase] || c.system_phase);
  return '遥操权限已选中，尚未启用。手柄回中立，松开 X 后重新长按约 1.5 秒，或点击“启用全部遥操作”。';
}

function controlSourceUI() {
  if (!$('source-console')) return;
  const readOnly = state?.system?.control_authorized === false;
  const c = state?.control, ready = !readOnly && connected && state?.system?.services?.control && c?.fresh && ['console','teleop'].includes(c.control_source);
  const consoleSelected = ready && c.control_source === 'console';
  const teleopSelected = ready && c.control_source === 'teleop';
  $('source-console').disabled = !ready || sourceSwitchBusy || consoleSelected;
  $('source-teleop').disabled = !ready || sourceSwitchBusy || teleopSelected;
  $('source-console').setAttribute('aria-pressed', String(consoleSelected));
  $('source-teleop').setAttribute('aria-pressed', String(teleopSelected));
  text('control-source-status', sourceSwitchError || (readOnly ? '此网关提供状态查看，运动控制请使用板端控制台。' : !ready ? '等待权限协调服务；可在设备采集中开启遥操作程序。' :
    consoleSelected ? '控制台持有权限：可操作关节、底盘和升降。遥操连接、追踪和普通输入不会接管。' :
    teleopStatusMessage()));
}
for (const source of ['console','teleop']) {
  const button = $('source-' + source);
  if (button) button.addEventListener('click', async () => {
    if (sourceSwitchBusy) return;
    sourceSwitchBusy = true; sourceSwitchError = ''; controlSourceUI();
    try {
      const result = await request('/api/control', {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify({command:'select_' + source, detail:''})});
      log(result.message); render(await request('/api/state'));
    } catch (error) {
      sourceSwitchError = error.message.includes('control_source_switch_requires_stopped') ?
        '请先停止当前动作并释放底盘、升降会话，再切换；机械臂可保持。' : error.message;
    } finally { sourceSwitchBusy = false; buttons(); }
  });
}

function liftAllowsTeleop(s) {
  const ls = s.lift_status;
  const isolated = !!(s.control?.fresh && s.control.lift_position_paused &&
    ls?.position_invalid_isolation && ls.position_invalid_paused &&
    !ls.fault && !(ls.flags & 128));
  return !!(ls?.fresh && ls.allow_motion &&
    (ls.feedback_healthy || isolated) &&
    (ls.stationary || (isolated && ls.position_invalid_stationary)));
}
function buttons() {
  controlSourceUI();
  const s = state, services = s?.system?.services || {};
  const poseEditing = ['editing','preparing','moving'].includes(s?.arm_startup?.manual_pose?.phase);
  keyboardUI();
  liftPositionUI();
  manualPoseUI();
  if (typeof jointControlUI === 'function') jointControlUI();
  document.querySelectorAll('[data-command]').forEach(b => {
    if (s?.system?.control_authorized === false) { b.disabled = true; return; }
    const kind = b.dataset.kind, cmd = b.dataset.command;
    let allow = connected && services[kind] && !busy;
    const sourceReason = consoleSourceReason();
    if (kind === 'arm_startup' && ['start','pose_enter','pose_hold'].includes(cmd) && sourceReason) {
      b.disabled = true; return;
    }
    if (kind === 'control' && ['start','stop','park'].includes(cmd) && s?.control?.control_source === 'console') {
      b.disabled = true; return;
    }
    if (kind === 'arm_startup') {
      const startup = s?.arm_startup;
      if (cmd.startsWith('pose_')) {
        const pose = startup?.manual_pose, armValid = s?.arms?.fresh && s.arms.valid?.length === 16 && s.arms.valid.every(Boolean);
        allow = connected && services[kind] && startup?.fresh && !!pose;
        if (cmd === 'pose_enter') allow &&= !busy && armValid && ['inactive','holding'].includes(pose.phase);
        else if (cmd === 'pose_hold') allow &&= !busy && armValid && !s.arms.enabled && pose.phase === 'editing' && pose.can_hold;
        else if (cmd === 'pose_cancel') allow &&= !manualPoseCancelPending && ['editing','preparing'].includes(pose.phase);
        else allow = false;
        b.disabled = !allow; return;
      }
      allow &&= startup?.fresh && startup.configured;
      const poseHolding = startup?.manual_pose?.phase === 'holding';
      if (cmd === 'start') allow &&= s.arms?.fresh && s.arms.valid.every(Boolean) &&
        !poseEditing && !['preparing','moving'].includes(startup.phase) && (!s.arms.enabled || startup.owned || poseHolding);
      else allow &&= startup.owned || poseHolding || startup?.manual_pose?.phase === 'moving';
      b.disabled = !allow; return;
    }
    if (cmd === 'start') {
      if (kind === 'control' || kind === 'episode') allow &&= !poseEditing;
      if (kind === 'control') {
        allow &&= s.arms?.fresh && s.arms.phase !== 'monitor_only' && s.arms.valid.every(Boolean) && s.quest?.fresh && s.quest.left_tracking < 2 && s.quest.right_tracking < 2 && s.control?.vr_fresh && s.control.quest_window_samples >= 3 && s.control?.fresh;
        if (s.control?.shared_enable) allow &&= ['disabled','stopped'].includes(s.control.system_phase) && s.base_status?.fresh && s.base?.valid && liftAllowsTeleop(s) && (['arms_home_lift_stopped','legacy_timed_return_lift_stopped'].includes(s.control.parking_policy) || s.control.supported_home_confirmed) && s.quest.grips.every(v => v < 0.35) && s.quest.sticks.every(v => Math.abs(v) < 0.16);
        else allow &&= ['disabled','hold'].includes(s.control?.phase);
        allow &&= !(s.base_status?.enabled && s.base_status.input_source === 'keyboard');
      }
      if (kind === 'lift') allow &&= s.lift_status?.feedback_healthy && s.lift_status?.stationary && s.lift_status?.fresh && !s.lift_status.enabled && s.lift_status.allow_motion && s.quest?.fresh && s.quest.grips.every(v => v < 0.35);
      if (kind === 'base') allow &&= s.base?.valid && s.base_status?.fresh && !s.base_status.enabled && s.quest?.fresh && !s.base_status.receive_only;
      if ((kind === 'base' || kind === 'lift') && (s.control?.shared_enable || s.base_status?.shared_enable || s.lift_status?.shared_enable)) allow = false;
      if (kind === 'episode') allow &&= s.recorder?.fresh && s.recorder.state === 'idle' && s.arms?.fresh && s.arms.enabled && $('task').value.trim().length > 0;
    } else if (kind === 'episode') allow &&= s.recorder?.fresh && s.recorder.state === 'recording';
    b.disabled = !allow;
  });
  deviceButtons();
}
function manualPoseUI() {
  const startup = state?.arm_startup, pose = startup?.manual_pose, arms = state?.arms;
  let label = '等待状态', detail = '', tone = 'warn';
  if (!connected) { label = '状态待确认'; detail = '网页服务未连接，无法确认电机状态。恢复连接后查看反馈；页面不会撤销姿态保持。'; }
  else if (!state?.system?.services?.arm_startup || !startup?.fresh) { label = '服务未就绪'; detail = '等待 OpenArm 服务及新鲜状态。'; }
  else if (!pose) { label = '驱动待更新'; detail = '当前 OpenArm 驱动尚不支持动作设置，请更新驱动后使用。'; }
  else if (pose.phase === 'editing') {
    label = '手动摆姿 · 已请求失能';
    detail = pose.reason === 'disable_or_enable_failed' ? '失能或使能操作未完成，请继续支撑并检查设备，暂勿手动摆动。' :
      arms?.fresh && !arms.enabled ? '双臂及夹爪已发送失能指令，请继续支撑并手动摆放。' : '正在确认双臂及夹爪失能，请继续支撑并等待反馈。';
    detail += pose.can_hold ? '摆放完成并停稳后，可点击“保持当前姿态”。' : '尚不能保持，请等待全部关节反馈有效且停稳。';
  } else if (pose.phase === 'moving') {
    label = '关节角度动作执行中';
    detail = '请在“关节角度控制”面板查看进度或停止并保持，完成后可使用其他控制入口接管。';
  } else if (pose.phase === 'preparing') {
    label = '准备保持';
    detail = '请继续支撑双臂及夹爪，等待反馈稳定（至少 300 ms）并确认使能。需要中止时可退出设置，继续保持失能。';
  } else if (pose.phase === 'holding') {
    const confirmed = !pose.fault_latched && arms?.fresh && arms.enabled && arms.valid?.length === 16 && arms.valid.every(Boolean);
    label = pose.fault_latched ? '保持已暂停 · 需重新确认' : confirmed ? '当前姿态已保持' : '姿态保持待确认'; tone = confirmed ? 'good' : 'warn';
    detail = pose.fault_latched ? '保持期间反馈或控制循环曾中断，已停止发送旧目标。请继续支撑；恢复反馈后可重新设置姿态，或明确启动正常控制接管，不会自动重发。' :
      confirmed ? '已保持双臂及夹爪的当前姿态。可由抬臂、遥操作等正常控制直接接管，无需另行解锁。' : '保持指令已接受，但使能或关节反馈尚未确认，请继续支撑并检查设备状态。';
  } else if (pose.phase === 'inactive') {
    label = '未进入设置';
    detail = '先支撑双臂及夹爪，再进入动作设置。进入后移除电机转矩，可手动摆姿。';
  } else { label = '状态未知'; detail = '动作设置状态无法识别，请检查驱动版本及原始状态。'; }
  const reasons = {feedback_lost:'关节反馈中断',feedback_invalid_or_stale:'全部关节反馈尚未新鲜有效',joint_outside_limits:'姿态超出关节限位，请手动移回限位内',arms_still_moving:'关节尚未停稳',stable_feedback_timeout:'等待稳定反馈超时，请重新保持',disable_or_enable_failed:'操作失败，请检查服务日志和设备连接',enable_feedback_timeout:'使能反馈超时',following_error:'保持跟随误差过大',operator_cancel:'已退出设置并保持失能',lifecycle_handover:'已交给正常控制',startup_handover:'已交给抬臂控制'};
  if (connected && startup?.fresh && pose?.reason && reasons[pose.reason]) detail += ' · ' + reasons[pose.reason];
  badge('manual-pose-badge', label, tone); text('manual-pose-status', detail);
}
function hideFrame(id, message) {
  $(id).hidden = true; $(id+'-empty').hidden = false;
  $(id+'-empty').querySelector('span').textContent = message;
}
function offline() {
  pauseKeyboard();
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
  text('shared-control-status', s.control?.control_source === 'console' ? '控制台持有控制权限，遥操作未启用。' : a?.phase === 'monitor_only' ? '当前为只读设备检查，运动控制未开启。' : shared ? (systemPhases[s.control.system_phase] || s.control.system_phase) + ' · ' + s.control.reason : '当前运行版本使用独立使能；统一控制尚未加载。');
  document.querySelector('[data-kind="control"][data-command="start"]').textContent = shared ? '启用全部遥操作' : '使能双臂';
  document.querySelector('[data-kind="control"][data-command="stop"]').textContent = shared ? '双臂归位并失能 / 底盘升降停车' : '双臂返回并失能';
  document.querySelector('[data-kind="control"][data-command="disable"]').textContent = '停止全部 / 双臂保持';
  if (startup?.manual_pose?.phase === 'moving') {
    document.querySelector('[data-kind="control"][data-command="stop"]').textContent = '停止关节动作并保持';
    document.querySelector('[data-kind="control"][data-command="disable"]').textContent = '停止关节动作并保持';
  }
  document.querySelectorAll('[data-independent-enable]').forEach(e => { e.hidden = shared; });
  document.querySelectorAll('[data-shared-control]').forEach(e => { e.hidden = !shared; });
  badge('quest-badge',q.fresh ? '已接收' : '未连接',q.fresh ? 'good' : 'bad');
  text('quest-rate',q.fresh ? q.rate_hz + ' Hz' : '— Hz');
  const tracking = v => ({0:'正常',1:'过期',2:'无效'}[v] || '未知');
  text('quest-detail',q.fresh ? '追踪：左 ' + tracking(q.left_tracking) + ' / 右 ' + tracking(q.right_tracking) : '请打开 Quest 应用并向板端发送 UDP');
  const armCount = a?.fresh ? a.valid?.filter(Boolean).length || 0 : 0;
  const armValid = armCount === 16;
  const missingJoints = a?.fresh ? Array.from({length:16}, (_, i) =>
    a.valid?.[i] ? '' : (i < 8 ? '右 ' : '左 ') + (i % 8 === 7 ? '夹爪' : 'J' + (i % 8 + 1))
  ).filter(Boolean) : [];
  badge('arms-badge',armValid ? (s.system.mock ? '模拟反馈' : '反馈在线') : armCount ? '部分在线 · ' + armCount + '/16' : '未就绪',armValid ? 'good' : armCount ? 'warn' : 'bad');
  const manualArmPhases = {moving:'关节角度动作执行中',editing:'动作设置 · 手动摆姿',preparing:'动作设置 · 准备保持',holding:startup?.manual_pose?.fault_latched ? '动作保持已暂停' : a?.enabled ? '动作设置 · 姿态保持' : '动作保持待确认'};
  text('arm-phase',!a?.fresh ? '采集节点离线' : a.phase === 'fault' ? '驱动通信异常' : startup?.fresh && manualArmPhases[startup.manual_pose?.phase] ? manualArmPhases[startup.manual_pose.phase] : startup?.fresh && startup.owned ? startupNames[startup.phase] || startup.phase : a?.fresh && a.phase === 'monitor_only' ? '状态采集 · 未使能' : s.control?.fresh && s.control.control_source !== 'console' ? (phases[s.control.phase] || s.control.phase) : a?.fresh ? (a.enabled ? '已使能' : '未使能') : '采集节点离线');
  const armSummary = a?.fresh ? '左臂 ' + (a.valid?.slice(8,16).filter(Boolean).length || 0) + '/8 · 右臂 ' +
    (a.valid?.slice(0,8).filter(Boolean).length || 0) + '/8 有效反馈' : '没有新鲜关节反馈';
  const act = startup?.fresh ? startup.act : null;
  const actLiftStop = act?.lift_stop_diagnostics ||
    (['stopped','fault'].includes(act?.stage) && act?.reason?.startsWith('act_lift_') ? {reason:act.reason} : null);
  const actLiftReasons = {
    act_lift_telemetry_timeout:'升降完整遥测超时',
    act_lift_publication_stale:'升降完整遥测超时',
    act_lift_feedback_unhealthy:'升降反馈异常',
    act_lift_motor_feedback_stale:'升降电机反馈过龄',
    act_lift_drive_fault:'升降驱动故障',
    act_lift_motion_inhibited:'升降运动受保护',
    act_lift_position_invalid:'升降位置无效',
    act_lift_not_stationary:'升降未静止',
    act_lift_height_not_ready:'升降未到 ACT 高度'
  };
  const actLiftDetail = actLiftStop ? ' · ACT 已停止 · ' + (actLiftReasons[actLiftStop.reason] || actLiftStop.reason) +
    (Number.isFinite(actLiftStop.lift_effective_age_ms) ? ' · 停止时电机反馈 ' + Math.round(actLiftStop.lift_effective_age_ms) + ' ms' : '') : '';
  text('arms-detail',armSummary + (armCount && missingJoints.length ? ' · 待恢复 ' + missingJoints.join('、') : '') + actLiftDetail);
  const jointRows = [];
  for (const [side, offset] of [['左臂',8],['右臂',0]]) {
    for (let j = 0; j < 8; j++) {
      const k = offset + j, valid = a?.fresh && a.valid?.[k];
      const value = field => valid && Number.isFinite(a[field]?.[k]) ? a[field][k].toFixed(3) : '—';
      const temperature = field => valid && Number.isFinite(a[field]?.[k]) ? a[field][k] : '—';
      jointRows.push(`<tr><td>${side}</td><td>${j === 7 ? '夹爪' : 'J' + (j + 1)}</td><td>${valid ? '在线' : '无反馈'}</td><td>${value('position')}</td><td>${valid && Number.isFinite(a.position?.[k]) ? (a.position[k]*180/Math.PI).toFixed(2) : '—'}</td><td>${value('velocity')}</td><td>${value('effort')}</td><td>${temperature('mos_temperature')} / ${temperature('rotor_temperature')}</td></tr>`);
    }
  }
  $('arm-joints').innerHTML = jointRows.join('');
  badge('base-badge',!s.system.base_enabled ? '未配置' : b?.valid ? (b.mock ? '模拟反馈' : '反馈在线') : '未连接',b?.valid ? 'good' : 'warn');
  text('base-speed',b?.valid ? Math.hypot(b.velocity[0],b.velocity[1]).toFixed(3) + ' m/s' : '— m/s');
  text('base-detail',b?.valid ? b.voltage.toFixed(1)+' V · '+(bs?.fresh ? (bs.enabled ? '已启用' : '未使能') : '许可未知')+' · FlagStop '+b.flag_stop+' · '+(bs?.input_source || 'quest') : '检查供电、CH3、共地和底盘固件');
  if (bs?.fresh) $('base-detail').textContent += ' · ' + (bs.interface || 'CH3') + ' · ' + (bs.receive_only ? '只读诊断，禁止运动' : bs.reason) + ' · 有效包 ' + bs.accepted_groups;
  text('quest-base-hint', bs?.shared_enable ? '底盘随 X 统一启用；Y 已预留，不再独立使能。左摇杆平移，右摇杆左右转向；回中停车。B 停止全部，不执行回位。' : '当前独立模式：Y 切换底盘；左摇杆平移，右摇杆左右转向，回中停车。');
  const ls = s.lift_status, lift = s.lift;
  // Display the most recent position within the drive's feedback watchdog
  // window. The stricter position sample validity applies to motion planning.
  const liftAgeMs = liftTelemetryAgeMs(s);
  const liftConnected = !!(ls?.fresh && !ls.uart_reconnecting && Number.isFinite(liftAgeMs) && liftAgeMs >= 0 && liftAgeMs <= 300);
  const liftTransport = ls?.transport === 'uart' ? 'UART' : 'CAN';
  text('lift-bus', ls?.transport === 'uart' ? 'USB 串口' : 'CH4');
  text('lift-transport', ls?.transport === 'uart' ? 'UART · 115200 bit/s' : '经典 CAN · 500 kbit/s');
  const liftFault = liftConnected && (!!lift.fault || !!(lift.flags & 128) || (liftTransport === 'CAN' && !!(lift.guard & 4)));
  const liftHasPosition = liftConnected && !!(lift.flags & 1) && Number.isFinite(lift.current_um);
  const motorAgeMs = liftConnected ? Number(lift.feedback_age_ms) + liftAgeMs : NaN;
  const motorFeedbackHealthy = liftConnected && ls.feedback_healthy && motorAgeMs >= 0 && motorAgeMs <= 500;
  const liftLabel = !ls?.fresh ? '服务离线' : ls.uart_reconnecting ? 'UART 重连中' : !liftConnected ? `无 ${liftTransport} 反馈` :
    liftFault ? '已连接 · 保护/故障' : (ls.position_invalid_isolation && ls.position_invalid_paused) ? '已连接 · 位置异常，仅暂停升降' : !(lift.flags & 1) ? '已连接 · 等待电机/位置参考' :
    ls.health === 'locked_startup_preparation' ? '已连接 · 锁闸启动准备' :
    !motorFeedbackHealthy ? '已连接 · 电机反馈过期' : `已连接 · ${liftTransport}`;
  badge('lift-badge', liftLabel, liftFault ? 'bad' : motorFeedbackHealthy && liftHasPosition && !ls.position_invalid_paused ? 'good' : 'warn');
  text('lift-height', liftHasPosition ? (lift.current_um / 1000).toFixed(3) + ' mm' : '— mm');
  const liftMeasurement = liftHasPosition ? '最近位置读数' : '位置无效';
  text('lift-detail', ls?.fresh ? (ls.enabled ? '已启用' : '未使能') + ' · ' + ls.phase + ' · ' + ls.reason +
    (liftConnected ? ' · ' + liftMeasurement + ' · 电机反馈年龄 ' + Math.round(motorAgeMs) + ' ms · ' + lift.speed_rpm + ' rpm · 抱闸输出' + ((lift.flags & 4) ? '释放' : '锁定') : ls.uart_reconnecting ? ' · 正在重新打开串口并确认新帧' : ` · 等待完整 ${liftTransport} 遥测`) : '检查升降服务、配置接口及设备供电');
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
  const kind = b.dataset.kind, command = b.dataset.command;
  const poseCancel = kind === 'arm_startup' && command === 'pose_cancel';
  if ((busy && !poseCancel) || b.disabled) return;
  if (poseCancel) manualPoseCancelPending = true;
  pendingCommands++; busy = true; buttons();
  try {
    const detail = command.startsWith('pose_') ? '' : command === 'start' ? $('task').value.trim() : $('reason').value.trim();
    const result = await request('/api/' + kind,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({command,detail})});
    log(b.textContent + '：' + result.message);
    render(await request('/api/state'));
    if (kind === 'episode') await episodes();
  } catch (e) { log('操作未确认：' + e.message + '。请查看设备状态，超时指令不会自动重发。'); }
  finally { if (poseCancel) manualPoseCancelPending = false; pendingCommands--; busy = pendingCommands > 0; buttons(); }
}));
$('task').addEventListener('input',buttons);
$('preview').addEventListener('change',() => { if (!$('preview').checked) cameraIds.forEach(id => hideFrame(id,'预览已暂停')); });
$('refresh-episodes').addEventListener('click',episodes);
setInterval(() => { if (connected && Date.now() - lastUpdate > 2500) offline(); },300);
poll(); episodes(); cameraIds.forEach(preview);
setInterval(episodes,5000);

function keyboardUI() {
  const s = state, bs = s?.base_status;
  if ($('web-keyboard-speed-limit')) text('web-keyboard-speed-limit', Number(bs?.web_keyboard_linear_limit ?? .05).toFixed(2));
  let reason = '';
  if (consoleSourceReason()) reason = consoleSourceReason();
  else if (!connected) reason = '网页服务未连接，请检查网络。';
  else if (!s.system.services.base) reason = '底盘控制服务未启动。';
  else if (!bs?.fresh) reason = '底盘控制状态已过期。';
  else if (bs.shared_enable && !bs.web_keyboard_control) reason = '底盘驱动尚未支持网页接管。';
  else if (bs.shared_enable && !keyboardActive && !bs.keyboard_takeover_available && !bs.enabled) reason = '控制协调尚未就绪，请检查遥操作程序服务；仅连接 Quest 不会占用权限。';
  else if (bs.receive_only) reason = '底盘处于只读诊断模式，需要启动控制模式。';
  else if (!s.base?.valid || !bs.feedback_fresh) reason = '没有有效底盘反馈，请检查供电和 CAN3。';
  else if (s.base.flag_stop) reason = '底盘反馈失能（FlagStop 非零），请检查下位机状态。';
  else if (s.recorder?.failure_menu) reason = '请先关闭录制失败菜单。';
  else if (!keyboardActive && bs.stationary === false) reason = '等待底盘反馈确认停稳后再开启网页控制。';
  else if (bs.enabled && (!keyboardActive || (bs.keyboard_owner && bs.keyboard_owner !== keyboardOwner))) reason = '底盘正由其他控制端使用，请先停车并释放控制权。';
  $('keyboard-start').disabled = (!keyboardActive && !!reason) || keyboardStarting;
  $('keyboard-start').setAttribute('aria-checked', String(keyboardActive));
  text('keyboard-start', keyboardStarting ? '网页控制：正在开启…' : keyboardActive ? '网页控制：开启' : '网页控制：关闭');
  $('keyboard-stop').disabled = !connected || !s?.system?.services?.base;
  badge('keyboard-badge', keyboardStarting ? '正在使能' : keyboardActive ? '键盘已使能' : reason ? '暂不可使能' : '待使能', keyboardActive ? 'good' : 'warn');
  text('keyboard-state', keyboardError || (keyboardStarting ? '正在等待底盘确认…' : keyboardActive ? (bs?.web_keyboard_paused ? '指令已暂停，网页权限保留；松开 WASD / Q/E 后再按，关闭开关可释放。' : '网页已取得底盘控制权，按住 WASD / Q/E 行驶；关闭开关后停车并让出控制权。') : reason || '准备就绪，无需连接 Quest；打开网页控制开关即可使用。'));
  text('keyboard-input', '当前按键：' + ([...heldKeys].map(k => k.slice(3)).join(' + ') || '无（零速）'));
  for (const key of ['w','a','s','d','q','e']) {
    const button = $('key-'+key);
    if (!button) continue;
    const pressed = heldKeys.has('Key'+key.toUpperCase());
    button.classList.toggle('pressed', pressed);
    button.setAttribute('aria-pressed', String(pressed));
    button.disabled = !keyboardActive;
  }
}
async function baseKeyboard(command, detail = '') {
  return request('/api/keyboard', {timeoutMs:command === 'keyboard' ? 800 : 12000, method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify({command,detail})});
}
function pauseKeyboard() {
  heldKeys.clear();
  if (keyboardActive) {
    keyboardError = '已停车，网页控制权保留；返回页面后重新按 WASD / Q/E，关闭开关可释放。';
    buttons();
    const owner = keyboardOwner, sequence = ++keyboardSequence;
    // A zero command may overtake a slow request; its newer sequence prevents
    // the delayed movement request from restarting the chassis afterwards.
    baseKeyboard('keyboard', JSON.stringify({owner, sequence, velocity:[0,0,0]}))
      .catch(e => { if (keyboardOwner === owner) log('停车请求暂未确认，等待指令超时停车：' + e.message); });
  }
}
function stopKeyboard() {
  const owner = keyboardOwner;
  keyboardActive = false; keyboardOwner = ''; heldKeys.clear(); buttons();
  if (owner) baseKeyboard('stop', owner).catch(e => log('停车请求失败：' + e.message));
}
async function keyboardTick(priority = false) {
  if (!keyboardActive || (keyboardPending && !priority)) return;
  keyboardPending += 1;
  const owner = keyboardOwner;
  const sequence = ++keyboardSequence;
  try {
    const result = await baseKeyboard('keyboard', JSON.stringify({owner, sequence, velocity:[Number(heldKeys.has('KeyW'))-Number(heldKeys.has('KeyS')), Number(heldKeys.has('KeyA'))-Number(heldKeys.has('KeyD')), Number(heldKeys.has('KeyQ'))-Number(heldKeys.has('KeyE'))]}));
    if (keyboardOwner === owner && sequence === keyboardSequence) {
      if (result.message?.includes('motion paused')) {
        heldKeys.clear(); keyboardError = '指令延迟，已停车并保留网页控制权；松开 WASD / Q/E 后再按。';
      } else keyboardError = '';
      buttons();
    }
  } catch (e) {
    if (keyboardOwner === owner && sequence === keyboardSequence) {
      heldKeys.clear();
      keyboardError = '请求暂时中断，已停车并保留网页控制权；松开 WASD / Q/E 后再按。' + e.message;
      if (e.status === 409 && e.message.includes('owned by another controller')) {
        keyboardActive = false; keyboardOwner = '';
        keyboardError = '网页控制权已释放，请重新打开开关。';
      }
      buttons(); log(keyboardError);
    }
  }
  finally { keyboardPending -= 1; }
}
$('keyboard-start').addEventListener('click', async () => {
  if (keyboardStarting) return;
  if (keyboardActive) { stopKeyboard(); return; }
  const owner = 'web-' + Date.now().toString(36) + '-' + Math.random().toString(36).slice(2);
  keyboardOwner = owner; keyboardSequence = 0;
  keyboardStarting = true; keyboardError = ''; buttons();
  try {
    await baseKeyboard('keyboard_start', owner);
    if (keyboardOwner !== owner) { await baseKeyboard('stop', owner); return; }
    keyboardActive = true; heldKeys.clear();
    if (document.hidden || !document.hasFocus()) { pauseKeyboard(); return; }
    $('keyboard-panel').focus();
    await keyboardTick(); if (keyboardActive) log('WASD / Q/E 已启用，限速 ' + Number(state?.base_status?.web_keyboard_linear_limit ?? .05).toFixed(2) + ' m/s');
  } catch(e) { keyboardError = '网页控制启用失败：' + e.message; stopKeyboard(); log(keyboardError); }
  finally { keyboardStarting = false; buttons(); }
});
window.addEventListener('keydown', e => {
  if (!keyboardActive) return;
  if (e.code === 'Space' || e.code === 'Escape') { e.preventDefault(); pauseKeyboard(); return; }
  if (e.target.closest('input,textarea,select,[contenteditable]')) return;
  if (['KeyW','KeyA','KeyS','KeyD','KeyQ','KeyE'].includes(e.code)) { e.preventDefault(); if (e.repeat) return; heldKeys.add(e.code); keyboardUI(); keyboardTick(); }
});
window.addEventListener('keyup', e => { if (heldKeys.delete(e.code)) { e.preventDefault(); keyboardUI(); keyboardTick(true); } });
['w','a','s','d','q','e'].forEach(name => {
  const button = $('key-'+name), key = 'Key'+name.toUpperCase();
  if (!button) return;
  button.addEventListener('pointerdown', e => {
    if (!keyboardActive || (e.button !== undefined && e.button !== 0)) return;
    e.preventDefault();
    heldKeys.add(key); keyboardUI(); keyboardTick();
    try { button.setPointerCapture(e.pointerId); } catch {}
  });
  const release = e => {
    if (!heldKeys.delete(key)) return;
    e.preventDefault(); keyboardUI(); keyboardTick(true);
  };
  for (const event of ['pointerup','pointercancel','lostpointercapture'])
    button.addEventListener(event, release);
});
window.addEventListener('blur', pauseKeyboard);
window.addEventListener('pagehide', pauseKeyboard);
document.addEventListener('visibilitychange', () => { if (document.hidden) pauseKeyboard(); });
document.querySelector('[data-kind="base"][data-command="stop"]').addEventListener('click', stopKeyboard);
setInterval(keyboardTick, 50);

$('keyboard-stop').addEventListener('click', async () => {
  const owner = keyboardOwner;
  keyboardActive = false; keyboardOwner = ''; heldKeys.clear(); keyboardError = ''; buttons();
  try { await baseKeyboard('stop', owner || '__web_operator_release__'); log('底盘已释放网页控制权'); render(await request('/api/state')); }
  catch(e) { keyboardError = '停车请求未确认：' + e.message; buttons(); log(keyboardError); }
});

// Service state and sensor freshness are displayed independently.
let deviceStates = {}, deviceCan = {}, deviceStatusRequest = 0;
const deviceBusy = new Set(), deviceResults = {};
function deviceManagementMode() {
  return state?.system?.device_management_mode ??
    (state?.system?.control_authorized === false ? 'read_only' : 'full');
}
function deviceInterlock(id, action) {
  const startup = state?.arm_startup;
  if (id === 'arms' && ['homing','ready','running'].includes(startup?.act?.stage)) return '请先结束 ACT 会话';
  if (id === 'arms' && ['editing','preparing','holding','moving'].includes(startup?.manual_pose?.phase)) return '请先退出双臂动作设置';
  const independentLiftRestart = id === 'lift' && action === 'restart' &&
    state?.system?.lift_reconnect_during_arm_hold === true;
  if ((state?.arms?.enabled && (id === 'arms' || state.arms.fresh) && !independentLiftRestart) ||
      (state?.base_status?.fresh && state.base_status.enabled) ||
      (state?.lift_status?.fresh && state.lift_status.enabled)) return '请先撤销电机使能';
  if (state?.recorder?.fresh && state.recorder.state === 'recording') return '请先结束录制';
  return '';
}
function deviceButtonReason(id, d, action) {
  if (!connected) return '等待网页连接';
  const mode = deviceManagementMode();
  if (mode === 'read_only') return '请在板端控制台管理采集服务';
  if (mode === 'reconnect' && action === 'toggle') return '此网关仅开放采集重连';
  if (deviceBusy.has(id)) return '此设备正在处理请求';
  if (d?.LoadState !== 'loaded') return '采集服务未就绪';
  if (d.ActiveState === 'deactivating') return '采集服务正在停止';
  return deviceInterlock(id, action);
}
function deviceRunning(d) {
  return ['active', 'activating', 'reloading'].includes(d?.ActiveState);
}
function deviceButtons() {
  document.querySelectorAll('[data-device]').forEach(box => {
    const id = box.dataset.device, d = deviceStates[id], result = deviceResults[id];
    const reason = deviceInterlock(id, 'restart');
    const busOff = id === 'arms' && (deviceCan.luxi_left === 'BUS-OFF' ? '左臂 CH1 BUS-OFF' :
      deviceCan.luxi_right === 'BUS-OFF' ? '右臂 CH2 BUS-OFF' : '');
    const labels = {active:'服务运行中', activating:'正在启动 / 自动重试', deactivating:'正在停止', inactive:'已关闭', failed:'启动失败，可重试'};
    const serviceLabel = d ? (d.LoadState === 'not-found' ? '服务未安装' : labels[d.ActiveState] || d.ActiveState) : '服务状态不可用';
    box.querySelector('.service-state').textContent = serviceLabel +
      (busOff ? ' · ' + busOff : '') + (reason ? ' · ' + reason : '') +
      (connected && deviceManagementMode() === 'read_only' ? ' · 网关只读' : '') +
      (result?.until > Date.now() ? ' · ' + result.message : '');
    box.querySelectorAll('button').forEach(b => {
      const action = b.dataset.serviceAction;
      b.title = deviceButtonReason(id, d, action);
      b.disabled = Boolean(b.title);
      if (action === 'toggle') {
        b.hidden = deviceManagementMode() === 'reconnect';
        b.textContent = (deviceRunning(d) ? '关闭' : '开启') + (id === 'program' ? '控制与录制程序' : '采集');
        b.setAttribute('role', 'switch'); b.setAttribute('aria-checked', String(deviceRunning(d)));
      }
    });
  });
}
async function pollDevices() {
  const requestId = ++deviceStatusRequest;
  let status;
  try { status = await request('/api/devices'); } catch {}
  const devices = status?.devices || {};
  if (requestId === deviceStatusRequest) {
    deviceStates = devices;
    deviceCan = status?.can || {};
    deviceButtons();
  }
  return devices;
}
function liftTelemetryAgeMs(s) {
  return (Number(s.server_mono_ns) - Number(s.lift?.receive_first_mono_ns)) / 1e6;
}
function deviceFeedback(id, s) {
  const now = Number(s?.server_mono_ns);
  if (id === 'arms') {
    const a = s?.arms;
    const valid = a?.fresh ? a.valid || [] : [];
    const left = valid.slice(8, 16).filter(Boolean).length;
    const right = valid.slice(0, 8).filter(Boolean).length;
    const missing = [left < 8 ? '左臂 CH1' : '', right < 8 ? '右臂 CH2' : ''].filter(Boolean);
    return {stamp:Number(a?.receive_mono_ns), ready:left + right === 16 && a.phase !== 'fault',
      detail:missing.length ? `关节反馈 ${left + right}/16 · 请检查 ${missing.join('、')} 的供电、接线和终端` : '双臂驱动通信异常'};
  }
  if (id === 'base') return {stamp:Number(s?.base?.receive_mono_ns), ready:!!s?.base?.valid,
    detail:'未收到完整底盘反馈，请检查底盘供电及 CH3'};
  if (id === 'quest') return {stamp:Number(s?.quest?.receive_mono_ns ?? now - 2e8), ready:!!s?.quest?.fresh,
    detail:'未收到有效 Quest 报文，请检查应用、Wi-Fi 和板端 UDP 5006 目标'};
  if (id === 'lift') {
    const ls = s?.lift_status, lift = s?.lift;
    const age = s ? liftTelemetryAgeMs(s) : NaN;
    const transport = ls?.transport === 'uart' ? 'UART' : ls?.transport === 'can' ? 'CAN' : '';
    const link = ls?.fresh && !ls.uart_reconnecting && age >= 0 && age <= 300;
    const fault = link && (lift.fault || (lift.flags & 128) || (transport === 'CAN' && (lift.guard & 4)));
    const protection = fault ? '升降保护/故障仍在' : link && (ls.position_invalid_paused || !(lift.flags & 1)) ?
      '升降位置保护仍在，保持暂停' : link && !ls.feedback_healthy ? '升降电机反馈仍异常' :
      link && !lift.valid ? '升降位置样本过期，等待新读数' : '';
    return {stamp:Number(lift?.receive_first_mono_ns), ready:!!link && !protection, protection,
      detail:protection || (ls?.uart_reconnecting ? '正在重开升降串口并确认新帧' :
        '未收到完整升降' + (transport ? ' ' + transport : '') + ' 遥测，请检查设备与接口')};
  }
  if (id === 'program') return {stamp:now - 1e9,
    ready:!!(s?.control?.fresh && s?.recorder?.fresh && s.system.services?.control && s.system.services?.episode),
    detail:'控制与录制节点尚未就绪'};
  const c = s?.cameras?.[id];
  return {stamp:now - Number(c?.age_ms) * 1e6, ready:!!c?.fresh,
    detail:c?.driver?.fresh && c.driver.phase === 'error' ? c.driver.reason : '未收到图像，请检查此相机的 USB 连接'};
}
function setDeviceResult(id, message, pending = false) {
  deviceResults[id] = {message, until:pending ? Infinity : Date.now() + 30000};
  deviceButtons();
  if (!pending) log(message);
}
function deviceAction(id, action) {
  return request('/api/devices/' + id, {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify({action})});
}
async function reconnectDevice(id, before, startOnly = false) {
  const baseline = Number(state?.server_mono_ns);
  setDeviceResult(id, '正在检查接口并恢复采集…', true);
  const following = before.ActiveState === 'activating' || (startOnly && deviceRunning(before));
  const submitted = following ? {} : await deviceAction(id,
    deviceManagementMode() === 'reconnect' || deviceRunning(before) ||
      (id === 'lift' && state?.system?.lift_reconnect_during_arm_hold === true) ? 'restart' : 'start');
  const selectedUnit = submitted.unit || before.Id;
  const deadline = Date.now() + 42000;
  const beforeStart = Number(before.ExecMainStartTimestampMonotonic) * 1000 || 0;
  let confirmed = following || !deviceRunning(before), lastStamp = 0, consecutive = 0;
  let runningSince = 0, knownState = before, lastProtection = '', lastStart = 0;
  while (Date.now() < deadline) {
    const current = (await pollDevices())[id];
    if (current?.Id && selectedUnit && current.Id !== selectedUnit) throw new Error('采集服务选择已变化，请重新确认');
    if (current) knownState = current;
    const startedNs = Number(current?.ExecMainStartTimestampMonotonic) * 1000 || 0;
    if (startedNs > Math.max(baseline, beforeStart) ||
        ['deactivating', 'inactive'].includes(current?.ActiveState)) confirmed = true;
    if (current?.ActiveState === 'failed') throw new Error('采集服务启动失败' + (current.Result ? ' · ' + current.Result : ''));
    if (current?.ActiveState === 'active') {
      if (!runningSince || startedNs !== lastStart) runningSince = Date.now();
      const feedback = deviceFeedback(id, state);
      if (startedNs !== lastStart || (feedback.protection || '') !== lastProtection) consecutive = 0;
      lastStart = startedNs; lastProtection = feedback.protection || '';
      if ((feedback.ready || feedback.protection) && feedback.stamp > Math.max(baseline, startedNs || 0)) {
        if (feedback.stamp > lastStamp) { lastStamp = feedback.stamp; consecutive++; }
        if (consecutive >= 2 && confirmed) return feedback.protection ? '遥测已恢复 · ' + feedback.protection : '重连完成 · 已确认实时反馈';
        if (consecutive >= 2 && !startedNs && Date.now() - runningSince >= 3000)
          return feedback.protection ? '已收到升降遥测 · ' + feedback.protection : '当前反馈在线 · 旧版管理服务未提供重启确认';
      } else {
        consecutive = 0;
      }
      if (id === 'quest' && Date.now() - runningSince >= 6000 && !feedback.ready)
        throw new Error(feedback.detail);
      setDeviceResult(id, '采集服务已运行，正在确认实时反馈…', true);
    } else {
      runningSince = 0; lastStamp = 0; consecutive = 0;
    }
    await new Promise(resolve => setTimeout(resolve, 750));
  }
  if (knownState?.ActiveState === 'active') {
    const feedback = deviceFeedback(id, state);
    throw new Error(feedback.ready ? '当前反馈在线 · 服务重启未得到确认' : feedback.detail);
  }
  throw new Error(knownState?.Result && knownState.Result !== 'success' ?
    '采集服务未就绪 · ' + knownState.Result : '等待采集服务启动超时');
}
document.querySelectorAll('[data-device] button').forEach(b => b.addEventListener('click', async () => {
  const id = b.closest('[data-device]').dataset.device;
  if (b.disabled || deviceBusy.has(id)) return;
  const toggle = b.dataset.serviceAction === 'toggle';
  const stop = toggle && deviceRunning(deviceStates[id]);
  deviceBusy.add(id); deviceButtons();
  try {
    const current = (await pollDevices())[id];
    if (current?.LoadState !== 'loaded' || current.ActiveState === 'deactivating') throw new Error('采集服务暂不可用');
    const message = stop ?
      (deviceRunning(current) ? (await deviceAction(id, 'stop')).message : '采集服务已关闭') :
      await reconnectDevice(id, current, toggle);
    setDeviceResult(id, message);
  }
  catch(e) { setDeviceResult(id, e.message); }
  finally { deviceBusy.delete(id); await pollDevices(); }
}));
pollDevices(); setInterval(pollDevices, 3000); setInterval(deviceButtons, 500);

function liftPositionUI() {
  const ls = state?.lift_status;
  let reason = consoleSourceReason() || (!connected ? '网页未连接' : !state?.system?.services?.lift ? '升降服务未就绪' :
    !ls?.fresh ? '升降状态已过期' : !ls.dashboard_position_available || !ls.allow_motion ? '升降位置控制未开启' :
    !ls.feedback_healthy ? '升降反馈异常' : state?.recorder?.failure_menu ? '请先关闭录制失败菜单' :
    state?.control?.fresh && !['disabled','stopped'].includes(state.control.system_phase) ? '请先停止统一遥操作' : '');
  const ours = !!liftPositionOwner && ls?.dashboard_position && ls.external_owner === liftPositionOwner;
  const positionTimeout = ls?.fresh && ls.reason === 'fresh_position_feedback_timeout';
  if (liftPositionOwner && ls?.fresh && !ours && !liftPositionBusy && Date.now()-liftPositionArmedAt>750) {
    liftPositionOwner = '';
    liftPositionError = '位置控制已停止，需要重新开启';
  }
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
  const progress = ls?.waiting_for_fresh_position ? '等待新鲜位置反馈' : detail[ls?.phase] || ls?.phase;
  text('lift-position-state', positionTimeout ? '等待新鲜位置反馈超时，目标已撤销，需要重新开启' :
    liftPositionError || reason || (ours ? progress : '开启后可独立控制升降位置，无需 Quest。'));
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
  if(!liftPositionOwner||liftPositionBusy||liftPositionPending)return;
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
