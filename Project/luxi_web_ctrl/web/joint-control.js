'use strict';
let jointControlBusy = false, jointStopPending = false, jointControlError = '';
const jointInputs = Array.from({length:16}, (_, i) => $('joint-angle-' + i));
function jointLimits() {
  const j = state?.arm_startup?.joint_control;
  return j?.unit === 'rad' && j.joint_order === 'right_then_left' &&
    j.lower?.length === 16 && j.upper?.length === 16 &&
    j.lower.every((v,i) => Number.isFinite(v) && Number.isFinite(j.upper[i]) && v <= j.upper[i]) ? j : null;
}
const jointDegreeInputs = Array.from({length:16}, (_, i) => $('joint-angle-deg-' + i));
const jointDegreesPerRadian = 180 / Math.PI;
const jointDegreeOutOfRange = Array(16).fill(false);
function jointRadiansFromDegrees(value, i) {
  const j = jointLimits();
  // Retain exact configured endpoints; reject values beyond them below.
  if (j && value === j.lower[i] * jointDegreesPerRadian) return j.lower[i];
  if (j && value === j.upper[i] * jointDegreesPerRadian) return j.upper[i];
  return value / jointDegreesPerRadian;
}
function jointSyncInput(i, degrees = false) {
  const source = degrees ? jointDegreeInputs[i] : jointInputs[i];
  const destination = degrees ? jointInputs[i] : jointDegreeInputs[i];
  const value = Number(source.value), valid = source.value.trim() && Number.isFinite(value);
  const limits = jointLimits();
  jointDegreeOutOfRange[i] = !!(degrees && valid && limits &&
    (value < limits.lower[i]*jointDegreesPerRadian || value > limits.upper[i]*jointDegreesPerRadian));
  destination.value = valid ? String(degrees ? jointRadiansFromDegrees(value, i) : value * jointDegreesPerRadian) : '';
}
function jointTargets() {
  const j = jointLimits();
  if (!j) throw new Error('等待驱动提供原关节限位');
  return jointInputs.map((input, i) => {
    const raw = input.value.trim(), value = Number(raw), radians = value;
    if (!raw || jointDegreeOutOfRange[i] || !Number.isFinite(value) || !Number.isFinite(radians) || radians < j.lower[i] || radians > j.upper[i])
      throw new Error((i < 8 ? '右臂' : '左臂') + ' J' + (i % 8 + 1) + ' 角度为空、无效或超出原限位');
    return radians;
  });
}
function jointSpeed() {
  const raw = $('joint-speed').value.trim(), speed = Number(raw);
  if (!raw || !Number.isFinite(speed) || speed < .02 || speed > .2) throw new Error('速度上限须在 0.02–0.2 rad/s 内');
  return speed;
}
function jointControlUI() {
  const j = jointLimits(), a = state?.arms, s = state?.arm_startup;
  const running = !!j && ['preparing','moving'].includes(j.phase);
  const valid = a?.fresh && a.valid?.length === 16 && a.valid.every(Boolean) &&
    a.position?.length === 16 && a.position.every(Number.isFinite);
  const service = connected && state?.system?.services?.arm_startup && s?.fresh;
  let reason = !connected ? '网页未连接' : !service ? '机械臂服务未就绪' : !j ? '驱动待更新，尚无关节限位信息' :
    !valid ? '需要全部 16 轴的新鲜有效反馈' : ['editing','preparing'].includes(s?.manual_pose?.phase) ? '请先完成或退出手动摆姿设置' :
    s?.owned && ['moving','preparing'].includes(s.phase) ? '请先停止抬臂动作' : '';
  const c = state?.control;
  if (!reason && j && valid) {
    const outside=a.position.findIndex((v,i)=>v<j.lower[i] || v>j.upper[i]);
    if (outside>=0) reason=(outside<8?'右臂':'左臂')+' J'+(outside%8+1)+' 当前角度超出原限位，需先手动移回范围内';
  }
  if (!reason && c?.fresh && (c.shared_enable ? !['disabled','stopped'].includes(c.system_phase) : !['disabled','hold'].includes(c.phase)))
    reason = '请先停止遥操作';
  let inputError = '';
  try { jointTargets(); jointSpeed(); } catch(e) { inputError = e.message; }
  const editable = service && !!j && !running && !busy && !jointControlBusy;
  jointInputs.forEach((input,i) => {
    input.disabled = jointDegreeInputs[i].disabled = !editable;
    if (j) {
      input.min = String(j.lower[i]); input.max = String(j.upper[i]);
      jointDegreeInputs[i].min = String(j.lower[i]*jointDegreesPerRadian);
      jointDegreeInputs[i].max = String(j.upper[i]*jointDegreesPerRadian);
    }
    const range = factor => '['+(Math.ceil(j.lower[i]*factor*1e6)/1e6).toFixed(6)+', '+(Math.floor(j.upper[i]*factor*1e6)/1e6).toFixed(6)+']';
    text('joint-limit-'+i, j ? range(1)+' rad\n'+range(jointDegreesPerRadian)+' °' : '等待限位');
    text('joint-current-'+i, valid ? a.position[i].toFixed(3)+' rad\n'+(a.position[i]*jointDegreesPerRadian).toFixed(2)+' °' : '—');
  });
  $('joint-speed').disabled = !editable;
  $('joint-read-current').disabled = !editable || !valid;
  $('joint-execute').disabled = !editable || !!reason || !!inputError;
  $('joint-stop').disabled = !service || !running || jointStopPending;
  const fault = j?.phase === 'fault' || s?.manual_pose?.fault_latched;
  const labels = {idle:'等待目标',preparing:'确认反馈并准备使能',moving:'正在执行关节目标',ready:'已到位 · 姿态保持',stopped:'动作已停止 · 姿态保持',fault:'动作已中止 · 需重新操作'};
  badge('joint-control-badge', !j ? '等待驱动' : fault ? labels.fault : labels[j.phase] || j.phase,
    !fault && j?.phase === 'ready' && valid && a.enabled ? 'good' : 'warn');
  const reasons = {joint_feedback_lost:'关节反馈中断',joint_feedback_outside_limits:'实测角度超出原限位',joint_prepare_timeout:'准备反馈超时',joint_start_position_changed:'准备期间姿态发生变化',joint_following_error:'跟随误差过大',joint_motion_timeout:'等待到位超时',joint_control_loop_stalled:'控制循环中断',control_loop_stalled:'保持控制循环中断',feedback_lost_latched:'保持反馈中断'};
  const detail = jointControlError || reason || (fault ? (reasons[j.reason] || reasons[s?.manual_pose?.reason] || '反馈或运动异常，旧动作不会自动恢复') :
    running ? labels[j.phase]+'，请保持运动空间畅通。' : j?.phase==='ready' ? '全部关节反馈确认到位，正在保持；其他正常控制可重新接管。' :
    j?.phase==='stopped' ? (a?.enabled ? '已停止在最后发送的姿态，保留转矩。' : '准备已取消，当前未使能。') : inputError || '目标已填写，点击执行后缓慢移动并保持。');
  text('joint-control-status', detail);
}
jointInputs.forEach((input,i) => input.addEventListener('input', () => { jointSyncInput(i); jointControlError=''; jointControlUI(); }));
jointDegreeInputs.forEach((input,i) => input.addEventListener('input', () => { jointSyncInput(i,true); jointControlError=''; jointControlUI(); }));
$('joint-speed').addEventListener('input', () => { jointControlError=''; jointControlUI(); });
$('joint-read-current').addEventListener('click', () => {
  if ($('joint-read-current').disabled) return;
  jointInputs.forEach((input,i) => { input.value = String(state.arms.position[i]); jointSyncInput(i); });
  jointControlError=''; jointControlUI();
});
async function jointRequest(command, detail = '') {
  return request('/api/arm_startup',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({command,detail})});
}
$('joint-execute').addEventListener('click', async () => {
  if ($('joint-execute').disabled || jointControlBusy) return;
  let detail;
  try { detail = JSON.stringify({positions:jointTargets(),speed_rad_s:jointSpeed(),unit:'rad',request_id:'web-joint-'+Date.now().toString(36)+'-'+Math.random().toString(36).slice(2)}); }
  catch(e) { jointControlError=e.message; jointControlUI(); return; }
  jointControlBusy=true; jointControlError=''; pendingCommands++; busy=true; buttons();
  try { const result=await jointRequest('joint_move',detail); log(result.message); render(await request('/api/state')); }
  catch(e) { jointControlError=e.message; log('关节目标未接受：'+e.message); }
  finally { jointControlBusy=false; pendingCommands=Math.max(0,pendingCommands-1); busy=pendingCommands>0; buttons(); }
});
$('joint-stop').addEventListener('click', async () => {
  if ($('joint-stop').disabled || jointStopPending) return;
  jointStopPending=true; jointControlError=''; jointControlUI();
  try { const result=await jointRequest('joint_stop'); log(result.message); render(await request('/api/state')); }
  catch(e) { jointControlError=e.message; log('停止关节动作未确认：'+e.message); }
  finally { jointStopPending=false; buttons(); }
});
jointControlUI();
