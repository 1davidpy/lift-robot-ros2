'use strict';
const $ = id => document.getElementById(id);
const cameraIds = ['head_rgb', 'left_wrist_rgb', 'right_wrist_rgb'];
const phases = {disabled:'未使能',homing:'回起始姿态',aligning:'等待手柄对齐',teleop:'遥操作中',hold:'保持',returning:'返回中',idle:'空闲',recording:'录制中'};
let state = null, connected = false, busy = false, lastUpdate = 0;
const frames = {};
function text(id, value) { $(id).textContent = value; }
function badge(id, label, tone = '') { text(id, label); $(id).className = 'pill ' + tone; }
function log(message) { text('result', new Date().toLocaleTimeString() + '  ' + message + '\n' + $('result').textContent.slice(0, 2500)); }
function auth() { return {'Authorization':'Bearer ' + $('token').value}; }
async function request(path, options = {}) {
  const r = await fetch(path, {...options, headers:{...auth(), ...options.headers}, signal:AbortSignal.timeout(options.method ? 12000 : 2500), cache:'no-store'});
  const body = await r.text();
  let data; try { data = JSON.parse(body); } catch { data = null; }
  if (!r.ok) throw new Error(data?.message || ({403:'控制令牌无效或未设置',409:'设备当前状态不允许此操作',503:'对应节点未就绪',504:'指令响应超时，请检查状态后再操作'}[r.status] || 'HTTP ' + r.status));
  return data;
}
function buttons() {
  const s = state, services = s?.system?.services || {};
  document.querySelectorAll('[data-command]').forEach(b => {
    const kind = b.dataset.kind, cmd = b.dataset.command;
    let allow = connected && s?.system?.control_authorized && services[kind] && !busy;
    if (cmd === 'start') {
      if (kind === 'control') allow &&= s.arms?.fresh && s.arms.valid.every(Boolean) && s.quest?.fresh && s.quest.rate_hz >= 50 && s.control?.fresh && s.control.phase === 'disabled';
      if (kind === 'base') allow &&= s.base?.valid && s.base_status?.fresh && !s.base_status.enabled && s.quest?.fresh;
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
  connected = false; $('offline').hidden = false;
  badge('connection','服务断开','bad');
  for (const id of ['quest-badge','arms-badge','base-badge','camera-badge','record-badge']) badge(id,'状态失效','bad');
  cameraIds.forEach(id => hideFrame(id,'连接中断，画面已隐藏'));
  buttons();
}
function render(s) {
  state = s; connected = true; lastUpdate = Date.now(); $('offline').hidden = true;
  badge('connection','● 服务在线','good');
  text('updated','更新于 ' + new Date().toLocaleTimeString());
  badge('mode',s.system.mock ? 'MOCK · 模拟设备' : 'REAL · 实机模式',s.system.mock ? 'warn' : 'good');
  badge('auth',s.system.control_authorized ? '可操作' : '只读',s.system.control_authorized ? 'good' : 'warn');
  const q = s.quest || {}, a = s.arms, b = s.base, bs = s.base_status, r = s.recorder;
  badge('quest-badge',q.fresh ? '已接收' : '未连接',q.fresh ? 'good' : 'bad');
  text('quest-rate',q.fresh ? q.rate_hz + ' Hz' : '— Hz');
  const tracking = v => ({0:'未指定',1:'正常',2:'无效'}[v] || '未知');
  text('quest-detail',q.fresh ? '追踪：左 ' + tracking(q.left_tracking) + ' / 右 ' + tracking(q.right_tracking) : '请打开 Quest 应用并向板端发送 UDP');
  const armValid = a?.fresh && a.valid?.every(Boolean);
  badge('arms-badge',armValid ? (s.system.mock ? '模拟反馈' : '反馈在线') : '未就绪',armValid ? 'good' : 'bad');
  text('arm-phase',s.control?.fresh ? (phases[s.control.phase] || s.control.phase) : '状态未知');
  text('arms-detail',a?.fresh ? '左臂 ' + a.valid.slice(0,8).filter(Boolean).length + '/8 · 右臂 ' + a.valid.slice(8).filter(Boolean).length + '/8 有效反馈' : '没有新鲜关节反馈');
  badge('base-badge',!s.system.base_enabled ? '未配置' : b?.valid ? (b.mock ? '模拟反馈' : '反馈在线') : '未连接',b?.valid ? 'good' : 'warn');
  text('base-speed',b?.valid ? Math.hypot(b.velocity[0],b.velocity[1]).toFixed(3) + ' m/s' : '— m/s');
  text('base-detail',b?.valid ? b.voltage.toFixed(1)+' V · '+(bs?.fresh ? (bs.enabled ? '已启用' : '未使能') : '许可未知')+' · FlagStop '+b.flag_stop : '检查供电、CH3 和底盘固件');
  const count = Object.values(s.cameras).filter(c => c.fresh).length;
  badge('camera-badge',s.system.no_cameras ? '已关闭' : count === 3 ? '全部在线' : '等待图像',count === 3 ? 'good' : 'warn');
  text('camera-count',count + ' / 3 在线');
  text('disk','磁盘可用 ' + (s.system.free_gib < 0 ? '未知' : s.system.free_gib.toFixed(1) + ' GiB'));
  text('camera-note',s.system.mock ? '当前显示模拟图像，不能作为实机采集数据。' : s.system.no_cameras ? '本次启动关闭了相机；接入正式 SDK 后按说明开启。' : '三路图像携带时间戳；主机取帧时间不等于硬件曝光同步。');
  for (const id of cameraIds) {
    const c = s.cameras[id];
    text(id+'-state',c.fresh ? '● 在线' : c.enabled ? '无新鲜图像' : '已关闭');
    text(id+'-meta',c.fresh ? `${c.width} × ${c.height} · #${c.sequence} · ${c.age_ms} ms · ${c.timestamp_quality}` : '等待有效图像');
    if (!c.fresh || !$('preview').checked) hideFrame(id,$('preview').checked ? '暂无实时图像' : '预览已暂停');
  }
  const recording = r?.fresh && r.state === 'recording';
  document.body.classList.toggle('recording',recording);
  badge('record-badge',r?.fresh ? (recording ? '● 录制中' : '待命') : '未连接',recording ? 'bad' : r?.fresh ? 'good' : 'warn');
  text('samples',r?.fresh ? r.samples : '—');
  text('record-phase',r?.fresh ? (phases[r.state] || r.state) : '未知');
  text('episode-id',r?.episode || '尚未开始录制');
  text('qc',(r?.qc?.length ? '质量检查：' + JSON.stringify(r.qc) + '。' : '') + '保存目录：' + s.system.data_directory);
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
$('unlock').addEventListener('click',async () => { try { const s = await request('/api/state'); render(s); log(s.system.control_authorized ? '控制权限验证通过' : '当前只读，请检查令牌'); } catch(e) { log(e.message); } });
$('token').addEventListener('input',() => { if (state) state.system.control_authorized = false; buttons(); });
$('task').addEventListener('input',buttons);
$('preview').addEventListener('change',() => { if (!$('preview').checked) cameraIds.forEach(id => hideFrame(id,'预览已暂停')); });
$('refresh-episodes').addEventListener('click',episodes);
setInterval(() => { if (connected && Date.now() - lastUpdate > 2500) offline(); },300);
poll(); episodes(); cameraIds.forEach(preview);
