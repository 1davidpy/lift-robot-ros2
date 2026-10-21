import json,os,subprocess,time,urllib.request
from pathlib import Path

root=Path('/home/sunrise/Desktop/LUXI_Lift_Robot/luxi_lift_ros2/lift_robot_ros2')
os.chdir(root)
pending=root/'work/gripper-zero-pending-20261021'
cfgdir=root/'Project/robot_bringup/config'
plan=json.loads((pending/'plan.json').read_text())
assert not plan['activated']
names=['arms-kh.json','arms-teleop.json','arms.json']
for name in names:
    assert json.loads((cfgdir/name).read_text())==json.loads((pending/(name+'.before')).read_text()),'Configuration changed since preparation'

def state():
    with urllib.request.urlopen('http://127.0.0.1:8000/api/state',timeout=3) as r:return json.load(r)

before=state()
assert before['arms']['fresh'] and all(before['arms']['valid'])
assert before['arm_startup']['owned'] and before['arms']['enabled']
assert not (before.get('control') or {}).get('fresh',False)
(pending/'before-restart.json').write_text(json.dumps(before,indent=2))
old=json.loads((pending/'arms-kh.json.before').read_text())
new=json.loads((pending/'arms-kh.json').read_text())
snapshot=json.loads((pending/'hold-snapshot.json').read_text())
raw_target=snapshot['raw_position_target']
target=[raw_target[k]-new['right' if k<8 else 'left']['offsets'][k%8] for k in range(16)]
for k in range(16):
    assert abs(target[k]+new['right' if k<8 else 'left']['offsets'][k%8]-raw_target[k])<1e-12
    actual_raw=before['arms']['position'][k]+old['right' if k<8 else 'left']['offsets'][k%8]
    assert abs(raw_target[k]-actual_raw)<.2

def write_configs(suffix):
    for name in names:
        dest=cfgdir/name; temp=dest.with_name('.'+name+'.zero-new')
        temp.write_bytes((pending/(name+suffix)).read_bytes())
        os.chmod(temp,dest.stat().st_mode & 0o777)
        os.replace(temp,dest)

write_configs('')
with (root/'log/gripper-zero-coordinate-test.log').open('w') as log:
    result=subprocess.run(['python3','Project/device_adapters/test/test_openarm_startup.py',str(root)],stdout=log,stderr=subprocess.STDOUT,timeout=55)
if result.returncode:
    write_configs('.before')
    raise RuntimeError('Coordinate regression failed; original files restored; real driver not restarted')
print('PASS: shifted coordinate regression',flush=True)
subprocess.run(['sudo','-n','systemctl','stop','luxi-openarm-monitor.service'],check=True,timeout=40)
started=time.monotonic_ns()
subprocess.run(['sudo','-n','systemctl','start','luxi-openarm-monitor.service'],check=True,timeout=40)
deadline=time.monotonic()+15
while time.monotonic()<deadline:
    j=state();a=j.get('arms') or {};s=j.get('arm_startup') or {}
    if (a.get('fresh') and all(a.get('valid',[])) and len(a.get('valid',[]))==16 and
            int(s.get('receive_mono_ns',0))>started and a.get('phase')=='monitor_only'):
        break
    time.sleep(.2)
else:raise RuntimeError('Fresh calibrated feedback unavailable; arms remain supported and disabled')
assert max(abs(a['position'][k]) for k in (7,15))<.002,'Gripper physically moved from calibrated point'
request=urllib.request.Request('http://127.0.0.1:8000/api/arm_startup',data=json.dumps({'command':'restore_hold','detail':json.dumps(target)}).encode(),headers={'Content-Type':'application/json'})
with urllib.request.urlopen(request,timeout=12) as r:res=json.load(r)
assert res['success']
deadline=time.monotonic()+5
while time.monotonic()<deadline:
    after=state()
    if after['arms']['enabled'] and after['arm_startup']['phase']=='hold':break
    time.sleep(.05)
else:raise RuntimeError('Holding restore not acknowledged')
time.sleep(.7)
after=state()
assert after['arms']['fresh'] and all(after['arms']['valid'])
assert max(abs(after['arms']['position'][k]) for k in (7,15))<.002

import rclpy
from luxi_interfaces.msg import ArmCommand
rclpy.init();node=rclpy.create_node('verify_gripper_zero_hold');applied=[]
node.create_subscription(ArmCommand,'/luxi/applied',lambda m:applied.append(m),1)
deadline=time.monotonic()+3
while not applied and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.03)
assert applied,'No applied command feedback after restore'
actual_target=[applied[-1].position[k]+new['right' if k<8 else 'left']['offsets'][k%8] for k in range(16)]
raw_change=max(abs(a-b) for a,b in zip(raw_target,actual_target))
assert raw_change<1e-9,'Physical target changed'
node.destroy_node();rclpy.shutdown()
drift=[abs(after['arms']['position'][k]+new['right' if k<8 else 'left']['offsets'][k%8]-before['arms']['position'][k]-old['right' if k<8 else 'left']['offsets'][k%8]) for k in range(16)]
report={'date':'2026-10-21','method':'persistent software coordinate offset; motor encoders not reset',
        'raw_zero_rad':plan['raw_zero_rad'],'zero_feedback_rad':{'right':after['arms']['position'][7],'left':after['arms']['position'][15]},
        'max_physical_target_change_rad':raw_change,'max_measured_arm_change_rad':max(drift[k] for k in range(16) if k%8!=7),
        'max_measured_gripper_change_rad':max(drift[k] for k in (7,15)),
        'valid_joints':sum(after['arms']['valid']),'enabled':after['arms']['enabled'],'phase':after['arm_startup']['phase'],
        'holding_targets_raw_before':raw_target,'holding_targets_raw_after':actual_target,
        'before_state':before,'after_state':after,'profiles':names}
(root/'log/gripper-zero-result.json').write_text(json.dumps(report,indent=2))
plan['activated']=True;plan['result']='log/gripper-zero-result.json'
(pending/'plan.json').write_text(json.dumps(plan,indent=2))
calibration={k:report[k] for k in ['date','method','raw_zero_rad','zero_feedback_rad','profiles']}
calibration['schema']='luxi_gripper_zero_v1'
(cfgdir/'gripper_zero_calibration.json').write_text(json.dumps(calibration,indent=2)+'\n')
print(json.dumps({k:report[k] for k in ['zero_feedback_rad','max_physical_target_change_rad','max_measured_arm_change_rad','max_measured_gripper_change_rad','valid_joints','enabled','phase']},indent=2),flush=True)
