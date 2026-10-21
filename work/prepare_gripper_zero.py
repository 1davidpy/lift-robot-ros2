import copy,json,time,urllib.request
from pathlib import Path
from statistics import median
root=Path('/home/sunrise/Desktop/LUXI_Lift_Robot/luxi_lift_ros2/lift_robot_ros2')
cfgdir=root/'Project/robot_bringup/config'
pending=root/'work/gripper-zero-pending-20261021'
pending.mkdir(exist_ok=True)
samples=[]
for _ in range(12):
    with urllib.request.urlopen('http://127.0.0.1:8000/api/state',timeout=3) as r:j=json.load(r)
    assert j['arms']['fresh'] and all(j['arms']['valid'])
    assert max(abs(j['arms']['velocity'][k]) for k in (7,15))<.05
    samples.append(j)
    time.sleep(.06)
live=json.loads((cfgdir/'arms-kh.json').read_text())
zeros={side:median(s['arms']['position'][k] for s in samples)+live[side]['offsets'][7] for side,k in [('right',7),('left',15)]}
changes={}
for name in ['arms-kh.json','arms-teleop.json','arms.json']:
    old=json.loads((cfgdir/name).read_text());new=copy.deepcopy(old)
    for side in ['right','left']:
        a=new[side];delta=zeros[side]-a['offsets'][7]
        a['offsets'][7]=zeros[side]
        a['home'][7]-=delta
        a['limits'][7]=[v-delta for v in a['limits'][7]]
        a['gripper_open']-=delta;a['gripper_closed']-=delta
        if 'startup_pose' in new:new['startup_pose'][side][7]-=delta
        # Coordinate translation preserves every physical target and limit.
        b=old[side]
        for field in ['home','offsets','limits']:
            assert a[field][:7]==b[field][:7]
        for field in ['gripper_open','gripper_closed']:
            assert abs((a[field]+a['offsets'][7])-(b[field]+b['offsets'][7]))<1e-12
        assert abs(a['home'][7]+a['offsets'][7]-b['home'][7]-b['offsets'][7])<1e-12
        assert all(abs(a['limits'][7][i]+a['offsets'][7]-b['limits'][7][i]-b['offsets'][7])<1e-12 for i in range(2))
        if 'startup_pose' in new:
            assert new['startup_pose'][side][:7]==old['startup_pose'][side][:7]
            assert abs(new['startup_pose'][side][7]+a['offsets'][7]-old['startup_pose'][side][7]-b['offsets'][7])<1e-12
    (pending/(name+'.before')).write_text(json.dumps(old,indent=2)+'\n')
    (pending/name).write_text(json.dumps(new,indent=2)+'\n')
    changes[name]={side:{'before_offset':old[side]['offsets'][7],'after_offset':new[side]['offsets'][7]} for side in ['right','left']}
plan={'method':'software coordinate origin; do not reset motor encoder or move targets', 'raw_zero_rad':zeros,'changes':changes,'before_state':samples[-1], 'activated':False}
(pending/'plan.json').write_text(json.dumps(plan,indent=2)+'\n')
print(json.dumps({'prepared':str(pending),'raw_zero_rad':zeros,'checks':'all J1-J7 unchanged; all raw gripper physical targets/limits preserved; not activated'},indent=2))
