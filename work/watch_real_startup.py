import json, time, urllib.request
from pathlib import Path
root=Path('/home/sunrise/Desktop/LUXI_Lift_Robot/luxi_lift_ros2/lift_robot_ros2')
deadline=time.monotonic()+90
active=False
last=0
with (root/'log/startup-pose-real-test.jsonl').open('w') as out:
    while time.monotonic()<deadline:
        with urllib.request.urlopen('http://127.0.0.1:8000/api/state',timeout=3) as r:j=json.load(r)
        out.write(json.dumps(j)+'\n');out.flush()
        a=j.get('arm_startup') or {}
        if a.get('owned') and not active:
            active=True;deadline=time.monotonic()+25
        if time.monotonic()-last>1:
            print(json.dumps({'phase':a.get('phase'),'reason':a.get('reason'),'error':a.get('max_error_rad'),'enabled':(j.get('arms') or {}).get('enabled'),'valid':sum((j.get('arms') or {}).get('valid',[]))}),flush=True)
            last=time.monotonic()
        time.sleep(.15)
print('Observation complete; arm holding state has not been changed.',flush=True)
