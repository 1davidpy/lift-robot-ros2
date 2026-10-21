"""Finite maintenance return to the recorded, initially unpowered pose."""
import json, time, urllib.request
from pathlib import Path
import rclpy
from luxi_interfaces.msg import ArmCommand, ArmState
from std_msgs.msg import String

root=Path('/home/sunrise/Desktop/LUXI_Lift_Robot/luxi_lift_ros2/lift_robot_ros2')
baseline=json.loads((root/'log/startup-pose-real-test.jsonl').open().readline())
target=baseline['arms']['position']
assert not baseline['arms']['enabled'] and sum(baseline['arms']['valid'])==16
live=json.load(urllib.request.urlopen('http://127.0.0.1:8000/api/state'))
assert not live['system']['services']['control'], 'Another controller is running'
config=json.loads((root/'Project/robot_bringup/config/arms-kh.json').read_text())
limits=config['right']['limits']+config['left']['limits']
rclpy.init();node=rclpy.create_node('openarm_maintenance_return')
feedback=None;applied=None
def on_state(m):
 global feedback
 feedback=m
def on_applied(m):
 global applied
 applied=m
node.create_subscription(ArmState,'/luxi/arms/state',on_state,5)
node.create_subscription(ArmCommand,'/luxi/applied',on_applied,5)
pub=node.create_publisher(ArmCommand,'/luxi/command',1)
life=node.create_publisher(String,'/luxi/driver_lifecycle',10)
start=time.monotonic()
while (feedback is None or applied is None or time.monotonic()-start<.5) and time.monotonic()-start<4:
 rclpy.spin_once(node,timeout_sec=.02)
assert feedback and applied and all(feedback.valid) and feedback.enabled
q=list(applied.position);nominal=list(q);bias=[0.]*16
life.publish(String(data='enable'))
previous=time.monotonic();deadline=previous+50;arrived=0;lastprint=0;failed=False
with (root/'log/startup-maintenance-return.jsonl').open('w') as log:
 while time.monotonic()<deadline:
  rclpy.spin_once(node,timeout_sec=.003)
  now=time.monotonic();dt=min(.03,now-previous)
  if dt<.018:continue
  previous=now
  fresh=feedback is not None and all(feedback.valid) and time.monotonic_ns()-feedback.receive_mono_ns<150000000
  if not fresh:
   failed=True;print('FEEDBACK LOST: stopping trajectory, preserving target',flush=True)
  lag=max(abs(a-b) for a,b in zip(q,feedback.position)) if fresh else 1
  if lag>.35:failed=True
  if not failed:
   for i in range(16):nominal[i]+=max(-.10*dt,min(.10*dt,target[i]-nominal[i]))
   done=max(abs(a-b) for a,b in zip(nominal,target))<.0001
   for i in range(16):
    if done and i%8!=7:bias[i]=max(-.2,min(.2,bias[i]+max(-.03*dt,min(.03*dt,.5*(target[i]-feedback.position[i])*dt))))
    desired=max(limits[i][0],min(limits[i][1],nominal[i]+bias[i]))
    q[i]+=max(-.10*dt,min(.10*dt,desired-q[i]))
  msg=ArmCommand();msg.source_mono_ns=time.monotonic_ns();msg.position=q;msg.phase='maintenance_return_hold' if failed else 'maintenance_return'
  pub.publish(msg)
  error=max(abs(a-b) for a,b in zip(target,feedback.position))
  log.write(json.dumps({'t':now,'position':list(feedback.position),'target':target,'command':q,'bias':bias,'error':error})+'\n')
  if now-lastprint>1:print(json.dumps({'error':error,'lag':lag,'failed':failed}),flush=True);lastprint=now
  if not failed and error<.02 and max(abs(v) for v in feedback.velocity)<.08:
   if not arrived:arrived=now
   if now-arrived>.7:
    life.publish(String(data='disable'))
    for _ in range(20):rclpy.spin_once(node,timeout_sec=.02)
    assert not feedback.enabled
    print('PASS: returned to the recorded initially unpowered pose, then disabled',flush=True)
    break
  else:arrived=0
 else:
  raise RuntimeError('Maintenance return did not converge; motors remain enabled at last target')
node.destroy_node();rclpy.shutdown()
