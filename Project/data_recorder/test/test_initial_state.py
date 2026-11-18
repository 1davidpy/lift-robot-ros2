#!/usr/bin/env python3
"""Exercise A-triggered snapshots in an isolated ROS domain, without hardware."""
import hashlib,json,os,signal,subprocess,sys,tempfile,time
from pathlib import Path
os.environ['ROS_DOMAIN_ID']='89'
os.environ['ROS_LOCALHOST_ONLY']='1'
import rclpy
from luxi_interfaces.msg import ArmState,QuestInput
from std_msgs.msg import String

binary=str(Path(sys.argv[1]).resolve())
rclpy.init()
node=rclpy.create_node('initial_state_test')
arms=node.create_publisher(ArmState,'/luxi/arms/state',10)
lift=node.create_publisher(String,'/luxi/lift/state',10)
quest=node.create_publisher(QuestInput,'/luxi/quest',10)
position=[.01*i for i in range(16)]
mode='missing';button=False;stale_ns=time.monotonic_ns()-2_000_000_000

def step():
 now=time.monotonic_ns()
 s=ArmState();s.receive_mono_ns=now;s.enabled=True;s.valid=[True]*16;s.position=position;s.phase='teleop';arms.publish(s)
 if mode!='missing':
  t=stale_ns if mode=='stale' else now
  body={'current_um':-64900,'receive_mono_ns':str(t),'receive_first_mono_ns':str(t),
        'feedback_age_ms':0,'flags':0 if mode=='invalid' else 1,'fault':0,'valid':mode!='invalid'}
  m=String();m.data='invalid JSON' if mode=='malformed' else json.dumps(body);lift.publish(m)
 q=QuestInput();q.receive_mono_ns=now;q.buttons=[button,False,False,False];quest.publish(q)
 rclpy.spin_once(node,timeout_sec=.005);time.sleep(.01)

def wait_for(check,seconds=4):
 end=time.monotonic()+seconds
 while time.monotonic()<end:
  step()
  if check():return
 raise AssertionError('timeout')

def settle(seconds=.25):
 end=time.monotonic()+seconds
 while time.monotonic()<end:step()

try:
 with tempfile.TemporaryDirectory(prefix='luxi-initial-state-') as tmp:
  root=Path(tmp)/'episodes';root.mkdir()
  with open(Path(tmp)/'recorder.log','w+') as log:
   p=subprocess.Popen([binary,'--ros-args','-p','directory:='+str(root),'-p','require_cameras:=false','-p','minimum_free_gib:=0.0'],stdout=log,stderr=subprocess.STDOUT,env=dict(os.environ,ROS_LOG_DIR=tmp))
   try:
    wait_for(lambda:quest.get_subscription_count()>0 and arms.get_subscription_count()>0,10)
    for mode in ['missing','fresh','stale','invalid','malformed']:
     button=False;position=[.01*i for i in range(16)];settle()
     button=True
     wait_for(lambda:len(list(root.glob('.partial-episode-*/initial_state.json')))==1)
     path=next(root.glob('.partial-episode-*/initial_state.json'));original=path.read_bytes();snap=json.loads(original)
     assert snap['trigger']=='quest_a' and snap['arms']['position']==position
     assert snap['arms']['right']==position[:8] and snap['arms']['left']==position[8:]
     assert snap['arms']['valid'] and int(snap['request_mono_ns'])>0
     assert snap['valid']==(mode=='fresh'),(mode,snap)
     if mode=='fresh':assert abs(snap['lift']['height_mm']+64.9)<1e-9
     if mode=='missing':assert snap['lift']['height_mm'] is None and not snap['lift']['present']
     if mode=='stale':assert not snap['lift']['fresh']
     if mode=='malformed':assert snap['lift']['reason']=='malformed_feedback'
     button=False;position=[.5]*16;settle()
     assert path.read_bytes()==original,'snapshot overwritten during episode'
     before=len(list(root.glob('episode-*')));button=True
     wait_for(lambda:len(list(root.glob('episode-*')))==before+1)
     final=root/path.parent.name.removeprefix('.partial-')
     assert (final/'initial_state.json').read_bytes()==original
     manifest=json.loads((final/'manifest.json').read_text());entry=next(f for f in manifest['files'] if f['path']=='initial_state.json')
     assert entry['sha256']==hashlib.sha256(original).hexdigest() and entry['bytes']==len(original)
     assert json.loads((final/'session.json').read_text())['initial_state_file']=='initial_state.json'
     print('PASS',mode,'A start/finish, immutable snapshot and manifest hash',flush=True)
    assert p.poll() is None
   except Exception:
    log.flush();log.seek(0);print(log.read());raise
   finally:
    p.send_signal(signal.SIGINT)
    try:p.wait(timeout=8)
    except subprocess.TimeoutExpired:p.kill();p.wait()
finally:
 node.destroy_node();rclpy.shutdown()
