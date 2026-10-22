"""Real SDK adapter and relay, simulated CAN, isolated ROS domain 178."""
import os
os.environ['ROS_DOMAIN_ID']='178'
os.environ['ROS_LOCALHOST_ONLY']='1'
import json, socket, subprocess, time, urllib.request, signal
from pathlib import Path
import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from luxi_interfaces.srv import Command

root=Path('/home/sunrise/Desktop/LUXI_Lift_Robot/luxi_lift_ros2/lift_robot_ros2')
out=root/'.local/atlas-navigation-20261022'
rclpy.init(); node=rclpy.create_node('r680_navigation_acceptance')
applied=[]; statuses=[]; twists=[]
node.create_subscription(String,'/luxi/base/applied',lambda m:applied.append((time.monotonic(),json.loads(m.data))),20)
node.create_subscription(String,'/luxi/base/status',lambda m:statuses.append(json.loads(m.data)),10)
node.create_subscription(Twist,'/cmd_vel',lambda m:twists.append((m.linear.x,m.linear.y,m.angular.z)),10)
pub=node.create_publisher(Twist,'/cmd_vel',1)
client=node.create_client(Command,'/luxi/base/control')
processes=[]; results={}
def spin(seconds,velocity=None):
 end=time.monotonic()+seconds;next_pub=0
 while time.monotonic()<end:
  if velocity is not None and time.monotonic()>=next_pub:
   m=Twist();m.linear.x,m.linear.y,m.angular.z=velocity;pub.publish(m);next_pub=time.monotonic()+.04
  rclpy.spin_once(node,timeout_sec=.01)
def command(name,detail=''):
 req=Command.Request();req.command=name;req.detail=detail
 f=client.call_async(req);end=time.monotonic()+2
 while not f.done() and time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.01)
 assert f.done() and f.result().success,(name,f.result() if f.done() else 'timeout')
def near(v,w):return all(abs(a-b)<.002 for a,b in zip(v,w))
opener=urllib.request.build_opener(urllib.request.ProxyHandler({}))
def http(path,body=None):
 req=urllib.request.Request('http://127.0.0.1:18051'+path,None if body is None else json.dumps(body).encode(),{'Content-Type':'application/json'})
 with opener.open(req,timeout=3) as res:return json.load(res)
try:
 log=open(out/'mock-sdk-navigation.log','w')
 p=subprocess.Popen([str(root/'build/packages/luxi_device_adapters/r680_adapter'),'--ros-args','-p','mock:=true','-p','require_left_grip:=false','-p','max_linear:=0.5'],stdout=log,stderr=subprocess.STDOUT)
 processes.append(p);spin(2)
 assert client.service_is_ready() and statuses and statuses[-1]['mock'] is True
 spin(.4,(.02,.01,.04));assert not statuses[-1]['enabled'];results['disarmed_twist_ignored']=True
 command('navigation_start');spin(.7,(.02,.01,.04))
 assert any(near(a['velocity'],(.02,.01,.04)) for _,a in applied[-20:]);results['three_axis_cmd_vel']=True
 dropped=time.monotonic();spin(.4)
 assert not statuses[-1]['enabled'] and near(applied[-1][1]['velocity'],(0,0,0))
 zeros=[t for t,a in applied if t>dropped and near(a['velocity'],(0,0,0))]
 assert zeros and zeros[0]-dropped<.35;results['sdk_input_timeout_ms']=round((zeros[0]-dropped)*1000,1)
 spin(.3,(.02,0.,.03));assert not statuses[-1]['enabled'];results['no_automatic_rearm']=True
 command('navigation_start');spin(.15,(0.,0.,.11));spin(.15)
 assert not statuses[-1]['enabled'];results['overspeed_disarms']=True
 command('keyboard_start')
 response_start=time.monotonic()
 for _ in range(12):command('keyboard','[0.2,0]');spin(.04)
 assert any(near(a['velocity'],(.10,0,0)) for _,a in applied[-12:]);command('stop');spin(.15)
 results['legacy_two_axis_keyboard']=True
 reached=[t for t,a in applied if t>=response_start and near(a['velocity'],(.1,0,0))]
 assert reached and reached[0]-response_start<.18
 results['manual_0_1_ramp_ms']=round((reached[0]-response_start)*1000,1)
 command('keyboard_start')
 for _ in range(24):command('keyboard','[0,0,1]');spin(.04)
 assert any(near(a['velocity'],(0,0,.3)) for _,a in applied[-20:]);command('stop');spin(.15)
 results['manual_in_place_turn']=True
 command('keyboard_start')
 for _ in range(50):command('keyboard','[1,0,0]');spin(.04)
 assert any(near(a['velocity'],(.5,0,0)) for _,a in applied[-20:]);command('stop');spin(.15)
 results['manual_half_metre_per_second_mock_only']=True
 relay=subprocess.Popen([str(out/'relay-build/carrier_relay'),'8000','18051'],stdout=log,stderr=subprocess.STDOUT)
 processes.append(relay);spin(2)
 state=http('/api/state');assert state['navigation_cmd_vel'] and state['base']['mock']
 ack=http('/api/keyboard',{'command':'navigation_start'});assert ack['success']
 udp=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);udp.settimeout(.4)
 for seq in range(1,15):
  data=dict(session_token=ack['session_token'],command_seq=seq,x=.04,y=.02,angular_z=-.04)
  udp.sendto(json.dumps(data).encode(),('127.0.0.1',ack['udp_port']))
  reply=json.loads(udp.recv(2048));assert reply['success'];spin(.04)
 assert any(near(v,(.02,.01,-.04)) for v in twists[-30:]);results['relay_preserves_full_twist']=True
 assert any(near(a['velocity'],(.02,.01,-.04)) for _,a in applied[-20:]);results['relay_to_sdk_feedback']=True
 # Replayed command cannot refresh the velocity lease.
 start=time.monotonic()
 for _ in range(12):udp.sendto(json.dumps(data).encode(),('127.0.0.1',ack['udp_port']));spin(.04)
 assert near(applied[-1][1]['velocity'],(0,0,0));results['replay_does_not_refresh_motion']=True
 spin(.85);assert not http('/api/relay/status')['armed'];assert not statuses[-1]['enabled']
 results['relay_owner_timeout_disarms']=True
 udp.close()
 print(json.dumps(results,indent=2));(out/'cmd-vel-tests.json').write_text(json.dumps(results,indent=2))
finally:
 for p in reversed(processes):
  p.send_signal(signal.SIGINT)
  try:p.wait(timeout=5)
  except subprocess.TimeoutExpired:p.kill();p.wait()
 node.destroy_node();rclpy.shutdown()

