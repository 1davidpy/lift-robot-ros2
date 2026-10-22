import os
os.environ['ROS_LOCALHOST_ONLY']='1';os.environ['ROS_DOMAIN_ID']='0'
import rclpy,json,time,threading,http.client,sys,urllib.request,socket
from pathlib import Path
from std_msgs.msg import String
rclpy.init();node=rclpy.create_node('r680_response_measurement');rows=[];running=True
for topic in ['applied','state','status']:
 node.create_subscription(String,'/luxi/base/'+topic,lambda m,t=topic:rows.append(dict(t=time.monotonic(),topic=t,value=json.loads(m.data))),50)
def spin():
 while running:rclpy.spin_once(node,timeout_sec=.01)
th=threading.Thread(target=spin);th.start();conn=http.client.HTTPConnection('127.0.0.1',18031,timeout=2);calls=[];client='response-measurement';start=None;stop=None

def api(action,body):
 t=time.monotonic();payload={'command':'keyboard_start' if action=='acquire' else 'stop'}
 conn.request('POST','/api/keyboard',json.dumps(payload),{'Content-Type':'application/json'});res=conn.getresponse();data=json.loads(res.read());calls.append(dict(action=action,t=t,end=time.monotonic(),status=res.status,body=data));assert res.status==200,(action,res.status,data);return data

def relay():return json.load(urllib.request.urlopen('http://127.0.0.1:18031/api/state',timeout=2))
try:
 end=time.monotonic()+4
 while time.monotonic()<end:
  if sum(x['topic']=='state' for x in rows)>=5 and sum(x['topic']=='applied' for x in rows)>=10:break
  time.sleep(.05)
 s=relay();assert not s['base_status']['enabled'] and max(abs(x) for x in s['base']['velocity'])<.005
 ack=api('acquire',dict(client_id=client));assert ack['success'];udp=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);udp.setblocking(False)
 start=time.monotonic();seq=0
 while time.monotonic()-start<3:
  seq+=1;udp.sendto(json.dumps(dict(session_token=ack['session_token'],command_seq=seq,x=.2,y=0,angular_z=0)).encode(),('127.0.0.1',ack['udp_port']));time.sleep(.05)
  try:
   while True:udp.recv(2048)
  except BlockingIOError:pass
 stop=time.monotonic();api('release',dict(client_id=client));time.sleep(4)
finally:
 try:api('release',dict(client_id=client))
 except Exception:pass
 final=relay();running=False;th.join(2);node.destroy_node();rclpy.shutdown();conn.close()
 metrics={}
 if start:
  for label,topic,field,limit in [('first_can','applied','tx_mono_ns',.001),('can_90','applied','tx_mono_ns',.09),('motion_10','state','receive_mono_ns',.01),('motion_80','state','receive_mono_ns',.08)]:
   times=[int(x['value'][field])*1e-9 for x in rows if x['topic']==topic and int(x['value'][field])*1e-9>=start and x['value']['velocity'][0]>=limit]
   metrics[label+'_ms']=round((min(times)-start)*1000,1) if times else None
  states=[x for x in rows if x['topic']=='state' and int(x['value']['receive_mono_ns'])*1e-9>=start]
  metrics['peak_actual_vx']=max([x['value']['velocity'][0] for x in states],default=0)
  metrics['path']='robot_local_relay_no_wifi_or_atlas'
 report=dict(label=sys.argv[1],metrics=metrics,start=start,stop=stop,calls=calls,records=rows,final_enabled=final['base_status']['enabled'],final_velocity=final['base']['velocity'])
 p=Path('/home/sunrise/Desktop/LUXI_Lift_Robot/luxi_lift_ros2/lift_robot_ros2/.local/response-tuning-20261022')/(sys.argv[1]+'.json');p.write_text(json.dumps(report,indent=2));print(json.dumps({k:v for k,v in report.items() if k not in ('calls','records')},indent=2),flush=True)
 assert not final['base_status']['enabled'] and max(abs(x) for x in final['base']['velocity'])<.005
