"""Real-node regression: telemetry must not run at the motor-feedback rate."""
import os,sys,time,json,subprocess,tempfile,signal
os.environ['ROS_DOMAIN_ID']='94';os.environ['ROS_LOCALHOST_ONLY']='1'
import rclpy
from luxi_interfaces.msg import ArmState
from std_msgs.msg import String
from rclpy.qos import QoSProfile,ReliabilityPolicy
adapter,manager,config=sys.argv[1:]
rclpy.init();n=rclpy.create_node('status_rate_test');counts={};active=False;observed={}
q=QoSProfile(depth=100,reliability=ReliabilityPolicy.BEST_EFFORT)
def cb(k):
 def f(m):
  if active:counts[k]=counts.get(k,0)+1
  if k=='feedback' and m.enabled:observed.setdefault('enabled_ns',m.receive_mono_ns)
  if k=='driver_status':
   j=json.loads(m.data);observed['last_report']=int(j['receive_mono_ns'])
   if j.get('output_enable_count',0)>0:observed.setdefault('enabled_report_ns',int(j['receive_mono_ns']))
 return f
subs=[n.create_subscription(ArmState,'/luxi/arms/state',cb('feedback'),q),n.create_subscription(String,'/luxi/arms/startup_status',cb('driver_status'),q),n.create_subscription(String,'/luxi/control_status',cb('control_status'),q),n.create_subscription(String,'/luxi/coordinated_control',cb('coordination'),q)]
lifecycle=n.create_publisher(String,'/luxi/driver_lifecycle',10)
with tempfile.TemporaryDirectory() as d:
 with open(d+'/nodes.log','w') as log:
  env=dict(os.environ,ROS_LOG_DIR=d)
  ps=[subprocess.Popen([adapter,'--ros-args','-p','mock:=true','-p','config:='+config],env=env,stdout=log,stderr=log),subprocess.Popen([manager,'--ros-args','-p','shared_enable:=true','-p','config:='+config],env=env,stdout=log,stderr=log)]
  try:
   start=time.monotonic()
   while time.monotonic()-start<3:rclpy.spin_once(n,timeout_sec=.02)
   def cpu(p):
    from pathlib import Path
    values=Path('/proc/'+str(p.pid)+'/stat').read_text().split()
    return (int(values[13])+int(values[14]))/os.sysconf('SC_CLK_TCK')
   before=[cpu(p) for p in ps]
   active=True;start=time.monotonic()
   while time.monotonic()-start<5:
    assert all(p.poll() is None for p in ps),'Node exited'
    rclpy.spin_once(n,timeout_sec=.02)
   print(json.dumps({'cpu_percent':{k:100*(cpu(p)-c)/(time.monotonic()-start) for k,p,c in zip(['adapter','manager'],ps,before)}}),flush=True)
   rates={k:v/(time.monotonic()-start) for k,v in counts.items()};print(json.dumps(rates),flush=True)
   assert rates.get('feedback',0)>70,'Motor feedback rate regressed'
   assert rates.get('coordination',0)>70,'Coordination heartbeat rate regressed'
   for k in ['driver_status','control_status']:assert 10<rates.get(k,0)<25,(k,'redundant status must be limited to 20 Hz',rates)
   # Immediately after a routine report, enable only this isolated mock driver.
   # Its new ownership/enabled state must bypass the periodic telemetry gate.
   previous=observed.get('last_report');deadline=time.monotonic()+1
   while observed.get('last_report')==previous and time.monotonic()<deadline:rclpy.spin_once(n,timeout_sec=.01)
   message=String();message.data='enable';lifecycle.publish(message)
   deadline=time.monotonic()+1
   while not {'enabled_ns','enabled_report_ns'} <= observed.keys() and time.monotonic()<deadline:rclpy.spin_once(n,timeout_sec=.01)
   assert {'enabled_ns','enabled_report_ns'} <= observed.keys(),'Mock enable transition missing'
   delay=(observed['enabled_report_ns']-observed['enabled_ns'])/1e6
   print(json.dumps({'enable_transition_report_delay_ms':delay}),flush=True)
   assert 0<=delay<30,'State transition was delayed by the telemetry gate'

  finally:
   for p in ps:
    if p.poll() is None:p.send_signal(signal.SIGINT)
   for p in ps:
    try:p.wait(timeout=5)
    except subprocess.TimeoutExpired:p.kill();p.wait()
n.destroy_node();rclpy.shutdown()
