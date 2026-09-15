#!/usr/bin/env python3
"""Replay a captured trapped IK state; no real actuator topics or devices."""
import json,os,signal,subprocess,sys,tempfile,time,unittest
from pathlib import Path
os.environ['ROS_DOMAIN_ID']='97';os.environ['ROS_LOCALHOST_ONLY']='1'
import rclpy
from rclpy.qos import QoSProfile,DurabilityPolicy
from luxi_interfaces.msg import ArmCommand,ArmState,ArmTargets
from std_msgs.msg import String
BINARY,CONFIG,MODEL=[Path(sys.argv.pop(1)) for _ in range(3)]
BAD=[-1.3,2.,-1.3928943459,0.,-1.471820804,-.0282164114,1.04367534,-.4,
     1.396263,.174533,-1.08403948,0.,-.99625419,-.785398,1.570796,.4]
GOALS=[[.3073478341,-.1078745946,-.1879209578,.4969194531,.0100975269,-.8648930788,-.0702068582],
       [.3083054125,.1746967584,-.1936238259,-.5302548409,-.0573516339,.8400858641,-.0989764258]]

class IkSession(unittest.TestCase):
 def test_homing_clears_trapped_reference_and_disabled_does_not_advance(self):
  with tempfile.TemporaryDirectory(prefix='luxi-ik-session-') as tmp:
   prefix='/ik_session_'+str(time.monotonic_ns())
   args=[str(BINARY),'--ros-args','-p','config:='+str(CONFIG),'-p','model:='+str(MODEL)]
   for topic in ['arms/state','targets','intent','control_status']:
    args+=['-r',f'/luxi/{topic}:={prefix}/{topic}']
   with open(Path(tmp)/'motion.log','w+') as log:
    process=subprocess.Popen(args,stdout=log,stderr=subprocess.STDOUT,env=dict(os.environ,ROS_LOG_DIR=tmp))
    node=rclpy.create_node('ik_session_test')
    sp=node.create_publisher(ArmState,prefix+'/arms/state',1)
    tp=node.create_publisher(ArmTargets,prefix+'/targets',1)
    cp=node.create_publisher(String,prefix+'/control_status',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    received=[];phase=None
    node.create_subscription(ArmCommand,prefix+'/intent',lambda m:received.append(m),10)
    def run(seconds):
     end=time.monotonic()+seconds
     while time.monotonic()<end:
      now=time.monotonic_ns();state=ArmState();state.position=BAD.copy();state.valid=[True]*16;state.receive_mono_ns=now;sp.publish(state)
      target=ArmTargets();target.valid=[True,True];target.receive_mono_ns=now;target.grippers=[-.4,.4]
      for p,g in zip([target.right,target.left],GOALS):
       p.position.x,p.position.y,p.position.z=g[:3];p.orientation.w,p.orientation.x,p.orientation.y,p.orientation.z=g[3:]
      tp.publish(target)
      if phase is not None:
       status=String();status.data=json.dumps({'phase':phase});cp.publish(status)
      rclpy.spin_once(node,timeout_sec=.001);time.sleep(.005)
    try:
     run(2)
     self.assertGreater(len(received),15)
     self.assertEqual(received[-1].phase,'ik_unconverged','Trapped joint-limit pose was advertised as a usable IK solution')
     phase='homing';run(3)
     self.assertEqual(received[-1].phase,'teleop','A new homing cycle retained the trapped IK branch')
     self.assertGreater(received[-1].position[3],.8)
     self.assertGreater(received[-1].position[11],.8)
     phase='disabled';run(.15);count=len(received);run(.2)
     self.assertEqual(len(received),count,'Disabled session kept integrating controller poses')
     phase='homing';run(2)
     self.assertEqual(received[-1].phase,'teleop')
     self.assertGreater(received[-1].position[3],.8)
    finally:
     node.destroy_node()
     if process.poll() is None:process.send_signal(signal.SIGINT)
     try:process.wait(timeout=3)
     except subprocess.TimeoutExpired:process.kill();process.wait()

if __name__=='__main__':
 rclpy.init()
 try:unittest.main(verbosity=2)
 finally:
  if rclpy.ok():rclpy.shutdown()
