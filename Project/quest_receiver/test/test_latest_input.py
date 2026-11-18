#!/usr/bin/env python3
"""Exercise actual UDP receiver on loopback, isolated from the robot domain."""
import os,sys,time,json,socket,subprocess,signal,tempfile,unittest
os.environ['ROS_DOMAIN_ID']='96';os.environ['ROS_LOCALHOST_ONLY']='1'
import rclpy
from luxi_interfaces.msg import QuestInput
BINARY=sys.argv.pop(1)
class LatestInput(unittest.TestCase):
 def setUp(self):
  self.tmp=tempfile.TemporaryDirectory();self.log=open(self.tmp.name+'/receiver.log','w+')
  self.socket=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);self.socket.bind(('127.0.0.1',0));self.port=self.socket.getsockname()[1];self.socket.close();self.socket=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
  self.topic='/receiver_test_'+str(time.monotonic_ns());self.node=rclpy.create_node('receiver_test');self.messages=[]
  self.node.create_subscription(QuestInput,self.topic,self.messages.append,10)
  self.process=subprocess.Popen([BINARY,'--ros-args','-p','host:=127.0.0.1','-p','port:='+str(self.port),'-r','/luxi/quest:='+self.topic],stdout=self.log,stderr=self.log,env=dict(os.environ,ROS_LOG_DIR=self.tmp.name))
  self.addCleanup(self.cleanup)
  self.spin(1.3);self.send(1);self.spin(.1);self.assertTrue(self.messages);self.messages.clear()
 def send(self,t,b=False):
  self.socket.sendto(json.dumps({'t':t,'lt':0,'rt':0,'lc':{'x':0,'y':1,'z':0,'qw':1,'qx':0,'qy':0,'qz':0},'rc':{'x':0,'y':1,'z':0,'qw':1,'qx':0,'qy':0,'qz':0},'b':b}).encode(),('127.0.0.1',self.port))
 def spin(self,seconds):
  end=time.monotonic()+seconds
  while time.monotonic()<end:rclpy.spin_once(self.node,timeout_sec=.005)
 def cleanup(self):
  self.process.send_signal(signal.SIGCONT);self.process.send_signal(signal.SIGINT);self.process.wait(timeout=3);self.node.destroy_node();self.socket.close();self.log.close();self.tmp.cleanup()
 def test_burst_only_latest_preserves_stop(self):
  self.process.send_signal(signal.SIGSTOP);time.sleep(.02)
  for i in range(2,42):self.send(i,i==10)
  self.process.send_signal(signal.SIGCONT);self.spin(.15)
  self.assertEqual([m.sender_time for m in self.messages],[41]);self.assertTrue(self.messages[-1].buttons[1])
 def test_socket_backlog_does_not_get_fresh_timestamp(self):
  self.process.send_signal(signal.SIGSTOP);time.sleep(.02);self.send(2);time.sleep(.22);self.process.send_signal(signal.SIGCONT);self.spin(.1)
  self.assertFalse(self.messages);self.send(3);self.spin(.1);self.assertEqual(self.messages[-1].sender_time,3)
 def test_reordered_packet_after_two_second_gap_is_not_new_session(self):
  self.send(20);self.spin(.1);self.messages.clear();self.spin(2);self.send(2);self.spin(.1);self.assertFalse(self.messages)
  self.send(21);self.spin(.1);self.assertEqual(self.messages[-1].sender_time,21)
if __name__=='__main__':
 rclpy.init()
 try:unittest.main(verbosity=2)
 finally:rclpy.shutdown()
