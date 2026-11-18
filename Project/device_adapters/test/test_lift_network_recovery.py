#!/usr/bin/env python3
"""Shared lift network recovery through the real adapter and PTY firmware only."""
import time,json,unittest
import test_lift_uart_session as fixture
class NetworkRecovery(unittest.TestCase):
 setUp=fixture.UartSession.setUp
 cleanup=fixture.UartSession.cleanup
 flags=fixture.UartSession.flags
 send=fixture.UartSession.send
 ack=fixture.UartSession.ack
 stop=fixture.UartSession.stop
 commands=fixture.UartSession.commands
 step=fixture.UartSession.step
 run_for=fixture.UartSession.run_for
 wait=fixture.UartSession.wait
 def extra_arguments(self):
  self.sending=True;self.network_phase=None
  self.network_pub=self.node.create_publisher(fixture.String,self.prefix+'/coordinated_control',1)
  return ['-p','shared_enable:=true','-p','network_recovery_timeout:=10.0']
 def enable_driver(self):
  self.network_phase='prepare'
  self.wait(lambda:self.status.get('shared_ready'),4,'prepare ready')
  self.network_phase='active'
  self.wait(lambda:self.status.get('enabled') and self.status.get('shared_phase')=='active',2,'active')
 def extra_step(self,now):
  if self.network_phase is None:return
  message=fixture.String();message.data=json.dumps({'epoch':'network-test:1','phase':self.network_phase,'sent_mono_ns':str(time.monotonic_ns())});self.network_pub.publish(message)
 def test_recovers_nine_seconds_without_replaying_held_grip(self):
  self.quest.grips=[0.,1.];self.wait(lambda:self.moving,4,'moving');self.sending=False;self.run_for(9.3)
  self.assertFalse(self.moving);self.assertTrue(self.status['enabled']);old=len(self.moves)
  self.sending=True;self.run_for(.3);self.assertEqual(len(self.moves),old)
  self.quest.grips=[0.,0.];self.run_for(.2);self.quest.grips=[1.,0.]
  self.wait(lambda:len(self.moves)>old,4,'new grip after recovery')
 def test_ten_second_expiry_and_hardware_fault_remain_latched(self):
  self.sending=False;self.run_for(10.3);self.assertFalse(self.status['enabled'])
  self.sending=True;self.run_for(.3);self.assertFalse(self.status['enabled'])
 def test_local_feedback_fault_is_not_given_ten_seconds(self):
  self.sending=False;self.telemetry=False;self.run_for(.45);self.assertFalse(self.status['enabled'])
  self.sending=True;self.telemetry=True;self.run_for(.3);self.assertFalse(self.status['enabled'])
if __name__=='__main__':
 fixture.rclpy.init()
 try:unittest.main(argv=[__file__]+fixture.remaining,verbosity=2)
 finally:fixture.rclpy.shutdown()
