#!/usr/bin/env python3
"""Direct ROS external-control client through the real adapter and PTY UART."""
import json,time,unittest
import rclpy
from std_msgs.msg import String
from test_lift_uart_session import UartSession,args

class ExternalLease(UartSession):
    def setUp(self):
        self.lease=False;self.owner='sdk-'+str(time.monotonic_ns());self.seq=0;self.last_hb=0
        super().setUp()

    def extra_arguments(self):
        return ['-p','external_control:=true']

    def publish(self,op,extra=None):
        self.seq+=1
        self.external.publish(String(data=json.dumps({'owner':self.owner,'sequence':self.seq,
            'source_mono_ns':str(time.monotonic_ns()),'op':op,**(extra or {})})))

    def enable_driver(self):
        self.external=self.node.create_publisher(String,'/luxi/lift/external_command',10)
        self.run_for(.3)
        self.publish('arm');self.lease=True

    def extra_step(self,now):
        if self.lease and now-self.last_hb>.065:
            self.publish('heartbeat');self.last_hb=now

    def test_jitter_then_disconnect_never_replays(self):
        self.assertEqual(self.status['external_lease_ms'],1000)
        self.publish('move',{'task_id':1,'target_um':10000})
        self.wait(lambda:self.moving,4,'direct ROS move via UART')
        self.lease=False;self.run_for(.6)
        self.assertTrue(self.status['enabled']);self.assertTrue(self.moving)
        self.lease=True;self.run_for(.3)
        self.assertTrue(self.status['enabled']);self.assertGreater(self.status['external_max_gap_ms'],500)
        self.lease=False
        self.wait(lambda:not self.status['enabled'] and not self.nonce,1.6,'SDK disconnect')
        self.assertEqual(self.status['reason'],'external_lease_expired')
        self.lease=True;self.run_for(.35)
        self.assertFalse(self.status['enabled']);self.assertEqual(self.moves,[10000])

    def test_sdk_stop_without_waiting_for_lease(self):
        self.publish('move',{'task_id':1,'target_um':10000})
        self.wait(lambda:self.moving,4,'SDK move')
        self.publish('stop');self.lease=False
        self.wait(lambda:not self.status['enabled'] and not self.nonce,.35,'explicit stop without lease wait')
        self.assertFalse(self.moving)

    def test_delayed_heartbeat_does_not_extend_lease(self):
        self.lease=False
        stale={'owner':self.owner,'sequence':1,'source_mono_ns':str(time.monotonic_ns()),'op':'heartbeat'}
        self.run_for(.45)
        self.external.publish(String(data=json.dumps(stale)));self.run_for(.15)
        self.assertTrue(self.status['enabled'])
        self.assertGreater(self.status['external_heartbeat_age_ms'],450)
        self.wait(lambda:not self.status['enabled'],.65,'old SDK heartbeat did not extend lease')

if __name__=='__main__':
    rclpy.init()
    suite=unittest.TestSuite(ExternalLease(k) for k in ExternalLease.__dict__ if k.startswith('test_'))
    try:result=unittest.TextTestRunner(verbosity=2).run(suite)
    finally:rclpy.shutdown()
    raise SystemExit(0 if result.wasSuccessful() else 1)
