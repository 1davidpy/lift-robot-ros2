#!/usr/bin/env python3
"""Dashboard absolute positioning against the actual adapter and PTY firmware."""
import json,time,unittest
import rclpy
from luxi_interfaces.srv import Command
from test_lift_uart_session import UartSession,args

class DashboardPosition(UartSession):
    def setUp(self):
        self.lease=False;self.owner='dashboard-'+str(time.monotonic_ns());self.seq=0;self.last_hb=0;self.hb=None
        super().setUp()
        self.assertTrue(self.status['dashboard_position'])
        self.assertFalse(self.status['shared_enable'])

    def extra_arguments(self):
        return ['-p','shared_enable:=true','-p','dashboard_position_control:=true','-p','host_min_um:=-180000']

    def detail(self,extra=None):
        self.seq+=1
        return json.dumps({'owner':self.owner,'sequence':self.seq,**(extra or {})})

    def enable_request(self):
        self.lease=True
        return 'position_arm',self.detail()

    def extra_step(self,now):
        if self.lease and now-self.last_hb>.065 and (self.hb is None or self.hb.done()):
            q=Command.Request();q.command='position_heartbeat';q.detail=self.detail()
            self.hb=self.client.call_async(q);self.last_hb=now

    def call(self,command,extra=None):
        q=Command.Request();q.command=command;q.detail=self.detail(extra)
        f=self.client.call_async(q);self.wait(f.done,2,command)
        return f.result()

    def arrive(self):
        self.position=self.target;self.moving=False;self.status_code=3;self.ready=True
        self.wait(lambda:self.status.get('external_result')=='reached',3,'confirmed locked arrival and READY')

    def test_exact_signed_endpoints(self):
        self.assertTrue(self.call('position_move',{'task_id':1,'target_um':180000}).success)
        self.wait(lambda:self.moving,4,'positive endpoint motion')
        self.assertEqual(self.moves,[180000]);self.arrive()
        self.assertTrue(self.call('position_move',{'task_id':2,'target_um':-180000}).success)
        self.wait(lambda:self.moving and len(self.moves)==2,4,'negative endpoint motion')
        self.assertEqual(self.moves,[180000,-180000]);self.arrive()

    def test_invalid_target_stops_and_cannot_replay(self):
        self.assertFalse(self.call('position_move',{'task_id':1,'target_um':180001}).success)
        self.lease=False;self.run_for(.25)
        self.assertEqual(self.moves,[]);self.assertFalse(self.status['enabled'])
        self.assertFalse(self.call('position_heartbeat').success)
        self.assertFalse(self.status['enabled'])

    def test_browser_lease_loss_closes_session(self):
        self.assertTrue(self.call('position_move',{'task_id':1,'target_um':10000}).success)
        self.wait(lambda:self.moving,4,'moving before disconnect')
        self.lease=False
        self.wait(lambda:not self.status['enabled'] and not self.nonce,1.6,'lease expiry stops motion')
        self.assertFalse(self.moving);self.assertFalse(self.status['dashboard_position'])
        self.assertEqual(self.status['reason'],'external_lease_expired')
        self.assertFalse(self.call('position_heartbeat').success)
        self.run_for(.3);self.assertFalse(self.status['enabled'])

    def test_half_second_gap_does_not_interrupt_motion(self):
        self.assertEqual(self.status['external_lease_ms'],1000)
        self.assertTrue(self.call('position_move',{'task_id':1,'target_um':10000}).success)
        self.wait(lambda:self.moving,4,'moving before jitter')
        self.lease=False;self.run_for(.55)
        self.assertTrue(self.status['enabled']);self.assertTrue(self.moving)
        self.lease=True;self.run_for(.25)
        self.assertTrue(self.status['enabled']);self.assertTrue(self.moving)
        self.assertGreater(self.status['external_max_gap_ms'],450)
        self.assertEqual(self.moves,[10000]);self.arrive()

    def test_delayed_duplicate_heartbeat_does_not_revoke_owner(self):
        self.lease=False
        q=Command.Request();q.command='position_heartbeat'
        q.detail=json.dumps({'owner':self.owner,'sequence':1})
        f=self.client.call_async(q);self.wait(f.done,1,'old heartbeat')
        self.assertTrue(f.result().success);self.assertTrue(self.status['enabled'])
        self.lease=True;self.run_for(.2);self.assertTrue(self.status['enabled'])

    def test_device_watchdog_still_stops_before_upper_lease(self):
        self.assertTrue(self.call('position_move',{'task_id':1,'target_um':10000}).success)
        self.wait(lambda:self.moving,4,'moving before lost device ACK')
        self.reply_heartbeat=False
        self.wait(lambda:not self.status['enabled'],.8,'device ACK watchdog unchanged')
        self.assertIn('heartbeat ACK timeout',self.status['reason'])

    def test_explicit_global_stop(self):
        self.assertTrue(self.call('position_move',{'task_id':1,'target_um':10000}).success)
        self.wait(lambda:self.prepare is not None,3,'queued motion')
        self.assertTrue(self.call('stop').success);self.lease=False
        self.run_for(.4)
        self.assertFalse(self.moving);self.assertFalse(self.nonce);self.assertFalse(self.status['enabled'])
        self.assertTrue(any(op==3 for op,seq,value in self.requests))

if __name__=='__main__':
    rclpy.init()
    suite=unittest.TestSuite(DashboardPosition(k) for k in DashboardPosition.__dict__ if k.startswith('test_'))
    try:result=unittest.TextTestRunner(verbosity=2).run(suite)
    finally:rclpy.shutdown()
    raise SystemExit(0 if result.wasSuccessful() else 1)
