#!/usr/bin/env python3
"""Real controller, isolated ROS domain, simulated arm feedback only."""
import unittest,time
import test_tracking_gap as fixture

class NetworkRecovery(unittest.TestCase):
    network_timeout=10.0
    setUp=fixture.TrackingGap.setUp
    cleanup=fixture.TrackingGap.cleanup
    step=fixture.TrackingGap.step
    run_for=fixture.TrackingGap.run_for

    def test_two_second_gap_recovers_only_from_new_converged_target(self):
        self.quest_on=False;self.intent_on=False
        old_stamp=time.monotonic_ns()
        self.run_for(.25);held=self.commands[-1]
        self.run_for(1.75)
        self.assertEqual(self.status.get('phase'),'teleop','2-second network outage latched a stop')
        self.assertTrue(self.status.get('input_paused'))
        self.assertLess(max(abs(a-b) for a,b in zip(held,self.commands[-1])),1e-8)
        self.quest_on=True;self.intent_on=True;self.intent_stamp=old_stamp;self.intent_offset=-.2
        self.run_for(.15)
        self.assertLess(max(abs(a-b) for a,b in zip(held,self.commands[-1])),1e-8,'replayed old IK')
        del self.intent_stamp
        self.intent_offset=.2;self.intent_phase='ik_unconverged';self.run_for(.15)
        self.assertLess(max(abs(a-b) for a,b in zip(held,self.commands[-1])),1e-8,'followed unconverged recovery IK')
        self.intent_phase='teleop';self.run_for(.3)
        self.assertEqual(self.status.get('phase'),'teleop')
        self.assertFalse(self.status.get('input_paused'))
        self.assertGreater(self.commands[-1][0]-held[0],.02)

    def test_gap_finishes_only_the_last_verified_goal_at_bounded_speed(self):
        self.intent_offset=.4;self.run_for(.04)
        self.quest_on=False;self.intent_on=False
        self.run_for(.2);start=self.commands[-1][0];begin=time.monotonic()
        self.run_for(.35)
        self.assertGreater(self.commands[-1][0],start+.015)
        self.assertLessEqual(self.commands[-1][0]-start,.2*(time.monotonic()-begin)+.015)
        self.run_for(2.)
        self.assertAlmostEqual(self.commands[-1][0],fixture.HOME[0]+.4,places=4)
        self.assertEqual(self.status.get('phase'),'teleop')

    def test_nine_second_gap_recovers(self):
        self.quest_on=False;self.intent_on=False;self.run_for(9.5)
        self.assertEqual(self.status.get('phase'),'teleop')
        self.quest_on=True;self.intent_on=True;self.intent_offset=.1;self.run_for(.3)
        self.assertEqual(self.status.get('phase'),'teleop')
        self.assertFalse(self.status.get('input_paused'))

    def test_over_ten_seconds_requires_explicit_restart(self):
        self.quest_on=False;self.intent_on=False;self.run_for(10.3)
        self.assertEqual(self.status.get('phase'),'hold')
        self.quest_on=True;self.intent_on=True;self.run_for(.3)
        self.assertEqual(self.status.get('phase'),'hold')

    def test_local_feedback_fault_during_network_gap_still_latches(self):
        self.quest_on=False;self.intent_on=False;self.feedback_on=False;self.run_for(.3)
        self.assertEqual(self.status.get('phase'),'fault')
        self.assertEqual(self.status.get('reason'),'feedback_unavailable')
        self.quest_on=True;self.intent_on=True;self.feedback_on=True;self.run_for(.3)
        self.assertEqual(self.status.get('phase'),'fault')

class LowRateStartup(unittest.TestCase):
    network_timeout=10.0
    quest_period=.05
    setUp=fixture.TrackingGap.setUp
    cleanup=fixture.TrackingGap.cleanup
    step=fixture.TrackingGap.step
    run_for=fixture.TrackingGap.run_for
    def test_startup_does_not_require_fifty_packets_per_second(self):
        self.assertEqual(self.status.get('phase'),'teleop')

if __name__=='__main__':
    fixture.rclpy.init()
    try:unittest.main(verbosity=2)
    finally:
        if fixture.rclpy.ok():fixture.rclpy.shutdown()
