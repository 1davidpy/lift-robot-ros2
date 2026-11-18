#!/usr/bin/env python3
"""Actual shared controller with remapped mock devices, never hardware."""
import unittest
import test_shared_parking as fixture


class LiftPositionIsolation(unittest.TestCase):
    cleanup = fixture.SharedParking.cleanup
    on_directive = fixture.SharedParking.on_directive
    on_command = fixture.SharedParking.on_command
    on_lifecycle = fixture.SharedParking.on_lifecycle
    publish_loop = fixture.SharedParking.publish_loop
    publish_inputs = fixture.SharedParking.publish_inputs
    step = fixture.SharedParking.step
    run_for = fixture.SharedParking.run_for
    wait_for = fixture.SharedParking.wait_for
    request_park = fixture.SharedParking.request_park

    def setUp(self):
        self.simulate_active_devices = True
        self.position_pause = self._testMethodName == 'test_start_with_position_paused_lift'
        self.extra_lift_status = {}
        self.intent_offset = 0.
        fixture.SharedParking.setUp(self)
        self.position = fixture.HOME.copy()
        self.active_intent_pub = self.node.create_publisher(
            fixture.ArmCommand, self.prefix+'/intent', 1)
        self.run_for(.3)
        request = fixture.Command.Request()
        request.command = 'start'
        future = self.client.call_async(request)
        self.wait_for(future.done, 2, 'explicit start')
        self.assertTrue(future.result().success, future.result().message)
        self.wait_for(lambda: self.status.get('system_phase') == 'active',
                      5, 'shared teleop active')

    def test_position_pause_preserves_arm_and_base_teleoperation(self):
        self.position_pause = True
        self.run_for(.3)
        self.assertEqual(self.status['system_phase'], 'active')
        self.assertTrue(self.status['lift_position_paused'])
        self.assertEqual(self.directive['phase'], 'active')
        self.intent_offset = .1
        self.run_for(.5)
        self.assertGreater(self.position[0], fixture.HOME[0]+.05)
        self.assertTrue(self.enabled)

    def test_start_with_position_paused_lift(self):
        self.assertTrue(self.status['lift_position_paused'])
        self.assertEqual(self.status['phase'], 'teleop')

    def test_drive_fault_still_stops_all_devices(self):
        self.position_pause = True
        self.extra_lift_status = {'fault': 1}
        self.wait_for(lambda: self.status.get('system_phase') == 'stopped',
                      2, 'drive fault remains global stop')

    def test_feedback_timeout_still_stops_all_devices(self):
        self.position_pause = True
        self.lift_sample_age_ns = 350000000
        self.wait_for(lambda: self.status.get('system_phase') == 'stopped',
                      2, 'transport/feedback deadline remains global stop')

    def test_b_still_stops_all_devices(self):
        self.position_pause = True
        self.run_for(.2)
        self.quest.buttons[1] = True
        self.wait_for(lambda: self.status.get('system_phase') == 'stopped',
                      2, 'B always stops')

    def test_arm_parking_still_works_while_lift_position_is_paused(self):
        self.position_pause = True
        self.run_for(.2)
        self.request_park()
        self.wait_for(lambda: not self.enabled, 6, 'park without commanding lift motion')
        self.assertNotIn('home', self.directives)


if __name__ == '__main__':
    fixture.rclpy.init()
    try:
        unittest.main(verbosity=2)
    finally:
        fixture.rclpy.shutdown()
