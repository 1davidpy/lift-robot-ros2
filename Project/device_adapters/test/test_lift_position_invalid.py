#!/usr/bin/env python3
"""Five-snapshot position validity isolation using the actual adapter and PTY."""
import struct
import time
import unittest
import test_lift_uart_session as fixture
import test_lift_network_recovery as shared


class PositionInvalid(unittest.TestCase):
    setUp = fixture.UartSession.setUp
    cleanup = fixture.UartSession.cleanup
    ack = fixture.UartSession.ack
    stop = fixture.UartSession.stop
    commands = fixture.UartSession.commands
    step = fixture.UartSession.step
    run_for = fixture.UartSession.run_for
    wait = fixture.UartSession.wait
    enable_driver = shared.NetworkRecovery.enable_driver
    extra_step = shared.NetworkRecovery.extra_step

    def extra_arguments(self):
        self.position_valid = True
        self.inject_fault = False
        return shared.NetworkRecovery.extra_arguments(self)

    def flags(self):
        flags = fixture.UartSession.flags(self)
        return flags if self.position_valid else flags & ~1

    def send(self, kind, payload):
        if kind == 0x90 and payload[0] == 2 and self.inject_fault:
            payload = payload[:3] + struct.pack('<H', 1) + payload[5:]
        fixture.UartSession.send(self, kind, payload)

    def snapshot(self, valid):
        self.telemetry = False
        self.position_valid = valid
        self.sequence = (self.sequence + 1) & 65535
        uptime = int((time.monotonic() - self.started) * 1000)
        pages = [struct.pack('<Hih', self.sequence, self.position, 20 if self.moving else 0),
                 struct.pack('<HiBB', self.sequence, self.target, self.status_code, self.flags()),
                 struct.pack('<HHBBBB', self.sequence, 0, 1, 1, 0, 4),
                 struct.pack('<HIH', self.sequence, uptime, 60)]
        for page, payload in enumerate(pages):
            self.send(0x90, bytes([page]) + payload)
        self.run_for(.035)

    def invalidate_five(self):
        for count in range(1, 6):
            self.snapshot(False)
            self.assertEqual(self.status.get('position_invalid_count'), count)
            if count < 5:
                self.assertTrue(self.status['enabled'])
                self.assertFalse(self.status['position_invalid_paused'])
        self.assertFalse(self.status['enabled'])
        self.assertTrue(self.status['position_invalid_paused'])
        self.assertFalse(self.status['shared_failed'])

    def test_five_complete_snapshots_not_five_timer_ticks(self):
        self.snapshot(False)
        self.run_for(.12)
        self.assertEqual(self.status['position_invalid_count'], 1)
        self.assertTrue(self.status['enabled'])
        for count in range(2, 6):
            self.snapshot(False)
            self.assertEqual(self.status['position_invalid_count'], count)
            self.assertEqual(self.status['enabled'], count < 5)
        self.assertTrue(self.status['position_invalid_paused'])
        self.assertFalse(self.status['shared_failed'])

    def test_good_snapshot_resets_the_count(self):
        for _ in range(4):
            self.snapshot(False)
        self.snapshot(True)
        self.assertEqual(self.status['position_invalid_count'], 0)
        self.snapshot(False)
        self.assertEqual(self.status['position_invalid_count'], 1)
        self.assertTrue(self.status['enabled'])

    def test_parking_keeps_position_pause_visible_to_the_coordinator(self):
        self.invalidate_five()
        self.network_phase = 'stop'
        self.run_for(.08)
        self.assertTrue(self.status['position_invalid_paused'])
        self.assertEqual(self.status['position_invalid_pause_count'], 5)
        self.assertFalse(self.status['shared_failed'])
        self.snapshot(False)
        self.assertTrue(self.status['position_invalid_stationary'])

    def test_recovery_does_not_replay_a_held_grip(self):
        self.quest.grips = [0., 1.]
        self.wait(lambda: self.moving, 4, 'initial grip motion')
        self.invalidate_five()
        self.position_valid = True
        self.telemetry = True
        previous_moves = len(self.moves)
        self.run_for(.4)
        self.assertFalse(self.status['enabled'])
        self.assertFalse(self.moving)
        self.assertEqual(len(self.moves), previous_moves)
        self.quest.grips = [0., 0.]
        self.wait(lambda: self.status.get('enabled') and self.status.get('phase') == 'idle',
                  3, 'valid feedback and released grips reconfigure without moving')
        self.run_for(.2)
        self.assertEqual(len(self.moves), previous_moves)
        self.quest.grips = [1., 0.]
        self.wait(lambda: len(self.moves) > previous_moves, 4, 'new grip after recovery')

    def test_feedback_timeout_is_not_debounced(self):
        self.snapshot(False)
        self.run_for(.4)
        self.assertFalse(self.status['enabled'])
        self.assertTrue(self.status['shared_failed'])
        self.assertIn('feedback_timeout', self.status['reason'])

    def test_motor_fault_is_not_isolated(self):
        self.inject_fault = True
        self.snapshot(False)
        self.assertFalse(self.status['enabled'])
        self.assertTrue(self.status['shared_failed'])
        self.assertEqual(self.status['reason'], 'drive_fault')


if __name__ == '__main__':
    fixture.rclpy.init()
    try:
        unittest.main(argv=[__file__] + fixture.remaining, verbosity=2)
    finally:
        fixture.rclpy.shutdown()
