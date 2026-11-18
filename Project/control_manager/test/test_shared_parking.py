#!/usr/bin/env python3
"""Actual shared coordinator with fake, remapped devices; never opens CAN."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import threading
import unittest

os.environ['ROS_DOMAIN_ID'] = '98'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
import rclpy
from rclpy.executors import SingleThreadedExecutor
from luxi_interfaces.msg import ArmCommand, ArmState, QuestInput
from luxi_interfaces.srv import Command
from std_msgs.msg import String

BINARY = Path(sys.argv.pop(1)).resolve()
CONFIG = Path(sys.argv.pop(1)).resolve()
config = json.loads(CONFIG.read_text())
HOME = []
for side in ('right', 'left'):
    HOME += [min(max(v, lo), hi) for v, (lo, hi) in
             zip(config[side]['home'], config[side]['limits'])]


class SharedParking(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(prefix='luxi-shared-park-')
        self.log = open(Path(self.tmp.name) / 'manager.log', 'w+')
        self.node = rclpy.create_node('shared_parking_test')
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.quest_times = []
        self.prefix = '/shared_park_' + str(time.monotonic_ns())
        self.status = {}
        self.directive = {}
        self.directives = []
        self.commands = []
        self.lifecycle = []
        self.enabled = True
        self.follow_commands = True
        self.settling_offset = [0.] * 16
        self.velocity = [0.] * 16
        self.disable_times = []
        self.position = HOME.copy()
        self.position[0] += .2
        self.position[8] -= .2
        self.position[7], self.position[15] = -.12, .12
        self.initial = self.position.copy()
        self.arm_valid = True
        self.base_stationary = True
        self.lift_stationary = True
        self.lift_session = False
        self.lift_healthy = True
        self.base_feedback_age_ns = 0
        self.lift_sample_age_ns = 0
        self.lift_measurement_age_ms = 30
        self.quest_sending = True
        self.quest = QuestInput()
        self.quest.buttons = [False] * 4
        self.quest.grips = [0., 0.]
        self.qp = self.node.create_publisher(QuestInput, self.prefix + '/quest', 1)
        self.sp = self.node.create_publisher(ArmState, self.prefix + '/arms/state', 1)
        self.bp = self.node.create_publisher(String, self.prefix + '/base/status', 1)
        self.lp = self.node.create_publisher(String, self.prefix + '/lift/status', 1)
        self.node.create_subscription(String, self.prefix + '/control_status',
                                      lambda m: self.status.update(json.loads(m.data)), 10)
        self.node.create_subscription(String, self.prefix + '/coordinated_control', self.on_directive, 10)
        self.node.create_subscription(ArmCommand, self.prefix + '/command', self.on_command, 100)
        self.node.create_subscription(String, self.prefix + '/driver_lifecycle', self.on_lifecycle, 10)
        self.client = self.node.create_client(Command, self.prefix + '/control')
        args = [str(BINARY), '--ros-args', '-p', 'config:=' + str(CONFIG),
                '-p', 'shared_enable:=true', '-p', 'home_joint_speed_limit:=0.2',
                '-p', 'home_tolerance:=0.1']
        for topic in ('control_status', 'quest', 'arms/state', 'intent', 'control', 'command',
                      'driver_lifecycle', 'coordinated_control', 'base/status', 'lift/status', 'recorder/status'):
            args += ['-r', '/luxi/' + topic + ':=' + self.prefix + '/' + topic]
        self.process = subprocess.Popen(args, env=dict(os.environ, ROS_LOG_DIR=self.tmp.name),
                                        stdout=self.log, stderr=subprocess.STDOUT)
        self.publisher_stop = threading.Event()
        self.publisher_error = None
        self.publisher_thread = threading.Thread(target=self.publish_loop, daemon=True)
        self.addCleanup(self.cleanup)
        self.publisher_thread.start()
        self.wait_for(lambda: self.client.service_is_ready() and self.status.get('system_phase') == 'stopped',
                      4, 'inherit enabled arms into hold')
        self.run_for(.3)

    def on_directive(self, message):
        self.directive = json.loads(message.data)
        self.directives.append(self.directive['phase'])

    def on_command(self, message):
        self.commands.append(message)
        if self.enabled and self.follow_commands:
            self.position = [target + offset for target, offset in
                             zip(message.position, self.settling_offset)]

    def on_lifecycle(self, message):
        self.lifecycle.append((message.data, self.position.copy()))
        if message.data == 'disable':
            self.disable_times.append(time.monotonic())
            self.enabled = False

    def publish_loop(self):
        # Publish feedback independently of assertion/subscription work. The
        # real enable gate requires 50 Quest arrivals per second; spin_once
        # overhead must not turn a 100 Hz fixture into a different input test.
        try:
            while not self.publisher_stop.is_set():
                started = time.monotonic()
                self.publish_inputs()
                self.publisher_stop.wait(max(0., .01 - (time.monotonic() - started)))
        except BaseException as error:
            self.publisher_error = error

    def publish_inputs(self):
        now = time.monotonic_ns()
        state = ArmState()
        state.receive_mono_ns = now
        state.enabled = self.enabled
        state.position = self.position
        state.velocity = self.velocity
        state.valid = [self.arm_valid] * 16
        self.sp.publish(state)
        if self.quest_sending:
            self.quest.receive_mono_ns = now
            self.qp.publish(self.quest)
            self.quest_times.append(now)
        shared = {'shared_enable': True, 'shared_failed': False,
                  'shared_epoch': self.directive.get('epoch', ''),
                  'shared_phase': self.directive.get('phase', ''),
                  'receive_mono_ns': str(now), 'enabled': False}
        base = dict(shared, receive_only=False, feedback_fresh=True,
                    feedback_mono_ns=str(now-self.base_feedback_age_ns), stationary=self.base_stationary)
        # Nonzero lift height, no home_complete: returning the arms must not
        # depend on lift HOME or on a claim that this height has physical support.
        lift = dict(shared, allow_motion=True, feedback_healthy=self.lift_healthy,
                    feedback_age_ms=self.lift_measurement_age_ms, feedback_first_mono_ns=str(now-self.lift_sample_age_ns),
                    stationary=self.lift_stationary, session_present=self.lift_session,
                    home_complete=False, current_um=75321)
        if getattr(self, 'simulate_active_devices', False):
            phase = self.directive.get('phase', '')
            paused = getattr(self, 'position_pause', False)
            base.update(enabled=phase == 'active', shared_ready=True)
            lift.update(enabled=phase in ('prepare', 'active') and not paused,
                        shared_ready=not paused, flags=1, fault=0, speed_rpm=0,
                        transport='uart')
            if paused:
                lift.update(enabled=False, shared_ready=False, stationary=False,
                            feedback_healthy=False, flags=0,
                            position_invalid_isolation=True, position_invalid_limit=5,
                            position_invalid_count=5, position_invalid_pause_count=5,
                            position_invalid_paused=True, position_invalid_stationary=True)
            lift.update(getattr(self, 'extra_lift_status', {}))
            intent_pub = getattr(self, 'active_intent_pub', None)
            if intent_pub is not None:
                intent = ArmCommand()
                intent.position = HOME.copy()
                intent.position[0] += getattr(self, 'intent_offset', 0.)
                intent.source_mono_ns = now
                intent.phase = 'teleop'
                intent_pub.publish(intent)
        for pub, data in ((self.bp, base), (self.lp, lift)):
            message = String()
            message.data = json.dumps(data)
            pub.publish(message)

    def step(self):
        if self.publisher_error:
            raise self.publisher_error
        for _ in range(8):
            self.executor.spin_once(timeout_sec=0)
        time.sleep(.003)

    def run_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.step()

    def wait_for(self, condition, seconds, description):
        end = time.monotonic() + seconds
        while not condition() and time.monotonic() < end:
            self.step()
        self.assertTrue(condition(), description + ': ' + json.dumps(self.status))

    def request_park(self):
        # A service can overtake topic discovery/feedback. Wait for a newly
        # published hold command: the real controller emits it only with
        # valid, fresh arm feedback. Do not weaken that production precondition.
        after = time.monotonic_ns()
        self.wait_for(lambda: bool(self.commands) and self.commands[-1].source_mono_ns >= after,
                      2, 'fresh arm feedback acknowledged before park')
        request = Command.Request()
        request.command = 'park'
        future = self.client.call_async(request)
        self.wait_for(future.done, 2, 'park response')
        self.assertTrue(future.result().success, future.result().message + "; status=" + json.dumps(self.status))

    def test_short_x_returns_arms_then_disables_without_lift_home(self):
        self.quest.buttons[2] = True
        self.run_for(.12)
        self.quest.buttons[2] = False
        self.wait_for(lambda: self.status.get('system_phase') == 'disabled' and not self.enabled,
                      6, 'short X returns arms and removes torque')
        self.assertNotIn('home', self.directives, 'no lift homing directive')
        off = [position for command, position in self.lifecycle if command == 'disable']
        self.assertEqual(len(off), 1)
        for i in range(16):
            self.assertAlmostEqual(off[0][i], HOME[i], delta=.03)
        self.assertTrue(any(c.phase == 'returning' for c in self.commands))

    def test_parking_waits_for_lift_to_stop_and_close_old_session(self):
        self.lift_stationary = False
        self.lift_session = True
        self.request_park()
        self.run_for(.5)
        self.assertEqual(self.status['system_phase'], 'stopping_base')
        self.assertFalse(any(c.phase == 'returning' for c in self.commands))
        self.lift_stationary = True
        self.lift_session = False
        self.wait_for(lambda: not self.enabled, 6, 'return only after stopped confirmation')
        self.assertNotIn('home', self.directives)

    def test_recorded_settling_offsets_do_not_block_legacy_stop(self):
        # Measured stationary offsets from the 2026-09-24 supervised return.
        # The real controller must handle imperfect feedback, not only an
        # ideal simulated arm that exactly follows every submitted target.
        for joint, offset in ((0, -.05303), (3, -.05606), (5, .04635),
                              (8, .06256), (11, -.05417), (13, -.04921)):
            self.settling_offset[joint] = offset
        self.request_park()
        self.wait_for(lambda: not self.enabled, 6,
                      'legacy return completes its command sequence despite measured offsets')
        off = [position for command, position in self.lifecycle if command == 'disable']
        self.assertEqual(len(off), 1)
        for i in range(16):
            if i % 8 != 7:
                self.assertLessEqual(abs(off[0][i] - HOME[i]), .1)
                self.assertAlmostEqual(self.commands[-1].position[i], HOME[i], delta=.01)
        self.assertNotIn('home', self.directives)

    def test_legacy_stop_does_not_wait_for_position_or_velocity_convergence(self):
        self.settling_offset[8] = .15
        self.velocity[8] = .06
        self.request_park()
        self.wait_for(lambda: self.status.get('system_phase') == 'returning_arms', 2, 'arm return')
        self.wait_for(lambda: not self.enabled, 6, 'legacy timed sequence removes torque without an arrival gate')
        self.assertAlmostEqual(self.position[8]-HOME[8], .15, delta=.01)
        self.assertEqual(len(self.disable_times), 1)
        self.assertFalse(self.status['parking_requires_measured_arrival'])
        self.assertNotIn('home', self.directives)

    def test_already_home_still_runs_the_full_timed_sequence(self):
        self.follow_commands = False
        self.initial = HOME.copy()
        self.position = HOME.copy()
        started = time.monotonic()
        self.request_park()
        self.run_for(1.5)
        self.assertTrue(self.enabled, 'home position must not shortcut the original timed sequence')
        self.wait_for(lambda: not self.enabled, 6, 'timed sequence completes')
        self.assertGreaterEqual(self.disable_times[0]-started, 2.)
        self.assertEqual(self.status['return_step'], 100)

    def test_arm_feedback_loss_aborts_without_torque_removal(self):
        self.request_park()
        self.wait_for(lambda: self.status.get('system_phase') == 'returning_arms', 2, 'arm return')
        self.arm_valid = False
        self.wait_for(lambda: self.status.get('system_phase') == 'stopped', 1, 'arm feedback abort')
        self.run_for(2.3)
        self.assertTrue(self.enabled)
        self.assertFalse(self.lifecycle)

    def test_lift_motion_during_arm_return_aborts_without_torque_removal(self):
        self.request_park()
        self.wait_for(lambda: self.status.get('system_phase') == 'returning_arms', 2, 'arm return')
        self.lift_stationary = False
        self.wait_for(lambda: self.status.get('system_phase') == 'stopped', 1, 'moving lift aborts')
        self.run_for(.6)
        self.assertTrue(self.enabled)
        self.assertFalse(self.lifecycle)

    def check_old_measurement_aborts(self, field, value):
        self.request_park()
        self.wait_for(lambda: self.status.get('system_phase') == 'returning_arms', 2, 'arm return')
        setattr(self, field, value)
        self.wait_for(lambda: self.status.get('system_phase') == 'stopped', 1, 'stale measurement aborts')
        self.assertEqual(self.status['reason'], 'base_or_lift_feedback_lost')
        snapshot = self.status.get('feedback_fault_snapshot')
        self.assertIsInstance(snapshot, dict, 'missing exact reports used for the stop decision')
        decision = int(snapshot['decision_mono_ns'])
        if field == 'base_feedback_age_ns':
            self.assertGreaterEqual(decision-int(snapshot['base']['feedback_mono_ns']), value)
        elif field == 'lift_sample_age_ns':
            self.assertGreaterEqual(decision-int(snapshot['lift']['feedback_first_mono_ns']), value)
        else:
            self.assertEqual(snapshot['lift']['feedback_age_ms'], value)
        self.assertTrue(self.enabled)
        self.assertFalse(self.lifecycle)

    def test_fresh_base_status_cannot_refresh_old_motor_feedback(self):
        self.check_old_measurement_aborts('base_feedback_age_ns', 300000000)

    def test_fresh_lift_status_cannot_refresh_old_sample(self):
        self.check_old_measurement_aborts('lift_sample_age_ns', 350000000)

    def test_fresh_lift_sample_cannot_refresh_old_measurement(self):
        self.check_old_measurement_aborts('lift_measurement_age_ms', 600)

    def test_lift_feedback_loss_aborts_without_torque_removal(self):
        self.request_park()
        self.wait_for(lambda: self.status.get('system_phase') == 'returning_arms', 2, 'arm return')
        self.lift_healthy = False
        self.wait_for(lambda: self.status.get('system_phase') == 'stopped', 1, 'feedback loss aborts')
        self.assertTrue(self.enabled)
        self.assertFalse(self.lifecycle)

    def test_b_holds_without_return_or_torque_removal(self):
        self.quest.buttons[1] = True
        self.run_for(.5)
        self.assertEqual(self.status['system_phase'], 'stopped')
        self.assertTrue(self.enabled)
        self.assertFalse(self.lifecycle)
        self.assertFalse(any(c.phase == 'returning' for c in self.commands))
        self.assertNotIn('home', self.directives)

    def test_short_quest_gap_does_not_pause_the_requested_fixed_return(self):
        self.request_park()
        self.wait_for(lambda: self.status.get('system_phase') == 'returning_arms', 2, 'arm return')
        self.quest_sending = False
        self.run_for(.21)
        earlier = list(self.commands[-1].position)
        self.run_for(.09)
        self.assertEqual(self.status['system_phase'], 'returning_arms')
        self.assertFalse(self.status['input_paused'])
        self.assertNotEqual(list(self.commands[-1].position), earlier, 'fixed return keeps advancing')
        self.assertTrue(self.enabled)
        self.quest_sending = True
        self.wait_for(lambda: not self.enabled, 6, 'fixed return completes')
        self.assertNotIn('home', self.directives)

    def test_fixed_return_finishes_without_further_quest_packets(self):
        self.request_park()
        self.wait_for(lambda: self.status.get('system_phase') == 'returning_arms', 2, 'arm return')
        self.quest_sending = False
        self.wait_for(lambda: not self.enabled, 6, 'legacy fixed return is independent of headset packets')
        self.quest_sending = True
        self.run_for(.4)
        self.assertEqual(self.status['system_phase'], 'disabled')
        self.assertEqual(len(self.disable_times), 1)

    def test_quest_recovery_does_not_cancel_an_in_progress_fixed_return(self):
        self.request_park()
        self.wait_for(lambda: self.status.get('system_phase') == 'returning_arms', 2, 'arm return')
        self.quest_sending = False
        self.run_for(.65)
        self.quest_sending = True
        self.quest.left_tracking = 2
        self.wait_for(lambda: not self.enabled, 6, 'recovery callback does not cancel the fixed trajectory')

    def test_b_interrupts_fixed_return_and_keeps_torque(self):
        self.request_park()
        self.wait_for(lambda: self.status.get('system_phase') == 'returning_arms', 2, 'arm return')
        self.quest.buttons[1] = True
        self.wait_for(lambda: self.status.get('system_phase') == 'stopped', 1, 'B interrupts return')
        self.run_for(2.3)
        self.assertTrue(self.enabled)
        self.assertFalse(self.lifecycle)

    def test_device_fault_during_short_quest_gap_still_aborts(self):
        self.request_park()
        self.wait_for(lambda: self.status.get('system_phase') == 'returning_arms', 2, 'arm return')
        self.quest_sending = False
        self.run_for(.2)
        self.lift_healthy = False
        self.wait_for(lambda: self.status.get('system_phase') == 'stopped', .2,
                      'device fault must stop before the 500 ms input-loss threshold')
        self.assertEqual(self.status['reason'], 'base_or_lift_feedback_lost')
        self.assertTrue(self.enabled)
        self.assertFalse(self.lifecycle)

    def test_long_x_press_survives_one_short_packet_gap(self):
        self.quest.buttons[2] = True
        self.run_for(.7)
        self.quest_sending = False
        self.run_for(.26)
        self.quest_sending = True
        self.run_for(.7)
        self.assertEqual(self.status['system_phase'], 'preparing',
                         'a short gap must not silently cancel a deliberate long X hold: ' + json.dumps(self.status) +
                         '; published in last second=' + str(sum(t > time.monotonic_ns()-1000000000 for t in self.quest_times)))

    def test_long_gap_cannot_complete_a_held_x_enable(self):
        self.quest.buttons[2] = True
        self.run_for(.7)
        self.quest_sending = False
        self.run_for(.65)
        self.quest_sending = True
        self.run_for(.9)
        self.assertEqual(self.status['system_phase'], 'stopped')
        self.assertNotIn('prepare', self.directives)

    def cleanup(self):
        self.publisher_stop.set()
        self.publisher_thread.join(timeout=2)
        if self.process.poll() is None:
            self.process.send_signal(signal.SIGINT)
        try:
            self.process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            self.process.kill()
            self.process.wait()
        self.executor.shutdown()
        self.node.destroy_node()
        self.log.close()
        self.tmp.cleanup()


if __name__ == '__main__':
    rclpy.init()
    try:
        unittest.main(verbosity=2)
    finally:
        rclpy.shutdown()
