#!/usr/bin/env python3
"""Actual lift_adapter + Quest callbacks against a finite-target CAN simulator.

Run only on an explicitly supplied, existing vcan interface. The simulator
models the firmware's 1.2 s BUSY interval after stopping. No physical CAN
interface, motion hardware, firmware flashing or interface setup is permitted.
Requires the ROS workspace environment and an already-built lift_adapter.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import socket
import struct
import subprocess
import tempfile
import time
import unittest

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('binary', type=Path)
parser.add_argument('--interface', required=True)
args, remaining = parser.parse_known_args()
link = json.loads(subprocess.check_output(
    ['ip', '-details', '-json', 'link', 'show', 'dev', args.interface], text=True))
if len(link) != 1 or link[0].get('linkinfo', {}).get('info_kind') != 'vcan':
    raise SystemExit('Refusing a non-vcan interface; no hardware tests in this script.')
os.environ['ROS_DOMAIN_ID'] = '99'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
import rclpy
from luxi_interfaces.msg import QuestInput
from luxi_interfaces.srv import Command
from std_msgs.msg import String

FRAME = struct.Struct('=IB3x8s')


class LiftHold(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(prefix='luxi-lift-hold-')
        self.log = open(Path(self.tmp.name) / 'adapter.log', 'w+')
        self.node = rclpy.create_node('lift_hold_regression')
        self.prefix = '/lift_hold_' + str(time.monotonic_ns())
        self.pub = self.node.create_publisher(QuestInput, self.prefix + '/quest', 1)
        self.shared = self._testMethodName.startswith('test_shared_')
        self.coordination_phase = None
        self.coordination_pub = self.node.create_publisher(String, self.prefix + '/coordinated_control', 1)
        self.status = {}
        self.state = {}
        self.events = []
        self.node.create_subscription(String, self.prefix + '/lift/status',
                                      lambda m: self.status.update(json.loads(m.data)), 10)
        self.node.create_subscription(String, self.prefix + '/lift/state',
                                      lambda m: self.state.update(json.loads(m.data)), 10)
        self.node.create_subscription(String, self.prefix + '/lift/events',
                                      lambda m: self.events.append(json.loads(m.data)), 100)
        self.client = self.node.create_client(Command, self.prefix + '/lift/control')
        self.quest = QuestInput()
        self.quest.grips = [0., 0.]
        self.quest.buttons = [False] * 4
        self.sending = True
        self.bus = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        # Command ID only; tests cannot consume the adapter's telemetry stream.
        self.bus.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER,
                            struct.pack('=II', 0x601, 0xC00007FF))
        self.bus.bind((args.interface,))
        self.bus.setblocking(False)
        self.started = self.last_step = time.monotonic()
        self.last_telemetry = 0.
        self.telemetry_sending = True
        self.heartbeat_acks = True
        self.sequence = 0
        self.nonce = None
        self.lease = 0.
        self.position = 20000 if self.shared else 0
        self.target = 0
        self.moving = False
        self.target_status = 0
        self.valid = True
        self.fault = 0
        self.busy_until = 0.
        self.moves = []
        self.rejections = []
        self.requests = []
        self.process = None
        command = [str(args.binary.resolve()), '--ros-args', '-p', 'allow_hardware:=true',
                   '-p', 'allow_motion:=true', '-p', 'shared_enable:=' + str(self.shared).lower(),
                   '-p', 'interface:=' + args.interface,
                   '-p', 'network_recovery_timeout:=' + ('10.0' if 'ten_second' in self._testMethodName else '0.5')]
        for topic in ('quest', 'recorder/status', 'coordinated_control', 'lift/state',
                      'lift/can_rx', 'lift/events', 'lift/status', 'lift/control'):
            command += ['-r', '/luxi/' + topic + ':=' + self.prefix + '/' + topic]
        self.process = subprocess.Popen(command, stdout=self.log, stderr=subprocess.STDOUT,
                                        env=dict(os.environ, ROS_LOG_DIR=self.tmp.name))
        self.addCleanup(self.cleanup_resources)
        self.wait_for(lambda: self.client.service_is_ready() and
                      self.status.get('feedback_healthy'), 5, 'adapter discovery')
        self.run_for(.15)  # Let the fresh neutral Quest callback arrive before service start.
        if self.shared:
            self.coordination_phase = 'prepare'
        else:
            request = Command.Request()
            request.command = 'start'
            future = self.client.call_async(request)
            self.wait_for(future.done, 2, 'enable response')
            self.assertTrue(future.result().success, future.result().message)
        self.wait_for(lambda: self.status.get('phase') == 'idle', 4, 'configuration complete')
        if self.shared:
            self.coordination_phase = 'active'
        self.run_for(.2)

    def flags(self):
        return (0x32 | int(self.valid) | (0x40 if self.nonce else 0) |
                (0x0C if self.moving else 0))

    def send(self, identifier, payload):
        self.bus.send(FRAME.pack(identifier, 8, payload))

    def stop_motion(self):
        if self.moving:
            self.moving = False
            self.busy_until = time.monotonic() + 1.2
            self.target_status = 0xE4

    def commands(self, now):
        for _ in range(128):
            try:
                identifier, length, data = FRAME.unpack(self.bus.recv(FRAME.size))
            except BlockingIOError:
                break
            if identifier != 0x601 or length != 8:
                continue
            op, seq, value, argument, version = struct.unpack('<BBiBB', data)
            self.requests.append((op, seq, value))
            self.assertEqual((argument, version), (0, 1))
            result, reply = 0, value
            if op == 4:
                reply = [1, 21, -97000, 197000, 2, 15, 3000, -97000, 197000, 4][value]
            elif op == 1:
                self.nonce, self.lease = value, now
            elif op == 2:
                if value != self.nonce:
                    result = 0x82
                else:
                    self.lease = now
                if not self.heartbeat_acks:
                    continue
            elif op == 5:
                if value == self.nonce:
                    self.stop_motion()
                    self.nonce = None
            elif op == 3:
                self.stop_motion()
            elif op in (0x10, 0x11):
                if not self.nonce:
                    result = 0x82
                elif not self.valid:
                    result = 2
                elif self.moving or now < self.busy_until:
                    result = 1
                else:
                    self.target = value if op == 0x10 else 0
                    self.moves.append((now, self.target))
                    self.target_status = 2
                    self.moving = True
                if result:
                    self.rejections.append((op, result))
            else:
                self.fail('Unexpected opcode; test must not write configuration: ' + str(op))
            self.send(0x581, struct.pack('<BBBBi', op, seq, result, self.flags(), reply))

    def step(self):
        now = time.monotonic()
        dt, self.last_step = now - self.last_step, now
        self.commands(now)
        if self.nonce and now - self.lease > .5:
            self.stop_motion()
            self.nonce = None
        if self.moving:
            remaining = self.target - self.position
            delta = 15000 * dt
            if abs(remaining) <= delta:
                self.position = self.target
                self.stop_motion()
                self.target_status = 3
            else:
                self.position += delta if remaining > 0 else -delta
        if self.telemetry_sending and now - self.last_telemetry >= .1:
            self.last_telemetry = now
            self.sequence = (self.sequence + 1) & 0xFFFF
            uptime = int((now - self.started) * 1000) + 10000
            speed = (900 if self.target > self.position else -900) if self.moving else 0
            pages = [struct.pack('<Hih', self.sequence, round(self.position), speed),
                     struct.pack('<HiBB', self.sequence, self.target, self.target_status, self.flags()),
                     struct.pack('<HHBBBB', self.sequence, self.fault, 2, 2, 0, 0),
                     struct.pack('<HIH', self.sequence, uptime, 10)]
            for page, data in enumerate(pages):
                self.send(0x181 + 0x100 * page, data)
        if self.sending:
            self.quest.receive_mono_ns = time.monotonic_ns()
            self.pub.publish(self.quest)
        if self.coordination_phase:
            message = String()
            message.data = json.dumps({'epoch': self.prefix, 'phase': self.coordination_phase,
                                       'sent_mono_ns': str(time.monotonic_ns())})
            self.coordination_pub.publish(message)
        for _ in range(8):
            rclpy.spin_once(self.node, timeout_sec=0)
        time.sleep(.005)

    def run_for(self, duration):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            self.step()

    def wait_for(self, predicate, duration, description):
        end = time.monotonic() + duration
        while not predicate() and time.monotonic() < end:
            self.step()
        self.assertTrue(predicate(), description + ': ' + json.dumps(self.status) +
                        '\nrequests=' + repr(self.requests[-10:]) +
                        '\nevents=' + json.dumps(self.events[-10:]))

    def start_up(self):
        self.quest.grips = [0., 1.]
        self.wait_for(lambda: self.moving, 3, 'right grip starts upward travel')

    def test_held_grip_continues_without_one_mm_busy_disable(self):
        self.start_up()
        self.run_for(2.5)
        self.assertTrue(self.status['enabled'], 'holding a grip must not disable at the first target; ' +
                        self.status.get('reason', '') + '; rejections=' + repr(self.rejections))
        self.assertTrue(self.moving)
        self.assertGreater(self.position, 1000)
        self.assertEqual([target for _, target in self.moves], [192000])
        self.assertEqual(self.rejections, [])
        self.quest.grips = [0., 0.]
        self.wait_for(lambda: not self.moving and not self.nonce, .5, 'release closes own motion')
        self.run_for(.25)
        self.assertTrue(self.status['enabled'], 'normal release preserves permission')

    def test_captured_stuff_error_after_release_keeps_permission_for_next_hold(self):
        self.start_up()
        self.quest.grips = [0., 0.]
        self.wait_for(lambda: not self.moving and not self.nonce, .5, 'release stops')
        self.wait_for(lambda: self.status.get('phase') == 'idle', 1, 'stopped idle')
        # Exact frame captured on CH4 after a normal release, 2026-09-24.
        self.send(0x20000088, bytes([0, 0, 4, 32, 0, 0, 0, 1]))
        self.run_for(.25)
        self.assertTrue(self.status['enabled'], 'isolated CAN frame error must not revoke permission')
        self.assertTrue(any(e.get('event') == 'can_error' and not e.get('fatal')
                            for e in self.events), 'frame error remains visible in diagnostics')
        self.quest.grips = [1., 0.]
        self.wait_for(lambda: len(self.moves) == 2, 3, 'next hold works without dashboard re-enable')
        self.assertEqual(self.moves[-1][1], -92000)
        self.assertEqual(self.rejections, [])

    def test_new_complete_feedback_is_reported_before_periodic_status_tick(self):
        previous = self.status['receive_mono_ns']
        self.wait_for(lambda: self.status['receive_mono_ns'] != previous, .3, 'next status tick')
        self.run_for(.02)
        self.last_telemetry = 0.  # A new complete CAN snapshot just after that tick.
        self.step()
        sequence = self.sequence
        self.wait_for(lambda: self.state.get('sequence') == sequence, .04, 'fresh state delivered')
        first = self.state['receive_first_mono_ns']
        self.wait_for(lambda: self.status.get('feedback_first_mono_ns') == first, .04,
                      'coordinator must receive the new feedback without another 100 ms wait')

    def test_shared_idle_gap_keeps_readiness_during_arm_preparation(self):
        self.coordination_phase = 'prepare'
        self.wait_for(lambda: self.status.get('shared_phase') == 'prepare' and
                      self.status.get('shared_ready'), 3, 'lift ready for arm preparation')
        before = len(self.events)
        self.sending = False
        self.run_for(.3)
        self.assertTrue(self.status['enabled'])
        self.assertTrue(self.status['shared_ready'])
        self.assertEqual(self.status['phase'], 'idle')
        self.assertTrue(self.status['input_recovery'])
        self.assertFalse(any(e.get('event') == 'cancel' for e in self.events[before:]))
        self.sending = True
        self.run_for(.2)
        self.assertFalse(self.status['input_recovery'])
        self.assertFalse(self.moves)

    def test_shared_short_gap_closes_motion_and_keeps_permission(self):
        self.start_up()
        self.sending = False
        self.run_for(.3)
        self.assertFalse(self.moving)
        self.assertFalse(self.nonce)
        self.assertTrue(self.status['enabled'])
        self.sending = True  # Right grip is still held: no replay.
        self.run_for(1.4)
        self.assertEqual(len(self.moves), 1)
        self.assertFalse(self.moving)
        self.quest.grips = [0., 0.]
        self.run_for(.2)
        self.quest.grips = [1., 0.]
        self.wait_for(lambda: len(self.moves) == 2, 2, 'new grip after release')
        self.assertTrue(self.status['enabled'])

    def test_shared_ten_second_window_stops_motion_and_requires_new_grip(self):
        self.start_up()
        self.sending = False
        self.run_for(9.3)
        self.assertFalse(self.moving)
        self.assertTrue(self.status['enabled'])
        self.sending = True
        self.run_for(.3)
        self.assertEqual(len(self.moves), 1)
        self.quest.grips = [0., 0.]
        self.run_for(.2)
        self.quest.grips = [1., 0.]
        self.wait_for(lambda: len(self.moves) == 2, 2, 'new grip after recovery')
        self.sending = False
        self.run_for(10.3)
        self.assertFalse(self.status['enabled'])

    def test_shared_long_gap_disables_until_explicit_enable(self):
        self.start_up()
        self.sending = False
        self.run_for(.65)
        self.assertFalse(self.status['enabled'])
        self.assertFalse(self.moving)
        self.assertFalse(self.nonce)
        self.quest.grips = [0., 0.]
        self.sending = True
        self.run_for(.3)
        self.assertFalse(self.status['enabled'])

    def test_shared_serious_can_fault_during_short_gap_still_disables(self):
        self.start_up()
        self.sending = False
        self.run_for(.25)
        self.send(0x20000040, bytes(8))  # bus off
        self.wait_for(lambda: not self.status.get('enabled'), .2, 'CAN fault overrides grace')
        self.sending = True
        self.run_for(.3)
        self.assertFalse(self.status['enabled'])

    def test_shared_b_during_input_recovery_still_disables(self):
        self.start_up()
        self.sending = False
        self.run_for(.3)
        self.assertTrue(self.status['enabled'])
        self.quest.buttons[1] = True
        self.sending = True
        self.wait_for(lambda: not self.status.get('enabled'), .4, 'B overrides recovery')

    def test_frame_errors_with_healthy_feedback_do_not_replay_motion(self):
        self.start_up()
        for payload in (bytes([0, 0, 4, 32, 0, 0, 0, 1]),
                        bytes([0, 0, 128, 25, 0, 0, 8, 0])):
            self.send(0x20000088, payload)
            self.run_for(.25)
            self.assertTrue(self.status['enabled'])
            self.assertTrue(self.moving)
            self.assertEqual(len(self.moves), 1, 'no new HELLO/MOVE or motion replay')
        self.assertEqual(sum(op == 1 for op, _, _ in self.requests), 1)
        self.quest.grips = [0., 0.]
        self.wait_for(lambda: not self.moving and not self.nonce, .5, 'release still stops')

    def assert_can_fault_disables(self, identifier, payload):
        self.start_up()
        self.send(identifier, payload)
        self.wait_for(lambda: not self.status.get('enabled') and not self.nonce, .5,
                      'serious CAN fault cancels and disables')
        self.run_for(1.5)
        self.assertFalse(self.status['enabled'], 'healthy feedback cannot automatically re-enable')
        self.assertFalse(self.moving)
        self.assertEqual(len(self.moves), 1, 'held grip cannot replay after CAN fault')

    def test_bus_off_still_disables(self):
        self.assert_can_fault_disables(0x20000040, bytes(8))

    def test_controller_restart_still_disables(self):
        self.assert_can_fault_disables(0x20000100, bytes(8))

    def test_kcan_warning_with_unspecified_detail_still_disables(self):
        # KCAN driver sets detail bits only when counters >96; at exactly 96
        # the observed warning was CAN_ERR_CRTL with eight zero bytes.
        self.assert_can_fault_disables(0x20000004, bytes(8))

    def test_warning_level_counter_still_disables(self):
        self.assert_can_fault_disables(0x20000088, bytes([0, 0, 128, 25, 0, 0, 96, 0]))

    def test_protocol_overload_still_disables(self):
        self.assert_can_fault_disables(0x20000008, bytes([0, 0, 32, 0, 0, 0, 0, 0]))

    def test_frame_error_does_not_extend_heartbeat_ack_deadline(self):
        self.start_up()
        self.heartbeat_acks = False
        for _ in range(6):
            self.send(0x20000088, bytes([0, 0, 4, 32, 0, 0, 0, 1]))
            self.run_for(.1)
        self.assertFalse(self.status['enabled'])
        self.assertIsNone(self.nonce)
        self.assertTrue(any(e.get('event') == 'cancel' and 'heartbeat ACK timeout' in e.get('detail', '')
                            for e in self.events))

    def test_error_frames_cannot_substitute_for_complete_telemetry(self):
        self.start_up()
        self.telemetry_sending = False
        for _ in range(6):
            self.send(0x20000088, bytes([0, 0, 4, 32, 0, 0, 0, 1]))
            self.run_for(.1)
        self.assertFalse(self.status['enabled'])
        self.assertIsNone(self.nonce)
        self.assertEqual(self.status['health'], 'can_feedback_timeout')
        self.assertTrue(any(e.get('event') == 'cancel' and e.get('detail') == 'can_feedback_timeout'
                            for e in self.events))

    def test_quick_repress_waits_for_settling_then_moves_down(self):
        self.start_up()
        self.run_for(.2)
        self.quest.grips = [0., 0.]
        self.wait_for(lambda: not self.moving and not self.nonce, .5, 'release stops')
        self.run_for(.15)
        self.quest.grips = [1., 0.]
        self.run_for(.25)
        self.assertEqual(len(self.moves), 1, 'no MOVE during firmware BUSY settling interval')
        self.wait_for(lambda: len(self.moves) == 2, 3, 'held left grip starts after settling')
        self.assertEqual(self.moves[-1][1], -92000)
        self.assertEqual(self.rejections, [])
        self.assertTrue(self.status['enabled'])

    def test_endpoint_stays_enabled_and_requires_release(self):
        self.start_up()
        self.position = self.target  # Simulate arrival without a long travel wait.
        self.wait_for(lambda: self.status.get('phase') == 'idle' and not self.nonce, 2,
                      'endpoint closes the session')
        self.run_for(1.5)
        self.assertTrue(self.status['enabled'])
        self.assertEqual(len(self.moves), 1, 'held endpoint must not create another MOVE')
        self.quest.grips = [0., 0.]
        self.run_for(.2)
        self.quest.grips = [0., 1.]
        self.run_for(1.)
        self.assertTrue(self.status['enabled'])
        self.assertEqual(len(self.moves), 1, 'outward press at boundary must not create a MOVE')

    def test_releasing_while_waiting_cancels_the_pending_hold(self):
        self.start_up()
        self.run_for(.2)
        self.quest.grips = [0., 0.]
        self.wait_for(lambda: not self.moving and not self.nonce, .5, 'release stops')
        self.run_for(.1)
        self.quest.grips = [1., 0.]
        self.wait_for(lambda: self.status.get('phase') == 'waiting_for_settle', .4, 'new hold waits')
        self.quest.grips = [0., 0.]
        self.run_for(1.6)
        self.assertEqual(len(self.moves), 1, 'released queued hold must never move later')
        self.assertTrue(self.status['enabled'])
        self.assertFalse(self.moving)

    def test_shared_home_still_waits_and_confirms_locked_center(self):
        self.busy_until = time.monotonic() + 1.2
        self.coordination_phase = 'home'
        self.wait_for(lambda: self.status.get('home_complete'), 7, 'shared home locks at center')
        self.assertEqual([op for op, _, _ in self.requests if op in (0x10, 0x11)], [0x11])
        self.assertEqual(self.rejections, [])
        self.assertEqual(self.position, 0)
        self.assertIsNone(self.nonce)
        self.assertFalse(self.moving)

    def test_both_grips_stop_and_do_not_reverse_without_neutral(self):
        self.start_up()
        self.quest.grips = [1., 1.]
        self.wait_for(lambda: not self.moving and not self.nonce, .5, 'both grips stop')
        self.quest.grips = [1., 0.]
        self.run_for(1.6)
        self.assertTrue(self.status['enabled'])
        self.assertEqual(len(self.moves), 1, 'both grips must be released before reversal')

    def test_b_disables_and_does_not_automatically_reenable(self):
        self.start_up()
        self.quest.buttons[1] = True
        self.wait_for(lambda: not self.status.get('enabled') and not self.nonce, .5, 'B disables')
        self.quest.buttons[1] = False
        self.run_for(.5)
        self.assertFalse(self.status['enabled'])

    def test_quest_loss_still_cancels_and_disables(self):
        self.start_up()
        self.sending = False
        self.wait_for(lambda: not self.status.get('enabled') and not self.nonce, .6,
                      'Quest loss cancels')

    def test_invalid_position_still_cancels_and_disables(self):
        self.start_up()
        self.valid = False
        self.wait_for(lambda: not self.status.get('enabled') and not self.nonce, .5,
                      'invalid position cancels')

    def cleanup_resources(self):
        if self.process is not None:
            self.process.send_signal(signal.SIGINT)
            try:
                self.process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=3)
        self.bus.close()
        self.node.destroy_node()
        self.log.close()
        self.tmp.cleanup()


if __name__ == '__main__':
    rclpy.init(args=[])
    try:
        unittest.main(argv=['test_lift_hold.py', *remaining])
    finally:
        rclpy.shutdown()
