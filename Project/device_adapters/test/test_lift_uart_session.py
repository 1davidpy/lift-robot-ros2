#!/usr/bin/env python3
"""Actual lift_adapter with a PTY firmware simulator. Never opens a hardware port.

Run explicitly with an already built adapter. All ROS topics are remapped into
an isolated namespace/domain; the only serial path is created by os.openpty().
"""
import argparse
import errno
import json
import os
from pathlib import Path
import signal
import struct
import subprocess
import tempfile
import time
import tty
import unittest

ap = argparse.ArgumentParser(description=__doc__)
ap.add_argument('binary', type=Path)
args, remaining = ap.parse_known_args()
os.environ['ROS_DOMAIN_ID'] = '97'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
import rclpy
from luxi_interfaces.msg import QuestInput
from luxi_interfaces.srv import Command
from std_msgs.msg import String


def crc(data):
    value = 0xffff
    for byte in data:
        value ^= byte << 8
        for _ in range(8):
            value = ((value << 1) ^ (0x1021 if value & 0x8000 else 0)) & 0xffff
    return value


def frame(kind, data):
    body = bytes((len(data) + 1, kind)) + data
    return b'\xa5\x5a' + body + struct.pack('<H', crc(body))


class UartSession(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(prefix='luxi-uart-pty-')
        self.log = open(Path(self.tmp.name) / 'adapter.log', 'w+')
        self.master, self.slave = os.openpty()
        tty.setraw(self.slave)  # No terminal echo before the adapter opens its end.
        self.port = os.ttyname(self.slave)
        self.assertTrue(self.port.startswith('/dev/pts/'))
        os.set_blocking(self.master, False)
        self.node = rclpy.create_node('lift_uart_pty_test')
        self.prefix = '/uart_test_' + str(time.monotonic_ns())
        self.publisher = self.node.create_publisher(QuestInput, self.prefix + '/quest', 1)
        self.quest = QuestInput(); self.quest.grips = [0., 0.]; self.quest.buttons = [False] * 4
        self.status = {}; self.events = []; self.requests = []; self.moves = []
        self.node.create_subscription(String, self.prefix + '/lift/status', lambda m: self.status.update(json.loads(m.data)), 20)
        self.node.create_subscription(String, self.prefix + '/lift/events', lambda m: self.events.append(json.loads(m.data)), 100)
        self.client = self.node.create_client(Command, self.prefix + '/lift/control')
        self.started = time.monotonic(); self.last_sample = 0.; self.sequence = 0
        self.buffer = bytearray(); self.nonce = 0; self.heartbeat = 0.
        self.target = 0; self.position = 0; self.moving = False; self.status_code = 3
        self.ready = True; self.prepare = None; self.prepare_delay = 1.1
        self.telemetry = True; self.motor_stale = False; self.reject_move = False
        self.corrupt_once = False; self.reply_heartbeat = True
        self.coordinator = None
        command = [str(args.binary.resolve()), '--ros-args', '-p', 'allow_hardware:=true',
                   '-p', 'allow_motion:=true', '-p', 'shared_enable:=false',
                   '-p', 'transport:=uart', '-p', 'serial_port:=' + self.port]
        for topic in ('quest', 'recorder/status', 'coordinated_control', 'lift/state',
                      'lift/uart_rx', 'lift/events', 'lift/status', 'lift/control'):
            command += ['-r', '/luxi/' + topic + ':=' + self.prefix + '/' + topic]
        self.process = subprocess.Popen(command, stdout=self.log, stderr=subprocess.STDOUT,
                                        env=dict(os.environ, ROS_LOG_DIR=self.tmp.name))
        self.addCleanup(self.cleanup)
        self.wait(lambda: self.client.service_is_ready() and self.status.get('feedback_healthy'), 5, 'discovery')
        self.run_for(.15)
        request = Command.Request(); request.command = 'start'
        future = self.client.call_async(request)
        self.wait(future.done, 2, 'enable response')
        self.assertTrue(future.result().success, future.result().message)
        self.wait(lambda: self.status.get('phase') == 'idle', 4, 'INFO and READY')
        self.run_for(.2)

    def cleanup(self):
        if self.process.poll() is None:
            self.process.send_signal(signal.SIGINT)
            try: self.process.wait(timeout=3)
            except subprocess.TimeoutExpired: self.process.kill(); self.process.wait()
        self.log.seek(0)
        text = self.log.read()
        if self.process.returncode not in (0, -signal.SIGINT): print(text)
        self.log.close()
        if self.master is not None: os.close(self.master)
        os.close(self.slave)
        self.node.destroy_node(); self.tmp.cleanup()

    def flags(self):
        return 0x33 | (0x40 if self.nonce else 0) | (0x0c if self.moving else 0)

    def send(self, kind, payload):
        os.write(self.master, frame(kind, payload))

    def ack(self, op, seq, result, value):
        self.send(0x81, struct.pack('<BBBBi', op, seq, result, self.flags(), value))

    def stop(self):
        self.moving = False; self.prepare = None; self.ready = True; self.status_code = 0xe4

    def commands(self, now):
        try: self.buffer.extend(os.read(self.master, 4096))
        except BlockingIOError: pass
        except OSError as exc:
            if exc.errno != errno.EIO: raise
        while len(self.buffer) >= 3:
            start = self.buffer.find(b'\xa5\x5a')
            if start < 0:
                self.buffer[:] = self.buffer[-1:] if self.buffer[-1:] == b'\xa5' else b''
                break
            if start: del self.buffer[:start]
            if len(self.buffer) < 3: break
            total = self.buffer[2] + 5
            if len(self.buffer) < total: break
            packet = bytes(self.buffer[:total]); del self.buffer[:total]
            self.assertEqual(crc(packet[2:-2]), struct.unpack('<H', packet[-2:])[0])
            self.assertEqual(packet[3], 1)
            op, seq, value, argument, version = struct.unpack('<BBiBB', packet[4:-2])
            self.assertEqual((argument, version), (0, 1))
            self.requests.append((op, seq, value)); result = 0; reply = value
            if op == 4:
                config = [1, 31, -180000, 180000, 2, 8, 3000, -180000, 180000, 4]
                reply = int(self.ready) if value == 25 else config[value]
            elif op == 1: self.nonce = value; self.heartbeat = now
            elif op == 2:
                if value != self.nonce: result = 0x82
                else: self.heartbeat = now
                if not self.reply_heartbeat: continue
            elif op == 5:
                if value == self.nonce: self.stop(); self.nonce = 0
            elif op == 3: self.stop()
            elif op in (0x10, 0x11):
                if self.reject_move: self.ack(op, seq, 0x85, 0); continue
                if not self.nonce: result = 0x82
                elif not self.ready: result = 1
                else:
                    self.moves.append(value); self.ready = False
                    self.prepare = (now, op, seq, value)
                    self.ack(op, seq, 0xfe, value); continue
            else: self.fail('Unexpected opcode, including calibration/reset: ' + str(op))
            self.ack(op, seq, result, reply)

    def step(self):
        now = time.monotonic()
        self.quest.receive_mono_ns = time.monotonic_ns()
        self.publisher.publish(self.quest)
        if self.coordinator is not None:
            message = String()
            message.data = json.dumps({'epoch': 'interrupted-test:1', 'phase': 'active',
                                       'sent_mono_ns': str(time.monotonic_ns())})
            self.coordinator.publish(message)
        self.commands(now)
        if self.nonce and now - self.heartbeat > .5: self.stop(); self.nonce = 0
        if self.prepare and now - self.prepare[0] >= self.prepare_delay:
            _, op, seq, value = self.prepare; self.prepare = None
            self.target = value; self.moving = True; self.status_code = 2
            self.ack(op, seq, 0, value)
        if self.telemetry and now - self.last_sample >= .1:
            self.last_sample = now; self.sequence = (self.sequence + 1) & 65535
            age = 800 if self.prepare or self.motor_stale else 60
            pages = [struct.pack('<Hih', self.sequence, self.position, 20 if self.moving else 0),
                     struct.pack('<HiBB', self.sequence, self.target, self.status_code, self.flags()),
                     struct.pack('<HHBBBB', self.sequence, 0, 1, 1, 0, 4),
                     struct.pack('<HIH', self.sequence, int((now-self.started)*1000), age)]
            if self.corrupt_once:
                bad = bytearray(frame(0x90, b'\x00' + pages[0])); bad[-1] ^= 1
                os.write(self.master, bad); self.corrupt_once = False
            for page, payload in enumerate(pages): self.send(0x90, bytes([page]) + payload)
        rclpy.spin_once(self.node, timeout_sec=.005)
        time.sleep(.005)

    def run_for(self, duration):
        end = time.monotonic() + duration
        while time.monotonic() < end: self.step()

    def wait(self, condition, timeout, description):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.step()
            if condition(): return
        self.log.flush(); self.log.seek(0)
        self.fail(description + ': ' + repr(self.status) + '\n' + self.log.read())

    def test_hold_release_and_reverse_preserve_permission(self):
        self.quest.grips = [0., 1.]
        self.wait(lambda: self.moving, 4, 'up after locked startup')
        self.run_for(.3)
        self.assertTrue(self.status['enabled'])
        self.assertEqual(self.moves, [175000])
        self.quest.grips = [0., 0.]
        self.wait(lambda: not self.nonce and self.status.get('phase') == 'idle', 3, 'release stop')
        self.assertTrue(self.status['enabled'])
        self.run_for(.2); self.quest.grips = [1., 0.]
        self.wait(lambda: self.moving and len(self.moves) == 2, 4, 'down while center sensor active')
        self.assertEqual(self.moves[-1], -92000)
        self.quest.grips = [1., 1.]
        self.wait(lambda: not self.nonce, 2, 'both grips stop')
        self.assertTrue(self.status['enabled'])

    def test_releasing_during_queued_start_cancels(self):
        self.quest.grips = [0., 1.]
        self.wait(lambda: self.prepare is not None, 2, 'queued request')
        self.run_for(.35)  # Release while a real startup-age exception is active.
        self.quest.grips = [0., 0.]
        self.run_for(1.5)
        self.assertFalse(self.moving); self.assertFalse(self.nonce)
        self.assertTrue(self.status['enabled']); self.assertEqual(len(self.moves), 1)

    def test_stale_motor_after_brake_release_stops(self):
        self.quest.grips = [0., 1.]
        self.wait(lambda: self.moving, 4, 'moving')
        self.motor_stale = True
        self.wait(lambda: not self.status.get('enabled', True), 1, 'stale motor rejected')
        self.assertFalse(self.nonce)

    def test_zero_echo_rejection_is_reported(self):
        self.reject_move = True; self.quest.grips = [0., 1.]
        self.wait(lambda: not self.status.get('enabled', True), 3, 'explicit rejection')
        self.assertIn('result=133', self.status['reason'])
        self.assertNotIn('timeout', self.status['reason'])

    def test_ready_zero_waits_without_sending_move(self):
        self.ready = False; self.quest.grips = [0., 1.]
        self.run_for(.8)
        self.assertEqual(self.moves, [])
        self.assertTrue(self.status['enabled'])
        self.ready = True
        self.wait(lambda: self.moving, 4, 'READY followed by motion acceptance')

    def test_preparation_grace_cannot_exceed_two_seconds(self):
        self.prepare_delay = 2.5; self.quest.grips = [0., 1.]
        self.wait(lambda: self.prepare is not None, 2, 'queued request')
        self.wait(lambda: not self.status.get('enabled', True), 3, 'bounded preparation timeout')
        self.assertFalse(self.moving)
        self.assertEqual(len(self.moves), 1)

    def test_crc_error_recovers_without_sticky_guard_stop(self):
        self.corrupt_once = True; self.run_for(.5)
        self.assertTrue(self.status['feedback_healthy'])
        self.assertGreaterEqual(self.status['uart_parser_errors'], 1)

    def test_lost_uart_feedback_never_replays(self):
        self.quest.grips = [0., 1.]
        self.wait(lambda: self.moving, 4, 'moving')
        self.telemetry = False
        self.wait(lambda: not self.status.get('enabled', True), 1, 'feedback loss')
        self.telemetry = True; self.run_for(.7)
        self.assertFalse(self.status['enabled']); self.assertEqual(len(self.moves), 1)

    def assert_usb_disconnect_exits(self):
        # Exercise the real UART poll/receive path. Keeping the process alive
        # on a dead fd prevents systemd Restart=on-failure from reopening USB.
        os.close(self.master); self.master = None
        try:
            code = self.process.wait(timeout=2)
        except subprocess.TimeoutExpired:
            self.fail('adapter kept running on a disconnected UART descriptor')
        self.assertEqual(code, 1, 'disconnect must request a supervised service restart')
        self.log.flush(); self.log.seek(0)
        # An unplug may occur between poll() and read(): Linux then reports
        # EIO instead of POLLHUP. Both must exit, rather than loop on the fd.
        self.assertRegex(self.log.read(), r'UART (USB disconnected|read: Input/output error)')

    def test_usb_disconnect_while_idle_exits_for_supervisor(self):
        self.assertEqual(self.moves, [])
        self.assert_usb_disconnect_exits()

    def test_usb_disconnect_during_move_exits_for_supervisor(self):
        self.quest.grips = [0., 1.]
        self.wait(lambda: self.moving, 4, 'moving before USB disconnect')
        self.assertEqual(len(self.moves), 1)
        self.assert_usb_disconnect_exits()

    def restart_and_assert_disabled(self, shared=False):
        # Model the supervisor opening a newly enumerated USB port. Keep the
        # grip held and, for shared mode, keep the old active epoch arriving.
        previous_requests = len(self.requests)
        previous_moves = list(self.moves)
        old_port = self.port
        os.close(self.slave)
        self.master, self.slave = os.openpty()
        tty.setraw(self.slave)
        self.port = os.ttyname(self.slave)
        self.assertTrue(self.port.startswith('/dev/pts/'))
        os.set_blocking(self.master, False)
        self.buffer.clear(); self.status.clear()
        self.stop(); self.nonce = 0  # Lost lease, no pending device task.
        self.last_sample = 0.
        restarted_ns = time.monotonic_ns()
        command = [value.replace('serial_port:=' + old_port, 'serial_port:=' + self.port)
                   for value in self.process.args]
        if shared:
            command = ['shared_enable:=true' if value == 'shared_enable:=false' else value
                       for value in command]
            self.coordinator = self.node.create_publisher(String, self.prefix + '/coordinated_control', 10)
        self.process = subprocess.Popen(command, stdout=self.log, stderr=subprocess.STDOUT,
                                        env=dict(os.environ, ROS_LOG_DIR=self.tmp.name))
        self.wait(lambda: self.status.get('feedback_healthy') and
                  int(self.status.get('receive_mono_ns', '0')) >= restarted_ns,
                  5, 'fresh feedback after reconnect')
        self.run_for(.8)
        self.assertFalse(self.status['enabled'])
        self.assertFalse(self.status['session'])
        self.assertEqual(self.status['phase'], 'disabled')
        self.assertEqual(self.requests[previous_requests:], [])
        self.assertEqual(self.moves, previous_moves)
        if shared:
            self.assertEqual(self.status['shared_phase'], 'active')
            self.assertTrue(self.status['shared_failed'])

    def test_reconnect_with_held_grip_does_not_replay_move(self):
        self.quest.grips = [0., 1.]
        self.wait(lambda: self.moving, 4, 'moving before reconnect')
        self.assert_usb_disconnect_exits()
        self.restart_and_assert_disabled()

    def test_reconnect_ignores_old_shared_active_epoch(self):
        self.quest.grips = [0., 1.]
        self.wait(lambda: self.moving, 4, 'moving before shared reconnect')
        self.assert_usb_disconnect_exits()
        self.restart_and_assert_disabled(shared=True)


if __name__ == '__main__':
    rclpy.init()
    try: unittest.main(argv=[__file__] + remaining)
    finally: rclpy.shutdown()
