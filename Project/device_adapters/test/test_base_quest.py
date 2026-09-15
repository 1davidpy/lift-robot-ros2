#!/usr/bin/env python3
"""Exercise real Quest-to-base callbacks with a mock chassis in an isolated domain."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import unittest

os.environ['ROS_DOMAIN_ID'] = '99'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from luxi_interfaces.msg import QuestInput
from luxi_interfaces.srv import Command
from std_msgs.msg import String

BINARY = Path(sys.argv.pop(1)).resolve()

class BaseQuest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(prefix='luxi-base-quest-')
        self.log = open(Path(self.tmp.name) / 'adapter.log', 'w+')
        self.node = rclpy.create_node('base_quest_test')
        self.prefix = '/base_case_' + str(time.monotonic_ns())
        self.quest = QuestInput()
        self.quest.grips = [0., 0.]
        self.sending = True
        self.status = {}
        self.applied = {}
        self.process = None
        self.pub = self.node.create_publisher(QuestInput, self.prefix + '/quest', 1)
        self.node.create_subscription(String, self.prefix + '/base/status',
                                      lambda m: self.status.update(json.loads(m.data)), 10)
        self.node.create_subscription(String, self.prefix + '/base/applied',
                                      lambda m: self.applied.update(json.loads(m.data)), 10)
        self.client = self.node.create_client(Command, self.prefix + '/base/control')

    def launch(self, require_grip, receive_only=False):
        args = [str(BINARY), '--ros-args', '-p', 'mock:=true', '-p',
                'require_left_grip:=' + str(require_grip).lower()]
        if receive_only:
            args += ['-p', 'receive_only:=true']
        for topic in ['quest', 'recorder/status', 'base/state', 'base/can_rx',
                      'base/applied', 'base/status', 'base/control']:
            args += ['-r', f'/luxi/{topic}:={self.prefix}/{topic}']
        for topic in ['velocity', 'imu', 'battery']:
            args += ['-r', f'/base/{topic}:={self.prefix}/{topic}']
        self.process = subprocess.Popen(args, env=dict(os.environ, ROS_LOG_DIR=self.tmp.name),
                                        stdout=self.log, stderr=subprocess.STDOUT)
        self.run_for(1.2)
        self.wait_for(lambda: self.client.service_is_ready() and bool(self.status), 3,
                      'mock base startup')
        self.assertTrue(self.status['mock'])
        self.assertFalse(self.status['enabled'])

    def step(self):
        if self.sending:
            self.quest.receive_mono_ns = time.monotonic_ns()
            self.pub.publish(self.quest)
        for _ in range(4):
            rclpy.spin_once(self.node, timeout_sec=0)
        time.sleep(.006)

    def run_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.step()

    def wait_for(self, condition, seconds, description):
        end = time.monotonic() + seconds
        while not condition() and time.monotonic() < end:
            self.step()
        self.assertTrue(condition(), f'{description}: {self.status}, {self.applied}')

    def sync_input(self):
        sent_after = time.monotonic_ns()
        self.wait_for(lambda: int(self.applied.get('source_quest_mono_ns', 0)) >= sent_after,
                      1, 'base acknowledged current Quest input')

    def request(self, command):
        # ROS services can overtake topic updates. Assert enable conditions only
        # after the adapter acknowledges the current sticks/buttons, not a sleep.
        if command == 'start' and not self.status.get('receive_only', False):
            self.sync_input()
        req = Command.Request()
        req.command = command
        future = self.client.call_async(req)
        self.wait_for(future.done, 2, 'service response')
        return future.result().success

    def test_stale_pose_flags_allow_sticks_but_invalid_tracking_stops(self):
        self.quest.tracking = self.quest.left_tracking = self.quest.right_tracking = 1
        self.launch(True)
        self.assertTrue(self.request('start'), 'fresh STALE poses must not block joystick enable')
        self.quest.sticks = [0., 1., 0., 0.]
        self.quest.grips = [1., 0.]
        self.wait_for(lambda: self.applied.get('velocity', [0])[0] > .05, 2,
                      'STALE joystick motion')
        self.quest.right_tracking = 2
        self.wait_for(lambda: not self.applied['enabled'] and self.applied['velocity'] == [0., 0., 0.],
                      .25, 'INVALID stops motion')
        self.quest.right_tracking = 0
        self.run_for(.25)
        self.assertFalse(self.applied['enabled'], 'tracking recovery cannot rearm')

    def test_direct_sticks_ignore_grips_center_stops_and_loss_disarms(self):
        self.launch(False)
        self.quest.sticks = [0., 1., 0., 0.]
        self.run_for(.08)
        self.assertFalse(self.request('start'), 'nonneutral enable must fail')
        self.quest.sticks = [0., 0., 0., 0.]
        self.run_for(.08)
        self.assertTrue(self.request('start'))
        self.quest.sticks = [0., 1., 0., 0.]
        self.wait_for(lambda: self.applied.get('velocity', [0])[0] > .05, 2,
                      'direct sticks with released grips')
        self.quest.grips = [1., 1.]
        self.run_for(.1)
        self.assertGreater(self.applied['velocity'][0], .05, 'future lift grips must not gate base')
        self.quest.grips = [0., 0.]
        self.quest.sticks = [0., 0., 0., 0.]
        self.run_for(.08)
        self.assertEqual(self.applied['velocity'], [0., 0., 0.], 'center must submit zero promptly')
        self.quest.sticks = [1., 0., 1., 0.]
        self.wait_for(lambda: self.applied['velocity'][1] < -.05 and self.applied['velocity'][2] < -.05,
                      2, 'strafe and turn')
        self.sending = False
        self.run_for(.35)
        self.assertFalse(self.applied['enabled'])
        self.assertEqual(self.applied['velocity'], [0., 0., 0.])
        self.sending = True
        self.run_for(.25)
        self.assertFalse(self.applied['enabled'], 'reconnect cannot rearm')
        self.assertFalse(self.status['require_left_grip'])

    def test_stop_buttons_and_recording_menu_disarm_direct_mode(self):
        self.launch(False)
        for index in [1, 3]:
            self.quest.sticks = [0., 0., 0., 0.]
            self.quest.buttons = [False] * 4
            self.run_for(.1)
            self.assertTrue(self.request('start'))
            self.quest.sticks = [0., 1., 0., 0.]
            self.wait_for(lambda: self.applied.get('velocity', [0])[0] > .02, 1, 'motion')
            self.quest.buttons[index] = True
            self.wait_for(lambda: not self.applied['enabled'] and self.applied['velocity'] == [0., 0., 0.],
                          .2, 'B/Y stop')
        self.quest.sticks = [0., 0., 0., 0.]
        self.quest.buttons = [False] * 4
        self.run_for(.1)
        self.assertTrue(self.request('start'))
        self.menu_pub = self.node.create_publisher(String, self.prefix + '/recorder/status',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.run_for(.2)
        menu = String()
        menu.data = '{"failure_menu":true}'
        for _ in range(10):
            self.menu_pub.publish(menu)
            self.run_for(.02)
        self.assertFalse(self.applied['enabled'])
        self.assertFalse(self.request('start'))

    def test_y_toggles_once_per_press_and_stops_while_moving(self):
        self.launch(False)
        self.quest.buttons[3] = True
        self.wait_for(lambda: self.applied.get('enabled'), 1, 'Y enables base')
        self.run_for(1.6)
        self.assertTrue(self.applied['enabled'], 'holding Y must not repeat the toggle')
        self.quest.buttons[3] = False
        self.quest.sticks = [0., 1., 0., 0.]
        self.wait_for(lambda: self.applied['velocity'][0] > .02, 1, 'motion after Y enable')
        self.quest.buttons[3] = True
        self.wait_for(lambda: not self.applied['enabled'] and self.applied['velocity'] == [0., 0., 0.],
                      .2, 'Y stops without needing centered sticks')
        self.quest.sticks = [0., 0., 0., 0.]
        self.run_for(.3)
        self.assertFalse(self.applied['enabled'], 'held Y cannot re-enable after stopping')

    def test_y_rejected_enable_needs_release_and_new_press(self):
        self.launch(False)
        self.quest.sticks = [0., 1., 0., 0.]
        self.quest.buttons[3] = True
        self.run_for(.15)
        self.assertFalse(self.applied['enabled'], 'nonneutral Y enable must fail')
        self.quest.sticks = [0., 0., 0., 0.]
        self.run_for(.2)
        self.assertFalse(self.applied['enabled'], 'centering while holding Y cannot enable')
        self.quest.buttons[3] = False
        self.run_for(.1)
        self.quest.buttons[3] = True
        self.wait_for(lambda: self.applied.get('enabled'), 1, 'new neutral Y press enables')
        self.sending = False
        self.run_for(.35)
        self.assertFalse(self.applied['enabled'])
        self.sending = True
        self.run_for(.3)
        self.assertFalse(self.applied['enabled'], 'held Y across reconnect cannot rearm')
        self.quest.buttons[3] = False
        self.run_for(.1)
        self.quest.buttons[3] = True
        self.wait_for(lambda: self.applied.get('enabled'), 1, 'new press after reconnect enables')

    def test_y_startup_hold_stop_priority_and_keyboard_handoff(self):
        self.quest.buttons[3] = True
        self.launch(False)
        self.run_for(.2)
        self.assertFalse(self.applied['enabled'], 'startup with held Y cannot enable')
        self.quest.buttons = [False] * 4
        self.run_for(.1)
        self.quest.buttons[3] = self.quest.buttons[1] = True
        self.run_for(.15)
        self.assertFalse(self.applied['enabled'], 'B must override simultaneous Y')
        self.quest.buttons[1] = False
        self.run_for(.15)
        self.assertFalse(self.applied['enabled'], 'releasing B cannot turn held Y into enable')
        self.quest.buttons = [False] * 4
        self.run_for(.1)
        self.assertTrue(self.request('keyboard_start'))
        self.quest.buttons[3] = True
        self.run_for(.12)
        self.assertFalse(self.applied['enabled'], 'Y must disable keyboard permission too')
        self.quest.buttons[3] = False
        self.run_for(.1)
        self.quest.buttons[3] = True
        self.wait_for(lambda: self.status.get('enabled') and self.status.get('input_source') == 'quest',
                      1, 'Y selects Quest input when enabling')

    def test_x_does_not_change_base_permission_or_joystick_commands(self):
        self.launch(False)
        self.quest.buttons[2] = True
        self.run_for(1.6)
        self.assertFalse(self.applied['enabled'], 'holding X must not enable the base')
        self.quest.buttons[3] = True
        self.wait_for(lambda: self.applied.get('enabled'), 1, 'Y can enable while X is held')
        self.quest.buttons[3] = False
        self.quest.sticks = [0., 1., 0., 0.]
        self.wait_for(lambda: self.applied['velocity'][0] > .02, 1, 'joystick motion with X held')
        for duration in [.1, 1.6]:
            self.quest.buttons[2] = False
            self.run_for(.1)
            self.quest.buttons[2] = True
            self.run_for(duration)
            self.assertTrue(self.applied['enabled'], 'short/long X must not revoke base permission')
            self.assertGreater(self.applied['velocity'][0], .02, 'X must not stop joystick motion')
            self.quest.buttons[2] = False
            self.run_for(.1)
            self.assertTrue(self.applied['enabled'], 'releasing X must not revoke base permission')
        self.quest.buttons[2] = self.quest.buttons[3] = True
        self.wait_for(lambda: not self.applied['enabled'] and self.applied['velocity'] == [0., 0., 0.],
                      .2, 'Y still stops while X is held')

    def test_y_cannot_enable_receive_only_adapter(self):
        self.launch(False, receive_only=True)
        self.quest.buttons[3] = True
        self.run_for(.3)
        self.assertFalse(self.status['enabled'])
        self.assertTrue(self.status['receive_only'])
        self.assertFalse(self.request('start'))

    def tearDown(self):
        self.node.destroy_node()
        if self.process is not None:
            if self.process.poll() is None:
                self.process.send_signal(signal.SIGINT)
            try:
                self.process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait()
        self.log.close()
        self.tmp.cleanup()

if __name__ == '__main__':
    rclpy.init()
    try:
        unittest.main(verbosity=2)
    finally:
        if rclpy.ok():
            rclpy.shutdown()
