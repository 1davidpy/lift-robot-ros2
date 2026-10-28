#!/usr/bin/env python3
"""Real coordinator + real adapter, mock motors only, in an isolated ROS domain.

Usage: test_manual_pose_handover.py MANAGER_BINARY ADAPTER_BINARY CONFIG_JSON
No CAN interfaces are opened. Base/lift acknowledgements and Quest/IK inputs are
synthetic; both independent and shared coordinator modes use the real arm path.
"""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import threading
import time
import unittest

os.environ['ROS_DOMAIN_ID'] = str(120 + os.getpid() % 80)
os.environ['ROS_LOCALHOST_ONLY'] = '1'

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from luxi_interfaces.msg import ArmCommand, ArmState, QuestInput
from luxi_interfaces.srv import Command
from std_msgs.msg import String

if len(sys.argv) != 4:
    raise SystemExit(__doc__)
MANAGER, ADAPTER, CONFIG = (Path(value).resolve() for value in sys.argv[1:])
CONFIG_DATA = json.loads(CONFIG.read_text())
HOME = [min(max(value, low), high)
        for side in ('right', 'left')
        for value, (low, high) in zip(CONFIG_DATA[side]['home'],
                                       CONFIG_DATA[side]['limits'])]


class ManualPoseHandover(unittest.TestCase):
    def run_case(self, shared):
        self.shared = shared
        self.tmp = tempfile.TemporaryDirectory(prefix='luxi-manual-handover-')
        self.logs = []
        self.processes = []
        self.addCleanup(self.cleanup)
        self.node = rclpy.create_node('manual_handover_test')
        self.prefix = '/manual_handover_' + str(time.monotonic_ns())
        self.state = None
        self.state_history = []
        self.status = {}
        self.startup = {}
        self.directive = {}
        self.prepare_since = 0.
        self.base_stationary = True
        self.control_instance = 0
        self.last_device_inputs = {}
        self.last_quest_input = {}
        self.last_intent_input = {}
        self.status_received_ns = 0
        self.block_start_ack = False
        self.relay_held_template = None
        self.relay_blocked_ack_count = 0
        self.last_pose_enter_ns = 0
        self.last_manager_suspend_ns = 0
        self.command_windows = {}
        self.hold_first_source_ns = {}
        self.feedback_failure_diagnostic = None
        self.device_publish_max_gap_ms = 0.
        self.quest_publish_max_block_ms = 0.
        self.commands = []
        self.applied = []
        self.lifecycle = []
        self.status_history = []
        self.startup_history = []
        self.publisher_error = None
        self.x_pressed = False
        self.initial = HOME.copy()
        self.initial[0] = min(HOME[0] + .04, CONFIG_DATA['right']['limits'][0][1] - .01)
        self.initial[8] = max(HOME[8] - .04, CONFIG_DATA['left']['limits'][0][0] + .01)
        self.intent_position = self.initial.copy()
        self.qp = self.node.create_publisher(QuestInput, self.topic('quest'), 1)
        self.ip = self.node.create_publisher(ArmCommand, self.topic('intent'), 1)
        self.inject_command = self.node.create_publisher(ArmCommand, self.topic('command'), 1)
        self.inject_life = self.node.create_publisher(String, self.topic('driver_lifecycle'), 10)
        self.bp = self.node.create_publisher(String, self.topic('base/status'), 1)
        self.lp = self.node.create_publisher(String, self.topic('lift/status'), 1)
        self.startup_relay = self.node.create_publisher(String, self.topic('manager/startup_status'), 1)
        self.node.create_subscription(ArmState, self.topic('arms/state'), self.on_state, 1)
        self.node.create_subscription(ArmCommand, self.topic('command'), self.commands.append, 100)
        self.node.create_subscription(ArmCommand, self.topic('applied'), self.applied.append, 100)
        self.node.create_subscription(String, self.topic('driver_lifecycle'),
                                      lambda m: self.lifecycle.append((time.monotonic_ns(), m.data)), 100)
        self.node.create_subscription(String, self.topic('control_status'), self.on_status,
                                      QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.node.create_subscription(String, self.topic('arms/startup_status'), self.on_startup, 1)
        self.node.create_subscription(String, self.topic('coordinated_control'), self.on_directive, 1)
        self.control = self.node.create_client(Command, self.control_topic())
        self.startup_control = self.node.create_client(Command, self.topic('arms/startup'))
        self.spawn_adapter()
        self.spawn_manager()
        self.publisher_stop = threading.Event()
        self.publisher_thread = threading.Thread(target=self.publish_loop, daemon=True)
        self.device_thread = threading.Thread(target=self.publish_device_loop, daemon=True)
        self.publisher_thread.start()
        self.device_thread.start()
        self.until(lambda: self.control.service_is_ready() and self.startup_control.service_is_ready()
                   and self.state is not None and all(self.state.valid)
                   and self.status.get('quest_window_samples', 0) >= 50,
                   6, 'ready mock arm/coordinator and 50 fresh Quest packets')

        # Create a real prior controller session with a non-home target. An edit
        # must interrupt this session instead of replaying its buffered intent.
        self.request(self.control, 'start')
        self.until(lambda: self.status.get('phase') == 'teleop'
                   and (not shared or self.status.get('system_phase') == 'active'),
                   9, 'initial explicit start reaches teleop')
        self.run_for(.3)
        self.request(self.startup_control, 'pose_enter')
        self.until(lambda: self.manual_phase() == 'editing'
                   and self.status.get('manual_pose_active') is True
                   and not self.state.enabled, 3, 'editing disables both arms and suspends manager')
        self.run_for(.1)  # Drain commands issued before the edit status arrived.
        edit_position = list(self.state.position)
        command_index = self.command_checkpoint('first editing after manager suspension')
        applied_index, life_index = len(self.applied), len(self.lifecycle)
        self.x_pressed = True
        self.request(self.control, 'start', expected=False)
        self.inject_life.publish(String(data='enable'))
        self.publish_old_command()
        self.run_for(1.7)
        self.assertEqual(self.manual_phase(), 'editing')
        self.assertFalse(self.state.enabled, 'held X/queued enable must not enable editing arms')
        self.assert_pose(self.state.position, edit_position, .00001)
        self.assert_no_manager_commands(command_index)
        edit_fence = self.command_windows[command_index]['source_fence_ns']
        self.assertFalse([{'phase': c.phase, 'source_mono_ns': c.source_mono_ns}
                          for c in self.applied if c.source_mono_ns <= 0 or c.source_mono_ns >= edit_fence],
                         'disabled edit must not submit post-suspension arm targets')
        self.assertFalse(any(value.startswith('enable:') for _, value in self.lifecycle[life_index:]))
        if shared:
            self.assertEqual(self.status.get('system_phase'), 'stopped')
            self.assertEqual(self.directive.get('phase'), 'halt')

        self.request(self.startup_control, 'pose_hold')
        self.until(lambda: self.manual_phase() == 'holding' and self.state.enabled
                   and self.status.get('manual_pose_phase') == 'holding',
                   4, 'stable post-click feedback enables original pose hold')
        generation = self.startup['manual_pose']['generation']
        self.assertTrue(generation)
        held = list(self.startup['manual_pose']['target'])
        self.assert_pose(held, edit_position, .00001)
        self.run_for(.1)
        command_index = self.command_checkpoint('holding without an explicit start')
        life_index = len(self.lifecycle)
        # These simulate lifecycle messages/targets queued before a new explicit
        # start. Neither a plain enable nor another generation owns this hold.
        self.inject_life.publish(String(data='enable'))
        self.inject_life.publish(String(data='enable:previous-generation'))
        self.publish_old_command()
        holding_count = len(self.startup_history)
        self.run_for(1.7)
        self.assertEqual(self.manual_phase(), 'holding')
        self.assert_pose(self.state.position, held, .00001)
        self.assert_no_manager_commands(command_index)
        self.assertGreater(sum(j.get('manual_pose', {}).get('phase') == 'holding'
                               for j in self.startup_history[holding_count:]), 10)
        self.assertFalse(any(value == 'enable:' + generation
                             for _, value in self.lifecycle[life_index:]),
                         'X held through editing/holding must not become a new start')

        # Starting a new manager while the driver already owns a hold also has
        # to suspend once, even though this process never saw editing/preparing.
        self.x_pressed = False
        self.run_for(.2)
        self.stop_process(self.manager_process)
        # A service client can keep the old DDS server endpoint during graph
        # rediscovery. Give this process incarnation its own remapped service,
        # then discover a newly created client before issuing an operation.
        self.node.destroy_client(self.control)
        self.control_instance += 1
        self.control = self.node.create_client(Command, self.control_topic())
        self.status = {}
        self.spawn_manager()
        self.until(lambda: self.control.service_is_ready()
                   and self.status.get('manual_pose_phase') == 'holding'
                   and self.status.get('phase') == 'disabled'
                   and self.status.get('system_phase') == 'stopped'
                   and self.status.get('quest_window_samples', 0) >= 50,
                   6, 'restarted manager adopts an existing hold without a target')
        self.run_for(.1)
        command_index = self.command_checkpoint('restarted manager holds without a start')
        self.run_for(.25)
        self.assert_no_manager_commands(command_index)
        self.assert_pose(self.state.position, held, .00001)

        self.intent_position = HOME.copy()
        lifecycle_index, applied_index = len(self.lifecycle), len(self.applied)
        self.request(self.control, 'start')
        old_command_stamp = self.last_request_sent_ns
        if shared:
            # Fake devices acknowledge prepare immediately, but remain unready
            # for 450 ms. Real repeated holding status must not cancel start.
            self.until(lambda: self.status.get('system_phase') == 'preparing',
                       2, 'shared explicit start accepted')
            repeated_from = len(self.startup_history)
            self.run_for(.22)
            self.assertEqual(self.status.get('system_phase'), 'preparing')
            self.assertEqual(self.manual_phase(), 'holding')
            self.assertGreater(sum(j.get('manual_pose', {}).get('phase') == 'holding'
                                   for j in self.startup_history[repeated_from:]), 5)
        self.until(lambda: self.manual_phase() == 'inactive'
                   and any(value == 'enable:' + generation
                           for _, value in self.lifecycle[lifecycle_index:]),
                   4, 'explicit start hands over with the captured generation')
        new_lifecycle = [value for _, value in self.lifecycle[lifecycle_index:]]
        self.assertNotIn('enable', new_lifecycle, 'manual hold handover must use its generation')
        # A delayed command stamped before handover must not replace the support
        # target after the driver releases ownership. It is still freshly dated.
        self.publish_old_command(stamp=old_command_stamp)
        self.until(lambda: any(c.phase != 'manual_pose_hold'
                               for c in self.applied[applied_index:]),
                   2, 'first post-handover arm target')
        first = next(c for c in self.applied[applied_index:] if c.phase != 'manual_pose_hold')
        # The manager clamps elapsed time to 50 ms and home speed to .2 rad/s.
        # Allow a small observation boundary margin over that maximum first step.
        self.assert_pose(first.position, held, .013)
        self.until(lambda: self.status.get('phase') == 'teleop'
                   and (not shared or self.status.get('system_phase') == 'active'),
                   8, 'new explicit session reaches teleop after manual handover')
        self.assertTrue(self.state.enabled)
        self.assertEqual(self.manual_phase(), 'inactive')
        self.assert_pose(self.state.position, HOME, .02)
        self.assertFalse(any(c.phase == 'stale_external' for c in self.applied[applied_index:]),
                         'queued target must not be applied after handover')

        # A stop/park is also an explicit handover. Re-enter a non-home hold so
        # a return that merely runs its timer and drops torque cannot pass.
        self.intent_position = self.initial.copy()
        self.until(lambda: max(abs(a - b) for a, b in zip(self.state.position, self.initial)) < .004,
                   3, 'non-home pose before stop/park regression')
        self.request(self.startup_control, 'pose_enter')
        self.until(lambda: self.manual_phase() == 'editing' and not self.state.enabled
                   and self.status.get('manual_pose_active') is True,
                   3, 'second edit suspends the active controller')
        self.request(self.startup_control, 'pose_hold')
        self.until(lambda: self.manual_phase() == 'holding' and self.state.enabled
                   and self.status.get('manual_pose_phase') == 'holding',
                   4, 'second non-home pose held before stop/park')
        return_generation = self.startup['manual_pose']['generation']
        self.assertNotEqual(return_generation, generation)
        return_held = list(self.startup['manual_pose']['target'])
        distance = max(abs(a - b) for a, b in zip(return_held, HOME))
        self.assertGreater(distance, .025, 'stop/park fixture must require an actual return')
        lifecycle_index, applied_index = len(self.lifecycle), len(self.applied)
        command_index = self.command_checkpoint('explicit return from manual hold')
        state_index = len(self.state_history)
        self.request(self.control, 'stop')
        self.until(lambda: self.status.get('phase') == 'disabled' and not self.state.enabled
                   and self.manual_phase() == 'inactive'
                   and (not shared or self.status.get('system_phase') == 'disabled'),
                   7, 'stop/park hands over, returns the arms, then disables torque')
        values = [value for _, value in self.lifecycle[lifecycle_index:]]
        self.assertIn('enable:' + return_generation, values,
                      'stop/park must acquire the manual hold before its return')
        disabled = next((value for value in values if value.startswith('disable:')), None)
        self.assertIsNotNone(disabled, 'explicit park must use the current driver ownership token')
        self.assertLess(values.index('enable:' + return_generation), values.index(disabled))
        submitted = [c for c in self.applied[applied_index:] if c.phase == 'returning']
        requested = [c for c in self.commands[command_index:] if c.phase == 'returning']
        self.assertTrue(submitted, 'the mock adapter must actually apply the return trajectory')
        self.assertTrue(requested)
        self.assert_pose(submitted[0].position, return_held, .013)
        self.assert_pose(requested[0].position, return_held, .013)
        self.assertTrue(any(s.enabled
                            and max(abs(a - b) for a, b in zip(s.position, HOME)) < distance * .6
                            for s in self.state_history[state_index:]),
                        'measured arm position must move toward home before torque is removed')
        self.assert_pose(self.state.position, HOME, .005)

        if shared:
            # Another existing arm entry point may acquire the pose while park
            # is still waiting for the chassis. An inactive manual phase alone
            # is not a lifecycle handover acknowledgement: startup now owns it.
            self.request(self.control, 'start')
            self.until(lambda: self.status.get('phase') == 'teleop'
                       and self.status.get('system_phase') == 'active'
                       and max(abs(a - b) for a, b in zip(self.state.position, self.initial)) < .004,
                       9, 'non-home shared session before competing owner regression')
            self.request(self.startup_control, 'pose_enter')
            self.until(lambda: self.manual_phase() == 'editing' and not self.state.enabled
                       and self.status.get('manual_pose_active') is True,
                       3, 'third edit suspends shared teleoperation')
            self.request(self.startup_control, 'pose_hold')
            self.until(lambda: self.manual_phase() == 'holding' and self.state.enabled
                       and self.status.get('manual_pose_phase') == 'holding',
                       4, 'third pose held before a competing startup owner')
            competing_held = list(self.startup['manual_pose']['target'])
            self.assertGreater(max(abs(a - b) for a, b in zip(competing_held, HOME)), .025)
            self.base_stationary = False
            self.run_for(.12)
            lifecycle_index, applied_index = len(self.lifecycle), len(self.applied)
            command_index = self.command_checkpoint('park interrupted by a different owner')
            self.request(self.control, 'stop')
            self.until(lambda: self.status.get('system_phase') == 'stopping_base',
                       2, 'park waits for the base to stop')
            self.assertEqual(self.manual_phase(), 'holding')
            self.request(self.startup_control, 'stop')
            self.until(lambda: self.manual_phase() == 'inactive'
                       and self.startup.get('owned') is True
                       and self.startup.get('phase') == 'hold'
                       and self.status.get('manual_pose_phase') == 'inactive',
                       3, 'startup stop explicitly takes ownership of the held pose')
            self.base_stationary = True
            self.until(lambda: self.status.get('system_phase') == 'stopped'
                       and self.status.get('reason') == 'manual_pose_changed_before_return',
                       3, 'park aborts when startup took over before lifecycle handover')
            self.run_for(2.4)  # Longer than the entire nominal legacy return.
            self.assertTrue(self.state.enabled, 'an aborted park must retain support torque')
            self.assertTrue(self.startup.get('owned'))
            self.assertEqual(self.status.get('system_phase'), 'stopped')
            self.assert_pose(self.state.position, competing_held, .00001)
            self.assertFalse(any(value.startswith('disable')
                                 for _, value in self.lifecycle[lifecycle_index:]),
                             'a competing owner must never finish an empty park by disabling arms')
            self.assertFalse(any(c.phase == 'returning' for c in self.commands[command_index:]),
                             'park must abort before publishing its unowned return')
            self.assertFalse(any(c.phase == 'returning' for c in self.applied[applied_index:]))

            # Drop only the lifecycle acknowledgement on the manager's status
            # relay. The real mock adapter still accepts the correct generation
            # and supplies live ArmState, so a missing ACK is the sole cause.
            self.request(self.startup_control, 'pose_enter')
            self.until(lambda: self.manual_phase() == 'editing' and not self.state.enabled
                       and self.status.get('manual_pose_active') is True,
                       3, 'new edit before missing-start-ACK regression')
            self.request(self.startup_control, 'pose_hold')
            self.until(lambda: self.manual_phase() == 'holding' and self.state.enabled
                       and self.status.get('manual_pose_phase') == 'holding',
                       4, 'hold captured before start acknowledgement is withheld')
            ack_held = list(self.startup['manual_pose']['target'])
            ack_generation = self.startup['manual_pose']['generation']
            self.run_for(.1)
            self.block_start_ack = True
            blocked_before = self.relay_blocked_ack_count
            lifecycle_index = len(self.lifecycle)
            command_index = self.command_checkpoint('start without ownership acknowledgement')
            self.request(self.control, 'start')
            self.until(lambda: self.manual_phase() == 'inactive'
                       and any(value == 'enable:' + ack_generation
                               for _, value in self.lifecycle[lifecycle_index:]),
                       3, 'real mock adapter accepts token while manager ACK is withheld')
            self.until(lambda: self.status.get('reason') == 'manual_pose_start_handover_timeout'
                       and self.status.get('phase') == 'disabled'
                       and self.status.get('system_phase') == 'stopped',
                       3, 'unacknowledged start times out without a trajectory')
            self.assertGreater(self.relay_blocked_ack_count - blocked_before, 10)
            self.assert_no_manager_commands(command_index)
            self.assertTrue(self.state.enabled, 'missing ACK must not automatically remove torque')
            self.assert_pose(self.state.position, ack_held, .00001)
            self.assertFalse(any(value.startswith('disable')
                                 for _, value in self.lifecycle[lifecycle_index:]))
            self.block_start_ack = False
            self.run_for(.3)
            self.assertEqual(self.status.get('phase'), 'disabled', 'late ACK must not restart homing')
            self.assertEqual(self.status.get('system_phase'), 'stopped')
            self.assert_no_manager_commands(command_index)
            self.assertTrue(self.state.enabled)
            self.assert_pose(self.state.position, ack_held, .00001)

            # Explicit shared disable/halt must cancel an unacknowledged start
            # without claiming ownership or removing the driver's support torque.
            self.request(self.startup_control, 'pose_enter')
            self.until(lambda: self.manual_phase() == 'editing' and not self.state.enabled
                       and self.status.get('manual_pose_active') is True,
                       3, 'new edit before pending-start halt regression')
            self.request(self.startup_control, 'pose_hold')
            self.until(lambda: self.manual_phase() == 'holding' and self.state.enabled
                       and self.status.get('manual_pose_phase') == 'holding',
                       4, 'hold captured before pending start is explicitly stopped')
            halt_held = list(self.startup['manual_pose']['target'])
            halt_generation = self.startup['manual_pose']['generation']
            self.run_for(.1)
            self.block_start_ack = True
            lifecycle_index = len(self.lifecycle)
            command_index = self.command_checkpoint('explicit halt during pending start')
            self.request(self.control, 'start')
            self.until(lambda: self.manual_phase() == 'inactive'
                       and any(value == 'enable:' + halt_generation
                               for _, value in self.lifecycle[lifecycle_index:]),
                       3, 'token accepted while explicit halt still has no ownership ACK')
            self.request(self.control, 'disable')
            self.until(lambda: self.status.get('phase') == 'disabled'
                       and self.status.get('system_phase') == 'stopped'
                       and self.status.get('reason') == 'operator_stop_all',
                       2, 'shared disable cancels pending start without claiming its target')
            self.assert_no_manager_commands(command_index)
            self.assertTrue(self.state.enabled)
            self.assert_pose(self.state.position, halt_held, .00001)
            self.assertFalse(any(value.startswith('disable')
                                 for _, value in self.lifecycle[lifecycle_index:]),
                             'shared halt must not become a driver torque-disable request')
            self.block_start_ack = False
            self.run_for(.3)
            self.assertEqual(self.status.get('phase'), 'disabled')
            self.assertEqual(self.status.get('system_phase'), 'stopped')
            self.assert_no_manager_commands(command_index)
            self.assertTrue(self.state.enabled)
            self.assert_pose(self.state.position, halt_held, .00001)
        print('PASS: %s real manager/mock adapter: edit interlock, held-X rejection, '
              'captured pose, existing-hold restart, generation handover, continuous first target, '
              'applied stop/park return before disable'
              % ('shared' if shared else 'independent'))

    def topic(self, suffix):
        return self.prefix + '/' + suffix

    def control_topic(self):
        return self.topic('control_instance_' + str(self.control_instance))

    def remaps(self, manager=False):
        args = []
        for suffix in ('quest', 'intent', 'command', 'applied', 'driver_lifecycle',
                       'control', 'control_status', 'coordinated_control', 'base/status',
                       'lift/status', 'recorder/status', 'arms/state', 'arms/startup',
                       'arms/startup_status', 'faults', 'shutdown'):
            target = self.control_topic() if suffix == 'control' else self.topic(suffix)
            if manager and suffix == 'arms/startup_status':
                target = self.topic('manager/startup_status')
            args += ['-r', '/luxi/' + suffix + ':=' + target]
        return args

    def spawn(self, binary, parameters, manager=False):
        log = open(Path(self.tmp.name) / ('process-%d.log' % len(self.logs)), 'w+')
        self.logs.append(log)
        process = subprocess.Popen([str(binary), '--ros-args', *parameters, *self.remaps(manager)],
                                   env=dict(os.environ, ROS_LOG_DIR=self.tmp.name),
                                   stdout=log, stderr=subprocess.STDOUT)
        self.processes.append(process)
        return process

    def spawn_adapter(self):
        self.spawn(ADAPTER, ['-p', 'mock:=true', '-p', 'monitor_only:=false',
                             '-p', 'config:=' + str(CONFIG),
                             '-p', 'mock_initial_positions:=' + json.dumps(self.initial)])

    def spawn_manager(self):
        self.manager_process = self.spawn(MANAGER, ['-p', 'config:=' + str(CONFIG),
            '-p', 'shared_enable:=' + ('true' if self.shared else 'false'),
            '-p', 'home_tolerance:=0.1', '-p', 'home_joint_speed_limit:=0.2'], manager=True)

    def on_status(self, message):
        was_manual = self.status.get('manual_pose_active', False)
        self.status = json.loads(message.data)
        self.status_received_ns = time.monotonic_ns()
        if self.status.get('manual_pose_active') and not was_manual:
            self.last_manager_suspend_ns = self.status_received_ns
        self.status_history.append(self.status)
        if self.status.get('reason') == 'base_or_lift_feedback_lost' and self.feedback_failure_diagnostic is None:
            now = time.monotonic_ns()
            inputs = self.last_device_inputs
            self.feedback_failure_diagnostic = {
                'observed_mono_ns': now,
                'device_publish_max_gap_ms': self.device_publish_max_gap_ms,
                'latest_fake_status': inputs,
                'fake_feedback_ages_ms': {
                    name: (now - int(value['receive_mono_ns'])) / 1e6
                    for name, value in inputs.items()},
                'control_status': self.status}

    def on_state(self, message):
        self.state = message
        self.state_history.append(message)

    def on_startup(self, message):
        self.startup = json.loads(message.data)
        self.startup_history.append(self.startup)
        if self.manual_phase() == 'holding':
            generation = self.startup['manual_pose']['generation']
            self.hold_first_source_ns.setdefault(generation, int(self.startup['receive_mono_ns']))
            self.relay_held_template = self.startup
        if self.block_start_ack and self.manual_phase() == 'inactive' and self.relay_held_template is not None:
            # Keep a fresh holding heartbeat while withholding the transition.
            # No motor command or ArmState is fabricated by this fault injector.
            value = json.loads(json.dumps(self.relay_held_template))
            value['receive_mono_ns'] = self.startup['receive_mono_ns']
            self.relay_blocked_ack_count += 1
            self.startup_relay.publish(String(data=json.dumps(value)))
        else:
            self.startup_relay.publish(message)

    def manual_phase(self):
        return self.startup.get('manual_pose', {}).get('phase')

    def on_directive(self, message):
        value = json.loads(message.data)
        if value.get('phase') == 'prepare' and (value.get('epoch'), value.get('phase')) != (
                self.directive.get('epoch'), self.directive.get('phase')):
            self.prepare_since = time.monotonic()
        self.directive = value

    def publish_loop(self):
        try:
            while not self.publisher_stop.is_set():
                started = time.monotonic()
                self.publish_inputs()
                self.publisher_stop.wait(max(0., .01 - (time.monotonic() - started)))
        except BaseException as error:
            self.publisher_error = error

    def publish_device_loop(self):
        # Device feedback must not share a potentially blocking reliable Quest/
        # intent write. Stamp each synthetic observation at its own publication,
        # rather than reusing the time captured before those other ROS writes.
        previous = time.monotonic()
        try:
            while not self.publisher_stop.is_set():
                started = time.monotonic()
                self.device_publish_max_gap_ms = max(self.device_publish_max_gap_ms,
                                                     (started - previous) * 1000.)
                previous = started
                self.publish_device_feedback()
                self.publisher_stop.wait(max(0., .01 - (time.monotonic() - started)))
        except BaseException as error:
            self.publisher_error = error

    def publish_inputs(self):
        stamp = time.monotonic_ns()
        quest = QuestInput()
        quest.receive_mono_ns = stamp
        quest.buttons = [False, False, self.x_pressed, False]
        quest.grips = [0., 0.]
        quest.sticks = [0., 0., 0., 0.]
        quest_publish_start = time.monotonic_ns()
        self.qp.publish(quest)
        self.quest_publish_max_block_ms = max(self.quest_publish_max_block_ms,
                                            (time.monotonic_ns() - quest_publish_start) / 1e6)
        self.last_quest_input = {'receive_mono_ns': stamp, 'buttons': list(quest.buttons),
                                 'grips': list(quest.grips), 'sticks': list(quest.sticks),
                                 'tracking': quest.tracking, 'left_tracking': quest.left_tracking,
                                 'right_tracking': quest.right_tracking}
        intent = ArmCommand()
        intent_stamp = time.monotonic_ns()
        intent.source_mono_ns = intent_stamp
        intent.position = self.intent_position
        intent.phase = 'teleop'
        self.ip.publish(intent)
        self.last_intent_input = {'source_mono_ns': intent_stamp, 'phase': intent.phase,
                                  'position': list(intent.position)}

    def publish_device_feedback(self):
        directive = self.directive
        phase = directive.get('phase', 'disabled')
        ready = phase in ('prepare', 'active') and time.monotonic() - self.prepare_since >= .45
        common = {'shared_enable': True, 'shared_failed': False,
                  'shared_epoch': directive.get('epoch', ''), 'shared_phase': phase,
                  'shared_ready': ready,
                  'enabled': phase == 'active', 'stationary': True}
        base_stamp = str(time.monotonic_ns())
        base = dict(common, receive_only=False, feedback_fresh=True,
                    receive_mono_ns=base_stamp, feedback_mono_ns=base_stamp,
                    stationary=self.base_stationary)
        self.bp.publish(String(data=json.dumps(base)))
        lift_stamp = str(time.monotonic_ns())
        lift = dict(common, allow_motion=True, feedback_healthy=True,
                    receive_mono_ns=lift_stamp, feedback_first_mono_ns=lift_stamp,
                    feedback_age_ms=30, session_present=False, input_recovery=False,
                    transport='uart', phase='idle', flags=0, fault=0,
                    speed_rpm=0, current_um=75321, home_complete=False)
        self.lp.publish(String(data=json.dumps(lift)))
        self.last_device_inputs = {'base': base, 'lift': lift}

    def publish_old_command(self, stamp=None):
        value = ArmCommand()
        value.source_mono_ns = time.monotonic_ns() if stamp is None else stamp
        value.phase = 'stale_external'
        value.position = HOME.copy()
        value.position[0] = min(HOME[0] + .5, CONFIG_DATA['right']['limits'][0][1] - .01)
        self.inject_command.publish(value)

    def request(self, client, command, expected=True):
        if client is self.control and command == 'start' and expected:
            # Discovery and an earlier packet-count observation do not prove the
            # restart has received all inputs at the instant this service runs.
            # Wait for the existing production guards, with freshness headroom;
            # never retry or relax a rejected operation.
            self.until(self.control_start_ready, 6, 'fresh inputs before explicit control start')
        if client is self.control and command in ('stop', 'park') and expected:
            self.until(self.control_park_ready, 4, 'post-hold feedback propagated before explicit park')
        before = self.diagnostic_snapshot()
        request = Command.Request()
        request.command = command
        self.last_request_sent_ns = time.monotonic_ns()
        if client is self.startup_control and command == 'pose_enter':
            self.last_pose_enter_ns = self.last_request_sent_ns
        future = client.call_async(request)
        self.until(future.done, 4, 'service reply: ' + command)
        response = future.result()
        self.assertEqual(response.success, expected,
                         {'command': command, 'message': response.message,
                          'before_request': before, 'after_response': self.diagnostic_snapshot()})
        return response

    def control_start_ready(self):
        now = time.monotonic_ns()
        status = self.status
        if not self.control.service_is_ready() or now - self.status_received_ns > 50000000:
            return False
        if status.get('manual_pose_active') or status.get('quest_window_samples', 0) < 60 or not status.get('vr_fresh'):
            return False
        if not 0 <= status.get('quest_age_ms', -1) < 60:
            return False
        if not 0 <= status.get('arm_feedback_age_ms', -1) < 60 or not status.get('arm_feedback_valid'):
            return False
        if self.state is None or not all(self.state.valid) or not 0 <= now - self.state.receive_mono_ns < 75000000:
            return False
        if self.last_quest_input.get('buttons') != [False, False, False, False]:
            return False
        if not self.shared:
            return status.get('phase') == 'disabled'
        if status.get('system_phase') not in ('disabled', 'stopped'):
            return False
        if self.bp.get_subscription_count() < 1 or self.lp.get_subscription_count() < 1:
            return False
        for value in self.last_device_inputs.values():
            if not 0 <= now - int(value['receive_mono_ns']) < 75000000:
                return False
            if value['shared_epoch'] != status.get('shared_epoch'):
                return False
        return len(self.last_device_inputs) == 2

    def control_park_ready(self):
        now = time.monotonic_ns()
        if not self.control.service_is_ready() or now - self.status_received_ns > 50000000:
            return False
        if self.status.get('manual_pose_active') or not self.status.get('arm_feedback_valid'):
            return False
        age_ms = self.status.get('arm_feedback_age_ms', -1)
        if not 0 <= age_ms < 60 or self.state is None or not self.state.enabled or not all(self.state.valid):
            return False
        if not 0 <= now - self.state.receive_mono_ns < 75000000:
            return False
        if self.manual_phase() == 'holding':
            generation = self.startup['manual_pose']['generation']
            held_at = self.hold_first_source_ns.get(generation, now)
            # StartupStatus and ArmState are separate ROS topics. A holding
            # transition can overtake the first enabled ArmState at the manager.
            # Require sustained post-hold observations before issuing one park.
            if now - held_at < 150000000 or self.state.receive_mono_ns < held_at + 100000000:
                return False
            manager_feedback_estimate = self.status_received_ns - int(age_ms * 1e6)
            if manager_feedback_estimate < held_at + 100000000:
                return False
        return True

    def diagnostic_snapshot(self):
        now = time.monotonic_ns()
        arm = None if self.state is None else {
            'enabled': self.state.enabled, 'valid': list(self.state.valid),
            'phase': self.state.phase, 'receive_age_ms': (now - self.state.receive_mono_ns) / 1e6,
            'position': list(self.state.position)}
        return {'now_mono_ns': now, 'control': self.status,
                'control_status_receive_age_ms': (now - self.status_received_ns) / 1e6,
                'arm': arm, 'startup': self.startup, 'quest_sent': self.last_quest_input,
                'intent_sent': self.last_intent_input, 'fake_devices': self.last_device_inputs,
                'directive': self.directive, 'device_publish_max_gap_ms': self.device_publish_max_gap_ms,
                'quest_publish_max_block_ms': self.quest_publish_max_block_ms,
                'hold_first_source_ns': self.hold_first_source_ns}

    def assert_pose(self, actual, expected, tolerance):
        self.assertEqual(len(actual), 16)
        self.assertLessEqual(max(abs(a - b) for a, b in zip(actual, expected)), tolerance,
                             (list(actual), list(expected)))

    def command_checkpoint(self, label):
        index = len(self.commands)
        observed_at = time.monotonic_ns()
        fence = self.last_manager_suspend_ns if self.last_manager_suspend_ns >= self.last_pose_enter_ns > 0 else observed_at
        self.command_windows[index] = {
            'label': label, 'delivery_index': index, 'source_fence_ns': fence,
            'checkpoint_observed_ns': observed_at,
            'pose_enter_request_ns': self.last_pose_enter_ns,
            'manager_suspend_observed_ns': self.last_manager_suspend_ns}
        return index

    def assert_no_manager_commands(self, start):
        suspicious = [c for c in self.commands[start:] if c.phase != 'stale_external']
        window = self.command_windows[start]
        fence = window['source_fence_ns']
        # Delivery order is not a source-time boundary. Only a positive source
        # stamp preceding the protected fence proves a delayed old message.
        # Check the full history for post-fence sources, including a message that
        # was already delivered before the observer recorded this checkpoint.
        violations = [c for c in self.commands if c.phase != 'stale_external'
                      and (c.source_mono_ns <= 0 or c.source_mono_ns >= fence)]
        if suspicious or violations:
            inspected = suspicious + [c for c in violations if c not in suspicious]
            details = [{'phase': c.phase, 'source_mono_ns': c.source_mono_ns,
                        'source_minus_fence_ms': (c.source_mono_ns - fence) / 1e6,
                        'source_minus_enter_ms': (c.source_mono_ns - window['pose_enter_request_ns']) / 1e6,
                        'source_minus_suspend_ms': (c.source_mono_ns - window['manager_suspend_observed_ns']) / 1e6}
                       for c in inspected]
            diagnostic = {
                'window': window, 'suspicious_count': len(inspected),
                'source_min_ns': min(c.source_mono_ns for c in inspected),
                'source_max_ns': max(c.source_mono_ns for c in inspected),
                'source_at_or_after_fence_count': len(violations),
                'suspicious_sources': details,
                'snapshot': self.diagnostic_snapshot()}
            print('COMMAND_WINDOW_DIAGNOSTIC ' + json.dumps(diagnostic), flush=True)
            self.assertFalse(violations, 'post-fence or unstamped manager command in a protected window: '
                             + json.dumps(diagnostic))

    def run_for(self, duration):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            if self.publisher_error is not None:
                raise self.publisher_error
            for process in self.processes:
                if process.poll() is not None and process is self.manager_process:
                    self.fail('active manager exited: ' + str(process.returncode))
            rclpy.spin_once(self.node, timeout_sec=.002)
            # Histories intentionally retain every command/applied sample, but
            # latest-state callbacks must not queue behind them for a second.
            for _ in range(15):
                rclpy.spin_once(self.node, timeout_sec=0)

    def until(self, predicate, timeout, description):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return
            self.run_for(.02)
        self.fail('%s: snapshot=%s feedback_diagnostic=%s' %
                  (description, self.diagnostic_snapshot(), self.feedback_failure_diagnostic))

    def stop_process(self, process):
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=5)

    def cleanup(self):
        if hasattr(self, 'publisher_stop'):
            self.publisher_stop.set()
            self.publisher_thread.join(timeout=2)
            self.device_thread.join(timeout=2)
        for process in self.processes:
            self.stop_process(process)
        for log in self.logs:
            log.seek(0)
            contents = log.read()
            if contents:
                print(contents[-6000:])
            log.close()
        if hasattr(self, 'node'):
            self.node.destroy_node()
        self.tmp.cleanup()

    def test_independent_handover(self):
        self.run_case(False)

    def test_shared_handover(self):
        self.run_case(True)


if __name__ == '__main__':
    rclpy.init()
    try:
        unittest.main(argv=[sys.argv[0]])
    finally:
        rclpy.shutdown()
