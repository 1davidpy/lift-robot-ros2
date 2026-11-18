#!/usr/bin/env python3
"""Exercise the actual OpenArm adapter in a local, isolated, mock-only process.

Usage: test_openarm_manual_pose.py <openarm_adapter> <arms.json>
No CAN interface, allow_hardware, HTTP server, or production ROS domain is used.
"""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time

os.environ['ROS_DOMAIN_ID'] = str(160 + os.getpid() % 40)
os.environ['ROS_LOCALHOST_ONLY'] = '1'

import rclpy
from luxi_interfaces.msg import ArmCommand, ArmState
from luxi_interfaces.srv import Command
from std_msgs.msg import String


class Harness:
    def __init__(self, binary, config, initial, directory, case, extra_args=None):
        self.state = None
        self.status = None
        self.applied = None
        self.applied_count = 0
        self.node = rclpy.create_node('manual_pose_test_' + case)
        self.node.create_subscription(ArmState, '/luxi/arms/state', self.on_state, 10)
        self.node.create_subscription(String, '/luxi/arms/startup_status', self.on_status, 10)
        self.node.create_subscription(ArmCommand, '/luxi/applied', self.on_applied, 10)
        self.life = self.node.create_publisher(String, '/luxi/driver_lifecycle', 10)
        self.external = self.node.create_publisher(ArmCommand, '/luxi/command', 1)
        self.client = self.node.create_client(Command, '/luxi/arms/startup')
        self.log = open(Path(directory) / (case + '.log'), 'w+')
        self.process = subprocess.Popen([
            str(binary), '--ros-args', '-p', 'mock:=true', '-p', 'monitor_only:=true',
            '-p', 'config:=' + str(config),
            '-p', 'mock_initial_positions:=' + json.dumps(initial),
        ] + (extra_args or []), stdout=self.log, stderr=subprocess.STDOUT, env=os.environ.copy())

    def on_state(self, message):
        self.state = message

    def on_status(self, message):
        self.status = json.loads(message.data)

    def on_applied(self, message):
        self.applied = message
        self.applied_count += 1

    @property
    def manual(self):
        return self.status['manual_pose'] if self.status else {}

    def pump(self, seconds, action=None):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            assert self.process.poll() is None, self.log_text()
            if action:
                action()
            rclpy.spin_once(self.node, timeout_sec=.01)

    def until(self, predicate, timeout=5):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return
            self.pump(.01)
        raise AssertionError('Timed out: ' + str(self.status) + '\n' + self.log_text())

    def request(self, name, expected=True):
        request = Command.Request()
        request.command = name
        future = self.client.call_async(request)
        self.until(future.done)
        result = future.result()
        assert result is not None and result.success == expected, (name, result)
        return result

    def target(self, position, stamp=None):
        message = ArmCommand()
        message.position = position
        message.source_mono_ns = time.monotonic_ns() if stamp is None else stamp
        message.phase = 'test_external'
        self.external.publish(message)

    def lifecycle(self, value):
        self.life.publish(String(data=value))

    def startup_ready(self):
        self.until(lambda: self.state is not None and all(self.state.valid) and
                   self.status is not None and self.client.service_is_ready() and
                   self.life.get_subscription_count() and self.external.get_subscription_count())

    def held(self):
        self.until(lambda: self.manual.get('phase') == 'holding' and self.state.enabled)

    def log_text(self):
        self.log.flush()
        self.log.seek(0)
        return self.log.read()

    def close(self):
        if self.process.poll() is None:
            self.process.send_signal(signal.SIGINT)
        try:
            self.process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            self.process.kill()
            self.process.wait()
        self.node.destroy_node()
        self.log.close()


def close_to(actual, expected, tolerance=1e-5):
    return max(abs(a-b) for a, b in zip(actual, expected)) <= tolerance


def normal_case(binary, config_path, config, directory):
    limits = config['right']['limits'] + config['left']['limits']
    # A deliberately non-home, non-startup pose verifies that hold does not
    # execute either configured trajectory or rewrite the logical zero.
    initial = [low + .4 * (high-low) for low, high in limits]
    h = Harness(binary, config_path, initial, directory, 'normal')
    try:
        h.startup_ready()
        h.request('pose_hold', False)
        h.request('pose_cancel', False)
        assert h.manual['phase'] == 'inactive' and not h.state.enabled
        h.lifecycle('enable')
        h.pump(.15)
        assert not h.state.enabled, 'monitor-only boot must remain passive'

        h.request('pose_enter')
        h.until(lambda: h.manual.get('phase') == 'editing' and h.manual['can_hold'] and not h.state.enabled)
        generation = h.manual['generation']
        assert generation and len(h.manual['target']) == 16
        h.request('start', False)
        intrusive = initial.copy()
        intrusive[0] += .1

        def stale_controller():
            h.lifecycle('enable')
            h.target(intrusive)

        h.pump(.15, stale_controller)
        assert not h.state.enabled and h.manual['phase'] == 'editing'
        h.request('pose_hold')
        h.until(lambda: h.manual.get('phase') == 'preparing')
        h.pump(.15, stale_controller)
        assert not h.state.enabled, 'stable fresh feedback must precede enable'
        h.held()
        assert close_to(h.state.position, initial)
        assert close_to(h.manual['target'], initial)
        assert not h.manual['fault_latched']
        assert not h.status['owned'], 'manual hold must not claim startup ownership'
        h.request('pose_cancel', False)
        h.pump(.05)
        assert h.manual['phase'] == 'holding' and h.state.enabled
        assert close_to(h.manual['target'], initial), 'late cancel must not release an established hold'
        h.pump(.25, stale_controller)
        h.lifecycle('enable:wrong-generation')
        h.lifecycle('disable')
        h.lifecycle('disable:wrong-generation')
        h.pump(.05)
        assert h.manual['phase'] == 'holding' and h.state.enabled and close_to(h.state.position, initial)

        # Stop only this Harness's explicitly mock process. A stalled update
        # loop must latch even when all 16 mock feedback values look fresh on
        # the very first tick after SIGCONT.
        os.kill(h.process.pid, signal.SIGSTOP)
        try:
            h.pump(.28)  # Drain DDS messages already published before SIGSTOP.
            applied_before_resume = h.applied_count
        finally:
            resumed_at = time.monotonic_ns()
            os.kill(h.process.pid, signal.SIGCONT)
        h.until(lambda: h.manual.get('fault_latched') and
                h.state.receive_mono_ns > resumed_at and all(h.state.valid))
        h.pump(.15)
        assert h.manual['phase'] == 'holding' and h.state.enabled
        assert h.manual['reason'] == 'control_loop_stalled'
        assert h.applied_count == applied_before_resume, 'fresh feedback must not replay a latched hold target'
        assert close_to(h.manual['target'], initial), 'retain the old target for diagnosis only'
        assert h.manual['generation'] != generation
        h.lifecycle('enable:' + generation)
        h.pump(.05)
        assert h.manual['fault_latched'] and h.applied_count == applied_before_resume, 'pre-fault token must not unlock a new fault'
        generation = h.manual['generation']
        h.request('stop')
        h.request('pose_cancel', False)
        h.lifecycle('enable')
        h.pump(.05)
        assert h.manual['fault_latched'] and h.applied_count == applied_before_resume

        # Only the explicit new-generation enable can release the local hold.
        # Even a still-fresh target issued before it must not be replayed.
        old_stamp = time.monotonic_ns()
        h.lifecycle('enable:' + generation)
        h.until(lambda: h.manual.get('phase') == 'inactive' and h.state.enabled)
        assert not h.manual['fault_latched']
        h.target(intrusive, old_stamp)
        h.pump(.05)
        assert close_to(h.state.position, initial), 'cached target replayed across handover'
        h.pump(.2, lambda: h.target(intrusive))
        h.until(lambda: abs(h.state.position[0] - initial[0]) > .005)
        assert h.state.enabled and h.manual['phase'] == 'inactive'

        # Enter must also interrupt an already-active external controller.
        h.request('pose_enter')
        h.until(lambda: h.manual.get('phase') == 'editing' and not h.state.enabled)
        h.request('pose_cancel')
        h.until(lambda: h.manual.get('phase') == 'inactive' and not h.state.enabled)
        h.request('pose_cancel', False)
        assert not h.state.enabled and h.manual['phase'] == 'inactive'
        h.lifecycle('enable:' + generation)
        h.pump(.1, lambda: h.target(intrusive, old_stamp))
        assert not h.state.enabled, 'cancel must not revive a previous generation'
        h.request('pose_hold', False)

        # Existing startup start/stop and disable retain their normal semantics.
        h.request('pose_enter')
        h.until(lambda: h.manual.get('can_hold', False))
        h.request('pose_hold')
        h.held()
        os.kill(h.process.pid, signal.SIGSTOP)
        try:
            h.pump(.25)
        finally:
            os.kill(h.process.pid, signal.SIGCONT)
        h.until(lambda: h.manual.get('fault_latched') and all(h.state.valid))
        h.request('start')
        h.until(lambda: h.manual.get('phase') == 'inactive' and h.status['owned'] and h.status['phase'] == 'moving')
        assert not h.manual['fault_latched'], 'explicit startup must re-anchor and clear the manual fault'
        h.request('stop')
        h.until(lambda: h.status['phase'] == 'hold' and h.state.enabled)
        h.request('pose_enter')
        h.until(lambda: h.manual.get('can_hold', False) and not h.state.enabled)
        h.request('pose_hold')
        h.held()
        captured = list(h.manual['target'])
        h.request('stop')
        h.until(lambda: h.manual.get('phase') == 'inactive' and h.status['owned'] and h.state.enabled)
        h.pump(.1)
        assert close_to(h.state.position, captured)
        h.lifecycle('disable')
        h.pump(.1)
        assert h.state.enabled, 'unscoped legacy disable must not unload a new controller'
        h.request('disable')
        h.until(lambda: not h.state.enabled and not h.status['owned'])

        # A queued legacy park/disable must not unload a newly established
        # manual hold. Explicit current-owner disable and service disable
        # remain available to the operator.
        h.request('pose_enter')
        h.until(lambda: h.manual.get('can_hold', False))
        h.request('pose_hold')
        h.held()
        h.lifecycle('disable')
        h.lifecycle('disable:wrong-generation')
        h.pump(.1)
        assert h.manual['phase'] == 'holding' and h.state.enabled
        h.lifecycle('disable:' + h.status['control_generation'])
        h.until(lambda: h.manual.get('phase') == 'inactive' and not h.state.enabled)
        h.request('pose_enter')
        h.until(lambda: h.manual.get('can_hold', False))
        h.request('pose_hold')
        h.held()
        h.request('disable')
        h.until(lambda: h.manual.get('phase') == 'inactive' and not h.state.enabled)

        # Cancel during preparation must remove the delayed enable as well.
        h.request('pose_enter')
        h.until(lambda: h.manual.get('can_hold', False))
        h.request('pose_hold')
        h.request('pose_cancel')
        h.pump(.45)
        assert not h.state.enabled and h.manual['phase'] == 'inactive'
        print('PASS: disabled manual editing, stable capture, generation fencing, stalled-loop fault latch, fresh-target handover, startup/stop/disable/cancel')
    finally:
        h.close()


def limit_case(binary, config_path, config, directory, joint, case):
    limits = config['right']['limits'] + config['left']['limits']
    initial = [low + .4 * (high-low) for low, high in limits]
    initial[joint] = limits[joint][1] + .001
    h = Harness(binary, config_path, initial, directory, case)
    try:
        h.startup_ready()
        h.request('pose_enter')
        h.until(lambda: h.manual.get('phase') == 'editing' and h.manual.get('reason') == 'joint_outside_limits')
        assert not h.manual['can_hold'] and not h.state.enabled
        h.request('pose_hold', False)
        h.pump(.4)
        assert not h.state.enabled and h.manual['phase'] == 'editing'
        assert close_to(h.state.position, initial), 'a rejected pose must not be clamped into motion'
        h.request('pose_cancel')
        print('PASS: strict manual-pose limit rejection for joint', joint)
    finally:
        h.close()


def main():
    binary = Path(sys.argv[1]).resolve()
    config_path = Path(sys.argv[2]).resolve()
    config = json.loads(config_path.read_text())
    assert binary.is_file() and config_path.is_file()
    rclpy.init()
    try:
        with tempfile.TemporaryDirectory(prefix='luxi-manual-pose-') as directory:
            normal_case(binary, config_path, config, directory)
            limit_case(binary, config_path, config, directory, 0, 'arm_limit')
            limit_case(binary, config_path, config, directory, 15, 'gripper_limit')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
