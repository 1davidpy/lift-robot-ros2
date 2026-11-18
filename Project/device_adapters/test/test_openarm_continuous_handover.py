#!/usr/bin/env python3
"""Real driver + manager + HTTP API, isolated ROS domain, mock motors only."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import urllib.request

import rclpy
from test_openarm_joint_target import JointHarness, assert_bounded, http_post
from test_openarm_manual_pose import close_to


class ContinuousHarness(JointHarness):
    def __init__(self, *args):
        self.check_continuity = False
        self.states = []
        super().__init__(*args)

    def on_state(self, message):
        super().on_state(message)
        if self.check_continuity:
            self.states.append(message)
            assert message.enabled, 'ownership switch briefly disabled the arms'

    def no_power_cycle(self, enabled_count, disabled_count):
        self.pump(.08)
        assert self.state.enabled
        assert self.status['output_enable_count'] == enabled_count, self.status
        assert self.status['output_disable_count'] == disabled_count, self.status


def main():
    binary, manager, web, config = (Path(p).resolve() for p in sys.argv[1:])
    original = config.read_bytes()
    cfg = json.loads(original)
    limits = cfg['right']['limits'] + cfg['left']['limits']
    initial = [lo + .4*(hi-lo) for lo, hi in limits]
    rclpy.init()
    try:
        with tempfile.TemporaryDirectory(prefix='luxi-continuous-handover-') as directory:
            h = ContinuousHarness(binary, config, initial, directory, 'continuous')
            extras = []
            try:
                h.startup_ready(); h.call('pose_enter'); h.call('pose_hold'); h.held()
                h.check_continuity = True
                ec, dc = h.status['output_enable_count'], h.status['output_disable_count']
                assert ec == 1 and dc == 1
                first_owner = h.status['control_generation']
                held = list(h.status['support_target'])

                # A malformed request from another endpoint used to run the
                # exception cleanup and disable the already-held robot.
                for detail in ['{broken', '{}', '[null]', '[true]', '["0"]']:
                    h.call('restore_hold', detail, False)
                h.no_power_cycle(ec, dc)
                assert close_to(h.state.position, held)

                wlog = open(Path(directory)/'web.log', 'w+')
                wp = subprocess.Popen([str(web), '--ros-args', '-p', 'mock:=true', '-p', 'no_cameras:=true',
                    '-p', 'port:=18043', '-p', 'directory:='+directory], stdout=wlog, stderr=subprocess.STDOUT)
                extras.append((wp, wlog))
                def web_ready():
                    try:
                        with urllib.request.urlopen('http://127.0.0.1:18043/api/state', timeout=.3) as f:
                            return (json.load(f).get('arm_startup') or {}).get('fresh', False)
                    except (OSError, ValueError): return False
                h.until(web_ready, 6)
                http_post(18043, 'restore_hold', '{broken', False)
                h.no_power_cycle(ec, dc)

                # Startup takes over a manually captured hold, then a different
                # lifecycle client takes over startup. Neither toggles outputs.
                h.call('start'); h.call('stop')
                h.until(lambda: h.status['owned'] and h.status['phase'] == 'hold')
                h.no_power_cycle(ec, dc)
                h.lifecycle('disable:'+first_owner)
                h.pump(.1)
                assert h.state.enabled
                support = list(h.status['support_target'])
                previous_owner = h.status['control_generation']
                h.lifecycle('enable')
                h.until(lambda: not h.status['owned'] and h.status['control_generation'] != previous_owner)
                assert close_to(h.status['support_target'], support)
                h.no_power_cycle(ec, dc)

                # No new target for 900 ms: the driver continues publishing
                # the existing support setpoint across the connection gap.
                before = len(h.history)
                h.pump(.9)
                assert len(h.history)-before > 40
                assert any(c.phase == 'controller_gap_hold' for c in h.history[before:])
                assert close_to(h.status['support_target'], support)
                h.no_power_cycle(ec, dc)
                for _ in range(3):
                    old = h.status['control_generation']
                    h.lifecycle('enable')
                    h.until(lambda: h.status['control_generation'] != old)
                    h.lifecycle('disable:'+old)
                    h.lifecycle('disable')
                    h.lifecycle('disable:wrong-owner')
                    h.no_power_cycle(ec, dc)

                # Switch from external streaming to an HTTP joint target, then
                # another HTTP client updates the held target in degrees.
                h.pump(.15)
                target = list(h.state.position)
                old = h.status['control_generation']
                http_post(18043, 'joint_move', json.dumps({'positions':target, 'request_id':'client-a'}), True)
                h.until(lambda: h.task['phase'] == 'ready')
                h.lifecycle('disable:'+old); h.no_power_cycle(ec, dc)
                target[0] += .025
                http_post(18043, 'joint_move', json.dumps({'positions':[p*180/3.141592653589793 for p in target],
                          'unit':'deg', 'request_id':'client-b'}), True)
                h.until(lambda: h.task['phase'] == 'ready')
                h.no_power_cycle(ec, dc)

                mlog = open(Path(directory)/'manager.log', 'w+')
                mp = subprocess.Popen([str(manager), '--ros-args', '-p', 'config:='+str(config),
                    '-p', 'home_joint_speed_limit:=0.2'], stdout=mlog, stderr=subprocess.STDOUT)
                extras.append((mp, mlog))
                h.until(lambda: h.manager_client.service_is_ready() and h.control.get('quest_window_samples', 0) >= 50, 6)
                # The independent API now matches shared mode: ordinary
                # disable/halt/hold releases control intent, never torque.
                for command in ['disable', 'halt', 'hold']:
                    h.call(command, client=h.manager_client)
                    h.no_power_cycle(ec, dc)
                h.call('start', client=h.manager_client)
                h.until(lambda: h.manual['phase'] == 'inactive' and h.control.get('phase') == 'homing')
                h.no_power_cycle(ec, dc)
                h.call('disable', client=h.manager_client)
                h.until(lambda: h.control.get('phase') == 'hold')
                h.no_power_cycle(ec, dc)
                h.pump(.2)
                old = h.status['control_generation']
                h.move(list(h.state.position), 'back-to-joint')
                h.until(lambda: h.task['phase'] == 'ready')
                h.lifecycle('disable:'+old); h.no_power_cycle(ec, dc)
                assert len(h.states) > 100 and all(s.enabled for s in h.states)
                assert_bounded(h, limits)
                print('PASS: uninterrupted enabled feedback; output enable count stays 1 across manual/startup/lifecycle/HTTP/manager switches; no extra disable writes; invalid requests and stale disables cannot unload; delayed new targets retain support')

                # Deliberate operator power-off remains available through the
                # explicit arm service, including after lifecycle ownership.
                h.check_continuity = False
                http_post(18043, 'disable', '', True)
                h.until(lambda: not h.state.enabled and h.status['output_disable_count'] == dc+1)
                assert h.status['output_disable_count'] == dc+1
                print('PASS: explicit operator torque removal remains effective')
            finally:
                for process, log in reversed(extras):
                    if process.poll() is None: process.send_signal(signal.SIGINT)
                    try: process.wait(timeout=4)
                    except subprocess.TimeoutExpired: process.kill(); process.wait()
                    log.close()
                h.close()
        assert config.read_bytes() == original
    finally:
        rclpy.shutdown()


if __name__ == '__main__': main()
