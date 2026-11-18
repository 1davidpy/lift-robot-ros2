#!/usr/bin/env python3
"""Isolated mock equilibrium load, real driver/manager, never opens CAN."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
from test_openarm_joint_target import JointHarness, assert_bounded
from test_openarm_manual_pose import close_to
import rclpy


class LoadedHarness(JointHarness):
    def __init__(self, *args):
        self.external_target = None
        self.monitor_enable = False
        self.enabled_samples = 0
        super().__init__(*args)

    def on_state(self, state):
        super().on_state(state)
        if self.monitor_enable:
            self.enabled_samples += 1
            assert state.enabled, 'static compensation handover removed torque'

    def pump(self, seconds, action=None):
        def publish():
            if self.external_target is not None:
                self.target(self.external_target)
            if action: action()
        super().pump(seconds, publish)

    def arm_error(self, target):
        return max(abs(self.state.position[i]-target[i]) for i in range(16) if i%8 != 7)

    def compensated(self, target, timeout=10):
        # Let the mock plant actually develop its load deflection; immediately
        # after enable the old sample still equals the captured target.
        self.pump(.4)
        self.until(lambda: self.arm_error(target) < .008, timeout)
        self.pump(.15)
        assert self.arm_error(target) < .008
        assert self.status['static_compensation']['grippers_enabled'] is False
        assert self.status['static_compensation_rad'][7] == self.status['static_compensation_rad'][15] == 0
        assert self.status['output_enable_count'] == 1
        assert self.status['output_disable_count'] == 1


def main():
    binary, manager, config = (Path(x).resolve() for x in sys.argv[1:])
    original = config.read_bytes(); cfg = json.loads(original)
    limits = cfg['right']['limits']+cfg['left']['limits']
    initial = [lo+.4*(hi-lo) for lo,hi in limits]
    # Nonzero signed gripper deflection deliberately verifies no gripper integral.
    load = [(.012 if i%8 == 7 else .045) * (-1 if i%2 else 1) for i in range(16)]
    rclpy.init()
    try:
        with tempfile.TemporaryDirectory(prefix='luxi-static-load-') as directory:
            h = LoadedHarness(binary,config,initial,directory,'static_load',
                ['-p','mock_static_load_rad:='+json.dumps(load)])
            mp = mlog = None
            try:
                h.startup_ready(); h.call('pose_enter'); h.call('pose_hold'); h.held()
                h.monitor_enable = True
                h.compensated(initial)
                assert all(abs(h.status['static_compensation_rad'][i]) > .025 for i in range(16) if i%8 != 7)
                print('PASS: all 14 manually held joints reject signed equilibrium load; both grippers retain zero compensation')

                support = list(h.status['support_target'])
                generation = h.manual['generation']
                h.lifecycle('enable:'+generation)
                h.until(lambda:h.manual['phase']=='inactive')
                h.pump(.12)
                assert close_to(h.status['support_target'],support,.003), 'handover doubled or dropped existing bias'
                assert close_to(h.status['nominal_target'],initial,1e-10)

                target=initial.copy();target[0]+=.025;target[8]-=.025
                h.external_target=target
                h.compensated(target)
                h.external_target=None
                h.pump(.5)
                assert close_to(h.status['nominal_target'],target,1e-10)
                h.compensated(target)
                print('PASS: streaming ROS target and controller gap retain nominal goals, load compensation and support torque')

                target[0]-=.05;target[8]+=.05
                h.move(target,'loaded-angle-goal')
                h.until(lambda:h.task['phase']=='ready',10)
                h.compensated(target)
                assert close_to(h.manual['target'],target,1e-10), 'ready hold must retain the requested nominal pose'
                print('PASS: loaded joint-angle trajectory arrives at requested measured pose and continues compensated hold')

                h.call('start')
                h.until(lambda:h.status['phase']=='ready',22)
                startup=cfg['startup_pose']['right']+cfg['startup_pose']['left']
                h.compensated(startup)
                print('PASS: startup arrival and ready hold use the same 14-axis compensator')

                mlog=open(Path(directory)/'manager.log','w+')
                mp=subprocess.Popen([str(manager),'--ros-args','-p','config:='+str(config),
                    '-p','home_joint_speed_limit:=0.2'],stdout=mlog,stderr=subprocess.STDOUT)
                h.until(lambda:h.manager_client.service_is_ready() and h.control.get('quest_window_samples',0)>=50,6)
                checkpoint=len(h.history); before=list(h.status['support_target'])
                h.call('start',client=h.manager_client)
                h.until(lambda:any(c.phase=='homing' for c in h.history[checkpoint:]),4)
                first=next(c for c in h.history[checkpoint:] if c.phase=='homing')
                for i in range(16):
                    # The original gripper trajectory retains its configured
                    # speed; the 0.2 rad/s home cap only covers J1-J7.
                    cap = cfg['velocity'][7] if i%8 == 7 else min(.2,cfg['velocity'][i%8])
                    assert abs(first.position[i]-before[i]) <= cap*.05+.004, (i,first.position[i],before[i],'handover output jump')
                h.call('disable',client=h.manager_client)
                h.until(lambda:h.control.get('phase')=='hold')
                h.pump(.2)
                nominal=list(h.status['nominal_target'])
                h.compensated(nominal)
                assert h.enabled_samples>100
                assert_bounded(h,limits)
                h.monitor_enable=False
                h.call('disable')
                h.until(lambda:not h.state.enabled and all(v==0 for v in h.status['static_compensation_rad']))
                print('PASS: manager handover preserves existing support without double compensation; explicit disable resets all bias; original limits hold')
            finally:
                if mp:
                    if mp.poll() is None:mp.send_signal(signal.SIGINT)
                    try:mp.wait(timeout=4)
                    except subprocess.TimeoutExpired:mp.kill();mp.wait()
                    mlog.close()
                h.close()
        assert config.read_bytes()==original
    finally:rclpy.shutdown()


if __name__=='__main__':main()
