#!/usr/bin/env python3
"""Real adapter/manager/web, isolated ROS domain, mock motors only. No CAN."""
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import urllib.error
import urllib.request

from test_openarm_manual_pose import Harness, close_to
import rclpy
from luxi_interfaces.msg import ArmCommand, QuestInput
from luxi_interfaces.srv import Command
from std_msgs.msg import String


class JointHarness(Harness):
    def __init__(self, *args):
        self.history = []
        self.control = {}
        self.b_pressed = False
        super().__init__(*args)
        self.node.create_subscription(String, '/luxi/control_status', self.on_control, 1)
        self.quest = self.node.create_publisher(QuestInput, '/luxi/quest', 1)
        self.intent = self.node.create_publisher(ArmCommand, '/luxi/intent', 1)
        self.manager_client = self.node.create_client(Command, '/luxi/control')
        self.intent_position = list(self.state.position) if self.state else None

    def on_control(self, m):
        self.control = json.loads(m.data)

    def on_applied(self, m):
        super().on_applied(m)
        self.history.append(m)

    @property
    def task(self):
        return self.status['joint_control'] if self.status else {}

    def pump(self, seconds, action=None):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            assert self.process.poll() is None, self.log_text()
            if action:
                action()
            stamp = time.monotonic_ns()
            q = QuestInput(); q.receive_mono_ns = stamp; q.triggers = [1., 1.]
            q.buttons = [False,self.b_pressed,False,False]
            self.quest.publish(q)
            if self.intent_position is not None:
                c = ArmCommand(); c.source_mono_ns = stamp; c.position = self.intent_position; c.phase = 'teleop'
                self.intent.publish(c)
            rclpy.spin_once(self.node, timeout_sec=.002)
            for _ in range(8):
                rclpy.spin_once(self.node, timeout_sec=0)

    def call(self, name, detail='', expected=True, client=None):
        req = Command.Request(); req.command = name; req.detail = detail
        future = (client or self.client).call_async(req)
        self.until(future.done)
        result = future.result()
        assert result.success == expected, (name, result, self.status)
        return result

    def move(self, target, request_id, expected=True, speed=.15, unit='rad'):
        return self.call('joint_move', json.dumps({'positions':target, 'request_id':request_id,
                                                'speed_rad_s':speed,'unit':unit}), expected)


def assert_bounded(h, limits):
    for m in h.history:
        for i, (low, high) in enumerate(limits):
            assert low <= m.position[i] <= high, (i, m.position[i], low, high, m.phase)


def http_post(port, command, detail, expected):
    data = json.dumps({'command':command,'detail':detail}).encode()
    req = urllib.request.Request('http://127.0.0.1:%d/api/arm_startup' % port, data=data,
                                 headers={'Content-Type':'application/json'})
    try:
        with urllib.request.urlopen(req, timeout=8) as f:
            code, body = f.status, json.load(f)
    except urllib.error.HTTPError as error:
        code, body = error.code, json.load(error)
    assert code == (200 if expected else 409) and body['success'] == expected, (code,body)


def normal(binary, manager, web, config_path, cfg, directory):
    limits = cfg['right']['limits'] + cfg['left']['limits']
    initial = [lo + .4*(hi-lo) for lo,hi in limits]
    h = JointHarness(binary, config_path, initial, directory, 'joint_normal')
    extras = []
    try:
        h.startup_ready()
        assert h.task['lower'] == [x[0] for x in limits] and h.task['upper'] == [x[1] for x in limits]
        before = h.applied_count
        # Every joint, both boundaries, must reject the whole batch unchanged.
        for i,(low,high) in enumerate(limits):
            for value in [low-1e-6, high+1e-6]:
                target = initial.copy(); target[i] = value
                h.move(target, 'bad-%d-%s' % (i,value), False)
                h.move([v*(180/math.pi) for v in target], 'bad-deg-%d-%s' % (i,value), False, unit='deg')
            # Reject even the next representable degree outside either bound.
            for endpoint, direction in [(low,-math.inf),(high,math.inf)]:
                target=[v*(180/math.pi) for v in initial]
                target[i]=math.nextafter(endpoint*(180/math.pi),direction)
                h.move(target,'ulp-deg-%d-%s'%(i,direction),False,unit='deg')
        for payload in [{}, {'positions':initial[:-1],'request_id':'short'},
                        {'positions':[True]+initial[1:],'request_id':'bool'},
                        {'positions':['0']+initial[1:],'request_id':'string'},
                        {'positions':[None]+initial[1:],'request_id':'null'},
                        {'positions':initial,'request_id':'unit','unit':'degrees'},
                        {'positions':initial,'request_id':'unit-null','unit':None},
                        {'positions':initial,'request_id':'unit-bool','unit':True},
                        {'positions':initial,'request_id':'speed','speed_rad_s':.201}]:
            h.call('joint_move',json.dumps(payload),False)
        h.call('joint_move','{broken',False)
        h.call('joint_move',json.dumps({'positions':[float('nan')]+initial[1:],'request_id':'nan'}),False)
        h.pump(.1)
        assert not h.state.enabled and h.manual['phase']=='inactive' and h.applied_count==before
        assert close_to(h.state.position,initial)

        h.move(initial,'preparing-stall')
        assert h.task['phase'] in ['idle','preparing']
        os.kill(h.process.pid,signal.SIGSTOP);time.sleep(.22);os.kill(h.process.pid,signal.SIGCONT)
        h.until(lambda:h.task['phase']=='fault')
        h.pump(.2)
        assert not h.state.enabled and h.applied_count==before,'interrupted preparation must never enable on recovery'
        h.move(initial,'preparing-stall');h.pump(.1)
        assert not h.state.enabled and h.task['phase']=='fault'

        log = open(Path(directory)/'joint_web.log','w+')
        web_process = subprocess.Popen([str(web),'--ros-args','-p','mock:=true','-p','no_cameras:=true',
            '-p','port:=18042','-p','directory:='+directory],stdout=log,stderr=subprocess.STDOUT,env=os.environ.copy())
        extras.append((web_process,log))
        def web_ready():
            try:
                with urllib.request.urlopen('http://127.0.0.1:18042/api/state',timeout=.3) as f:
                    return (json.load(f).get('arm_startup') or {}).get('fresh',False)
            except (OSError,ValueError): return False
        h.until(web_ready,6)
        bad=initial.copy(); bad[0]=limits[0][1]+.001
        http_post(18042,'joint_move',json.dumps({'positions':bad,'request_id':'http-bad'}),False)
        assert not h.state.enabled

        mlog = open(Path(directory)/'joint_manager.log','w+')
        mp = subprocess.Popen([str(manager),'--ros-args','-p','config:='+str(config_path),
            '-p','home_joint_speed_limit:=0.2'],stdout=mlog,stderr=subprocess.STDOUT,env=os.environ.copy())
        extras.append((mp,mlog))
        h.until(lambda:h.manager_client.service_is_ready() and h.control.get('quest_window_samples',0)>=50,6)
        target = [p+.04 for p in initial]
        h.intent_position = target.copy()
        http_post(18042,'joint_move',json.dumps({'positions':[v*(180/math.pi) for v in target], 'unit':'deg', 'request_id':'http-valid','speed_rad_s':.15}),True)
        h.until(lambda:h.task['phase']=='moving' and h.control.get('manual_pose_active'),4)
        h.call('start',expected=False,client=h.manager_client)
        h.lifecycle('enable')
        h.until(lambda:h.task['phase']=='ready' and h.state.enabled,5)
        h.pump(.1)
        assert h.manual['phase']=='holding' and close_to(h.state.position,target,.00001)
        assert h.control.get('phase')=='disabled'
        before = h.applied_count
        # Replay the same degree request without moving a second time.
        h.move([v*(180/math.pi) for v in target],'http-valid',unit='deg')
        assert h.task['phase']=='ready'
        altered=target.copy();altered[0]+=.01
        h.move(altered,'http-valid',False)
        h.move(bad,'while-holding-invalid',False)
        assert h.state.enabled and h.manual['phase']=='holding'

        moving = [p+.15 for p in target]
        h.move(moving,'stop-me')
        h.until(lambda:h.task['phase']=='moving')
        h.pump(.15)
        h.call('joint_stop')
        h.until(lambda:h.task['phase']=='stopped')
        held=list(h.manual['target']); h.pump(.25)
        assert h.state.enabled and close_to(h.state.position,held,.00001)
        assert not close_to(h.state.position,moving,.005)
        h.move(moving,'stop-me');h.pump(.1)
        assert h.task['phase']=='stopped' and close_to(h.state.position,held,.00001)

        h.move(moving,'manager-stop')
        h.until(lambda:h.task['phase']=='moving' and h.control.get('manual_pose_phase')=='moving')
        h.call('disable',client=h.manager_client)
        h.until(lambda:h.task['phase']=='stopped')
        assert h.state.enabled,'global stop must retain torque for a joint action'
        h.pump(.15)
        h.move(moving,'quest-stop')
        h.until(lambda:h.task['phase']=='moving' and h.control.get('manual_pose_phase')=='moving')
        h.b_pressed=True
        h.until(lambda:h.task['phase']=='stopped')
        assert h.state.enabled,'Quest B must retain torque for a joint action'
        h.b_pressed=False;h.pump(.15)
        held=list(h.state.position)

        h.move([p+.15 for p in held],'stall-me')
        h.until(lambda:h.task['phase']=='moving')
        os.kill(h.process.pid,signal.SIGSTOP);time.sleep(.22);os.kill(h.process.pid,signal.SIGCONT)
        h.until(lambda:h.task['phase']=='fault' and h.manual['fault_latched'])
        h.pump(.1); count=h.applied_count;h.pump(.25)
        assert h.applied_count==count and h.state.enabled
        # Fresh explicit recovery is permitted; an old request never replays.
        h.move([p+.15 for p in held],'stall-me');h.pump(.1)
        assert h.task['phase']=='fault'
        h.move(list(h.state.position),'recover')
        h.until(lambda:h.task['phase']=='ready',4)
        h.intent_position=list(h.state.position)
        h.until(lambda:h.control.get('manual_pose_phase')=='holding' and h.control.get('vr_fresh') and
                h.control.get('quest_window_samples',0)>=50 and h.control.get('arm_feedback_age_ms',999)<50)
        old = list(h.state.position); fence=time.monotonic_ns()
        h.call('start',client=h.manager_client)
        h.until(lambda:h.manual['phase']=='inactive')
        h.until(lambda:any(x.source_mono_ns>fence and x.phase=='homing' for x in h.history))
        first=next(x for x in h.history if x.source_mono_ns>fence and x.phase=='homing')
        for i in range(16):
            # Legacy teleoperation retains its configured gripper speed;
            # the .2 rad/s slow-home cap applies only to the seven arm joints.
            cap = cfg['velocity'][7] if i % 8 == 7 else min(.2,cfg['velocity'][i%8])
            assert abs(first.position[i]-old[i]) <= cap*.05+.005, (i,first.position[i],old[i],cap)
        assert_bounded(h,limits)
        print('PASS: all 32 out-of-range batches rejected without enable/motion; malformed/nonfinite/type/speed/unit rejection; real HTTP execution; manager interlock; stop hold; request dedup; stalled-loop latch; normal-control takeover; all applied targets within unchanged limits')
    finally:
        for process,log in reversed(extras):
            if process.poll() is None:process.send_signal(signal.SIGINT)
            try:process.wait(timeout=4)
            except subprocess.TimeoutExpired:process.kill();process.wait()
            log.close()
        h.close()


def boundary(binary,config_path,cfg,directory,upper,unit='rad'):
    limits=cfg['right']['limits']+cfg['left']['limits']
    target=[v[1] if upper else v[0] for v in limits]
    initial=[p-.025 if upper else p+.025 for p in target]
    h=JointHarness(binary,config_path,initial,directory,('upper' if upper else 'lower')+'_'+unit)
    try:
        h.startup_ready();h.move([v*(180/math.pi) for v in target] if unit=='deg' else target,'exact-boundary',unit=unit)
        h.until(lambda:h.task['phase']=='ready',5);h.pump(.1)
        assert close_to(h.state.position,target,.000001)
        assert_bounded(h,limits)
        print('PASS: exact %s %s boundaries for all 16 axes, no output beyond limits' % ('upper' if upper else 'lower',unit))
    finally:h.close()


def main():
    binary,manager,web,config_path=(Path(x).resolve() for x in sys.argv[1:])
    original=config_path.read_bytes();cfg=json.loads(original)
    rclpy.init()
    try:
        with tempfile.TemporaryDirectory(prefix='luxi-joint-target-') as directory:
            normal(binary,manager,web,config_path,cfg,directory)
            boundary(binary,config_path,cfg,directory,True)
            boundary(binary,config_path,cfg,directory,False)
            boundary(binary,config_path,cfg,directory,True,'deg')
            boundary(binary,config_path,cfg,directory,False,'deg')
            limits=cfg['right']['limits']+cfg['left']['limits']
            initial=[(low+high)/2 for low,high in limits];initial[15]=limits[15][1]+.001
            h=JointHarness(binary,config_path,initial,directory,'bad_current')
            try:
                h.startup_ready();h.move([(low+high)/2 for low,high in limits],'bad-current',False)
                assert not h.state.enabled and h.applied_count==0
                print('PASS: out-of-limit current pose cannot be enabled or clamped into a new action')
            finally:h.close()
        assert config_path.read_bytes()==original,'limit/offset configuration must be unchanged'
        print('PASS: configuration SHA-256 unchanged:',hashlib.sha256(original).hexdigest())
    finally:rclpy.shutdown()


if __name__=='__main__':main()
