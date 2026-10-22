#!/usr/bin/env python3
"""Real adapter + HTTP gateway, isolated ROS domain, mock motors only."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import urllib.request
import urllib.error

os.environ['ROS_DOMAIN_ID'] = '108'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from luxi_interfaces.msg import ArmCommand
from std_msgs.msg import String

root = Path(sys.argv[1]).resolve()
config = json.loads((root/'Project/robot_bringup/config/arms-kh.json').read_text())
case=os.environ.get('LUXI_REBOOT_CASE','normal')
extra=[]
if case!='normal':
    initial=config['right']['home']+config['left']['home']
    if case in ('recover_gripper','reject_gripper'):
        initial[15]=config['left']['limits'][7][0]-(.0132 if case=='recover_gripper' else .021)
    else:
        assert case in ('recover','reject'), case
        initial[13]=config['left']['limits'][5][1]+(.091 if case=='recover' else .151)
    extra=['-p','mock_initial_positions:='+json.dumps(initial)]
rclpy.init()
node = rclpy.create_node('startup_test')
life = node.create_publisher(String, '/luxi/driver_lifecycle', 10)
external = node.create_publisher(ArmCommand, '/luxi/command', 1)
manager = node.create_publisher(String, '/luxi/control_status', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
processes = []
logs = []
url = 'http://127.0.0.1:18041'

def wait(seconds):
    end = time.monotonic()+seconds
    while time.monotonic()<end:
        rclpy.spin_once(node, timeout_sec=.01)

def state():
    return json.load(urllib.request.urlopen(url+'/api/state', timeout=3))

def until(predicate, timeout=5):
    end=time.monotonic()+timeout
    while time.monotonic()<end:
        try:
            j=state()
            if predicate(j): return j
        except (urllib.error.URLError, TypeError, KeyError): pass
        wait(.04)
    raise AssertionError('state timeout: '+str(state()))

def command(value, expected=200, detail=''):
    req=urllib.request.Request(url+'/api/arm_startup',data=json.dumps({'command':value,'detail':detail}).encode(),headers={'Content-Type':'application/json'})
    try:
        res=urllib.request.urlopen(req,timeout=4)
    except urllib.error.HTTPError as e: res=e
    assert res.code==expected,(value,res.code,res.read())
    return json.load(res)

try:
    with tempfile.TemporaryDirectory(prefix='luxi-startup-') as tmp:
        for binary,args in [
            ('luxi_device_adapters/lib/luxi_device_adapters/openarm_adapter',['-p','mock:=true','-p','monitor_only:=true','-p','config:='+str(root/'Project/robot_bringup/config/arms-kh.json'),*extra]),
            ('luxi_web_ctrl/lib/luxi_web_ctrl/web_ctrl',['-p','port:=18041','-p','mock:=true','-p','no_cameras:=true','-p','directory:='+tmp])]:
            log=open(Path(tmp)/(str(len(logs))+'.log'),'w+'); logs.append(log)
            processes.append(subprocess.Popen([str(root/'build/install'/binary),'--ros-args',*args],stdout=log,stderr=subprocess.STDOUT))
        j=until(lambda s:s['system']['services']['arm_startup'] and s['arms']['fresh'] and s['arm_startup']['fresh'])
        assert j['arm_startup']['tolerance_rad']==.02 and len(j['arm_startup']['static_compensation_rad'])==16
        assert not j['arms']['enabled'] and j['arms']['phase']=='monitor_only'
        initial=j['arms']['position']
        if case in ('reject','reject_gripper'):
            command('start',409)
            assert not state()['arms']['enabled']
            until(lambda s:s['arm_startup']['last_rejection'])
            print('PASS: outside recovery margin rejected without enabling motors')
            sys.exit(0)
        life.publish(String(data='enable')); wait(.3)
        assert not state()['arms']['enabled'], 'monitor boot must reject lifecycle enable'
        command('invalid',409)
        manager.publish(String(data=json.dumps({'phase':'teleop','system_phase':'active'}))); wait(.1)
        command('start',409)
        manager.publish(String(data=json.dumps({'phase':'disabled','system_phase':'disabled'}))); wait(.1)
        command('start')
        until(lambda s:s['arm_startup']['phase']=='moving')
        command('start',409)
        # External targets cannot replace the local startup trajectory.
        msg=ArmCommand(); msg.position=[0.]*16; msg.source_mono_ns=time.monotonic_ns(); msg.phase='external'
        external.publish(msg)
        life.publish(String(data='enable')); wait(.2)
        assert state()['arm_startup']['owned']
        command('stop')
        held=until(lambda s:s['arm_startup']['phase']=='hold')
        assert held['arms']['enabled']
        wait(.4)
        assert max(abs(a-b) for a,b in zip(held['arms']['position'],state()['arms']['position']))<.005
        command('start')
        previous=state(); maximum=0
        deadline=time.monotonic()+22
        while time.monotonic()<deadline:
            wait(.04); j=state()
            dt=(int(j['arms']['receive_mono_ns'])-int(previous['arms']['receive_mono_ns']))/1e9
            if dt>0:
                rate=max(abs(a-b)/dt for a,b in zip(j['arms']['position'],previous['arms']['position']))
                maximum=max(maximum,rate)
                # Feedback follows the previous tick's submitted target; allow
                # one 10 ms timer interval for the HTTP sampling boundary.
                assert max(abs(a-b) for a,b in zip(j['arms']['position'],previous['arms']['position']))<=.15*dt+.0016,rate
            previous=j
            if j['arm_startup']['phase']=='ready': break
        assert j['arm_startup']['phase']=='ready',j['arm_startup']
        assert j['arms']['enabled'] and j['arm_startup']['max_error_rad']<=j['arm_startup']['tolerance_rad']
        assert j['arm_startup']['target']==config['startup_pose']['right']+config['startup_pose']['left']
        wait(.6)
        assert state()['arm_startup']['phase']=='ready'
        command('disable')
        until(lambda s:not s['arms']['enabled'] and not s['arm_startup']['owned'])
        wait(.3)
        assert not state()['arms']['enabled']
        target=state()['arms']['position']
        command('restore_hold',409,json.dumps([0.]*3))
        command('restore_hold',200,json.dumps(target))
        restored=until(lambda s:s['arms']['enabled'] and s['arm_startup']['phase']=='hold')
        assert max(abs(a-b) for a,b in zip(restored['arms']['position'],target))<.002
        wait(.3)
        assert state()['arm_startup']['phase']=='hold'
        command('disable')
        print('PASS: no automatic enable; manager interlock; HTTP rejection; stop/hold/resume; target ordering; measured arrival; disable; sampled feedback peak %.4f rad/s (one timer interval tolerance)'%maximum)
finally:
    for p in processes: p.send_signal(signal.SIGINT)
    for p in processes:
        try:p.wait(timeout=5)
        except subprocess.TimeoutExpired:p.kill();p.wait()
    node.destroy_node();rclpy.shutdown()
