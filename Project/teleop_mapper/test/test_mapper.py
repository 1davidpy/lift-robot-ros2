#!/usr/bin/env python3
"""Real mapper with isolated synthetic Quest input; never connects to actuators."""
import json
import argparse
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time

os.environ['ROS_DOMAIN_ID'] = '118'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
import rclpy
from luxi_interfaces.msg import ArmTargets, QuestInput
from rclpy.qos import QoSProfile, ReliabilityPolicy

parser=argparse.ArgumentParser()
parser.add_argument('root',type=Path)
parser.add_argument('--cutoff',type=float)
parser.add_argument('--expect-left-open',type=float)
parser.add_argument('--max-step90-ms',type=float)
parser.add_argument('--label',default='mapper-probe')
parser.add_argument('--binary',type=Path)
parser.add_argument('--config',type=Path)
parser.add_argument('--output-dir',type=Path)
args=parser.parse_args()
root = args.root
cfg_path = args.config or root / 'Project/robot_bringup/config/arms-teleop.json'
cfg = json.loads(cfg_path.read_text())
limits = cfg.get('teleop_gripper_max_open_degrees', [-1., -1.])
expected_open = []
for side, limit in zip(('right', 'left'), limits):
    arm = cfg[side]
    expected_open.append(arm['gripper_open'] if limit == -1 else arm['gripper_closed'] +
                         math.copysign(math.radians(limit), arm['gripper_open']-arm['gripper_closed']))
prefix = '/mapper_debug_' + str(time.monotonic_ns())
rclpy.init()
node = rclpy.create_node('mapper_probe_debug')
pub = node.create_publisher(QuestInput, prefix + '/quest', 1)
rows = []
node.create_subscription(ArmTargets, prefix + '/targets',
    lambda m: rows.append(dict(sequence=int(m.sequence), source_ns=int(m.receive_mono_ns),
                              observed_ns=time.monotonic_ns(), grippers=list(m.grippers),
                              position=[m.right.position.x, m.right.position.y, m.right.position.z],
                              orientation=[[p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z]
                                           for p in (m.right, m.left)])),
    QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
quest = QuestInput()
quest.reference.orientation.w = quest.left.orientation.w = quest.right.orientation.w = 1.
sequence = 0

def pump(seconds):
    global sequence
    end = time.monotonic() + seconds
    next_publish = 0.
    while time.monotonic() < end:
        if time.monotonic() >= next_publish:
            sequence += 1
            quest.sequence = sequence
            quest.receive_mono_ns = time.monotonic_ns()
            pub.publish(quest)
            next_publish = time.monotonic() + 1/72
        rclpy.spin_once(node, timeout_sec=.001)

with tempfile.TemporaryDirectory(prefix='mapper-debug-') as directory:
    log = open(Path(directory)/'mapper.log', 'w+')
    binary = args.binary or root/'build/install/luxi_teleop_mapper/lib/luxi_teleop_mapper/teleop_mapper'
    parameters=['-p','pose_filter_cutoff_hz:='+str(args.cutoff)] if args.cutoff is not None else []
    process = subprocess.Popen([str(binary), '--ros-args', '-p', 'config:='+str(cfg_path),*parameters,
        '-r', '/luxi/quest:='+prefix+'/quest', '-r', '/luxi/targets:='+prefix+'/targets'],
        env=dict(os.environ, ROS_LOG_DIR=directory), stdout=log, stderr=subprocess.STDOUT)
    try:
        discovery_deadline = time.monotonic()+8
        while not rows and time.monotonic()<discovery_deadline and process.poll() is None:
            pump(.25)
        if not rows:
            log.flush();log.seek(0)
            raise AssertionError('isolated mapper not discovered: '+log.read())
        def angle(a, b):
            dot = abs(sum(x*y for x,y in zip(a,b)))
            return 2*math.acos(min(1., dot))
        pump(.4)
        neutral = rows[-1]['orientation']
        quest.reference.orientation.w = math.cos(.52/2)
        quest.reference.orientation.x = math.sin(.52/2)
        pump(.6)
        for before, after in zip(neutral, rows[-1]['orientation']):
            assert angle(before, after)<.01, 'Headset pitch leaked into wrist target'
        for hand in (quest.right, quest.left):
            hand.orientation.w = math.cos(.25/2)
            hand.orientation.x = math.sin(.25/2)
        pump(.6)
        for before, after in zip(neutral, rows[-1]['orientation']):
            assert abs(angle(before, after)-.25)<.01, 'Controller wrist pitch was lost'
        for pose in (quest.reference, quest.left, quest.right):
            pose.orientation.w = 1.
            pose.orientation.x = 0.
        pump(.6)
        for bad in ([-2., -1.], [200., -1.], [-1., 200.], [10.]):
            invalid = dict(cfg, teleop_gripper_max_open_degrees=bad)
            invalid_path = Path(directory)/'invalid.json'
            invalid_path.write_text(json.dumps(invalid))
            rejected = subprocess.run([str(binary), '--ros-args', '-p', 'config:='+str(invalid_path)],
                env=dict(os.environ, ROS_LOG_DIR=directory), stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT, timeout=5)
            assert rejected.returncode != 0, 'Invalid gripper limit accepted: '+str(bad)
        cases = {}
        for name, triggers in [('released', [0., 0.]), ('left_pressed', [1., 0.]),
                               ('right_pressed', [0., 1.])]:
            quest.triggers = triggers
            first = len(rows)
            pump(.35)
            result = rows[-1]['grippers']
            assert len(rows)>first, 'no output for '+name
            cases[name] = result
        right_open, left_open = expected_open
        assert abs(cases['released'][0]-right_open)<1e-8
        assert abs(cases['released'][1]-left_open)<1e-8
        assert abs(cases['left_pressed'][0]-right_open)<1e-8
        assert abs(cases['left_pressed'][1]-cfg['left']['gripper_closed'])<1e-8
        assert abs(cases['right_pressed'][0]-cfg['right']['gripper_closed'])<1e-8
        assert abs(cases['right_pressed'][1]-left_open)<1e-8
        quest.triggers = [.25, .75]
        pump(.3)
        assert abs(rows[-1]['grippers'][0]-(.25*right_open+.75*cfg['right']['gripper_closed']))<1e-8
        assert abs(rows[-1]['grippers'][1]-(.75*left_open+.25*cfg['left']['gripper_closed']))<1e-8
        quest.triggers = [0., 0.]
        pump(.3)
        baseline = rows[-1]['position']
        first_sequence = sequence + 1
        step_ns = time.monotonic_ns()
        quest.right.position.x = .1
        first = len(rows)
        pump(.7)
        samples = [r for r in rows[first:] if r['sequence']>=first_sequence]
        fractions = [(r, math.sqrt(sum((a-b)**2 for a,b in zip(r['position'],baseline)))/.1)
                     for r in samples]
        response = {}
        for threshold in (.5, .9):
            crossing = next(((r['source_ns']-step_ns)/1e6 for r, fraction in fractions
                             if fraction>=threshold), None)
            response[str(threshold)] = crossing
        ages = sorted((r['observed_ns']-r['source_ns'])/1e6 for r in rows)
        noise=[]
        for i in range(100):
            quest.right.position.x=.1+(.001 if i%2 else -.001)
            pump(1/72)
            if i>=30:noise.append(rows[-1]['position'])
        mean=[sum(v[k] for v in noise)/len(noise) for k in range(3)]
        rms=math.sqrt(sum(sum((v[k]-mean[k])**2 for k in range(3)) for v in noise)/len(noise))
        quest.right.position.x=.1
        pump(.3)
        settled=rows[-1]['position']
        time.sleep(.3)  # No Quest input: the old filter state must be discarded.
        quest.right.position.x=.4
        recovery_sequence=sequence+1
        pump(.1)
        first_recovered=next(r for r in rows if r['sequence']>=recovery_sequence)
        recovery_distance=math.sqrt(sum((a-b)**2 for a,b in zip(first_recovered['position'],settled)))
        assert abs(recovery_distance-.3)<.001, 'Recovery blended the old pose into the latest target'
        summary = dict(heading_only_orientation_reference='PASS', controller_pitch='PASS',
                       recovery_filter_reset='PASS',trigger_mapping='PASS', gripper_open_limits='PASS',
                       partial_trigger_mapping='PASS', cases=cases,
                       pose_step_cm=10, filtered_response_ms=response,
                       cutoff_requested=args.cutoff,stationary_input_jitter_m=.001,stationary_output_rms_m=rms,
                       mapper_delivery_ms=dict(p50=ages[len(ages)//2],
                           p95=ages[int(len(ages)*.95)], max=max(ages)))
        debug=args.output_dir or root/'work/gripper-lag-debug-software-20261102'
        debug.mkdir(parents=True,exist_ok=True)
        (debug/(args.label+'-summary.json')).write_text(json.dumps(summary,indent=2))
        (debug/(args.label+'-trace.json')).write_text(json.dumps(rows,separators=(',',':')))
        print(json.dumps(summary),flush=True)
        if args.expect_left_open is not None:
            assert abs(cases['released'][1]-args.expect_left_open)<1e-6, 'Left trigger release must command the SDK-verified opening endpoint'
        if args.max_step90_ms is not None:
            assert response['0.9'] is not None and response['0.9']<=args.max_step90_ms, 'Pose smoothing exceeds the step-response target'
        assert rms<=.00025, 'Stationary output jitter must remain below 0.25 mm for a 1 mm input'
    finally:
        if process.poll() is None: process.send_signal(signal.SIGINT)
        try: process.wait(timeout=4)
        except subprocess.TimeoutExpired: process.kill(); process.wait()
        log.close()
        node.destroy_node()
        rclpy.try_shutdown()
