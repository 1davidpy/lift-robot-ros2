#!/usr/bin/env python3
"""Exercise the real mapper in an isolated ROS domain without actuators."""
import argparse
import json
import math
import os
from pathlib import Path
import subprocess
import tempfile
import time

os.environ['ROS_DOMAIN_ID'] = '119'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
import rclpy
from luxi_interfaces.msg import ArmTargets, QuestInput

parser = argparse.ArgumentParser()
parser.add_argument('--binary', type=Path, required=True)
parser.add_argument('--config', type=Path, required=True)
args = parser.parse_args()

def quaternion(p):
    q = p.orientation
    return [q.w, q.x, q.y, q.z]

def multiply(a, b):
    w, x, y, z = a
    v, i, j, k = b
    return [w*v-x*i-y*j-z*k, w*i+x*v+y*k-z*j,
            w*j-x*k+y*v+z*i, w*k+x*j-y*i+z*v]

def inverse(q):
    return [q[0], -q[1], -q[2], -q[3]]

def distance(a, b):
    return 2*math.acos(min(1., abs(sum(x*y for x, y in zip(a, b)))))

rclpy.init()
node = rclpy.create_node('wrist_roll_regression')
latest = {}
offsets = [17., -23.]
processes, subscriptions, publishers = [], [], []
quest = QuestInput()
quest.reference.orientation.w = 1.
quest.left.orientation.w = quest.right.orientation.w = 1.

with tempfile.TemporaryDirectory(prefix='wrist-roll-test-') as temporary:
    directory = Path(temporary)
    env = dict(os.environ, ROS_LOG_DIR=temporary)
    try:
        for index, roll in enumerate(([0., 0.], offsets)):
            config = json.loads(args.config.read_text())
            config['teleop_wrist_roll_offset_degrees'] = roll
            path = directory / f'config-{index}.json'
            path.write_text(json.dumps(config))
            prefix = f'/wrist_roll_test_{os.getpid()}_{index}'
            publishers.append(node.create_publisher(QuestInput, prefix+'/quest', 1))
            def receive(message, index=index):
                latest[index] = message
            subscriptions.append(node.create_subscription(ArmTargets, prefix+'/targets', receive, 1))
            processes.append(subprocess.Popen([
                str(args.binary), '--ros-args', '-p', 'config:='+str(path),
                '-p', 'pose_filter_cutoff_hz:=4.0',
                '-r', '/luxi/quest:='+prefix+'/quest',
                '-r', '/luxi/targets:='+prefix+'/targets'], env=env,
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))

        def pump(seconds):
            end = time.monotonic()+seconds
            while time.monotonic() < end:
                quest.sequence += 1
                quest.receive_mono_ns = time.monotonic_ns()
                for publisher in publishers:
                    publisher.publish(quest)
                rclpy.spin_once(node, timeout_sec=.005)

        deadline = time.monotonic()+8
        while len(latest) < 2 and time.monotonic() < deadline:
            pump(.1)
        assert len(latest) == 2, 'Mapper discovery failed'
        previous = None
        for angle in (0., .35, -.2):
            for pose in (quest.left, quest.right):
                pose.orientation.w = math.cos(angle/2)
                pose.orientation.y = math.sin(angle/2)
            quest.left.position.x = -.15
            quest.right.position.x = .15
            pump(.8)
            for side, name in enumerate(('right', 'left')):
                original = getattr(latest[0], name)
                corrected = getattr(latest[1], name)
                assert abs(original.position.x-corrected.position.x) < 1e-9
                assert abs(original.position.y-corrected.position.y) < 1e-9
                assert abs(original.position.z-corrected.position.z) < 1e-9
                relative = multiply(inverse(quaternion(original)), quaternion(corrected))
                half = math.radians(offsets[side])/2
                assert distance(relative, [math.cos(half), math.sin(half), 0., 0.]) < .003
                if previous is not None:
                    baseline_change = multiply(quaternion(original), inverse(previous[0][side]))
                    corrected_change = multiply(quaternion(corrected), inverse(previous[1][side]))
                    assert distance(baseline_change, corrected_change) < .003, 'Wrist motion changed'
            assert list(latest[0].grippers) == list(latest[1].grippers), 'Gripper mapping changed'
            previous = [[quaternion(getattr(latest[i], side)) for side in ('right', 'left')]
                        for i in (0, 1)]

        for invalid in ([0.], [181., 0.], [0., -181.]):
            config['teleop_wrist_roll_offset_degrees'] = invalid
            path = directory/'invalid.json'
            path.write_text(json.dumps(config))
            result = subprocess.run([str(args.binary), '--ros-args', '-p', 'config:='+str(path)],
                                    env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                                    timeout=5)
            assert result.returncode != 0, 'Invalid wrist calibration accepted'
        print(json.dumps({'passed': True, 'offset_degrees_right_left': offsets,
                          'position_gripper_and_wrist_motion_preserved': True}))
    finally:
        for process in processes:
            process.terminate()
        for process in processes:
            process.wait(timeout=5)
        node.destroy_node()
        rclpy.shutdown()
