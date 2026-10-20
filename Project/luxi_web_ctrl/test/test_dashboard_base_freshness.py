"""No hardware: exercise dashboard telemetry and publisher reconnect on ROS domain 76."""
import json
import os
from pathlib import Path
import signal
import socket
import statistics
import subprocess
import sys
import tempfile
import time
import urllib.request

os.environ['ROS_DOMAIN_ID'] = '76'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
import rclpy
from std_msgs.msg import String

binary = Path(sys.argv[1]).resolve()
port = 18126
with socket.socket() as check:
    check.bind(('127.0.0.1', port))

def state():
    with urllib.request.urlopen(f'http://127.0.0.1:{port}/api/state', timeout=1) as response:
        return json.load(response)

with tempfile.TemporaryDirectory(prefix='luxi-base-freshness-') as directory:
    with open(Path(directory) / 'server.log', 'w') as log:
        server = subprocess.Popen([str(binary), '--ros-args', '-p', f'port:={port}',
                                   '-p', f'directory:={directory}', '-p', 'mock:=true',
                                   '-p', 'no_cameras:=true', '-p', 'base_enabled:=true'],
                                  stdout=log, stderr=subprocess.STDOUT)
        node = None
        try:
            deadline = time.monotonic() + 8
            while True:
                try:
                    state()
                    break
                except Exception:
                    if server.poll() is not None or time.monotonic() >= deadline:
                        raise RuntimeError('isolated dashboard did not start')
                    time.sleep(.1)
            rclpy.init()
            node = rclpy.create_node('dashboard_base_freshness_test')
            pub = node.create_publisher(String, '/luxi/base/state', 20)
            sequence = 0

            def sample_phase(name):
                global sequence
                begin = time.monotonic()
                next_send = next_read = begin
                ages = []
                while time.monotonic() - begin < 6:
                    now = time.monotonic()
                    if now >= next_send:
                        sequence += 1
                        msg = String()
                        msg.data = json.dumps({'schema': 'r680_v1', 'valid': True,
                                               'receive_mono_ns': str(time.monotonic_ns()),
                                               'sequence': sequence, 'mock': True,
                                               'velocity': [0, 0, 0], 'voltage': 24.2,
                                               'flag_stop': 0})
                        pub.publish(msg)
                        next_send = now + .05
                    rclpy.spin_once(node, timeout_sec=.005)
                    if now - begin > 3 and now >= next_read:
                        s = state()
                        b = s.get('base') or {}
                        age = (int(s['server_mono_ns']) - int(b.get('receive_mono_ns', 0))) / 1e6
                        if not b.get('valid') or age < 0 or age >= 250:
                            raise AssertionError(f'{name}: stale/missing state, age {age:.1f} ms')
                        ages.append(age)
                        next_read = now + .1
                if len(ages) < 20:
                    raise AssertionError('insufficient telemetry samples')
                return {'phase': name, 'samples': len(ages),
                        'median_ms': round(statistics.median(ages), 2),
                        'maximum_ms': round(max(ages), 2)}

            result = [sample_phase('dashboard_started_before_publisher')]
            node.destroy_publisher(pub)
            # A stopped publisher must remain offline; no freshness threshold is relaxed.
            time.sleep(.65)
            assert state()['base']['valid'] is False, 'stopped publisher shown online'
            pub = node.create_publisher(String, '/luxi/base/state', 20)
            result.append(sample_phase('publisher_recreated_without_dashboard_restart'))
            print(json.dumps({'passed': True, 'domain': 76, 'results': result,
                              'stale_publisher_rejected': True}, indent=2))
        finally:
            if node is not None:
                node.destroy_node()
                rclpy.shutdown()
            if server.poll() is None:
                server.send_signal(signal.SIGINT)
                try:
                    server.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    server.kill()
                    server.wait()
