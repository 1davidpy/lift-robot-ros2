#!/usr/bin/env python3
"""Run actual camera/gateway processes on an isolated DDS domain, no actuators."""
import copy
import importlib.util
import os
from pathlib import Path
import signal
import struct
import subprocess
import sys
import tempfile
import time
from urllib.error import HTTPError, URLError
from urllib.request import urlopen

os.environ['ROS_DOMAIN_ID'] = '87'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
import rclpy
from rclpy.qos import qos_profile_sensor_data
from luxi_interfaces.msg import RGBDFrame
from sensor_msgs.msg import Image, CameraInfo
import numpy as np

spec = importlib.util.spec_from_file_location('receiver', Path(__file__).parents[1] / 'scripts/receive_rgbd.py')
receiver = importlib.util.module_from_spec(spec)
spec.loader.exec_module(receiver)


def stop(process):
    if process is None or process.poll() is not None:
        return
    process.send_signal(signal.SIGINT)
    try:
        assert process.wait(timeout=6) == 0, 'process failed during shutdown'
    except subprocess.TimeoutExpired:
        process.kill(); process.wait()
        raise AssertionError('process did not stop promptly')


def camera():
    return subprocess.Popen([sys.argv[1], '--ros-args', '-p', 'mock:=true',
                             '-p', 'width:=64', '-p', 'height:=48', '-p', 'fps:=10'])


def status(path):
    try:
        with urlopen('http://127.0.0.1:18031' + path, timeout=1) as response:
            return response.status
    except HTTPError as error:
        return error.code
    except URLError:
        return 0


rclpy.init()
node = rclpy.create_node('rgbd_output_test')
pairs, images, infos = {}, {}, {}
key = lambda m: (m.header.stamp.sec, m.header.stamp.nanosec)
subscriptions = [
    node.create_subscription(RGBDFrame, '/luxi/cameras/head_rgb/rgbd',
        lambda m: pairs.update({key(m): m}), qos_profile_sensor_data),
    node.create_subscription(Image, '/camera/head_rgb/depth/image_raw',
        lambda m: images.update({key(m): m}), qos_profile_sensor_data),
    node.create_subscription(CameraInfo, '/camera/head_rgb/depth/camera_info',
        lambda m: infos.update({key(m): m}), qos_profile_sensor_data),
]
publisher = node.create_publisher(RGBDFrame, '/luxi/cameras/head_rgb/rgbd', qos_profile_sensor_data)
capture, gateway = None, None
with tempfile.TemporaryDirectory(prefix='luxi_rgbd_test_') as directory:
    try:
        gateway = subprocess.Popen([sys.argv[2], '--ros-args', '-p', 'host:=127.0.0.1',
            '-p', 'port:=18031', '-p', 'directory:=' + directory])
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline and status('/api/state') != 200:
            time.sleep(.1)
        assert status('/api/cameras/head_rgb/rgbd') == 503, 'missing pair must return 503'
        capture = camera()
        deadline = time.monotonic() + 12
        common = set()
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
            common = set(pairs) & set(images) & set(infos)
            if len(common) >= 3 and status('/api/cameras/head_rgb/rgbd') == 200:
                break
        assert len(common) >= 3, 'missing paired/standard ROS output'
        for stamp in common:
            p, image, info = pairs[stamp], images[stamp], infos[stamp]
            assert p.header == p.color.header == p.depth.header == p.camera_info.header == image.header == info.header
            assert p.header.frame_id == 'head_rgb_color_optical'
            assert p.sequence == p.color.sequence == p.depth.sequence
            assert bytes(image.data) == bytes(p.depth.data)
            assert image.width == p.color.width == info.width == 64
            assert image.height == p.color.height == info.height == 48
        meta, rgb, depth = receiver.receive_rgbd('http://127.0.0.1:18031')
        assert meta['aligned_to_color'] and meta['mock'] and not meta['hardware_synchronized']
        assert rgb.shape == (48, 64, 3) and depth.shape == (48, 64)
        assert np.allclose(rgb.mean(axis=(0, 1)), [200, 40, 10], atol=5), 'RGB/BGR ordering incorrect'
        assert np.isnan(depth[0, 0]) and abs(float(depth[0, 1]) - 1) < 1e-6
        assert meta['calibration_available'] and len(meta['intrinsics']['K']) == 9
        assert meta['ros_projection_compatible'], 'zero-distortion pinhole must be ROS compatible'
        with urlopen('http://127.0.0.1:18031/api/cameras/head_rgb/depth') as response:
            assert response.headers['X-Aligned-To-Color'] == 'true'
            assert len(response.read()) == 64 * 48 * 4
        old_session = meta['pair_id'].split(':')[0]
        stop(capture); capture = None
        time.sleep(1.2)
        assert status('/api/cameras/head_rgb/rgbd') == 503, 'stale pair returned'
        assert status('/api/cameras/head_rgb/rgbd/info') == 503
        # Fresh malformed bundles may never refresh the last valid pair.
        template = pairs[max(common)]
        for mutate in (lambda p: setattr(p.depth, 'width', 65),
                       lambda p: setattr(p.color.header, 'frame_id', 'wrong'),
                       lambda p: setattr(p, 'pair_id', 'bad\r\nheader'),
                       lambda p: setattr(p.depth, 'data', [1, 2, 3])):
            bad = copy.deepcopy(template)
            bad.color.receive_mono_ns = bad.depth.receive_mono_ns = time.monotonic_ns()
            mutate(bad)
            publisher.publish(bad)
            time.sleep(.15)
            assert status('/api/cameras/head_rgb/rgbd') == 503, 'malformed pair accepted'
        capture = camera()
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline and status('/api/cameras/head_rgb/rgbd') != 200:
            time.sleep(.1)
        restored, _, _ = receiver.receive_rgbd('http://127.0.0.1:18031')
        assert restored['pair_id'].split(':')[0] != old_session, 'restart reused pair identity'
        print('RGBD PASS: atomic multipart decode, ROS pairing, metres/NaN, malformed rejection, expiry and restart')
    finally:
        try:
            stop(capture)
        finally:
            try:
                stop(gateway)
            finally:
                node.destroy_node()
                rclpy.shutdown()
