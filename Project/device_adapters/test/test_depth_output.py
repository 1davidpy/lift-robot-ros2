#!/usr/bin/env python3
"""Exercise the real depth node's synthetic source, standard ROS outputs and shutdown."""
import math
import os
import signal
import struct
import subprocess
import sys
import time

os.environ['ROS_DOMAIN_ID'] = '84'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from luxi_interfaces.msg import CameraFrame

rclpy.init()
node = rclpy.create_node('depth_output_test')
images, infos, frames = {}, {}, {}
def key(msg):
    return msg.header.stamp.sec, msg.header.stamp.nanosec
subscriptions = [
    node.create_subscription(Image, '/camera/head_rgb/depth/image_raw',
                             lambda m: images.update({key(m): m}), qos_profile_sensor_data),
    node.create_subscription(CameraInfo, '/camera/head_rgb/depth/camera_info',
                             lambda m: infos.update({key(m): m}), qos_profile_sensor_data),
    node.create_subscription(CameraFrame, '/luxi/cameras/head_rgb/depth',
                             lambda m: frames.update({key(m): m}), qos_profile_sensor_data),
]
process = subprocess.Popen([sys.argv[1], '--ros-args', '-p', 'mock:=true',
                            '-p', 'width:=64', '-p', 'height:=48', '-p', 'fps:=10'])
try:
    deadline = time.monotonic() + 15
    common = set()
    while time.monotonic() < deadline and len(common) < 3:
        rclpy.spin_once(node, timeout_sec=.1)
        common = set(images) & set(infos) & set(frames)
        assert process.poll() is None, 'depth node exited early'
    assert len(common) >= 3, 'missing standard images/calibration or gateway frames'
    for stamp in common:
        image, info, frame = images[stamp], infos[stamp], frames[stamp]
        assert image.encoding == frame.encoding == '32FC1'
        assert not image.is_bigendian and image.step == 64 * 4
        assert image.width == info.width == frame.width == 64
        assert image.height == info.height == frame.height == 48
        assert image.header.frame_id == info.header.frame_id == frame.header.frame_id == 'head_rgb_depth_optical'
        assert frame.timestamp_quality == 'mock_host'
        assert bytes(image.data) == bytes(frame.data)
        first, second = struct.unpack_from('<ff', bytes(image.data))
        assert math.isnan(first) and abs(second - 1.) < 1e-6, 'invalid/unit conversion wrong'
        assert info.k[0] > 0 and info.k[4] > 0
    assert len({frames[s].sequence for s in common}) == len(common)
    print('Depth conversion, NaN invalids, paired CameraInfo, standard ROS images and sequencing passed')
finally:
    process.send_signal(signal.SIGINT)
    try:
        process.wait(timeout=5)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait()
        raise AssertionError('depth node did not stop promptly')
    node.destroy_node()
    rclpy.shutdown()
assert process.returncode == 0, 'depth node failed during shutdown'
