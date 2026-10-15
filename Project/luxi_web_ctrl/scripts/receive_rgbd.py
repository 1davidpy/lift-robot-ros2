#!/usr/bin/env python3
"""Read one atomic aligned RGB/depth response. No control requests are sent."""
import argparse
from email import policy
from email.parser import BytesParser
import json
from urllib.request import urlopen

import cv2
import numpy as np


def decode_rgbd(content_type, body):
    message = BytesParser(policy=policy.default).parsebytes(
        ('Content-Type: ' + content_type + '\r\nMIME-Version: 1.0\r\n\r\n').encode('ascii') + body)
    if message.get_content_type() != 'multipart/mixed' or not message.is_multipart():
        raise ValueError('Expected multipart/mixed RGBD')
    parts = {}
    for part in message.iter_parts():
        name = part.get_param('name', header='content-disposition')
        if name not in ('metadata', 'color', 'depth') or name in parts:
            raise ValueError('Unexpected or duplicate RGBD part')
        parts[name] = part.get_payload(decode=True)
    if set(parts) != {'metadata', 'color', 'depth'}:
        raise ValueError('Missing RGBD part')
    meta = json.loads(parts['metadata'])
    if meta['schema'] != 'luxi_rgbd_v1' or meta['aligned_to_color'] is not True:
        raise ValueError('Expected depth-to-color aligned data')
    d = meta['depth']
    if (meta['color']['encoding'] != 'jpeg' or d['encoding'] != '32FC1'
            or d['unit'] != 'metre' or d['byte_order'] != 'little-endian'):
        raise ValueError('Unsupported RGBD format')
    width, height = meta['width'], meta['height']
    if (not 0 < width <= 1920 or not 0 < height <= 1080 or d['step'] != width * 4
            or len(parts['depth']) != width * height * 4
            or d['bytes'] != len(parts['depth']) or meta['color']['bytes'] != len(parts['color'])):
        raise ValueError('RGBD dimensions or payload size mismatch')
    color_bgr = cv2.imdecode(np.frombuffer(parts['color'], np.uint8), cv2.IMREAD_COLOR)
    if color_bgr is None or color_bgr.shape[:2] != (height, width):
        raise ValueError('Invalid RGB JPEG or dimensions')
    depth_m = np.frombuffer(parts['depth'], dtype='<f4').reshape(height, width).copy()
    return meta, color_bgr, depth_m


def receive_rgbd(base_url, camera='head_rgb'):
    if camera not in ('head_rgb', 'left_wrist_rgb', 'right_wrist_rgb'):
        raise ValueError('Unknown camera')
    with urlopen(base_url.rstrip('/') + '/api/cameras/' + camera + '/rgbd', timeout=5) as response:
        # Bound allocations even if the peer is misconfigured.
        body = response.read(24 * 1024 * 1024 + 1)
        if len(body) > 24 * 1024 * 1024:
            raise ValueError('RGBD response too large')
        meta, color, depth = decode_rgbd(response.headers['Content-Type'], body)
        if meta['pair_id'] != response.headers['X-Pair-Id'] or meta['camera_id'] != camera:
            raise ValueError('RGBD identity mismatch')
        return meta, color, depth


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('base_url')
    parser.add_argument('--camera', default='head_rgb')
    parser.add_argument('--save', action='store_true', help='Save rgb.jpg and depth_m.npy')
    args = parser.parse_args()
    meta, rgb, depth = receive_rgbd(args.base_url, args.camera)
    valid = np.isfinite(depth) & (depth > 0)
    print(json.dumps(meta, ensure_ascii=False))
    print('RGB:', rgb.shape, 'depth:', depth.shape, 'valid fraction:', float(valid.mean()))
    if args.save:
        cv2.imwrite('rgb.jpg', rgb)
        np.save('depth_m.npy', depth)
