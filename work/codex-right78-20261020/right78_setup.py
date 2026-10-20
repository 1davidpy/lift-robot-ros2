"""Targeted DM register/zero setup. No enable or motion commands are implemented."""
import argparse
import json
from pathlib import Path
import socket
import struct
import time

TARGETS = (7, 8)
PARAMS = {7: ('master_id', int), 8: ('motor_id', int), 9: ('timeout', int),
          10: ('control_mode', int), 20: ('gear_ratio', float),
          21: ('pmax', float), 22: ('vmax', float), 23: ('tmax', float),
          35: ('can_baud_code', int)}

class Link:
    def __init__(self, interface, fd=False):
        self.fd = fd
        self.log = []
        self.sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.sock.setsockopt(socket.SOL_CAN_RAW, 5, 1)
        self.sock.bind((interface,))
        self.sock.settimeout(0.05)

    def send(self, canid, data):
        assert canid in (7, 8, 0x7ff)
        assert len(data) == 8
        if canid == 0x7ff:
            assert struct.unpack_from('<H', data)[0] in TARGETS
            assert data[2] in (0x33, 0x55, 0xaa, 0xcc)
            if data[2] == 0x55:
                assert data[3] in (7, 10, 21, 22, 23, 35)
        else:
            assert data[:7] == b'\xff' * 7 and data[7] in (0xfd, 0xfe)
        frame = (struct.pack('=IBBBB64s', canid, 8, 1, 0, 0, data) if self.fd
                 else struct.pack('=IB3x8s', canid, 8, data))
        self.sock.send(frame)
        self.log.append({'direction': 'tx', 'canid': hex(canid), 'fd': self.fd, 'data': data.hex()})

    def receive(self, motor, predicate, timeout=0.3):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            try:
                raw = self.sock.recv(72)
            except socket.timeout:
                continue
            cid = struct.unpack_from('=I', raw)[0]
            data = raw[8:8 + raw[4]]
            self.log.append({'direction': 'rx', 'canid': hex(cid), 'fd': len(raw) == 72, 'data': data.hex()})
            if cid == motor + 0x10 and len(data) == 8 and predicate(data):
                return data, len(raw) == 72
        raise TimeoutError(f'No verified reply from motor {motor}')

    def read(self, motor, rid, kind):
        assert motor in TARGETS
        for attempt in range(3):
            self.send(0x7ff, struct.pack('<HBBI', motor, 0x33, rid, 0))
            try:
                data, _ = self.receive(motor, lambda d: struct.unpack_from('<H', d)[0] == motor and d[2] == 0x33 and d[3] == rid)
                return struct.unpack('<I' if kind is int else '<f', data[4:])[0]
            except TimeoutError:
                if attempt == 2:
                    raise

    def status(self, motor):
        self.send(0x7ff, struct.pack('<HBBI', motor, 0xcc, 0, 0))
        data, fd = self.receive(motor, lambda d: (d[0] & 0xf) == motor and (d[0] >> 4) in (0, 1, 8, 9, 10, 11, 12, 13, 14))
        p = (data[1] << 8) | data[2]
        v = (data[3] << 4) | (data[4] >> 4)
        t = ((data[4] & 15) << 8) | data[5]
        return {'status_code': data[0] >> 4, 'position_rad': p / 65535 * 25 - 12.5,
                'velocity_rad_s': v / 4095 * 60 - 30, 'torque_nm': t / 4095 * 20 - 10,
                'mos_c': data[6], 'rotor_c': data[7], 'reply_fd': fd, 'raw': data.hex()}

    def write(self, motor, rid, value, kind):
        packed = struct.pack('<I' if kind is int else '<f', value)
        self.send(0x7ff, struct.pack('<HBB', motor, 0x55, rid) + packed)
        self.receive(motor, lambda d: struct.unpack_from('<H', d)[0] == motor and d[2] == 0x55 and d[3] == rid and d[4:] == packed)
        readback = self.read(motor, rid, kind)
        if abs(readback - value) > 1e-5:
            raise RuntimeError(f'Write readback mismatch motor={motor} rid={rid}')

    def disable(self, motor):
        self.send(motor, b'\xff' * 7 + b'\xfd')
        time.sleep(0.12)
        state = self.status(motor)
        if state['status_code'] != 0:
            raise RuntimeError(f'Motor {motor} is not disabled: {state}')

    def zero(self, motor):
        self.disable(motor)
        self.send(motor, b'\xff' * 7 + b'\xfe')
        time.sleep(0.2)
        self.disable(motor)
        state = self.status(motor)
        if abs(state['position_rad']) > 0.01:
            raise RuntimeError(f'Motor {motor} zero readback outside tolerance: {state}')
        return state

    def snapshot(self):
        out = {}
        for motor in TARGETS:
            row = {name: self.read(motor, rid, kind) for rid, (name, kind) in PARAMS.items()}
            row['state'] = self.status(motor)
            out[str(motor)] = row
        return out


parser = argparse.ArgumentParser()
parser.add_argument('action', choices=('inspect', 'configure', 'verify'))
parser.add_argument('--fd', action='store_true')
args = parser.parse_args()
link = Link('can2', args.fd)
report = {'action': args.action, 'interface': 'can2', 'targets': TARGETS, 'tx_fd': args.fd,
          'timestamp': time.strftime('%Y-%m-%dT%H:%M:%S%z'), 'writes': []}
try:
    report['before'] = link.snapshot()
    for motor in TARGETS:
        if report['before'][str(motor)]['motor_id'] != motor or report['before'][str(motor)]['master_id'] != motor + 16:
            raise RuntimeError('Motor identity mismatch; refusing writes')
    if args.action == 'configure':
        assert not args.fd
        for motor in TARGETS:
            link.disable(motor)
        for motor in TARGETS:
            for rid, value, kind in ((10, 1, int), (21, 12.5, float), (22, 30.0, float), (23, 10.0, float)):
                old = link.read(motor, rid, kind)
                if abs(old - value) > 1e-5:
                    link.write(motor, rid, value, kind)
                    report['writes'].append({'motor': motor, 'rid': rid, 'before': old, 'after': value})
            report.setdefault('zero_results', {})[str(motor)] = link.zero(motor)
        # Save only after both targeted mechanical zero operations succeed.
        for motor in TARGETS:
            old = link.read(motor, 35, int)
            if old != 9:
                link.write(motor, 35, 9, int)
                report['writes'].append({'motor': motor, 'rid': 35, 'before': old, 'after': 9})
            link.send(0x7ff, struct.pack('<HBBI', motor, 0xaa, 0, 0))
            time.sleep(0.3)
            report.setdefault('save_requests', []).append(motor)
    report['after'] = link.snapshot()
    if args.action == 'verify':
        for motor in TARGETS:
            row = report['after'][str(motor)]
            for name, expected in (('master_id', motor + 16), ('motor_id', motor),
                                   ('control_mode', 1), ('pmax', 12.5),
                                   ('vmax', 30.0), ('tmax', 10.0), ('can_baud_code', 9)):
                if abs(row[name] - expected) > 1e-5:
                    raise RuntimeError(f'Unexpected motor {motor} {name}: {row[name]}')
            if row['state']['status_code'] != 0:
                raise RuntimeError(f'Motor {motor} did not stay disabled')
            if abs(row['state']['position_rad']) > 0.01:
                raise RuntimeError(f'Motor {motor} is not at zero reference')
            if args.fd and not row['state']['reply_fd']:
                raise RuntimeError(f'Motor {motor} did not return CAN FD feedback')
    report['success'] = True
except Exception as exc:
    report['success'] = False
    report['error'] = repr(exc)
    raise
finally:
    report['trace'] = link.log
    path = Path(__file__).with_name(f'{args.action}-{"fd" if args.fd else "classic"}.json')
    path.write_text(json.dumps(report, ensure_ascii=False, indent=2))
    compact = dict(report)
    compact.pop('trace', None)
    print(json.dumps(compact, ensure_ascii=False, indent=2))
    print('REPORT_PATH:', path)
    link.sock.close()
