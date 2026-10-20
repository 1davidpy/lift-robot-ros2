"""Read-only DM motor audit: parameter requests and status refresh only."""
import json
from pathlib import Path
import socket
import struct
import time

class Probe:
    def __init__(self, iface):
        self.s = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.s.setsockopt(socket.SOL_CAN_RAW, 5, 1)
        self.s.bind((iface,))
        self.s.settimeout(0.025)

    def request(self, motor, opcode, rid=0, wait=0.18):
        assert motor in range(1,9) and opcode in (0x33, 0xcc)
        self.s.send(struct.pack('=IB3x8s', 0x7ff, 8, struct.pack('<HBBI', motor, opcode, rid, 0)))
        end = time.monotonic() + wait
        while time.monotonic() < end:
            try:
                raw = self.s.recv(72)
            except socket.timeout:
                continue
            cid = struct.unpack_from('=I',raw)[0]
            d = raw[8:8+raw[4]]
            if cid & 0xe0000000 or len(d) != 8:
                continue
            if opcode == 0x33 and struct.unpack_from('<H',d)[0] == motor and d[2] == 0x33 and d[3] == rid:
                return cid, d, len(raw) == 72
            if opcode == 0xcc and cid == motor+16 and d[0]&15 == motor:
                return cid, d, len(raw) == 72
        return None

out={'timestamp':time.strftime('%Y-%m-%dT%H:%M:%S%z'),'method':'0x33 parameter reads and 0xCC status refresh only','buses':{}}
for iface in ('can2','luxi_left'):
    p = Probe(iface)
    rows=[]
    for motor in range(1,9):
        row={'motor_id_requested':motor}
        response=p.request(motor,0x33,7)
        if response is None:
            row['responding']=False
        else:
            cid,d,fd=response
            row.update(responding=True,master_id=struct.unpack('<I',d[4:])[0],reply_id=hex(cid),parameter_reply_fd=fd)
            for rid,name,kind in ((8,'motor_id',int),(10,'mode',int),(21,'pmax',float),(22,'vmax',float),(23,'tmax',float),(35,'baud_code',int)):
                r=p.request(motor,0x33,rid)
                row[name]=struct.unpack('<I' if kind is int else '<f',r[1][4:])[0] if r else None
            r=p.request(motor,0xcc)
            if r:
                _,d,fd=r
                row['state']={'status_code':d[0]>>4,'position_rad':((d[1]<<8)|d[2])/65535*2*row['pmax']-row['pmax'],
                    'velocity_rad_s':((d[3]<<4)|(d[4]>>4))/4095*2*row['vmax']-row['vmax'],
                    'torque_nm':(((d[4]&15)<<8)|d[5])/4095*2*row['tmax']-row['tmax'],
                    'mos_c':d[6],'rotor_c':d[7],'reply_fd':fd}
        rows.append(row)
    p.s.close()
    out['buses'][iface]=rows
Path(__file__).with_name('arms-readonly.json').write_text(json.dumps(out,ensure_ascii=False,indent=2))
print(json.dumps(out,ensure_ascii=False,indent=2))
