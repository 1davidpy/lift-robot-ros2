"""SDK against an in-memory UART firmware: no hardware or serial port."""
import importlib.util
from pathlib import Path
import struct
import sys
import time
import unittest

spec=importlib.util.spec_from_file_location('worker_client',Path(__file__).resolve().parents[1]/'tools/lift_uart_client.py')
client=importlib.util.module_from_spec(spec);sys.modules[spec.name]=client;spec.loader.exec_module(client)

class FirmwarePort:
    timeout=None
    write_timeout=None
    def __init__(self):
        self.incoming=bytearray();self.parser=client.FrameParser()
        self.nonce=0;self.last_hb=0.;self.last_sample=0.;self.finish=0.
        self.current=0;self.target=0;self.moves=0;self.expirations=0
        self.heartbeat_acks=True;self.telemetry=True;self.max_gap=0.;self.sequence=0
        self.feedback_age=10;self.telemetry_period=.02

    @property
    def in_waiting(self):return len(self.incoming)

    def write(self,frame):
        now=time.monotonic()
        for _,data in self.parser.feed(frame):
            op,seq,value,_,_=struct.unpack('<BBiBB',data)
            reply=value
            if op==client.OPCODES['hello']:
                self.nonce=value;self.last_hb=now
            elif op==client.OPCODES['keepalive']:
                self.max_gap=max(self.max_gap,now-self.last_hb);self.last_hb=now
                if not self.heartbeat_acks:continue
            elif op in (client.OPCODES['close'],client.OPCODES['stop']):
                self.nonce=0;self.finish=0.
            elif op==client.OPCODES['move']:
                self.moves+=1;self.target=value;self.finish=now+.5
            elif op==client.OPCODES['info']:
                reply={2:-180000,3:180000}.get(value,1)
            flags=19|(64 if self.nonce else 0)
            self.incoming.extend(client.encode_frame(client.FRAME_ACK,struct.pack('<BBBBi',op,seq,0,flags,reply)))
        return len(frame)

    def read(self,size):
        now=time.monotonic()
        if self.nonce and now-self.last_hb>.5:
            self.expirations+=1;self.nonce=0;self.finish=0.
        if self.finish and now>=self.finish:
            self.finish=0.;self.current=self.target
        if self.telemetry and now-self.last_sample>self.telemetry_period:
            self.last_sample=now;self.sequence=(self.sequence+1)&65535
            moving=bool(self.finish);flags=19|(64 if self.nonce else 0)|(8 if moving else 0)
            pages=[struct.pack('<Hih',self.sequence,self.current,20 if moving else 0),
                struct.pack('<HiBB',self.sequence,self.target,2 if moving else 3,flags),
                struct.pack('<HHBBBB',self.sequence,0,0,0,0,0),
                struct.pack('<HIH',self.sequence,int(now*1000)&0xffffffff,self.feedback_age)]
            for page,data in enumerate(pages):
                self.incoming.extend(client.encode_frame(client.FRAME_TELEMETRY,bytes([page])+data))
        result=bytes(self.incoming[:size]);del self.incoming[:size];return result

class WorkerTests(unittest.TestCase):
    def setUp(self):
        self.port=FirmwarePort();self.device=client.LiftClient(self.port)
        self.device.connect();self.addCleanup(self.device.close)

    def test_slow_callback_does_not_expire_or_replay_move(self):
        result=self.device.move_to_mm('20',timeout=5,on_sample=lambda s:time.sleep(.6))
        self.assertEqual(result.current_um,20000)
        self.assertEqual(self.port.moves,1);self.assertEqual(self.port.expirations,0)
        self.assertLess(self.port.max_gap,.3)
        self.assertLessEqual(self.port.timeout,.02)
        self.device.close();self.assertEqual(self.port.nonce,0);self.assertIsNone(self.device._worker)

    def test_caller_pause_keeps_finite_task_alive(self):
        self.device.request(client.OPCODES['move'],10000)
        time.sleep(.8)
        self.assertTrue(self.device.session);self.assertEqual(self.port.expirations,0)
        self.assertEqual(self.port.moves,1)
        self.device.wait_reached(10000,timeout=2)

    def test_slow_callback_with_100ms_telemetry_and_200ms_motor_age(self):
        self.port.feedback_age=200;self.port.telemetry_period=.1
        result=self.device.move_to_mm('20',timeout=3,on_sample=lambda s:time.sleep(.6))
        self.assertEqual(result.current_um,20000)
        self.assertEqual(self.port.moves,1);self.assertEqual(self.port.expirations,0)

    def test_move_after_idle_does_not_reset_feedback_watchdog_origin(self):
        time.sleep(.7)
        self.device.request(client.OPCODES['move'],10000)
        self.device.wait_reached(10000,timeout=2)
        self.assertTrue(self.device.session);self.assertEqual(self.port.moves,1)

    def test_lost_ack_closes_during_slow_callback(self):
        self.port.heartbeat_acks=False
        with self.assertRaisesRegex(client.LiftError,'heartbeat ACK lost'):
            self.device.move_to_mm('20',on_sample=lambda s:time.sleep(.6))
        self.assertFalse(self.device.session);self.assertEqual(self.port.nonce,0)
        count=self.port.moves;time.sleep(.2);self.assertEqual(self.port.moves,count)

    def test_lost_telemetry_abandons_session(self):
        self.device.request(client.OPCODES['move'],10000)
        self.port.telemetry=False
        time.sleep(.7)
        with self.assertRaisesRegex(client.LiftError,'telemetry lost'):
            self.device.wait_reached(10000,timeout=1)
        self.assertFalse(self.device.session);self.assertEqual(self.port.nonce,0)

if __name__=='__main__':unittest.main(verbosity=2)
