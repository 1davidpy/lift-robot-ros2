"""Pure host tests for the UART protocol/client. No serial device or motor is used."""
import importlib.util
from pathlib import Path
import struct
import sys
import unittest

MODULE_PATH = Path(__file__).resolve().parents[1] / "tools" / "lift_uart_client.py"
SPEC = importlib.util.spec_from_file_location("lift_uart_client", MODULE_PATH)
client = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = client
SPEC.loader.exec_module(client)


def telemetry_pages(sequence=7, current=-50000, target=-50000, speed=0,
                    status=3, flags=19, fault=0, raw=0, stable=0,
                    result=0, guard=0, uptime=1000, age=20):
    return [
        struct.pack("<Hih", sequence, current, speed),
        struct.pack("<HiBB", sequence, target, status, flags),
        struct.pack("<HHBBBB", sequence, fault, raw, stable, result, guard),
        struct.pack("<HIH", sequence, uptime, age),
    ]


class ProtocolTests(unittest.TestCase):
    def test_live_ack_callback_filters_unrelated_replies(self):
        class Port:
            incoming = bytearray()
            @property
            def in_waiting(self): return len(self.incoming)
            def write(self, frame):
                _, data = client.FrameParser().feed(frame)[0]
                op, seq, value = data[0], data[1], struct.unpack_from('<i', data, 2)[0]
                for reply_op, reply_seq, result, reply_value in (
                    (op, (seq + 1) % 256, 0, value),
                    (op, seq, 0, value + 1),
                    (op, seq, client.QUEUED, value),
                    (op, seq, 0, value),
                ):
                    self.incoming.extend(client.encode_frame(client.FRAME_ACK,
                        struct.pack('<BBBBi', reply_op, reply_seq, result, 115, reply_value)))
                return len(frame)
            def read(self, size):
                data = bytes(self.incoming[:size]); del self.incoming[:size]; return data
        device = client.LiftClient(Port())
        device.session = True
        device.last_heartbeat = device.clock()
        received = []
        reply = device.request(client.OPCODES['move'], -20000, on_ack=received.append)
        self.assertEqual([ack.result for ack in received], [client.QUEUED, 0])
        self.assertEqual(reply.value, -20000)

    def test_reset_ack_stops_old_session_heartbeat(self):
        class Port:
            incoming = bytearray()
            @property
            def in_waiting(self): return len(self.incoming)
            def write(self, frame):
                _, data = client.FrameParser().feed(frame)[0]
                self.incoming.extend(client.encode_frame(client.FRAME_ACK,
                    struct.pack('<BBBBi', data[0], data[1], 0, 115,
                                struct.unpack_from('<i', data, 2)[0])))
                return len(frame)
            def read(self, size):
                data = bytes(self.incoming[:size]); del self.incoming[:size]; return data
        device = client.LiftClient(Port())
        device.session = True
        device.last_heartbeat = device.clock()
        device.request(client.OPCODES['reset'], client.RESET_KEY)
        self.assertFalse(device.session)
        self.assertIsNone(device.heartbeat)

    def test_known_crc_vector(self):
        self.assertEqual(client.crc16_ccitt(b"123456789"), 0x29B1)

    def test_frame_round_trip_fragmented_and_noise(self):
        payload = client.encode_request(client.OPCODES["move"], 4, -50000)
        frame = client.encode_frame(client.FRAME_REQUEST, payload)
        parser = client.FrameParser()
        self.assertEqual(parser.feed(b"noise" + frame[:3]), [])
        self.assertEqual(parser.feed(frame[3:]), [(client.FRAME_REQUEST, payload)])

    def test_bad_crc_is_discarded_and_parser_recovers(self):
        payload = client.encode_request(client.OPCODES["stop"], 9)
        bad = bytearray(client.encode_frame(client.FRAME_REQUEST, payload))
        bad[-1] ^= 1
        parser = client.FrameParser()
        self.assertEqual(parser.feed(bytes(bad) + client.encode_frame(client.FRAME_REQUEST, payload)),
                         [(client.FRAME_REQUEST, payload)])
        self.assertEqual(parser.errors, 1)

    def test_request_layout_and_version(self):
        data = client.encode_request(client.OPCODES["move"], 0xA7, -180000)
        self.assertEqual(data, bytes((0x10, 0xA7, 0xE0, 0x40, 0xFD, 0xFF, 0, 1)))

    def test_request_validation(self):
        with self.assertRaises(ValueError):
            client.encode_request(client.OPCODES["move"], 0, 180001)
        with self.assertRaises(ValueError):
            client.encode_request(client.OPCODES["goto-point"], 0, 0, 2)
        self.assertEqual(client.encode_request(client.OPCODES["goto-point"], 0, 0, 3)[6], 3)
        with self.assertRaises(ValueError):
            client.encode_request(0xFF, 0)

    def test_mm_conversion(self):
        self.assertEqual(client.millimeters_to_um("-50.125"), -50125)
        self.assertEqual(client.millimeters_to_um("180"), 180000)
        for value in ("180.001", "-180.001", "0.0001", "nan"):
            with self.assertRaises(ValueError):
                client.millimeters_to_um(value)

    def test_reset_and_ready_info(self):
        data = client.encode_request(client.OPCODES['reset'], 7, client.RESET_KEY)
        self.assertEqual(data[0], 0x1A)
        with self.assertRaises(ValueError):
            client.encode_request(client.OPCODES['reset'], 7, 0)
        for page in range(25, 31):
            self.assertEqual(client.encode_request(client.OPCODES['info'], 9, page)[2], page)
        with self.assertRaises(ValueError):
            client.encode_request(client.OPCODES['info'], 9, 31)

    def test_ack_decode(self):
        ack = client.decode_ack(struct.pack("<BBBBi", 0x10, 2, 0, 0x51, -50000))
        self.assertEqual((ack.opcode, ack.sequence, ack.flags, ack.value),
                         (0x10, 2, 0x51, -50000))

    def test_four_page_telemetry_requires_same_sequence(self):
        assembler = client.TelemetryAssembler()
        sample = None
        for page, data in enumerate(telemetry_pages()):
            sample = assembler.feed(page, data, 1.0)
        self.assertEqual((sample.current_um, sample.target_um, sample.speed_rpm),
                         (-50000, -50000, 0))
        self.assertEqual(sample.health_state(1.1), "ok")

    def test_mixed_sequences_do_not_join(self):
        assembler = client.TelemetryAssembler()
        first = telemetry_pages(sequence=1)
        second = telemetry_pages(sequence=2)
        self.assertIsNone(assembler.feed(0, first[0], 1.0))
        for page in (1, 2, 3):
            self.assertIsNone(assembler.feed(page, second[page], 1.0))

    def test_health_labels_distinguish_link_and_position(self):
        assembler = client.TelemetryAssembler()
        for page, data in enumerate(telemetry_pages(flags=18)):
            sample = assembler.feed(page, data, 1.0)
        self.assertEqual(sample.health_state(1.1), "position_invalid")
        self.assertEqual(sample.health_state(1.4), "uart_feedback_timeout")

    def test_reached_requires_fresh_stationary_valid_feedback(self):
        assembler = client.TelemetryAssembler()
        for page, data in enumerate(telemetry_pages(flags=19)):
            sample = assembler.feed(page, data, 1.0)
        self.assertTrue(sample.reached(-50000, 0.9, 1.1))
        self.assertFalse(sample.reached(0, 0.9, 1.1))
        self.assertFalse(sample.reached(-50000, 1.1, 1.1))


if __name__ == "__main__":
    unittest.main()
