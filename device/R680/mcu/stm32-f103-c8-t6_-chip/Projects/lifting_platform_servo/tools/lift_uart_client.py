"""UART host client for lifting_platform_servo protocol v1.

The module uses 115200 8N1 on PA9/PA10. Commands are binary framed and CRC
protected; this program deliberately performs no automatic reconnect or blind
motion retry after an uncertain outcome.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
from decimal import Decimal, InvalidOperation
import json
import secrets
import struct
import time
import threading
from functools import wraps
from typing import Callable

VERSION = 1
SOF = b"\xA5\x5A"
FRAME_REQUEST = 0x01
FRAME_ACK = 0x81
FRAME_TELEMETRY = 0x90
SESSION_SECONDS = 0.5
HEARTBEAT_SECONDS = 0.1
QUEUED = 0xFE
OPCODES = {
    "hello": 0x01, "keepalive": 0x02, "stop": 0x03, "info": 0x04,
    "close": 0x05, "move": 0x10, "home": 0x11, "set-center": 0x12,
    "abs-check": 0x13, "abs-read": 0x14, "goto-point": 0x16,
    "restore-ref": 0x17, "speed": 0x18, "boot-home": 0x19, "reset": 0x1A,
}
SET_CENTER_KEY = 0x33455453
RESET_KEY = 0x52535421


class LiftError(RuntimeError):
    """A command was rejected, timed out, or lost its control session."""


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            crc = ((crc << 1) ^ (0x1021 if crc & 0x8000 else 0)) & 0xFFFF
    return crc


def encode_frame(frame_type: int, data: bytes) -> bytes:
    if not 1 <= frame_type <= 255 or len(data) > 16:
        raise ValueError("invalid UART frame type or length")
    body = bytes((len(data) + 1, frame_type)) + data
    return SOF + body + struct.pack("<H", crc16_ccitt(body))


class FrameParser:
    """Incremental parser that discards noise and CRC-corrupt frames."""

    def __init__(self):
        self.buffer = bytearray()
        self.errors = 0

    def feed(self, data: bytes) -> list[tuple[int, bytes]]:
        self.buffer.extend(data)
        frames = []
        while True:
            start = self.buffer.find(SOF)
            if start < 0:
                self.buffer[:] = self.buffer[-1:] if self.buffer.endswith(SOF[:1]) else b""
                break
            if start:
                del self.buffer[:start]
            if len(self.buffer) < 3:
                break
            length = self.buffer[2]
            if not 1 <= length <= 17:
                del self.buffer[0]
                self.errors += 1
                continue
            total = length + 5
            if len(self.buffer) < total:
                break
            frame = bytes(self.buffer[:total])
            del self.buffer[:total]
            expected = struct.unpack_from("<H", frame, total - 2)[0]
            if crc16_ccitt(frame[2:-2]) != expected:
                self.errors += 1
                continue
            frames.append((frame[3], frame[4:-2]))
        return frames


def millimeters_to_um(text: str) -> int:
    try:
        value = Decimal(text) * 1000
    except InvalidOperation as exc:
        raise ValueError("height must be a decimal number of millimetres") from exc
    if not value.is_finite() or value != value.to_integral_value():
        raise ValueError("height must be finite with at most 0.001 mm resolution")
    if not -180000 <= value <= 180000:
        raise ValueError("height is outside -180 to +180 mm")
    return int(value)


def encode_request(opcode: int, sequence: int, value: int = 0, argument: int = 0) -> bytes:
    if not 0 <= sequence <= 255 or not 0 <= argument <= 255:
        raise ValueError("sequence and argument must be bytes")
    if opcode not in OPCODES.values():
        raise ValueError("unknown opcode")
    if not -(1 << 31) <= value < (1 << 31):
        raise ValueError("value must fit signed int32")
    if opcode == OPCODES["goto-point"]:
        valid = value == 0 and argument == 3
    elif argument:
        valid = False
    elif opcode in (OPCODES["hello"], OPCODES["keepalive"], OPCODES["close"]):
        valid = value != 0
    elif opcode == OPCODES["info"]:
        valid = 0 <= value <= 30
    elif opcode == OPCODES["move"]:
        valid = -180000 <= value <= 180000
    elif opcode == OPCODES["speed"]:
        valid = 120 <= value <= 3000
    elif opcode == OPCODES["boot-home"]:
        valid = value in (0, 1)
    elif opcode == OPCODES["reset"]:
        valid = value == RESET_KEY
    elif opcode == OPCODES["set-center"]:
        valid = value == SET_CENTER_KEY
    else:
        valid = value == 0
    if not valid:
        raise ValueError("invalid value/argument for opcode")
    return struct.pack("<BBiBB", opcode, sequence, value, argument, VERSION)


@dataclass(frozen=True)
class Ack:
    opcode: int
    sequence: int
    result: int
    flags: int
    value: int


def decode_ack(data: bytes) -> Ack:
    if len(data) != 8:
        raise ValueError("ACK payload must be exactly 8 bytes")
    return Ack(*struct.unpack("<BBBBi", data))


@dataclass(frozen=True)
class Telemetry:
    sequence: int
    current_um: int
    speed_rpm: int
    target_um: int
    target_status: int
    flags: int
    fault: int
    raw_mask: int
    stable_mask: int
    result: int
    guard: int
    uptime_ms: int
    feedback_age_ms: int
    received_monotonic: float
    received_first_monotonic: float

    @property
    def current_mm(self) -> float:
        return self.current_um / 1000

    def feedback_age_at(self, now: float) -> float:
        return self.feedback_age_ms + max(0.0, now - self.received_first_monotonic) * 1000

    def health_state(self, now: float) -> str:
        if now - self.received_monotonic > 0.3:
            return "uart_feedback_timeout"
        if self.feedback_age_at(now) > 500:
            return "motor_feedback_stale"
        if self.fault:
            return "drive_fault"
        if not self.flags & 0x01:
            return "position_invalid"
        if self.flags & 0x80:
            return "motion_inhibited"
        return "ok"

    def reached(self, target_um: int | None, after: float, now: float) -> bool:
        return (
            self.received_monotonic >= after
            and now - self.received_monotonic <= 0.3
            and self.feedback_age_at(now) <= 250
            and self.flags & 3 == 3
            and not self.flags & (4 | 8 | 128)
            and self.fault == 0
            and self.target_status == 3
            and abs(self.speed_rpm) <= 2
            and abs(self.current_um - self.target_um) <= 10
            and (target_um is None or self.target_um == target_um)
        )


class TelemetryAssembler:
    def __init__(self):
        self.pending: dict[int, tuple[float, dict[int, bytes]]] = {}
        self.last_completed: tuple[int, int] | None = None

    def feed(self, page: int, data: bytes, now: float) -> Telemetry | None:
        self.pending = {seq: item for seq, item in self.pending.items() if now - item[0] <= 0.3}
        if page not in range(4) or len(data) != 8:
            return None
        sequence = struct.unpack_from("<H", data)[0]
        if sequence not in self.pending:
            if len(self.pending) >= 4:
                del self.pending[min(self.pending, key=lambda seq: self.pending[seq][0])]
            self.pending[sequence] = (now, {})
        first, pages = self.pending[sequence]
        if page in pages and pages[page] != data:
            del self.pending[sequence]
            return None
        pages[page] = bytes(data)
        if len(pages) != 4:
            return None
        del self.pending[sequence]
        current, speed = struct.unpack_from("<ih", pages[0], 2)
        target, status, flags = struct.unpack_from("<iBB", pages[1], 2)
        fault, raw, stable, result, guard = struct.unpack_from("<HBBBB", pages[2], 2)
        uptime, age = struct.unpack_from("<IH", pages[3], 2)
        if self.last_completed == (sequence, uptime):
            return None
        self.last_completed = (sequence, uptime)
        return Telemetry(sequence, current, speed, target, status, flags, fault,
                         raw, stable, result, guard, uptime, age, now, first)


def _io_locked(method):
    @wraps(method)
    def locked(self, *args, **kwargs):
        with self._io_lock:
            return method(self, *args, **kwargs)
    return locked


class LiftClient:
    """One command caller; independent serial/heartbeat worker after connect().

    User callbacks run on the caller, never on the heartbeat worker. The caller
    must close() in a finally block. A worker fault closes the session and never
    reconnects or resends a motion command automatically.
    """

    def __init__(self, serial_port, *, clock: Callable[[], float] = time.monotonic):
        self.serial = serial_port
        # A long serial read must not starve the 100 ms device keepalive.
        if hasattr(serial_port, "timeout") and (serial_port.timeout is None or serial_port.timeout > .02):
            serial_port.timeout = .02
        if hasattr(serial_port, "write_timeout") and (serial_port.write_timeout is None or serial_port.write_timeout > .2):
            serial_port.write_timeout = .2
        self._io_lock = threading.RLock()
        self._worker_stop = threading.Event()
        self._worker: threading.Thread | None = None
        self._worker_error: BaseException | None = None
        self._session_started = 0.0
        self._last_telemetry_received = 0.0
        self._recent_samples: list[Telemetry] = []
        self.clock = clock
        self.parser = FrameParser()
        self.assembler = TelemetryAssembler()
        self.session = False
        self.nonce = 0
        self.next_sequence = 0
        self.used_sequences: dict[int, float] = {}
        self.heartbeat: tuple[int, float] | None = None
        self.last_heartbeat = 0.0
        self.pending_request: tuple[int, int] | None = None
        self.acks: list[Ack] = []
        self.samples: list[Telemetry] = []
        self.command_started = 0.0
        self.latest: Telemetry | None = None
        self.last_device_uptime: int | None = None

    @_io_locked
    def _sequence(self) -> int:
        now = self.clock()
        for _ in range(256):
            sequence = self.next_sequence
            self.next_sequence = (sequence + 1) & 255
            if now - self.used_sequences.get(sequence, -1e30) >= 1.0:
                self.used_sequences[sequence] = now
                return sequence
        raise LiftError("sequence space exhausted")

    @_io_locked
    def _send(self, payload: bytes) -> None:
        frame = encode_frame(FRAME_REQUEST, payload)
        if self.serial.write(frame) != len(frame):
            raise LiftError("short UART write")

    def _service_heartbeat(self) -> None:
        if not self.session:
            return
        now = self.clock()
        if self.heartbeat is not None:
            if now - self.heartbeat[1] > 0.3:
                raise LiftError("heartbeat ACK lost; session abandoned")
            return
        if now - self.last_heartbeat >= HEARTBEAT_SECONDS:
            sequence = self._sequence()
            self._send(encode_request(OPCODES["keepalive"], sequence, self.nonce))
            self.heartbeat = (sequence, now)
            self.last_heartbeat = now

    def _pump_io(self, service_heartbeat: bool = True, *, consume_sample: bool = True) -> Telemetry | None:
        if service_heartbeat:
            self._service_heartbeat()
        waiting = getattr(self.serial, "in_waiting", 0)
        chunk = self.serial.read(max(1, min(waiting or 1, 256)))
        for frame_type, data in self.parser.feed(chunk):
            if frame_type == FRAME_ACK and len(data) == 8:
                ack = decode_ack(data)
                if ack.opcode == OPCODES["keepalive"] and self.heartbeat and ack.sequence == self.heartbeat[0]:
                    if ack.result != 0 or ack.value != self.nonce or not ack.flags & 64:
                        raise LiftError(f"heartbeat rejected: 0x{ack.result:02X}")
                    self.heartbeat = None
                elif self.pending_request == (ack.opcode, ack.sequence):
                    self.acks.append(ack)
            elif frame_type == FRAME_TELEMETRY and len(data) == 9:
                sample = self.assembler.feed(data[0], data[1:], self.clock())
                if sample is not None:
                    if self.last_device_uptime is not None:
                        elapsed = (sample.uptime_ms - self.last_device_uptime) & 0xFFFFFFFF
                        if elapsed == 0:
                            continue
                        if elapsed >= 0x80000000:
                            self.last_device_uptime = None
                            self.latest = None
                            self.assembler.pending.clear()
                            raise LiftError("device restarted or telemetry moved backwards")
                    self.last_device_uptime = sample.uptime_ms
                    self.latest = sample
                    self._last_telemetry_received = sample.received_monotonic
                    self._recent_samples[:] = [*self._recent_samples[-1:], sample]
                    self.samples.append(sample)
                    self.samples[:] = self.samples[-32:]
        return self.samples.pop(0) if consume_sample and self.samples else None

    def _pump(self, service_heartbeat: bool = True) -> Telemetry | None:
        with self._io_lock:
            if self._worker is None:
                return self._pump_io(service_heartbeat)
            if self._worker_error is not None:
                raise LiftError(f"UART session stopped: {self._worker_error}") from self._worker_error
            # Slow callbacks must receive current feedback rather than drain an
            # ever-growing backlog of obsolete samples.
            sample = self.samples[-1] if self.samples else None
            self.samples.clear()
        if sample is None:
            time.sleep(.002)
        return sample

    def _run_io(self) -> None:
        try:
            while not self._worker_stop.wait(.002):
                with self._io_lock:
                    if not self.session:
                        break
                    self._pump_io(consume_sample=False)
                    last = self._last_telemetry_received or self._session_started
                    if self.clock() - last > .5:
                        raise LiftError("fresh complete UART telemetry lost during session")
                    if self.latest and (self.latest.fault or self.latest.flags & 128):
                        raise LiftError("drive fault or motion guard during session")
        except BaseException as exc:
            with self._io_lock:
                self._worker_error = exc
                self.abandon_session()

    def _stop_worker(self) -> None:
        self._worker_stop.set()
        worker = self._worker
        if worker is not None and worker is not threading.current_thread():
            worker.join(timeout=1)
            if worker.is_alive():
                raise LiftError("UART worker did not stop; do not reuse the port")
        self._worker = None

    @_io_locked
    def abandon_session(self) -> None:
        owned = self.session
        self.session = False
        self._worker_stop.set()
        self.heartbeat = None
        if owned:
            try:
                self._send(encode_request(OPCODES["close"], self._sequence(), self.nonce))
            except Exception:
                pass

    def request(self, opcode: int, value: int = 0, argument: int = 0,
                *, timeout: float = 2.0,
                on_sample: Callable[[Telemetry], None] | None = None,
                on_ack: Callable[[Ack], None] | None = None) -> Ack:
        with self._io_lock:
            if self.pending_request is not None:
                raise LiftError("nested/concurrent requests are not supported")
            if opcode not in (OPCODES["hello"], OPCODES["stop"], OPCODES["info"]) and not self.session:
                raise LiftError("this command requires connect() first")
            sequence = self._sequence()
            payload = encode_request(opcode, sequence, value, argument)
            self.pending_request = (opcode, sequence)
            self.acks.clear()
        started = self.clock()
        last_send = started
        attempts = 1
        try:
            self._send(payload)
            while self.clock() - started < timeout:
                sample = self._pump(opcode != OPCODES["close"])
                if sample is not None and on_sample:
                    on_sample(sample)
                while True:
                    with self._io_lock:
                        ack = self.acks.pop(0) if self.acks else None
                    if ack is None:
                        break
                    if opcode != OPCODES["info"] and ack.value != value:
                        continue
                    if on_ack:
                        on_ack(ack)
                    if ack.result == QUEUED:
                        attempts = 3
                        continue
                    if ack.result != 0:
                        raise LiftError(f"opcode 0x{opcode:02X} rejected: result 0x{ack.result:02X}")
                    if opcode in (OPCODES["move"], OPCODES["home"], OPCODES["goto-point"]):
                        with self._io_lock:
                            self.command_started = self.clock()
                            self.latest = None
                            self.samples.clear()
                            self._recent_samples.clear()
                            self.assembler.pending.clear()
                    if opcode == OPCODES["reset"]:
                        self.session = False
                        self.heartbeat = None
                        self._worker_stop.set()
                    return ack
                now = self.clock()
                if attempts < 3 and now - last_send >= 0.1 and now - started < 0.4:
                    self._send(payload)
                    last_send = now
                    attempts += 1
            raise LiftError("command ACK timeout; outcome unknown, do not repeat motion blindly")
        except BaseException:
            self.abandon_session()
            raise
        finally:
            with self._io_lock:
                self.pending_request = None
                self.acks.clear()

    def connect(self) -> Ack:
        if self.session:
            raise LiftError("already connected")
        self._stop_worker()
        self._worker_error = None
        self.nonce = secrets.randbelow((1 << 31) - 1) + 1
        ack = self.request(OPCODES["hello"], self.nonce)
        if not ack.flags & 64:
            raise LiftError("HELLO ACK did not establish a session")
        self.session = True
        self.last_heartbeat = self.clock() - HEARTBEAT_SECONDS
        self.last_device_uptime = None
        self.latest = None
        self.samples.clear()
        self._recent_samples.clear()
        self._last_telemetry_received = 0.0
        self._session_started = self.clock()
        self._worker_stop.clear()
        self._worker = threading.Thread(target=self._run_io, name="lift-uart-io", daemon=True)
        self._worker.start()
        return ack

    def close(self) -> None:
        self._stop_worker()
        if self.session:
            try:
                self.request(OPCODES["close"], self.nonce)
            finally:
                self.session = False
                self.heartbeat = None

    def monitor(self, duration: float, on_sample: Callable[[Telemetry], None]) -> None:
        started = self.clock()
        try:
            while self.clock() - started < duration:
                sample = self._pump()
                if sample is not None:
                    on_sample(sample)
        except BaseException:
            self.abandon_session()
            raise

    def wait_reached(self, target_um: int | None, *, timeout: float = 300.0,
                     on_sample: Callable[[Telemetry], None] | None = None) -> Telemetry:
        started = self.clock()
        last_sample = started
        previous = None
        try:
            while self.clock() - started < timeout:
                sample = self._pump()
                if sample is None:
                    if self.clock() - last_sample > 0.5:
                        raise LiftError("fresh complete UART telemetry lost during motion")
                    continue
                last_sample = self.clock()
                if on_sample:
                    on_sample(sample)
                if self._worker is not None:
                    with self._io_lock:
                        if self._worker_error is not None:
                            raise LiftError(f"UART session stopped: {self._worker_error}") from self._worker_error
                        recent = list(self._recent_samples)
                    # Arrival is judged from consecutive fresh UART samples,
                    # independently of the user's callback execution time.
                    # The earlier sample must have been fresh when received;
                    # only the current sample must still be fresh now. Re-aging
                    # both at callback return can make 100 ms telemetry with
                    # 150-250 ms motor feedback age impossible to confirm.
                    if (len(recent) == 2
                            and recent[0].reached(target_um, self.command_started, recent[0].received_monotonic)
                            and recent[1].reached(target_um, self.command_started, self.clock())):
                        elapsed = (recent[1].uptime_ms - recent[0].uptime_ms) & 0xFFFFFFFF
                        if recent[1].sequence != recent[0].sequence and 0 < elapsed <= 300:
                            return recent[1]
                if sample.fault or sample.flags & 128 or sample.target_status >= 0xE0:
                    raise LiftError(f"motion stopped/rejected: status=0x{sample.target_status:02X}, fault=0x{sample.fault:04X}")
                if not sample.flags & 64 or not sample.flags & 1:
                    raise LiftError("session or valid position lost during motion")
                if sample.reached(target_um, self.command_started, self.clock()):
                    if previous is not None:
                        elapsed = (sample.uptime_ms - previous.uptime_ms) & 0xFFFFFFFF
                        if sample.sequence != previous.sequence and 0 < elapsed <= 300:
                            return sample
                    previous = sample
                else:
                    previous = None
            raise LiftError("motion completion timeout")
        except BaseException:
            self.abandon_session()
            raise

    def read_configuration(self) -> dict[str, int]:
        names = ("protocol_version", "firmware_version", "min_um", "max_um",
                 "absolute_stage", "profile_flags", "cruise_rpm", "lower_edge_um",
                 "upper_edge_um", "saved_mask")
        return {name: self.request(OPCODES["info"], page).value
                for page, name in enumerate(names)}

    def wait_ready(self, *, timeout: float = 120.0,
                   on_sample: Callable[[Telemetry], None] | None = None) -> Telemetry:
        """Wait through motor-first/USB-first startup; never issue a move or rebase."""
        started = self.clock()
        while self.clock() - started < timeout:
            self.monitor(0.2, on_sample or (lambda sample: None))
            sample = self.latest
            if sample is None or self.clock() - sample.received_monotonic > 0.3:
                continue
            if sample.fault or sample.flags & 128:
                raise LiftError("drive fault or motion guard; not ready")
            if (sample.flags & 1 and not sample.flags & 12 and abs(sample.speed_rpm) <= 2
                    and sample.feedback_age_at(self.clock()) <= 500):
                ready = self.request(OPCODES["info"], 25, on_sample=on_sample)
                if ready.value == 1:
                    return self.latest or sample
        raise LiftError("device not ready before deadline; check motor power/reference")

    def move_to_mm(self, height: str, *, timeout: float = 300.0,
                   on_sample: Callable[[Telemetry], None] | None = None) -> Telemetry:
        target = millimeters_to_um(str(height))
        if not self.session:
            raise LiftError("move_to_mm requires connect() first")
        lower = self.request(OPCODES["info"], 2).value
        upper = self.request(OPCODES["info"], 3).value
        if not lower <= target <= upper:
            raise ValueError(f"target outside configured limits {lower / 1000}..{upper / 1000} mm")
        self.request(OPCODES["move"], target, on_sample=on_sample)
        return self.wait_reached(target, timeout=timeout, on_sample=on_sample)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True, help="serial port, e.g. COM5 or /dev/ttyUSB0")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--timeout", type=float, default=300)
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("move").add_argument("value", help="height in mm")
    sub.add_parser("info").add_argument("value", type=int)
    sub.add_parser("goto-point").add_argument("value", type=int)
    sub.add_parser("speed").add_argument("value", type=int)
    sub.add_parser("boot-home").add_argument("value", type=int)
    for name in ("monitor", "ready", "reset", "stop", "home", "set-center", "abs-check", "abs-read", "restore-ref"):
        sub.add_parser(name)
    args = parser.parse_args(argv)
    import serial
    port = serial.Serial(args.port, args.baudrate, timeout=0.02, write_timeout=0.2)
    client = LiftClient(port)
    report = lambda sample: print(json.dumps(asdict(sample), ensure_ascii=True), flush=True)
    try:
        if args.command == "monitor":
            client.monitor(args.timeout, report)
            return 0
        if args.command == "ready":
            client.wait_ready(timeout=args.timeout, on_sample=report)
            print('{"event":"ready"}', flush=True)
            return 0
        if args.command not in ("stop", "info"):
            client.connect()
        if args.command == "move":
            client.move_to_mm(args.value, timeout=args.timeout, on_sample=report)
            return 0
        value = getattr(args, "value", 0)
        argument = 0
        if args.command == "goto-point":
            if value != 3:
                raise ValueError("only centre point 3 is supported")
            argument, value = value, 0
        elif args.command == "set-center":
            value = SET_CENTER_KEY
        elif args.command == "reset":
            value = RESET_KEY
        ack = client.request(OPCODES[args.command], value, argument)
        print(json.dumps(asdict(ack)), flush=True)
        if args.command in ("home", "goto-point"):
            client.wait_reached(0, timeout=args.timeout, on_sample=report)
        return 0
    except (LiftError, ValueError, serial.SerialException) as exc:
        print(f"ERROR: {exc}")
        return 1
    except KeyboardInterrupt:
        return 130
    finally:
        try:
            client.close()
        except Exception:
            pass
        port.close()


if __name__ == "__main__":
    raise SystemExit(main())
