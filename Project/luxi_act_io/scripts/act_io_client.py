"""Standard-library client for the receive-only ACT network services."""
import argparse
import email.policy
from email.parser import BytesParser
import json
import math
from pathlib import Path
import socket
import struct
import time
import urllib.request
import uuid

HEADER = struct.Struct("!8s16sQQHHI")
VALUES = struct.Struct("!16f")


def get_observation(base_url, timeout=3):
    """Return metadata and the exact three JPEG payloads described by it."""
    with urllib.request.urlopen(base_url.rstrip("/") + "/v1/observation", timeout=timeout) as response:
        raw = b"Content-Type: " + response.headers["Content-Type"].encode() + b"\r\nMIME-Version: 1.0\r\n\r\n" + response.read()
    message = BytesParser(policy=email.policy.default).parsebytes(raw)
    if not message.is_multipart():
        raise ValueError("Expected multipart observation")
    parts = {p.get_param("name", header="Content-Disposition"): p.get_payload(decode=True)
             for p in message.iter_parts()}
    metadata = json.loads(parts.pop("metadata"))
    if set(parts) != {"head_rgb", "left_wrist_rgb", "right_wrist_rgb"}:
        raise ValueError("Incomplete camera bundle")
    return metadata, parts


class ActionSender:
    """One socket/session; one monotonic chunk ID per prediction. ACK is NOT execution."""
    def __init__(self, host, port=18041, timeout=0.1):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.connect((host, port))
        self.socket.settimeout(timeout)
        self.session = uuid.uuid4().bytes
        self.chunk_id = 0

    def close(self):
        self.socket.close()

    def send_chunk(self, positions, observation_mono_ns):
        positions = [[float(v) for v in row] for row in positions]
        if not 1 <= len(positions) <= 60:
            raise ValueError("Expected 1..60 action steps")
        if any(len(row) != 16 or not all(math.isfinite(v) for v in row) for row in positions):
            raise ValueError("Every action must contain 16 finite positions in radians")
        self.chunk_id += 1
        packets = [HEADER.pack(b"LACTv1\0\0", self.session, self.chunk_id,
                               int(observation_mono_ns), index, len(positions), 33333) + VALUES.pack(*row)
                   for index,row in enumerate(positions)]
        pending = set(range(len(packets)))
        original_timeout = self.socket.gettimeout()
        deadline = time.monotonic() + 0.4  # Below the receiver's 500 ms assembly window.
        try:
            while pending and time.monotonic() < deadline:
                # Send one small datagram per step, without waiting an RTT for each step.
                for index in sorted(pending):
                    self.socket.send(packets[index])
                retry_at = min(deadline, time.monotonic()+0.05)
                while time.monotonic() < retry_at:
                    self.socket.settimeout(max(.001,retry_at-time.monotonic()))
                    try:
                        ack = json.loads(self.socket.recv(2048))
                    except socket.timeout:
                        break
                    if ack.get("session_id") != self.session.hex() or ack.get("chunk_id") != str(self.chunk_id):
                        continue
                    index = ack.get("step_index")
                    if index not in range(len(packets)):
                        continue
                    if ack.get("status") == "rejected":
                        raise RuntimeError(f"Robot rejected chunk: {ack.get('reason')}")
                    if ack.get("status") in ("accepted","duplicate") and ack.get("received_steps") == len(packets):
                        return ack
                    if ack.get("status") == "partial":
                        pending.discard(index)
            raise TimeoutError("Complete UDP acknowledgment missing; receipt is unconfirmed")
        finally:
            self.socket.settimeout(original_timeout)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot", default="192.168.2.199")
    parser.add_argument("--http-port", type=int, default=18040)
    parser.add_argument("--udp-port", type=int, default=18041)
    parser.add_argument("--save-dir", type=Path, help="Save exact JPEGs and metadata")
    parser.add_argument("--transport-test", action="store_true",
                        help="Send one current-state sample to RECEIVE-ONLY ingress; no motor execution")
    args = parser.parse_args()
    meta, images = get_observation(f"http://{args.robot}:{args.http_port}")
    if args.save_dir:
        args.save_dir.mkdir(parents=True, exist_ok=True)
        for name, data in images.items():
            (args.save_dir / f"{name}.jpg").write_bytes(data)
        (args.save_dir / "metadata.json").write_text(json.dumps(meta, indent=2))
    print(json.dumps(meta, ensure_ascii=False, indent=2))
    if args.transport_test:
        sender = ActionSender(args.robot, args.udp_port)
        try:
            print(json.dumps(sender.send_chunk([meta["state"]["position"]], meta["observation_mono_ns"])))
        finally:
            sender.close()


if __name__ == "__main__":
    main()
