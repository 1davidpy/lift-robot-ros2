#!/usr/bin/env python3
"""Isolated transport test: synthetic files, no ROS publishers or hardware."""
import hashlib
import http.server
import json
import pathlib
import signal
import socket
import subprocess
import sys
import tempfile
import threading
import time

receiver, pull = sys.argv[1:3]
with tempfile.TemporaryDirectory(prefix='luxi-transfer-test-') as temporary:
    destination = pathlib.Path(temporary) / 'receiver'
    destination.mkdir()
    episode = 'transfer-test'
    payload = bytes(range(256)) * 8192
    manifest = {'episode_id': episode, 'status': 'transport_test_only', 'files': [
        {'path': 'bag/data.bin', 'bytes': len(payload), 'sha256': hashlib.sha256(payload).hexdigest()},
        {'path': 'empty.txt', 'bytes': 0, 'sha256': hashlib.sha256(b'').hexdigest()}]}
    body = (json.dumps(manifest, indent=2) + '\n').encode()
    body_hash = hashlib.sha256(body).hexdigest()
    ranges, receipts = [], []
    pending = destination / ('.partial-' + episode) / 'bag'
    pending.mkdir(parents=True)
    (pending / 'data.bin').write_bytes(payload[:262144])

    class Handler(http.server.BaseHTTPRequestHandler):
        protocol_version = 'HTTP/1.1'
        def log_message(self, *args):
            pass
        def send(self, code, content):
            self.send_response(code)
            self.send_header('Content-Length', str(len(content)))
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            self.wfile.write(content)
        def do_POST(self):
            request = json.loads(self.rfile.read(int(self.headers['Content-Length'])))
            if request['action'] == 'verified':
                assert request['manifest_sha256'] == body_hash
                assert (destination / episode / 'bag/data.bin').read_bytes() == payload
                assert (destination / episode / 'manifest.json').read_bytes() == body
                receipts.append(request)
            self.send(200, b'{"success":true}')
        def do_GET(self):
            if self.path == '/api/episodes':
                self.send(200, json.dumps([] if receipts else [episode]).encode())
            elif self.path == '/api/episodes/transfer-test/manifest':
                self.send(200, body)
            elif self.path == '/api/episodes/unsafe/manifest':
                bad = {'episode_id': 'unsafe', 'files': [{'path': '../escaped', 'bytes': 1, 'sha256': 'a' * 64}]}
                self.send(200, json.dumps(bad).encode())
            elif self.path.endswith('/empty.txt'):
                self.send(200, b'')
            elif self.path.endswith('/bag/data.bin'):
                offset = int(self.headers.get('Range', 'bytes=0-')[6:-1])
                ranges.append(offset)
                content = payload[offset:]
                if len(ranges) == 1:
                    content = bytes([content[0] ^ 1]) + content[1:]  # checksum failure
                self.send_response(206 if offset else 200)
                self.send_header('Content-Length', str(len(content)))
                if offset:
                    self.send_header('Content-Range', f'bytes {offset}-{len(payload)-1}/{len(payload)}')
                self.end_headers()
                if len(ranges) == 2:  # interrupted transfer, retain half for resume
                    self.wfile.write(content[:1048576]); self.wfile.flush()
                    self.connection.shutdown(socket.SHUT_RDWR)
                    self.close_connection = True
                else:
                    self.wfile.write(content)
            else:
                self.send(404, b'{}')

    server = http.server.ThreadingHTTPServer(('127.0.0.1', 0), Handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    url = f'http://127.0.0.1:{server.server_port}'
    log = pathlib.Path(temporary) / 'receiver.log'
    with log.open('w') as output:
        process = subprocess.Popen([receiver, url, str(destination), '--release-source'], stdout=output, stderr=subprocess.STDOUT)
        try:
            deadline = time.monotonic() + 35
            while not receipts and time.monotonic() < deadline and process.poll() is None:
                time.sleep(.1)
            assert receipts, log.read_text()
            assert ranges[:3] == [262144, 0, 1048576], ranges
            assert receipts[0]['release_source'] is True
            assert (destination / episode / 'empty.txt').stat().st_size == 0
            assert not (destination / ('.partial-' + episode)).exists()
            # Re-pulling the same committed episode is idempotent.
            result = subprocess.run([pull, url, episode, str(destination)], capture_output=True, text=True, timeout=10)
            assert result.returncode == 0, result.stderr
            # An unsafe manifest is rejected before any write outside its directory.
            result = subprocess.run([pull, url, 'unsafe', str(destination)], capture_output=True, text=True, timeout=10)
            assert result.returncode != 0 and 'unsafe manifest path' in result.stderr, result
            assert not (destination / 'escaped').exists()
        finally:
            process.send_signal(signal.SIGTERM)
            try:
                process.wait(timeout=12)
            except subprocess.TimeoutExpired:
                process.kill(); process.wait()
            server.shutdown(); server.server_close()
    print(json.dumps({'passed': True, 'resume_offsets': ranges[:3], 'verified_receipts': len(receipts),
                      'checked': ['corruption retry', 'disconnect resume', 'empty files', 'atomic commit',
                                  'idempotent pull', 'path traversal rejection', 'acknowledge after verification']}))
