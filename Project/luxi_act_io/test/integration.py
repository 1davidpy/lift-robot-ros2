"""Isolated ROS domain, fake sensors, loopback ingress; never sends motor commands."""
import os
os.environ["ROS_DOMAIN_ID"] = "87"
os.environ["ROS_LOCALHOST_ONLY"] = "1"
import json
import signal
import select
import socket
import struct
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.request
import uuid
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from act_io_client import get_observation, ActionSender, HEADER, VALUES
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from luxi_interfaces.msg import CameraFrame, ArmState
from std_msgs.msg import String

def check(value, message):
    if not value:
        raise AssertionError(message)

def available_port(kind):
    with socket.socket(socket.AF_INET, kind) as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]

http_port = available_port(socket.SOCK_STREAM)
udp_port = available_port(socket.SOCK_DGRAM)
children = []
logs = []
rclpy.init()
node = rclpy.create_node("act_io_test")
received = []
sub = node.create_subscription(String, "/luxi/act/received_chunk", lambda m: received.append(json.loads(m.data)), 10)
qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
ids = ("head_rgb", "left_wrist_rgb", "right_wrist_rgb")
pubs = {name: node.create_publisher(CameraFrame, "/luxi/cameras/" + name, qos) for name in ids}
state_pub = node.create_publisher(ArmState, "/luxi/arms/state", qos)
running = True
publishing = True
skew = False
sequence = 0

def worker():
    global sequence
    while running:
        if publishing:
            sequence += 1
            now = time.monotonic_ns()
            for index, (name, pub) in enumerate(pubs.items()):
                f = CameraFrame(camera_id=name, sequence=sequence, receive_mono_ns=now-(150_000_000 if skew and index else 0),
                                width=640, height=360, encoding="jpeg", timestamp_quality="synthetic_test")
                # The ingress preserves opaque JPEG bytes; physical image decoding is tested separately.
                f.data = bytes([255,216,1+index,255,217])
                pub.publish(f)
            state = ArmState(receive_mono_ns=now, position=[0.0]*16, valid=[True]*16)
            state_pub.publish(state)
        rclpy.spin_once(node, timeout_sec=0.01)
        time.sleep(0.01)

thread = threading.Thread(target=worker)
thread.start()
try:
    for binary, port in zip(sys.argv[1:3], (http_port, udp_port)):
        log = open(f"/tmp/{Path(binary).name}-integration.log", "w+")
        logs.append(log)
        children.append(subprocess.Popen([binary, "--ros-args", "-p", "host:=127.0.0.1", "-p", f"port:={port}"], stdout=log, stderr=log))
    base = f"http://127.0.0.1:{http_port}"
    deadline = time.monotonic()+12
    while True:
        try:
            meta, images = get_observation(base)
            break
        except (OSError, urllib.error.HTTPError):
            if time.monotonic()>deadline: raise
            time.sleep(0.1)
    check(len(images)==3 and len(meta["state"]["position"])==16, "observation bundle")
    with urllib.request.urlopen(base+"/v1/cameras/head_rgb/rgb") as response:
        check(response.read()==bytes([255,216,1,255,217]), "exact RGB payload")
    # Discover the ingress subscriber before testing a volatile (non-latched) publication.
    deadline = time.monotonic()+5
    while node.count_publishers("/luxi/act/received_chunk") == 0:
        check(time.monotonic()<deadline,"ingress publisher discovery")
        time.sleep(.05)
    sender = ActionSender("127.0.0.1", udp_port)
    stamp = time.monotonic_ns()
    ack = sender.send_chunk([[0.01]*16 for _ in range(60)], stamp)
    check(ack["status"]=="accepted" and ack["execution"]=="not_confirmed", "60-step ACK")
    sender.close()
    time.sleep(.15)
    check(len(received)==1 and len(received[0]["positions"])==60, "exactly one full ROS chunk")
    # Network impairment: lose a data packet AND the first complete-block ACK.
    # The client must selectively retry; the receiver must publish exactly once.
    front=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);front.bind(("127.0.0.1",0))
    back=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);back.connect(("127.0.0.1",udp_port))
    proxy_running=True
    dropped={"data":False,"ack":False}
    def proxy():
        peer=None
        while proxy_running:
            readable,_,_=select.select([front,back],[],[],.01)
            for transport in readable:
                if transport is front:
                    data,peer=front.recvfrom(2048)
                    index=HEADER.unpack(data[:48])[4]
                    if index==5 and not dropped["data"]:
                        dropped["data"]=True;continue
                    back.send(data)
                else:
                    data=back.recv(2048)
                    if json.loads(data).get("status")=="accepted" and not dropped["ack"]:
                        dropped["ack"]=True;continue
                    front.sendto(data,peer)
    relay=threading.Thread(target=proxy);relay.start()
    impaired=ActionSender("127.0.0.1",front.getsockname()[1])
    try:
        result=impaired.send_chunk([[0.02]*16 for _ in range(60)],time.monotonic_ns())
        check(result["status"]=="duplicate" and all(dropped.values()),"lost packet and ACK recovery")
    finally:
        impaired.close();proxy_running=False;relay.join(2);front.close();back.close()
    time.sleep(.1)
    check(len(received)==2,"retry publishes once")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.connect(("127.0.0.1",udp_port));sock.settimeout(1)
    session=uuid.uuid4().bytes
    def packet(chunk, idx=0, count=1, stamp=None, values=None):
        return HEADER.pack(b"LACTv1\0\0",session,chunk,stamp if stamp is not None else time.monotonic_ns(),idx,count,33333)+VALUES.pack(*(values if values is not None else [0.0]*16))
    def send(data):
        sock.send(data);return json.loads(sock.recv(2048))
    def reject(data, reason):
        a=send(data);check(a.get("status")=="rejected" and a.get("reason")==reason,str(a))
    reject(b"invalid", "bad_size_or_magic")
    reject(b"x"*200, "bad_size_or_magic")
    reject(packet(1,stamp=time.monotonic_ns()-3_000_000_000),"observation_stale_or_future")
    reject(packet(1,stamp=time.monotonic_ns()+1_000_000_000),"observation_stale_or_future")
    reject(packet(1,values=[float("nan")]*16),"nonfinite_position")
    stamp=time.monotonic_ns()
    p1=packet(2,1,2,stamp);p0=packet(2,0,2,stamp)
    check(send(p1)["status"]=="partial","out-of-order part")
    check(send(p1)["status"]=="partial","duplicate partial")
    reject(packet(2,1,2,stamp,[0.1]*16),"conflicting_retransmission")
    check(send(p0)["status"]=="accepted","reordered full block")
    check(send(p0)["status"]=="duplicate","completed retransmission")
    reject(packet(1),"old_chunk")
    incomplete=packet(3,0,2)
    check(send(incomplete)["status"]=="partial","incomplete block")
    time.sleep(.55)
    reject(incomplete,"assembly_timeout")
    time.sleep(.1)
    check(len(received)==3,"no duplicate or incomplete ROS publication")
    sock.close()
    # No production drive topic/service is exposed by either new node.
    for name in ("act_rgb_http", "act_action_udp"):
        published={topic for topic,_ in node.get_publisher_names_and_types_by_node(name,"/")}
        check(not published.intersection({"/luxi/command","/luxi/applied"}),"no motor publisher")
    skew=True;time.sleep(.2)
    try: get_observation(base);raise AssertionError("excess skew accepted")
    except urllib.error.HTTPError as e: check(e.code==503,"skew 503")
    skew=False;time.sleep(.1)
    publishing=False;time.sleep(.35)
    for path in ("/v1/observation", "/v1/cameras/head_rgb/rgb"):
        try: urllib.request.urlopen(base+path);raise AssertionError("stale image accepted")
        except urllib.error.HTTPError as e: check(e.code==503,"stale 503")
    check(all(p.poll() is None for p in children),"services remained alive")
    print("PASS: HTTP bundle/JPEG/stale/skew; UDP 60 steps/reorder/retry/conflict/NaN/age/timeout; isolated ROS publication; no motor publisher")
finally:
    running=False;thread.join(3)
    for p in children:
        p.send_signal(signal.SIGINT)
    for p in children:
        try:p.wait(timeout=5)
        except subprocess.TimeoutExpired:p.kill();p.wait()
    node.destroy_node();rclpy.shutdown()
    for log in logs:
        log.seek(0);print(log.read()[-2500:]);log.close()
