"""Recorded-data sensor replay exclusively in ROS domain 87. No production control."""
import os
os.environ["ROS_DOMAIN_ID"] = "87"
os.environ["ROS_LOCALHOST_ONLY"] = "1"
import argparse
import json
from pathlib import Path
import signal
import subprocess
import time
import rclpy
from luxi_interfaces.msg import CameraFrame, ArmState
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fixture", type=Path, required=True)
    parser.add_argument("--bin-dir", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--duration", type=float, default=120)
    args = parser.parse_args()
    state_values = json.loads((args.fixture/"state.json").read_text())["observation.state"]
    images = {name: (args.fixture/f"{name}.jpg").read_bytes()
              for name in ("head_rgb","left_wrist_rgb","right_wrist_rgb")}
    rclpy.init()
    node = rclpy.create_node("act_recorded_sample_replay")
    qos = QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
    pubs = {name: node.create_publisher(CameraFrame,"/luxi/cameras/"+name,qos) for name in images}
    state_pub = node.create_publisher(ArmState,"/luxi/arms/state",qos)
    received = []
    sub = node.create_subscription(String,"/luxi/act/received_chunk",lambda msg: received.append(json.loads(msg.data)),10)
    children = []
    logs = []
    try:
        for binary,port in (("act_rgb_http",18042),("act_action_udp",18043)):
            log = open(args.fixture/f"{binary}.log","w")
            logs.append(log)
            children.append(subprocess.Popen([str(args.bin_dir/binary),"--ros-args","-p","host:=0.0.0.0","-p",f"port:={port}"],stdout=log,stderr=log))
        start = time.monotonic()
        sequence = 0
        frames={}
        for name,data in images.items():
            frame=CameraFrame(camera_id=name,width=640,height=360,encoding="jpeg",timestamp_quality="recorded_sample_replay")
            frame.data=data
            frames[name]=frame
        state=ArmState(position=state_values,valid=[True]*16)
        last_saved=0
        while time.monotonic()-start < args.duration:
            if any(p.poll() is not None for p in children):
                raise RuntimeError("Replay service exited unexpectedly")
            sequence += 1
            stamp = time.monotonic_ns()
            for name,frame in frames.items():
                frame.sequence=sequence;frame.receive_mono_ns=stamp;pubs[name].publish(frame)
            state.receive_mono_ns=stamp;state_pub.publish(state)
            rclpy.spin_once(node,timeout_sec=.01)
            if len(received)!=last_saved:
                args.output.write_text(json.dumps({"source":"recorded_dataset_replay_domain_87",
                                                   "received":received},indent=2))
                last_saved=len(received)
            time.sleep(.02)
    except KeyboardInterrupt:
        pass
    finally:
        for p in children:
            if p.poll() is None:p.send_signal(signal.SIGINT)
        for p in children:
            try:p.wait(timeout=5)
            except subprocess.TimeoutExpired:p.kill();p.wait()
        for log in logs:log.close()
        args.output.write_text(json.dumps({"source":"recorded_dataset_replay_domain_87","received":received},indent=2))
        node.destroy_node();rclpy.try_shutdown()
        print(f"Replay stopped. Received {len(received)} chunks; production domain untouched.")


if __name__ == "__main__":
    main()
