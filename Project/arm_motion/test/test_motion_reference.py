#!/usr/bin/env python3
"""Exercise the running IK node with stationary feedback, as during servo lag."""
import json, os
from pathlib import Path
import signal, subprocess, sys, tempfile, time, unittest
os.environ["ROS_DOMAIN_ID"]="92"
os.environ["ROS_LOCALHOST_ONLY"]="1"
import rclpy
from luxi_interfaces.msg import ArmCommand, ArmState, ArmTargets

BINARY, CONFIG, MODEL = [Path(sys.argv.pop(1)) for _ in range(3)]
CFG=json.loads(CONFIG.read_text())
HOME=CFG["right"]["home"]+CFG["left"]["home"]
for s,key in enumerate(["right","left"]):
    HOME[8*s+7]=min(max(HOME[8*s+7],CFG[key]["limits"][7][0]),CFG[key]["limits"][7][1])

class MotionReference(unittest.TestCase):
    def test_reference_advances_with_stationary_feedback(self):
        with tempfile.TemporaryDirectory(prefix="luxi-reference-") as tmp:
            prefix="/reference_"+str(time.monotonic_ns())
            args=[str(BINARY),"--ros-args","-p","config:="+str(CONFIG),"-p","model:="+str(MODEL)]
            for topic in ["arms/state","targets","intent"]:
                args+=["-r",f"/luxi/{topic}:={prefix}/{topic}"]
            with open(Path(tmp)/"motion.log","w+") as log:
                process=subprocess.Popen(args,stdout=log,stderr=subprocess.STDOUT,
                    env=dict(os.environ,ROS_LOG_DIR=tmp))
                node=rclpy.create_node("motion_reference_test")
                states=node.create_publisher(ArmState,prefix+"/arms/state",1)
                targets=node.create_publisher(ArmTargets,prefix+"/targets",1)
                received=[]
                node.create_subscription(ArmCommand,prefix+"/intent",
                    lambda m:received.append((time.monotonic_ns(),list(m.position))),10)
                def step(feedback=True):
                    now=time.monotonic_ns()
                    if feedback:
                        state=ArmState();state.position=HOME.copy()
                        state.valid=[True]*16;state.receive_mono_ns=now
                        states.publish(state)
                    target=ArmTargets();target.valid=[True,True]
                    target.receive_mono_ns=now
                    for side,y in [(target.right,-.25),(target.left,.25)]:
                        side.position.x=.35;side.position.y=y;side.position.z=.3
                        side.orientation.w=1.
                    target.grippers=[HOME[7],HOME[15]]
                    targets.publish(target)
                    rclpy.spin_once(node,timeout_sec=.002)
                    time.sleep(.006)
                try:
                    until=time.monotonic()+4
                    while time.monotonic()<until:step()
                    self.assertIsNone(process.poll())
                    self.assertGreater(len(received),40)
                    first,last=received[0][1],received[-1][1]
                    advance=max(abs(last[i]-first[i]) for i in range(16) if i%8!=7)
                    self.assertGreater(advance,.02,
                        f"IK repeatedly reset to measured feedback; reference advance={advance}")
                    # Fresh targets cannot keep producing commands without fresh feedback.
                    until=time.monotonic()+.3
                    while time.monotonic()<until:step(False)
                    count=len(received)
                    until=time.monotonic()+.2
                    while time.monotonic()<until:step(False)
                    self.assertEqual(len(received),count,"IK ignored feedback expiry")
                    for _,position in received:
                        for s,key in enumerate(["right","left"]):
                            for i in range(8):
                                lo,hi=CFG[key]["limits"][i]
                                self.assertGreaterEqual(position[s*8+i],lo-1e-8)
                                self.assertLessEqual(position[s*8+i],hi+1e-8)
                finally:
                    node.destroy_node()
                    if process.poll() is None:process.send_signal(signal.SIGINT)
                    try:process.wait(timeout=3)
                    except subprocess.TimeoutExpired:process.kill();process.wait()

if __name__=="__main__":
    rclpy.init()
    try:unittest.main(verbosity=2)
    finally:
        if rclpy.ok():rclpy.shutdown()
