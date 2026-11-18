#!/usr/bin/env python3
"""Exercise homing arrival with measured static load error in an isolated ROS domain."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import unittest
os.environ["ROS_DOMAIN_ID"]="91"
os.environ["ROS_LOCALHOST_ONLY"]="1"
import rclpy
from luxi_interfaces.msg import ArmCommand, ArmState, QuestInput
from luxi_interfaces.srv import Command
from std_msgs.msg import String
BINARY=Path(sys.argv.pop(1))
CONFIG=Path(sys.argv.pop(1))
CFG=json.loads(CONFIG.read_text())
HOME=CFG["right"]["home"]+CFG["left"]["home"]

class HomingTolerance(unittest.TestCase):
    def run_case(self,tolerance,error,expected):
        with tempfile.TemporaryDirectory(prefix="luxi-homing-") as tmp:
            env=dict(os.environ,ROS_LOG_DIR=tmp)
            args=[str(BINARY),"--ros-args","-p","config:="+str(CONFIG)]
            prefix="/homing_case_"+str(time.monotonic_ns())
            topics=["control_status","quest","arms/state","intent","control"]
            for topic in topics:
                args += ["-r",f"/luxi/{topic}:={prefix}/{topic}"]
            if tolerance is not None:
                args+=["-p","home_tolerance:="+str(tolerance)]
            with open(Path(tmp)/"manager.log","w+") as log:
                process=subprocess.Popen(args,env=env,stdout=log,stderr=subprocess.STDOUT)
                node=rclpy.create_node("homing_tolerance_test")
                phases=[]
                node.create_subscription(String,prefix+"/control_status",
                    lambda m:phases.append(json.loads(m.data)["phase"]),1)
                qp=node.create_publisher(QuestInput,prefix+"/quest",100)
                sp=node.create_publisher(ArmState,prefix+"/arms/state",1)
                ip=node.create_publisher(ArmCommand,prefix+"/intent",1)
                client=node.create_client(Command,prefix+"/control")
                def step():
                    now=time.monotonic_ns()
                    state=ArmState()
                    state.receive_mono_ns=now
                    state.position=HOME.copy()
                    # Both fourth joints settled about 0.09 rad short on real hardware.
                    state.position[3]-=error
                    state.position[11]-=error
                    state.valid=[True]*16
                    for side,key in enumerate(["right","left"]):
                        state.position[side*8+7]=min(max(state.position[side*8+7],
                            CFG[key]["limits"][7][0]),CFG[key]["limits"][7][1])
                    sp.publish(state)
                    quest=QuestInput()
                    quest.receive_mono_ns=now
                    qp.publish(quest)
                    intent=ArmCommand()
                    intent.source_mono_ns=now
                    intent.position=HOME.copy()
                    intent.position[3]-=.4
                    ip.publish(intent)
                    rclpy.spin_once(node,timeout_sec=.003)
                    time.sleep(.007)
                try:
                    until=time.monotonic()+1.5
                    while time.monotonic()<until:step()
                    self.assertIsNone(process.poll(),"manager failed to start")
                    self.assertTrue(client.service_is_ready())
                    # DDS discovery and the 50-packets/second gate must both be ready.
                    accepted=False
                    until=time.monotonic()+5
                    future=None
                    while not accepted and time.monotonic()<until:
                        if future is None:
                            req=Command.Request();req.command="start"
                            future=client.call_async(req)
                        step()
                        if future.done():
                            accepted=future.result().success
                            future=None
                    self.assertTrue(accepted,"fresh mock inputs must permit start")
                    until=time.monotonic()+2.6
                    while time.monotonic()<until:step()
                    self.assertEqual(phases[-1],expected,
                        f"error={error}, tolerance={tolerance}, phases={sorted(set(phases))}")
                    if expected=="aligning":
                        req=Command.Request();req.command="stop"
                        future=client.call_async(req)
                        # The legacy sequence sends 100 distinct 50 Hz steps;
                        # scheduling delays extend its wall time rather than
                        # skipping commands. Wait for completion, not 2.6 s.
                        until=time.monotonic()+5
                        while time.monotonic()<until:
                            step()
                            if future.done() and future.result().success and phases[-1]=="disabled":break
                        self.assertTrue(future.done())
                        self.assertTrue(future.result().success)
                        self.assertEqual(phases[-1],"disabled",
                            "return must complete with the same configured tolerance")
                finally:
                    node.destroy_node()
                    if process.poll() is None:process.send_signal(signal.SIGINT)
                    try:process.wait(timeout=3)
                    except subprocess.TimeoutExpired:process.kill();process.wait()

    def test_configured_tolerance_allows_observed_load_error(self):
        self.run_case(.1,.09,"aligning")

    def test_default_tolerance_remains_strict(self):
        self.run_case(None,.09,"homing")

    def test_error_outside_configured_tolerance_still_blocks(self):
        self.run_case(.1,.12,"homing")

    def test_invalid_tolerance_rejected(self):
        for value in ["0.0","-0.1","0.2",".nan"]:
            with self.subTest(value=value),tempfile.TemporaryDirectory() as tmp:
                result=subprocess.run([str(BINARY),"--ros-args","-p","config:="+str(CONFIG),
                    "-p","home_tolerance:="+value],env=dict(os.environ,ROS_LOG_DIR=tmp),
                    stdout=subprocess.PIPE,stderr=subprocess.STDOUT,timeout=3,text=True)
                self.assertNotEqual(result.returncode,0)
                self.assertIn("home_tolerance",result.stdout)

if __name__=="__main__":
    rclpy.init()
    try:unittest.main(verbosity=2)
    finally:
        if rclpy.ok():rclpy.shutdown()
