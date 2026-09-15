#!/usr/bin/env python3
"""Check transient tracking loss and sustained-loss latching without actuators."""
import json, os
from pathlib import Path
import signal, subprocess, sys, tempfile, time, unittest
os.environ["ROS_DOMAIN_ID"]="93"
os.environ["ROS_LOCALHOST_ONLY"]="1"
import rclpy
from luxi_interfaces.msg import ArmCommand, ArmState, QuestInput
from luxi_interfaces.srv import Command
from std_msgs.msg import String
BINARY,CONFIG=[Path(sys.argv.pop(1)) for _ in range(2)]
CFG=json.loads(CONFIG.read_text())
HOME=CFG["right"]["home"]+CFG["left"]["home"]
for s,key in enumerate(["right","left"]):
    HOME[8*s+7]=min(max(HOME[8*s+7],CFG[key]["limits"][7][0]),CFG[key]["limits"][7][1])

class TrackingGap(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory(prefix="luxi-gap-")
        self.log=open(Path(self.tmp.name)/"manager.log","w+")
        prefix="/gap_"+str(time.monotonic_ns())
        args=[str(BINARY),"--ros-args","-p","config:="+str(CONFIG)]
        for topic in ["quest","arms/state","intent","control","control_status","command"]:
            args+=["-r",f"/luxi/{topic}:={prefix}/{topic}"]
        self.process=subprocess.Popen(args,stdout=self.log,stderr=subprocess.STDOUT,
            env=dict(os.environ,ROS_LOG_DIR=self.tmp.name))
        self.node=rclpy.create_node("tracking_gap_test")
        self.status={};self.commands=[]
        self.node.create_subscription(String,prefix+"/control_status",
            lambda m:setattr(self,"status",json.loads(m.data)),1)
        self.node.create_subscription(ArmCommand,prefix+"/command",
            lambda m:self.commands.append(list(m.position)),1)
        self.qp=self.node.create_publisher(QuestInput,prefix+"/quest",100)
        self.sp=self.node.create_publisher(ArmState,prefix+"/arms/state",1)
        self.ip=self.node.create_publisher(ArmCommand,prefix+"/intent",1)
        self.client=self.node.create_client(Command,prefix+"/control")
        self.flag=1 # Ordinary STALE is permitted, including during startup.
        self.quest_on=True;self.intent_on=True;self.intent_offset=0.
        self.intent_phase='ik_unconverged' if self._testMethodName=='test_alignment_waits_for_ik_solution' else 'teleop'
        self.run_for(1.5)
        end=time.monotonic()+5;future=None;accepted=False
        while time.monotonic()<end and not accepted:
            if self.client.service_is_ready() and future is None:
                request=Command.Request();request.command="start"
                future=self.client.call_async(request)
            self.step()
            if future is not None and future.done():
                accepted=future.result().success;future=None
        self.assertTrue(accepted)
        end=time.monotonic()+4
        expected='aligning' if self.intent_phase=='ik_unconverged' else 'teleop'
        while self.status.get("phase")!=expected and time.monotonic()<end:self.step()
        self.assertEqual(self.status.get("phase"),expected)

    def step(self):
        now=time.monotonic_ns()
        state=ArmState();state.position=HOME.copy();state.valid=[True]*16
        state.receive_mono_ns=now;self.sp.publish(state)
        if self.quest_on:
            quest=QuestInput();quest.receive_mono_ns=now
            quest.tracking=self.flag;quest.left_tracking=self.flag;quest.right_tracking=self.flag
            quest.triggers=[1.,1.]
            self.qp.publish(quest)
        if self.intent_on:
            intent=ArmCommand();intent.source_mono_ns=now;intent.position=HOME.copy()
            intent.phase=self.intent_phase
            intent.position[0]+=self.intent_offset;self.ip.publish(intent)
        rclpy.spin_once(self.node,timeout_sec=.001)
        time.sleep(.004)

    def run_for(self,seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:self.step()

    def tearDown(self):
        self.node.destroy_node()
        if self.process.poll() is None:self.process.send_signal(signal.SIGINT)
        try:self.process.wait(timeout=3)
        except subprocess.TimeoutExpired:self.process.kill();self.process.wait()
        self.log.close();self.tmp.cleanup()

    def test_brief_invalid_tracking_freezes_without_latching(self):
        self.flag=2;self.run_for(.025)
        before=self.commands[-1]
        self.intent_offset=.2;self.run_for(.055)
        self.assertLess(max(abs(a-b) for a,b in zip(before,self.commands[-1])),1e-8,
            "invalid tracking moved the target")
        self.flag=1;self.run_for(.2)
        self.assertEqual(self.status.get("phase"),"teleop",
            "an 80 ms tracking interruption permanently latched hold")
        self.assertGreater(self.commands[-1][0]-before[0],.01)

    def test_sustained_invalid_tracking_latches_hold(self):
        self.flag=2;self.run_for(.3)
        self.assertEqual(self.status.get("phase"),"hold")
        self.flag=1;self.run_for(.2)
        self.assertEqual(self.status.get("phase"),"hold")

    def test_recovery_waits_for_new_ik_before_moving(self):
        self.flag=2;self.run_for(.025)
        before=self.commands[-1]
        self.intent_offset=.2;self.run_for(.04)
        self.intent_on=False;self.flag=1;self.run_for(.05)
        self.assertEqual(self.status.get("phase"),"teleop")
        self.assertTrue(self.status.get("input_paused"))
        self.assertLess(max(abs(a-b) for a,b in zip(before,self.commands[-1])),1e-8)
        self.intent_on=True;self.run_for(.2)
        self.assertEqual(self.status.get("phase"),"teleop")
        self.assertGreater(self.commands[-1][0]-before[0],.01)

    def test_packet_loss_still_latches_hold(self):
        self.quest_on=False;self.run_for(.6)
        self.assertEqual(self.status.get("phase"),"hold")
        self.quest_on=True;self.run_for(.2)
        self.assertEqual(self.status.get("phase"),"hold")

    def test_network_burst_gap_freezes_then_waits_for_recovered_ik(self):
        self.quest_on=False;self.intent_on=False
        self.run_for(.19)
        before=self.commands[-1]
        self.intent_offset=.2;self.run_for(.04)
        self.assertEqual(self.status.get("phase"),"teleop",
            "a measured 230 ms packet burst gap permanently latched hold")
        self.assertTrue(self.status.get("input_paused"))
        self.quest_on=True;self.run_for(.035)
        self.assertEqual(self.status.get("phase"),"teleop")
        self.assertTrue(self.status.get("input_paused"))
        self.assertLess(max(abs(a-b) for a,b in zip(before,self.commands[-1])),1e-8)
        self.intent_on=True;self.run_for(.15)
        self.assertEqual(self.status.get("phase"),"teleop")
        self.assertFalse(self.status.get("input_paused"))
        self.assertGreater(self.commands[-1][0]-before[0],.01)

    def test_brief_ik_gap_recovers_but_sustained_gap_latches(self):
        self.intent_on=False;self.run_for(.23)
        self.assertEqual(self.status.get("phase"),"teleop")
        self.assertTrue(self.status.get("input_paused"))
        self.intent_on=True;self.run_for(.15)
        self.assertFalse(self.status.get("input_paused"))
        self.intent_on=False;self.run_for(.6)
        self.assertEqual(self.status.get("phase"),"hold")
        self.assertEqual(self.status.get("reason"),"ik_timeout")
        self.intent_on=True;self.run_for(.15)
        self.assertEqual(self.status.get("phase"),"hold")

    def test_alignment_waits_for_ik_solution(self):
        self.run_for(.1);before=self.commands[-1]
        self.intent_offset=.3;self.run_for(.25)
        self.assertEqual(self.status.get('phase'),'aligning')
        self.assertEqual(self.status.get('reason'),'waiting_for_ik_solution')
        self.assertTrue(self.status.get('input_paused'))
        self.assertLess(max(abs(a-b) for a,b in zip(before,self.commands[-1])),1e-8,
            'Alignment followed an unconverged IK target')
        self.intent_phase='teleop';self.run_for(.25)
        self.assertGreater(self.commands[-1][0]-before[0],.01)

if __name__=="__main__":
    rclpy.init()
    try:unittest.main(verbosity=2)
    finally:
        if rclpy.ok():rclpy.shutdown()
