"""Actual adapter + UDP ingress, mock motors, isolated ROS domain; no CAN."""
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import tempfile
import time
import uuid
from test_openarm_manual_pose import Harness
import rclpy
from luxi_interfaces.srv import Command
from std_msgs.msg import String
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/"luxi_act_io/scripts"))
from act_io_client import ActionSender

Q=[.246624,-.086404,-.032235,1.724079,.021553,.531586,.052071,-.793469,
   -.113870,-.006294,.007820,1.639391,-.062753,-.311856,-.161555,-.826658]

class ActHarness(Harness):
    def __init__(self,*args):
        self.grace=False;self.grace_started=0;self.lease=False;self.session="";self.last_hb=0;self.hb=None
        self.lift_um=-64543;self.lift_speed=0;self.lift_age=0;self.lift_flags=51;self.base_moving=False;self.publish_base=False;self.source="console";self.history=[]
        super().__init__(*args)
        self.control_pub=self.node.create_publisher(String,"/luxi/control_status",1)
        self.lift_pub=self.node.create_publisher(String,"/luxi/lift/state",1)
        self.lift_status_pub=self.node.create_publisher(String,"/luxi/lift/status",1)
        self.base_pub=self.node.create_publisher(String,"/luxi/base/state",1)
        self.chunk_pub=self.node.create_publisher(String,"/luxi/act/received_chunk",1)

    @property
    def act(self):return self.status.get("act",{}) if self.status else {}

    def on_applied(self,msg):
        super().on_applied(msg);self.history.append(msg)

    def pump(self,seconds,action=None):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            assert self.process.poll() is None,self.log_text()
            stamp=time.monotonic_ns()
            self.control_pub.publish(String(data=json.dumps({"control_source":self.source,"phase":"disabled","system_phase":"disabled","shared_enable":True})))
            self.lift_pub.publish(String(data=json.dumps({"valid":self.lift_age<=250,"current_um":self.lift_um,"speed_rpm":self.lift_speed,"fault":0,"receive_mono_ns":str(stamp),"receive_first_mono_ns":str(stamp),"feedback_age_ms":self.lift_age,"flags":self.lift_flags,"guard":0,"transport":"uart"})))
            self.lift_status_pub.publish(String(data=json.dumps({"receive_mono_ns":str(stamp),"startup_feedback_grace":self.grace,
                "enabled":True,"external_owner":"act-"+self.session,"phase":"move_ack",
                "motion_started_mono_ns":str(self.grace_started),"startup_origin_um":self.lift_um})))
            if self.publish_base:self.base_pub.publish(String(data=json.dumps({"valid":True,"velocity":[.1 if self.base_moving else 0.,0.,0.],"receive_mono_ns":str(stamp)})))
            if self.lease and time.monotonic()-self.last_hb>.1 and (self.hb is None or self.hb.done()):
                req=Command.Request(command="act_heartbeat",detail=json.dumps({"session_id":self.session}))
                self.hb=self.client.call_async(req);self.last_hb=time.monotonic()
            if action:action()
            rclpy.spin_once(self.node,timeout_sec=.002)
            for _ in range(5):rclpy.spin_once(self.node,timeout_sec=0)
            time.sleep(.003)

    def call(self,command,expected=True,session=None,detail=None):
        req=Command.Request(command=command,detail=json.dumps(detail or {"session_id":session or self.session}))
        future=self.client.call_async(req);self.until(future.done)
        result=future.result();assert result.success==expected,(command,result,self.status)
        return result

    def begin(self):
        self.session=uuid.uuid4().hex;self.lease=True
        self.call("act_prepare")
        self.call("act_begin",False)
        self.until(lambda:self.act.get("stage")=="ready",15)
        assert max(abs(a-b) for a,b in zip(self.state.position,Q))<.03
        self.call("act_begin")
        self.until(lambda:self.act.get("stage")=="running")

    def packet(self,seq=1,stamp=None,session=None,positions=None):
        return {"schema":"luxi_act_action_chunk_v1","session_id":session or self.session,
          "chunk_id":str(seq),"observation_mono_ns":str(stamp or time.monotonic_ns()),
          "units":"rad","semantics":"post_compensation_position_target",
          "joint_order":"right.J1..J7,right.gripper,left.J1..J7,left.gripper",
          "step_interval_us":33333,"positions":positions or [Q]*60}

def main():
    binary,config,udp_binary=map(Path,sys.argv[1:])
    rclpy.init()
    with tempfile.TemporaryDirectory(prefix="act-execution-test-") as temp:
        initial=Q.copy();initial[0]+=.03;initial[7]+=.02;initial[15]+=.02
        load=[.02]+[0.]*15
        with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as s:
            s.bind(("127.0.0.1",0));stream_port=s.getsockname()[1]
        h=ActHarness(binary,config,initial,temp,"act",["-p","mock_static_load_rad:="+json.dumps(load),"-p",f"joint_stream_port:={stream_port}"])
        udp=None;udp_log=None
        try:
            h.startup_ready();h.pump(.1)
            assert not h.state.enabled and h.act["stage"]=="idle"
            diagnostics=h.act["feedback_diagnostics"]
            assert diagnostics["environment_rejection"]=="",diagnostics
            assert 0<=diagnostics["lift_source_age_ms"]<300,diagnostics
            assert 0<=diagnostics["lift_effective_age_ms"]<250,diagnostics
            h.session=uuid.uuid4().hex;h.lift_um=0;h.pump(.1)
            h.lease=True;h.call("act_prepare")
            h.until(lambda:h.act.get("stage")=="ready",15)
            h.call("act_begin",False)  # Arms ready, but lift has not arrived.
            h.grace=True;h.grace_started=time.monotonic_ns();h.lift_age=600;h.pump(.2)
            assert h.act["stage"]=="ready",h.act
            h.grace=False;h.lift_age=0
            h.lift_speed=10;h.pump(.3)
            assert h.act["stage"]=="ready"  # Keep arm support while lift moves.
            h.call("act_begin",False)
            h.lift_um=-64543;h.lift_speed=0;h.lift_age=300;h.pump(.1)
            assert "measurement_stale" in h.call("act_begin",False).message
            h.lift_age=0;h.pump(.1)
            h.call("act_begin");h.until(lambda:h.act.get("stage")=="running")
            h.call("act_prepare",False,session=uuid.uuid4().hex)
            h.call("joint_move",False,detail={"request_id":"intruder","positions":Q,"speed_rad_s":.1})
            h.call("act_stop",False,session=uuid.uuid4().hex)
            print("PASS: explicit homing, initial-pose gate, lift gate, owner exclusion; no boot enable")
            sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);sock.bind(("127.0.0.1",0));port=sock.getsockname()[1];sock.close()
            udp_log=open(Path(temp)/"udp.log","w+")
            udp=subprocess.Popen([str(udp_binary),"--ros-args","-p","host:=127.0.0.1","-p",f"port:={port}"],stdout=udp_log,stderr=subprocess.STDOUT)
            # Renew task lease while waiting for UDP node discovery; first-action
            # deadline is intentionally tested below, so re-arm a fresh session.
            h.pump(1.2)
            assert h.act["stage"]=="stopped" and h.act["reason"]=="act_action_timeout"
            assert h.state.enabled
            h.begin()
            sender=ActionSender("127.0.0.1",port);sender.session=bytes.fromhex(h.session)
            rows=[]
            for step in range(60):
                q=Q.copy();q[7]+=.001*(step+1);q[15]+=.001*(step+1);rows.append(q)
            before=list(h.state.position)
            ack=sender.send_chunk(rows,time.monotonic_ns());sender.close()
            assert ack["status"]=="accepted" and ack["execution"]=="not_confirmed"
            h.until(lambda:h.act.get("chunk_id")=="1")
            h.pump(2.1)
            assert h.act["stage"]=="running" and h.act["target_step"]==60,h.act
            assert h.state.position[7]>before[7]+.04 and h.state.position[15]>before[15]+.04
            assert h.act["raw_post_compensation"] and all(abs(x)<1e-12 for x in h.status["static_compensation_rad"])
            hold=list(h.status["support_target"]);h.pump(.15)
            assert max(abs(a-b) for a,b in zip(hold,h.status["support_target"]))<1e-10
            h.call("act_stop");h.pump(.15)
            assert h.state.enabled and h.act["stage"]=="stopped" and h.act["raw_post_compensation"]
            print("PASS: real UDP -> ROS -> mock CAN path, both grippers, all 60 steps, raw compensation, stop holds support")
            h.begin()
            h.chunk_pub.publish(String(data=json.dumps(h.packet(session=uuid.uuid4().hex))))
            h.pump(.1);assert h.act["chunk_id"]=="0"
            h.chunk_pub.publish(String(data=json.dumps(h.packet(stamp=time.monotonic_ns()-2_000_000_000))))
            h.until(lambda:h.act["stage"]=="fault")
            assert "stale_or_prestart" in h.act["reason"]
            h.begin()
            bad=[Q.copy() for _ in range(60)];bad[0][0]=99.
            h.chunk_pub.publish(String(data=json.dumps(h.packet(positions=bad))))
            h.until(lambda:h.act["stage"]=="fault")
            assert h.act["reason"]=="act_action_outside_joint_limits"
            h.begin();h.lease=False;h.pump(1.2)
            assert h.act["stage"]=="stopped";h.call("act_heartbeat",False)
            h.begin();h.publish_base=True;h.base_moving=True;h.pump(.15)
            assert h.act["stage"]=="running"
            h.call("act_stop")
            print("PASS: wrong-session ignore, stale/limit rejection, lease loss, base feedback absent/moving ignored, no automatic restart")
            h.base_moving=False;h.pump(.1);h.begin()
            h.lift_age=300;h.lift_speed=2;h.pump(.2)
            assert h.act["stage"]=="running",h.act
            h.lift_speed=-2;h.pump(.1)
            assert h.act["stage"]=="running",h.act
            h.grace=True;h.grace_started=time.monotonic_ns();h.lift_age=501;h.pump(.1)
            assert h.act["stage"]=="stopped" and h.act["reason"]=="act_lift_feedback_unhealthy",h.act
            h.grace=False;h.lift_age=0;h.lift_speed=0;h.pump(.1);h.begin()
            h.lift_flags=51|4;h.pump(.1)
            assert h.act["reason"]=="act_lift_not_stationary",h.act
            h.lift_flags=51;h.pump(.1);h.begin()
            h.lift_flags=51 & ~1;h.pump(.1)
            assert h.act["reason"]=="act_lift_position_invalid",h.act
            h.lift_flags=51;h.lift_age=300;h.pump(.1)
            h.session=uuid.uuid4().hex;h.call("act_prepare",False)
            h.lift_age=0;h.pump(.1);h.call("act_prepare")
            h.until(lambda:h.act["stage"]=="ready",15)
            h.grace=True;h.grace_started=time.monotonic_ns()-3_000_000_000;h.lift_age=600;h.pump(.1)
            assert h.act["stage"]=="stopped" and h.act["reason"]=="act_lift_feedback_unhealthy",h.act
            print("PASS: bounded SDK lift startup grace only during initial-pose hold, never during ACT or after expiry")
            print("PASS: SDK +/-2 rpm, 300 ms healthy scan, >500 ms watchdog, moving/invalid-position flags, strict prepare")
            for previous,current in zip(h.history,h.history[1:]):
                if previous.phase==current.phase=="act_raw_position":
                    dt=current.source_mono_ns-previous.source_mono_ns
                    assert max(abs(a-b) for a,b in zip(previous.position,current.position))<=.2*dt/1e9+.002
            print("PASS: ACT output speed bound retained")
        finally:
            if udp:
                udp.send_signal(signal.SIGINT)
                try:udp.wait(5)
                except subprocess.TimeoutExpired:udp.kill();udp.wait()
            if udp_log:udp_log.close()
            h.close();rclpy.try_shutdown()

if __name__=="__main__":main()
