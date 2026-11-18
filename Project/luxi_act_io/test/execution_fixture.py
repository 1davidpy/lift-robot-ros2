"""Actual HTTP/UDP/arm driver, recorded cameras, mock motors and lift, ROS domain 88."""
import os
os.environ["ROS_DOMAIN_ID"]="88"
os.environ["ROS_LOCALHOST_ONLY"]="1"
import argparse,json,signal,subprocess,time
from pathlib import Path
import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
from std_msgs.msg import String
from luxi_interfaces.msg import CameraFrame,ArmState,ArmCommand
from luxi_interfaces.srv import Command

Q=[.246624,-.086404,-.032235,1.724079,.021553,.531586,.052071,-.793469,
   -.113870,-.006294,.007820,1.639391,-.062753,-.311856,-.161555,-.826658]

def main():
    parser=argparse.ArgumentParser();parser.add_argument("--sdk",type=Path,required=True)
    parser.add_argument("--fixture",type=Path,required=True);parser.add_argument("--duration",type=float,default=180)
    args=parser.parse_args();root=args.sdk
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO);node=rclpy.create_node("act_execution_mock_world")
    pubs={k:node.create_publisher(String,"/luxi/"+k,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL) if k=="control_status" else 1) for k in ("control_status","base/state","lift/state","lift/status")}
    qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
    images={}
    for name in ("head_rgb","left_wrist_rgb","right_wrist_rgb"):
        frame=CameraFrame(camera_id=name,width=640,height=360,encoding="jpeg",timestamp_quality="recorded_camera_mock_world")
        frame.data=(args.fixture/f"{name}.jpg").read_bytes()
        images[name]=(node.create_publisher(CameraFrame,"/luxi/cameras/"+name,qos),frame)
    world={"lift_um":0,"lift_enabled":False,"owner":"","result":"idle","stage":"idle","last_state":[],
           "initial_pose_at_begin":None,"act_applied_count":0,"chunks":0,"lift_calls":[],"stages":[],"last_driver_status":None}
    def lift(req,res):
        j=json.loads(req.detail);op=req.command
        world["lift_calls"].append({"command":op,"detail":j})
        if op=="position_arm" and world["stage"]!="ready":
            res.success=False;res.message="Arms must reach initial pose before lift";return res
        if op=="position_arm":world.update(lift_enabled=True,owner=j["owner"],result="idle")
        elif j["owner"]!=world["owner"]:res.success=False;res.message="wrong owner";return res
        elif op=="position_move":world.update(lift_um=j["target_um"],result="reached")
        elif op=="position_stop":world.update(lift_enabled=False,result="stopped")
        elif op!="position_heartbeat":res.success=False;return res
        res.success=True;res.message="mock lift accepted";return res
    service=node.create_service(Command,"/luxi/lift/control",lift)
    def status(msg):
        j=json.loads(msg.data);stage=j.get("act",{}).get("stage")
        if stage!=world["stage"]:
            world["stage"]=stage;world["stages"].append(stage)
            if stage=="running":world["initial_pose_at_begin"]=world["last_state"]
        world["last_driver_status"]=j
    def state(msg):world["last_state"]=list(msg.position)
    def applied(msg):
        if msg.phase=="act_raw_position":world["act_applied_count"]+=1
    def chunk(msg):world["chunks"]+=1
    subscriptions=[node.create_subscription(String,"/luxi/arms/startup_status",status,1),
      node.create_subscription(ArmState,"/luxi/arms/state",state,1),
      node.create_subscription(ArmCommand,"/luxi/applied",applied,10),
      node.create_subscription(String,"/luxi/act/received_chunk",chunk,10)]
    initial=Q.copy();initial[0]+=.03;initial[7]+=.02;initial[15]+=.02
    commands=[
      [str(root/"build/packages/luxi_device_adapters/openarm_adapter"),"--ros-args","-p","mock:=true","-p",
       "config:="+str(root/"Project/robot_bringup/config/arms-kh.json"),"-p","mock_initial_positions:="+json.dumps(initial),"-p","joint_stream_port:=18048"],
      [str(root/"build/packages/luxi_act_io/act_rgb_http"),"--ros-args","-p","port:=18042"],
      [str(root/"build/packages/luxi_act_io/act_action_udp"),"--ros-args","-p","port:=18043"],
      [str(root/"build/install/luxi_web_ctrl/lib/luxi_web_ctrl/web_ctrl"),"--ros-args","-p","mock:=true","-p","port:=18044","-p","host:=0.0.0.0","-p","directory:=/tmp/act-execution-mock-web"]]
    children=[];logs=[]
    try:
        for i,cmd in enumerate(commands):
            log=open(args.fixture/f"execution-{i}.log","w");logs.append(log)
            children.append(subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT))
        start=time.monotonic();last_save=0;sequence=0
        while rclpy.ok() and time.monotonic()-start<args.duration:
            if any(p.poll() is not None for p in children):raise RuntimeError("Mock service exited; inspect execution logs")
            stamp=time.monotonic_ns();sequence+=1
            data={"control_status":{"control_source":"console","system_phase":"disabled","phase":"disabled","shared_enable":True},
              "base/state":{"valid":True,"velocity":[0.,0.,0.],"receive_mono_ns":str(stamp)},
              "lift/state":{"valid":True,"current_um":world["lift_um"],"speed_rpm":0,"fault":0,"effective_age_ms":0,"receive_mono_ns":str(stamp),"receive_first_mono_ns":str(stamp),"feedback_age_ms":0,"flags":51,"guard":0,"transport":"uart"},
              "lift/status":{"enabled":world["lift_enabled"],"phase":"idle","feedback_healthy":True,"stationary":True,
                             "external_owner":world["owner"],"external_result":world["result"]}}
            for key,j in data.items():pubs[key].publish(String(data=json.dumps(j)))
            for publisher,frame in images.values():
                frame.sequence=sequence;frame.receive_mono_ns=stamp;publisher.publish(frame)
            for _ in range(30):rclpy.spin_once(node,timeout_sec=0)
            if time.monotonic()-last_save>1:
                (args.fixture/"execution_result.json").write_text(json.dumps(world,indent=2));last_save=time.monotonic()
            time.sleep(.025)
    except KeyboardInterrupt:pass
    finally:
        for p in children:
            if p.poll() is None:p.send_signal(signal.SIGINT)
        for p in children:
            try:p.wait(5)
            except subprocess.TimeoutExpired:p.kill();p.wait()
        (args.fixture/"execution_result.json").write_text(json.dumps(world,indent=2))
        for log in logs:log.close()
        node.destroy_node();rclpy.try_shutdown()
        print(json.dumps({"stages":world["stages"],"chunks":world["chunks"],"act_applied_count":world["act_applied_count"]}))

if __name__=="__main__":main()
