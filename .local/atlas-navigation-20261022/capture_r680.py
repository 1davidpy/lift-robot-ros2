import os
os.environ['ROS_LOCALHOST_ONLY']='1';os.environ['ROS_DOMAIN_ID']='0'
import json,time
from pathlib import Path
import rclpy
from std_msgs.msg import String
rclpy.init();n=rclpy.create_node('r680_acceptance_passive_capture');rows=[]
def callback(topic):
 def cb(msg):rows.append(dict(t=time.monotonic(),topic=topic,value=json.loads(msg.data)))
 return cb
for t in ['state','applied','status']:n.create_subscription(String,'/luxi/base/'+t,callback(t),50)
print('READY',flush=True);end=time.monotonic()+18
while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.02)
p=Path('/home/sunrise/Desktop/LUXI_Lift_Robot/luxi_lift_ros2/lift_robot_ros2/.local/atlas-navigation-20261022/real-motion-ros.json')
p.write_text(json.dumps(rows));print(len(rows),flush=True);n.destroy_node();rclpy.shutdown()
