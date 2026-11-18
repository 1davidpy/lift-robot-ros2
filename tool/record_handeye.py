#!/usr/bin/env python3
"""Read-only hand-eye recording; never sends actuator or lifecycle commands."""
import argparse, csv, collections, datetime, hashlib, json, os, pathlib, shutil, signal, subprocess, time, uuid
import rclpy
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from sensor_msgs.msg import CameraInfo
from luxi_interfaces.msg import ArmState

ROOT=pathlib.Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser();p.add_argument('--seconds',type=float,default=0);p.add_argument('--preflight',action='store_true');a=p.parse_args()
base=ROOT/'data/handeye_sessions';base.mkdir(parents=True,exist_ok=True)
if shutil.disk_usage(base).free<5*1024**3: raise SystemExit('Need at least 5 GiB free')
run=base/(('preflight_' if a.preflight else 'handeye_')+datetime.datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:6]);run.mkdir()
(base/'latest_session.txt').write_text(str(run)+'\n')
def write(path,obj):
 tmp=path.with_suffix('.tmp');tmp.write_text(json.dumps(obj,ensure_ascii=False,indent=2));tmp.replace(path)
cams=['head_rgb','left_wrist_rgb','right_wrist_rgb']
topics=['/luxi/arms/state','/luxi/arms/startup_status','/luxi/applied','/luxi/command','/luxi/intent','/luxi/targets','/luxi/quest','/luxi/control_status','/luxi/coordinated_control','/luxi/base/state','/luxi/base/status','/luxi/base/applied','/luxi/lift/state','/luxi/lift/status','/luxi/lift/events','/luxi/lift/uart_rx','/luxi/faults','/tf','/tf_static']
for c in cams: topics += ['/luxi/cameras/'+c,'/luxi/cameras/'+c+'/status','/camera/'+c+'/color/camera_info']
write(run/'session.json',{'purpose':'hand_eye_calibration','preflight':a.preflight,'joint_order':'right[8],left[8]','joint_units':'rad','lift_units':'um; position_m is metres','time_domains':'bag ROS wall timestamps; payload host monotonic ns; camera device time unmapped','segment_policy':'heuristic: valid lift samples spanning >=0.8s, height range <=500um, speed <=2rpm; new stable anchor after >1000um change; invalid feedback remains flagged in CSV','topics':topics,'depth_recorded':False,'bag_shared_by_segments':True})
snap=run/'snapshot';snap.mkdir()
shutil.copy2(__file__,snap/'record_handeye.py')
target=ROOT/'tool/handeye_target.json'
if target.exists():shutil.copy2(target,snap/'target_config.json')
shutil.copytree(ROOT/'Project/robot_bringup/config',snap/'source_config')
shutil.copytree(ROOT/'build/install/luxi_bringup/share/luxi_bringup/config',snap/'installed_config')
shutil.copytree(ROOT/'Project/robot_description',snap/'robot_description',ignore=shutil.ignore_patterns('__pycache__','.git'))
shutil.copytree(ROOT/'build/install/luxi_description/share/luxi_description/model',snap/'installed_model')
for cmd,name in [(['git','rev-parse','HEAD'],'git_head.txt'),(['git','diff','--','Project'],'git_diff.patch'),(['systemctl','cat','luxi-program.service','luxi-openarm-monitor.service','luxi-base.service','luxi-lift.service'],'services.txt')]:
 with (snap/name).open('w') as f: subprocess.run(cmd,cwd=ROOT,stdout=f,stderr=subprocess.STDOUT)
raw=open(run/'lift_timeline.csv','w',newline='');csvout=csv.writer(raw);csvout.writerow(['callback_mono_ns','lift_receive_mono_ns','current_um','target_um','speed_rpm','valid','fault','feedback_age_ms','run_segment','height_segment','classification'])
rclpy.init();node=rclpy.create_node('handeye_record_index_'+str(os.getpid()))
window=collections.deque();segments=[];current=None;arm_seen=0;info_seen={};stop=False;last_lift_callback=0
def segment(kind,h,source,now):
 global current
 if current is not None and current['kind']==kind and (kind!='stable' or abs(h-current['anchor_um'])<=1000): return
 if current:
  current['end_mono_ns']=str(now);current['end_lift_mono_ns']=str(source);write(run/current['directory']/'segment.json',current)
 label=(f'h{h/1000:+.1f}mm' if kind=='stable' else kind)
 directory=pathlib.Path('height_segments')/(f'{len(segments):04d}_'+label)
 (run/directory).mkdir(parents=True)
 current={'id':len(segments),'kind':kind,'anchor_um':h,'start_mono_ns':str(now),'start_lift_mono_ns':str(source),'directory':str(directory),'bag':'../../bag','frame_reference':'choose arm-base frame or include lift/base transforms for fixed world target'};segments.append(current)
 write(run/directory/'segment.json',current);write(run/'height_segments.json',segments)
def lift(msg):
 global last_lift_callback
 try: j=json.loads(msg.data)
 except (ValueError,TypeError): return
 now=time.monotonic_ns();last_lift_callback=now;source=int(j.get('receive_mono_ns',0));h=int(j['current_um']);good=bool(j.get('valid')) and j.get('fault',0)==0 and float(j.get('effective_age_ms',j.get('feedback_age_ms',1e9)))<=250
 if good:
  window.append((now,h,abs(j.get('speed_rpm',999))))
  while window and now-window[0][0]>1200000000:window.popleft()
  stable=len(window)>=5 and now-window[0][0]>=800000000 and max(x[1] for x in window)-min(x[1] for x in window)<=500 and max(x[2] for x in window)<=2
  segment('stable' if stable else 'transition',round(sum(x[1] for x in window)/len(window)) if stable else h,source,now)
 elif current is None: segment('invalid_feedback',h,source,now)
 csvout.writerow([now,source,h,j.get('target_um'),j.get('speed_rpm'),j.get('valid'),j.get('fault'),j.get('effective_age_ms',j.get('feedback_age_ms')),j.get('run_segment'),current['id'] if current else '',current['kind'] if good else 'invalid_feedback']);raw.flush()
def arm(msg):
 global arm_seen
 arm_seen+=1
def camera_info(msg,c):
 info={'width':msg.width,'height':msg.height,'k':list(msg.k),'d':list(msg.d),'r':list(msg.r),'p':list(msg.p),'distortion_model':msg.distortion_model,'frame_id':msg.header.frame_id,'stamp_sec':msg.header.stamp.sec,'stamp_nanosec':msg.header.stamp.nanosec}
 if c not in info_seen:write(snap/(c+'_camera_info.json'),info)
 info_seen[c]=info
node.create_subscription(String,'/luxi/lift/state',lift,100)
node.create_subscription(ArmState,'/luxi/arms/state',arm,100)
for c in cams:node.create_subscription(CameraInfo,'/camera/'+c+'/color/camera_info',lambda m,c=c:camera_info(m,c),qos_profile_sensor_data)
log=open(run/'rosbag.log','w')
bag=subprocess.Popen(['ros2','bag','record','-o',str(run/'bag'),'-b','1073741824','--max-cache-size','33554432',*topics],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
def halt(*_):
 global stop
 stop=True
signal.signal(signal.SIGINT,halt);signal.signal(signal.SIGTERM,halt)
print('RECORDING_DIRECTORY='+str(run),flush=True);start=time.monotonic();reason='operator_stop'
try:
 while not stop:
  rclpy.spin_once(node,timeout_sec=.1)
  if bag.poll() is not None:reason='bag_process_exited';break
  if a.seconds and time.monotonic()-start>=a.seconds:reason='duration_complete';break
  if shutil.disk_usage(base).free<2*1024**3:reason='free_space_below_2GiB';break
  if current and last_lift_callback and time.monotonic_ns()-last_lift_callback>700000000:
   segment('missing_feedback',current['anchor_um'],0,time.monotonic_ns());window.clear()
finally:
 if bag.poll() is None:
  bag.send_signal(signal.SIGINT)
  try:bag.wait(timeout=30)
  except subprocess.TimeoutExpired:bag.terminate();bag.wait(timeout=10)
 if current:
  current['end_mono_ns']=str(time.monotonic_ns());write(run/current['directory']/'segment.json',current)
 write(run/'height_segments.json',segments)
 write(run/'recording_result.json',{'reason':reason,'bag_returncode':bag.returncode,'arm_messages_indexer':arm_seen,'camera_info_received':list(info_seen),'duration_s':time.monotonic()-start})
 raw.close();log.close();node.destroy_node();rclpy.try_shutdown()
 print('FINALIZED='+str(run),flush=True)
