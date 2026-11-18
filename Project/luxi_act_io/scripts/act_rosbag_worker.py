"""Restricted SSH entry point: record fixed ROS topics; never issues motor commands."""
import json
import os
from pathlib import Path
import re
import select
import shutil
import signal
import subprocess
import sys
import time

SDK = Path('/home/sunrise/Desktop/LUXI_Lift_Robot/luxi_lift_ros2/lift_robot_ros2')
TOPICS = ['/luxi/arms/state', '/luxi/applied', '/luxi/arms/startup_status',
          '/luxi/lift/state', '/luxi/lift/status', '/luxi/control_status',
          '/luxi/act/received_chunk', '/luxi/act/ingress_status', '/rosout']
CAMERAS = ['/luxi/cameras/' + name for name in ('head_rgb', 'left_wrist_rgb', 'right_wrist_rgb')]


def emit(event, **fields):
    try:
        print(json.dumps(dict(event=event, **fields)), flush=True)
    except (BrokenPipeError, OSError):
        pass


def main():
    process = None
    folder = None
    reason = 'stopped'
    def interrupted(signum, frame):
        raise RuntimeError('Recording connection interrupted: signal ' + str(signum))
    for sig in (signal.SIGTERM, signal.SIGHUP, signal.SIGINT):
        signal.signal(sig, interrupted)
    try:
        if not select.select([sys.stdin], [], [], 10)[0]:
            raise RuntimeError('Recording handshake timeout')
        request = json.loads(sys.stdin.readline())
        session = request['session_id']
        if not isinstance(session, str) or not re.fullmatch('[0-9a-f]{32}', session):
            raise ValueError('Invalid session')
        domain = request.get('domain', 0)
        if type(domain) is not int or domain not in (0, 88):
            raise ValueError('Invalid ROS domain')
        if type(request.get('images', False)) is not bool:
            raise ValueError('Invalid images option')
        if shutil.disk_usage(SDK).free < 1024**3:
            raise RuntimeError('Less than 1 GiB free for recording')
        folder = SDK / 'outputs/act_rosbags' / (time.strftime('%Y%m%d_%H%M%S_') + session)
        folder.mkdir(parents=True, exist_ok=False)
        topics = TOPICS + (CAMERAS if request.get('images') else [])
        (folder / 'session.json').write_text(json.dumps(dict(request, topics=topics), indent=2))
        # Preserve the ROS environment used by the SDK. All shell code is fixed.
        command = ['bash', '-c', 'source /opt/ros/humble/setup.bash && source "$1/build/install/setup.bash" && shift && exec ros2 bag record "$@"',
                   'act-rosbag', str(SDK), '-s', 'sqlite3', '-o', str(folder / 'bag'),
                   '--max-bag-size', '268435456', *topics]
        env = dict(os.environ, ROS_DOMAIN_ID=str(domain), ROS_LOCALHOST_ONLY='1', PYTHONUNBUFFERED='1')
        with (folder / 'recorder.log').open('w') as log:
            process = subprocess.Popen(command, env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        deadline = time.monotonic() + 15
        while True:
            if process.poll() is not None:
                raise RuntimeError('ros2 bag exited; inspect recorder.log')
            output = (folder / 'recorder.log').read_text(errors='replace')
            if 'Recording...' in output and all("Subscribed to topic '" + topic + "'" in output for topic in topics):
                break
            if time.monotonic() > deadline:
                raise RuntimeError('ros2 bag startup timeout; inspect recorder.log')
            time.sleep(.1)
        emit('started', path=str(folder / 'bag'), session_id=session, topics=topics)
        last_ping = began = time.monotonic()
        while process.poll() is None:
            if select.select([sys.stdin], [], [], 1)[0]:
                line = sys.stdin.readline()
                if not line or line.strip() == 'STOP':
                    break
                if line.strip() != 'PING':
                    raise ValueError('Unexpected recording command')
                last_ping = time.monotonic()
            if time.monotonic() - last_ping > 30:
                raise RuntimeError('Recording client heartbeat expired')
            if time.monotonic() - began > 14400 or shutil.disk_usage(folder).free < 512 * 1024**2:
                raise RuntimeError('Recording duration/disk reserve reached')
        else:
            raise RuntimeError('ros2 bag exited unexpectedly')
    except Exception as exc:
        reason = str(exc)
        emit('error', reason=reason, path=str(folder) if folder else None)
    finally:
        for sig in (signal.SIGTERM, signal.SIGHUP, signal.SIGINT):
            signal.signal(sig, signal.SIG_IGN)
        if process and process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
                reason = 'Recorder shutdown timed out; bag may require reindex'
        if folder:
            complete = (folder / 'bag/metadata.yaml').exists()
            result = dict(path=str(folder / 'bag'), complete=complete, reason=reason)
            (folder / 'result.json').write_text(json.dumps(result, indent=2))
            emit('stopped', **result)


if __name__ == '__main__':
    main()
