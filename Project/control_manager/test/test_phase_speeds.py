#!/usr/bin/env python3
"""Check following response and slow transitions with an isolated fake actuator."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import unittest

os.environ["ROS_DOMAIN_ID"] = "98"
os.environ["ROS_LOCALHOST_ONLY"] = "1"
import rclpy
from luxi_interfaces.msg import ArmCommand, ArmState, QuestInput
from luxi_interfaces.srv import Command
from std_msgs.msg import String

BINARY = Path(sys.argv.pop(1))
CONFIG = Path(sys.argv.pop(1))
CFG = json.loads(CONFIG.read_text())
HOME = CFG["right"]["home"] + CFG["left"]["home"]
for side, name in enumerate(["right", "left"]):
    for joint, (lo, hi) in enumerate(CFG[name]["limits"]):
        i = side * 8 + joint
        HOME[i] = min(max(HOME[i], lo), hi)


class PhaseSpeeds(unittest.TestCase):
    def test_fast_following_preserves_slow_home_alignment_and_timed_return(self):
        with tempfile.TemporaryDirectory(prefix="luxi-phase-speeds-") as tmp:
            prefix = "/phase_speeds_" + str(time.monotonic_ns())
            args = [str(BINARY), "--ros-args", "-p", "config:=" + str(CONFIG),
                    "-p", "home_joint_speed_limit:=0.2"]
            for topic in ["control_status", "quest", "arms/state", "intent", "control",
                          "command", "driver_lifecycle"]:
                args += ["-r", f"/luxi/{topic}:={prefix}/{topic}"]
            with open(Path(tmp) / "manager.log", "w+") as log:
                process = subprocess.Popen(args, env=dict(os.environ, ROS_LOG_DIR=tmp),
                                           stdout=log, stderr=subprocess.STDOUT)
                node = rclpy.create_node("phase_speeds_test")
                status = {}
                measured = HOME.copy()
                measured[0] -= .3
                target = HOME.copy()
                target[0] += .3
                commands = []

                def on_command(message):
                    measured[:] = list(message.position)
                    commands.append(message)

                node.create_subscription(String, prefix + "/control_status",
                                         lambda m: status.update(json.loads(m.data)), 1)
                node.create_subscription(ArmCommand, prefix + "/command", on_command, 100)
                qp = node.create_publisher(QuestInput, prefix + "/quest", 1)
                sp = node.create_publisher(ArmState, prefix + "/arms/state", 1)
                ip = node.create_publisher(ArmCommand, prefix + "/intent", 1)
                client = node.create_client(Command, prefix + "/control")

                def step():
                    now = time.monotonic_ns()
                    state = ArmState()
                    state.receive_mono_ns = now
                    state.position = measured.copy()
                    state.valid = [True] * 16
                    sp.publish(state)
                    quest = QuestInput()
                    quest.receive_mono_ns = now
                    quest.triggers = [1., 1.]
                    qp.publish(quest)
                    intent = ArmCommand()
                    intent.source_mono_ns = now
                    intent.position = target.copy()
                    intent.phase = "teleop"
                    ip.publish(intent)
                    for _ in range(4):
                        rclpy.spin_once(node, timeout_sec=0)
                    time.sleep(.006)

                def run_for(seconds):
                    end = time.monotonic() + seconds
                    while time.monotonic() < end:
                        step()

                def wait_for(condition, seconds, description):
                    end = time.monotonic() + seconds
                    while not condition() and time.monotonic() < end:
                        step()
                    self.assertTrue(condition(), f"{description}; status={status}")

                def request(command):
                    req = Command.Request()
                    req.command = command
                    future = client.call_async(req)
                    wait_for(future.done, 2, "service response")
                    return future.result().success

                try:
                    run_for(1.4)
                    self.assertIsNone(process.poll())
                    wait_for(client.service_is_ready, 3, "manager discovery")
                    accepted = False
                    for _ in range(5):
                        accepted = request("start")
                        if accepted:
                            break
                        run_for(.3)
                    self.assertTrue(accepted)
                    wait_for(lambda: status.get("phase") == "teleop", 6, "home and align")
                    run_for(.7)
                    before = measured[0]
                    target[0] += .5
                    run_for(.65)
                    advance = measured[0] - before
                    print(f"following: {advance:.3f} rad in 0.65 s for a 0.5 rad target step",
                          flush=True)
                    self.assertGreater(advance, .4, "following remains limited by the slow profile; status="+json.dumps(status))
                    self.assertLessEqual(advance, .501)
                    self.assertTrue(request("stop"))
                    wait_for(lambda: status.get("phase") == "disabled", 7, "slow return")
                    for phase in ["homing", "aligning", "returning"]:
                        samples = [c for c in commands if c.phase == phase]
                        self.assertGreater(len(samples), 20, phase)
                        rates = []
                        for previous, current in zip(samples, samples[1:]):
                            dt = (current.source_mono_ns - previous.source_mono_ns) * 1e-9
                            for i in range(16):
                                if i % 8 == 7:
                                    continue
                                rate = abs(current.position[i] - previous.position[i]) / dt
                                rates.append(rate)
                                # The legacy two-second return uses the configured
                                # joint limits; preparation has the slower cap.
                                limit = CFG["velocity"][i % 8] if phase == "returning" else .2
                                self.assertLessEqual(rate, limit + 1e-5, phase + " accelerated")
                        peak = max(rates)
                        print(f"{phase}: maximum commanded slew {peak:.6f} rad/s", flush=True)
                    self.assertAlmostEqual(status["home_joint_speed_limit"], .2)
                    self.assertEqual(status["return_steps"], 100)
                    self.assertAlmostEqual(status["return_nominal_duration_s"], 2.0)
                finally:
                    node.destroy_node()
                    if process.poll() is None:
                        process.send_signal(signal.SIGINT)
                    try:
                        process.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait()


if __name__ == "__main__":
    rclpy.init()
    try:
        unittest.main(verbosity=2)
    finally:
        if rclpy.ok():
            rclpy.shutdown()
