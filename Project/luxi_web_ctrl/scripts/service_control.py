#!/usr/bin/python3
"""Root-owned allowlist installed in /usr/local/libexec; no caller-supplied paths."""
import json
import subprocess
import sys
UNITS = {"arms": "luxi-openarm-monitor.service", "base": "luxi-base.service", "quest": "luxi-quest.service", "program": "luxi-program.service", **{k: "luxi-rgb-" + k + ".service" for k in ("head_rgb", "left_wrist_rgb", "right_wrist_rgb")}}
def run(args):
    return subprocess.run(["/usr/bin/systemctl", *args], text=True, capture_output=True, timeout=5)
try:
    if sys.argv[1:] == ["status"]:
        result = run(["show", *UNITS.values(), "--property=Id,LoadState,ActiveState,SubState,Result"])
        devices = {}
        reverse = {v: k for k, v in UNITS.items()}
        for block in result.stdout.strip().split("\n\n"):
            values = dict(line.split("=", 1) for line in block.splitlines() if "=" in line)
            if values.get("Id") in reverse:
                devices[reverse[values["Id"]]] = values
        print(json.dumps({"success": result.returncode == 0, "devices": devices}))
    elif len(sys.argv) == 3 and sys.argv[1] in ("start", "stop", "restart") and sys.argv[2] in UNITS:
        result = run(["--no-block", sys.argv[1], UNITS[sys.argv[2]]])
        print(json.dumps({"success": result.returncode == 0, "message": "操作已提交，请观察服务状态和实时反馈" if result.returncode == 0 else result.stderr.strip()}))
    else:
        print(json.dumps({"success": False, "message": "Invalid action/device"}))
        sys.exit(2)
except Exception:
    print(json.dumps({"success": False, "message": "设备管理响应超时或不可用"}))
