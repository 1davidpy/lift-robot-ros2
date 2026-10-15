#!/usr/bin/python3
"""Root-owned allowlist installed in /usr/local/libexec; no caller-supplied paths."""
import json
import subprocess
import sys
import shlex
UNITS = {"arms": "luxi-openarm-monitor.service", "base": "luxi-base.service", "lift": "luxi-lift.service", "quest": "luxi-quest.service", "program": "luxi-program.service", **{k: "luxi-rgb-" + k + ".service" for k in ("head_rgb", "left_wrist_rgb", "right_wrist_rgb")}}
def run(args):
    return subprocess.run(["/usr/bin/systemctl", *args], text=True, capture_output=True, timeout=5)

def selected_units():
    # Only fixed, root-owned units are eligible. A native-depth-only service
    # must never replace the RGB device control entry.
    units = dict(UNITS)
    cameras = ("head_rgb", "left_wrist_rgb", "right_wrist_rgb")
    depth_units = {"luxi-depth-" + camera + ".service": camera for camera in cameras}
    result = run(["show", *depth_units, "--property=Id,ActiveState,UnitFileState,Environment"])
    for block in result.stdout.strip().split("\n\n"):
        values = dict(line.split("=", 1) for line in block.splitlines() if "=" in line)
        unit = values.get("Id")
        if (unit in depth_units and "ALIGN_TO_COLOR=true" in shlex.split(values.get("Environment", ""))
                and (values.get("ActiveState") in ("active", "activating")
                     or values.get("UnitFileState") in ("enabled", "enabled-runtime"))):
            units[depth_units[unit]] = unit
    return units

try:
    units = selected_units()
    if sys.argv[1:] == ["status"]:
        result = run(["show", *units.values(), "--property=Id,LoadState,ActiveState,SubState,Result"])
        devices = {}
        reverse = {v: k for k, v in units.items()}
        for block in result.stdout.strip().split("\n\n"):
            values = dict(line.split("=", 1) for line in block.splitlines() if "=" in line)
            if values.get("Id") in reverse:
                devices[reverse[values["Id"]]] = values
        print(json.dumps({"success": result.returncode == 0, "devices": devices}))
    elif len(sys.argv) == 3 and sys.argv[1] in ("start", "stop", "restart") and sys.argv[2] in UNITS:
        result = run(["--no-block", sys.argv[1], units[sys.argv[2]]])
        print(json.dumps({"success": result.returncode == 0, "message": "操作已提交，请观察服务状态和实时反馈" if result.returncode == 0 else result.stderr.strip()}))
    else:
        print(json.dumps({"success": False, "message": "Invalid action/device"}))
        sys.exit(2)
except Exception:
    print(json.dumps({"success": False, "message": "设备管理响应超时或不可用"}))
