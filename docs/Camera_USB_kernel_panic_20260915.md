# Camera USB kernel panic — 2026-09-15

## Confirmed result

The operator confirmed that the 11:03:25 restart was not manual. The board's `hrut_resetreason` reports `PANIC`. Archived pstore data contains the crash from the two-camera SDK comparison, including the same xHCI transfer errors seen in the ROS camera test.

The immediate reset mechanism is a fatal kernel exception during USB transfer submission. The initiating cause (host-controller/driver defect, SDK/USB interaction, or hardware/power instability causing corruption) is not yet proven. Do not present insufficient battery power as the established root cause.

## Evidence

- Board: D-Robotics RDK X5 V1.0; RDKOS 3.5.0; Ubuntu 22.04.5; Linux 6.1.83, build April 8, 2026.
- Vendor SDK: MV3D RGB-D 1.4 ARM64. Native stream: 1280x720 YUY2, 30 FPS. Production ROS image output is resized on the host to 640x360.
- Head and left wrist each produced visually inspected real images and sustained fresh frames separately (approximately 22.7 and 23.6 FPS delivered through ROS).
- Running both streams together generated xHCI transfer-ring/DMA errors. One attempt killed the shared USB host, disconnecting CAN; a later attempt was stopped while CAN remained fresh.
- A single-process SDK comparison also produced transfer errors and then a kernel panic. It did not complete. This prevents attributing the failure solely to separate ROS camera processes.

Latest crash in `log/camera-panic-20260915-dmesg-ramoops-1.log`:

```text
[472.145714] ERROR Transfer event TRB DMA ptr not part of current TD ep_index 6 comp_code 4
[472.177756] Unable to handle kernel paging request at virtual address 0000802f80308230
[472.327618] CPU: 4 PID: 5520 Comm: python3 ... 6.1.83 #1
[472.347226] pc : __kmem_cache_alloc_node+0xbc/0x41c
[472.438811] __kmem_cache_alloc_node
[472.438818] __kmalloc
[472.451170] proc_do_submiturb
[472.451179] usbdev_ioctl
[472.502068] Kernel panic - not syncing: Oops: Fatal exception
```

Both original panic records have been preserved under `log/camera-panic-20260915-dmesg-ramoops-{0,1}.log`. The source is `/var/lib/systemd/pstore/`; systemd moved records there on boot. Earlier inspection of `/sys/fs/pstore` alone did not establish that crash evidence was absent. An older archived `dmesg.txt` also contains a similar C++ `hikrobot_camera` crash in `proc_do_submiturb`.

Current temperatures were approximately 54–55 C after recovery, not at the time of the crash. Exposed PVT voltage channels are not a measurement of the USB 5 V supply. The battery converter rating remains unknown.

## Current state and remaining work

All three camera services and the program service are stopped. Both arms are disabled, with fresh feedback for all 16 joints. No real dataset episode was created. No production driver, camera transport setting, kernel, firmware or boot configuration was changed during this diagnosis.

Do not repeat the same full-rate concurrent test as a stability check: it is a known kernel-panic reproducer. Further tests should change one identified variable, for example a verified board/camera supply, a temporary reduced native sensor data rate (not just ROS image resizing), or a vendor-supported USB/kernel/SDK compatibility change. A reduced-rate test is a diagnostic experiment, not proof of an adequate hardware supply or a complete fix.

Before claiming dataset collection works, require stable concurrent streams from all three cameras, fresh CAN feedback under that load, and a supervised real episode with valid aligned samples, image references and an accepted manifest.

Official release history currently lists RDKOS 3.5.0; the inspected notes do not identify a fix for this exact panic:
https://developer.d-robotics.cc/rdk_x_doc/en/Release_Note/release_note

Vendor kernel source:
https://github.com/D-Robotics/x5-kernel


## Operator PC comparison and supply-isolation plan

The operator reports that the system worked on their PC and suspects the X5's power supply. They have connected the right camera through the unpowered hub. This is useful evidence for the X5-specific power/USB path, but the exact PC camera/cable/software combination has not been independently checked.

Passive enumeration at 11:22 showed only head serial 00DA8573794 at USB2 480 Mbps plus the X5 internal hub. Right wrist and the external hub were not enumerated. Camera services remained stopped; no new concurrent capture was attempted.

The next proposed controlled comparison changes the X5 supply to a verified 5 V / 5 A adapter while keeping camera connections and stream settings fixed. Adapter availability has been requested. If a battery converter or its wiring is responsible, a known-good replacement supply should change the result. A powered camera hub is a separate comparison for USB peripheral power. PC success alone does not distinguish these from host-controller/kernel compatibility.


## Replacement supply, first passive check

The operator reports changing the X5 power supply. After reboot (boot time 11:24:04), arms are disabled with all 16 joints fresh and valid. All camera/program services remain inactive. Only head camera 00DA8573794 enumerates, at USB2 480 Mbps. The external hub and wrist cameras are absent. A direct right-camera connection and the replacement supply rating have been requested. No concurrent stream comparison has yet been run on the replacement supply, so its effect on the crash is unknown.


## Head and right directly connected on replacement supply

The operator confirmed direct head/right connections and a 5 V replacement supply. Its amperage rating remains unconfirmed. Both cameras enumerated at USB2 480 Mbps (head 1-1.1, right 1-1.4). The original two 1280x720 YUY2 streams at 30 FPS request 110,592,000 bytes/s combined (884.736 Mbps before overhead), exceeding the USB2 link rate. A full-rate concurrent test was not attempted.

A temporary right-camera-only diagnostic attempted 640x360 at 15 FPS. The SDK returned USB write error 0x80060030 when setting AcquisitionFrameRate. Width/height were restored; a later readback confirmed the original frame rate remained 30 FPS.

A second right-only diagnostic changed only native resolution to 640x360. The SDK confirmed the dimensions and 30 FPS, but acquisition stalled without reporting frames. The external 45-second timeout plus 5-second kill grace ended it; no reboot occurred, and CAN feedback remained fresh. The precise stalled SDK call was not instrumented.

A separate restore-only command succeeded and read back Width=1280, Height=720, AcquisitionFrameRate=30. All handles were closed. Production camera and program services remain stopped; no production source, kernel or boot configuration was changed. No real dataset episode was recorded.

Evidence: `log/probe-sdk-right-low-rate-20260915.log`, `log/probe-sdk-right-low-resolution-20260915.log`, `log/restore-right-camera-after-probe-20260915.log`, and `log/camera-new-supply-direct-check-20260915.json`.

The next requested physical checks are the complete adapter OUTPUT rating and using the previously working left-wrist USB3 cable for the right camera. The supply-change effect on stable simultaneous streaming is still unresolved because the current USB links differ from the earlier USB3 tests.


## Replacement supply confirmed below board specification

The operator confirmed that the replacement adapter is rated 5 V / 2 A (10 W). The X5 manufacturer specifies a 5 V / 5 A adapter (25 W). The replacement supply is therefore below the board's specified supply capacity; changing to it did not constitute a known-good power-supply comparison.

This materially strengthens the power-supply hypothesis but does not by itself prove that insufficient power caused the recorded USB/kernel memory fault. No USB 5 V voltage under camera load has been measured. USB2 negotiation and the kernel/SDK compatibility issue still require retesting with an adequate supply.

Keep simultaneous camera acquisition stopped until an X5-compatible 5 V / 5 A supply and suitable power cable are available. Then verify USB3 negotiation before staged one-, two-, and three-camera streaming and a supervised real recording test. No additional camera stream was started after learning the 2 A rating.

Current manufacturer reference: https://d-robotics.github.io/rdk_x_doc/en/Quick_start/hardware_introduction/rdk_x5/


## 2026-09-15 13:02 three-camera test after wrist cable swap

User requested all three cameras and reported swapping the left/right wrist cables. Immediately before this test, USB enumerated right 00DA8573792 at 2-1.2 (5000 Mbps), left 00DB1265540 at 2-1.4 (5000 Mbps), and head 00DA8573794 at 1-1.1 (480 Mbps). No external camera hub appeared in this topology. Arms were disabled with fresh feedback from all 16 joints; base and program/recorder services were stopped.

The normal three camera services were started through the dashboard API with a 45-second independent stop timer and a live stale-frame/CAN/kernel-fault monitor. All three produced non-mock 640x360 frames. At monitor elapsed 5.32 s, sequences were head=15, left=26, right=13. At 8.53 s, head had advanced to 85 (41 ms old), but left was stuck at 63 (1219 ms old) and right at 50 (1260 ms old). The monitor requested all camera services stop immediately at 13:02:38.

At 13:02:43 the kernel reported `xHCI host not responding to stop endpoint command`, `host controller not responding, assume dead`, and `HC died; cleaning up`. Both internal hub branches and the KCAN adapter disconnected. The right camera process reported `double free or corruption (!prev)` during shutdown; the left camera service exceeded its 15-second shutdown timeout. These observations do not establish whether the SDK/driver, USB hardware/link, or inadequate supply initiated the failure. In particular, the USB host failure was logged during shutdown after the wrist streams had already stalled.

Cleanup confirmed MainPID=0 for all three cameras, arm monitor, program, and base. The camera stop timer and arm monitor retry loop were stopped. Head service was inactive, both wrist services retained failed status. No reboot was issued; boot time remained 12:56:06. USB/CAN recovery is pending a separate reboot decision. GET /api/episodes returned []; real dataset recording remains unverified.

Evidence retained on the robot:
- `log/camera-three-after-cable-swap-20260915-130229.json`
- `log/camera-three-after-cable-swap-20260915-130229-kernel.log`
- `log/camera-three-after-cable-swap-20260915-130229-services.log`


## 2026-09-15 13:08 all three cameras on USB 3 after head cable replacement

The user authorized reboot and checking after replacing the head camera cable. Reboot at 13:06:45 restored the internal hubs and KCAN adapter. All three camera serials enumerated directly under the internal hub at 5000 Mbps: head 00DA8573794 on 2-1.1, right 00DA8573792 on 2-1.2, left 00DB1265540 on 2-1.4. Arm feedback was fresh for all 16 joints and arms remained disabled; camera, program, and base services were initially inactive.

At 13:08:17, a bounded three-camera test used the normal dashboard start API, an independent 70-second camera stop timer, and a fault monitor. Wrist cameras produced real frames but froze (last observed sequences left=118, right=105). The head produced no observed frame and later reported MV3D OpenDeviceBySerialNumber failure 0x80060030 (USB write failure). At monitor elapsed 6.65 s the wrist image ages were 1819/1737 ms, triggering immediate camera shutdown. No simultaneous three-camera image set was captured.

At 13:08:28 the kernel again reported xHCI stop-endpoint timeout, host controller not responding, and HC died. The USB hubs and CAN adapter disconnected. Both wrist services required timeout termination; all three camera processes exited. The CAN monitor retry loop was stopped. No firmware, SDK, native camera settings, or production control code was changed. All three cameras being on USB 3 did not resolve the streaming failure. The previously reported 5 V / 2 A supply remains below the board specification; the initiating cause is still unproven.

Evidence: `log/camera-three-usb3-after-reboot-20260915-130817.json`, matching `-kernel.log` and `-services.log`. A recovery reboot with camera/program/base autostart disabled is being performed to restore USB/CAN; no additional streaming test is planned for this setup.

Recovery check: boot 2026-09-15 13:09:53. Final observation {"time": "2026-09-15T13:11:07", "arms_enabled": false, "arms_fresh": true, "valid_joints": 16, "devices": [{"path": "2-1.2", "serial": "00DA8573792", "speed": "5000", "product": "MV-EB435i"}, {"path": "1-1.3", "serial": "KCANX4WCID001", "speed": "480", "product": "KCAN-USB X4"}, {"path": "2-1.1", "serial": "00DA8573794", "speed": "5000", "product": "MV-EB435i"}, {"path": "2-1.4", "serial": "00DB1265540", "speed": "5000", "product": "MV-EB435i"}], "services": {"base": false, "control": false, "episode": false}}. All camera/program/base process states and boot USB errors are retained in `log/camera-three-usb3-recovery-20260915.json`. No camera streams were restarted during recovery.
