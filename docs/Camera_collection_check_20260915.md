# Camera and real collection check — 2026-09-15

Real visual collection is not confirmed. No real episode was created during this check.

## Observations

- Initial USB enumeration found only head camera `00DA8573794` at `2-1.4`, 5000 Mbps. All camera acquisition services were stopped.
- Starting only the managed head-camera service returned success, but the driver produced no frames and repeatedly failed `MV3D OpenDeviceBySerialNumber` with `0x80060030` (USB write failure).
- The service user has read/write access to the camera USB node. Exactly one camera process was running, using the project-local ARM64 MV3D RGB-D 1.4 SDK. This was not a missing-SDK or duplicate-process observation.
- Kernel events show repeated head/right-wrist disconnects and xHCI setup/address failures (`-71`). During the final 30-second passive observation, the right wrist (`00DA8573792`) briefly appeared at USB 3 speed, the head briefly appeared at USB 2 speed (480 Mbps), then neither remained enumerated. Left wrist `00DB1265540` was not observed.
- All three camera preview endpoints returned HTTP 503. USB permissions and service startup success do not establish a working image stream.
- The head service was stopped after repeated open failures. No camera, arm or base motion configuration was changed; the arms remained disabled with 16 fresh valid joints.
- The current real episode list is empty. Control, base and recorder services are stopped; Quest input is 0 Hz. Free disk space is approximately 37.1 GiB (above the recorder's 20 GiB reserve).

## Interpretation and next check

USB communication is failing before usable images reach the recorder. Cabling, hub/power conditions or ongoing manual reconnection must be distinguished; the operator has been asked whether cables are currently being changed. No specific cable or hub is proven faulty by these observations alone.

Once connections are settled, test one camera directly on a USB 3 port with a known-good USB 3 data cable, leaving the CAN adapter connected. Confirm stable enumeration and real frames before adding cameras one at a time. After all three views are stable, run an operator-supervised short teleop episode and inspect its manifest, samples, bag image/action counts and timestamps. Do not substitute missing images or label diagnostic data as an accepted demonstration.

Software/mock collection validation is documented in `Quest_button_mapping_validation_20260915.md`; that is not proof of real camera collection.

Evidence: `log/camera-collection-check-20260915.json`, `journalctl -u luxi-rgb-head_rgb.service`, and kernel USB events around 10:26–10:38 local time.


## Follow-up after cabling, 2026-09-15 10:49 CST

The operator confirmed that cables were being changed during the earlier USB disconnects, then reported all three connected. Earlier disconnects are therefore not proof of an untouched-hardware fault.

- Before the reconnect, head RGB produced real 640x360 frames, sequence 549 to 754 over approximately 9 seconds. The stream was not continuously stable through the following restart.
- The computer reports a boot time of 10:42:22; camera and program services were inactive after reconnect. Arms still reported disabled with 16 valid joints.
- At 10:46:29 to 10:46:49, only left wrist (00DB1265540, USB3 5000 Mbps) and head (00DA8573794, USB2 480 Mbps) enumerated. Right wrist was absent.
- Starting only the left wrist stream produced real 640x360 frames up to sequence 92. The last image became stale; the kernel reported USB disconnect at 10:47:02, followed by SDK frame timeout and camera-not-connected errors. Right wrist had address-setup error -71 at 10:46:43.
- Left wrist re-enumerated at 10:47:29. Its streaming service was stopped to avoid repeated retries while cabling is isolated.
- The operator says both wrists share a hub. A one-wrist connection bypassing that hub has been requested; no result yet. The evidence does not distinguish cable, hub, power, or ongoing physical handling.
- No actual image was visually inspected: image requests occurred after frames became stale and returned HTTP 503.
- No real episode was recorded. The recorder requires all three camera frames younger than 200 ms, fresh valid enabled arm state, and applied teleop commands for aligned samples. Real collection is not yet verified.

Evidence snapshot: `log/camera-collection-check-20260915-after-reconnect.json`.


## Direct-connection test, shared USB failure and recovery

- Left wrist was moved to port 2-1.4 at USB3 5000 Mbps. A real live image (frame 189) was visually inspected: it showed robot hardware, not mock imagery.
- From 10:49:46 to 10:50:36, all 11 sampled checks were fresh. Sequence advanced 538 to 1719 (about 23.6 FPS), image ages 11 to 50 ms. It continued streaming until approximately 10:52:30. Evidence: `log/camera-left-direct-20260915.json`.
- Head was added on port 2-1.1, USB3 5000 Mbps, and its service started at 10:52:25. It produced only 17 frames before both camera streams stalled.
- At 10:52:35 the kernel reported `xHCI host controller not responding, assume dead` and `HC died; cleaning up`. KCAN returned errors -108/-19; both cameras and the CAN adapter disappeared together. Evidence: `log/camera-usb-host-failure-20260915.log`.
- Arms had remained disabled throughout testing. Their feedback became stale when CAN disappeared. Both camera services were stopped. No robot motion or real recording was started.
- Operator explicitly authorized reboot. After reboot (boot time 10:55:21), all 16 arm joints again had fresh valid feedback and enabled=false. All camera and program services remained inactive. Left wrist enumerated USB3 5000 Mbps; head enumerated USB2 480 Mbps.
- Both cameras use the X5's shared internal USB hub. Bypassing the external wrist hub improves isolation but does not provide an independent host controller. Official hardware documentation: https://d-robotics.github.io/rdk_doc/en/Quick_start/hardware_introduction/rdk_x5/
- Head-cable replacement/reseating and the board power-supply specification have been requested. The cause remains unproven: connection/power stability and host/SDK transport behavior require separation.
- The SDK is configured for ImageMode=8 (color). Output size 640x360 is a host-side resize; it does not establish the camera's native USB transport resolution. No transport-rate or camera-mode changes were made during these tests.
- No completed real episode exists; full three-camera dataset capture remains unverified.


## After head cable replacement/reseating

The operator reported replacing/reseating the head cable. Both head and left enumerated USB3 5000 Mbps. Power source is reportedly a battery; converter voltage/current and capacity are unknown.

- Head alone: a real image (frame 8) was visually inspected. From 10:58:45 to 10:59:25, all 9 checks had fresh images and fresh CAN feedback. Sequence 325 to 1231 over 40 seconds gives about 22.7 FPS. Frame ages were 13 to 60 ms. Evidence: `log/camera-head-alone-20260915.json`.
- Starting left while head streamed again froze the head stream. At 10:59:49, the kernel reported repeated `Transfer event TRB DMA ptr not part of current TD` errors. Both services were stopped by the diagnostic monitor at 10:59:50-51, when head age exceeded 1 second.
- After stopping, both cameras were still enumerated USB3 and CAN feedback was fresh; arms remained disabled. A second reboot was not performed.
- Evidence: `log/camera-head-left-after-reseat-20260915.json` and `log/camera-usb-transfer-errors-after-reseat-20260915.log`.
- An externally powered USB3 hub has been requested to isolate camera supply from the board. This is a diagnostic step, not a confirmed fix. SDK/host-controller behavior is still an alternative explanation.
- The supplied SDK includes a single-process multi-device example; that capture path has been inspected but not yet run as a comparison.


## SDK comparison interrupted by another restart

- A bounded diagnostic adapted the vendor's single-process multi-device sequence, using the same SDK 1.4, CameraWorkingMode=2 and ImageMode=8 as the production drivers. Production camera services were stopped. No production source or transport settings were changed.
- GetParam confirmed that both cameras use native 1280x720 YUY2 at 30 FPS. Supported acquisition-rate range reported 15 to approximately 30 FPS. ROS output is resized later to 640x360.
- Both SDK handles opened successfully and returned images for approximately 2 seconds. The tool output observed frame counts 22/29 and raw frame size 1,843,200 bytes. The diagnostic did not reach its completion or cleanup reports.
- SSH disconnected; after reconnection the computer reported a new boot time of 11:03:25. The agent did not issue this reboot. Operator clarification was requested to distinguish a manual restart from an unexpected reset; no answer had arrived when this section was written.
- The on-disk probe log retained parameter results and a NUL-filled tail after the restart, so it is not a complete test record. `journalctl -b -1` returned an older archived September 8 boot, not the immediately preceding test; pstore was empty. Neither provides the reset cause.
- Following recovery, all 16 arm joints again reported fresh valid feedback with motors disabled. All camera and program services were inactive. No SDK probe remained running. No real episode has been created.
- Current conclusion: head and left cameras work individually; concurrent acquisition remains unreliable. Full dataset collection has not been validated. Power/converter capacity, cables and USB-controller/SDK compatibility remain to be separated before another concurrent test.


## Confirmed automatic kernel panic

The operator confirmed no manual reboot at 11:03:25. `hrut_resetreason` reports PANIC, and archived records in `/var/lib/systemd/pstore/` show xHCI transfer errors followed by a fatal kernel exception in the `proc_do_submiturb` allocation path. This corrects the earlier incomplete pstore inspection. Detailed evidence and current conclusions: `docs/Camera_USB_kernel_panic_20260915.md`. Cameras remain stopped; arms remain disabled with fresh valid feedback. No real recording has been validated.
