# RealSense recording and selectable LAN receiver

## Active configuration

- X5: `sunrise@192.168.3.179`, managed dashboard at `http://192.168.3.179:8000`.
- D455f `261043062235`: head candidate; D435i `254843065015`: provisional left;
  D435i `261443062039`: provisional right. None is mounted yet.
- The C++ V4L2 node discovers the RGB interface by USB serial rather than a fixed
  `/dev/videoN`. It captures native 1280×720 YUYV at 30 FPS and publishes 640×360
  JPEG at quality 90. Depth is not recorded. No RealSense SDK is required for
  this RGB implementation. Hikrobot remains an explicitly selectable backend.
- Per-frame timestamps are V4L2 host-monotonic timestamps when available,
  otherwise host receive time, with the quality field distinguishing them.
  Device timestamp is zero. Cameras are not hardware-synchronized.
- Camera metadata is saved as `camera_config.json`; the raw schema remains
  `luxi_arms_v1` with right-eight then left-eight state/action vectors. These are
  raw ROS bags plus aligned JSONL samples, not a converted LeRobot dataset.

## Recording/storage workflow

Use the dashboard's managed services; do not launch a second competing stack.
`run_web.sh` and the program branch of `run_managed.sh` source `collection_env.sh`.
The active profile is `Project/robot_bringup/config/collection.env`:

```sh
COLLECTION_DIRECTORY=/run/luxi_dataset/episodes
COLLECTION_MIN_FREE_GIB=0.25
COLLECTION_RAM=true
```

`run-luxi_dataset.mount` supplies a 2 GiB tmpfs owned by sunrise. Web/program
services require the mount. The launch guard refuses a missing RAM filesystem
or enabled swap. OS/service logs can still write to the SD card. Existing
recordings in `data/real` are preserved, but the active RAM queue does not list
that old directory.

On the selected receiving PC, run the standalone receiver with the board URL
and a local destination folder. No receiver IP is configured on the X5:

```sh
episode_receive http://192.168.3.179:8000 /chosen/dataset/folder --release-source
```

The receiving PC initiates outbound HTTP. It claims one queue lease, renews it,
polls completed episodes every two seconds, resumes partial files with Range,
checks every file's length/SHA-256, fsyncs all files/directories and atomically
commits the final directory. It then acknowledges the exact manifest. Only that
acknowledgement permits X5 RAM cleanup. Interrupted/corrupt transfers retain the
source; incomplete destinations remain `.partial-*`. A second active receiver
is rejected. Stop the old receiver before choosing a new PC; crashed leases
expire after 20 seconds. All completed labels are transferred, including failed
and cancelled episodes. Only `status=accepted` episodes with suitable QC and
correct physical camera roles should be considered for training.

The tested receiver on this Ubuntu x86_64 workstation is:

```sh
/home/robot/luxi-dataset-receiver/receive.sh \
  http://192.168.3.179:8000 /home/robot/datasets/luxi --release-source
```

Its process log/PID are in `/home/robot/luxi-dataset-receiver/receiver.log` and
`receiver.pid`. The process is active for this commissioning session, not
installed as a boot service. Rebuild/copy the receiver directly to another
chosen PC as described in `tool/receiver/README.md`; there is no public source
archive served by the dashboard. The GUI shows the active PC, destination,
progress, and last verified episode. The board's episode list is the pending
queue; transferred episodes are in the PC folder.

A LAN outage leaves completed episodes queued in RAM. Recording stops at the
free-space reserve. Transfer starts after an episode finishes, so an individual
episode must fit in RAM. Power loss/reboot before transfer loses queued data;
the verified PC copy persists. Automatic cleanup applies only to RAM mode.

## Verified results

- `luxi_device_adapters`, `luxi_data_recorder`, `luxi_web_ctrl` built on ARM64.
- Four storage/queue cases passed: durable hashes/manifests, ID validation,
  rejection of a wrong manifest acknowledgement/competing receiver, idempotent
  verified cleanup, and refusal to auto-delete persistent storage.
- Native receiver transport regression passed: resume from 262144 bytes,
  corrupted-response retry from zero, interrupted-transfer resume from 1048576
  bytes, zero-length files, atomic final commit, idempotent pull, traversal
  rejection, and acknowledgement only after verification.
- Dashboard HTTP regression passed (including device freshness, camera expiry
  and control service handling). A headless Chromium render showed 3/3 cameras,
  2.0 GiB RAM, the selected PC/directory, and the released diagnostic receipt.
- Full isolated mock pipeline passed: Quest UDP -> mapping -> C++ IK -> mock
  arms -> three cameras -> bag/aligned samples -> Range/resumed verified pull.
  Loss/reconnect/pause and base recording checks passed. Evidence run:
  `build/test_runs/20260915T163933-41157`.
- Real read-only camera bag: `camera-diagnostic-20260915-163704`, 22-second
  bounded capture, 18.766992-second common measurement window:

| Stream | Total stored frames | FPS in common window | Maximum interval |
| --- | ---: | ---: | ---: |
| Head RGB | 594 | 29.9995 | 33.859 ms |
| Provisional left RGB | 593 | 29.8929 | 66.752 ms |
| Provisional right RGB | 594 | 29.8929 | 66.662 ms |

There were no missing publisher sequence numbers in the stored common window.
The wrist 66 ms gaps still show that physical capture is not perfectly lossless
30 FPS. All 1,981 stored arm-state messages were valid and disabled; no guard
errors occurred. The diagnostic is marked `status=diagnostic`, `samples=0`,
`training_episode=false`; it is not evidence of aligned action samples.

Five files totaling 83,814,002 bytes transferred automatically to
`/home/robot/datasets/luxi/camera-diagnostic-20260915-163704`; an independent
workstation read verified every size and SHA-256. The queue acknowledged the
manifest and released the X5 RAM copy. Small diagnostic evidence remains at
`log/camera-diagnostic-20260915-163704-report.json` on X5.

## Operator teleop recording passed

The operator pressed A to start and A again to finish the second arm cycle.
The first cycle had tested arm motion without starting a recording. The completed
real episode is `episode-1789461980863601370`, with `status=accepted`, `qc=[]`,
220 aligned samples over 7.819960107 seconds, and six files totaling 44,374,684
bytes. It is saved on the chosen PC at
`/home/robot/datasets/luxi/episode-1789461980863601370`.

Stored camera counts were head 236, provisional left 236 and provisional right
235. All 220 rows were decoded and checked against the actual ROS bag: the
16 state/velocity/action values match their referenced messages, applied commands
are in teleop, and all three image IDs/timestamps/quality fields exist and match.
The greatest state age was 10.809734 ms; the greatest image age was 94.257041 ms.
Every file's size and SHA-256 matched the manifest on the received PC copy.
The automatic receiver acknowledged that exact manifest and the X5 released
its original RAM episode. A temporary RAM copy of the PC files was used only
for validation with the canonical ROS decoder.

Both arm joint trajectories vary in the clip. Gripper action entries remain
constant (-0.4 right, +0.4 left), so this clip verifies their recorded fields,
not a new open/close demonstration. The bag's base topics contain real
`mock=false` feedback and commands; they are not messages from the isolated
mock test. The saved camera configuration records the unconfirmed wrist mapping.
Mount and identify the cameras before collecting training demonstrations.

Validation report: `log/realsense-real-episode-1789461980863601370-validation.json`
on X5, and `/home/robot/luxi-dataset-receiver/real-episode-validation.json` on the
PC. At completion the arms are disabled, recorder idle and receiver watching for
new episodes. No motor-enable or arm motion command was sent by the agent during
this work; the operator controlled all physical motion. The operator had already
finished the successful A/A episode before the optional remote recorder helper
could act, and that helper sent no recording command.

## Post-test hardware interruption

At 16:47:11 the kernel reported `xHCI host controller not responding, assume dead`
and `HC died; cleaning up` for `xhci-hcd.2.auto`. The USB2/USB3 hub branches,
three cameras and KCAN adapter then disappeared together. Camera processes had
not been restarted (`NRestarts=0`, started at 16:22:31). CAN drivers exited and
retried because the interfaces were absent. The X5 itself did not reboot.

The accepted episode and verified PC copy above predate this event and remain
valid. At the subsequent check, control was disabled and recorder idle, but arm
feedback and all camera frames were stale. Thus the short functional workflow
passed; sustained USB/hardware readiness is not established. It is not currently
ready for another real recording. The operator confirmed that connections and power were left untouched, so this
was spontaneous rather than a deliberate unplug. Evidence is
`log/realsense-post-test-usb-failure-20260915.json`.

## USB recovery and next controlled test

At 16:57:04, with the controller disabled and recorder idle, the agent stopped the
program, base, arm monitor and camera services, then unbound/rebound only the
already-dead `xhci-hcd.2.auto` platform host. The root filesystem is MMC-backed,
and the dashboard/Wi-Fi/PC receiver remained running. The X5 did not reboot.
The same presence probe changed from missing all four serials to detecting all
three RealSense cameras at 5000 Mbps and KCAN at 480 Mbps. Restarting only the
arm monitor restored fresh disabled feedback from all 16 joints. This is a
successful recovery, not proof that the root cause has been fixed.

The X5 is powered by a battery converter with unknown rating. D-Robotics
specifies 5 V / 5 A at the board input. No readable 5 V USB rail voltage/current
sensor was found; the exposed PVT sensors cannot confirm USB supply health.
A known adequate supply is requested before the next sustained test. Official
references:
- https://d-robotics.github.io/rdk_x_doc/en/Quick_start/hardware_introduction/rdk_x5/
- https://dev.realsenseai.com/docs/multiple-depth-cameras-configuration/

The managed RealSense launcher and metadata now prepare native 640×360 YUYV at
30 FPS, instead of 1280×720 followed by software downsampling. All three devices
advertise this mode. Published JPEG dimensions remain 640×360. The nominal raw
image payload falls from 165.888 MB/s to 41.472 MB/s for all three cameras,
excluding USB overhead. This profile has NOT yet been streamed/stability-tested;
`camera_config.json` marks that explicitly. Existing verified episode metadata
retains the original 1280×720 capture settings.

Camera, base and teleop/recording services remain stopped; the arm monitor,
dashboard, Quest receiver and PC dataset receiver remain running. Arms are
confirmed disabled. Do not interpret restored enumeration as sustained USB
stability. The next check needs a known 5 V / 5 A supply, fixed cables, the new
native profile, a bounded recording/stream test, and a longer soak exceeding the
previous approximately 25-minute run before drawing a stability conclusion.
The USB host power setting was already `on`; idle camera/hub devices use `auto`.
No power-management setting or kernel version was changed, and no automatic USB
reset or automatic motor re-enable was added.
