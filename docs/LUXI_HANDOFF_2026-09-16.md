# LUXI teleoperation and data collection handoff

Prepared: 2026-09-16. Live status snapshot: approximately 14:05 CST (UTC+8).

## 1. Current state and immediate priority

The operator has completed the teleoperation and recording workflow on the X5
and considers the current setup acceptable for now. The three RealSense cameras
and USB-to-CAN adapter are connected through one hub to the X5. Use this host for
the current session; the Raspberry Pi base CAN fault remains unresolved.

**One completed episode is still in X5 RAM. No receiver PC was connected at the
snapshot. Transfer it before rebooting or powering off the X5.**

| Item | Observed state |
| --- | --- |
| Dashboard | Available at `http://192.168.3.179:8000` |
| Arm driver | Running; all 16 joints fresh and valid; motors disabled |
| Base driver | Running; feedback fresh and valid; permission disabled; velocity zero |
| Teleop/recording program | Running; control disabled; recorder idle |
| Cameras | All three streaming, fresh, approximately 29.98 FPS |
| Quest receiver | Running; sender currently inactive/stale, 0 Hz |
| LAN dataset receiver | Offline; no selected receiving PC |
| RAM buffer | 2 GiB total; approximately 1.94 GiB free |
| CAN statistics | Zero RX/TX errors and dropped packets on all three active channels |

The base decoder had two cumulative rejected groups. Their cause/timing was not
investigated during this documentation task; this count is distinct from the
zero SocketCAN error counters. Camera source-gap counters in this running
session were zero at the snapshot. These observations are a current-state read,
not a new sustained hardware test.

## 2. Hosts, repository and hardware

| Host | SSH account | Software | Role |
| --- | --- | --- | --- |
| X5 | `sunrise@192.168.3.179` | Ubuntu 22.04 ARM64, ROS 2 Humble, kernel 6.1.83 | Current robot host |
| Raspberry Pi | `raspberrypi-fxx@192.168.4.13` | Debian 13 ARM64, ROS 2 Jazzy, kernel 6.12.75+rpt-rpi-2712 | Port tested previously; base CAN fault unresolved |
| This workstation | Local account `robot` | Ubuntu x86_64 | Existing standalone receiver and archived diagnostics |

Passwords are intentionally omitted. Use the credentials supplied by the owner.
Addresses are the current known LAN addresses, not guaranteed permanent leases.

X5 repository:

```text
/home/sunrise/Desktop/LUXI_Lift_Robot/luxi_lift_ros2/lift_robot_ros2
```

Pi repository uses the same suffix under `/home/raspberrypi-fxx`.

- X5 branch: `master`, commit `f102c71` — `feat: stabilize Quest teleop and add RealSense LAN recording`.
- Configured remote: `https://gitee.com/fxx360724/lift_robot_ros2.git`; that commit was pushed with the owner's authorization.
- Before this document, X5 tracked changes were three generated Hikrobot SDK log files. No production source fix was made during the latest dashboard repair.
- Last confirmed Pi branch/commit: `pi-jazzy`, `a694724` — `feat: support Raspberry Pi Jazzy deployment`; not pushed at the last check. This Pi revision was not refreshed for this handoff.
- The dashboard startup repair lives in `/etc/systemd/system`, outside Git; preserve it separately when reinstalling.

### Camera assignment

| Logical role | Model | Serial |
| --- | --- | --- |
| `head_rgb` | RealSense D455f | `261043062235` |
| `left_wrist_rgb` | RealSense D435i | `254843065015` |
| `right_wrist_rgb` | RealSense D435i | `261443062039` |

These are configured roles. Physical mounting and wrist-side identification were
previously unconfirmed; no subsequent mounting/calibration confirmation has been
recorded. Confirm them before collecting a training dataset.

Current capture: native 640×360 YUYV at nominal 30 FPS, published as JPEG at
quality 90. RGB only; depth is not recorded. Cameras use host-monotonic V4L2
timestamps and are not hardware-synchronized. The C++ backend uses V4L2 and does
not require librealsense. Device selection is by serial, not `/dev/videoN`.

### CAN assignment

Adapter: KH/KCAN USB X4, VID:PID `395e:0020`, serial `KCANX4WCID001`.

| Connector / zero-based channel | Interface | Device | Bus settings |
| --- | --- | --- | --- |
| CH1 / 0 | `luxi_left` | Left arm and gripper | CAN FD, 1 Mbps nominal / 5 Mbps data |
| CH2 / 1 | `luxi_right` | Right arm and gripper | CAN FD, 1 Mbps nominal / 5 Mbps data |
| CH3 / 2 | `luxi_base` | R680 mobile base | Classic CAN, 1 Mbps |
| CH4 / 3 | `luxi_lift` | Reserved; no lift connected | Disabled |

Never infer connector identity from transient names such as `can1` or `can2`.
The X5 also has a separate onboard CAN interface. The latest hub attachment used
USB paths `1-1.4` and `2-1.4`; the CAN adapter was at `1-1.4.1` and cameras under
`2-1.4.2` through `.4`.

## 3. Start the complete X5 session

Run on the X5. These commands launch acquisition and control software; they do
not request motor enable. Do not launch a second teleop stack alongside them.

```bash
sudo systemctl start luxi-web.service luxi-quest.service \
  luxi-openarm-monitor.service luxi-base.service luxi-program.service \
  luxi-rgb-head_rgb.service \
  luxi-rgb-left_wrist_rgb.service \
  luxi-rgb-right_wrist_rgb.service
```

Open `http://192.168.3.179:8000`. Set the Quest sender destination to
`192.168.3.179:5006`. A receiver for the headset must run on the same robot host
whose dashboard is being used.

The earlier five-service command omitted all cameras. Starting the dashboard
does not by itself start the camera streams or select a PC to save datasets.

| Service | Purpose |
| --- | --- |
| `luxi-web` | Dashboard and dataset HTTP endpoints |
| `luxi-quest` | Quest UDP receiver |
| `luxi-openarm-monitor` | Normal real arm driver; historical name, **not** the diagnostic `monitor_only` mode |
| `luxi-base` | Real R680 driver with independently gated base control |
| `luxi-program` | Mapper, IK, arm control manager and recorder |
| Three `luxi-rgb-*` units | Serial-selected RealSense streams |
| `run-luxi_dataset.mount` | Bounded RAM staging filesystem |

Avoid `tool/start_robot.sh --hardware` and the generic `tool/run.sh` for this
session: their profiles differ from the separately managed RealSense/RAM setup.

### Operator workflow

1. Start the services and, preferably, the chosen PC's dataset receiver.
2. Check fresh arm/base feedback and all three camera previews. Wear the headset,
   keep the sender in the foreground and wake both controllers.
3. With the arm home path clear and stop access available, hold **X for 1.5 s**,
   then release. During alignment, hold the controllers steady and fully squeeze
   both index triggers. Wait for `teleop` before demonstrating.
4. Leave the base disabled unless base motion is needed. Center both sticks and
   press **Y** to enable it. Left stick controls translation; right-stick
   horizontal movement controls rotation. Centering the sticks stops motion.
5. Press **A** or click `开始录制` to start an episode. Confirm the recorder says
   recording. Perform the demonstration.
6. Press **A** again or click `成功并保存` to finish successfully. Wait for the
   recorder to return to idle and, when a receiver is running, for transfer
   verification to finish.
7. Press **X once** for arm return/home followed by disable. Disable the base
   separately. Confirm disabled states before leaving or stopping services.

For an orderly software shutdown, first finish/save recording, complete transfer
of queued RAM episodes, and disable/support the robot as appropriate. Then:

```bash
sudo systemctl stop luxi-program.service luxi-base.service \
  luxi-openarm-monitor.service \
  luxi-rgb-head_rgb.service \
  luxi-rgb-left_wrist_rgb.service \
  luxi-rgb-right_wrist_rgb.service \
  luxi-web.service luxi-quest.service
```

These service commands are not a physical emergency stop. Do not stop/unmount
`run-luxi_dataset.mount`, reboot, or remove power with untransferred recordings.

## 4. Exact Quest button behavior

The current installed source was inspected for this handoff. **B is not an
all-system enable/disable toggle.**

| Input | Implemented behavior |
| --- | --- |
| X held ~1.5 s while arms disabled | Enable arms and begin home/alignment, subject to readiness checks |
| X short press while arms active | Request return/home, then disable; does not change base permission |
| Y press | Toggle base permission; enabling requires centered sticks, fresh usable input/feedback and no active stop/failure-menu interlock |
| B press | Always stops/disables the base; never enables it |
| B while recording | Opens failure-reason menu and requests arm return/home followed by disable |
| B short press while not recording | Does not request arm stop |
| B held ~1.5 s while not recording | Requests arm return and emits the session-exit request after fresh disabled feedback |
| A | Start recording, or finish successfully; in the failure menu, confirm failure instead |
| Left stick vertical while failure menu is open | Select failure reason |
| Index triggers | Respective grippers; also used during alignment |
| Middle-finger grips | Reserved for future lift: left down, right up; release or both pressed means stop |

There is no connected lift and no verified live lift control path. The grip
assignment is an agreed future mapping. Base driving no longer requires holding
the left grip. Recording cancellation remains a dashboard/episode-service action.

Arm return is motion, not an immediate stationary stop. For unexpected motion,
use the physical stop appropriate to the robot; do not assume a short B press
outside recording disables both arms.

## 5. Recorded data and the episode waiting now

Active configuration: `Project/robot_bringup/config/collection.env`:

```bash
COLLECTION_DIRECTORY=/run/luxi_dataset/episodes
COLLECTION_MIN_FREE_GIB=0.25
COLLECTION_RAM=true
```

The mount is 2 GiB. Episodes are staged in RAM to avoid writing their bulk data
to the SD card; OS/service logs can still use the SD card. Recording stops at
the free-space reserve. An individual unfinished episode must fit in the buffer,
because automatic transfer starts only after it is finalized.

Current pending episode:

```text
/run/luxi_dataset/episodes/episode-1789538003142448477
```

- Manifest: `status=accepted`, `qc=[]`, 322 aligned samples.
- Each camera: 346 recorded image messages.
- Manifest-listed files total 62,712,514 bytes, approximately 59.8 MiB.
- Recorder is idle; the episode is finalized, not an in-progress `.partial-*`.
- Manifest and metadata were read for this handoff. No independent hash, SQLite,
  image-decode or sample-reference validation was run on this latest episode.
- It had not been transferred at the snapshot. No data transfer/deletion was
  performed as part of writing this document.

Copy the entire episode directory:

```text
episode-1789538003142448477/
  manifest.json
  samples.jsonl
  session.json
  camera_config.json
  robot_config.json
  bag/
    metadata.yaml
    bag_0.db3
```

`samples.jsonl` contains `observation.state`, `observation.velocity`, `action`,
sample/command IDs, timestamps and image references. The actual images are
stored as ROS messages inside the SQLite ROS bag, not as standalone JPEG files.
The raw arm schema is `luxi_arms_v1`, ordered right eight then left eight
(seven arm joints plus gripper per side). Base feedback, commands and raw CAN
are separate bag topics; they do not extend the aligned 16-dimensional arm
vector. No absent lift state is fabricated. This is not yet a converted LeRobot
dataset; training conversion and camera-role/calibration review are separate.

## 6. Save on any chosen LAN PC

The receiving PC initiates the connection. No fixed receiver IP needs to be
configured on the X5. Merely opening the dashboard on a PC does not make it the
receiver. Keep the receiving terminal running during collection.

### Existing receiver on this workstation

The Ubuntu x86_64 workstation already has a standalone receiver:

```bash
/home/robot/luxi-dataset-receiver/receive.sh \
  http://192.168.3.179:8000 \
  /home/robot/datasets/luxi \
  --release-source
```

It was not running/connected at the snapshot. This command is provided for the
operator; it was not launched during the handoff task.

### One-time setup on another Ubuntu/Debian PC or Windows WSL

Run on the destination PC. Copy only the receiver source folders; no ROS or
camera SDK is required there. Windows uses WSL; native Windows support is not
provided by this POSIX receiver. macOS is documented but not hardware-tested.

```bash
mkdir -p ~/luxi-receiver/tool ~/luxi-receiver/Project

scp -r \
  sunrise@192.168.3.179:/home/sunrise/Desktop/LUXI_Lift_Robot/luxi_lift_ros2/lift_robot_ros2/tool/receiver \
  ~/luxi-receiver/tool/

scp -r \
  sunrise@192.168.3.179:/home/sunrise/Desktop/LUXI_Lift_Robot/luxi_lift_ros2/lift_robot_ros2/Project/data_recorder \
  ~/luxi-receiver/Project/

sudo apt install g++ pkg-config libcpp-httplib-dev libssl-dev nlohmann-json3-dev

cd ~/luxi-receiver
bash tool/receiver/build.sh
```

Then run for each collection session:

```bash
~/luxi-receiver/build/receiver/episode_receive \
  http://192.168.3.179:8000 \
  "$HOME/datasets/luxi" \
  --release-source
```

Replace the destination with any writable disk/folder on that PC. For Windows
storage from WSL, use an appropriate `/mnt/c/...` or other mounted-drive path.
Quote paths containing spaces. Build on the destination architecture; do not
copy the X5 ARM64 executable to an x86_64 PC.

The receiver picks up already queued episodes and future completed ones. It:

1. Claims the single active receiver lease.
2. Downloads into `.partial-episode-*`, resuming interrupted files.
3. Checks each manifest-listed file's size and SHA-256.
4. Flushes files/directories to disk and atomically commits the final folder.
5. Acknowledges the exact manifest. With `--release-source`, only then does the
   X5 remove that episode's RAM copy.

Expected terminal completion:

```text
Verified .../episode-1789538003142448477 (RAM copy released)
```

The dashboard shows receiver identity/destination and the last verified episode.
An episode disappearing from the X5 queue after verified transfer is expected;
its persistent copy is on the receiving PC. Checksums establish transfer
integrity, not demonstration quality.

To switch PCs, stop the old receiver with Ctrl+C and run the receiver on the new
PC. Only one can own the queue. After an unclean disconnect, the lease expires
after 20 seconds. Previously transferred datasets stay on the original PC;
switching receivers does not migrate them. Failed/cancelled episodes are also
transferred with their labels preserved.

If the PC/network is unavailable, completed episodes remain in X5 RAM. Do not
reboot before recovering them. Omitting `--release-source` retains the X5 copy
after successful transfer, but it remains volatile and consumes buffer space.

### One-off SSH copy

For an immediate copy of the currently queued episode, run on the destination:

```bash
mkdir -p ~/datasets/luxi
scp -r \
  sunrise@192.168.3.179:/run/luxi_dataset/episodes/episode-1789538003142448477 \
  ~/datasets/luxi/
```

This does not release RAM or perform the application's manifest-verification
and acknowledgement workflow. Do not manually delete the only source copy
before verifying the destination. The automatic receiver is the intended
ongoing workflow.

## 7. Dashboard startup repair to preserve

After the user started teleop, the dashboard and program entered restart loops:

```text
Dataset RAM mount unavailable: /run/luxi_dataset/episodes
```

The parent RAM filesystem was correctly mounted, but `episodes/` did not exist.
`collection_env.sh` called `findmnt -T` on that missing directory before its
later `mkdir -p`, so both launchers exited before starting.

Both existing systemd drop-ins were updated to create the child directory after
the mount dependency is satisfied:

```text
/etc/systemd/system/luxi-web.service.d/collection-ram.conf
/etc/systemd/system/luxi-program.service.d/collection-ram.conf
```

Each now contains:

```ini
[Unit]
Requires=run-luxi_dataset.mount
After=run-luxi_dataset.mount

[Service]
ExecStartPre=/usr/bin/mkdir -p /run/luxi_dataset/episodes
```

Timestamped `.bak-*` files preserve the previous drop-ins. Systemd was reloaded
and both services started. The operator subsequently confirmed the dashboard
opened. Camera services were then started separately, and the operator completed
the recording workflow. The original RAM filesystem/swap safeguards remain.

For a new installation, recreate these drop-ins alongside the mount unit; they
are not part of commit `f102c71`. Do not work around the error by silently
redirecting recordings to the SD card.

## 8. Evidence and limitations

### X5 five-minute simultaneous camera/CAN test

The entire hub was moved from Pi to X5. The test ran for 300.002 s with real arm
`monitor_only:=true`, base `receive_only:=true`, all three camera streams, and no
motion controller or Quest input. The current X5 USB port differed from the
earlier base-only comparison port; retain that distinction.

| Measurement | Result |
| --- | --- |
| Head camera | 8,996 frames, 29.986 FPS |
| Left wrist camera | 8,993 frames, 29.976 FPS |
| Right wrist camera | 8,994 frames, 29.980 FPS |
| JPEG decoding | All 26,983 received images decoded |
| Arms | 30,001 state messages, 100.003 Hz, all 16 joints valid |
| Base | 6,000 valid groups, 20.000 Hz, zero velocity |
| CAN errors/drops | Zero new RX/TX errors or drops on all three channels |
| USB/kernel | No disconnect, re-enumeration, new kernel message or reboot |

The diagnostic's strict overall `passed` flag was **false**: cumulative camera
source-gap counters were head=2, left=1, right=1. Those counters include startup,
and their baseline was not captured. No published message sequence gaps occurred
in the measured interval. Do not call this a perfectly lossless run or a proof
of long-term stability. The test did not exercise motor motion or recording.

Evidence remains on X5:

```text
~/camera_diagnostics/x5_combined_test/combined-io-20260916T115900.*
```

### Pi port and base fault

- Operator confirmed Pi arm/gripper teleop worked, including with all three cameras streaming.
- A previous Pi camera/arm test sustained approximately 30 FPS per camera and 100 Hz valid arm feedback.
- A real Pi episode, `episode-1789526615079766818`, was transferred and independently verified under `/home/robot/datasets/luxi-pi/` on the workstation. It had 362 aligned samples and 1,153 images; its RAM source was released after verification.
- Pi CH3/base produced intermittent stuffing/form receive errors even with cameras off, arm channels down and no application command frames. Matching the X5 nominal SJW did not eliminate the fault.
- In a matching 120 s receive-only comparison, X5 had 2,400 valid groups and zero errors; Pi had 2,396 valid groups, four rejected groups and four receive errors.
- Pi usbmon captured an adapter firmware error report before its corresponding SocketCAN error. This rules out the error being invented by ROS or the CAN error translation; it does not isolate power/reference, wiring, firmware/configuration or host interactions.
- Do not weaken the base error gate or tracking watchdog to mask the fault.

The owner's statements about CAN ground wiring were uncertain and contradictory.
Missing CAN_GND is **not proven**; no corrective wire change was confirmed. Leave
existing wiring unchanged until marked terminals and end-to-end wiring can be
identified. Do not add a guessed ground connection.

### Remaining limits

- The X5 previously suffered spontaneous xHCI failure affecting cameras and CAN together. The latest bounded test did not reproduce it; the root cause is not established.
- X5 battery/converter rating remains unknown. Pi supply was last reported as 5 V / 4 A; that was not rechecked for this handoff.
- Physical camera roles, mounting and calibration require confirmation before training collection.
- Lift hardware/control is not commissioned.
- The latest accepted X5 episode has not been independently validated or transferred. Earlier verified episodes do not validate this new one automatically.
- Older docs contain historical status snapshots and superseded camera settings; use the dated current configuration in this handoff, then inspect live state before acting.

## 9. Troubleshooting and next operator actions

Suggested next actions, without automatically launching new tests:

1. Start the receiver on the selected PC and preserve the currently queued episode.
2. Confirm that the PC reports `Verified` and contains the complete episode.
3. Keep the current X5 hub arrangement for ongoing operation; avoid changing host,
   power supply and cables simultaneously when investigating a recurrence.
4. Confirm camera mounting/side labels before recording training demonstrations.
5. Revisit the Pi base issue or perform a longer X5 soak only as a separately
   requested diagnostic task.

Useful read-only commands on X5 when troubleshooting is requested:

```bash
systemctl --no-pager status luxi-web.service luxi-program.service
sudo journalctl -u luxi-web.service -u luxi-program.service -n 60 --no-pager
findmnt /run/luxi_dataset
ls -ld /run/luxi_dataset /run/luxi_dataset/episodes
lsusb -t
ip -details -statistics link show type can
curl --noproxy '*' -s http://127.0.0.1:8000/api/state
curl --noproxy '*' -s http://127.0.0.1:8000/api/receiver
curl --noproxy '*' -s http://127.0.0.1:8000/api/episodes
```

`/api/state` is the status endpoint; `/api/status` is not. If USB devices vanish,
inspect kernel/service logs and motor state before restarting or rebooting. A
reboot loses any episode still in RAM.

The user requires explicit authorization in the specific task before running
post-change tests, builds, linting or other validation. This handoff task used
read-only inspection before writing; no new tests/builds, motion, recording,
dataset transfers, service restarts or commits were performed.

## 10. Files for the next engineer

On X5, repository-relative:

- `docs/LUXI_HANDOFF_2026-09-16.md` — this handoff.
- `Project/luxi_web_ctrl/scripts/run_managed.sh` — arm/base/program launchers.
- `Project/luxi_web_ctrl/scripts/run_rgb_camera.sh` — current serial-selected camera launcher.
- `Project/luxi_web_ctrl/scripts/collection_env.sh` and `Project/robot_bringup/config/collection.env` — RAM recording profile.
- `Project/robot_bringup/config/arms-teleop.json`, `arms-kh.json`, `cameras-realsense.json` — teleop, CAN and camera configuration.
- `Project/control_manager/src/control_manager.cpp` — arm enable/return state machine.
- `Project/device_adapters/src/r680_adapter.cpp` — base permission, joystick mapping and CAN fault handling.
- `Project/data_recorder/src/data_recorder.cpp` — episode state machine and A/B behavior.
- `Project/data_recorder/src/episode_receive.cpp`, `include/luxi/transfer.hpp`, `include/luxi/receiver_queue.hpp` — receiver and verified cleanup.
- `tool/receiver/README.md` — standalone receiver setup.
- `docs/Quest_button_mapping_validation_20260915.md` — detailed button behavior and earlier software tests.
- `docs/RealSense_LAN_collection_20260915.md` — earlier collection/transfer results and USB failure history; dated snapshots, not current status.

On this workstation:

- `/home/robot/luxi-pi-diagnostics/LUXI_HANDOFF_2026-09-16.md` — local copy of this handoff.
- `/home/robot/luxi-pi-diagnostics/RASPBERRY_PI.md` — detailed migration and CAN/USB diagnostic history.
- `/home/robot/luxi-pi-diagnostics/x5_combined_io_check.py` — bounded diagnostic source, not a normal teleop launcher.
- `/home/robot/luxi-dataset-receiver/receive.sh` — existing x86_64 receiver launcher.
- `/home/robot/datasets/luxi/` and `/home/robot/datasets/luxi-pi/` — prior persistent episode archives.

Raw Pi USB error evidence remains at
`~/camera_diagnostics/realsense_lan_test/base-usb-error-origin.json` on Pi.
Prior automatic approval review declined exporting that raw telemetry and an
internal R680 protocol document; neither should be copied by an alternate route
without explicit authorization. Reading and summarizing their diagnostic findings
was sufficient for this handoff.
