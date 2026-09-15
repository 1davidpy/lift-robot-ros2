# Dual-arm and R680 teleoperation and collection

Repository on the robot:
`/home/sunrise/Desktop/LUXI_Lift_Robot/luxi_lift_ros2/lift_robot_ros2`

Use the existing managed dashboard workflow. Its device services and control/recording program are separate. Do not also launch `tool/run.sh`, `start_robot.sh`, the old Dora stack, or `luxi-robot.service`: these are alternative launch paths and can compete for the same devices or UDP port.

## RealSense and LAN storage — 2026-09-15

The managed dashboard now uses the three RealSense RGB cameras and stages real
recordings in a bounded 2 GiB RAM filesystem. A standalone C++ receiver on the
chosen LAN PC automatically downloads each committed episode, resumes interrupted
files, verifies every SHA-256, fsyncs and commits the local directory, then
acknowledges the exact manifest. With `--release-source`, the X5 frees that RAM
copy only after acknowledgement. It refuses automatic cleanup for disk profiles.
See [implementation and measured results](RealSense_LAN_collection_20260915.md)
and [receiver setup](../tool/receiver/README.md).

Current camera roles are provisional: D455f `261043062235` is the head candidate;
D435i `254843065015` and `261443062039` are labelled left/right respectively.
The operator confirmed that none is mounted yet. Serial numbers are visible on
the previews. After mounting, cover the RGB lens on the robot's left wrist to
identify it, then confirm/update the service serial assignments and
`Project/robot_bringup/config/cameras-realsense.json`. Its
`wrist_mapping_confirmed` remains false until that physical check is complete.

Real-camera recording and automatic LAN transfer passed with motors disabled.
The full mock pipeline and an operator-controlled real teleop episode then passed.
`episode-1789461980863601370` contains 220 validated aligned samples over 7.82 s;
all six files were verified on the PC and its original X5 RAM copy was released.
All indexed state/action/image references match messages in the bag. Wrist
mounting is unconfirmed, and gripper commands stayed constant in this short test.
After that successful recording, the shared USB host died at 16:47:11 and all
cameras/CAN disappeared. The PC data remains verified, but another hardware
recording was blocked. A targeted USB host reset recovered all four devices
and fresh disabled arm feedback. Camera/control streams now remain stopped
pending a power-controlled stability test with a prepared lower-traffic native
640×360/30 FPS profile; see the linked report for evidence.
The older Hikrobot failures below are historical and do not describe the new
active RealSense services.

## X/Y mapping source update — 2026-09-15

Short Y now toggles base permission in source: enable when ready and sticks are centered, or stop/disable when already enabled. Holding Y performs only one toggle. Startup/reconnection requires a released Y before a new press can enable; a rejected enable is not retried while held. B retains stop priority. X is exclusive to arm control and does not enable, stop or disable the base. Y no longer cancels recording; use the dashboard's **取消本次并保留数据** instead. The base adapter, recorder and dashboard have now built and passed isolated software checks; see [button mapping validation](Quest_button_mapping_validation_20260915.md). Live services were not restarted, so the new mapping takes effect on their next start. The September 14 hardware results below apply to the previous button mapping.

## Verified on 2026-09-14

- `tool/test.sh`: 33 tests, zero errors, zero failures, zero skipped, across 11 packages, after the Quest receiver fix below.
- A stale compiled IK test referenced `device/external/openarm_mujoco`. Current source already uses `device/openarm/openarm_mujoco`. Rebuilding `luxi_arm_motion` fixed the test without a source change.
- The complete isolated mock pipeline produced accepted episode `episode-1789366537310870865`: 73 aligned samples, 78 frames per camera, and `qc=[]`.
- The post-fix rerun produced accepted mock episode `episode-1789368417974210909`: 74 aligned samples, 78 frames per camera, and `qc=[]`, with a verified downloaded copy.
- The test checked 16-dimensional state/action vectors, observation timestamps preceding commands, image references actually present in the bag, base streams, HTTP byte ranges, resumed transfer and SHA-256 hashes. It also checked input loss, no automatic resume, and return followed by disable.
- Real feedback was fresh from all 16 arm joints and the base. Base voltage was about 24.19 V, reported velocity was zero, and `flag_stop=0`. Both motion permissions were disabled.
- The real control service was initially running in `disabled` and the recorder in `idle`; both were paused after the USB incident below.
- With the headset sender active, real Quest input reached the robot at about 72 packets/second. A 15-second capture received 1,075 packets, including 415 with all tracking flags valid. Moving each controller produced roughly 10 cm of position change at the receiver. Intermittent STALE flags do not by themselves indicate a broken connection.
- The operator's X hold enabled both arms and moved them toward home. After adjusting the supervised arrival tolerance, the observed phases were homing, teleop, hold, returning and disabled. After the IK session fix, the operator confirmed both arms align and follow correctly and both index-trigger grippers open/close correctly. Two observed cycles at about 17:24 and 17:25 completed home, alignment, teleop, return and disable with all 16 joints fresh. The operator subsequently reported slow following; after the speed adjustment below, the operator confirmed it feels good.
- After the operator connected the cameras, all three serials enumerated and each camera produced images at some point, but simultaneous acquisition failed the live stability checks below. A real visual episode remains pending; the passing recording test used mocks.

Evidence is in `log/workflow-20260914/`: `software-tests-after-quest-fix.log`, `ik-before-rebuild.log`, `ik-rebuild.log`, `ik-after-rebuild.log`, `live-preflight.json`, `live-ready-state.json`, `quest-controller-motion.json`, and `mock-episode-summary.json`.

### Quest receiver compatibility fix

The real sender sometimes omits a controller pose while continuing to send timestamps and buttons. The C++ parser previously required both `lc` and `rc`, causing these packets to be discarded. It now accepts missing/null controller poses while marking that controller INVALID (2), preserving reception and button handling without exposing a usable motion target. Existing STALE (1) behavior is unchanged.

Three regression cases cover missing poses, null poses and ordinary STALE input. An isolated ROS domain with mock actuators additionally received 295 missing-pose packets and confirmed that neither held X nor a start service request enabled arms. The full mock recording/transfer suite passed after the fix. The real Quest service was restarted to load the rebuilt receiver; no arm or base enable command was sent.

Evidence: `quest-parser-before.log`, `quest-parser-after.log`, `quest-wire-check.log`, and `software-tests-after-quest-fix.log`.

### Live USB failure and recovery

At 14:21:14 the kernel reported `xHCI host controller not responding` and `HC died; cleaning up`. The shared USB hub, CAN adapter and cameras disappeared together. Motion permissions were disabled. Device/control services were stopped, the failed `xhci-hcd.2.auto` controller was rebound, and arm/base acquisition recovered without enabling motion.

The right-wrist camera then streamed successfully alone. All 30 observations had fresh arm/base feedback; 25 had fresh right-camera images, with the initial samples covering startup. Adding the left-wrist camera caused further USB failures: the right camera disconnected at about 14:29:42 and the left at 14:30:04, both following address/setup failures and USB error `-71`. Arm/base feedback remained available in that test. The wrist-camera services were stopped again.

The operator confirmed no cable disconnections during the test and that the camera hub has no external power input. This reproduces a USB-layer reliability problem; it does not establish whether the cause is power delivery, cables, or controller/SDK interaction. The next diagnostic is one camera connected directly to a USB 3 port, or a separately powered hub, while leaving the CAN adapter connected. The head camera negotiated 480 Mbit/s; wrist cameras negotiated 5000 Mbit/s through a second hub. Do not infer that lower preview resolution reduces the native USB stream bandwidth.

After the failure, camera acquisition and the normal control/recording program were paused. Cameras later re-enumerated intermittently, which does not establish reliable acquisition. Quest packet reception and controller pose changes were verified while the headset sender was active.

The operator subsequently requested testing teleop without cameras first. The temporary session described below is now used for that check. Camera stability remains a separate requirement for the complete visual recording workflow.

Additional evidence: `operator-readiness.jsonl`, `usb-controller-failure.log`, `right-camera-alone.json`, `two-wrist-cameras.json`, `two-camera-usb-failure.log`, and `quest-packet-check.log` in the same log directory. Short successful feedback windows do not establish long-term USB stability.

### Current camera-free supervised session

The temporary systemd unit `luxi-supervised-teleop.service` starts only the mapper, IK and control manager. The existing arm adapter and Quest receiver provide real feedback/input. Cameras and the recorder are not started. Base acquisition remains available with base motion permission disabled.

This session uses `log/workflow-20260914/arms-supervised.json`, copied from `arms-kh.json` with joint speed limits reduced to 0.2 rad/s for each arm joint and 0.5 rad/s for each gripper. The normal profile is unchanged. The configuration limits commanded target motion; it is not a guarantee of measured physical speed under all conditions.

The first real X-enable attempt moved both arms toward home with all 16 joints reporting valid feedback. Homing then timed out: right/left J4 settled about 0.088/0.090 rad short of their targets, outside the original 0.05 rad arrival threshold. The operator confirmed both arms were stationary in free space. A five-second read-only capture received 501 commands, 501 applied commands and 499 states; commands and applied targets matched home, while J4 reported approximately 5.23/5.39 Nm. This is consistent with a static position offset under load; it does not establish a need to change motor gains.

The control manager now accepts startup parameter `home_tolerance`, default 0.05 rad and limited to `(0, 0.1]`. The supervised launch is prepared with 0.10 rad. The isolated regression passed for the observed 0.09 rad error, return followed by disable, unchanged default behavior, larger-error rejection and invalid-parameter rejection. The complete mock capture/transfer integration also passed. Evidence: `homing-timeout-diagnosis.json`, `homing-regression-before.log`, `homing-regression-after.log`, and `integration-after-homing-fix.log`.

The operator confirmed both arms were supported, after which an authenticated disable request succeeded and fresh feedback confirmed all 16 joints disabled. The temporary control session was restarted, and live status confirmed `home_tolerance=0.1`, control disabled and healthy feedback. The next real run reached homing at 16:11:01, teleop at 16:11:07, hold at 16:11:15, returning at 16:12:36 and disabled at 16:12:41. The 10 Hz observer did not capture the brief aligning phase. This verifies the home/return tolerance change, but does not establish satisfactory controller following. Evidence: `disabled-before-controller-reload.json`, `controller-reloaded.json` and `supervised-motion.jsonl`.

#### Slow progress and tracking hold

The operator confirmed both index triggers were nearly fully squeezed. Two defects were reproduced independently: IK was reseeded from measured positions every update, losing accumulated progress under servo lag; one brief INVALID tracking report immediately latched hold. Ordinary STALE was already accepted and remains accepted.

IK now integrates from its previous commanded configuration after initialization from valid feedback, matching the original teleop graph's behavior. INVALID tracking freezes the current submitted target immediately. It may resume within the existing 150 ms freshness window only after tracking and a corresponding new IK result recover. Sustained loss still latches hold. The status now exposes `reason`, `input_paused`, `quest_age_ms` and `intent_age_ms`.

Both new regressions failed before the fixes and passed afterward. The IK limit/convergence test and home-tolerance regressions also passed. An isolated captured-packet replay with simulated stationary feedback measured about 30.30 Hz IK both before and after; maximum output gaps were 38.1/39.2 ms and maximum source age was below 25 ms. The old reference made no progress; the corrected reference advanced 0.897 rad across the run while respecting the supervised limits. No actuator nodes were connected to this comparison. Separately, live arm feedback and control status measured about 100 Hz; there was no active Quest input during that timing window. Evidence: `reference-after.log`, `tracking-gap-after.log`, `benchmark-before.json`, `benchmark-after.json` and `teleop-timing-diagnostic.json`.

These fixes were loaded at 16:33:57. The next attempt exposed a further `ik_timeout` while controller tracking was valid. A later attempt reached aligning at 16:35:12 and the operator requested return at 16:35:17. The supervised 0.2 rad/s arm cap remains in place; increased loop frequency is not a substitute for correcting lost IK progress.

Live timing with Quest active measured 72.24 Hz Quest, 30.21 Hz IK and 100 Hz feedback/status. Quest delivery had nine gaps above 150 ms in 30 seconds, while IK continued producing results with source ages occasionally beyond the 150 ms freshness threshold. A passive network capture confirmed seven packet arrival gaps above 150 ms in 20 seconds, maximum 212 ms. This is a packet-delivery issue despite the healthy average rate. Disabling the robot's Wi-Fi power saving did not eliminate it (eight gaps in 25 seconds, maximum 161 ms); the original power-save setting was restored. The exact sender/network origin is not established.

The manager now separates freezing from a latched hold for Quest/IK timeouts. Targets still freeze at 150 ms; fresh input can recover within `input_hold_timeout` (default 0.5 seconds, allowed range 0.15–1.0 seconds). Network recovery must include a newly processed IK target. Longer loss latches hold and requires operator recovery. INVALID tracking and motor feedback thresholds remain unchanged. Isolated 230 ms packet/IK-gap regressions reproduced the prior unwanted latch. Evidence: `quest-network-powersave-comparison.json`, `live-timing-powersave-on.json`, `burst-gap-before.log`, `burst-gap-after.log` and `integration-after-burst-gap.log`. The later successful arm/gripper demonstration below includes this input-gap change.

All six tracking/network/IK recovery cases and the home-tolerance suite passed after this change. The control session was reloaded while arms/base were disabled; live status confirmed `input_hold_timeout=0.5`, `home_tolerance=0.1`, disabled permissions and all 16 valid joints (`burst-controller-reloaded.json`). The recording integration's previous 400 ms outage assertion predates the new deadline; its sustained-loss probe now uses 700 ms. The updated complete integration passed in 17.67 seconds, including recording, verified download, persistent-loss hold and return/disable (`integration-after-burst-gap-updated-probe.log`).

#### Backward alignment and stale IK configuration

The subsequent operator attempt stayed in aligning for 27 seconds and moved backward, so the operator returned/disabled the arms. The old follower sends measured positions while its trigger gate is closed, allowing the holding target to follow any load-induced sag. The C++ manager holds its commanded home position. Sag was not a calibration step: the old graph did not feed measured arm positions into IK, and both mappings use the Quest reference pose.

An initial captured-pose replay showed matching coordinate transforms (about `1e-8` numerical difference), identical model FK and native IK hand errors below 3 mm. The different position/orientation weights therefore did not explain that case, and those weights were left unchanged.

A new capture with the operator holding the controllers normally and all motors disabled reproduced the fault. Requested hand positions were about 0.31 m forward and 0.18–0.19 m below arm_origin. The running IK reference was trapped at multiple joint limits, including both elbows at zero. Model FK placed the right hand about 0.52 m from its target and the left about 0.75 m away, behind the origin. The legacy solver reached the same goals. In isolated C++ replays, both a home seed and the measured resting seed reached the targets within 1.6 mm, while the trapped seed remained wrong. This identifies accumulated solver state across sessions as the immediate cause; it is not corrected by allowing the real arms to sag.

The temporary control session was stopped after fresh feedback confirmed all 16 joints disabled. The fix resets IK to configured home on each new homing cycle and stops accumulating IK state outside active preparation/teleop phases. Native IK marks a result `ik_unconverged` unless both hand pose residuals are within 0.02 m and 0.25 rad. During aligning, the manager keeps holding home with reason `waiting_for_ik_solution` until a converged result arrives; it then performs the existing trigger-gated slow alignment. This checks the calculated IK pose, separately from physical joint alignment.

Evidence: `pose-comparison-pose-baseline.json`, `pose-comparison-pose-current.json`, `ik-seed-comparison.json`, `disabled-before-ik-diagnosis.json`, `ik-session-before.log` and `alignment-solution-before.log`. Both regressions failed before the patch. The rebuilt IK convergence/limits, motion-reference, trapped-session, home-tolerance and seven control-manager cases passed. The complete mock recording/download/return integration passed in 17.56 seconds (`ik-session-after.log`, `alignment-solution-after.log`, `integration-after-ik-session.log`). The camera-free session was reloaded disabled. The operator subsequently confirmed correct forward alignment and following, then confirmed both grippers. The two completed real cycles had no latched hold or fault; brief pauses recovered. Evidence: `operator-following-confirmed.json`, `real-teleop-acceptance-summary.json` and `supervised-motion.jsonl`. This establishes arm/gripper functionality at the initial slow speed, not camera or base-motion acceptance.

#### Following speed

The first supervised profile used only `0.2 rad/s` for all seven joints on each arm. Both IK and the manager applied this limit during teleop, so a 0.5 rad joint target change needed at least 2.5 seconds. The isolated fake-actuator reproduction advanced only 0.130 rad in 0.65 seconds (`following-speed-before.log`). This is a commanded-target slew limit; physical measured velocity can differ because of tracking dynamics and feedback resolution.

`Project/robot_bringup/config/arms-teleop.json` raises following to `1.0 rad/s` while retaining `0.5 rad/s` for grippers. Both IK and manager use this file. The new manager parameter `home_joint_speed_limit=0.2` independently keeps homing and returning slow; aligning retains its existing `0.2 rad/s` cap. The default parameter value `0` preserves configured speeds for other launch paths, and a positive cap can only lower a configured joint speed. Gains, targets and input-loss thresholds are unchanged.

The managed `Project/luxi_web_ctrl/scripts/program.launch.xml` uses this profile, `home_tolerance=0.1` and `home_joint_speed_limit=0.2`, preserving the working settings for the later recording workflow. The arm device adapter continues using `arms-kh.json` for physical interfaces and its higher hardware limits. Do not start the managed program alongside the camera-free session. The build and all 13 focused cases passed: four home-tolerance, seven input-gap/alignment, one phase-speed and one IK-session case. The same 0.5 rad target step reached 0.500 rad within 0.65 seconds, and the three transition phases each stayed at a maximum commanded slew of 0.200000 rad/s (`following-speed-after.log`, `following-speed-ik.log`). The camera-free controller was reloaded only after fresh feedback confirmed all motion permissions disabled. Loaded status reports the 1.0/0.5 rad/s limits, `home_joint_speed_limit=0.2` and `home_tolerance=0.1`; all 16 joints remain fresh and Quest is at 72 Hz (`following-speed-reloaded.json`). The operator subsequently confirmed the faster profile feels good. Arm following and both grippers therefore have operator acceptance with the loaded 1.0 rad/s following profile. The observed faster run at 17:43 completed homing, alignment, teleop, return and disable; all 207 teleop observations had 16 fresh joints, with three briefly paused samples and no latched hold/fault (`following-speed-operator-accepted.json`).

For future reloads, support both arms before using **撤销双臂使能**, since disabling removes holding torque. Confirm arm feedback says disabled before restarting `luxi-supervised-teleop.service`. Dashboard password authentication was removed at the operator's request. Grey buttons now reflect service/readiness conditions, not a login requirement. Fault state itself does not block the disable operation.

At startup the control manager is disabled. Wear the Quest with the sender in the foreground; wait for a sustained rate of at least 50 packets/s. After clearing the home path and making the physical stop accessible, hold X for 1.5 seconds and release it. Leave index triggers released during homing. Once aligning, hold the controllers steady and squeeze the corresponding index triggers above 0.92 to permit alignment. After teleop starts, make small movements with one controller at a time, then check each gripper. A short X commands return to home followed by disable.

Do not start the normal managed program alongside this session. After the arms have returned and are confirmed disabled, stop the temporary unit before switching back:

```bash
sudo systemctl stop luxi-supervised-teleop.service
```

Read-only evidence for this check is appended to `supervised-readiness.jsonl` and `supervised-motion.jsonl`. Actual motion acceptance must be recorded after observation; starting the session alone does not establish teleop works.

## 1. Connect and inspect

Open `http://192.168.3.179:8000` in a browser. The dashboard requires no password or token for base/arm control, recording or device management. The login form, backend password checks and startup token-file dependency have been removed; refresh an already-open page to load the new interface. Any client that can reach this LAN dashboard can send control requests. Robot readiness, neutral-stick enable, explicit motion permissions and stop/timeout behavior remain enforced by the controllers.

After resolving the live USB problem above, use the dashboard to start acquisition for Quest, arms, base and the three cameras, then start **控制与录制程序**. Starting these services does not itself enable motion. Check their current state before clicking; camera and program services were deliberately paused during diagnosis.

The normal managed arm adapter uses `luxi_left` and `luxi_right` from `arms-kh.json`; the control/recording program uses the lower-speed `arms-teleop.json` profile. The base uses `luxi_base`. Do not reuse the old PEAK interface numbering.

Required readiness for a three-camera demonstration:

| Component | Ready condition |
|---|---|
| Arms | All 16 joint validity flags true and fresh feedback |
| Quest | Real packets continuously arriving; at least 50 accepted packets in the preceding second to enable arms |
| Tracking | Both controllers tracked; use fully valid tracking for the supervised check |
| Cameras | All three previews update; a running service alone is insufficient |
| Program | Control and episode services available; control initially disabled, recorder idle |
| Disk | At least 20 GiB available under the default collection configuration |

Camera serials:

| View | Configured serial |
|---|---|
| Head | `00DA8573794` |
| Left wrist | `00DB1265540` |
| Right wrist | `00DA8573792` |

If no camera appears in `lsusb`, check camera power and camera-side USB connections first. Avoid disconnecting the shared CAN hub. The ARM64 SDK is installed; restarting software cannot make an electrically disconnected camera appear. Drivers retry automatically. A different physical serial must be configured deliberately for the corresponding view.

## 2. Connect the Quest

Run the existing teleoperation sender application on the Quest, with target **192.168.3.179, UDP 5006**. Keep the sender in the foreground, both controllers awake, and the Quest on a network that can reach the robot. Merely opening a browser on the Quest does not send teleop packets.

Keep X and Y unpressed while checking readiness. Confirm the dashboard packet rate is nonzero and sustained above the enable threshold. For base driving the APK must also send joystick fields and the Y button. The managed profile does not require a grip hold; `lg`/`rg` remain reserved for future lift control.

## 3. Supervised arm enable and alignment

Keep base permission disabled for the first arm demonstration. Clear the arms' home path and keep the physical stop accessible. The inspected starting pose requires about 82 degrees of movement at both fourth joints to reach the configured home pose; enabling is a motion command.

1. When the operator confirms the path is clear, hold **X for approximately 1.5 seconds**, or click **使能双臂**.
2. Observe `disabled -> homing -> aligning`. Homing moves to the configured start pose; it does not calibrate motor zero offsets.
3. Hold the controllers steady in a comfortable reference pose. Squeeze each corresponding index trigger nearly fully (greater than 0.92) to permit slow alignment on that side.
4. Wait for `control.phase=teleop`. Make small, slow movements with one controller at a time; confirm the corresponding arm follows. Check both grippers with small trigger changes.

Short X or **暂停 / 返回** commands a return motion, then disables the arms. **撤销双臂使能** is the separate disable operation. The browser is not a hardware emergency stop.

Brief INVALID controller tracking immediately freezes the target and can recover within 150 ms after a new IK result arrives. Quest/IK data aged beyond 150 ms also freeze the target, but allow fresh-input recovery until `input_hold_timeout` (default 500 ms). Longer loss enters latched `hold`; inspect `control.reason`. Reconnection alone does not resume a latched hold. Restore input, use the normal return/disable procedure when its path is clear, then explicitly enable and align again.

## 4. Record a short real acceptance episode

Start with 10–15 seconds of small, slow arm and gripper movements. Keep the base disabled for this first visual episode.

1. Confirm `teleop`, fresh arm feedback and all three updating cameras.
2. Enter a task description in the dashboard. Click **开始录制** and verify `recorder.state=recording` and the sample count increases.
3. Perform the short demonstration while checking previews and feedback.
4. Click **成功并保存**. Wait for `recorder.state=idle` and note the episode ID.
5. Clear the return path, short-press X, and wait for arms to be disabled.

Quest shortcuts: short A starts a recording or finishes it as successful; B opens the failure menu during recording and requests arm return; left-stick vertical movement selects a reason and A confirms failure. Y is exclusively the base enable/disable toggle, including when held. To cancel while retaining data, use **取消本次并保留数据** on the dashboard. A-start uses the recorder's configured task name, not text typed into the dashboard.

Saving a successful recording does not itself return or disable the arms. Use the separate return step.

The current managed recorder requires all three cameras. Do not bypass this requirement or introduce mock images to make a visual episode appear valid. Missing views need a deliberately chosen collection configuration and dataset definition.

## 5. Check the saved episode

Managed real episodes are stored in `data/real/episode-*`. A completed episode contains:

```text
session.json
robot_config.json
samples.jsonl
manifest.json
bag/metadata.yaml
bag/*.db3
```

Set the actual episode directory before reading it:

```bash
episode_dir="$PWD/data/real/episode-REPLACE_WITH_ACTUAL_ID"
python3 -m json.tool "$episode_dir/manifest.json"
source /opt/ros/humble/setup.bash
source build/install/setup.bash
ros2 bag info "$episode_dir/bag"
```

Require `status=accepted`, `samples>0`, `qc=[]`, and nonzero arm state, applied-command and all three camera topic counts. The `.partial-episode-*` name indicates an uncommitted recording and must not be treated as accepted data.

Each sample has 16 measured state values and 16 final submitted action values, ordered right eight then left eight. Base feedback/commands are separate asynchronous bag topics. A submitted motor command does not prove the robot reached it, and camera timestamps do not establish hardware exposure synchronization.

Failed and cancelled episodes remain available with their outcome labels. Preserve them separately from accepted demonstrations.

## 6. Optional R680 demonstration

Only add base motion after the arm-only acceptance and an operator check of the base's travel area. Base permission is independent of arm permission.

1. Leave the arms disabled for the first base demonstration. Clear the base travel area, center both sticks and close any recording failure menu. Grips are independent of base control in the managed profile.
2. Confirm fresh base feedback and `flag_stop=0`; short-press **Y**, or click **启用 Quest 底盘控制**. If enabling is rejected, restore readiness, release Y and press it again.
3. Move the left stick slightly forward/backward to drive forward/backward, or left/right to strafe. Move the right stick horizontally to turn. Start with a small displacement on one axis at a time; no grip hold is required.
4. Release the sticks to center and immediately command zero velocity. Confirm the chassis stops. Press **Y** again or use **底盘停车** to revoke permission after the check. Re-enable is explicit after input loss, invalid tracking or a stop button.

The direct mode and tracking change passed three isolated running-node cases plus ten codec/mapping/gate cases (`base-quest-before.log`, `base-quest-after.log`). The rebuilt base adapter was loaded only after fresh feedback confirmed a stationary, disabled base and disabled arms. Live status confirms `input_source=quest`, `require_left_grip=false`, hardware mode and fresh base feedback (`base-quest-reloaded.json`). The base remains disabled until the operator enables it. At 18:13–18:15, the operator confirmed the base physically moves with the joysticks. The trace captured nonzero translation/yaw commands and corresponding real chassis feedback (translation peaks about 0.19 m/s and yaw about 0.31 rad/s). All 1,899 sampled commands marked `sticks_centered` were zero. The last captured state was stationary and disabled after Quest reception stopped. Evidence: `base-quest-live.jsonl`, `base-no-motion-current.json`, `base-no-motion-timeline.json` and `base-operator-accepted.json`. No further runtime code change was made during this follow-up: its initial snapshot showed `operator_stop`, while the subsequent enabled trial moved. The specific earlier stop trigger was not captured.

The managed launcher sets `require_left_grip=false`; the standalone adapter default remains `true` for existing launch compatibility. The dashboard displays the active mode from base status. Startup requires centered sticks and fresh feedback/input. Ordinary STALE (1) pose flags accept current joystick input; INVALID (2) stops and revokes permission. Defaults are 0.2 m/s maximum translation and about 0.314 rad/s maximum yaw. Reverse steering follows the existing PS2 convention. Opening the failure menu, pressing B, invalid tracking or timeout can revoke permission. Y toggles permission and stops when already enabled; it does not bypass readiness or failure-menu gates. Browser WASD is a separate low-speed translation mode, capped at 0.05 m/s.

### Using arms and base together

1. Keep the base disabled while using X to enable/home the arms and the index triggers to align them.
2. Once arm teleop is ready, center the sticks and short-press **Y**, or use **启用 Quest 底盘控制** on the dashboard. Base and arm permissions are separate.
3. Left stick translates the chassis; right stick horizontal turns it. Center the sticks to submit zero base velocity.
4. B, invalid tracking or a Quest timeout revoke base permission. Y toggles permission; pressing it while enabled stops the base. X only controls the arms; using it does not change base permission or joystick commands. After restoring input, center the sticks, release Y and press it again to enable, or use the dashboard; recovery does not automatically rearm it.
5. When finished, press **Y** while base permission is enabled or use **底盘停车**, then perform the normal arm return/disable with a clear return path.

The base uses its existing 200 ms Quest deadline. The latest trace included timeout stops followed by explicit re-enable; no timeout threshold was changed during the movement diagnosis.

### Future lift mapping (hardware not installed)

The operator selected **left middle-finger grip = down**, **right middle-finger grip = up**. Hold one grip to move in that direction; release both or hold both to stop. Index triggers continue operating the arm grippers. These two grip inputs are already decoded as `QuestInput.grips` (`lg`/`rg`) and are reserved independently of the base.

No lift driver or lift command is enabled by this change. Once the lift is fitted, its interface, travel limits, feedback and commanded speed must be connected to this mapping before it can move. The present 16-dimensional arm state/action schema does not imply that lift state/action is already collected.

## 7. Transfer and end the session

On a receiving machine with this project's tools installed:

```bash
source build/install/setup.bash
ros2 run luxi_data_recorder episode_pull \
  http://192.168.3.179:8000 episode-REPLACE_WITH_ACTUAL_ID /path/to/datasets
```

The receiver resumes partial files, verifies hashes and commits its final directory only after success. It does not delete the robot-side data.

Before stopping services or powering down: save/cancel recording, wait for idle, stop the base, return the arms along a clear path and wait for disabled. Then stop the control/recording program and acquisition services through the dashboard as needed.

Software checks and real Quest pose reception are complete. Arm alignment, faster following, both grippers and return/disable have operator acceptance. Joystick base motion is also operator-confirmed, with real command/feedback and centered-zero evidence. Remaining collection work is stable real camera streams and one inspected real accepted visual episode. Lift control remains planned until the hardware is installed.
