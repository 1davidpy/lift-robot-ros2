# Quest X/Y/B mapping validation — 2026-09-15

The changed base adapter, recorder and dashboard packages built and installed successfully on the robot (5 min 56 s). No live device or control services were restarted, and no real motion was commanded during these checks. The new mapping takes effect when the corresponding services next start.

## Mapping

- X controls only the arms: hold about 1.5 seconds while disabled to enable/home; short press requests return/home followed by disable. X does not change base permission or joystick commands.
- Y toggles base permission once per press. Enabling requires centered sticks, fresh valid input and feedback, no failure menu and no chassis stop. A rejected enable or reconnect requires release and a new press. Holding Y does not cancel recording.
- B always stops/disables the base. During recording, a B press opens the failure menu and sends the arm controller a `stop` request (return/home and disable, including arm motion). When idle, a short B press does not request arm stop; holding B about 1.5 seconds requests arm return and session exit after fresh disabled feedback. B never enables either subsystem.
- Recording cancellation remains on the dashboard / episode `cancel` service.

## Software results

- Base mapping: all 8 cases passed, followed by two further consecutive complete passes. Covers X independence, Y enable/disable/hold/reconnect/rejected enable, B priority, keyboard handoff, receive-only blocking, tracking, centered stopping and recording-menu interlock.
- Recorder buttons: 4 cases passed against the installed recorder using synthetic arm feedback and a fake arm service. Confirms short/long B behavior, the disabled-feedback exit gate, and Y not cancelling recording. The explicit cancel service still retains a cancelled episode.
- Existing arm regressions: 4 homing-tolerance, 7 tracking-gap and 1 phase-speed cases passed.
- Existing storage (2), R680 codec/mapping/gate (10), CAN mapping (1) and fake-camera SDK (3) cases passed.
- R680 mock UDP integration and password-free dashboard gateway checks passed.
- Full mock collection passed: UDP to mapping/IK to mock arms and three cameras, saved bag/samples, resumed HTTP download and hash verification; base raw/state/applied/status topics were present. Episode: `episode-1789438590929347324`.

These are software/mock results. The remapped buttons have not been operator-tested on moving hardware. Real camera USB reliability and a real visual collection episode remain outstanding.

## Test-harness diagnosis

Initial base cases raced enable service requests against joystick topic delivery. The test now waits for `source_quest_mono_ns` acknowledgment before asserting service enable conditions. Further full-suite failures showed delayed input and watchdog stops. A latest-sample publisher queue (depth 1 instead of 10) eliminated the failures across the repeated complete suites. A pacing experiment and a fresh-context experiment were not retained. No production timeout, tracking acceptance, speed or acceleration limit was changed for these test failures.

## Evidence and reproduction

Robot-relative logs:

- `log/button-tests-20260915-recorder.log`
- `log/button-tests-20260915-core.log` (initial base failures plus passing arm/storage/codec checks)
- `log/button-tests-20260915-workflow.log`
- `log/button-tests-20260915-race-probe.log`
- `log/button-tests-20260915-base-latest-queue.log`
- `log/button-tests-20260915-base-repeat.log`
- `log/button-tests-20260915-base-confirmed.log` (colcon result refresh)

The recorder test script is preserved in `log/button-tests-20260915/test_recorder_buttons.py`. These commands use isolated domains and synthetic inputs; they do not enable real motors:

```bash
source /opt/ros/humble/setup.bash
source build/install/setup.bash
python3 Project/device_adapters/test/test_base_quest.py build/packages/luxi_device_adapters/r680_adapter
python3 log/button-tests-20260915/test_recorder_buttons.py build/install/luxi_data_recorder/lib/luxi_data_recorder/data_recorder
```
