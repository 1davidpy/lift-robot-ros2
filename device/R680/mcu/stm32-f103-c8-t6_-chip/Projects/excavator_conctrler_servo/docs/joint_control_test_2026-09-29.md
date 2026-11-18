# Full-servo joint-control test — 2026-09-29

## Firmware and targets

- Receiver project: `excavator_controlled_servo`
- Receiver ST-Link: `E1007200D0D2139393740544`
- Controller project: `excavator_conctrler_servo`
- Controller ST-Link: `10004400180000513751594E`
- Both targets: STM32F103 medium-density, 64 KB Flash, approximately 3.22 V
- NRF identity: device `0x02`, address `34 43 10 10 02`, channel 76,
  250 kbps, 14-byte payload

Both Release images were downloaded with STM32CubeProgrammer using SWD Under
Reset at 100 kHz and passed download verification. Existing 64 KB Flash images
were backed up before programming.

## Safety correction

The first controller boot exposed real input-startup transients before the ADC
and debounced keys settled. The receiver briefly reported non-zero track levels
and changed position-axis feedback. The controller was stopped and updated with
a startup interlock: all joystick channels and drive buttons must remain neutral
for one continuous second before motion commands are armed. Before arming, the
controller transmits stop commands only. Normal control mapping is unchanged
after arming. The automatic drive-key self-test remains disabled.

## Final observations

- Controller SWD wireless state: `5` (`WIRELESS_STATUS_CONNECTED`, `LINK OK`)
- Stable joystick ADC: `1873, 1967, 2077, 2013`
- Stable drive-key mask: `0`
- Startup interlock: armed after stable neutral input
- Receiver status: `online_mask=63`, all six servos online
- Receiver limits: `limit_mask=7`, limits enabled only for IDs 1–3
- Final repeated track state: `0, 0`
- Final observed positions after the initial joint test: `1013, 1226, 658, 4094`

This validates neutral command transmission, NRF acknowledgement, receiver
packet acceptance, six-servo discovery, configured limit mask, and stopped
track output. No further automatic motion sequence was run after the startup
interlock was installed.

## ID4 continuous-rotation update

ID4 was subsequently changed from endpoint-based position motion to STS3215
continuous wheel mode. Holding LX commands signed speed 1000 indefinitely;
release, radio timeout, or `STOP 3` writes zero speed. `MOVE 3` now returns
`ERR CONTINUOUS`, and limit commands continue to return `ERR NO_LIMIT`.

After the update, the receiver reported all six servos online. ID4 crossed the
4095/0 position counter boundary and remained online, confirming that the old
position endpoint is no longer used as a stop. The final repeated neutral state
was `STATUS 63 7 1016 1966 1292 318 0 0`.

Programming the receiver while servo power remained on allowed IDs 1–3 to
continue toward goals retained inside the servos while the MCU was held in
reset. Receiver startup now commands IDs 1–3 to hold their first feedback
positions, but servo power should still be disconnected during future receiver
firmware programming because the MCU cannot issue a stop while held in reset.

## Direction and speed update burn verification

The receiver Release image was rebuilt, downloaded to ST-Link
`E1007200D0D2139393740544`, verified successfully, and reset on 2026-09-29.
The active wireless settings are:

- ID1 boom: direction unchanged, speed 250
- ID2 arm: vertical direction reversed, speed 500
- ID3 bucket: direction unchanged, speed 500
- ID4 swing: left/right direction reversed, continuous speed 1000, no limit
- ID5 and ID6 tracks: maximum motion speed 3000, no limits

Post-reset SWD inspection found IDs 1-6 online. IDs 1-3 had position mode and
limits verified; IDs 4-6 had continuous mode verified and a zero motion command.
The controller reported `WIRELESS_STATUS_CONNECTED`, neutral inputs, no pressed
keys, and an armed startup interlock. No serial port was enumerated, so the Qt
serial status page was not used in this check. No automatic physical motion was
commanded; direction verification still requires an operator to move each
control while watching the mechanism and being ready to stop it.

## 2026-09-30 chassis speed control update

Field feedback showed that the ID4 left/right direction was reversed, so its
configured positive direction was switched back. BR is now a rising-edge speed
selector and no longer participates in the chassis direction mask. Each press
cycles levels 1 through 6 and then wraps to level 1; the display shows `V1`
through `V6`. The levels map to raw wheel speed commands 1000 through 6000.

Because BR was formerly part of the reverse combination, BL alone now commands
reverse. TL and TR retain left/right turning, and TL+TR retains forward motion.
Both controller and receiver Debug/Release images build successfully.

On 2026-09-30, both Release images were programmed and passed STM32CubeProgrammer
download verification. Post-reset SWD diagnostics showed controller speed level
1, no pressed keys, neutral ADC inputs, an armed startup interlock, and
`WIRELESS_STATUS_CONNECTED`. Receiver diagnostics showed IDs 1-6 online,
position mode and limits verified for IDs 1-3, continuous mode verified for
IDs 4-6, and zero track commands. Physical direction and BR button-edge checks
still require an operator to briefly actuate the controls while observing the
machine.

After further field feedback on 2026-09-30, the TL and TR differential-track
commands were swapped because TL still produced a physical right turn. The
controller default speed level was changed from V1 to V3 (raw speed 3000), then
rebuilt, programmed, verified, and reset. SWD confirmed level 3 after reset,
neutral inputs, no pressed keys, and `WIRELESS_STATUS_CONNECTED`. The existing
V1-V6 command mapping remains 1000-6000. Limited physical speed increase above
V3 is consistent with the servo approaching its achievable speed at the
present supply voltage and load; the transmitted command values still increase
for V4-V6.

The global joystick largest-displacement interlock was removed on 2026-09-30.
The final mapping allows the left and right joysticks to operate together, but
selects only the larger-displacement axis within each individual joystick.
LX/LY therefore select ID4 swing or ID2 arm, while RY/RX select ID1 boom or
ID3 bucket. This permits one left-stick function and one right-stick function
to run simultaneously without allowing both axes of one stick to move together.
