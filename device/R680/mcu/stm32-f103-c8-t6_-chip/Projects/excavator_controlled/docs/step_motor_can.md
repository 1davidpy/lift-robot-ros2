# PD42S1 CAN Control

The controlled unit uses CAN1 on PA11/PA12 to control three PD42S1 drivers. A physical CAN transceiver is required between the MCU pins and CAN_H/CAN_L.

## Bus configuration

| Item | Value |
|---|---:|
| CAN bitrate | 500 kbit/s |
| Frame type | 29-bit extended data frame |
| Transmit identifier | `0x1000` |
| Motor addresses | `0x01`, `0x02`, `0x03` |
| Acceleration/deceleration | `100` RPM/s^2 |
| Allowed target speed | 1 to 3000 RPM |

The default-pin wiring is:

| STM32F103 | CAN transceiver |
|---|---|
| PA11 | RXD |
| PA12 | TXD |
| 3.3 V | Logic supply, when required by the transceiver |
| GND | GND |

Connect CAN_H/CAN_L to the motor bus, share ground, and terminate both physical bus ends according to the hardware documentation.

## Safe startup

After CAN initialization, firmware sends a zero-speed broadcast and clears the state of addresses `0x01`, `0x02`, and `0x03`. It then prints:

```text
STEP_MOTORS_READY
```

No step motor starts automatically. If the CAN bus or motor power is unavailable, firmware prints `STEP_MOTORS_WAITING` once and retries initialization every second. It prints `STEP_MOTORS_READY` as soon as the drivers become available. A later CAN transmit failure marks the subsystem unavailable and starts the same automatic recovery sequence.

## Host commands

The Qt host sends ASCII commands over the 115200 baud debug serial port:

```text
STEP_RUN address direction rpm
STEP_CENTER 3
STEP_ANGLE 3 angle rpm
STEP_STOP address
STEP_STOP_ALL
```

Addresses are decimal `1`, `2`, and `3`. Direction `0` is CW and `1` is CCW. Addresses `1` and `2` keep their independent 1 to 3000 RPM continuous-rotation controls. Address `3` always runs at 5 RPM (30 motor-degrees/second); its `rpm` command field is retained only for host-protocol compatibility.

Address `3` additionally supports persistent centered positioning. `STEP_CENTER 3` stores the current encoder position in the MCU's reserved final 1 KB flash page. Startup loads that value and automatically returns to it. `STEP_ANGLE 3 angle rpm` accepts -360 through 360 motor degrees. Firmware approaches the target in speed mode at 5 RPM, reads encoder position every 40 ms, and stops when it enters the approximately 1-degree arrival band or crosses the target. It never reverses automatically to correct overshoot, and it also stops if feedback shows that the motor is moving away from the target. The driver scale is 51200 pulses per turn, so each endpoint is 51200 pulses from the selected center. With the installed 1:2 reduction this produces a -180 through 180 degree mechanism range. A normal segmented firmware download preserves the center page, while a full-chip erase clears it.

The global stop button, serial disconnect, and application close action all send `STEP_STOP_ALL`.

Wireless drive commands use fixed 30 RPM track speed. Forward is address
`0x01` CCW plus address `0x02` CW; backward reverses both directions. Left and
right turns drive both motors CCW or both motors CW respectively.

The wireless LX command for address `0x03` preserves the centered-position
limit. Moving LX left or right commands the corresponding +360 or -360 motor-degree
endpoint at 5 RPM (30 motor-degrees/second), and releasing LX stops the motor.

Track release is fail-safe at the CAN command layer: after the first stop for
addresses `0x01` and `0x02`, firmware sends two more individual zero-speed
commands 25 ms apart. The NRF receiver also submits an all-zero command after
200 ms without a valid packet.
