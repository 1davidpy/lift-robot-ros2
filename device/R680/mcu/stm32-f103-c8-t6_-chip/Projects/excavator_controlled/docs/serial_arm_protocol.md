# Excavator Arm Serial Protocol

The host port uses USART1 on PA9/PA10 at 115200 baud, 8 data bits, no parity, and one stop bit. Commands are ASCII lines terminated by LF. CR before LF is accepted.

Axis mapping:

| Axis | Servo ID | Mechanism |
|---:|---:|---|
| 0 | 1 | Boom |
| 1 | 2 | Arm |
| 2 | 3 | Bucket |

The project parameter table applies these hard defaults at every startup:

| Axis | Minimum | Maximum | Positive wireless motion |
|---:|---:|---:|---|
| 0, boom | 949 | 2640 | Raise, decreasing position |
| 1, arm | 1174 | 3123 | Raise, decreasing position |
| 2, bucket | 897 | 3137 | Raise, decreasing position |

## Commands

| Command | Description |
|---|---|
| `PING` | Return `PONG`. |
| `STATUS` | Return the current state immediately. |
| `JOG axis direction speed` | Start or refresh smooth jogging on only the selected online axis. Direction is -1 or 1. Refresh every 50 ms to continue. |
| `STOP axis` | Stop jogging by reading the current position once and holding it. |
| `CLEAR axis` | Cancel queued motion and restore that axis to its project-table hard limits without issuing a position command. |
| `LIMIT axis minimum maximum` | Cancel queued motion and apply a narrower runtime range. The requested range must remain inside the project-table hard limits and contain the current position. |
| `MOVE axis position speed` | Move to an absolute position inside the active hard/runtime limits. The axis must be online. |
| `STEP_RUN address direction rpm` | Run one PD42S1 continuously. Addresses are 1 to 3 and direction is 0 for CW or 1 for CCW. Addresses 1 and 2 use the requested RPM; address 3 is fixed at 5 RPM. |
| `STEP_CENTER 3` | Stop address 3, use the current encoder position as center, and persist it in the reserved flash page. |
| `STEP_ANGLE 3 angle rpm` | Move address 3 to an absolute motor angle relative to the stored center. Angle is limited to -360 through 360 degrees. The RPM field is accepted for compatibility but address 3 always uses 5 RPM. |
| `STEP_CENTER_STATUS 3` | Return `STEP_CENTER_STATUS 3 1` when centered motion is ready, otherwise return a zero. |
| `STEP_POSITION address` | Read the signed encoder position in pulses for diagnostics. |
| `STEP_SPEED address` | Read the signed real-time speed in RPM for diagnostics. |
| `STEP_STOP address` | Stop one PD42S1 using the configured deceleration. |
| `STEP_STOP_ALL` | Stop addresses 1 to 3 with one CAN broadcast command. |

Position range is 0 to 4095. Speed range is 1 to 3400.

## Responses

- Successful commands return `OK` followed by the command name.
- Invalid commands return `ERR COMMAND`.
- Motion can also return `ERR OFFLINE` or `ERR RANGE`.
- Step-motor commands return `OK` followed by their command name. Centered positioning can also return `ERR STEP_CENTER_REQUIRED`; other errors include `ERR STEP_NOT_READY`, `ERR STEP_ARGUMENT`, and `ERR STEP_CAN`.
- Status format is:

      STATUS onlineMask limitMask pos0 pos1 pos2 min0 max0 min1 max1 min2 max2

The device emits a status line every 100 ms. At startup it reads servo feedback, emits `ARM_READY`, and remains stationary until a motion command is received.

Position writes are queued per axis. A command for one axis never re-sends stored targets for the other axes. `LIMIT` and `CLEAR` do not issue position writes. Starting a jog sends the selected mechanical endpoint once at the requested speed; refresh frames only keep the watchdog alive and do not resend small incremental positions. Releasing the control reads the current position once and holds it. The host refreshes the jog state every 50 ms, and a 200 ms watchdog stops and holds the axis if refresh frames are lost.

Before an axis becomes online, the firmware writes zero only to its goal-speed register, reads operating mode, and changes a nonzero mode to position mode. This stops a servo left in wheel mode without changing the goal position. An axis that cannot be verified in position mode remains offline and rejects motion commands.

The PD42S1 CAN task sends a zero-speed broadcast during startup, so the step motors remain stopped until the host sends `STEP_RUN`. Disconnecting or closing the Qt host sends `STEP_STOP_ALL` before the serial port closes.

The PD42S1 encoder scale is 51200 pulses per motor turn, so 360 motor degrees is 51200 pulses. With the installed 1:2 reduction, the motor's center-relative +/-360-degree range produces a mechanism range of +/-180 degrees. The encoder does not provide a mechanical midpoint for the excavator. `STEP_CENTER 3` stores the selected encoder position in the final 1 KB internal-flash page. On every startup the firmware loads that value and commands address 3 back to 0 degrees. Normal segmented firmware downloads preserve the parameter page; a full-chip erase clears it, after which the first startup position becomes the new default center.

Wireless controls use fixed speeds. Track motors `0x01` and `0x02` run at 30
RPM according to the L1/R1/L3/R3 combinations. Servo joystick motion runs at
410 position counts per second, approximately ten seconds across 0..4095.
LX drives motor `0x03` toward the center-relative +/-360-degree motor endpoint at 5 RPM
(30 motor-degrees/second). Target supervision checks position every 40 ms and stops
at the arrival band or on the first target crossing. It never commands an
automatic reverse correction.

Wireless track release sends the initial zero-speed command and schedules two
additional per-motor zero-speed commands 25 ms apart. The 200 ms NRF command
watchdog remains active as a separate lost-link stop path.
