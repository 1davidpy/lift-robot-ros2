# STS3215 replacement assessment

Historical mechanical assessment below. Current firmware is the seven-servo
calibration version: ID3 is gripper pitch and ID8 is gripper opening. Wireless
motion, swing and tracks are inhibited. See `servo_control.md` for the current
axis map and calibration workflow.

## Conclusion

The six-servo design can reproduce the same logical controls, but identical
loaded mechanical performance cannot be confirmed by compilation alone.

ID5 and ID6 use STS3215 mode 1 (closed-loop wheel speed), so they can provide
continuous bidirectional track motion. The configured maximum command is 6000
speed units. Direction is encoded with bit 15 and the left side is reversed to
match the mirrored installation.

ID4 uses mode 1 continuous wheel control at speed 1000. Holding the swing input
runs continuously in the selected direction, and releasing it writes zero
speed. It does not use the host-configurable software limits that protect ID1,
ID2, and ID3.

## Important difference from the former swing motor

The former PD42S1 configuration allowed -360..+360 motor degrees around a
stored center. With the documented 1:2 reduction, that produced -180..+180
degrees at the upper structure. A single 0..4095 STS3215 position range is 360
motor degrees:

- With direct 1:1 coupling, it can cover the same 360-degree mechanism span.
- With the former 1:2 reduction retained, it covers only 180 mechanism degrees.
- STS3215 supports multi-turn commands, but its turn count is not retained
  across a power cycle. Matching the former centered range through multi-turn
  operation therefore requires a homing/index sensor or a mandatory center
  procedure after every power-up.

The earlier position-mode swing proposal was superseded by continuous wheel
mode for ID4. The current calibration version inhibits ID4 movement entirely.

## Mechanical checks required later

1. Confirm the exact STS3215 voltage/SKU before applying power.
2. Lift both tracks, verify ID5/ID6 forward direction, then test the 200 ms
   wireless-loss stop.
3. Measure loaded current and case temperature during repeated starts, turns,
   and stall-prone maneuvers.
4. Verify that available continuous torque is sufficient at the selected wheel
   radius; stall torque must not be used as continuous design torque.
5. Measure ID4 backlash and loaded stopping error near the absolute-position
   wrap point before relying on unrestricted steering motion.

## Manufacturer references

- STS3215 overview, operating modes, feedback, and multi-turn behavior:
  https://www.feetechrc.com/20210430-56680.html
- Official Arduino driver showing mode 1 wheel control and bit-15 signed speed:
  https://github.com/ftservo/FTServo_Arduino/blob/main/src/SMS_STS.cpp
