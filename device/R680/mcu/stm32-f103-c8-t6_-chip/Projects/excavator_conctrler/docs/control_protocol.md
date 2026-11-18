# Excavator Control Protocol

## Radio

```text
Module:   NRF24L01+
Address:  34 43 10 10 01
Channel:  76
Rate:     250kbps
TX rate:  30Hz
Payload:  14 bytes
```

The NRF24L01 hardware Auto ACK and ACK payload are enabled by the common driver. The receiver must preload ACK payload bytes `0x4F 0x4B` (`"OK"`) after it validates a control packet.

Connection state:

```text
LINK OK  - TX succeeded and ACK payload is "OK"
LINK ERR - TX succeeded without a valid "OK", TX failed, or ACK payload is missing
```

## Packet

```c
typedef struct
{
    uint8_t head1;      /* 0xAA */
    uint8_t head2;      /* 0x55 */
    uint8_t version;    /* 0x01 */
    uint8_t device_id;  /* 0x01 */
    uint8_t msg_type;   /* 0x01: velocity command */
    uint8_t seq;
    uint8_t length;     /* 6 */
    int8_t left_track;
    int8_t right_track;
    int8_t swing;
    int8_t boom;
    int8_t arm;
    int8_t bucket;
    uint8_t crc;
} ControlPacket_t;
```

CRC uses CRC8 with polynomial `0x07`, initial value `0x00`, and covers all packet bytes except the final `crc` byte.

## ACK Payload

```c
uint8_t ack[2] = {0x4F, 0x4B}; /* "OK" */
```

The receiver should return this ACK payload only after:

```text
header/version/device_id/msg_type/length are valid
CRC8 matches
the control command has been accepted by the receiver application
```

The sender treats the device as connected only when this ACK payload is received.

## Sign Convention

```text
left_track   > 0: forward,  < 0: backward
right_track  > 0: forward,  < 0: backward
swing        > 0: left,     < 0: right
boom         > 0: raise,    < 0: lower
arm          > 0: raise,    < 0: lower
bucket       > 0: outward,  < 0: inward
```

## Button and joystick mapping

The four debounced buttons drive the tracks at a fixed 30 RPM. Only the listed
combinations are accepted; any other combination stops both tracks.

```text
L1:       left turn  -> motor 0x01 CCW, motor 0x02 CCW
R1:       right turn -> motor 0x01 CW,  motor 0x02 CW
L1 + R1:  forward    -> motor 0x01 CCW, motor 0x02 CW
L3 + R3:  backward   -> motor 0x01 CW,  motor 0x02 CCW
```

Joystick centers come from the measured controller values. Each axis uses a
150-count dead zone and emits only `0`, `-10`, or `+10`, so actuator speed is
constant outside the dead zone.

Only one of swing, boom, arm, and bucket is emitted in each control packet.
When motion starts, the raw ADC distance from each axis center is compared and
the largest displacement is locked. Other axes cannot take control until the
locked axis returns to its dead zone, preventing ADC noise from causing rapid
axis switching and start-stop motion.

```text
LX center 1967: left -> swing +10, right -> swing -10
LY center 1967: up   -> arm +10,   down  -> arm -10
RX center 1975: left -> bucket -10,right -> bucket +10
RY center 2086: up   -> boom -10,  down  -> boom +10
```

The receiver runs the three servos at 410 position counts per second, which is
approximately ten seconds across the 0 to 4095 range. Swing motor address
`0x03` runs at 5 RPM (30 motor-degrees/second) toward the selected
center-relative `-360` or `+360` motor-degree endpoint. With the installed 1:2
reduction this corresponds to a mechanism range of `-180` through `+180` degrees. Startup automatically uses
the most recently saved center and returns to it.
