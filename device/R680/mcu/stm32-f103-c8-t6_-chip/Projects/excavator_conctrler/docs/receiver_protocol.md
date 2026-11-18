# Excavator Receiver Protocol

## Radio Layer

The sender uses NRF24L01+ Enhanced ShockBurst. The receiver must listen with the
same radio parameters:

```text
Module:        NRF24L01+
Mode:          PRX
Address width: 5 bytes
RX address:    34 43 10 10 01
Channel:       76
Data rate:     250kbps
Payload:       14 bytes
Auto ACK:      enabled
ACK payload:   enabled
```

The 5-byte address is not part of the application message. It is used by the
NRF24L01 radio hardware to filter packets before the payload is exposed to the
MCU.

The over-the-air radio frame is conceptually:

```text
NRF preamble + NRF address + NRF packet control + 14-byte application payload + NRF CRC
```

The MCU application receives only the 14-byte application payload:

```text
AA 55 01 01 01 seq 06 left_track right_track swing boom arm bucket crc
```

## Receiver Address Setup

The receiver should configure pipe 0 with:

```text
RX_ADDR_P0 = 34 43 10 10 01
RX_PW_P0   = 14
```

If ACK payload is used, also enable:

```text
EN_AA.P0      = 1
EN_RXADDR.P0  = 1
FEATURE.EN_DPL     = 1
FEATURE.EN_ACK_PAY = 1
DYNPD.DPL_P0       = 1
```

Some NRF24L01+ compatible modules require the `ACTIVATE 0x73` command before the
`FEATURE` register can be written.

## Application Packet

All multi-byte concerns are avoided in the current protocol. Every field is one
byte.

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

Payload length must be exactly 14 bytes.

## Byte Layout

```text
Offset  Size  Field        Value / Meaning
0       1     head1        0xAA
1       1     head2        0x55
2       1     version      0x01
3       1     device_id    0x01
4       1     msg_type     0x01
5       1     seq          Rolling sequence number, 0..255
6       1     length       0x06
7       1     left_track   int8_t velocity command
8       1     right_track  int8_t velocity command
9       1     swing        int8_t velocity command
10      1     boom         int8_t velocity command
11      1     arm          int8_t velocity command
12      1     bucket       int8_t velocity command
13      1     crc          CRC8 over offsets 0..12
```

## CRC8

CRC parameters:

```text
Polynomial: 0x07
Initial:    0x00
Input:      payload bytes 0..12
Output:     one byte, stored at offset 13
```

Reference implementation:

```c
uint8_t Excavator_Crc8(const uint8_t *data, uint8_t length)
{
    uint8_t crc = 0;

    for (uint8_t i = 0; i < length; i++)
    {
        crc ^= data[i];

        for (uint8_t bit = 0; bit < 8; bit++)
        {
            if ((crc & 0x80) != 0)
            {
                crc = (uint8_t)((crc << 1) ^ 0x07);
            }
            else
            {
                crc <<= 1;
            }
        }
    }

    return crc;
}
```

## Validation Rules

The receiver should accept a packet only when all checks pass:

```text
payload length == 14
head1          == 0xAA
head2          == 0x55
version        == 0x01
device_id      == 0x01
msg_type       == 0x01
length         == 0x06
crc            == CRC8(payload[0..12])
```

If any check fails, discard the packet and do not update motor or actuator
commands from that packet.

## ACK Payload

The sender treats the receiver as connected only when it receives this ACK
payload:

```text
4F 4B
```

This is the ASCII string:

```text
OK
```

Recommended receiver behavior:

```text
1. Receive the 14-byte payload from NRF RX FIFO.
2. Validate header, version, device_id, msg_type, length, and CRC8.
3. If valid, copy the six velocity values into the receiver control state.
4. Load ACK payload "OK" for pipe 0.
5. If invalid, do not load "OK"; optionally load an error ACK only if the sender
   is updated to understand it.
```

NRF24L01 ACK payload must be available when the radio sends the hardware ACK.
Depending on the receiver driver design, the ACK payload may need to be
preloaded before the next packet arrives. A robust pattern is:

```text
Startup:
  preload ACK payload "OK" only after the receiver is ready to accept commands

On every valid packet:
  refresh/preload ACK payload "OK" for the next transaction

On receiver fault or application not ready:
  stop preloading "OK" or preload a non-OK diagnostic payload
```

With this pattern, the sender may need one packet period before the displayed
state changes to `LINK OK`.

## Command Meaning

Signed values use `0` as stop/neutral.

```text
left_track   > 0: forward,  < 0: backward, range -3..+3
right_track  > 0: forward,  < 0: backward, range -3..+3
swing        > 0: left,     < 0: right,    range -10..+10
boom         > 0: raise,    < 0: lower,    range -10..+10
arm          > 0: raise,    < 0: lower,    range -10..+10
bucket       > 0: outward,  < 0: inward,   range -10..+10
```

Controller input mapping is fixed-speed: `L1`, `R1`, `L1+R1`, and `L3+R3`
generate left turn, right turn, forward, and backward track commands. The four
joystick axes generate only zero or full signed level after their center dead
zone. The controller locks the actuator axis with the greatest initial raw
joystick displacement until that axis returns to neutral. The receiver applies
30 RPM to the tracks, 410 position counts/s to the servos, and a center-limited
5 RPM (30 motor-degrees/second) command to swing motor `0x03`. Its +/-360-degree
motor travel becomes +/-180 degrees after the 1:2 reduction. Each track stop is
reinforced by two additional zero-speed commands.

The receiver should clamp values before driving actuators:

```text
track channels:    -3..+3
joystick channels: -10..+10
```

## Sequence Number

`seq` increments once per transmitted control packet and wraps from `255` to
`0`. The receiver can use it to detect duplicate or missed packets.

Recommended use:

```text
same seq as previous valid packet: duplicate packet
seq skipped: one or more packets may have been missed
seq progressing: link is active
```

The sender currently transmits at about 30Hz, so a new valid packet should arrive
approximately every 33ms while the link is healthy.

## Failsafe

The receiver should implement a command timeout independent of radio ACK.

Recommended timeout:

```text
If no valid packet is received for 100ms to 300ms, set all velocity channels to 0.
```

This prevents stale commands from continuing to drive the excavator if the
transmitter loses power, leaves range, or fails validation.

## Minimal Receive Loop

```c
void Receiver_Loop(void)
{
    uint8_t payload[14];
    uint8_t length;

    if (Nrf_ReadPayload(payload, sizeof(payload), &length) == NRF_OK)
    {
        ControlPacket_t *packet = (ControlPacket_t *)payload;

        if (length == 14 && Receiver_ValidatePacket(packet))
        {
            ApplyVelocity(packet->left_track,
                          packet->right_track,
                          packet->swing,
                          packet->boom,
                          packet->arm,
                          packet->bucket);

            Nrf_WriteAckPayload(0, (const uint8_t *)"OK", 2);
        }
        else
        {
            ApplyVelocity(0, 0, 0, 0, 0, 0);
        }
    }

    if (Receiver_CommandTimedOut())
    {
        ApplyVelocity(0, 0, 0, 0, 0, 0);
    }
}
```
