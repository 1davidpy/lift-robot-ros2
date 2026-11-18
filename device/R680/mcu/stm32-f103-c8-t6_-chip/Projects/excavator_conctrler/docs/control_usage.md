# Excavator Control Usage

## Build

```bash
cd /home/fxx/STM32/F103C8T6_Chip
cmake --preset excavator-conctrler-release
cmake --build --preset excavator-conctrler-release
```

## Flash

```bash
/home/fxx/.local/share/stm32cube/bundles/programmer/2.22.0+st.1/bin/STM32_Programmer_CLI \
  -c port=SWD \
  -w build/excavator-conctrler-release/Projects/excavator_conctrler/excavator_conctrler.hex \
  -v \
  -rst
```

## Runtime

The control TX task sends one `ControlPacket_t` every 33ms, about 30Hz.

Screen radio status:

```text
NRF OK   - last reported TX state is OK
NRF TXF  - NRF TX failed, usually no ACK from receiver
NRF MISS - NRF module init or presence check failed
NRF BOOT - radio task is starting
LINK OK  - receiver returned ACK payload "OK"
LINK ERR - ACK payload is missing or not "OK"
```

Before the first ADC sample arrives, the TX task uses neutral defaults so the first packet is safe.

## Controls

```text
L1:       turn left at 30 RPM
R1:       turn right at 30 RPM
L1 + R1:  move forward at 30 RPM
L3 + R3:  move backward at 30 RPM
LY:       up raises the arm; down retracts the arm
LX:       left/right rotates the body
RY:       up retracts the boom; down raises the boom
RX:       left retracts the bucket; right raises the bucket
```

The joystick axes use the measured centers LX/LY/RX/RY =
1967/1967/1975/2086 and a 150-count dead zone. Motion is fixed-speed rather
than proportional: servos run at 410 counts/s (about ten seconds over
0..4095), while swing address `0x03` runs at 5 RPM (30 motor-degrees/second).
The controlled unit loads the latest center saved by the host and returns to
that center during startup; LX is
then limited to the center-relative -360 through +360 motor-degree range. The
installed 1:2 reduction converts this to -180 through +180 mechanism degrees.

Swing, boom, arm, and bucket are mutually exclusive. At motion start, the
axis with the greatest raw distance from its measured center is selected and
remains locked until it returns to the dead zone. Track-button control remains
independent.

For an end-to-end bench check, configure with
`-DEXCAVATOR_CONTROL_KEY_SELF_TEST=ON`. After boot the controller simulates L1
for 400 ms, releases it, then simulates R1 for 400 ms. This option is for an
attended test only; restore it to `OFF` before installing production firmware.

## Receiver Requirements

The receiver must match:

```text
Address:  34 43 10 10 01
Channel:  76
Rate:     250kbps
Payload:  14 bytes
CRC/ACK:  NRF hardware CRC, Auto ACK, and ACK payload enabled
```

After reading a packet, validate:

```text
head1 == 0xAA
head2 == 0x55
version == 0x01
device_id == 0x01
length == 6
crc == CRC8(packet[0..12])
```

When the packet is accepted, preload NRF ACK payload:

```c
uint8_t ack[2] = {0x4F, 0x4B}; /* "OK" */
```
