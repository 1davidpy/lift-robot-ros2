# Config2 seven-servo controller

Paired only with excavator_controlled_servo. Radio protocol v2 has seven
signed channels in a15-byte packet. Update both endpoints together; old
version1 packets are rejected.

| Input | Receiver action |
| --- | --- |
| LX | ID4 rotation1000; release stops |
| LY | ID2 small arm |
| RY | ID1 large arm |
| RX | ID8 gripper opening |
| BL / BR | ID3 pitch decreasing / increasing |
| BL + BR | Tracks backward; pitch stop; overrides L1/R1 |
| L1 | Tracks turn left |
| R1 | Tracks turn right |
| L1 + R1 | Tracks forward |

Receiver position speeds are250/500/200/200 for IDs1/2/3/8. Targets increment by5/10/4/4
per20ms, with a96-count fresh-feedback guard and4 counts reserved for resampling.
Release holds fresh feedback. Qt calibration remains capped at100 and10 counts.
BR never selects speed. Track magnitude stays3000; fixed levelV3.
Single BL controls pitch; the BL+BR chord reverses tracks. Within each joystick, the larger
displacement axis takes priority, preserving the existing joystick rule.
Both joysticks and pitch buttons can act independently.

Controller title LUXI CONFIG2. At startup require1s stopped command and
released buttons. Receiver independently requires1s neutral after reset,
stop, resume or radio loss, and validates all position axes, coordinate
offsets and retained goals. Never command a stale center or full endpoint.

```text
Device ID: 0x02
Version: 0x02
NRF address: 34 43 10 10 02
Channel: 76
Rate: 250kbps
Payload: 15 bytes
Period: 33ms
```

Build excavator-conctrler-servo-debug/release. The receiver uses permanent
config/config2.json limits and coordinate metadata; its prep/control guide
is ../excavator_controlled_servo/docs/config2_control.md.

SWD read-only diagnostics: g_wireless_last_status=5 means connected,
g_control_adc_diagnostics is LY/LX/RY/RX/left-wheel/right-wheel, and
g_control_key_diagnostics is the key mask. Reading does not inject commands.
