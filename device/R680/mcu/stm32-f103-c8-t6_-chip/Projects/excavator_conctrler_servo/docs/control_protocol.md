# Full-servo wireless protocol

The packet remains compatible at the field level with the existing excavator
control protocol. Only the device identity and NRF address are separated.

```text
Offset  Size  Field
0       1     0xAA
1       1     0x55
2       1     version = 0x01
3       1     device_id = 0x02
4       1     message_type = 0x01
5       1     sequence
6       1     payload_length = 6
7       1     left_track   (-6..6; magnitude is chassis speed level)
8       1     right_track  (-6..6; magnitude is chassis speed level)
9       1     swing        (-10..10, receiver ID4)
10      1     boom         (-10..10, receiver ID1)
11      1     arm          (-10..10, receiver ID2)
12      1     bucket       (-10..10, receiver ID3)
13      1     CRC8
```

CRC8 uses polynomial `0x07`, initial value zero, over bytes 0 through 12.
The receiver returns the two-byte ACK payload `OK` after accepting a valid
packet. If no valid packet is accepted for 200 ms, the receiver issues a stop
command to all logical axes.
