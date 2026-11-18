# UART SDK session scheduling

`tools/lift_uart_client.py` starts a serial I/O worker after `connect()`. It
services the 100 ms keepalive and parses replies/telemetry independently of
the command caller and `on_sample`/`on_ack` callbacks. Slow logging or a caller
pause no longer starves the MCU's 500 ms session lease. Arrival uses two
consecutive fresh device samples rather than callback timing; old samples are
bounded and coalesced for the caller.

For arrival, the earlier sample must have been fresh on receipt and the newest
sample must still be fresh at the check. Re-aging both samples after a slow
callback incorrectly prevents arrival confirmation with 100 ms UART telemetry
and normal 150-250 ms motor-feedback age. The sample-sequence, device-uptime
continuity, speed, position, fault and brake checks remain required.

Keep one command caller per client. Always call `close()` in `finally`, before
closing the serial port. Do not open the port while the ROS adapter owns it.

```python
device = LiftClient(port)
try:
    device.connect()
    device.move_to_mm("20.000", on_sample=record_sample)
finally:
    device.close()
    port.close()
```

The SDK still abandons the session on missing keepalive ACK (300 ms), missing
complete telemetry (500 ms), device restart, drive fault or motion guard. It
sends CLOSE when possible and never reconnects or repeats a motion command
automatically. A worker error is reported by the next caller operation. A
process/USB failure still relies on the unchanged MCU session watchdog.

No MCU firmware change or flashing is required for this SDK scheduling fix.

Host regression tests, without hardware:

```sh
python3 Projects/lifting_platform_servo/tests/test_uart_client.py
python3 Projects/lifting_platform_servo/tests/test_uart_session_worker.py
```
