# Lucid Atlas Sync

## Signals

```text
PA1   TIM2_CH2   10 Hz, 10% duty PWM
PB5   TIM3_CH2    1 Hz, 10% duty PPS (TIM3 partial remap)
PA9   USART1_TX  simulated GPS NMEA output, 9600 baud, 8N1
PA10  USART1_RX  floating input, RXNE interrupt, 200-byte CR/LF line buffer
PC13  GPIO_OD     1 Hz breathing cycle, 200 Hz software PWM carrier
```

TIM2 is the 10 Hz master time base. Each TIM2 update event clocks TIM3 once,
so TIM3 completes one cycle for every ten TIM2 cycles. The PB5 rising edge is
therefore aligned in hardware with every tenth PA1 rising edge and cannot drift
relative to it. The 1 Hz compare value is preloaded on the first synchronized
update, suppressing an unaligned PB5 startup edge.

Once per 1 Hz cycle, USART1 sends a simulated GPS RMC sentence:

```text
$GPRMC,000001.00,A,2237.496474,N,11356.089515,E,0.0,225.5,230520,2.3,W,A*hh<CR><LF>
```

UTC advances once per second and wraps after 23:59:59. `hh` is the calculated
NMEA XOR checksum. The UART pin, baud rate, framing, coordinates, date, and
navigation fields match the migrated Keil implementation.

## Build in VS Code or PowerShell

From the workspace root:

```powershell
cmake --preset lucid-atlas-sync-debug
cmake --build --preset lucid-atlas-sync-debug

cmake --preset lucid-atlas-sync-release
cmake --build --preset lucid-atlas-sync-release
```

Generated `.elf`, `.hex`, `.bin`, and `.map` files are placed in the matching
`build/lucid-atlas-sync-*` directory.
