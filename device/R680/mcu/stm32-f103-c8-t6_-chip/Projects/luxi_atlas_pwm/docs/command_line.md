# Luxi Atlas PWM Command Line

## Output Pins

```text
PA1   TIM2_CH2   10 Hz, 10% duty PWM
PB5   TIM3_CH2    1 Hz, 10% duty PPS (TIM3 partial remap)
PA9   USART1_TX  GPS NMEA output, 9600 baud, 8N1
PC13  GPIO_OD     1 Hz breathing cycle, 200 Hz software PWM carrier
```

TIM2 is the 10 Hz master timer. Its update event is routed internally to TIM3,
which counts ten TIM2 periods per 1 Hz cycle. The PB5 rising edge is therefore
aligned with every tenth PA1 rising edge. Both PWM signals are generated in
hardware.

Once per PB5 cycle, USART1 transmits a simulated GPS RMC sentence in this form:

```text
$GPRMC,000001.00,A,2237.496474,N,11356.089515,E,0.0,225.5,230520,2.3,W,A*hh<CR><LF>
```

UTC time advances once per second and wraps after 23:59:59. `hh` is replaced
with the calculated NMEA XOR checksum. The coordinates and other navigation
fields follow the reference `stm32_timersync-open` implementation.

PC13 has no timer PWM alternate function on STM32F103C8T6, so the active-low
board LED uses open-drain software PWM. A 200 Hz carrier is modulated from 0%
to 100% and back to 0% once per second to provide a smooth 1 Hz breathing
cycle.

## Build

```bash
cd /home/fxx/STM32/F103C8T6_Chip
cmake --preset luxi-atlas-pwm-release
cmake --build --preset luxi-atlas-pwm-release
```

Debug build:

```bash
cd /home/fxx/STM32/F103C8T6_Chip
cmake --preset luxi-atlas-pwm-debug
cmake --build --preset luxi-atlas-pwm-debug
```

## Flash

Release:

```bash
cd /home/fxx/STM32/F103C8T6_Chip
/home/fxx/.local/share/stm32cube/bundles/programmer/2.22.0+st.1/bin/STM32_Programmer_CLI \
  -c port=SWD \
  -w build/luxi-atlas-pwm-release/Projects/luxi_atlas_pwm/luxi_atlas_pwm.hex \
  -v \
  -rst
```

Debug:

```bash
cd /home/fxx/STM32/F103C8T6_Chip
/home/fxx/.local/share/stm32cube/bundles/programmer/2.22.0+st.1/bin/STM32_Programmer_CLI \
  -c port=SWD \
  -w build/luxi-atlas-pwm-debug/Projects/luxi_atlas_pwm/luxi_atlas_pwm.hex \
  -v \
  -rst
```
