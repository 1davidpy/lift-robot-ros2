# excavator_conctrler Command Line Guide

Run all commands from the workspace root:

```bash
cd /home/fxx/STM32/F103C8T6_Chip
```

## Toolchain PATH

If `arm-none-eabi-size` or `arm-none-eabi-objcopy` cannot be found, prepend the STM32Cube toolchain path:

```bash
export PATH=/home/fxx/.local/share/stm32cube/bundles/gnu-tools-for-stm32/14.3.1+st.2/bin:$PATH
```

## Configure

Debug:

```bash
cmake --preset excavator-conctrler-debug
```

Release:

```bash
cmake --preset excavator-conctrler-release
```

## Build

Debug:

```bash
cmake --build --preset excavator-conctrler-debug
```

Release:

```bash
cmake --build --preset excavator-conctrler-release
```

## Output Files

Debug output:

```text
build/excavator-conctrler-debug/Projects/excavator_conctrler/excavator_conctrler.elf
build/excavator-conctrler-debug/Projects/excavator_conctrler/excavator_conctrler.hex
build/excavator-conctrler-debug/Projects/excavator_conctrler/excavator_conctrler.bin
```

Release output:

```text
build/excavator-conctrler-release/Projects/excavator_conctrler/excavator_conctrler.elf
build/excavator-conctrler-release/Projects/excavator_conctrler/excavator_conctrler.hex
build/excavator-conctrler-release/Projects/excavator_conctrler/excavator_conctrler.bin
```

## Detect ST-LINK

```bash
/home/fxx/.local/share/stm32cube/bundles/programmer/2.22.0+st.1/bin/STM32_Programmer_CLI -l
```

## Flash Debug HEX

```bash
/home/fxx/.local/share/stm32cube/bundles/programmer/2.22.0+st.1/bin/STM32_Programmer_CLI \
  -c port=SWD \
  -w build/excavator-conctrler-debug/Projects/excavator_conctrler/excavator_conctrler.hex \
  -v \
  -rst
```

## Flash Release HEX

```bash
/home/fxx/.local/share/stm32cube/bundles/programmer/2.22.0+st.1/bin/STM32_Programmer_CLI \
  -c port=SWD \
  -w build/excavator-conctrler-release/Projects/excavator_conctrler/excavator_conctrler.hex \
  -v \
  -rst
```

## Flash BIN

When flashing a `.bin` file, specify the Flash start address `0x08000000`.

Debug:

```bash
/home/fxx/.local/share/stm32cube/bundles/programmer/2.22.0+st.1/bin/STM32_Programmer_CLI \
  -c port=SWD \
  -w build/excavator-conctrler-debug/Projects/excavator_conctrler/excavator_conctrler.bin 0x08000000 \
  -v \
  -rst
```

Release:

```bash
/home/fxx/.local/share/stm32cube/bundles/programmer/2.22.0+st.1/bin/STM32_Programmer_CLI \
  -c port=SWD \
  -w build/excavator-conctrler-release/Projects/excavator_conctrler/excavator_conctrler.bin 0x08000000 \
  -v \
  -rst
```

## One-Shot Build And Flash

Debug:

```bash
export PATH=/home/fxx/.local/share/stm32cube/bundles/gnu-tools-for-stm32/14.3.1+st.2/bin:$PATH
cmake --preset excavator-conctrler-debug
cmake --build --preset excavator-conctrler-debug
/home/fxx/.local/share/stm32cube/bundles/programmer/2.22.0+st.1/bin/STM32_Programmer_CLI \
  -c port=SWD \
  -w build/excavator-conctrler-debug/Projects/excavator_conctrler/excavator_conctrler.hex \
  -v \
  -rst
```

Release:

```bash
export PATH=/home/fxx/.local/share/stm32cube/bundles/gnu-tools-for-stm32/14.3.1+st.2/bin:$PATH
cmake --preset excavator-conctrler-release
cmake --build --preset excavator-conctrler-release
/home/fxx/.local/share/stm32cube/bundles/programmer/2.22.0+st.1/bin/STM32_Programmer_CLI \
  -c port=SWD \
  -w build/excavator-conctrler-release/Projects/excavator_conctrler/excavator_conctrler.hex \
  -v \
  -rst
```

