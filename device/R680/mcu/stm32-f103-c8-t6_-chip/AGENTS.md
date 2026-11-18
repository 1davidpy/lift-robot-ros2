# Codex Project Instructions

This repository is a reusable STM32F103C8T6 FreeRTOS workspace. Follow this structure when adding or changing code.

Chinese version: `AGENTS.zh-CN.md`. When updating project structure, placement rules, naming rules, or verification commands, update both files in the same change.

## Project Layout

```text
F103C8T6_Chip/
├── Common/
│   ├── BSP/
│   │   ├── Board/
│   │   ├── Peripherals/
│   │   │   ├── Analog/
│   │   │   ├── LED/
│   │   │   ├── KEY/
│   │   │   ├── OLED/
│   │   │   ├── SENSOR/
│   │   │   ├── PositionSensor/
│   │   │   └── BrakeRelay/
│   │   ├── Protocols/
│   │   │   ├── SoftI2C/
│   │   │   ├── SoftSPI/
│   │   │   └── OneWire/
│   │   └── Components/
│   │       ├── OLED_SSD1306/
│   │       ├── LCD_ST7789/
│   │       ├── NRF24L01/
│   │       ├── EEPROM_AT24Cxx/
│   │       └── Flash_W25Qxx/
│   ├── Drivers/
│   └── AppShared/
│       └── LVGL/
├── Libraries/
│   ├── CMSIS/
│   └── STM32F10x_StdPeriph_Driver/
├── Middlewares/
│   ├── FreeRTOS-Kernel/
│   └── LVGL/
├── Projects/
│   └── F103C8T6_Chip/
│       ├── App/
│       │   ├── RTOS/
│       │   └── Tasks/
│       ├── Inc/
│       ├── Src/
│       ├── Startup/
│       ├── Linker/
│       └── CMakeLists.txt
├── cmake/
├── build/
├── CMakeLists.txt
└── CMakePresets.json
```

## Placement Rules

- Put project-specific FreeRTOS task modules in `Projects/<ProjectName>/App/Tasks/`.
- Put project-specific task creation and scheduler startup code in `Projects/<ProjectName>/App/RTOS/`.
- Put reusable board-wide init code in `Common/BSP/Board/`.
- Put project-specific board pin mapping headers in `Projects/<ProjectName>/Inc/`.
- Put board hardware peripherals in `Common/BSP/Peripherals/<PeripheralName>/`.
- Put bit-banged or software bus/protocol implementations in `Common/BSP/Protocols/<ProtocolName>/`.
- Put reusable device/component drivers in `Common/BSP/Components/<DeviceName>/`.
- Put generic reusable MCU drivers that are not board-specific in `Common/Drivers/`.
- Put shared application utilities that are reusable across projects and are not tasks and not BSP in `Common/AppShared/`.
- Put common LVGL configuration and thin port wrappers in `Common/AppShared/LVGL/`.
- Put project-specific startup files, linker scripts, config headers, pin mapping headers, syscall code, and `main.c` under `Projects/<ProjectName>/`.
- Do not place new application code directly under `Libraries/` or `Middlewares/`; those directories are vendor/middleware code.
- Put LVGL vendor source in `Middlewares/LVGL/lvgl/`. Keep LVGL disabled by default; projects must opt in explicitly before LVGL sources or port code are compiled.

## Decoupling Rules

- Code outside `Projects/<ProjectName>/` must stay reusable across projects.
- Common code must not hard-code a specific project name, board name, or concrete project path.
- Common BSP, drivers, components, protocols, tasks, and shared app code may include project-provided config headers only through include paths, not relative project paths.
- Project-specific choices such as pin maps, enabled peripherals, FreeRTOS task sets, task priorities, task creation order, heap sizes, startup files, linker scripts, and chip capacity macros belong in `Projects/<ProjectName>/`.

## BSP Rules

- All board pin definitions must be centralized in a project-specific header such as `Projects/<ProjectName>/Inc/bsp_pin.h`.
- Common BSP modules may include `bsp_pin.h`, but the file must be supplied by the active project include path.
- Tasks must not directly manipulate GPIO registers or STM32 GPIO APIs for board peripherals.
- Task code should call BSP or component APIs, such as `BSP_LED_Toggle(...)`, not `GPIO_SetBits(...)`.
- Peripheral BSP modules should translate board pin definitions into standard library calls.
- Component drivers should avoid hard-coded STM32 pins and should depend on IO callbacks or protocol abstractions where practical.
- Component drivers that need board pins may include project-provided headers such as `bsp_pin.h`, but must not define concrete ports or pins inside the common component.
- Optional common components, such as `LCD_ST7789`, must stay disabled by default in CMake and be enabled explicitly by a project option.
- Hardware SPI device components, such as `NRF24L01`, must keep chip pins in the active project's `bsp_pin.h`; common code may provide the SPI/register driver but must not own project-specific radio addresses or application packet formats.
- GPIO key peripherals must keep pin maps in the active project's `bsp_pin.h`; common key BSP code may initialize GPIOs and board-level remaps, while debounce, long-press, and combinations belong in the project task layer.

## FreeRTOS Rules

- Each task should have its own `<name>_task.c` and `<name>_task.h` under `Projects/<ProjectName>/App/Tasks/`.
- Each task header should define task name, stack size, priority, and a `*_TaskCreate(void)` function.
- Register task creation in `Projects/<ProjectName>/App/RTOS/freertos.c`.
- Keep `Projects/<ProjectName>/Src/main.c` minimal: board init, FreeRTOS app init, scheduler start.
- Tune task stack sizes and `configTOTAL_HEAP_SIZE` in the project-specific `FreeRTOSConfig.h`.

## LVGL Rules

- Do not call `lv_init()`, `LVGL_PortInit()`, or create LVGL tasks from common code.
- A project that uses LVGL must enable it explicitly with CMake option `ENABLE_LVGL=ON`.
- Project-specific LVGL tasks belong in `Projects/<ProjectName>/App/Tasks/`.
- Display, touch, and input hardware glue belongs in BSP peripheral/component modules and should be connected to LVGL through callbacks.
- Keep reusable LVGL helpers in `Common/AppShared/LVGL/`; keep screen-specific UI code inside the project that owns that UI.

## NRF24L01 Rules

- Put the reusable NRF24L01 register and hardware SPI driver in `Common/BSP/Components/NRF24L01/`.
- Enable the common NRF24L01 component with CMake option `ENABLE_NRF24L01=ON`.
- Keep SPI instance, SPI speed, CE, CSN, IRQ, SCK, MISO, and MOSI pin definitions in `Projects/<ProjectName>/Inc/bsp_pin.h`.
- Keep radio addresses, packet layouts, retry policy choices, and FreeRTOS communication tasks in the project that owns the wireless protocol.
- Start with polling APIs for board bring-up; add EXTI IRQ dispatch in the project layer when receive latency matters.

## Key Rules

- Put reusable GPIO key initialization and raw key reads in `Common/BSP/Peripherals/KEY/`.
- Enable the common key peripheral with CMake option `ENABLE_KEYS=ON`.
- Keep key debounce, long-press detection, multi-key state handling, and UI feedback in project tasks.
- When using PA15, PB3, or PB4 as keys on STM32F103, disable JTAG and keep SWD enabled in the key BSP.

## Project Rules

- The active project is selected by the `PROJECT_BOARD` CMake cache variable.
- Add a new project by creating `Projects/<ProjectName>/` with `Inc/`, `Src/`, `Startup/`, `Linker/`, and `CMakeLists.txt`.
- Add matching configure/build presets in `CMakePresets.json`.
- Keep target output names aligned with the project directory name.

## Naming

- BSP public APIs use `BSP_<Peripheral>_<Action>()`.
- Task create functions use `<NAME>_TaskCreate()`.
- Common headers use include guards matching their module name.
- Prefer ASCII-only source edits unless an existing file already uses non-ASCII.

## Comment Style

- Use English comments in common reusable modules.
- Use block comments in Doxygen form:

```c
/**
 * @brief Summary.
 * @param value Parameter meaning.
 * @return Return value meaning.
 */
```

- Use line comments in Doxygen trailing form, such as `///< description`.
- Public callable APIs in common modules should have concise Doxygen comments.
- Prefer `@brief`, `@param`, and `@return` for simple functions.
- Use `@warning` for important caveats and `@exception` only when an exception-like object is actually documented.
- Do not over-comment tiny internal helpers when the implementation is already clear.

## Verification

After structural or code changes, build both presets:

```bash
cmake --preset f103c8-chip-debug
cmake --build --preset f103c8-chip-debug
cmake --preset f103c8-chip-release
cmake --build --preset f103c8-chip-release
```

For `lifting_platform_servo`, an additional optional host compile check covers
the controller/calibration/profile suite and the pure CAN wire protocol:

```powershell
./Projects/lifting_platform_servo/tests/run_host_tests.ps1 -Compiler '<path-to-tcc.exe>' -CompileOnly
```

`-CompileOnly` does not execute the host binaries and must not be reported as
runtime test success. Omit it only where local application-control policy allows
the binaries to run; do not change security policy or relocate binaries to bypass
a rejection. This optional check does not replace the baseline builds above or
the active project's Debug/Release builds and hardware validation.

For `excavator_controlled_servo`, configure its Debug preset first, then optionally
run `./Projects/excavator_controlled_servo/tests/run_host_tests.ps1 -CompileOnly`.
This compiles the seven-servo calibration parser/state tests with simulated IO;
omit `-CompileOnly` only when local policy permits execution. Do not change
execution policy to run the script. Qt checks run through
`Projects/Qt_centers/scripts/build.cmd`; serial smoke tests require hardware and
are not included in the automated checks.
Use `qt_centers_serial_smoke.exe <COM-port> --protocol-only` for serial protocol
checks with offline servos; this does not validate servo discovery or mechanical
limits. Omit the flag for the full seven-servo hardware check.
The Qt build script deploys `QtCenters.exe` and its runtime DLLs, plugins, and
config copy directly into `Projects/Qt_centers/` for double-click startup.
Keep these generated deployment files ignored by that project's `.gitignore`.
The user has authorized Config2 remote control and requested normal speed after
the hardware test: IDs1/2/3/8 currently use250/500/200/200, ID4 uses1000 and
IDs5/6 use3000. Receiver increments are5/10/4/4 per20ms, at most10 per update.
Position goals may lead fresh feedback by at most96 through a separate remote API;
planning reserves4 counts for resampling. Startup adopts loaded retained holds
only within16 counts and the reference/limit checks; it still rejects old centers.
Qt calibration remains limited to10 per fresh read. Recovery may enter the safe
range in small inward steps; it must not stall just outside a boundary.
The canonical profile is
Projects/excavator_controlled_servo/config/config2.json; CMake compiles its
limits and coordinate-reference metadata into Flash. Qt defaults to the same
file and must preserve reference_mode/encoder_offset when saving. Radio v2
has seven channels and a15-byte packet; RX maps to ID8, single BL/BR to ID3,
and BL+BR together reverses the tracks with pitch stopped, as authorized by the user. There is no
speed-selection key. Never send an endpoint or old center on startup.
Startup reads feedback; SYNC_WRITE remains blocked. The user requested autonomous
power-on recovery on 2026-10-21. After stable neutral input/feedback for1s, prepare
unpowered servos into their proper modes and latch fresh current holds, never an
endpoint or a retained distant goal. Do not unload a powered wrong-mode axis.
Keep explicit maintenance pauses, offset validation and neutral re-arming.
Explicit preparation
verifies torque off, waits for stable mode/feedback, checks reference-position
invariance and latches the current goal before torque on. The user approved
supported, neutral preparation; automatic motion tests remain forbidden.
Validate the stored encoder offset before applying coordinate conversion.
Outside a limit allow only small inward recovery; never snap to the boundary.
Qt manual motion pauses radio, is capped at100 and10 per step, and cannot widen
the Flash limits. CLEAR restores Flash defaults. Radio loss stops and requires
neutral for1s before arming again.
Host tests also cover the ID/register restrictions, setup ordering, receive
buffering, corrupt packets, echo, neutral arming and radio-loss stops.
