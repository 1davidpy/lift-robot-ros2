# Codex 项目规则

本仓库是一个可长期复用的 STM32F103C8T6 FreeRTOS 工程工作区。添加或修改代码时必须遵守本文档中的目录和分层规则。

英文版本：`AGENTS.md`。当修改工程结构、代码放置规则、命名规则或验证命令时，必须在同一次修改中同步更新中英文两个文件。

## 工程结构

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

## 代码放置规则

- 具体工程的 FreeRTOS 任务模块放在 `Projects/<ProjectName>/App/Tasks/`。
- 具体工程的任务创建和调度器启动代码放在 `Projects/<ProjectName>/App/RTOS/`。
- 可复用的板级初始化代码放在 `Common/BSP/Board/`。
- 具体工程的板级引脚映射头文件放在 `Projects/<ProjectName>/Inc/`。
- 板载硬件外设驱动放在 `Common/BSP/Peripherals/<PeripheralName>/`。
- 软件模拟总线或软件协议放在 `Common/BSP/Protocols/<ProtocolName>/`。
- 可复用器件或组件驱动放在 `Common/BSP/Components/<DeviceName>/`。
- 不绑定具体板子的通用 MCU 驱动放在 `Common/Drivers/`。
- 可被多个工程复用、且不属于任务也不属于 BSP 的共享应用工具放在 `Common/AppShared/`。
- 公共 LVGL 配置和薄封装适配层放在 `Common/AppShared/LVGL/`。
- 项目专属的启动文件、链接脚本、配置头文件、引脚映射头文件、syscall 代码和 `main.c` 放在 `Projects/<ProjectName>/`。
- 不要把新的应用代码直接放到 `Libraries/` 或 `Middlewares/`，这两个目录用于厂商库和中间件源码。
- LVGL 厂商源码放在 `Middlewares/LVGL/lvgl/`。LVGL 默认保持禁用，具体工程必须显式选择启用后，才允许参与编译。

## 解耦规则

- `Projects/<ProjectName>/` 之外的代码必须保持可被多个工程复用。
- 公共代码不得硬编码具体工程名、板子名或具体工程路径。
- 公共 BSP、驱动、组件、协议、任务和共享应用代码可以通过 include path 包含工程提供的配置头文件，但不得使用相对路径直接引用某个工程目录。
- 引脚表、启用的外设、FreeRTOS 任务集合、任务优先级、任务创建顺序、堆大小、启动文件、链接脚本、芯片容量宏等具体工程选择，应放在 `Projects/<ProjectName>/`。

## BSP 规则

- 所有板级引脚定义必须集中在工程专属头文件中，例如 `Projects/<ProjectName>/Inc/bsp_pin.h`。
- 公共 BSP 模块可以包含 `bsp_pin.h`，但该文件必须由当前活动工程的 include path 提供。
- 任务层不得直接操作 GPIO 寄存器，也不得直接调用 STM32 GPIO API 控制板载外设。
- 任务代码应调用 BSP 或组件 API，例如使用 `BSP_LED_Toggle(...)`，而不是 `GPIO_SetBits(...)`。
- BSP 外设模块负责把板级引脚定义转换为标准库调用。
- 组件驱动应尽量避免硬编码 STM32 引脚，优先依赖 IO 回调或协议抽象。
- 需要板级引脚的组件驱动可以包含工程提供的 `bsp_pin.h`，但不得在公共组件内部定义具体端口和引脚。
- `LCD_ST7789` 这类可选公共组件在 CMake 中必须默认禁用，由具体工程通过选项显式启用。
- `NRF24L01` 这类硬件 SPI 器件组件，片选和控制引脚必须放在当前工程的 `bsp_pin.h` 中；公共层只提供 SPI/寄存器驱动，不承载具体工程的无线地址或业务包格式。
- GPIO 按键外设的引脚映射必须放在当前工程的 `bsp_pin.h` 中；公共 KEY BSP 可以负责 GPIO 初始化和板级复用配置，消抖、长按、组合键识别应放在具体工程任务层。

## FreeRTOS 规则

- 每个任务应有独立的 `<name>_task.c` 和 `<name>_task.h`，并放在 `Projects/<ProjectName>/App/Tasks/`。
- 每个任务头文件应定义任务名、栈大小、优先级，以及 `*_TaskCreate(void)` 创建函数。
- 任务创建应统一注册在 `Projects/<ProjectName>/App/RTOS/freertos.c`。
- `Projects/<ProjectName>/Src/main.c` 应保持最小化，只做板级初始化、FreeRTOS 应用初始化和启动调度器。
- 任务栈大小和 `configTOTAL_HEAP_SIZE` 在项目专属的 `FreeRTOSConfig.h` 中调整。

## LVGL 规则

- 公共代码不得调用 `lv_init()`、`LVGL_PortInit()`，也不得创建 LVGL 任务。
- 使用 LVGL 的工程必须通过 CMake 选项 `ENABLE_LVGL=ON` 显式启用。
- 项目专属 LVGL 任务放在 `Projects/<ProjectName>/App/Tasks/`。
- 显示屏、触摸、输入设备等硬件适配放在 BSP 外设或组件模块中，并通过回调接入 LVGL。
- 可复用 LVGL 工具放在 `Common/AppShared/LVGL/`；具体界面和业务 UI 代码放在所属工程内部。

## NRF24L01 规则

- 可复用的 NRF24L01 寄存器和硬件 SPI 驱动放在 `Common/BSP/Components/NRF24L01/`。
- 公共 NRF24L01 组件通过 CMake 选项 `ENABLE_NRF24L01=ON` 启用。
- SPI 实例、SPI 速度、CE、CSN、IRQ、SCK、MISO、MOSI 引脚定义放在 `Projects/<ProjectName>/Inc/bsp_pin.h`。
- 无线地址、数据包格式、重发策略选择和 FreeRTOS 通信任务放在实际使用无线协议的工程内部。
- 板级调试阶段优先使用轮询 API；对接收延迟有要求时，再在工程层添加 EXTI 中断分发。

## KEY 规则

- 可复用的 GPIO 按键初始化和原始读取放在 `Common/BSP/Peripherals/KEY/`。
- 公共 KEY 外设通过 CMake 选项 `ENABLE_KEYS=ON` 启用。
- 按键消抖、长按检测、多键同时状态处理和 UI 反馈放在具体工程任务中。
- STM32F103 使用 PA15、PB3、PB4 作为按键时，应在 KEY BSP 中关闭 JTAG 并保留 SWD。

## 项目规则

- 当前活动工程通过 CMake 缓存变量 `PROJECT_BOARD` 选择。
- 新增工程时，创建 `Projects/<ProjectName>/`，并包含 `Inc/`、`Src/`、`Startup/`、`Linker/` 和 `CMakeLists.txt`。
- 新增工程时，需要在 `CMakePresets.json` 中添加对应的 configure/build preset。
- 目标输出文件名应和项目目录名保持一致。

## 命名规则

- BSP 对外 API 使用 `BSP_<Peripheral>_<Action>()`。
- 任务创建函数使用 `<NAME>_TaskCreate()`。
- 公共头文件 include guard 应和模块名匹配。
- 除非文件已有非 ASCII 内容，否则源码编辑默认使用 ASCII。

## 注释规范

- 通用可复用模块中的注释使用英文。
- 块注释使用 Doxygen 格式：

```c
/**
 * @brief Summary.
 * @param value Parameter meaning.
 * @return Return value meaning.
 */
```

- 行注释使用 Doxygen 尾随格式，例如 `///< description`。
- 通用模块中可被外部调用的 public API 应编写简洁 Doxygen 注释。
- 简单函数优先只使用 `@brief`、`@param`、`@return`。
- 重要注意事项使用 `@warning`，只有确实需要说明异常对象时才使用 `@exception`。
- 实现已经清晰的内部小 helper 不要过度注释。

## 验证

结构或代码修改后，构建 Debug 和 Release 两个 preset：

```bash
cmake --preset f103c8-chip-debug
cmake --build --preset f103c8-chip-debug
cmake --preset f103c8-chip-release
cmake --build --preset f103c8-chip-release
```

对于 `lifting_platform_servo`，可额外执行主机编译检查，覆盖控制器、标定、速度规划及 CAN 帧协议：

```powershell
./Projects/lifting_platform_servo/tests/run_host_tests.ps1 -Compiler '<path-to-tcc.exe>' -CompileOnly
```

`-CompileOnly` 不执行主机测试程序，不得将编译成功报告为运行测试通过。只有本机应用控制策略允许执行时才省略该参数；不得修改安全策略或移动程序路径来绕过拦截。此可选检查不能替代上述基础工程构建、当前工程的 Debug/Release 构建或真机验证。

对于 `excavator_controlled_servo`，先配置其 Debug preset，然后可选执行
`./Projects/excavator_controlled_servo/tests/run_host_tests.ps1 -CompileOnly`。
该检查通过模拟 IO 编译七舵机标定解析与状态测试；仅在本机策略允许时省略
`-CompileOnly`，不得修改执行策略来运行脚本。Qt 自动检查通过
`Projects/Qt_centers/scripts/build.cmd` 执行；串口 smoke 测试需要硬件，
不包含在自动检查中。
舵机离线时可用 `qt_centers_serial_smoke.exe <COM口> --protocol-only` 检查串口协议，
该模式不验证舵机在线或机械限位；省略参数才执行完整七舵机硬件检查。
Qt构建脚本将 `QtCenters.exe`、运行库DLL、插件和配置副本直接部署到
`Projects/Qt_centers/`，便于双击启动；这些生成的部署文件由工程 `.gitignore` 排除。
用户已授权配置2遥控，并在实机后要求正常速度：当前1/2/3/8号250/500/200/200，
4号1000、5/6号3000。每20ms递增5/10/4/4，单次更新不超过10。
独立遥控接口允许目标最多领先实时反馈96，规划留4计数采样裕量；Qt标定仍不超过10。
启动接管带载保持目标时允许16以内偏差，仍核对坐标、限位并拒绝旧中心。
越界时可小步向内进入安全范围，不得因边界夹紧停在范围外。
规范配置源为Projects/excavator_controlled_servo/config/config2.json，CMake将限位及参考坐标
元数据编译到Flash；Qt默认同一文件，保存须保留reference_mode/encoder_offset。
无线v2使用七通道15字节包：RX对应8号、单按BL/BR对应3号，不设按键调速。
用户已授权BL+BR同时按后退，此时俯仰停止。
启动先读反馈，不发送行程终点或旧中心，SYNC_WRITE仍禁用。用户于2026-10-21要求
上电独立恢复：输入中立且反馈稳定1秒后，将未带力矩舵机准备为正确模式并锁定实时位置；
不自动卸掉错误模式下仍带力矩的轴。保留显式维护暂停、偏移校验和准备后的中立允许。
显式准备须验证卸力、
等待模式/反馈稳定、核对参考坐标切换前后不变，并以新读数写入目标再恢复力矩。
用户已确认支撑机构及遥控中立，可做此准备；自动运动测试仍禁止。
换算坐标前须核对舵机偏移与配置相符；越界只允许小步向内回退，不跳至限位。
Qt手动操作暂停遥控，速度不超过100、单步不超过10，不得放宽Flash限位；CLEAR恢复
Flash默认值。无线失联停止，重新允许前须中立1秒。
主机测试还覆盖ID/寄存器限制、准备顺序、接收缓冲、损坏帧、回显、中立使能和失联停止。
