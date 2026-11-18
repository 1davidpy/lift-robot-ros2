# STM32 工程编译、烧录与固件切换指南

本文中的路径都相对于仓库根目录，不依赖某一台电脑的用户名或 STM32Cube 工具版本。优先使用 VS Code 右键菜单；命令行方式用于自动化和排错。

## 1. 当前工程与预设

| 用途 | 工程目录 / `PROJECT_BOARD` | Debug 预设 | Release 预设 | 固件目标名 |
|---|---|---|---|---|
| 框架基础示例 | `F103C8T6_Chip` | `f103c8-chip-debug` | `f103c8-chip-release` | `F103C8T6_Chip` |
| 挖机遥控端 | `excavator_conctrler` | `excavator-conctrler-debug` | `excavator-conctrler-release` | `excavator_conctrler` |
| 挖机被控端 | `excavator_controlled` | `excavator-controlled-debug` | `excavator-controlled-release` | `excavator_controlled` |
| USB 转串口测试 | `uart_test` | `uart-test-debug` | `uart-test-release` | `uart_test` |
| Atlas PWM/GPS | `luxi_atlas_pwm` | `luxi-atlas-pwm-debug` | `luxi-atlas-pwm-release` | `luxi_atlas_pwm` |

`excavator_conctrler` 是仓库现有名称，虽然拼写特殊，但路径、preset 和构建命令中必须保持一致。

## 2. VS Code 右键编译和烧录

安装 `STM32 Project Tools` 后：

1. 在 VS Code 中打开仓库根目录。
2. 右键 `Projects/<工程>`，例如 `Projects/excavator_conctrler`。
3. “STM32: 编译 Debug + Release”会依次构建该工程的 Debug 和 Release。
4. “STM32: 选择固件并烧录”会让用户选择固件版本及已连接的 ST-Link。
5. 最终确认框会再次显示工程、固件和探针，确认后才执行烧录。

扩展自动从 `CMakePresets.json` 查找预设，并在 STM32Cube bundle 目录及 `PATH` 中查找 CMake 和 STM32CubeProgrammer。

## 3. 固件输出位置

每个 preset 使用独立目录：

```text
build/<preset>/<target>.elf
build/<preset>/<target>.hex
build/<preset>/<target>.bin
```

例如遥控端 Debug 固件：

```text
build/excavator-conctrler-debug/excavator_conctrler.hex
```

`build/` 被 `.gitignore` 忽略，属于本机生成结果。克隆到新电脑后需要重新编译。

## 4. 手工命令行编译

确保 CMake、Ninja 和 GNU Arm 编译器在 `PATH` 中，然后在仓库根目录运行：

```powershell
cmake --preset excavator-conctrler-debug
cmake --build --preset excavator-conctrler-debug --parallel

cmake --preset excavator-conctrler-release
cmake --build --preset excavator-conctrler-release --parallel
```

其他工程只需要把 preset 替换为第 1 节表格中的对应名称。不要让不同工程共用同一个构建目录，也不要在已有构建目录中手工改变 `PROJECT_BOARD`。

如果 CMake 未加入 `PATH`，可以使用 STM32Cube 安装目录中的完整路径，或在 VS Code 设置中填写 `stm32ProjectTools.cmakePath`。

## 5. 手工命令行烧录

先确认 `STM32_Programmer_CLI.exe` 的真实安装路径，再执行：

```powershell
$stm32Programmer = 'C:\path\to\STM32_Programmer_CLI.exe'
$stm32Firmware = (Resolve-Path '.\build\excavator-conctrler-debug\excavator_conctrler.hex').Path

& $stm32Programmer -c port=SWD -w $stm32Firmware -v -rst
if ($LASTEXITCODE -ne 0) { throw 'Flash failed.' }
```

多个 ST-Link 同时连接时，先查询探针：

```powershell
& $stm32Programmer -l
```

然后明确指定序列号：

```powershell
& $stm32Programmer -c port=SWD sn=<ST-LINK序列号> -w $stm32Firmware -v -rst
```

不要把某块 ST-Link 的序列号硬编码进通用脚本或提交到工程配置中。

## 6. 安全注意事项

- 烧录前核对所选工程、Debug/Release、固件时间戳和 ST-Link 序列号。
- 必须保留 `-v` 校验，并在成功后使用 `-rst` 复位运行。
- 烧录期间不要拔掉 ST-Link、目标板电源或 SWD 线。
- 如果提示没有探针，先检查供电、USB 数据线、驱动、SWDIO/SWCLK/GND 和目标芯片连接。
- 如果连接不稳定，可尝试降低 SWD 频率或使用 under-reset 连接，但不要在未确认目标时解除读保护或整片擦除。
- 单片机只运行最后一次烧录的固件；切换工程就是重新编译并烧录另一个工程的 `.hex`。

## 7. 常见问题

### 右键菜单不存在

扩展通常装到了错误的 VS Code Profile。检查当前窗口 Profile，并执行：

```powershell
code --profile STM32 --list-extensions --show-versions | Select-String 'luxi-embedded.stm32-project-tools'
```

随后重新加载 VS Code 窗口，并确认右键的是 `Projects` 的直接子工程目录。

### 找不到 Debug 或 Release preset

检查 `CMakePresets.json`：configure preset 的 `PROJECT_BOARD` 必须等于工程目录名，且相应 build preset 不能被隐藏。

### 找不到固件

先查看构建输出是否成功，再检查 preset 的 `binaryDir` 下是否产生 `.hex`。不要从另一个工程的构建目录选择同名或旧固件。

### 改了代码但行为没变化

核对编译日志和固件修改时间，必要时删除该工程对应的单个 `build/<preset>` 目录后重新配置。不要删除整个仓库或工作区根目录。

## 8. 新增工程规则

新增 `Projects/<ProjectName>/` 后，需要：

1. 提供 `Inc/`、`Src/`、`Startup/`、`Linker/` 和 `CMakeLists.txt` 等工程内容。
2. 在 `CMakePresets.json` 添加独立的 Debug/Release configure preset 和 build preset。
3. 将 `PROJECT_BOARD` 设置为 `<ProjectName>`。
4. 保证输出目标名与工程目录名一致。
5. 分别执行 Debug 和 Release 构建验证。

满足这些约定后，不需要修改扩展源码，新工程会自动获得相同的右键编译和烧录功能。
