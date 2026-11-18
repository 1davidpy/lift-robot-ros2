# excavator_controlled 编译与烧录命令

以下命令适用于本机 PowerShell，可以整段复制执行。

## 工程和工具路径

```powershell
$ProjectRoot = 'C:\Users\embedded\Desktop\luxi_excavator\stm32-f103-c8-t6_-chip'
Set-Location $ProjectRoot

$ArmGccBin = 'C:\Users\embedded\AppData\Local\stm32cube\bundles\gnu-tools-for-stm32\14.3.1+st.2\bin'
$CMakeExe = 'C:\Users\embedded\AppData\Local\stm32cube\bundles\cmake\4.3.1+st.1\bin\cmake.exe'
$ProgrammerExe = 'C:\Users\embedded\AppData\Local\stm32cube\bundles\programmer\2.23.0\bin\STM32_Programmer_CLI.exe'
$StLinkSerialNumber = 'E1007200D0D2139393740544'
$env:PATH = "$ArmGccBin;$env:PATH"
```

每次新开 PowerShell 后，先执行上面的路径设置，再执行下面的命令。

## 编译 Debug 固件

```powershell
$ProjectRoot = 'C:\Users\embedded\Desktop\luxi_excavator\stm32-f103-c8-t6_-chip'
$ArmGccBin = 'C:\Users\embedded\AppData\Local\stm32cube\bundles\gnu-tools-for-stm32\14.3.1+st.2\bin'
$CMakeExe = 'C:\Users\embedded\AppData\Local\stm32cube\bundles\cmake\4.3.1+st.1\bin\cmake.exe'
$env:PATH = "$ArmGccBin;$env:PATH"
Set-Location $ProjectRoot
& $CMakeExe --preset excavator-controlled-debug
if ($LASTEXITCODE -ne 0) { throw 'Debug configure failed' }

& $CMakeExe --build --preset excavator-controlled-debug
if ($LASTEXITCODE -ne 0) { throw 'Debug build failed' }
```

生成文件：

```text
build\excavator-controlled-debug\excavator_controlled.elf
build\excavator-controlled-debug\excavator_controlled.hex
build\excavator-controlled-debug\excavator_controlled.bin
```

## 编译 Release 固件

```powershell
$ProjectRoot = 'C:\Users\embedded\Desktop\luxi_excavator\stm32-f103-c8-t6_-chip'
$ArmGccBin = 'C:\Users\embedded\AppData\Local\stm32cube\bundles\gnu-tools-for-stm32\14.3.1+st.2\bin'
$CMakeExe = 'C:\Users\embedded\AppData\Local\stm32cube\bundles\cmake\4.3.1+st.1\bin\cmake.exe'
$env:PATH = "$ArmGccBin;$env:PATH"
Set-Location $ProjectRoot
& $CMakeExe --preset excavator-controlled-release
if ($LASTEXITCODE -ne 0) { throw 'Release configure failed' }

& $CMakeExe --build --preset excavator-controlled-release
if ($LASTEXITCODE -ne 0) { throw 'Release build failed' }
```

生成文件：

```text
build\excavator-controlled-release\excavator_controlled.elf
build\excavator-controlled-release\excavator_controlled.hex
build\excavator-controlled-release\excavator_controlled.bin
```

## 查看 ST-Link

```powershell
$ProjectRoot = 'C:\Users\embedded\Desktop\luxi_excavator\stm32-f103-c8-t6_-chip'
$ProgrammerExe = 'C:\Users\embedded\AppData\Local\stm32cube\bundles\programmer\2.23.0\bin\STM32_Programmer_CLI.exe'
Set-Location $ProjectRoot
& $ProgrammerExe -l probe
if ($LASTEXITCODE -ne 0) { throw 'ST-Link probe query failed' }
```

## 烧录 Debug 固件

```powershell
$ProjectRoot = 'C:\Users\embedded\Desktop\luxi_excavator\stm32-f103-c8-t6_-chip'
$ProgrammerExe = 'C:\Users\embedded\AppData\Local\stm32cube\bundles\programmer\2.23.0\bin\STM32_Programmer_CLI.exe'
$StLinkSerialNumber = 'E1007200D0D2139393740544'
Set-Location $ProjectRoot
$DebugHex = Join-Path $ProjectRoot 'build\excavator-controlled-debug\excavator_controlled.hex'
if (-not (Test-Path -LiteralPath $DebugHex)) { throw "Firmware not found: $DebugHex" }

& $ProgrammerExe -c "port=SWD" "sn=$StLinkSerialNumber" -w $DebugHex -v -rst
if ($LASTEXITCODE -ne 0) { throw 'Debug flash or verify failed' }
```

## 烧录 Release 固件

```powershell
$ProjectRoot = 'C:\Users\embedded\Desktop\luxi_excavator\stm32-f103-c8-t6_-chip'
$ProgrammerExe = 'C:\Users\embedded\AppData\Local\stm32cube\bundles\programmer\2.23.0\bin\STM32_Programmer_CLI.exe'
$StLinkSerialNumber = 'E1007200D0D2139393740544'
Set-Location $ProjectRoot
$ReleaseHex = Join-Path $ProjectRoot 'build\excavator-controlled-release\excavator_controlled.hex'
if (-not (Test-Path -LiteralPath $ReleaseHex)) { throw "Firmware not found: $ReleaseHex" }

& $ProgrammerExe -c "port=SWD" "sn=$StLinkSerialNumber" -w $ReleaseHex -v -rst
if ($LASTEXITCODE -ne 0) { throw 'Release flash or verify failed' }
```

`-v` 表示烧录后校验，`-rst` 表示校验成功后复位 MCU。开发调试使用 Debug，最终运行可烧录 Release。

## 一次完成 Debug 编译和烧录

```powershell
$ProjectRoot = 'C:\Users\embedded\Desktop\luxi_excavator\stm32-f103-c8-t6_-chip'
Set-Location $ProjectRoot

$ArmGccBin = 'C:\Users\embedded\AppData\Local\stm32cube\bundles\gnu-tools-for-stm32\14.3.1+st.2\bin'
$CMakeExe = 'C:\Users\embedded\AppData\Local\stm32cube\bundles\cmake\4.3.1+st.1\bin\cmake.exe'
$ProgrammerExe = 'C:\Users\embedded\AppData\Local\stm32cube\bundles\programmer\2.23.0\bin\STM32_Programmer_CLI.exe'
$StLinkSerialNumber = 'E1007200D0D2139393740544'
$DebugHex = Join-Path $ProjectRoot 'build\excavator-controlled-debug\excavator_controlled.hex'
$env:PATH = "$ArmGccBin;$env:PATH"

& $CMakeExe --preset excavator-controlled-debug
if ($LASTEXITCODE -ne 0) { throw 'Debug configure failed' }

& $CMakeExe --build --preset excavator-controlled-debug
if ($LASTEXITCODE -ne 0) { throw 'Debug build failed' }

& $ProgrammerExe -c "port=SWD" "sn=$StLinkSerialNumber" -w $DebugHex -v -rst
if ($LASTEXITCODE -ne 0) { throw 'Debug flash or verify failed' }
```

## 安全检查上位机串口

该命令只发送 `PING` 和 `STATUS`，不会发送舵机运动指令。USB 转串口当前为 COM3、115200、8N1。

```powershell
$Serial = [System.IO.Ports.SerialPort]::new('COM3', 115200, [System.IO.Ports.Parity]::None, 8, [System.IO.Ports.StopBits]::One)
$Serial.ReadTimeout = 500
$Serial.WriteTimeout = 500

try {
    $Serial.Open()
    Start-Sleep -Milliseconds 300
    [void]$Serial.ReadExisting()
    $Serial.Write("PING`nSTATUS`n")
    Start-Sleep -Milliseconds 300
    $Serial.ReadExisting()
}
finally {
    if ($Serial.IsOpen) { $Serial.Close() }
    $Serial.Dispose()
}
```

正常结果应包含 `PONG` 和 `STATUS`。`STATUS` 的第一个数字是在线掩码：三轴全部在线时为 `7`，全部离线时为 `0`。

## 安全说明

- 烧录本身不需要舵机动力电源，首次调试建议断开舵机动力后烧录。
- 本固件上电不会发送位置运动命令。
- 舵机重新上电后，固件先把轮式模式速度写为零，再确认工作模式为位置模式；未确认成功的轴不会接受运动命令。
- 不要在未设置机械限位时直接发送大位置变化的 `MOVE` 或高速度 `JOG`。
- 串口协议详见同目录下的 `serial_arm_protocol.md`。
