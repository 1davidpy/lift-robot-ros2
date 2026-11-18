param(
    [string]$Compiler = 'C:/Qt/Tools/mingw1310_64/bin/gcc.exe',
    [switch]$CompileOnly
)
$ErrorActionPreference = 'Stop'
$env:PATH = (Split-Path -Parent $Compiler) + ';' + $env:PATH
$workspace = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '../../..'))
# MinGW handles ASCII relative arguments reliably under a non-ASCII workspace path.
Push-Location -LiteralPath $workspace
try {
$workspace = '.'
$testRoot = 'Projects/excavator_controlled_servo/tests'
$generated = Join-Path $workspace 'build/excavator-controlled-servo-debug/Projects/excavator_controlled_servo/generated'
if (-not (Test-Path (Join-Path $generated 'excavator_machine_limits.h'))) {
    throw 'Configure excavator-controlled-servo-debug first.'
}
$outputDir = Join-Path $workspace 'build/excavator-servo-host-tests'
New-Item -ItemType Directory -Force $outputDir | Out-Null
$binary = Join-Path $outputDir 'servo_calibration_test.exe'
$includes = @(
    "$testRoot/stubs", "$testRoot/../Inc", "$testRoot/../App/Tasks",
    "$testRoot/../App/Protocol", $generated,
    "$workspace/Common/BSP/Peripherals/LED", "$workspace/Common/BSP/Peripherals/UART",
    "$workspace/Common/BSP/Components/Feetech_STS3215"
)
$compilerArgs = @('-std=c11', '-Wall', '-Wextra', '-Werror', '-DEXCAVATOR_POSITION_REMOTE_MODE=0')
foreach ($include in $includes) { $compilerArgs += '-I' + $include }
$compilerArgs += @("$testRoot/test_servo_calibration.c", "$testRoot/../Src/excavator_servo_config.c",
    "$testRoot/../App/Protocol/excavator_position_control.c", '-o', $binary)
& $Compiler @compilerArgs
if ($LASTEXITCODE -ne 0) { throw 'Host test compilation failed.' }
if (-not $CompileOnly) {
    & $binary
    if ($LASTEXITCODE -ne 0) { throw 'Host calibration tests failed.' }
}
$remoteBinary = Join-Path $outputDir 'remote_control_test.exe'
$remoteArgs = @('-std=c11', '-Wall', '-Wextra', '-Werror')
foreach ($include in $includes) { $remoteArgs += '-I' + $include }
$remoteArgs += @("$testRoot/test_remote_control.c", "$testRoot/../Src/excavator_servo_config.c",
    "$testRoot/../App/Protocol/excavator_position_control.c", '-o', $remoteBinary)
& $Compiler @remoteArgs
if ($LASTEXITCODE -ne 0) { throw 'Host remote control compilation failed.' }
if (-not $CompileOnly) {
    & $remoteBinary
    if ($LASTEXITCODE -ne 0) { throw 'Host remote control tests failed.' }
}
$busBinary = Join-Path $outputDir 'servo_bus_test.exe'
& $Compiler '-std=c11' '-Wall' '-Wextra' '-Werror' "-I$testRoot/driver_stubs" `
    "-I$workspace/Common/BSP/Components/Feetech_STS3215" `
    "$testRoot/test_servo_bus.c" "$workspace/Common/BSP/Components/Feetech_STS3215/bsp_sts3215.c" `
    '-o' $busBinary
if ($LASTEXITCODE -ne 0) { throw 'Host bus test compilation failed.' }
if (-not $CompileOnly) {
    & $busBinary
    if ($LASTEXITCODE -ne 0) { throw 'Host bus tests failed.' }
}
$chassisBinary = Join-Path $outputDir 'servo_bus_chassis_test.exe'
& $Compiler '-std=c11' '-Wall' '-Wextra' '-Werror' '-DTEST_CHASSIS_WRITE_MODE=1' `
    "-I$testRoot/driver_stubs" "-I$workspace/Common/BSP/Components/Feetech_STS3215" `
    "$testRoot/test_servo_bus.c" "$workspace/Common/BSP/Components/Feetech_STS3215/bsp_sts3215.c" `
    '-o' $chassisBinary
if ($LASTEXITCODE -ne 0) { throw 'Host chassis bus test compilation failed.' }
if (-not $CompileOnly) {
    & $chassisBinary
    if ($LASTEXITCODE -ne 0) { throw 'Host chassis bus tests failed.' }
}
$manualBinary = Join-Path $outputDir 'servo_bus_calibration_test.exe'
& $Compiler '-std=c11' '-Wall' '-Wextra' '-Werror' '-DTEST_CHASSIS_WRITE_MODE=1' '-DTEST_CALIBRATION_WRITE_MODE=1' `
    "-I$testRoot/driver_stubs" "-I$workspace/Common/BSP/Components/Feetech_STS3215" `
    "$testRoot/test_servo_bus.c" "$workspace/Common/BSP/Components/Feetech_STS3215/bsp_sts3215.c" `
    '-o' $manualBinary
if ($LASTEXITCODE -ne 0) { throw 'Host manual calibration bus compilation failed.' }
if (-not $CompileOnly) {
    & $manualBinary
    if ($LASTEXITCODE -ne 0) { throw 'Host manual calibration bus tests failed.' }
}
} finally {
    Pop-Location
}
