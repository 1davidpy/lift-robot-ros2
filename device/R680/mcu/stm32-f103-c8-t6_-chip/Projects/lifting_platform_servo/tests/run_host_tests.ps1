param(
    [Parameter(Mandatory=$true)][string]$Compiler,
    [switch]$CompileOnly
)
$ErrorActionPreference = 'Stop'
$repo = (Resolve-Path (Join-Path $PSScriptRoot '../../..')).Path
$outputFile = Join-Path $env:TEMP 'lift-position-test.exe'
$protocolOutputFile = Join-Path $repo 'build/uart-protocol-host-test.exe'
Push-Location $repo
try {
    if (-not (Test-Path -LiteralPath (Join-Path $repo 'build'))) {
        New-Item -ItemType Directory -Path (Join-Path $repo 'build') | Out-Null
    }
    & $Compiler '-IProjects/lifting_platform_servo/tests/host' '-IProjects/lifting_platform_servo/Inc' `
        '-ICommon/BSP/Components/ISL60PR' '-ICommon/BSP/Peripherals/KEY' `
        '-ICommon/BSP/Peripherals/LED' '-ICommon/BSP/Peripherals/BrakeRelay' `
        '-ICommon/BSP/Peripherals/PositionSensor' '-ICommon/BSP/Peripherals/UART' `
        'Projects/lifting_platform_servo/tests/host/position_test.c' -o $outputFile
    if ($LASTEXITCODE -ne 0) { throw 'Position/controller host test compilation failed' }
    Write-Output 'COMPILE OK: position/controller/parser/profile/calibration host suite.'
    & $Compiler '-IProjects/lifting_platform_servo/Inc' `
        'Projects/lifting_platform_servo/tests/host/uart_protocol_test.c' `
        'Projects/lifting_platform_servo/Src/lift_uart_protocol.c' -o $protocolOutputFile
    if ($LASTEXITCODE -ne 0) { throw 'UART protocol host test compilation failed' }
    Write-Output 'COMPILE OK: UART protocol wire-contract host suite.'
    if ($CompileOnly) {
        Write-Output 'NOT RUN: -CompileOnly selected. Compilation is not a runtime test pass.'
        return
    }
    & $outputFile
    if ($LASTEXITCODE -ne 0) { throw 'Position/controller host tests failed' }
    & $protocolOutputFile
    if ($LASTEXITCODE -ne 0) { throw 'UART protocol host tests failed' }
    Write-Output 'EXECUTION PASS: both host suites completed successfully; no hardware was exercised.'
} finally { Pop-Location }
