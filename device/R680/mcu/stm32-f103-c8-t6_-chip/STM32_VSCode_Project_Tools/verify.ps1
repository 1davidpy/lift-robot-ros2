[CmdletBinding()]
param()

$ErrorActionPreference = 'Stop'

$stm32ToolRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
$stm32SourceRoot = Join-Path $stm32ToolRoot 'extension'
$stm32ManifestPath = Join-Path $stm32SourceRoot 'package.json'
$stm32Manifest = Get-Content -LiteralPath $stm32ManifestPath -Raw -Encoding UTF8 | ConvertFrom-Json
$stm32ExtensionId = "$($stm32Manifest.publisher).$($stm32Manifest.name)"
$stm32VsixPath = Join-Path $stm32ToolRoot "package\stm32-project-tools-$($stm32Manifest.version).vsix"
. (Join-Path $stm32ToolRoot 'tool-paths.ps1')
$stm32NodeCommand = Resolve-Stm32NodeCommand -Name node

$stm32RequiredCommands = @(
    'stm32ProjectTools.buildProject',
    'stm32ProjectTools.flashProject'
)
$stm32DeclaredCommands = @($stm32Manifest.contributes.commands | ForEach-Object { $_.command })
foreach ($stm32RequiredCommand in $stm32RequiredCommands) {
    if ($stm32DeclaredCommands -notcontains $stm32RequiredCommand) {
        throw "Missing command in package.json: $stm32RequiredCommand"
    }
}

& $stm32NodeCommand --check (Join-Path $stm32SourceRoot 'extension.js')
if ($LASTEXITCODE -ne 0) {
    throw 'extension.js syntax check failed.'
}

& $stm32NodeCommand (Join-Path $stm32SourceRoot 'test\core.test.js')
if ($LASTEXITCODE -ne 0) {
    throw 'Core unit tests failed.'
}

& $stm32NodeCommand (Join-Path $stm32SourceRoot 'test\extension.test.js')
if ($LASTEXITCODE -ne 0) {
    throw 'Extension command tests failed.'
}

if (-not (Test-Path -LiteralPath $stm32VsixPath -PathType Leaf)) {
    throw "Current VSIX is missing: $stm32VsixPath"
}

$stm32Vsix = Get-Item -LiteralPath $stm32VsixPath
Write-Host "Verified $stm32ExtensionId@$($stm32Manifest.version)."
Write-Host "VSIX: $($stm32Vsix.FullName) ($($stm32Vsix.Length) bytes)"
