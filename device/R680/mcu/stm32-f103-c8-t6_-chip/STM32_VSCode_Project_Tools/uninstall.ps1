[CmdletBinding()]
param(
    [string]$ProfileName = 'STM32'
)

$ErrorActionPreference = 'Stop'

$stm32ToolRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
$stm32ManifestPath = Join-Path $stm32ToolRoot 'extension\package.json'
$stm32Manifest = Get-Content -LiteralPath $stm32ManifestPath -Raw -Encoding UTF8 | ConvertFrom-Json
$stm32ExtensionId = "$($stm32Manifest.publisher).$($stm32Manifest.name)"
$stm32CodeInfo = Get-Command code -ErrorAction Stop
$stm32CodeCommand = if ($stm32CodeInfo.Source) { $stm32CodeInfo.Source } else { $stm32CodeInfo.Name }
$stm32UninstallArgs = @()
if (-not [string]::IsNullOrWhiteSpace($ProfileName)) {
    $stm32UninstallArgs += @('--profile', $ProfileName)
}
$stm32UninstallArgs += @('--uninstall-extension', $stm32ExtensionId)

& $stm32CodeCommand @stm32UninstallArgs
if ($LASTEXITCODE -ne 0) {
    throw "VS Code extension uninstall failed with exit code $LASTEXITCODE."
}

Write-Host "Uninstalled $stm32ExtensionId. Reload the VS Code window to apply the change."
