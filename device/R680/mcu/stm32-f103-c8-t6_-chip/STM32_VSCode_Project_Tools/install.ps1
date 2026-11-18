[CmdletBinding()]
param(
    [string]$ProfileName = 'STM32'
)

$ErrorActionPreference = 'Stop'

$stm32ToolRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
$stm32ManifestPath = Join-Path $stm32ToolRoot 'extension\package.json'
$stm32Manifest = Get-Content -LiteralPath $stm32ManifestPath -Raw -Encoding UTF8 | ConvertFrom-Json
$stm32ExtensionId = "$($stm32Manifest.publisher).$($stm32Manifest.name)"
$stm32VsixPath = Join-Path $stm32ToolRoot "package\stm32-project-tools-$($stm32Manifest.version).vsix"

if (-not (Test-Path -LiteralPath $stm32VsixPath -PathType Leaf)) {
    throw "VSIX not found: $stm32VsixPath"
}

$stm32CodeInfo = Get-Command code -ErrorAction Stop
$stm32CodeCommand = if ($stm32CodeInfo.Source) { $stm32CodeInfo.Source } else { $stm32CodeInfo.Name }
$stm32InstallArgs = @()
if (-not [string]::IsNullOrWhiteSpace($ProfileName)) {
    $stm32InstallArgs += @('--profile', $ProfileName)
}
$stm32InstallArgs += @('--install-extension', $stm32VsixPath, '--force')

Write-Host "Installing $stm32ExtensionId@$($stm32Manifest.version)..."
& $stm32CodeCommand @stm32InstallArgs
if ($LASTEXITCODE -ne 0) {
    throw "VS Code extension installation failed with exit code $LASTEXITCODE."
}

$stm32ListArgs = @()
if (-not [string]::IsNullOrWhiteSpace($ProfileName)) {
    $stm32ListArgs += @('--profile', $ProfileName)
}
$stm32ListArgs += @('--list-extensions', '--show-versions')
$stm32Installed = @(& $stm32CodeCommand @stm32ListArgs)
if ($LASTEXITCODE -ne 0) {
    throw "Unable to verify the installed extension."
}

$stm32Expected = "$stm32ExtensionId@$($stm32Manifest.version)"
if ($stm32Installed -notcontains $stm32Expected) {
    throw "Installation verification failed: $stm32Expected was not listed."
}

$stm32ProfileLabel = if ([string]::IsNullOrWhiteSpace($ProfileName)) { 'default profile' } else { "profile '$ProfileName'" }
Write-Host "Verified $stm32Expected in $stm32ProfileLabel."
Write-Host 'Reload the VS Code window before using the Explorer context menu.'
