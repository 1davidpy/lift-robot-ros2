[CmdletBinding()]
param()

$ErrorActionPreference = 'Stop'

$stm32ToolRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
$stm32SourceRoot = Join-Path $stm32ToolRoot 'extension'
$stm32ManifestPath = Join-Path $stm32SourceRoot 'package.json'
$stm32Manifest = Get-Content -LiteralPath $stm32ManifestPath -Raw -Encoding UTF8 | ConvertFrom-Json
$stm32PackageRoot = Join-Path $stm32ToolRoot 'package'
$stm32VsixPath = Join-Path $stm32PackageRoot "stm32-project-tools-$($stm32Manifest.version).vsix"
. (Join-Path $stm32ToolRoot 'tool-paths.ps1')
$stm32NpxCommand = Resolve-Stm32NodeCommand -Name npx
$stm32NodeCommand = Resolve-Stm32NodeCommand -Name node

New-Item -ItemType Directory -Force -Path $stm32PackageRoot | Out-Null

Push-Location $stm32SourceRoot
$stm32OriginalPath = $env:PATH
try {
    $env:PATH = (Split-Path -Parent $stm32NodeCommand) + [IO.Path]::PathSeparator + $env:PATH
    & $stm32NpxCommand --yes '@vscode/vsce' package --no-dependencies --out $stm32VsixPath
    if ($LASTEXITCODE -ne 0) {
        throw "VSIX packaging failed with exit code $LASTEXITCODE."
    }
}
finally {
    $env:PATH = $stm32OriginalPath
    Pop-Location
}

if (-not (Test-Path -LiteralPath $stm32VsixPath -PathType Leaf)) {
    throw "Packaging completed without producing: $stm32VsixPath"
}

$stm32Package = Get-Item -LiteralPath $stm32VsixPath
Write-Host "Created $($stm32Package.FullName) ($($stm32Package.Length) bytes)."
