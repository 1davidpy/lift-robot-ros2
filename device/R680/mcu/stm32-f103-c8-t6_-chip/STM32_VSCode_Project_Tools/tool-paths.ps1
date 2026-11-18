function Resolve-Stm32NodeCommand {
    [CmdletBinding()]
    param([ValidateSet('node', 'npx')][string]$Name = 'node')

    $command = Get-Command $Name -ErrorAction SilentlyContinue
    if ($command) {
        if ($command.Source) { return $command.Source }
        return $command.Name
    }

    $roots = @($env:CUBE_BUNDLE_PATH)
    if ($env:LOCALAPPDATA) {
        $roots += Join-Path $env:LOCALAPPDATA 'stm32cube\bundles'
    }
    foreach ($root in ($roots | Where-Object { $_ } | Select-Object -Unique)) {
        $nodeRoot = Join-Path $root 'node'
        if (-not (Test-Path -LiteralPath $nodeRoot -PathType Container)) { continue }
        $versions = Get-ChildItem -LiteralPath $nodeRoot -Directory | Sort-Object -Property @{
            Expression = { try { [version](($_.Name -split '\+')[0]) } catch { [version]'0.0' } }
            Descending = $true
        }
        foreach ($version in $versions) {
            foreach ($relative in @('node\bin', 'bin', 'node', '')) {
                $directory = Join-Path $version.FullName $relative
                $fileName = if ($Name -eq 'node') { 'node.exe' } else { 'npx.cmd' }
                $candidate = Join-Path $directory $fileName
                if (Test-Path -LiteralPath $candidate -PathType Leaf) { return $candidate }
            }
        }
    }
    throw "Cannot find $Name. Install Node.js or the STM32Cube node bundle, or set CUBE_BUNDLE_PATH."
}
