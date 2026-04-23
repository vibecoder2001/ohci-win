<#
.SYNOPSIS
  Build a driver project under an EWDK or VS+WDK environment.

.EXAMPLE
  .\tools\build_driver.ps1 -Project spike\ucx_spike\OhciSpike.vcxproj -Arch x64 -Config Debug
#>
[CmdletBinding()]
param(
    [Parameter(Mandatory)][string]$Project,
    [ValidateSet('x64','ARM64')][string]$Arch = 'x64',
    [ValidateSet('Debug','Release')][string]$Config = 'Debug'
)

$ErrorActionPreference = 'Stop'

if (-not (Test-Path $Project)) {
    throw "Project not found: $Project"
}

$envScript = & (Join-Path $PSScriptRoot 'ewdk_env.ps1')
Write-Host "Using build env: $envScript"

$projectFull = (Resolve-Path $Project).Path

# If we landed on VsDevCmd.bat, tell it which host/target arch to configure.
# For EWDK scripts (SetupBuildEnv.cmd / LaunchBuildEnv.cmd) we pass no args;
# msbuild's /p:Platform drives the output arch.
# host_arch is the build machine's arch, not the target arch. Assume x64 host.
$envInvoke = if ($envScript -like '*VsDevCmd.bat') {
    "call `"$envScript`" -arch=$Arch -host_arch=x64 -no_logo"
} else {
    "call `"$envScript`""
}

$cmd = @(
    $envInvoke,
    "msbuild `"$projectFull`" /p:Configuration=$Config /p:Platform=$Arch /nologo /m"
) -join ' && '

cmd /c $cmd
if ($LASTEXITCODE -ne 0) {
    throw "Driver build failed (exit $LASTEXITCODE)."
}
