<#
.SYNOPSIS
  Locate a WDK build environment (EWDK or VS+WDK) and return its
  environment-setup .cmd/.bat path.

.DESCRIPTION
  Searches in this order:
    1. $env:EWDKDIR (user override)
    2. Common EWDK mount roots: C:\EWDK, D:\EWDK, E:\EWDK
    3. Visual Studio 2022 installs with VsDevCmd.bat on C: or D:
  Fails with a descriptive error if nothing is found.

  Returns a single absolute path. The consumer (build_driver.ps1) wraps it
  with `call "<path>"` inside cmd.exe so the environment applies before
  msbuild is invoked.
#>
[CmdletBinding()]
param()

# 1) Explicit EWDK override
if ($env:EWDKDIR) {
    $candidates = @(
        (Join-Path $env:EWDKDIR 'BuildEnv\SetupBuildEnv.cmd'),
        (Join-Path $env:EWDKDIR 'LaunchBuildEnv.cmd')
    )
    foreach ($c in $candidates) {
        if (Test-Path $c) { return (Resolve-Path $c).Path }
    }
}

# 2) Common EWDK mount roots
foreach ($root in @('C:\EWDK','D:\EWDK','E:\EWDK')) {
    if (-not (Test-Path (Split-Path $root))) { continue }
    foreach ($rel in @('BuildEnv\SetupBuildEnv.cmd','LaunchBuildEnv.cmd')) {
        $p = Join-Path $root $rel
        if (Test-Path $p) { return (Resolve-Path $p).Path }
    }
}

# 3) Visual Studio 2022 fallback (any edition on C: or D:)
$vsRoots = @(
    'C:\Program Files\Microsoft Visual Studio\2022',
    'C:\Program Files (x86)\Microsoft Visual Studio\2022',
    'D:\Microsoft Visual Studio\2022'
)
$vsEditions = @('Enterprise','Professional','Community','BuildTools','Preview')
foreach ($root in $vsRoots) {
    foreach ($ed in $vsEditions) {
        $p = Join-Path $root "$ed\Common7\Tools\VsDevCmd.bat"
        if (Test-Path $p) { return (Resolve-Path $p).Path }
    }
}

throw "No WDK build environment found. Either set EWDKDIR to your EWDK mount root (containing BuildEnv\SetupBuildEnv.cmd) or install Visual Studio 2022 with the Windows Driver Kit workload."
