# Build instructions

## Userspace test harness (Tier 1)

Requires CMake 3.22+ and a C11 compiler (MSVC, clang, or gcc).

```
cmake -S . -B build
cmake --build build --config Debug
ctest --test-dir build -C Debug --output-on-failure
```

On Windows with MSVC installed via Visual Studio, CMake selects the VS
generator automatically. No special PATH setup is needed if CMake is
installed system-wide.

## Kernel driver (WDK)

`tools/build_driver.ps1` works with either:

1. **EWDK** — point `EWDKDIR` at the root of your EWDK mount (the folder
   containing `BuildEnv\SetupBuildEnv.cmd`). This is the preferred option
   for CI because the EWDK is self-contained and side-effect-free.
2. **Visual Studio 2022 + WDK** — any edition (Community, Professional,
   Enterprise, Build Tools) with the Windows Driver Kit workload
   installed. No env var required; the locator script autodetects.

Build a driver project:

```
.\tools\build_driver.ps1 -Project spike\ucx_spike\OhciSpike.vcxproj -Arch x64 -Config Debug
```

Supported `-Arch` values: `x64` (QEMU test VM), `ARM64` (RK3588 target).
