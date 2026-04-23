<#
.SYNOPSIS
  Launch a test VM with pci-ohci attached, ready to install OhciSpike.sys.

.DESCRIPTION
  Assumes a pre-made Windows 11 x64 VHDX at $env:OHCI_VM_IMAGE and a
  QEMU install on PATH. Boots with debugcon to serial for DbgPrint capture.

.NOTES
  Before running, on the guest:
    1. Copy OhciSpike.sys, OhciSpike.inf, OhciSpike.cat to C:\ohci-spike\
    2. Run as admin: pnputil /add-driver C:\ohci-spike\OhciSpike.inf /install
    3. Enable test-signing: bcdedit /set testsigning on
    4. Shut down the guest before running this script from the host
#>
[CmdletBinding()]
param(
    [string]$Image        = $env:OHCI_VM_IMAGE,
    [string]$SerialLog    = "spike-serial.log",
    [string]$DebugconLog  = "spike-debugcon.log"
)

if (-not $Image) {
    throw "Set OHCI_VM_IMAGE to a Windows 11 x64 VHDX path."
}
if (-not (Test-Path $Image)) {
    throw "Image not found: $Image"
}

$qemuArgs = @(
    '-machine', 'q35,accel=whpx,kernel-irqchip=off',
    '-cpu', 'max',
    '-m', '4G',
    '-drive', "file=$Image,if=virtio",
    '-device', 'pci-ohci,id=ohci0',
    '-device', 'usb-kbd,bus=ohci0.0',
    '-serial', "file:$SerialLog",
    '-debugcon', "file:$DebugconLog",
    '-net', 'nic,model=virtio',
    '-net', 'user'
)

Write-Host "Launching QEMU. Kernel DbgPrint output will be captured to $DebugconLog."
& qemu-system-x86_64 @qemuArgs
