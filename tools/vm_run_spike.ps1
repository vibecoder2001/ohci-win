<#
.SYNOPSIS
  Launch a UEFI Windows 11 x64 VM with both OHCI and xHCI controllers.
  Input (keyboard/mouse) is on xHCI so the in-box usbxhci.sys handles it,
  leaving pci-ohci as an unbound device that OhciSpike.sys can claim.

.DESCRIPTION
  Boot disk is on virtio-scsi (Win11 25H2 removed in-box LSI drivers).
  Pass -VirtioIso during install so the user can F6-load vioscsi from
  the virtio-win ISO at the disk-picker step. Install + virtio-win ISOs
  ride on an ich9-ahci controller so Windows setup sees them OOTB.

  Default is -Accel whpx — on Windows QEMU picks WHPX as the default
  when Hypervisor Platform is available, and this host runs steady-state
  Windows fine under it. Pass -Accel tcg for a fresh install (WHPX
  triple-faults Win11 25H2 install-time bootmgr here).

.EXAMPLE
  # Fresh install (TCG required here for install-time bootmgr)
  .\vm_run_spike.ps1 -Image G:\Win11-qemu-ohci\Win11.qcow2 `
                     -Iso        F:\D\Win11_25H2_English_x64.iso `
                     -VirtioIso  G:\Win11-qemu-ohci\virtio-win-0.1.285.iso `
                     -Accel      tcg

.EXAMPLE
  # Post-install run (no ISOs attached, WHPX default)
  .\vm_run_spike.ps1 -Image G:\Win11-qemu-ohci\Win11.qcow2

.NOTES
  Install workflow:
    1. Boot with -Iso and -VirtioIso attached, -Accel tcg.
    2. At "Where do you want to install Windows?" click Load Driver,
       browse the virtio-win CDROM, pick vioscsi\w11\amd64. Target disk
       will appear. (Also load NetKVM and Balloon now if desired.)
    3. Complete install. Eject both ISOs and drop -Accel on next boot.

  Driver-install workflow (OhciSpike):
    1. Copy OhciSpike.sys, OhciSpike.inf, ohcispike.cat, OhciSpike.cer to C:\ohci-spike\
    2. As admin:
         certutil -addstore root            C:\ohci-spike\OhciSpike.cer
         certutil -addstore trustedpublisher C:\ohci-spike\OhciSpike.cer
         bcdedit /set testsigning on     (reboot required)
         pnputil /add-driver C:\ohci-spike\OhciSpike.inf /install
#>
[CmdletBinding()]
param(
    [string]$QemuExe      = 'C:\Program Files\qemu\qemu-system-x86_64.exe',
    [string]$OvmfCode     = 'C:\Program Files\qemu\share\edk2-x86_64-code.fd',
    [string]$OvmfVars     = 'G:\Win11-qemu-ohci\edk2-i386-vars.fd',
    [string]$Image        = 'G:\Win11-qemu-ohci\Win11.qcow2',
    # Windows install ISO. When set, attached as CDROM on AHCI port 0
    # with bootindex=0 so UEFI picks it over the disk on first boot.
    [string]$Iso          = '',
    # virtio-win ISO (paravirt drivers). Attached as a second CDROM on
    # AHCI port 1. During Win11 setup, F6-load vioscsi\w11\amd64 so the
    # installer can see the virtio-scsi boot disk.
    [string]$VirtioIso    = 'G:\Win11-qemu-ohci\virtio-win-0.1.285.iso',
    # whpx is default: QEMU on Windows auto-selects WHPX when Hypervisor
    # Platform is available and this host runs steady-state Windows fine
    # under it. Use tcg for Win11 install — install-time bootmgr
    # triple-faults under WHPX on this host.
    [ValidateSet('whpx','tcg','hax')][string]$Accel = 'whpx',
    [int]   $Cpus         = 4,
    [string]$Memory       = '6G',
    [string]$SerialLog    = 'spike-serial.log',
    [string]$DebugconLog  = 'spike-debugcon.log'
)

if (-not (Test-Path $QemuExe))  { throw "QEMU exe not found: $QemuExe" }
if (-not (Test-Path $OvmfCode)) { throw "OVMF code not found: $OvmfCode" }
if (-not (Test-Path $OvmfVars)) { throw "OVMF vars not found: $OvmfVars" }
if (-not $Image) {
    throw "Set OHCI_VM_IMAGE (or pass -Image) to a Windows 11 x64 UEFI disk image path."
}
if (-not (Test-Path $Image))                         { throw "Image not found: $Image" }
if ($Iso       -and -not (Test-Path $Iso))           { throw "ISO not found: $Iso" }
if ($VirtioIso -and -not (Test-Path $VirtioIso))     { throw "virtio-win ISO not found: $VirtioIso" }

# accel= in -machine is fine; kernel-irqchip=off is the poison. The
# userspace IRQ chip it forces runs OVMF but wrecks Windows post-install.
# Omit kernel-irqchip entirely — QEMU picks a mode Windows tolerates.
$machineOpts = if ($Accel -eq 'tcg') {
    # thread=multi makes TCG distribute vCPUs across host cores.
    'q35,accel=tcg,thread=multi'
} else {
    "q35,accel=$Accel"
}

$qemuArgs = @(
    '-machine', $machineOpts,
    '-cpu',     'Skylake-Client',
    '-smp',     "$Cpus",
    '-m',       $Memory,

    # UEFI firmware: split pflash keeps Windows's EFI NVRAM entries persistent.
    '-drive', "if=pflash,format=raw,readonly=on,file=$OvmfCode",
    '-drive', "if=pflash,format=raw,file=$OvmfVars",

    # Boot disk on virtio-scsi. Win11 25H2 has no in-box driver for this,
    # so first install requires F6-loading vioscsi from the virtio-win ISO.
    # Benefit: far less TCG overhead than AHCI/LSI, and QEMU's virtio block
    # path is the most mature. Format auto-detected from file header.
    # cache=writeback batches host writes through the page cache — on TCG
    # this is the difference between ~8 MB/s and tens-of-hundreds MB/s on
    # NVMe. Safe because a guest-crash at worst costs the guest FS journal
    # entry, and we're not doing anything durability-critical here.
    # discard=unmap lets qcow2/raw sparse files reclaim space on TRIM.
    '-drive',  "file=$Image,if=none,id=disk0,cache=writeback,aio=threads,discard=unmap",
    '-device', 'virtio-scsi-pci,id=scsi0,num_queues=4',
    '-device', 'scsi-hd,drive=disk0,bus=scsi0.0,bootindex=1',

    # Both host controllers present. Input pinned to xHCI; OHCI starts empty
    # so OhciSpike.sys can claim an unattached controller at boot.
    '-device', 'qemu-xhci,id=xhci0',
    '-device', 'pci-ohci,id=ohci0',
    '-device', 'usb-kbd,bus=xhci0.0',
    '-device', 'usb-tablet,bus=xhci0.0',

    # Networking + logging.
    '-nic',      'user,model=virtio-net-pci',
    '-serial',   "file:$SerialLog",
    '-debugcon', "file:$DebugconLog"
)

# CDROMs ride on ich9-ahci so Windows setup sees them OOTB, before any
# virtio-scsi driver is loaded.
if ($Iso -or $VirtioIso) {
    $qemuArgs += @('-device', 'ich9-ahci,id=ahci')
    $qemuArgs += @('-boot',   'menu=on,splash-time=10000')
}
if ($Iso) {
    $qemuArgs += @(
        '-drive',  "file=$Iso,if=none,id=cd0,media=cdrom,readonly=on",
        '-device', 'ide-cd,drive=cd0,bus=ahci.0,bootindex=0'
    )
}
if ($VirtioIso) {
    $qemuArgs += @(
        '-drive',  "file=$VirtioIso,if=none,id=cd1,media=cdrom,readonly=on",
        '-device', 'ide-cd,drive=cd1,bus=ahci.1'
    )
}

Write-Host "Launching QEMU ($Cpus vCPU, $Memory, UEFI, accel=$Accel). DbgPrint -> $DebugconLog."
& $QemuExe @qemuArgs
