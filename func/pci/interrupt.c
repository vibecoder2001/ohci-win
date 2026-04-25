#include <ntddk.h>
#include <wdf.h>
#include "device_context.h"
#include "ohci_drain.h"
#include "ohci_regs.h"

#define LOG(fmt, ...) DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL, \
                                  "OhciPci: " fmt "\n", ##__VA_ARGS__)

#define REG_HcInterruptStatus  0x0C
#define REG_HcInterruptEnable  0x10
#define REG_HcInterruptDisable 0x14

static EVT_WDF_INTERRUPT_ISR EvtIsr;
static EVT_WDF_INTERRUPT_DPC EvtDpc;

_Use_decl_annotations_
static BOOLEAN EvtIsr(WDFINTERRUPT Interrupt, ULONG MessageID) {
    UNREFERENCED_PARAMETER(MessageID);
    PDEVICE_CONTEXT dc = DeviceContextGet(WdfInterruptGetDevice(Interrupt));

    /* If the controller isn't initialised yet (race between PnP and
     * a stray interrupt), don't claim the IRQ. */
    if (!dc->HcInitialized || dc->MmioBase == NULL) return FALSE;

    ULONG istat = READ_REGISTER_ULONG((PULONG)((PUCHAR)dc->MmioBase + REG_HcInterruptStatus));
    /* Spurious / unplugged: 0xFFFFFFFF means the device is gone. */
    if (istat == 0 || istat == 0xFFFFFFFF) return FALSE;

    ULONG ie = READ_REGISTER_ULONG((PULONG)((PUCHAR)dc->MmioBase + REG_HcInterruptEnable));
    if ((istat & ie & ~OHCI_INT_MIE) == 0) return FALSE;

    /* Mask future interrupts until the DPC drains. */
    WRITE_REGISTER_ULONG((PULONG)((PUCHAR)dc->MmioBase + REG_HcInterruptDisable),
                         OHCI_INT_MIE);

    WdfInterruptQueueDpcForIsr(Interrupt);
    return TRUE;
}

_Use_decl_annotations_
static VOID EvtDpc(WDFINTERRUPT Interrupt, WDFOBJECT AssociatedObject) {
    UNREFERENCED_PARAMETER(AssociatedObject);
    PDEVICE_CONTEXT dc = DeviceContextGet(WdfInterruptGetDevice(Interrupt));

    /* Snapshot HcInterruptStatus to see what fired. */
    ULONG istat = READ_REGISTER_ULONG(
                      (PULONG)((PUCHAR)dc->MmioBase + REG_HcInterruptStatus));

    if (istat & OHCI_INT_WDH) {
        /* ohci_drain_done processes retired TDs; WDH is W1C inside the core.
         * CoreLock serialises against the per-EP queue submit path which
         * mutates hc->in_flight and the bounce-pool bitmap from PASSIVE/
         * DISPATCH context. */
        WdfSpinLockAcquire(dc->CoreLock);
        ohci_drain_done(&dc->Hc);
        WdfSpinLockRelease(dc->CoreLock);
    }

    if ((istat & OHCI_INT_RHSC) && dc->RootHub) {
        LOG("RHSC fired — notifying UCX root hub");
        OhciPci_NotifyPortChanged(dc);
        /* W1C the RHSC bit so it does not re-fire. */
        WRITE_REGISTER_ULONG(
            (PULONG)((PUCHAR)dc->MmioBase + REG_HcInterruptStatus),
            OHCI_INT_RHSC);
    }

    /* Re-enable the master interrupt. */
    if (dc->MmioBase) {
        WRITE_REGISTER_ULONG((PULONG)((PUCHAR)dc->MmioBase + REG_HcInterruptEnable),
                             OHCI_INT_MIE);
    }
}

NTSTATUS OhciPci_CreateInterrupt(PDEVICE_CONTEXT dc) {
    WDF_INTERRUPT_CONFIG cfg;
    WDF_INTERRUPT_CONFIG_INIT(&cfg, EvtIsr, EvtDpc);
    cfg.ShareVector = WdfTrue;  /* OHCI may share with other USB controllers */
    NTSTATUS status = WdfInterruptCreate(dc->Device, &cfg,
                                          WDF_NO_OBJECT_ATTRIBUTES, &dc->Interrupt);
    LOG("WdfInterruptCreate -> 0x%08X", status);
    return status;
}
