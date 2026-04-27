#include <ntddk.h>
#include <wdf.h>
#include "device_context.h"
#include "ohci_drain.h"
#include "ohci_regs.h"
#include "ohci_log.h"

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
         * DISPATCH context. OhciPci_UrbComplete (called from inside the
         * drain) only stages completions onto dc->DeferredCompletions —
         * we drain the list AFTER releasing CoreLock so WdfRequestComplete
         * can re-enter our submit path without deadlocking. */
        LIST_ENTRY local;
        InitializeListHead(&local);

        WdfSpinLockAcquire(dc->CoreLock);
        ohci_drain_done(&dc->Hc);
        /* Plan 8 Task 7 — refill isoch ED chains immediately after the
         * drain so any URB whose tail just retired has a fresh window
         * queued before the HC walks past. Already under CoreLock; the
         * refill walker requires the caller to hold it. */
        OhciPci_IsocRefillAll_Locked(dc);
        while (!IsListEmpty(&dc->DeferredCompletions)) {
            PLIST_ENTRY le = RemoveHeadList(&dc->DeferredCompletions);
            InsertTailList(&local, le);
        }
        WdfSpinLockRelease(dc->CoreLock);

        while (!IsListEmpty(&local)) {
            PLIST_ENTRY le = RemoveHeadList(&local);
            POHCIPCI_URB_CTX uc =
                CONTAINING_RECORD(le, OHCIPCI_URB_CTX, DeferredEntry);
            WdfRequestCompleteWithInformation(uc->Request,
                                              uc->DeferredStatus,
                                              uc->DeferredInfo);
        }
    }

    if ((istat & OHCI_INT_RHSC) && dc->RootHub) {
        LOG("RHSC fired — notifying UCX root hub");
        OhciPci_NotifyPortChanged(dc);
        /* W1C the RHSC bit so it does not re-fire. */
        WRITE_REGISTER_ULONG(
            (PULONG)((PUCHAR)dc->MmioBase + REG_HcInterruptStatus),
            OHCI_INT_RHSC);
    }

    /* SO (Scheduling Overrun) and UE (Unrecoverable Error) are W1C
     * status bits; if we leave them set, the ISR keeps re-claiming the
     * IRQ (istat & ie & ~MIE stays non-zero) and we loop. Log+ack so a
     * single occurrence is visible without livelocking the CPU. */
    if (istat & OHCI_INT_SO) {
        LOG("SO (Scheduling Overrun) — periodic schedule didn't fit in frame");
        WRITE_REGISTER_ULONG(
            (PULONG)((PUCHAR)dc->MmioBase + REG_HcInterruptStatus),
            OHCI_INT_SO);
    }
    if (istat & OHCI_INT_UE) {
        LOG("UE (Unrecoverable Error) — HCR + re-init; failing in-flight URBs");
        /* W1C the UE bit first so the IRQ doesn't keep re-firing while
         * we sit in the recovery path. */
        WRITE_REGISTER_ULONG(
            (PULONG)((PUCHAR)dc->MmioBase + REG_HcInterruptStatus),
            OHCI_INT_UE);

        /* Reinit. Splices URBs out of in_flight and calls their
         * complete callbacks, which (via OhciPci_UrbComplete) stage
         * WDFREQUEST completions on dc->DeferredCompletions while we
         * still hold CoreLock. After HCR + register reprogramming, we
         * still need to W1S the driver-glue interrupt bits (RHSC|UE|SO)
         * that aren't part of ohci_hc_reinit_after_ue's WDH|MIE set. */
        LIST_ENTRY ueLocal;
        InitializeListHead(&ueLocal);

        WdfSpinLockAcquire(dc->CoreLock);
        ohci_hc_reinit_after_ue(&dc->Hc);
        while (!IsListEmpty(&dc->DeferredCompletions)) {
            PLIST_ENTRY le = RemoveHeadList(&dc->DeferredCompletions);
            InsertTailList(&ueLocal, le);
        }
        WdfSpinLockRelease(dc->CoreLock);

        /* Re-enable the extended interrupt set the driver glue wired up
         * after ohci_hc_init. HcInterruptEnable is W1S so this OR's the
         * bits on top of the WDH|MIE that reinit just programmed. */
        WRITE_REGISTER_ULONG(
            (PULONG)((PUCHAR)dc->MmioBase + REG_HcInterruptEnable),
            OHCI_INT_RHSC | OHCI_INT_UE | OHCI_INT_SO);

        while (!IsListEmpty(&ueLocal)) {
            PLIST_ENTRY le = RemoveHeadList(&ueLocal);
            POHCIPCI_URB_CTX uc =
                CONTAINING_RECORD(le, OHCIPCI_URB_CTX, DeferredEntry);
            WdfRequestCompleteWithInformation(uc->Request,
                                              uc->DeferredStatus,
                                              uc->DeferredInfo);
        }
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
