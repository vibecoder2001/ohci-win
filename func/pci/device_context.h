#ifndef OHCIPCI_DEVICE_CONTEXT_H
#define OHCIPCI_DEVICE_CONTEXT_H

#include <ntddk.h>
#include <wdf.h>
#include "ohci_mmio.h"
#include "ohci_dma.h"
#include "ohci_hc.h"

/* Per-device state for one OhciPci instance. WDF gives us a typed pointer
 * to this struct via DeviceContextGet(device). */
typedef struct _DEVICE_CONTEXT {
    WDFDEVICE                Device;

    /* Mapped MMIO region (BAR0). Tasks 4+ populate. */
    PVOID                    MmioBase;
    SIZE_T                   MmioLength;

    /* Translated interrupt resource (Tasks 4 + 6). */
    ULONG                    InterruptVector;
    KIRQL                    InterruptIrql;
    KINTERRUPT_MODE          InterruptMode;
    WDFINTERRUPT             Interrupt;

    /* DMA enabler + common buffer (Task 3). */
    WDFDMAENABLER            DmaEnabler;
    WDFCOMMONBUFFER          DmaBuffer;

    /* Wired into core lib (Tasks 2/3/5). */
    struct ohci_mmio_ops     MmioOps;
    struct ohci_dma_region   DmaRegion;
    struct ohci_hc           Hc;

    BOOLEAN                  HcInitialized;  /* Set after ohci_hc_init success. */
} DEVICE_CONTEXT, *PDEVICE_CONTEXT;

WDF_DECLARE_CONTEXT_TYPE_WITH_NAME(DEVICE_CONTEXT, DeviceContextGet)

/* Defined in mmio.c — installs kernel read32/write32/barrier into dc->MmioOps. */
void OhciPci_InitMmioOps(PDEVICE_CONTEXT dc);

#endif /* OHCIPCI_DEVICE_CONTEXT_H */
