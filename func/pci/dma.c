/* Kernel-mode ohci_dma_region setup. Allocates a single contiguous
 * DMA-coherent buffer via WDFCOMMONBUFFER and wraps it for the core lib's
 * bump allocator. OHCI is a 32-bit DMA device; WDF's default DMA mask
 * is 32-bit so no extra DMA-config plumbing is required. */
#include <ntddk.h>
#include <wdf.h>
#include "device_context.h"

/* Sized for ohci_hc_init's overhead (HCCA + 4 ED pools + TD pool + 63-ED
 * interrupt skeleton ≈ 6 KB) PLUS the 256 KB bounce buffer pool (Plan 5
 * Task 1). 512 KB gives generous headroom. */
#define OHCIPCI_DMA_BUFFER_SIZE  (512 * 1024)

NTSTATUS OhciPci_AllocateDma(PDEVICE_CONTEXT dc) {
    NTSTATUS status;

    WDF_DMA_ENABLER_CONFIG dmaCfg;
    WDF_DMA_ENABLER_CONFIG_INIT(&dmaCfg, WdfDmaProfileScatterGather, OHCIPCI_DMA_BUFFER_SIZE);
    /* OHCI accepts only 32-bit physical addresses; the default profile
     * already constrains to 32-bit, but be explicit for documentation. */
    status = WdfDmaEnablerCreate(dc->Device, &dmaCfg,
                                 WDF_NO_OBJECT_ATTRIBUTES, &dc->DmaEnabler);
    if (!NT_SUCCESS(status)) return status;

    status = WdfCommonBufferCreate(dc->DmaEnabler, OHCIPCI_DMA_BUFFER_SIZE,
                                   WDF_NO_OBJECT_ATTRIBUTES, &dc->DmaBuffer);
    if (!NT_SUCCESS(status)) return status;

    PVOID virt = WdfCommonBufferGetAlignedVirtualAddress(dc->DmaBuffer);
    PHYSICAL_ADDRESS phys = WdfCommonBufferGetAlignedLogicalAddress(dc->DmaBuffer);

    /* OHCI uses 32-bit phys; LowPart is the address. */
    ohci_dma_init(&dc->DmaRegion, virt, phys.LowPart, OHCIPCI_DMA_BUFFER_SIZE);
    return STATUS_SUCCESS;
}
