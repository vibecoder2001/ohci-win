/* Kernel-mode ohci_dma_region setup. Allocates a single contiguous
 * DMA-coherent buffer via WDFCOMMONBUFFER and wraps it for the core lib's
 * bump allocator. OHCI is a 32-bit DMA device; WDF's default DMA mask
 * is 32-bit so no extra DMA-config plumbing is required. */
#include <ntddk.h>
#include <wdf.h>
#include "device_context.h"

#define OHCIPCI_DMA_BUFFER_SIZE  (256 * 1024)  /* HCCA + descriptors + small URB buffers */

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
