/* Kernel-mode ohci_dma_region setup. Allocates a single contiguous
 * DMA-coherent buffer via WDFCOMMONBUFFER and wraps it for the core lib's
 * bump allocator. OHCI is a 32-bit DMA device; WDF's default DMA mask
 * is 32-bit so no extra DMA-config plumbing is required. */
#include <ntddk.h>
#include <wdf.h>
#include "device_context.h"
#include "ohci_bulk.h"

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

    /* Cap HAL SG fragments at half the OHCI core's per-URB TD limit. Each
     * SG element gets split to ≤ PAGE_SIZE chunks in BulkProgramDma so a
     * worst-case 2-page-contiguous element becomes 2 TDs; the /2 keeps the
     * post-split chunk count within OHCI_BULK_MAX_SG_PAGES even when every
     * HAL element straddles a page boundary. */
    WdfDmaEnablerSetMaximumScatterGatherElements(dc->DmaEnabler,
                                                 OHCI_BULK_MAX_SG_PAGES / 2);

    /* Plan 8: separate DMA enabler for isoch with WdfDmaProfilePacket so
     * HAL bounces the caller's audio buffer into a single physically-
     * contiguous chunk before invoking EvtProgramDma. usbaudio.sys hands
     * us page-fragmented buffers (cross page boundary at offset 1472 of a
     * 1920-byte URB) which the SG profile would surface as 2 disjoint
     * elements — packet 7 then straddles the cut and IsocProgramDma
     * rejects the URB. Packet profile sidesteps this entirely; cap is
     * 4 KB which covers UAC1 audio (10 ms @ 48k stereo 16-bit = 1920 B,
     * 20 ms = 3840 B). */
    WDF_DMA_ENABLER_CONFIG isocCfg;
    WDF_DMA_ENABLER_CONFIG_INIT(&isocCfg, WdfDmaProfilePacket, 4096);
    status = WdfDmaEnablerCreate(dc->Device, &isocCfg,
                                 WDF_NO_OBJECT_ATTRIBUTES, &dc->IsocDmaEnabler);
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
