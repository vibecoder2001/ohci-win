/* Kernel-mode ohci_mmio_ops adapter. Backs read32/write32/barrier with
 * READ_REGISTER_ULONG / WRITE_REGISTER_ULONG / KeMemoryBarrier. The
 * `context` pointer in ohci_mmio_ops is a PDEVICE_CONTEXT. */
#include <ntddk.h>
#include "device_context.h"
#include "ohci_mmio.h"

static uint32_t mmio_read32(void *ctx, uint32_t reg_offset) {
    PDEVICE_CONTEXT dc = (PDEVICE_CONTEXT)ctx;
    return READ_REGISTER_ULONG((PULONG)((PUCHAR)dc->MmioBase + reg_offset));
}

static void mmio_write32(void *ctx, uint32_t reg_offset, uint32_t value) {
    PDEVICE_CONTEXT dc = (PDEVICE_CONTEXT)ctx;
    WRITE_REGISTER_ULONG((PULONG)((PUCHAR)dc->MmioBase + reg_offset), value);
}

static void mmio_barrier(void *ctx) {
    UNREFERENCED_PARAMETER(ctx);
    KeMemoryBarrier();
}

void OhciPci_InitMmioOps(PDEVICE_CONTEXT dc) {
    dc->MmioOps.context = dc;
    dc->MmioOps.read32  = mmio_read32;
    dc->MmioOps.write32 = mmio_write32;
    dc->MmioOps.barrier = mmio_barrier;
}
