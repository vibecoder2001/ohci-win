#ifndef OHCIPCI_DEVICE_CONTEXT_H
#define OHCIPCI_DEVICE_CONTEXT_H

#include <ntddk.h>
#include <wdf.h>
#include <UcxClass.h>
#include "ohci_mmio.h"
#include "ohci_dma.h"
#include "ohci_hc.h"
#include "ohci_control.h"
#include "ohci_urb.h"

/* Bounce buffer pool sized for Plan 5 enumeration workload:
 * 64 slabs × 4 KB = 256 KB of the DMA region reserved. */
#define OHCIPCI_BOUNCE_SLAB_COUNT  64
#define OHCIPCI_BOUNCE_SLAB_BYTES  4096

struct ohcipci_bounce_pool {
    uint8_t *base;       /* virtual base of the pool's chunk in DMA region */
    uint32_t base_phys;  /* matching physical address */
    uint32_t free_bitmap[(OHCIPCI_BOUNCE_SLAB_COUNT + 31) / 32];
};

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

    /* UCX controller and root hub handles (Plan 5 Tasks 1-4). */
    UCXCONTROLLER            Controller;    /* Saved from UcxControllerCreate. */
    UCXROOTHUB               RootHub;      /* From UcxRootHubCreate. */

    BOOLEAN                  HcInitialized;  /* Set after ohci_hc_init success. */

    struct ohcipci_bounce_pool BouncePool;
} DEVICE_CONTEXT, *PDEVICE_CONTEXT;

WDF_DECLARE_CONTEXT_TYPE_WITH_NAME(DEVICE_CONTEXT, DeviceContextGet)

/* --------------------------------------------------------------------------
 * Per-endpoint context (Task 6: ucx_endpoint.c).
 *
 * Attached to each UCXENDPOINT WDF object. WDF_DECLARE_CONTEXT_TYPE_WITH_NAME
 * requires a single-token type so we use a typedef'd name.
 * For the default (EP0) endpoint, Core holds the OHCI control endpoint state.
 * -------------------------------------------------------------------------- */
typedef struct _OHCIPCI_EP_CONTEXT {
    PDEVICE_CONTEXT              Dc;        /* back-pointer to device context  */
    UCXENDPOINT                  UcxEp;    /* the endpoint handle itself       */
    WDFQUEUE                     UrbQueue;  /* queue UCX delivers URBs to      */
    struct ohci_control_endpoint Core;      /* OHCI core EP state              */
} OHCIPCI_EP_CONTEXT, *POHCIPCI_EP_CONTEXT;

WDF_DECLARE_CONTEXT_TYPE_WITH_NAME(OHCIPCI_EP_CONTEXT, OhciPci_EpContextGet)

/* --------------------------------------------------------------------------
 * Per-URB context (Task 6: ucx_endpoint.c).
 *
 * Allocated as object context on each WDFREQUEST that wraps a Control URB.
 * CoreUrb must be the first field so CONTAINING_RECORD works from the
 * completion callback.
 * -------------------------------------------------------------------------- */
typedef struct _OHCIPCI_URB_CTX {
    struct ohci_urb              CoreUrb;        /* must be first (CONTAINING_RECORD) */
    void                        *SetupBounce;    /* 8-byte SETUP packet bounce buffer */
    uint32_t                     SetupBouncePhys;
    void                        *DataBounce;     /* Data-stage bounce buffer (or NULL) */
    uint32_t                     DataBouncePhys;
    uint32_t                     DataLength;     /* 0 if no data stage */
    uint8_t                      DataDirection;  /* OHCI_URB_DIR_IN / _OUT */
    PMDL                         UserMdl;        /* caller's MDL (may be NULL) */
    PVOID                        UserVa;         /* caller's KVA if no MDL    */
    WDFREQUEST                   Request;        /* the owning WDFREQUEST      */
    POHCIPCI_EP_CONTEXT          EpCtx;         /* owning endpoint context    */
} OHCIPCI_URB_CTX, *POHCIPCI_URB_CTX;

WDF_DECLARE_CONTEXT_TYPE_WITH_NAME(OHCIPCI_URB_CTX, OhciPci_UrbCtxGet)

/* --------------------------------------------------------------------------
 * Per-queue context (Task 6: ucx_endpoint.c).
 *
 * Attached to the WDFQUEUE created for each endpoint. Holds a back-pointer
 * to the owning EP context, because WDF provides no WdfIoQueueGetParentObject
 * API (only DPC/timer/workitem have typed GetParentObject helpers).
 * -------------------------------------------------------------------------- */
typedef struct _OHCIPCI_QUEUE_CTX {
    POHCIPCI_EP_CONTEXT          EpCtx;
} OHCIPCI_QUEUE_CTX, *POHCIPCI_QUEUE_CTX;

WDF_DECLARE_CONTEXT_TYPE_WITH_NAME(OHCIPCI_QUEUE_CTX, OhciPci_QueueCtxGet)

/* g_DeviceContext — single-instance shortcut. Defined in ucx_roothub.c;
 * set once in OhciPci_RootHubCreate before any UCX callbacks fire. */
extern PDEVICE_CONTEXT g_DeviceContext;

/* Defined in mmio.c — installs kernel read32/write32/barrier into dc->MmioOps. */
void OhciPci_InitMmioOps(PDEVICE_CONTEXT dc);

/* Defined in dma.c — allocates WDFCOMMONBUFFER and wraps it in dc->DmaRegion. */
NTSTATUS OhciPci_AllocateDma(PDEVICE_CONTEXT dc);

/* Defined in interrupt.c — creates WDFINTERRUPT with ISR + DPC. */
NTSTATUS OhciPci_CreateInterrupt(PDEVICE_CONTEXT dc);

/* Defined in ucx_glue.c — UCX 1.6 controller registration helpers. */
NTSTATUS OhciPci_UcxInitDeviceInit(PWDFDEVICE_INIT DeviceInit);
NTSTATUS OhciPci_UcxControllerCreate(PDEVICE_CONTEXT dc);
NTSTATUS OhciPci_CreateDefaultQueue(PDEVICE_CONTEXT dc);

/* Defined in ucx_roothub.c — UCX 1.6 root hub registration. */
NTSTATUS OhciPci_RootHubCreate(PDEVICE_CONTEXT dc, UCXCONTROLLER controller);

/* Thin wrapper so interrupt.c can trigger UCX port-change without a direct
 * UcxClass.h dependency in that translation unit. */
void OhciPci_NotifyPortChanged(PDEVICE_CONTEXT dc);

/* Defined in ucx_usbdevice.c — EvtControllerUsbDeviceAdd.
 * Note: info struct is UCXUSBDEVICE_INFO (confirmed from ucxusbdevice.h). */
NTSTATUS OhciPci_UsbDeviceAdd(UCXCONTROLLER      Controller,
                               PUCXUSBDEVICE_INFO UsbDeviceInfo,
                               PUCXUSBDEVICE_INIT UsbDeviceInit);

NTSTATUS OhciPci_DefaultEndpointAdd(UCXCONTROLLER     UcxController,
                                     UCXUSBDEVICE      UcxUsbDevice,
                                     ULONG             MaxPacketSize,
                                     PUCXENDPOINT_INIT UcxEndpointInit);

NTSTATUS OhciPci_EndpointAdd(UCXCONTROLLER                              UcxController,
                              UCXUSBDEVICE                               UcxUsbDevice,
                              PUSB_ENDPOINT_DESCRIPTOR                   UsbEndpointDescriptor,
                              ULONG                                      UsbEndpointDescriptorBufferLength,
                              PUSB_SUPERSPEED_ENDPOINT_COMPANION_DESCRIPTOR SuperSpeedEndpointCompanionDescriptor,
                              PUCXENDPOINT_INIT                          UcxEndpointInit);

/* Defined in bounce.c — per-URB bounce buffer slab pool. */
NTSTATUS OhciPci_BounceInit(PDEVICE_CONTEXT dc);

/* Allocate one slab. Returns NULL on exhaustion. *phys_out gets the
 * physical address. Buffer size is OHCIPCI_BOUNCE_SLAB_BYTES. */
void *OhciPci_BounceAlloc(PDEVICE_CONTEXT dc, uint32_t *phys_out);

void  OhciPci_BounceFree(PDEVICE_CONTEXT dc, void *ptr);

#endif /* OHCIPCI_DEVICE_CONTEXT_H */
