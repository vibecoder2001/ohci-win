#ifndef OHCIPCI_DEVICE_CONTEXT_H
#define OHCIPCI_DEVICE_CONTEXT_H

#include <ntddk.h>
#include <wdf.h>
#include <UcxClass.h>
#include "ohci_mmio.h"
#include "ohci_dma.h"
#include "ohci_hc.h"
#include "ohci_control.h"
#include "ohci_bulk.h"
#include "ohci_interrupt.h"
#include "ohci_isoc.h"
#include "ohci_urb.h"

/* TRANSFER_URB layout, copied from the dwusb reference (Driver.h). The struct
 * is a UCX-private contract not exported in any public WDK header; UCX puts a
 * pointer to it in Parameters.Others.Arg1 of every request enqueued to a
 * per-EP WDFQUEUE. SetupPacket[8] lives at u.SetupPacket for control transfers.
 * Lives here so isoc_mdl.c and ucx_endpoint.c share one definition. */
typedef struct _OHCIPCI_UCX_URB_DATA {
    PVOID Reserved[8];
} OHCIPCI_UCX_URB_DATA;

typedef struct _OHCIPCI_TRANSFER_URB {
    struct _URB_HEADER Hdr;
    PVOID UsbdPipeHandle;
    ULONG TransferFlags;
    ULONG TransferBufferLength;
    PVOID TransferBuffer;
    PMDL  TransferBufferMDL;
    union {
        ULONG Timeout;
        PVOID ReservedMBNull;
    };
    OHCIPCI_UCX_URB_DATA UrbData;
    union {
        struct {
            ULONG StartFrame;
            ULONG NumberOfPackets;
            ULONG ErrorCount;
            USBD_ISO_PACKET_DESCRIPTOR IsoPacket[1];
        } Isoch;
        UCHAR SetupPacket[8];
    } u;
} OHCIPCI_TRANSFER_URB, *POHCIPCI_TRANSFER_URB;

/* Bounce buffer pool sized for Plan 5 enumeration workload:
 * 64 slabs × 4 KB = 256 KB of the DMA region reserved. */
#define OHCIPCI_BOUNCE_SLAB_COUNT  64
#define OHCIPCI_BOUNCE_SLAB_BYTES  4096

/* OHCI 1.0a §5.4: HCD reserves <=90% of the 1500-byte FS frame for
 * the periodic schedule. */
#define OHCIPCI_PERIODIC_BUDGET_BYTES 1350

/* Refill watermarks (frames). Keep >= OHCIPCI_ISOC_PRIME_LOOKAHEAD ahead
 * of HcFmNumber so the HC always has dispatchable ITDs. */
#define OHCIPCI_ISOC_REFILL_HIGH        16   /* refill up to this lead */
#define OHCIPCI_ISOC_PRIME_LOOKAHEAD     4   /* first-prime sf = HcFmNumber + 4 */
#define OHCIPCI_ISOC_BACKSTOP_TIMER_MS   1   /* 1 ms periodic backstop */
#define OHCIPCI_ISOC_SILENCE_BURST_MAX   4   /* silence ITDs (32 frames) per refill call */

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

    /* DMA enabler + common buffer (Task 3). Isoch no longer uses a
     * separate WDFDMAENABLER — the MDL-walk path in func/pci/isoc_mdl.c
     * walks PFNs directly and bounces only when they're non-contiguous. */
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

    /* Coarse spinlock serialising every call into the OHCI core that
     * touches hc->in_flight or the bounce-pool bitmap. Acquired by:
     *   - EvtUrbDefault before ohci_*_submit / OhciPci_BounceAlloc
     *   - The interrupt DPC before ohci_drain_done
     *   - Endpoint create paths before splicing onto the HC list
     * Created in EvtDriverDeviceAdd; tied to WDFDEVICE lifetime. */
    WDFSPINLOCK              CoreLock;

    /* Deferred-completion list. WdfRequestComplete() must NOT be called
     * with CoreLock held: completion can synchronously re-enter our queue
     * EvtIoDefault, which then tries to acquire CoreLock again — and
     * WDFSPINLOCK is not re-entrant. So OhciPci_UrbComplete (running
     * under CoreLock from inside the WDH DPC) only stages the result on
     * this list; EvtDpc drains the list and completes each request
     * AFTER releasing CoreLock. Protected by CoreLock. */
    LIST_ENTRY               DeferredCompletions;

    /* USB address allocator. UCX expects the driver to assign each new
     * device an address (USBDEVICE_ADDRESS.Address is OUT, not IN — see
     * dwusb UsbDevice_UcxEvtAddress). Range is 1..127 per USB spec.
     * Plan 7 doesn't free addresses on disconnect so this just monotonically
     * increments; a future plan can do real allocation. */
    volatile LONG            NextUsbAddress;

    /* Periodic budget tracker (Plan 8). Sum of MaxPacketSize across all
     * Isoc + Interrupt EPs at period 1; rejected new EPs that would push
     * past 90% of the FS frame budget. */
    ULONG                    PeriodicBytesPerFrame;

    /* Plan 8 Task 7 — isoch refill state.
     * IsocEps is a list of OHCIPCI_EP_CONTEXT chained on IsocEpEntry,
     * walked by OhciPci_IsocRefillAll_Locked from EvtDpc and from the
     * 1 ms periodic IsocRefillTimer backstop. IsocEpsLock is INDEPENDENT
     * of CoreLock; the refill walker holds CoreLock first, then briefly
     * IsocEpsLock — no inversion since no other path takes both. */
    WDFTIMER                 IsocRefillTimer;
    LIST_ENTRY               IsocEps;
    WDFSPINLOCK              IsocEpsLock;
} DEVICE_CONTEXT, *PDEVICE_CONTEXT;

WDF_DECLARE_CONTEXT_TYPE_WITH_NAME(DEVICE_CONTEXT, DeviceContextGet)

/* --------------------------------------------------------------------------
 * Per-endpoint context (Task 6: ucx_endpoint.c).
 *
 * Attached to each UCXENDPOINT WDF object. WDF_DECLARE_CONTEXT_TYPE_WITH_NAME
 * requires a single-token type so we use a typedef'd name.
 * For the default (EP0) endpoint, Core holds the OHCI control endpoint state.
 * -------------------------------------------------------------------------- */
typedef enum _OHCIPCI_EP_KIND {
    OhciPciEpKindControl   = 0,
    OhciPciEpKindBulk      = 1,
    OhciPciEpKindInterrupt = 2,
    OhciPciEpKindIsoc      = 3,
} OHCIPCI_EP_KIND;

typedef struct _OHCIPCI_EP_CONTEXT {
    PDEVICE_CONTEXT              Dc;        /* back-pointer to device context  */
    UCXENDPOINT                  UcxEp;     /* the endpoint handle itself      */
    WDFQUEUE                     UrbQueue;  /* queue UCX delivers URBs to      */
    OHCIPCI_EP_KIND              Kind;      /* discriminator for Core union    */
    union {
        struct ohci_control_endpoint   Control;
        struct ohci_bulk_endpoint      Bulk;
        struct ohci_interrupt_endpoint Interrupt;
        struct ohci_isoc_endpoint      Isoc;
    } Core;                                  /* OHCI core EP state             */

    /* Isoch refill state (Plan 8 Task 7). Used only when Kind==Isoc; left
     * zero for other kinds. Silence buffer is a zero-filled PAGE_SIZE
     * region from dc->DmaRegion that the refill DPC sources bytes from
     * when no caller URB is queued. IsocEpEntry chains us on dc->IsocEps. */
    PVOID                          IsocSilenceVa;
    uint32_t                       IsocSilencePhys;
    ULONG                          IsocSilenceItdCount;
    LIST_ENTRY                     IsocQueuedUrbs;   /* OHCIPCI_URB_CTX::QueueEntry */
    WDFSPINLOCK                    IsocQueueLock;
    LIST_ENTRY                     IsocEpEntry;     /* on dc->IsocEps */

    /* MDL-walk path — URBs with ITDs linked into the ED but not yet retired.
     * Drained by OhciPci_IsocOnUrbRetire_Locked / EP teardown. */
    LIST_ENTRY                     IsocInFlightUrbs;

    /* KeQueryPerformanceCounter timestamp of the previous isoc[N] retire
     * trace, used to log per-URB cadence delta in µs. Zero on the first
     * retire => suppresses the first delta print. */
    LARGE_INTEGER                  IsocLastTraceQpc;
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
    void                        *DataBounce;     /* Control data-stage bounce (or NULL) */
    uint32_t                     DataBouncePhys;
    uint32_t                     DataLength;     /* 0 if no data stage */
    uint8_t                      DataDirection;  /* OHCI_URB_DIR_IN / _OUT */
    PMDL                         UserMdl;        /* caller's MDL (may be NULL) */
    PVOID                        UserVa;         /* caller's KVA if no MDL    */
    WDFREQUEST                   Request;        /* the owning WDFREQUEST      */
    POHCIPCI_EP_CONTEXT          EpCtx;         /* owning endpoint context    */
    WDFDMATRANSACTION            DmaTransaction; /* Bulk path; NULL for Control */
    PMDL                         OurMdl;         /* MDL we built from a flat KVA URB; free in completion */
    PVOID                        TransferUrb;    /* TRANSFER_URB; UCX reads back
                                                   * TransferBufferLength + Hdr.Status
                                                   * after the request completes. */
    /* Deferred-completion staging. Populated by OhciPci_UrbComplete while
     * CoreLock is held; consumed by EvtDpc after CoreLock release. */
    LIST_ENTRY                   DeferredEntry;
    NTSTATUS                     DeferredStatus;
    ULONG_PTR                    DeferredInfo;

    /* Plan 8 Task 7 — pending-URB queue entry on EP->IsocQueuedUrbs.
     * HandleIsocUrb queues here instead of executing the WdfDmaTransaction
     * synchronously; the refill DPC drains entries when the ED chain has
     * room (lead < OHCIPCI_ISOC_REFILL_HIGH frames ahead of HcFmNumber). */
    LIST_ENTRY                   QueueEntry;

    /* Isoch MDL-walk bookkeeping. */
    LIST_ENTRY  InFlightEntry;   /* on EP->IsocInFlightUrbs while ITDs linked */
    PVOID       MdlSysVa;        /* cached MmGetSystemAddressForMdlSafe result */

    /* Page-straddle bounce. When the URB's MDL spans non-contiguous PFNs,
     * OhciPci_IsocBuildAndSubmit_Locked allocates one OHCIPCI_BOUNCE_SLAB
     * and copies the URB through it (OUT) so per-packet phys addresses
     * come from a single contiguous physical run. Freed on retire and on
     * EP teardown. NULL when the URB's PFNs were already contiguous. */
    PVOID       IsocBounceVa;
    uint32_t    IsocBouncePhys;
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

/* --------------------------------------------------------------------------
 * Per-USB-device context (Plan 7).
 *
 * Attached to each UCXUSBDEVICE WDF object. Replaces the single-instance
 * PendingDeviceSpeed / PendingFuncAddr fields previously on DEVICE_CONTEXT
 * so concurrent enumerations (e.g. a USB hub with two children) don't race.
 *
 * EP0 is also tracked here so EvtUsbDeviceAddress can rewrite the
 * func_addr field on the existing OHCI ED after SET_ADDRESS completes.
 * -------------------------------------------------------------------------- */
typedef struct _OHCIPCI_USBDEV_CTX {
    USB_DEVICE_SPEED              Speed;
    UCHAR                         FuncAddr;     /* 0 until EvtUsbDeviceAddress runs */
    struct _OHCIPCI_EP_CONTEXT   *Ep0;          /* set by OhciPci_DefaultEndpointAdd */
} OHCIPCI_USBDEV_CTX, *POHCIPCI_USBDEV_CTX;

WDF_DECLARE_CONTEXT_TYPE_WITH_NAME(OHCIPCI_USBDEV_CTX, OhciPci_UsbDevContextGet)

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

/* Defined in ucx_endpoint.c (Plan 8 Task 7).
 * Walks dc->IsocEps and refills each isoch ED chain with caller URBs
 * (executed via WdfDmaTransactionExecute) or silence ITDs to maintain
 * OHCIPCI_ISOC_REFILL_HIGH frames of lookahead beyond HcFmNumber.
 * Caller MUST hold dc->CoreLock. */
void OhciPci_IsocRefillAll_Locked(PDEVICE_CONTEXT dc);

/* WDFTIMER callback for the periodic backstop. Defined in ucx_endpoint.c. */
EVT_WDF_TIMER OhciPci_EvtIsocBackstopTimer;

/* Defined in ucx_endpoint.c — completion callback installed on CoreUrb. */
struct ohci_urb;
VOID OhciPci_UrbComplete(struct ohci_urb *u);

/* Defined in bounce.c — per-URB bounce buffer slab pool. */
NTSTATUS OhciPci_BounceInit(PDEVICE_CONTEXT dc);

/* Allocate one slab. Returns NULL on exhaustion. *phys_out gets the
 * physical address. Buffer size is OHCIPCI_BOUNCE_SLAB_BYTES. */
void *OhciPci_BounceAlloc(PDEVICE_CONTEXT dc, uint32_t *phys_out);
void  OhciPci_BounceFree(PDEVICE_CONTEXT dc, void *ptr);

#endif /* OHCIPCI_DEVICE_CONTEXT_H */
