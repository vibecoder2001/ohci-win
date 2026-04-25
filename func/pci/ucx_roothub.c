/*++

Module Name:

    ucx_roothub.c

Abstract:

    UCX 1.6 root hub skeleton for OhciPci.

    Implements OhciPci_RootHubCreate, which registers stub callbacks with
    UCX and creates the UCXROOTHUB object. Tasks 3 + 4 replace the stubs
    with real OHCI port-management logic.

Environment:

    Kernel mode only.

--*/

/*
 * UCX 1.6 Root Hub API — findings from reading
 *   D:\Windows Kits\10\Include\10.0.26100.0\km\ucx\1.6\ucxroothub.h
 *
 * Struct:
 *   UCX_ROOTHUB_CONFIG (confirmed, plan name was correct)
 *   Fields relevant to us:
 *     PFN_UCX_ROOTHUB_CONTROL_URB    EvtRootHubClearHubFeature
 *     PFN_UCX_ROOTHUB_CONTROL_URB    EvtRootHubClearPortFeature
 *     PFN_UCX_ROOTHUB_CONTROL_URB    EvtRootHubGetHubStatus
 *     PFN_UCX_ROOTHUB_CONTROL_URB    EvtRootHubGetPortStatus
 *     PFN_UCX_ROOTHUB_CONTROL_URB    EvtRootHubSetHubFeature
 *     PFN_UCX_ROOTHUB_CONTROL_URB    EvtRootHubSetPortFeature
 *     PFN_UCX_ROOTHUB_CONTROL_URB    EvtRootHubGetPortErrorCount
 *     PFN_UCX_ROOTHUB_CONTROL_URB    EvtRootHubControlUrb  (OR the above 7)
 *     PFN_UCX_ROOTHUB_INTERRUPT_TX   EvtRootHubInterruptTx
 *     PFN_UCX_ROOTHUB_GET_INFO       EvtRootHubGetInfo
 *     PFN_UCX_ROOTHUB_GET_20PORT_INFO EvtRootHubGet20PortInfo
 *     PFN_UCX_ROOTHUB_GET_30PORT_INFO EvtRootHubGet30PortInfo  (REQUIRED even for OHCI)
 *
 * Init macros (two variants):
 *   UCX_ROOTHUB_CONFIG_INIT(Config, ClearHubFeat, ClearPortFeat,
 *       GetHubStatus, GetPortStatus, SetHubFeat, SetPortFeat,
 *       GetPortErrCnt, InterruptTx, GetInfo, Get20PortInfo, Get30PortInfo)
 *     — sets the 7 individual URB handlers (EvtRootHubControlUrb = NULL)
 *
 *   UCX_ROOTHUB_CONFIG_INIT_WITH_CONTROL_URB_HANDLER(Config,
 *       ControlUrb, InterruptTx, GetInfo, Get20PortInfo, Get30PortInfo)
 *     — sets one ControlUrb dispatcher; individual handlers = NULL
 *
 * We use UCX_ROOTHUB_CONFIG_INIT_WITH_CONTROL_URB_HANDLER for the skeleton;
 * Tasks 3/4 can promote to individual callbacks if needed.
 *
 * Callback signatures (DEVIATION from plan skeleton guesses):
 *   Plan guessed callbacks take (UCXROOTHUB, PROOTHUB_INFO / struct pointer).
 *   ACTUAL: ALL callbacks take (UCXROOTHUB UcxRootHub, WDFREQUEST Request).
 *   ROOTHUB_INFO, ROOTHUB_20PORTS_INFO, ROOTHUB_30PORTS_INFO are the data
 *   structs retrieved from the WDF request memory by the real implementations.
 *
 * UcxRootHubCreate signature (confirmed, matches plan):
 *   UcxRootHubCreate(Controller, Config, Attributes, RootHub*)
 *
 * USB3 port info callback: REQUIRED (marked __notnull in annotations).
 *   OHCI is USB 2.0 only; we stub it with STATUS_NOT_IMPLEMENTED but must
 *   provide a non-NULL pointer.
 */

#include <ntddk.h>
#include <wdf.h>
#include <UcxClass.h>
#include "device_context.h"

#define LOG(fmt, ...) DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL, \
                                  "OhciPci: " fmt "\n", ##__VA_ARGS__)

/* --------------------------------------------------------------------------
 * Stub callbacks — Tasks 3 + 4 replace with real implementations.
 *
 * DEVIATION from plan skeleton: plan guessed a separate typedef per callback
 * with struct-pointer second parameters. Actual WDK 10.0.26100.0 defines one
 * typedef for all URB-style callbacks (EVT_UCX_ROOTHUB_CONTROL_URB) and all
 * non-URB callbacks take (UCXROOTHUB, WDFREQUEST) — real data is fetched from
 * the request's memory by the callee.
 * -------------------------------------------------------------------------- */

/*
 * StubControlUrb — EVT_UCX_ROOTHUB_CONTROL_URB
 *
 * Dispatched for all standard root hub control requests (GetHubStatus,
 * GetPortStatus, SetPortFeature, etc.) when using the combined handler path.
 * Plan skeleton had separate callbacks for each; we use the combined variant
 * at skeleton stage — Tasks 3/4 will decompose as needed.
 */
static EVT_UCX_ROOTHUB_CONTROL_URB StubControlUrb;

_Use_decl_annotations_
static VOID StubControlUrb(UCXROOTHUB UcxRootHub, WDFREQUEST Request)
{
    UNREFERENCED_PARAMETER(UcxRootHub);
    LOG("RootHub StubControlUrb (Tasks 3+4 will implement)");
    WdfRequestComplete(Request, STATUS_NOT_IMPLEMENTED);
}

/*
 * StubInterruptTx — EVT_UCX_ROOTHUB_INTERRUPT_TX
 *
 * Plan skeleton name matches exactly. Signature confirmed: (UCXROOTHUB, WDFREQUEST).
 */
static EVT_UCX_ROOTHUB_INTERRUPT_TX StubInterruptTx;

_Use_decl_annotations_
static VOID StubInterruptTx(UCXROOTHUB UcxRootHub, WDFREQUEST Request)
{
    UNREFERENCED_PARAMETER(UcxRootHub);
    LOG("RootHub StubInterruptTx — completing as STATUS_NOT_IMPLEMENTED");
    WdfRequestComplete(Request, STATUS_NOT_IMPLEMENTED);
}

/*
 * StubGetInfo — EVT_UCX_ROOTHUB_GET_INFO
 *
 * Plan skeleton name matches. Signature: (UCXROOTHUB, WDFREQUEST) — not the
 * guessed (UCXROOTHUB, PROOTHUB_INFO). Real impl retrieves ROOTHUB_INFO from
 * WdfRequestRetrieveOutputMemory; stub just completes the request.
 */
static EVT_UCX_ROOTHUB_GET_INFO StubGetInfo;

_Use_decl_annotations_
static VOID StubGetInfo(UCXROOTHUB UcxRootHub, WDFREQUEST Request)
{
    UNREFERENCED_PARAMETER(UcxRootHub);
    LOG("RootHub StubGetInfo (Task 3 will implement)");
    WdfRequestComplete(Request, STATUS_NOT_IMPLEMENTED);
}

/*
 * StubGet20PortInfo — EVT_UCX_ROOTHUB_GET_20PORT_INFO
 *
 * Plan skeleton name matches. Signature: (UCXROOTHUB, WDFREQUEST) — not the
 * guessed (UCXROOTHUB, ULONG count, PROOTHUB_20PORT_INFO). Real impl retrieves
 * ROOTHUB_20PORTS_INFO from request memory.
 */
static EVT_UCX_ROOTHUB_GET_20PORT_INFO StubGet20PortInfo;

_Use_decl_annotations_
static VOID StubGet20PortInfo(UCXROOTHUB UcxRootHub, WDFREQUEST Request)
{
    UNREFERENCED_PARAMETER(UcxRootHub);
    LOG("RootHub StubGet20PortInfo (Task 3 will implement)");
    WdfRequestComplete(Request, STATUS_NOT_IMPLEMENTED);
}

/*
 * StubGet30PortInfo — EVT_UCX_ROOTHUB_GET_30PORT_INFO
 *
 * DEVIATION from plan: plan's skeleton guessed USB3 could be passed NULL.
 * Actual header marks EvtRootHubGet30PortInfo as __notnull — a non-NULL
 * pointer is always required. OHCI is USB 2.0 only, so we provide a stub
 * that immediately completes with STATUS_NOT_IMPLEMENTED.
 */
static EVT_UCX_ROOTHUB_GET_30PORT_INFO StubGet30PortInfo;

_Use_decl_annotations_
static VOID StubGet30PortInfo(UCXROOTHUB UcxRootHub, WDFREQUEST Request)
{
    UNREFERENCED_PARAMETER(UcxRootHub);
    LOG("RootHub StubGet30PortInfo — OHCI is USB 2.0 only; not implemented");
    WdfRequestComplete(Request, STATUS_NOT_IMPLEMENTED);
}

/* --------------------------------------------------------------------------
 * OhciPci_RootHubCreate — declared in device_context.h
 * -------------------------------------------------------------------------- */

NTSTATUS
OhciPci_RootHubCreate(
    _In_ PDEVICE_CONTEXT dc,
    _In_ UCXCONTROLLER   controller
    )
{
    UCX_ROOTHUB_CONFIG cfg;

    /*
     * Use UCX_ROOTHUB_CONFIG_INIT_WITH_CONTROL_URB_HANDLER rather than
     * UCX_ROOTHUB_CONFIG_INIT: the former takes one ControlUrb dispatcher
     * instead of the 7 individual request handlers, which keeps the skeleton
     * compact. Tasks 3+4 can switch to UCX_ROOTHUB_CONFIG_INIT with separate
     * handlers once real port logic is wired.
     *
     * Args: Config, ControlUrb, InterruptTx, GetInfo, Get20PortInfo, Get30PortInfo
     * DEVIATION from plan: plan called UCX_ROOTHUB_CONFIG_INIT with 4 args
     * including a NULL USB3 arg; actual macro takes 5 non-config args and all
     * must be non-NULL.
     */
    UCX_ROOTHUB_CONFIG_INIT_WITH_CONTROL_URB_HANDLER(
        &cfg,
        StubControlUrb,
        StubInterruptTx,
        StubGetInfo,
        StubGet20PortInfo,
        StubGet30PortInfo
    );

    NTSTATUS status = UcxRootHubCreate(controller, &cfg,
                                       WDF_NO_OBJECT_ATTRIBUTES, &dc->RootHub);
    LOG("UcxRootHubCreate -> 0x%08X", status);
    return status;
}
