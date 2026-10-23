#include "S32K144.h"
#include "Can.h"
#include "CanIf_Cbk.h"
/* Debug/DET stubs: replace by a real DET and generated SchM in AUTOSAR. */
volatile uint32_t can_step = 0U;
volatile uint32_t can_mcr = 0U;
volatile uint32_t can_ctrl1 = 0U;
volatile uint32_t can_esr1 = 0U;
volatile uint32_t can_iflag1 = 0U;
volatile uint32_t can_tx_count = 0U;
volatile uint32_t can_rx_count = 0U;
volatile uint32_t can_error_count = 0U;
volatile uint8_t can_last_status = E_NOT_OK;
volatile uint32_t can_det_count = 0U;
volatile uint8 can_det_last_api = 0U;
volatile uint8 can_det_last_error = 0U;
volatile uint32 can_runtime_count = 0U;
volatile uint8 can_runtime_last_error = 0U;
volatile uint32 can_busoff_cancel_count = 0U;
volatile uint8 can_deinit_failure = 0U;

static Std_ReturnType Det_ReportError(uint16 module, uint8 instance,
                                      uint8 service, uint8 error)
{
    (void)module;
    (void)instance;
    ++can_det_count;
    can_det_last_api = service;
    can_det_last_error = error;
    return E_OK;
}
static void Can_ReportDet(uint8 service, uint8 error)
{
#if (CAN_DEV_ERROR_DETECT == STD_ON)
    (void)Det_ReportError(CAN_MODULE_ID, CAN_INSTANCE_ID, service, error);
#else
    (void)service;
    (void)error;
#endif
}

/* Local SchM stand-in: save/restore PRIMASK; do not use fixed __enable_irq().
 * Valid for the single-core Cortex-M4 example, not a replacement for OS SchM.
 */
/* Runtime error reporting stand-in; requires real Det_ReportRuntimeError in BSW. */
static void Can_ReportRuntimeError(uint8 service, uint8 error)
{
    (void)service;
    ++can_runtime_count;
    can_runtime_last_error = error;
}
static uint32 Can_EnterCritical(void)
{
    uint32 primask;

    /* ARM Cortex-M4 (S32K144): read PRIMASK before masking IRQs. */
    __asm volatile("mrs %0, PRIMASK" : "=r"(primask) : : "memory");
    __asm volatile("cpsid i" ::: "memory");
    return primask;
}
static void Can_ExitCritical(uint32 primask)
{
    /* Restore original mask: never unconditionally enable interrupts. */
    __asm volatile("msr PRIMASK, %0" : : "r"(primask) : "memory");
}

static Can_ModuleStateType Can_ModuleState = CAN_UNINIT;
static volatile Can_ControllerStateType Can_ControllerState = CAN_CS_UNINIT;
static volatile Can_ControllerStateType Can_RequestedMode = CAN_CS_UNINIT;
static volatile boolean Can_ModePending = FALSE;
static volatile boolean Can_Hth0Mutex = FALSE;
static volatile boolean Can_Tx0Pending = FALSE;
static volatile PduIdType Can_Tx0SwHandle = 0U;
static const Can_ConfigType *Can_ActiveConfig = NULL_PTR;
static volatile uint16 Can_ActiveBaudrateId = 0U;
static volatile uint8 Can_IrqDisableDepth = 0U;
static volatile boolean Can_BusOffPending = FALSE;

/* SWS_Can_00259: create a real hardware STOPPED state. */
static Std_ReturnType Can_HwEnterFreeze(void)
{
    uint32 timeout = CAN_HW_TIMEOUT;
    CAN0->MCR |= CAN_MCR_FRZ_MASK | CAN_MCR_HALT_MASK;
    CAN0->MCR &= ~CAN_MCR_MDIS_MASK;
    while (((CAN0->MCR & CAN_MCR_FRZACK_MASK) == 0U) &&
           (timeout != 0U))
    {
        --timeout;
    }
    return (timeout != 0U) ? E_OK : E_NOT_OK;
}
static Std_ReturnType Can_HwConfigureTiming(const Can_BaudrateConfigType *b)
{
    uint32 tq, divisor, prescaler;
    if ((b == NULL_PTR) || (b->clockHz == 0U) ||
        (b->baudrateKbps == 0U) ||
        (b->propSegTq < 1U) || (b->propSegTq > 8U) ||
        (b->phaseSeg1Tq < 1U) || (b->phaseSeg1Tq > 8U) ||
        (b->phaseSeg2Tq < 2U) || (b->phaseSeg2Tq > 8U) ||
        (b->sjwTq < 1U) || (b->sjwTq > 4U) ||
        (b->sjwTq > b->phaseSeg2Tq))
    {
        return E_NOT_OK;
    }
    tq = 1U + b->propSegTq + b->phaseSeg1Tq + b->phaseSeg2Tq;
    divisor = (uint32)b->baudrateKbps * 1000UL * tq;
    if ((divisor == 0U) || (b->clockHz % divisor != 0U))
        return E_NOT_OK;
    prescaler = b->clockHz / divisor;
    if ((prescaler == 0U) || (prescaler > 256U))
        return E_NOT_OK;
    /* Preserve other CTRL1 fields; only update classical bit timing. */
    uint32 ctrl1 = CAN0->CTRL1;
    ctrl1 &= ~(CAN_CTRL1_PRESDIV_MASK | CAN_CTRL1_RJW_MASK |
               CAN_CTRL1_PSEG1_MASK | CAN_CTRL1_PSEG2_MASK |
               CAN_CTRL1_PROPSEG_MASK);
    ctrl1 |= CAN_CTRL1_PRESDIV(prescaler - 1U) |
             CAN_CTRL1_RJW(b->sjwTq - 1U) |
             CAN_CTRL1_PSEG1(b->phaseSeg1Tq - 1U) |
             CAN_CTRL1_PSEG2(b->phaseSeg2Tq - 1U) |
             CAN_CTRL1_PROPSEG(b->propSegTq - 1U) |
             CAN_CTRL1_BOFFREC_MASK; /* manual bus-off recovery */
    CAN0->CTRL1 = ctrl1;
    /* CLKSRC=0: 8 MHz oscillator. FlexCAN disabled when first selected. */
    return E_OK;
}
/* Search only ROM configuration declared to this driver instance. */
static const Can_BaudrateConfigType *Can_FindBaudrate(
    const Can_ConfigType *cfg, uint16 id)
{
    uint8 i;
    if ((cfg == NULL_PTR) || (cfg->baudrates == NULL_PTR))
        return NULL_PTR;
    for (i = 0U; i < cfg->baudrateCount; ++i)
    {
        if (cfg->baudrates[i].configId == id)
            return &cfg->baudrates[i];
    }
    return NULL_PTR;
}
static Std_ReturnType Can_HwInitStopped(const Can_ConfigType *cfg)
{
    uint32 i, timeout;
    const Can_BaudrateConfigType *timing;
    if ((cfg == NULL_PTR) || (cfg->baudrateCount == 0U) ||
        (cfg->baudrateCount > 8U) || (cfg->baudrates == NULL_PTR) || (cfg->hwObjects == NULL_PTR) ||
        (cfg->hwObjectCount != 3U))
        return E_NOT_OK;
    /* Explicitly restrict this first driver to MB0, MB4, MB5 and STD 11-bit. */
    if ((cfg->hwObjects[0].objectId != CAN_HTH_TX) ||
        (cfg->hwObjects[0].type != CAN_OBJECT_TRANSMIT) ||
        (cfg->hwObjects[0].mbIndex != CAN_TX_MB) ||
        (cfg->hwObjects[1].objectId != CAN_HRH_COMMAND) ||
        (cfg->hwObjects[1].mbIndex != CAN_CMD_RX_MB) ||
        (cfg->hwObjects[1].type != CAN_OBJECT_RECEIVE) ||
        (cfg->hwObjects[2].objectId != CAN_HRH_HEARTBEAT) ||
        (cfg->hwObjects[2].mbIndex != CAN_HB_RX_MB) ||
        (cfg->hwObjects[2].type != CAN_OBJECT_RECEIVE) ||
        (cfg->hwObjects[1].standardMask != 0x7FFU) ||
        (cfg->hwObjects[2].standardMask != 0x7FFU) ||
        (cfg->hwObjects[1].standardId > 0x7FFU) ||
        (cfg->hwObjects[2].standardId > 0x7FFU))
        return E_NOT_OK;

    timing = Can_FindBaudrate(cfg, cfg->defaultBaudrateId);
    if (timing == NULL_PTR)
        return E_NOT_OK;
    can_step = 3U;
    /* Select CLKSRC only when module disabled (FlexCAN requirement).
     * Works both on POR and after a successful Can_DeInit. */
    CAN0->MCR |= CAN_MCR_MDIS_MASK;
    timeout = CAN_HW_TIMEOUT;
    while (((CAN0->MCR & CAN_MCR_LPMACK_MASK) == 0U) &&
           (timeout != 0U))
    {
        --timeout;
    }
    if (timeout == 0U)
    {
        can_step = 108U;
        return E_NOT_OK;
    }
    CAN0->CTRL1 &= ~CAN_CTRL1_CLKSRC_MASK;
    CAN0->MCR &= ~CAN_MCR_MDIS_MASK;
    timeout = CAN_HW_TIMEOUT;
    while (((CAN0->MCR & CAN_MCR_LPMACK_MASK) != 0U) &&
           (timeout != 0U))
    {
        --timeout;
    }
    if (timeout == 0U)
    {
        can_step = 102U;
        return E_NOT_OK;
    }
    if (Can_HwEnterFreeze() != E_OK)
    {
        can_step = 103U;
        return E_NOT_OK;
    }
    CAN0->MCR &= ~(CAN_MCR_FDEN_MASK | CAN_MCR_RFEN_MASK |
                   CAN_MCR_MAXMB_MASK | CAN_MCR_IRMQ_MASK);
    CAN0->MCR |= CAN_MCR_MAXMB(CAN_MAX_MB) | CAN_MCR_SRXDIS_MASK;
    if (Can_HwConfigureTiming(timing) != E_OK)
    {
        can_step = 106U;
        return E_NOT_OK;
    }
    /* Preserve original FlexCAN RAM initialization (ECC/parity safe setup):
     * 32 Classical CAN MB slots, even though MAXMB=15 is selected. */
    for (i = 0U; i < 128U; ++i)
        CAN0->RAMn[i] = 0U;
    CAN0->RXMGMASK = 0xFFFFFFFFUL;               /* Global exact acceptance in 11-bit mode */
    CAN0->IMASK1 = 0U;                           /* polling-only: disable every MB interrupt */
    CAN0->RAMn[CAN_TX_MB * 4U] = (0x8UL << 24U); /* inactive */
    for (i = 1U; i <= 2U; ++i)
    {
        const Can_HardwareObjectConfigType *obj = &cfg->hwObjects[i];
        uint32 base = 4U * (uint32)obj->mbIndex;
        CAN0->RAMn[base] = 0U;
        CAN0->RAMn[base + 1U] = (uint32)obj->standardId << 18U;
        CAN0->RAMn[base + 2U] = 0U;
        CAN0->RAMn[base + 3U] = 0U;
        CAN0->RAMn[base] = (0x4UL << 24U); /* RX EMPTY */
    }
    CAN0->IFLAG1 = (1UL << CAN_TX_MB) |
                   (1UL << CAN_CMD_RX_MB) |
                   (1UL << CAN_HB_RX_MB); /* W1C */
    can_mcr = CAN0->MCR;
    can_ctrl1 = CAN0->CTRL1;
    can_esr1 = CAN0->ESR1;
    /* SWS_Can_00259: DO NOT clear HALT here. */
    if ((CAN0->MCR & CAN_MCR_FRZACK_MASK) == 0U)
        return E_NOT_OK;
    can_step = 11U;
    return E_OK;
}

/* Public AUTOSAR MCAL-style APIs */
void Can_Init(const Can_ConfigType *Config)
{
    if (Can_ModuleState != CAN_UNINIT)
    {
        Can_ReportDet(CAN_SID_INIT, CAN_E_TRANSITION);
        return;
    }
    if (Config == NULL_PTR)
    {
        Can_ReportDet(CAN_SID_INIT, CAN_E_PARAM_POINTER);
        return;
    }
    /* Failed MDIS/LPMACK deinit must be resolved by reset, not silently retried. */
    if (can_deinit_failure != 0U)
    {
        Can_ReportDet(CAN_SID_INIT, CAN_E_INIT_FAILED);
        return;
    }
    Can_ControllerState = CAN_CS_UNINIT;
    Can_Hth0Mutex = FALSE;
    Can_Tx0Pending = FALSE;
    Can_ModePending = FALSE;
    Can_BusOffPending = FALSE;
    Can_IrqDisableDepth = 0U;
    Can_ActiveConfig = NULL_PTR;
    can_deinit_failure = 0U;
    can_last_status = E_NOT_OK;
    if (Can_HwInitStopped(Config) != E_OK)
    {
        ++can_error_count;
        Can_ReportDet(CAN_SID_INIT, CAN_E_INIT_FAILED);
        return;
    }
    Can_ControllerState = CAN_CS_STOPPED;
    Can_ActiveConfig = Config;
    Can_ActiveBaudrateId = Config->defaultBaudrateId;
    Can_ModuleState = CAN_READY;
    can_last_status = E_OK;
}

/* SWS_Can_91002, 91009, 91010, 91011, 91012:
 * Can_DeInit is permitted only from READY with all controllers not STARTED.
 * Our demonstrator implements the STOPPED->UNINIT path only; SLEEP unsupported.
 */
void Can_DeInit(void)
{
    uint32 timeout;
    uint32 key;
    if (Can_ModuleState != CAN_READY || Can_ControllerState == CAN_CS_STARTED ||
        Can_ModePending != FALSE || Can_BusOffPending != FALSE ||
        Can_ControllerState != CAN_CS_STOPPED)
    {
        Can_ReportDet(CAN_SID_DEINIT, CAN_E_TRANSITION);
        return;
    }
    /* Module state MUST change before hardware deinitialization (91009). */
    key = Can_EnterCritical();
    Can_ModuleState = CAN_UNINIT;
    Can_ExitCritical(key);
    can_deinit_failure = 0U;

    /* STOPPED implies HALT+FRZACK; disable all controller interrupts. */
    CAN0->IMASK1 = 0U;
    CAN0->IFLAG1 = (1UL << CAN_TX_MB) | (1UL << CAN_CMD_RX_MB) |
                   (1UL << CAN_HB_RX_MB); /* W1C */
    CAN0->RAMn[CAN_TX_MB * 4U] = (0x8UL << 24U);
    CAN0->RAMn[CAN_CMD_RX_MB * 4U] = 0U; /* RX INACTIVE */
    CAN0->RAMn[CAN_HB_RX_MB * 4U] = 0U;
    CAN0->MCR |= CAN_MCR_MDIS_MASK;
    timeout = CAN_HW_TIMEOUT;
    while (((CAN0->MCR & CAN_MCR_LPMACK_MASK) == 0U) && timeout != 0U)
    {
        --timeout;
    }
    if (timeout == 0U)
    {
        ++can_error_count;
        can_deinit_failure = 1U;
        can_step = 107U; /* hardware deinit did not acknowledge */
    }
    Can_ControllerState = CAN_CS_UNINIT;
    Can_RequestedMode = CAN_CS_UNINIT;
    Can_ModePending = FALSE;
    Can_BusOffPending = FALSE;
    Can_Hth0Mutex = FALSE;
    Can_Tx0Pending = FALSE;
    Can_Tx0SwHandle = 0U;
    Can_IrqDisableDepth = 0U;
    Can_ActiveConfig = NULL_PTR;
    can_last_status = (can_deinit_failure == 0U) ? E_OK : E_NOT_OK;
}

#if (CAN_GET_VERSION_INFO_API == STD_ON)
void Can_GetVersionInfo(Std_VersionInfoType *versioninfo)
{
    if (versioninfo == NULL_PTR)
    {
        Can_ReportDet(CAN_SID_GET_VERSION, CAN_E_PARAM_POINTER);
        return;
    }
    versioninfo->vendorID = CAN_VENDOR_ID; /* educational vendor ID, not assigned */
    versioninfo->moduleID = CAN_MODULE_ID;
    versioninfo->sw_major_version = 0U;
    versioninfo->sw_minor_version = 3U;
    versioninfo->sw_patch_version = 0U;
}
#endif

/* Baudrate config IDs 0=500kbps, 1=250kbps; only while controller STOPPED.
 * No hardware reinitialization here: CTRL1 timing fields updated in Freeze.
 */
#if (CAN_SET_BAUDRATE_API == STD_ON)
Std_ReturnType Can_SetBaudrate(uint8 Controller, uint16 BaudRateConfigID)
{
    const Can_BaudrateConfigType *selected;
    if (Can_ModuleState != CAN_READY)
    {
        Can_ReportDet(CAN_SID_SET_BAUDRATE, CAN_E_UNINIT);
        return E_NOT_OK;
    }
    if (Controller != CAN_CTRL_0)
    {
        Can_ReportDet(CAN_SID_SET_BAUDRATE, CAN_E_PARAM_CONTROLLER);
        return E_NOT_OK;
    }
    selected = Can_FindBaudrate(Can_ActiveConfig, BaudRateConfigID);
    if (selected == NULL_PTR)
    {
        Can_ReportDet(CAN_SID_SET_BAUDRATE, CAN_E_PARAM_BAUDRATE);
        return E_NOT_OK;
    }
    if (Can_ControllerState != CAN_CS_STOPPED || Can_ModePending != FALSE ||
        Can_BusOffPending != FALSE ||
        (CAN0->MCR & CAN_MCR_FRZACK_MASK) == 0U)
        return E_NOT_OK;
    if (Can_HwConfigureTiming(selected) != E_OK)
        return E_NOT_OK;
    Can_ActiveBaudrateId = BaudRateConfigID;
    return E_OK;
}
#endif

/* Polling-only configuration: no FlexCAN IRQ enabled or restored.
 * Nested requests tracked per SWS_Can_00202. */
void Can_DisableControllerInterrupts(uint8 Controller)
{
    uint32 key;
    if (Can_ModuleState != CAN_READY)
    {
        Can_ReportDet(CAN_SID_DISABLE_IRQ, CAN_E_UNINIT);
        return;
    }
    if (Controller != CAN_CTRL_0)
    {
        Can_ReportDet(CAN_SID_DISABLE_IRQ, CAN_E_PARAM_CONTROLLER);
        return;
    }
    key = Can_EnterCritical();
    if (Can_IrqDisableDepth < 255U)
        ++Can_IrqDisableDepth;
    CAN0->IMASK1 = 0U; /* all HW objects use polling */
    Can_ExitCritical(key);
}
void Can_EnableControllerInterrupts(uint8 Controller)
{
    uint32 key;
    if (Can_ModuleState != CAN_READY)
    {
        Can_ReportDet(CAN_SID_ENABLE_IRQ, CAN_E_UNINIT);
        return;
    }
    if (Controller != CAN_CTRL_0)
    {
        Can_ReportDet(CAN_SID_ENABLE_IRQ, CAN_E_PARAM_CONTROLLER);
        return;
    }
    key = Can_EnterCritical();
    if (Can_IrqDisableDepth != 0U)
        --Can_IrqDisableDepth;
    /* No interrupts allowed by this POLLING-only configuration. */
    CAN0->IMASK1 = 0U;
    Can_ExitCritical(key);
}

static Std_ReturnType Can_CheckQueryArgs(uint8 controller, const void *ptr,
                                         uint8 service)
{
    if (Can_ModuleState != CAN_READY)
    {
        Can_ReportDet(service, CAN_E_UNINIT);
        return E_NOT_OK;
    }
    if (controller != CAN_CTRL_0)
    {
        Can_ReportDet(service, CAN_E_PARAM_CONTROLLER);
        return E_NOT_OK;
    }
    if (ptr == NULL_PTR)
    {
        Can_ReportDet(service, CAN_E_PARAM_POINTER);
        return E_NOT_OK;
    }
    return E_OK;
}
Std_ReturnType Can_GetControllerErrorState(uint8 ControllerId,
                                           Can_ErrorStateType *ErrorStatePtr)
{
    uint32 fltconf;
    if (Can_CheckQueryArgs(ControllerId, ErrorStatePtr, CAN_SID_GET_ERROR) != E_OK)
        return E_NOT_OK;
    fltconf = (CAN0->ESR1 >> 4U) & 3U; /* ESR1[FLTCONF] */
    if (fltconf == 0U)
        *ErrorStatePtr = CAN_ERRORSTATE_ACTIVE;
    else if (fltconf == 1U)
        *ErrorStatePtr = CAN_ERRORSTATE_PASSIVE;
    else if (fltconf == 2U)
        *ErrorStatePtr = CAN_ERRORSTATE_BUSOFF;
    else
        return E_NOT_OK; /* reserved encoding */
    return E_OK;
}
Std_ReturnType Can_GetControllerRxErrorCounter(uint8 ControllerId,
                                               uint8 *RxErrorCounterPtr)
{
    if (Can_CheckQueryArgs(ControllerId, RxErrorCounterPtr,
                           CAN_SID_GET_RX_COUNT) != E_OK)
        return E_NOT_OK;
    *RxErrorCounterPtr = (uint8)((CAN0->ECR >> 8U) & 0xFFUL);
    return E_OK;
}
Std_ReturnType Can_GetControllerTxErrorCounter(uint8 ControllerId,
                                               uint8 *TxErrorCounterPtr)
{
    if (Can_CheckQueryArgs(ControllerId, TxErrorCounterPtr,
                           CAN_SID_GET_TX_COUNT) != E_OK)
        return E_NOT_OK;
    *TxErrorCounterPtr = (uint8)(CAN0->ECR & 0xFFUL);
    return E_OK;
}

/* SWS Can_SetControllerMode asynchronous: request here, confirm in main function. */
Std_ReturnType Can_SetControllerMode(uint8 Controller,
                                     Can_ControllerStateType Transition)
{
    if (Can_ModuleState != CAN_READY)
    {
        Can_ReportDet(CAN_SID_SET_MODE, CAN_E_UNINIT);
        return E_NOT_OK;
    }
    if (Controller != CAN_CTRL_0)
    {
        Can_ReportDet(CAN_SID_SET_MODE, CAN_E_PARAM_CONTROLLER);
        return E_NOT_OK;
    }
    if ((Transition != CAN_CS_STARTED && Transition != CAN_CS_STOPPED) ||
        (Can_ModePending != FALSE))
        return E_NOT_OK;
    if (Transition == Can_ControllerState)
        return E_OK;
    if ((Transition == CAN_CS_STARTED) &&
        (Can_ControllerState != CAN_CS_STOPPED))
        return E_NOT_OK;
    if ((Transition == CAN_CS_STOPPED) &&
        (Can_ControllerState != CAN_CS_STARTED))
        return E_NOT_OK;
    Can_RequestedMode = Transition;
    Can_ModePending = TRUE;
    if (Transition == CAN_CS_STARTED)
    {
        /* Reapply the active bit timing while frozen before STARTED.
         * Full SWS_Can_00384 controller reinitialization remains pending. */
        const Can_BaudrateConfigType *timing =
            Can_FindBaudrate(Can_ActiveConfig, Can_ActiveBaudrateId);
        if ((CAN0->MCR & CAN_MCR_FRZACK_MASK) == 0U ||
            Can_Tx0Pending != FALSE ||
            Can_HwConfigureTiming(timing) != E_OK)
        {
            Can_ModePending = FALSE;
            return E_NOT_OK;
        }
        CAN0->MCR &= ~CAN_MCR_HALT_MASK;
    }
    else
        CAN0->MCR |= CAN_MCR_FRZ_MASK | CAN_MCR_HALT_MASK;
    return E_OK;
}
void Can_MainFunction_Mode(void)
{
    Can_ControllerStateType mode;
    if (Can_ModuleState != CAN_READY || Can_ModePending == FALSE)
        return;
    mode = Can_RequestedMode;
    if (mode == CAN_CS_STARTED)
    {
        if ((CAN0->MCR & (CAN_MCR_FRZACK_MASK | CAN_MCR_NOTRDY_MASK)) != 0U)
            return;
    }
    else if (mode == CAN_CS_STOPPED)
    {
        if ((CAN0->MCR & CAN_MCR_FRZACK_MASK) == 0U)
            return;
    }
    else
        return;
    /* SWS_Can_00282: STOPPED invalidates outstanding TX requests.
     * Do not call TX success confirmation for a cancelled message.
     * The controller has acknowledged Freeze before touching MB0. */
    if (mode == CAN_CS_STOPPED)
    {
        uint32 key = Can_EnterCritical();
        CAN0->RAMn[CAN_TX_MB * 4U] = (0x8UL << 24U);
        CAN0->IFLAG1 = (1UL << CAN_TX_MB); /* W1C */
        if (Can_Tx0Pending != FALSE)
            ++can_busoff_cancel_count;
        Can_Tx0Pending = FALSE;
        Can_Hth0Mutex = FALSE;
        Can_ExitCritical(key);
    }
    Can_ControllerState = mode;
    Can_ModePending = FALSE;
    CanIf_ControllerModeIndication(CAN_CTRL_0, mode);
}
Std_ReturnType Can_GetControllerMode(uint8 Controller,
                                     Can_ControllerStateType *ModePtr)
{
    if (Can_CheckQueryArgs(Controller, ModePtr, CAN_SID_GET_MODE) != E_OK)
        return E_NOT_OK;
    *ModePtr = Can_ControllerState;
    return E_OK;
}

/* One HTH mapped to TX MB0. SWS_Can_00212 / 00213 / 00214 / 00275. */
Std_ReturnType Can_Write(Can_HwHandleType Hth, const Can_PduType *PduInfo)
{
    uint32 key, i, word0 = 0U, word1 = 0U;
    uint32 cs, code;
    if (Can_ModuleState != CAN_READY)
    {
        Can_ReportDet(CAN_SID_WRITE, CAN_E_UNINIT);
        return E_NOT_OK;
    }
    if (Hth != CAN_HTH_TX)
    {
        Can_ReportDet(CAN_SID_WRITE, CAN_E_PARAM_HANDLE);
        return E_NOT_OK;
    }
    if ((PduInfo == NULL_PTR) ||
        (PduInfo->sdu == NULL_PTR))
    {
        Can_ReportDet(CAN_SID_WRITE, CAN_E_PARAM_POINTER);
        return E_NOT_OK;
    }
    if (PduInfo->length > 8U)
    {
        Can_ReportDet(CAN_SID_WRITE, CAN_E_PARAM_DATA_LENGTH);
        return E_NOT_OK;
    }
    /* Only 11-bit Classical CAN data frames in this example. */
    if ((PduInfo->id & ~0x7FFUL) != 0U)
        return E_NOT_OK;
    if (Can_ControllerState != CAN_CS_STARTED || Can_BusOffPending != FALSE ||
        ((CAN0->ESR1 & (3UL << 4U)) == (2UL << 4U)))
        return E_NOT_OK;

    key = Can_EnterCritical();
    if (Can_Hth0Mutex != FALSE)
    {
        Can_ExitCritical(key);
        return CAN_BUSY;
    }
    Can_Hth0Mutex = TRUE;
    Can_ExitCritical(key);

    cs = CAN0->RAMn[0];
    code = (cs >> 24U) & 0x0FU;
    /* pending covers a completed but not yet polled TX confirmation. */
    if ((Can_Tx0Pending != FALSE) || (code != 0x8U))
    {
        key = Can_EnterCritical();
        Can_Hth0Mutex = FALSE;
        Can_ExitCritical(key);
        return CAN_BUSY;
    }

    for (i = 0U; i < (uint32)PduInfo->length; ++i)
    {
        if (i < 4U)
            word0 |= (uint32)PduInfo->sdu[i] << (24U - (8U * i));
        else
            word1 |= (uint32)PduInfo->sdu[i] << (24U - 8U * (i - 4U));
    }
    CAN0->IFLAG1 = (1UL << CAN_TX_MB); /* W1C: old flag only */
    CAN0->RAMn[1] = (PduInfo->id & 0x7FFUL) << 18U;
    CAN0->RAMn[2] = word0;
    CAN0->RAMn[3] = word1;

    /* SWS_Can_00276: record handle BEFORE hardware sees TX activate. */
    Can_Tx0SwHandle = PduInfo->swPduHandle;
    Can_Tx0Pending = TRUE;
    __asm volatile("dmb" ::: "memory");
    CAN0->RAMn[0] = (0xCUL << 24U) | (1UL << 22U) |
                    ((uint32)PduInfo->length << 16U);
    key = Can_EnterCritical();
    Can_Hth0Mutex = FALSE;
    Can_ExitCritical(key);
    return E_OK; /* accepted, not yet transmitted */
}

/* SWS_Can_00225: successful completion confirmed from polling context. */
void Can_MainFunction_Write(void)
{
    PduIdType handle;
    uint32 key, code;
    if ((Can_ModuleState != CAN_READY) || (Can_Tx0Pending == FALSE))
        return;
    if ((CAN0->IFLAG1 & (1UL << CAN_TX_MB)) == 0U)
        return;
    code = (CAN0->RAMn[0] >> 24U) & 0x0FUL;
    /* No success confirmation for aborted or ambiguous TX events. */
    if ((code != 0x8U) ||
        ((CAN0->ESR1 & (3UL << 4U)) == (2UL << 4U)))
    {
        /* Don't overwrite a potentially failed frame; requires error handling. */
        ++can_error_count;
        return;
    }
    key = Can_EnterCritical();
    handle = Can_Tx0SwHandle;
    CAN0->IFLAG1 = (1UL << CAN_TX_MB);
    Can_Tx0Pending = FALSE;
    Can_ExitCritical(key);
    ++can_tx_count;
    can_esr1 = CAN0->ESR1;
    CanIf_TxConfirmation(handle); /* never inside critical section */
}

/* Receive mb4/mb5 in polling context; callback data valid until return. */
static void Can_HwPollReceive(uint8 mb, Can_HwHandleType hrh)
{
    uint32 base, cs, identifier, word0, word1, dummy, code;
    uint8 i, dlc, data[8];
    Can_HwType mailbox;
    PduInfoType pdu;
    if ((CAN0->IFLAG1 & (1UL << mb)) == 0U)
        return;
    base = (uint32)mb * 4U;
    cs = CAN0->RAMn[base];
    code = (cs >> 24U) & 0x0FUL;
    identifier = (CAN0->RAMn[base + 1U] >> 18U) & 0x7FFUL;
    word0 = CAN0->RAMn[base + 2U];
    word1 = CAN0->RAMn[base + 3U];
    dummy = CAN0->TIMER;
    (void)dummy; /* hardware RX unlock sequence */
    CAN0->IFLAG1 = (1UL << mb);
    dlc = (uint8)((cs >> 16U) & 0x0FU);
    if (code == 0x6U)
    {
        ++can_error_count;
        Can_ReportRuntimeError(CAN_SID_MAIN_READ, CAN_E_DATALOST);
    }
    if (((code != 0x2U) && (code != 0x6U)) ||
        (dlc > 8U) || ((cs & ((1UL << 21U) | (1UL << 20U))) != 0U))
        return; /* Reject extended/RTR/invalid frames for this application */
    for (i = 0U; i < dlc; ++i)
    {
        uint32 word = (i < 4U) ? word0 : word1;
        data[i] = (uint8)(word >> (24U - 8U * (uint32)(i & 3U)));
    }
    mailbox.CanId = identifier;
    mailbox.Hoh = hrh;
    mailbox.ControllerId = CAN_CTRL_0;
    pdu.SduDataPtr = data;
    pdu.SduLength = dlc;
    ++can_rx_count;
    CanIf_RxIndication(&mailbox, &pdu);
}
void Can_MainFunction_Read(void)
{
    if ((Can_ModuleState != CAN_READY) ||
        (Can_ControllerState != CAN_CS_STARTED))
        return;
    /* Heartbeat first, then command, matching system_OK.c order. */
    Can_HwPollReceive(CAN_HB_RX_MB, CAN_HRH_HEARTBEAT);
    Can_HwPollReceive(CAN_CMD_RX_MB, CAN_HRH_COMMAND);
}

void Can_MainFunction_BusOff(void)
{
    uint32 key;
    if (Can_ModuleState != CAN_READY)
        return;
    if (Can_ControllerState != CAN_CS_STARTED && Can_BusOffPending == FALSE)
        return;
    if (Can_BusOffPending == FALSE)
    {
        if ((CAN0->ESR1 & (3UL << 4U)) != (2UL << 4U))
            return;
        /* Hardware is bus-off: request frozen/non-participating controller. */
        Can_BusOffPending = TRUE;
        CAN0->MCR |= CAN_MCR_FRZ_MASK | CAN_MCR_HALT_MASK;
    }
    /* Do not claim STOPPED / notify CanIf until hardware freeze acknowledged. */
    if ((CAN0->MCR & CAN_MCR_FRZACK_MASK) == 0U)
        return;
    key = Can_EnterCritical();
    /* Stop all pending TX and invalidate associated swPduHandle. */
    if (Can_Tx0Pending != FALSE)
        ++can_busoff_cancel_count;
    CAN0->RAMn[CAN_TX_MB * 4U] = (0x8UL << 24U);
    CAN0->IFLAG1 = (1UL << CAN_TX_MB);
    Can_Tx0Pending = FALSE;
    Can_Hth0Mutex = FALSE;
    Can_ModePending = FALSE;
    Can_ControllerState = CAN_CS_STOPPED;
    Can_BusOffPending = FALSE;
    Can_ExitCritical(key);
    can_esr1 = CAN0->ESR1;
    CAN0->ESR1 = CAN_ESR1_BOFFINT_MASK; /* W1C latched bus-off event */
    CanIf_ControllerBusOff(CAN_CTRL_0);
}

/* Board debugging/monitoring only. Diagnostic API, not an AUTOSAR SWS API. */
Std_ReturnType Can_GetRawDiagnostics(uint32 *esr1, uint32 *ecr)
{
    if (Can_ModuleState != CAN_READY || esr1 == NULL_PTR || ecr == NULL_PTR)
        return E_NOT_OK;
    *esr1 = CAN0->ESR1;
    *ecr = CAN0->ECR;
    return E_OK;
}
