#ifndef AUTODOOR_CAN_H
#define AUTODOOR_CAN_H
#include "Can_GeneralTypes.h"
#include "Can_Cfg.h"
typedef struct {
    uint16 configId;
    uint32 clockHz;       /* Actual FlexCAN clock; set by MCU/platform */
    uint16 baudrateKbps;
    uint8 propSegTq;
    uint8 phaseSeg1Tq;
    uint8 phaseSeg2Tq;
    uint8 sjwTq;
} Can_BaudrateConfigType;
typedef struct {
    Can_HwHandleType objectId;
    uint8 mbIndex;
    Can_ObjectType type;
    uint16 standardId;
    uint16 standardMask;
} Can_HardwareObjectConfigType;
typedef struct {
    const Can_BaudrateConfigType *baudrates;
    uint8 baudrateCount;
    uint16 defaultBaudrateId;
    const Can_HardwareObjectConfigType *hwObjects;
    uint8 hwObjectCount;
} Can_ConfigType;


extern const Can_ConfigType Can_Config;

void Can_Init(const Can_ConfigType *Config);
void Can_DeInit(void);
void Can_GetVersionInfo(Std_VersionInfoType *versioninfo);
Std_ReturnType Can_SetBaudrate(uint8 Controller, uint16 BaudRateConfigID);
void Can_DisableControllerInterrupts(uint8 Controller);
void Can_EnableControllerInterrupts(uint8 Controller);
Std_ReturnType Can_GetControllerErrorState(uint8 ControllerId,
                                          Can_ErrorStateType *ErrorStatePtr);
Std_ReturnType Can_GetControllerRxErrorCounter(uint8 ControllerId, uint8 *RxErrorCounterPtr);
Std_ReturnType Can_GetControllerTxErrorCounter(uint8 ControllerId, uint8 *TxErrorCounterPtr);
Std_ReturnType Can_SetControllerMode(uint8 Controller, Can_ControllerStateType Transition);
void Can_MainFunction_Mode(void);
Std_ReturnType Can_GetControllerMode(uint8 Controller, Can_ControllerStateType *ModePtr);
Std_ReturnType Can_Write(Can_HwHandleType Hth, const Can_PduType *PduInfo);
void Can_MainFunction_Write(void);
void Can_MainFunction_Read(void);
void Can_MainFunction_BusOff(void);

/* DEMO-ONLY diagnostic helper, NOT a seventh core AUTOSAR CAN API.
 * Application must not directly read CAN0 registers. */
Std_ReturnType Can_GetRawDiagnostics(uint32 *esr1, uint32 *ecr);

/* Developer diagnostics: not standardized AUTOSAR APIs. */
extern volatile uint32_t can_step;
extern volatile uint32_t can_mcr;
extern volatile uint32_t can_ctrl1;
extern volatile uint32_t can_esr1;
extern volatile uint32_t can_iflag1;
extern volatile uint32_t can_tx_count;
extern volatile uint32_t can_rx_count;
extern volatile uint32_t can_error_count;
extern volatile uint8_t can_last_status;
extern volatile uint32_t can_det_count;
extern volatile uint8 can_det_last_api;
extern volatile uint8 can_det_last_error;
extern volatile uint32 can_runtime_count;
extern volatile uint8 can_runtime_last_error;
extern volatile uint32 can_busoff_cancel_count;
extern volatile uint8 can_deinit_failure;

#endif
