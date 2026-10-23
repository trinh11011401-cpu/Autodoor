#ifndef AUTODOOR_CANIF_CBK_H
#define AUTODOOR_CANIF_CBK_H
#include "Can_GeneralTypes.h"
void CanIf_RxIndication(const Can_HwType *Mailbox, const PduInfoType *PduInfoPtr);
void CanIf_TxConfirmation(PduIdType CanTxPduId);
void CanIf_ControllerModeIndication(uint8 Controller, Can_ControllerStateType ControllerMode);
void CanIf_ControllerBusOff(uint8 Controller);
#endif
