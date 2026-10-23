#ifndef AUTODOOR_CAN_GENERAL_TYPES_H
#define AUTODOOR_CAN_GENERAL_TYPES_H
#include "Std_Types.h"
#include "ComStack_Types.h"
typedef uint16 Can_HwHandleType;
typedef uint32 Can_IdType;
typedef struct {
    PduIdType swPduHandle;
    uint8 length;
    Can_IdType id;
    uint8 *sdu;
} Can_PduType;
typedef struct {
    Can_IdType CanId;
    Can_HwHandleType Hoh;
    uint8 ControllerId;
} Can_HwType;
typedef enum {
    CAN_CS_UNINIT = 0,
    CAN_CS_STARTED,
    CAN_CS_STOPPED,
    CAN_CS_SLEEP
} Can_ControllerStateType;
typedef enum {
    CAN_ERRORSTATE_ACTIVE = 0,
    CAN_ERRORSTATE_PASSIVE,
    CAN_ERRORSTATE_BUSOFF
} Can_ErrorStateType;
typedef enum {
    CAN_UNINIT = 0,
    CAN_READY
} Can_ModuleStateType;
typedef enum {
    CAN_OBJECT_TRANSMIT = 0,
    CAN_OBJECT_RECEIVE
} Can_ObjectType;


#endif
