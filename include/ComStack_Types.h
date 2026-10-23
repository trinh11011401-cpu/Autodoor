#ifndef AUTODOOR_COMSTACK_TYPES_H
#define AUTODOOR_COMSTACK_TYPES_H
#include "Std_Types.h"
typedef uint16 PduIdType;
typedef uint32 PduLengthType;
typedef struct {
    uint8 *SduDataPtr;
    PduLengthType SduLength;
} PduInfoType;
#endif
