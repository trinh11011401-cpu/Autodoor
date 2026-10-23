#ifndef AUTODOOR_STD_TYPES_H
#define AUTODOOR_STD_TYPES_H
#include <stdint.h>
typedef uint8_t uint8;
typedef uint16_t uint16;
typedef uint32_t uint32;
typedef uint8 Std_ReturnType;
typedef uint8 boolean;
#define TRUE 1U
#define FALSE 0U
#define E_OK 0U
#define E_NOT_OK 1U
#define NULL_PTR ((void *)0)
/* Compact demo subset of AUTOSAR Std_VersionInfoType (Std_Types.h). */
typedef struct {
    uint16 vendorID;
    uint16 moduleID;
    uint8 sw_major_version;
    uint8 sw_minor_version;
    uint8 sw_patch_version;
} Std_VersionInfoType;


#endif
