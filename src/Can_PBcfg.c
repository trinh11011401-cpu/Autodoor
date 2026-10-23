#include "Can.h"
/* Two selectable bitrate profiles (500 default, 250 optional); change only while STOPPED. */
static const Can_BaudrateConfigType Can_Baudrates[2] = {
    { 0U, 8000000UL, 500U, 5U, 7U, 3U, 3U },
    { 1U, 8000000UL, 250U, 5U, 7U, 3U, 3U }
};
static const Can_HardwareObjectConfigType Can_HardwareObjects[3] = {
    { CAN_HTH_TX, CAN_TX_MB, CAN_OBJECT_TRANSMIT, 0U, 0U },
    { CAN_HRH_COMMAND, CAN_CMD_RX_MB, CAN_OBJECT_RECEIVE, 0x100U, 0x7FFU },
    { CAN_HRH_HEARTBEAT, CAN_HB_RX_MB, CAN_OBJECT_RECEIVE, 0x102U, 0x7FFU }
};
const Can_ConfigType Can_Config = {
    Can_Baudrates, 2U, 0U,
    Can_HardwareObjects, 3U
};
