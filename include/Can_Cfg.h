#ifndef AUTODOOR_CAN_CFG_H
#define AUTODOOR_CAN_CFG_H
/* One CAN Driver instance, vendor ID is DEMONSTRATION ONLY. */
#define CAN_MODULE_ID          80U
#define CAN_INSTANCE_ID         0U
#define CAN_VENDOR_ID           0U
#define STD_ON                 1U
#define STD_OFF                0U
#define CAN_DEV_ERROR_DETECT    STD_ON
#define CAN_BUSY               2U
#define CAN_SID_INIT           0x00U
#define CAN_SID_DEINIT         0x10U
#define CAN_SID_GET_VERSION    0x07U
#define CAN_SID_SET_BAUDRATE   0x0FU
#define CAN_SID_GET_MODE       0x12U
#define CAN_SID_GET_ERROR      0x11U
#define CAN_SID_GET_RX_COUNT   0x30U
#define CAN_SID_GET_TX_COUNT   0x31U
#define CAN_SID_DISABLE_IRQ    0x04U
#define CAN_SID_ENABLE_IRQ     0x05U
#define CAN_SID_MAIN_READ      0x08U
#define CAN_E_PARAM_BAUDRATE   0x07U
#define CAN_E_DATALOST        0x01U
#define CAN_GET_VERSION_INFO_API STD_ON
#define CAN_SET_BAUDRATE_API   STD_ON
#define CAN_WAKEUP_API        STD_OFF  /* no wakeup feature in demo */
#define CAN_TX_RX_PROCESSING_POLLING STD_ON
#define CAN_SID_SET_MODE       0x03U
#define CAN_SID_WRITE          0x06U
#define CAN_E_PARAM_POINTER    0x01U
#define CAN_E_PARAM_HANDLE     0x02U
#define CAN_E_PARAM_DATA_LENGTH 0x03U
#define CAN_E_PARAM_CONTROLLER 0x04U
#define CAN_E_UNINIT           0x05U
#define CAN_E_TRANSITION       0x06U
#define CAN_E_INIT_FAILED      0x09U

/* Hardware and user CAN protocol mapping. */
#define CAN_HTH_TX             0U
#define CAN_HRH_COMMAND        1U
#define CAN_HRH_HEARTBEAT      2U
#define CAN_CTRL_0            0U
#define CAN_TX_MB             0U
#define CAN_CMD_RX_MB         4U
#define CAN_HB_RX_MB          5U
#define CAN_MAX_MB            15U
#define CAN_HW_TIMEOUT        1000000UL


#endif
