#ifndef CAN_BUS_H
#define CAN_BUS_H
#include <stdint.h>
#include "app_protocol.h"

extern volatile uint32_t dbg_can_rx_count;
extern volatile uint32_t dbg_can_tx_count;
extern volatile uint32_t dbg_can_error_count;
extern volatile uint32_t dbg_can_init_step;

uint8_t Can_Init(void);
uint8_t Can_ReadCommand(CanFrame *frame);
uint8_t Can_ReadHeartbeat(CanFrame *frame);
uint8_t Can_TrySend(const CanFrame *frame, uint32_t now);
void Can_TxPoll(uint32_t now);
uint8_t Can_BusOff(void);

#endif
