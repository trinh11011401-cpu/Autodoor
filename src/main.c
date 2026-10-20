#include "app_config.h"
#include "app_protocol.h"
#include "board.h"
#include "can_bus.h"
#include "door.h"
#include "encoder.h"
#include "telemetry.h"

volatile uint32_t dbg_firmware_version = 20261020UL;
volatile uint32_t dbg_received_command = 0U;
volatile uint32_t dbg_valid_commands = 0U;

int main(void)
{
    CanFrame frame;
    uint32_t now;
    uint32_t last_led_ms = 0U;
    uint32_t last_rx_ms = 0U;
    uint32_t prev_rx_count = 0U;
    uint8_t can_ready;

    Board_WatchdogDisableBench();
    Board_MotorInit();
    Board_SwitchInit();
    Board_LedsInit();
    Encoder_Init();
    Door_Init();

    can_ready = Can_Init();
    if (Board_TickInit() == 0U || can_ready == 0U)
    {
        Board_MotorStop();
        dbg_fault |= FAULT_BUS_OFF;
        for (;;) { }
    }

    for (;;)
    {
        now = g_ms;
        if (Can_BusOff() != 0U) Door_BusOff(now);

        if (Can_ReadHeartbeat(&frame) != 0U &&
            frame.id == CAN_ID_HEARTBEAT && frame.dlc == 1U &&
            frame.data[0] == 0xA5U)
        {
            Door_Heartbeat(now);
        }
        if (Can_ReadCommand(&frame) != 0U &&
            frame.id == CAN_ID_COMMAND && frame.dlc == 2U)
        {
            dbg_received_command = ((uint32_t)frame.data[0] << 8U) |
                                   (uint32_t)frame.data[1];
            if (frame.data[0] <= CMD_JOG_CLOSE && frame.data[1] <= 100U)
            {
                ++dbg_valid_commands;
                Door_Command(frame.data[0], frame.data[1], now);
            }
        }
        Door_Tick(now);
        if (dbg_can_rx_count != prev_rx_count)
        {
            prev_rx_count = dbg_can_rx_count;
            last_rx_ms = now;
        }
        Can_TxPoll(now);
        Telemetry_Poll(now);
        if ((uint32_t)(now - last_led_ms) >= 20U)
        {
            last_led_ms = now;
            Board_LedsUpdate(Door_LinkOk(now),
                (dbg_fault & (FAULT_BLOCKING | FAULT_OBSTACLE)) != 0U,
                (uint32_t)(now - last_rx_ms) < 80U && dbg_can_rx_count != 0U);
        }
    }
}
