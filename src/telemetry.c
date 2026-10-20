#include "app_config.h"
#include "app_protocol.h"
#include "can_bus.h"
#include "door.h"
#include "board.h"
#include "encoder.h"
#include "telemetry.h"

static uint32_t last_status_ms = 0U;
static uint32_t last_encoder_ms = 0U;
static uint8_t sequence = 0U;

void Telemetry_Poll(uint32_t now)
{
    CanFrame f = {0};
    EncoderSnapshot enc;
    if (dbg_status_dirty != 0U ||
        (uint32_t)(now - last_status_ms) >= STATUS_PERIOD_MS)
    {
        f.id = CAN_ID_STATUS;
        f.dlc = 6U;
        f.data[0] = dbg_state;
        f.data[1] = dbg_locked;
        f.data[2] = dbg_fault;
        f.data[3] = dbg_motor_pwm;
        f.data[4] = Door_StatusFlags(now);
        f.data[5] = sequence;
        if (Can_TrySend(&f, now) != 0U)
        {
            ++sequence;
            dbg_status_dirty = 0U;
            last_status_ms = now;
        }
    }
    else if ((uint32_t)(now - last_encoder_ms) >= ENCODER_PERIOD_MS)
    {
        uint32_t pos;
        Encoder_GetSnapshot(&enc);
        pos = (uint32_t)enc.position;
        f.id = CAN_ID_ENCODER;
        f.dlc = 8U;
        f.data[0] = (uint8_t)(pos >> 24U);
        f.data[1] = (uint8_t)(pos >> 16U);
        f.data[2] = (uint8_t)(pos >> 8U);
        f.data[3] = (uint8_t)(pos);
        f.data[4] = (uint8_t)(dbg_encoder_speed_100ms >> 8U);
        f.data[5] = (uint8_t)(dbg_encoder_speed_100ms);
        f.data[6] = dbg_motor_dir;
        f.data[7] = Board_SwitchPressed() ? 0U : 1U;
        if (Can_TrySend(&f, now) != 0U) last_encoder_ms = now;
    }
}
