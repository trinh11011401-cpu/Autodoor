#include "S32K144.h"
#include "app_config.h"
#include "encoder.h"

#define ENC_A_PIN 3U
#define ENC_B_PIN 2U
#define ENC_PIN_MASK ((1UL << ENC_A_PIN) | (1UL << ENC_B_PIN))

volatile int32_t dbg_encoder_count = 0L;
volatile uint32_t dbg_encoder_edges = 0U;
volatile uint32_t dbg_encoder_a_edges = 0U;
volatile uint32_t dbg_encoder_b_edges = 0U;
volatile uint32_t dbg_encoder_invalid = 0U;
volatile uint32_t dbg_encoder_counts_per_sec = 0U;
volatile uint32_t dbg_encoder_a_pulses_per_sec = 0U;
volatile uint32_t dbg_encoder_motor_rpm = 0U;
volatile float dbg_encoder_output_rpm = 0.0f;
volatile float dbg_encoder_angle_deg = 0.0f;
volatile float dbg_output_angle_deg = 0.0f;
volatile float dbg_encoder_revolutions = 0.0f;
volatile uint16_t dbg_encoder_speed_100ms = 0U;
volatile uint32_t dbg_encoder_sample_ms = 0U;
volatile uint32_t dbg_encoder_invalid_delta = 0U;

static volatile uint32_t a_rising = 0U;
static volatile uint8_t prev_ab = 0U;
static volatile int32_t raw_position = 0L;
static volatile int32_t zero_position = 0L;
static uint32_t sample_ms = 0U;
static uint32_t sample_edges = 0U;
static uint32_t sample_a_rising = 0U;
static uint32_t sample_invalid = 0U;

void Encoder_Init(void)
{
    uint32_t pins;
    PCC->PCCn[PCC_PORTB_INDEX] |= PCC_PCCn_CGC_MASK;
    PTB->PDDR &= ~ENC_PIN_MASK;
    PORTB->PCR[ENC_A_PIN] = PORT_PCR_MUX(1U) | PORT_PCR_PE_MASK |
                            PORT_PCR_PS_MASK | PORT_PCR_IRQC(11U);
    PORTB->PCR[ENC_B_PIN] = PORT_PCR_MUX(1U) | PORT_PCR_PE_MASK |
                            PORT_PCR_PS_MASK | PORT_PCR_IRQC(11U);
    pins = PTB->PDIR;
    prev_ab = (uint8_t)((((pins >> ENC_A_PIN) & 1UL) << 1U) |
                         ((pins >> ENC_B_PIN) & 1UL));
    PORTB->ISFR = ENC_PIN_MASK;
    S32_NVIC->IP[(uint32_t)PORTB_IRQn] = 0x60U;
    S32_NVIC->ICPR[(uint32_t)PORTB_IRQn / 32U] =
        1UL << ((uint32_t)PORTB_IRQn % 32U);
    S32_NVIC->ISER[(uint32_t)PORTB_IRQn / 32U] =
        1UL << ((uint32_t)PORTB_IRQn % 32U);
}

void PORTB_IRQHandler(void)
{
    static const int8_t step_table[16] = {
         0, -1,  1,  0,  1,  0,  0, -1,
        -1,  0,  0,  1,  0,  1, -1,  0
    };
    uint32_t pending = PORTB->ISFR & ENC_PIN_MASK;
    uint32_t pins;
    uint8_t curr, old, changed;
    int8_t step;
    if (pending == 0U) return;
    PORTB->ISFR = pending;
    pins = PTB->PDIR;
    curr = (uint8_t)((((pins >> ENC_A_PIN) & 1UL) << 1U) |
                     ((pins >> ENC_B_PIN) & 1UL));
    old = prev_ab;
    prev_ab = curr;
    changed = old ^ curr;
    if ((changed & 2U) != 0U)
    {
        ++dbg_encoder_a_edges;
        if ((curr & 2U) != 0U) ++a_rising;
    }
    if ((changed & 1U) != 0U) ++dbg_encoder_b_edges;
    if (changed == 3U) { ++dbg_encoder_invalid; return; }
    if (changed == 0U) return;
    step = step_table[((uint32_t)old << 2U) | curr];
    if (step != 0)
    {
        raw_position += (int32_t)step * ENC_OPEN_SIGN;
        dbg_encoder_count = raw_position - zero_position;
        ++dbg_encoder_edges;
    }
}

void Encoder_GetSnapshot(EncoderSnapshot *out)
{
    out->position = dbg_encoder_count;
    out->total = dbg_encoder_edges;
    out->a_edges = dbg_encoder_a_edges;
    out->b_edges = dbg_encoder_b_edges;
    out->a_rising = a_rising;
    out->invalid = dbg_encoder_invalid;
}

void Encoder_SetZero(void)
{
    /* Capture current origin without resetting the monotonic travel counter. */
    zero_position = raw_position;
    dbg_encoder_count = 0L;
}

void Encoder_UpdateSpeed(uint32_t now)
{
    EncoderSnapshot e;
    uint32_t dt = now - sample_ms;
    uint32_t delta;
    if (dt < SPEED_SAMPLE_MS) return;
    Encoder_GetSnapshot(&e);
    delta = e.total - sample_edges;
    dbg_encoder_sample_ms = dt;
    dbg_encoder_speed_100ms = (delta > 65535U) ? 65535U : (uint16_t)delta;
    dbg_encoder_counts_per_sec = (uint32_t)(((uint64_t)delta * 1000ULL) / dt);
    dbg_encoder_a_pulses_per_sec =
        (uint32_t)(((uint64_t)(e.a_rising - sample_a_rising) * 1000ULL) / dt);
    dbg_encoder_motor_rpm = (uint32_t)
        ((((uint64_t)delta * 60000ULL) + ((uint64_t)ENC_CPR * dt / 2ULL)) /
         ((uint64_t)ENC_CPR * dt));
    dbg_encoder_output_rpm = (float)dbg_encoder_motor_rpm / (float)GEAR_RATIO;
    dbg_encoder_revolutions = (float)e.total / (float)ENC_CPR;
    dbg_encoder_angle_deg = (float)e.position * 360.0f / (float)ENC_CPR;
    dbg_output_angle_deg = dbg_encoder_angle_deg / (float)GEAR_RATIO;
    dbg_encoder_invalid_delta = e.invalid - sample_invalid;
    sample_ms = now;
    sample_edges = e.total;
    sample_a_rising = e.a_rising;
    sample_invalid = e.invalid;
}
