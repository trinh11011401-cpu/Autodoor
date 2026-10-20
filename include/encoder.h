#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

typedef struct {
    int32_t position;
    uint32_t total;
    uint32_t a_edges;
    uint32_t b_edges;
    uint32_t a_rising;
    uint32_t invalid;
} EncoderSnapshot;

extern volatile int32_t dbg_encoder_count;
extern volatile uint32_t dbg_encoder_edges;
extern volatile uint32_t dbg_encoder_a_edges;
extern volatile uint32_t dbg_encoder_b_edges;
extern volatile uint32_t dbg_encoder_invalid;
extern volatile uint32_t dbg_encoder_counts_per_sec;
extern volatile uint32_t dbg_encoder_a_pulses_per_sec;
extern volatile uint32_t dbg_encoder_motor_rpm;
extern volatile float dbg_encoder_output_rpm;
extern volatile float dbg_encoder_angle_deg;
extern volatile float dbg_output_angle_deg;
extern volatile float dbg_encoder_revolutions;
extern volatile uint16_t dbg_encoder_speed_100ms;
extern volatile uint32_t dbg_encoder_sample_ms;
extern volatile uint32_t dbg_encoder_invalid_delta;

void Encoder_Init(void);
void Encoder_GetSnapshot(EncoderSnapshot *out);
void Encoder_SetZero(void);
void Encoder_UpdateSpeed(uint32_t now);

#endif
