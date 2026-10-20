#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include <stdint.h>

/* S32K144EVB: external SOSC 8 MHz; FlexCAN 500 kbps; FTM0 PWM 20 kHz. */
#define CAN_BITRATE             500000UL
#define MOTOR_PWM_PERIOD       399U
#define MOTOR_PWM_MAX          90U
#define MOTOR_PWM_MIN          35U
#define MOTOR_RECOVERY_PWM     50U

/* Encoder on motor shaft: 11 periods/channel/rev, A/B quadrature x4. */
#define ENC_PPR                 11U
#define ENC_CPR                 (ENC_PPR * 4U)
#define ENC_OPEN_SIGN          1L  /* Flip to -1 if OPEN decreases signed counts. */
#define GEAR_RATIO             40U
#define TARGET_OUTPUT_DEG      60U
#define TARGET_COUNTS_NOMINAL  ((ENC_CPR * GEAR_RATIO * TARGET_OUTPUT_DEG + 180U) / 360U)

/* Replace with the measured CLOSED -> 60 degree encoder count if available. */
#define DOOR_OPEN_TARGET_COUNTS TARGET_COUNTS_NOMINAL

#define HEARTBEAT_TIMEOUT_MS   700U
#define CLOSED_DEBOUNCE_MS     15U
#define OPEN_SWITCH_RELEASE_MS 300U
#define MAX_TRAVEL_MS          6500U
#define REVERSE_DEADTIME_MS    120U
#define START_CONFIRM_MS       300U
#define START_MIN_COUNTS       6U
#define SPEED_SAMPLE_MS        100U
#define STALL_SAMPLE_MS        25U
#define STALL_START_GRACE_MS   80U
#define STALL_THRESHOLD_PCT    20U
#define STALL_LOW_SAMPLES      3U
#define STALL_MIN_COUNTS       2U
#define MIN_BASELINE_COUNTS    8U
#define INVALID_SAMPLE_LIMIT   8U
#define STATUS_PERIOD_MS       250U
#define ENCODER_PERIOD_MS      250U

/* 0: normal motion protection; 1: motor-only bench test (anti-stall disabled). */
#define BENCH_ENCODER_TEST     0U
#define BENCH_RUN_MS           2000U

#endif
