#ifndef DOOR_H
#define DOOR_H
#include <stdint.h>
#include "app_protocol.h"

extern volatile uint8_t dbg_state;
extern volatile uint8_t dbg_locked;
extern volatile uint8_t dbg_fault;
extern volatile uint8_t dbg_last_reject;
extern volatile uint8_t dbg_motor_dir;
extern volatile uint8_t dbg_motor_pwm;
extern volatile uint8_t dbg_homed;
extern volatile uint8_t dbg_obstacle_events;
extern volatile uint32_t dbg_last_run_ms;
extern volatile uint32_t dbg_last_run_counts;
extern volatile float dbg_last_run_average_rpm;
extern volatile float dbg_last_run_output_angle_deg;
extern volatile uint32_t dbg_last_running_pps;
extern volatile uint32_t dbg_last_fault_at_ms;
extern volatile uint8_t dbg_last_fault_cause;
extern volatile uint32_t dbg_stall_baseline_counts;
extern volatile uint32_t dbg_stall_last_counts;
extern volatile uint8_t dbg_stall_low_count;
extern volatile uint32_t dbg_last_hb_ms;
extern volatile uint32_t dbg_hb_count;
extern volatile uint8_t dbg_status_dirty;

void Door_Init(void);
void Door_Heartbeat(uint32_t now);
uint8_t Door_LinkOk(uint32_t now);
void Door_Command(uint8_t cmd, uint8_t pwm, uint32_t now);
void Door_Tick(uint32_t now);
void Door_BusOff(uint32_t now);
uint8_t Door_StatusFlags(uint32_t now);

#endif
