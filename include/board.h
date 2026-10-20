#ifndef BOARD_H
#define BOARD_H

#include <stdint.h>

extern volatile uint32_t g_ms;
extern volatile uint8_t g_limit_irq_stop;

void Board_WatchdogDisableBench(void);
uint8_t Board_TickInit(void);
void Board_MotorInit(void);
void Board_MotorSet(uint8_t direction, uint8_t pwm);
void Board_MotorStop(void);
void Board_SwitchInit(void);
uint8_t Board_SwitchPressed(void);
void Board_LedsInit(void);
void Board_LedsUpdate(uint8_t led_green, uint8_t led_red, uint8_t led_blue);

#endif
