#include "S32K144.h"
#include "app_config.h"
#include "board.h"
#include "app_protocol.h"

#define SWITCH_MASK  (1UL << 13U)
#define PWM_MASK     (1UL << 1U)
#define LED_BLUE     (1UL << 0U)
#define LED_RED      (1UL << 15U)
#define LED_GREEN    (1UL << 16U)

volatile uint32_t g_ms = 0U;
volatile uint8_t g_limit_irq_stop = 0U;
static volatile uint8_t active_motor_direction = CMD_STOP;

static uint32_t CoreClockHz(void)
{
    uint32_t src = (SCG->CSR & SCG_CSR_SCS_MASK) >> SCG_CSR_SCS_SHIFT;
    uint32_t div = ((SCG->CSR & SCG_CSR_DIVCORE_MASK) >> SCG_CSR_DIVCORE_SHIFT) + 1U;
    uint32_t hz = 0U;
    if (src == 3U) hz = 48000000UL;
    else if (src == 2U) hz = 8000000UL;
    else if (src == 1U) hz = (SCG->SIRCCFG & SCG_SIRCCFG_RANGE_MASK) ? 8000000UL : 2000000UL;
    else if (src == 6U)
    {
        uint32_t p = ((SCG->SPLLCFG & SCG_SPLLCFG_PREDIV_MASK) >> SCG_SPLLCFG_PREDIV_SHIFT) + 1U;
        uint32_t m = ((SCG->SPLLCFG & SCG_SPLLCFG_MULT_MASK) >> SCG_SPLLCFG_MULT_SHIFT) + 16U;
        hz = ((8000000UL / p) * m) / 2U;
    }
    return hz / div;
}

void Board_WatchdogDisableBench(void)
{
    WDOG->CNT = 0xD928C520UL;
    WDOG->TOVAL = 0x0000FFFFUL;
    WDOG->CS = 0x00002100UL;
}

uint8_t Board_TickInit(void)
{
    uint32_t reload = CoreClockHz() / 1000UL;
    if (reload == 0U || reload > 0x1000000UL) return 0U;
    S32_SysTick->CSR = 0U;
    S32_SysTick->RVR = reload - 1U;
    S32_SysTick->CVR = 0U;
    S32_SysTick->CSR = S32_SysTick_CSR_CLKSOURCE_MASK |
                       S32_SysTick_CSR_TICKINT_MASK | S32_SysTick_CSR_ENABLE_MASK;
    return 1U;
}

void SysTick_Handler(void) { ++g_ms; }

void Board_MotorStop(void)
{
    FTM0->CONTROLS[3].CnV = 0U;
    PORTD->PCR[1] = PORT_PCR_MUX(1U);
    PTD->PCOR = PWM_MASK;
    PTE->PCOR = 3UL;
    active_motor_direction = CMD_STOP;
}

void Board_MotorInit(void)
{
    PCC->PCCn[PCC_PORTE_INDEX] |= PCC_PCCn_CGC_MASK;
    PCC->PCCn[PCC_PORTD_INDEX] |= PCC_PCCn_CGC_MASK;
    PORTE->PCR[0] = PORT_PCR_MUX(1U);
    PORTE->PCR[1] = PORT_PCR_MUX(1U);
    PTE->PCOR = 3UL;
    PTE->PDDR |= 3UL;
    PORTD->PCR[1] = PORT_PCR_MUX(1U);
    PTD->PCOR = PWM_MASK;
    PTD->PDDR |= PWM_MASK;
    PCC->PCCn[PCC_FTM0_INDEX] &= ~PCC_PCCn_CGC_MASK;
    PCC->PCCn[PCC_FTM0_INDEX] = PCC_PCCn_PCS(1U) | PCC_PCCn_CGC_MASK;
    FTM0->SC = 0U;
    FTM0->MODE |= FTM_MODE_WPDIS_MASK;
    FTM0->COMBINE = 0U;
    FTM0->POL = 0U;
    FTM0->OUTMASK = 0U;
    FTM0->CNTIN = 0U;
    FTM0->CNT = 0U;
    FTM0->MOD = MOTOR_PWM_PERIOD;
    FTM0->CONTROLS[3].CnSC = FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK;
    FTM0->CONTROLS[3].CnV = 0U;
    FTM0->SC = FTM_SC_PWMEN3_MASK | FTM_SC_CLKS(3U) | FTM_SC_PS(0U);
    Board_MotorStop();
}

void Board_MotorSet(uint8_t direction, uint8_t pwm)
{
    uint32_t compare = ((uint32_t)pwm * (MOTOR_PWM_PERIOD + 1U)) / 100U;
    if (compare > MOTOR_PWM_PERIOD) compare = MOTOR_PWM_PERIOD;
    Board_MotorStop();
    if (direction == CMD_OPEN) { PTE->PCOR = 2UL; PTE->PSOR = 1UL; }
    else if (direction == CMD_CLOSE) { PTE->PCOR = 1UL; PTE->PSOR = 2UL; }
    else return;
    FTM0->CONTROLS[3].CnV = compare;
    PORTD->PCR[1] = PORT_PCR_MUX(2U);
    active_motor_direction = direction;
}

uint8_t Board_SwitchPressed(void)
{
    return ((PTD->PDIR & SWITCH_MASK) == 0U) ? 1U : 0U;
}

void Board_SwitchInit(void)
{
    PCC->PCCn[PCC_PORTD_INDEX] |= PCC_PCCn_CGC_MASK;
    PTD->PDDR &= ~SWITCH_MASK;
    PORTD->PCR[13] = PORT_PCR_MUX(1U) | PORT_PCR_PE_MASK |
                     PORT_PCR_PS_MASK | PORT_PCR_IRQC(10U);
    PORTD->ISFR = SWITCH_MASK;
    S32_NVIC->IP[(uint32_t)PORTD_IRQn] = 0x40U;
    S32_NVIC->ICPR[(uint32_t)PORTD_IRQn / 32U] =
        1UL << ((uint32_t)PORTD_IRQn % 32U);
    S32_NVIC->ISER[(uint32_t)PORTD_IRQn / 32U] =
        1UL << ((uint32_t)PORTD_IRQn % 32U);
}

void PORTD_IRQHandler(void)
{
    if ((PORTD->ISFR & SWITCH_MASK) != 0U)
    {
        PORTD->ISFR = SWITCH_MASK;
        if (active_motor_direction == CMD_CLOSE && Board_SwitchPressed() != 0U)
        {
            /* The switch immediately stops the drive. Door_Tick validates it. */
            Board_MotorStop();
            g_limit_irq_stop = 1U;
        }
    }
}

void Board_LedsInit(void)
{
    PTD->PSOR = LED_GREEN | LED_RED | LED_BLUE;
    PORTD->PCR[0] = PORT_PCR_MUX(1U);
    PORTD->PCR[15] = PORT_PCR_MUX(1U);
    PORTD->PCR[16] = PORT_PCR_MUX(1U);
    PTD->PDDR |= LED_GREEN | LED_RED | LED_BLUE;
}

void Board_LedsUpdate(uint8_t green, uint8_t red, uint8_t blue)
{
    uint32_t mask = LED_GREEN | LED_RED | LED_BLUE;
    uint32_t on = (green ? LED_GREEN : 0U) |
                  (red ? LED_RED : 0U) | (blue ? LED_BLUE : 0U);
    PTD->PSOR = mask & ~on;
    PTD->PCOR = on;
}
