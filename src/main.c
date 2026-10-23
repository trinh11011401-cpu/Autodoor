/* AutoDoor ver3 application integrated with AUTOSAR CP4.4 inspired CAN Driver.
 * Keeps encoder 146 counts, PWM 65%, obstacle recovery and fail-safe CAN policy.
 * This is an educational 6-API demonstration, not certified AUTOSAR MCAL.
 */
#include "S32K144.h"
#include <stdint.h>
#include "Can.h"
#include "Board_Can.h"
#include "CanIf_Cbk.h"

/* =======================================================================
 * AUTODOOR - S32K144EVB, single file. Encoder-only obstacle detection.
 *   CAN0       : PTE4 RX / PTE5 TX (ALT5), 500 kbps, standard CAN.
 *   Motor      : PTD1 FTM0_CH3 PWM / PTE0 IN1 / PTE1 IN2 -> L298N.
 *   Encoder A  : PTB3 GPIO, both-edge PORTB IRQ (3.3V input).
 *   Encoder B  : PTB2 GPIO, both-edge PORTB IRQ (3.3V input).
 *   CLOSED NO  : PTD13 (J2-1) -> external switch -> GND; internal pullup.
 *   On-board   : PTD16 green CAN; PTD15 red fault; PTD0 blue limit.
 *
 * No ACS712. No automatic motion after power-up. L298N ENA requires an
 * external pull-down; firmware alone cannot guarantee deenergized reset.
 * This is a laboratory prototype, not an automotive safety-certified ECU.
 * ======================================================================= */

/* ----------------------- USER CALIBRATION ----------------------------- */
/* Measured: 293 encoder x4 counts moved the output shaft about 120 deg.
 * Initial 60-deg target = round(293 * 60 / 120) = 147 counts.
 * Use 146 as requested for the first bench trial; adjust after measuring
 * the final angle, including any coast/overshoot after PWM is switched off.
 * The 40:1 nominal gearbox ratio is NOT used to infer this target.
 */
#define ENCODER_PPR              11U
#define ENCODER_X4_CPR           (4U * ENCODER_PPR)
#define GEAR_RATIO_NOMINAL       40U  /* Reference only, not used for endpoint */
#define OPEN_ANGLE_DEG           60U
#define DOOR_TRAVEL_COUNTS       146L /* Preliminary endpoint based on 293->120deg */
#define OPEN_TARGET_CALIBRATED   0U  /* 0=approximate; change to 1 after angle verification */
#define ENCODER_OPEN_SIGN        1L  /* initial mapping; corrected once during first OPEN */
#define ENC_A_PIN                 3U
#define ENC_B_PIN                 2U
#define ENC_MASK                 ((1UL << ENC_A_PIN) | (1UL << ENC_B_PIN))
#define CALIBRATION_MODE         0U   /* 0=normal; 1=short JOG calibration only */
#define REQUIRE_HEARTBEAT        1U   /* 0x102 [A5] every 200 ms */
/* ---------------------------------------------------------------------- */

#define ID_CMD                  0x100U
#define ID_STATE                0x101U
#define ID_HEARTBEAT            0x102U
#define ID_ENCODER              0x103U

#define CMD_STOP                0U
#define CMD_OPEN                1U
#define CMD_CLOSE               2U
#define CMD_LOCK                3U
#define CMD_UNLOCK              4U
#define CMD_CLEAR               5U
#define CMD_JOG_OPEN            6U
#define CMD_JOG_CLOSE           7U

#define DOOR_UNKNOWN            0U
#define DOOR_CLOSED             1U
#define DOOR_OPENING            2U
#define DOOR_OPENED             3U
#define DOOR_CLOSING            4U
#define DOOR_STOPPED            5U

#define FAULT_OBSTACLE          0x01U  /* Event: first stall, reversing */
#define FAULT_CAN_LOST          0x02U
#define FAULT_BUS_OFF           0x04U
#define FAULT_ENCODER           0x08U  /* Encoder absent/broken or repeat stall */
#define FAULT_TRAVEL            0x10U  /* Limit/travel time expired */
#define FAULT_LIMIT             0x20U  /* CLOSED switch inconsistent */
#define FAULT_UNHOMED           0x40U  /* Position unreferenced */
#define FAULT_POWER_UP          0x80U  /* Informational: last restart */
#define FAULT_BLOCKING_MASK     (FAULT_CAN_LOST | FAULT_BUS_OFF | \
                                 FAULT_ENCODER | FAULT_TRAVEL | FAULT_LIMIT)

#define FLAG_CLOSED_SWITCH      0x01U
#define FLAG_HOMED              0x02U
#define FLAG_REVERSING          0x04U
#define FLAG_HEARTBEAT_OK       0x08U
#define FLAG_CALIBRATING        0x10U
#define FLAG_ENCODER_PROGRESS   0x20U
#define FLAG_OPEN_ESTIMATED     0x40U /* OPEN target learned from 2s, not calibrated */

#define PWM_PERIOD              399U /* 8MHz/(399+1)=20kHz, formerly 10kHz */
#define PWM_MAX                 90U
#define PWM_MIN_MOVING          50U  /* JOG only */
#define MOTOR_NORMAL_PWM        65U  /* OPEN, CLOSE and automatic reversal */
#define JOG_PWM_MAX             60U
#define JOG_PULSE_MS            180UL
#define CLOSED_PIN              13U
#define CLOSED_MASK             (1UL << CLOSED_PIN)
#define CLOSED_DEBOUNCE_MS      15UL
#define OPEN_LIMIT_RELEASE_MS   500UL
#define HEARTBEAT_TIMEOUT_MS    700UL
#define STATUS_PERIOD_MS        250UL
#define POSITION_PERIOD_MS      250UL
#define STALL_GRACE_MS          220UL
#define START_MOTION_VERIFY_MS  450UL
#define STALL_WINDOW_MS         120UL
#define STALL_NO_EDGE_MS        300UL /* No new A/B count for 300 ms => possible stall. */
#define MIN_WINDOW_PROGRESS     3L
#define MAX_TRAVEL_MS           6500UL  /* CLOSE global safety timeout */
#define OPEN_MAX_RUN_MS         2000UL  /* maximum OPEN duration, even in reverse */
#define FIRST_OPEN_TIMED_MS     2000UL  /* legacy fallback only if target=0 */
#define REVERSAL_WAIT_MS        120UL
#define CAN_TX_GUARD_MS         1000UL
#define CAN_RECOVER_RETRY_MS     1000UL  /* peripheral retry, motor remains OFF */
#define LED_GREEN               (1UL << 16U)
#define LED_RED                 (1UL << 15U)
#define LED_BLUE                (1UL << 0U)

/* Variables for S32DS Expressions and 0x101 / 0x103. */
volatile uint32_t dbg_firmware_version = 2026102302UL; /* ver3 integrated 6 API AUTOSAR CAN */
volatile uint32_t dbg_ms = 0UL;
volatile uint32_t dbg_core_hz = 0UL;
volatile uint32_t dbg_valid_cmd = 0UL;
volatile uint32_t dbg_hb_count = 0UL;
/* CAN diagnostics: ESR1 FLTCONF[5:4] == 2 means actual Bus-Off.
 * Read these in S32DS AFTER the fault; the snapshot is latched. */
volatile uint32_t dbg_can_esr1_now = 0UL;
volatile uint32_t dbg_can_ecr_now = 0UL;
volatile uint32_t dbg_can_esr1_at_busoff = 0UL;
volatile uint32_t dbg_can_ecr_at_busoff = 0UL;
volatile uint32_t dbg_can_busoff_ms = 0UL;
volatile uint32_t dbg_can_busoff_count = 0UL;
volatile uint32_t dbg_can_ackerr_seen = 0UL;
volatile uint32_t dbg_can_biterr_seen = 0UL;
volatile uint32_t dbg_can_frameerr_seen = 0UL;
volatile uint32_t dbg_can_rx_at_busoff = 0UL;
volatile uint32_t dbg_can_tx_at_busoff = 0UL;
volatile uint32_t dbg_can_reinit_attempts = 0UL;
volatile uint32_t dbg_can_reinit_success = 0UL;
volatile uint32_t dbg_can_reinit_failed = 0UL;
volatile uint8_t dbg_can_recovery_pending = 0U;

volatile uint32_t dbg_last_cmd_word = 0UL;
volatile uint32_t dbg_last_hb_ms = 0UL;
volatile uint32_t dbg_limit_irq_count = 0UL;
volatile uint32_t dbg_limit_tick_stops = 0UL;
/* GPIO encoder diagnostics: independent raw A/B transitions and valid x4 steps. */
volatile uint32_t dbg_encoder_edges = 0UL;       /* valid quadrature x4 steps */
volatile uint32_t dbg_encoder_raw_edges = 0UL;   /* any observed change on A or B */
volatile uint32_t dbg_encoder_a_edges = 0UL;     /* PTB3 changes, both polarities */
volatile uint32_t dbg_encoder_b_edges = 0UL;     /* PTB2 changes, both polarities */
volatile uint32_t dbg_encoder_invalid = 0UL;     /* 2-bit simultaneous changes */
volatile uint32_t dbg_encoder_irq_count = 0UL;   /* times PORTB_IRQHandler runs */
volatile uint32_t dbg_encoder_irq_a_flags = 0UL;
volatile uint32_t dbg_encoder_irq_b_flags = 0UL;
volatile uint8_t dbg_encoder_ab_state = 0U;     /* bit1=A, bit0=B */
volatile uint8_t dbg_encoder_pin_levels = 0U;   /* live A/B levels (1ms) */
volatile uint32_t dbg_encoder_pcr2 = 0UL;
volatile uint32_t dbg_encoder_pcr3 = 0UL;
volatile uint32_t dbg_stall_count = 0UL;
volatile uint32_t dbg_can_timeout_count = 0UL;
volatile uint32_t dbg_status_tx_count = 0UL;
volatile uint32_t dbg_tx_stuck_count = 0UL;
volatile uint8_t dbg_limit_level = 1U; /* PTD13: 1 unpressed, 0 pressed */
volatile uint8_t dbg_limit_stable = 0U;
volatile uint8_t dbg_last_reject = 0U;
volatile uint8_t dbg_motor_dir = CMD_STOP;
volatile uint8_t dbg_motor_pwm = 0U;
volatile uint8_t dbg_state = DOOR_UNKNOWN;
volatile uint8_t dbg_locked = 1U;
volatile uint8_t dbg_fault = FAULT_POWER_UP | FAULT_UNHOMED;
volatile uint8_t dbg_homed = 0U;
volatile uint8_t dbg_recovery = 0U;
volatile uint8_t dbg_motion_progress = 0U;
volatile int32_t dbg_encoder_count = 0L;
volatile int32_t dbg_encoder_open_sign = ENCODER_OPEN_SIGN;
volatile uint8_t dbg_encoder_sign_learned = 0U;
volatile uint32_t dbg_open_completed_count = 0U;
volatile uint32_t dbg_closed_completed_count = 0U;
volatile uint32_t dbg_open_time_limit_count = 0U;
volatile uint32_t dbg_obstacle_clear_count = 0U;
volatile uint8_t dbg_obstacle_clear_at_state = 0U;
volatile uint32_t dbg_recovery_open_completed_count = 0U;
volatile uint32_t dbg_recovery_close_completed_count = 0U;
volatile int32_t dbg_open_target_count = 0L;
volatile uint8_t dbg_open_target_valid = 0U;
volatile uint8_t dbg_open_estimated = 0U;
volatile uint32_t dbg_pwm_hz = 20000UL;
volatile int32_t dbg_last_window_progress = 0L;
volatile uint16_t dbg_encoder_speed_100ms = 0U; /* x4 counts / latest ~100ms */
volatile uint32_t dbg_encoder_pps = 0U;         /* x4 counts per second, live */
volatile uint32_t dbg_encoder_rpm = 0U;         /* encoder shaft RPM, live */
volatile uint32_t dbg_encoder_run_edges = 0U;   /* accumulated during current run */
volatile uint32_t dbg_encoder_run_ms = 0U;
volatile uint32_t dbg_encoder_run_pps = 0U;     /* average since motor START */
volatile uint32_t dbg_encoder_run_rpm = 0U;
volatile uint32_t dbg_last_run_edges = 0U;      /* retained after STOP */
volatile uint32_t dbg_last_run_avg_pps = 0U;
volatile uint32_t dbg_last_run_avg_rpm = 0U;
volatile uint32_t dbg_last_nonzero_pps = 0U;  /* retained after STOP */
volatile uint32_t dbg_last_nonzero_rpm = 0U;
volatile uint32_t dbg_last_fault_edges = 0U;

/* STARTUP DIAGNOSTICS -- persist after the motor has been stopped. */
volatile uint32_t dbg_motor_start_count = 0U;
volatile uint32_t dbg_motor_start_ms = 0U;
volatile uint8_t  dbg_motor_start_direction = 0U;
volatile uint16_t dbg_motor_applied_compare = 0U;
volatile uint16_t dbg_ftm0_compare_now = 0U;
volatile uint16_t dbg_ftm1_count_now = 0U; /* LEGACY, no longer a valid encoder readout */
volatile int32_t  dbg_motor_start_encoder = 0L;
volatile int32_t  dbg_start_progress = 0L;
volatile uint8_t  dbg_start_motion_confirmed = 0U;
volatile uint32_t dbg_start_no_motion_count = 0U;
volatile uint8_t  dbg_last_fault_cause = 0U;
volatile uint8_t  dbg_last_fault_motor_dir = 0U;
volatile uint32_t dbg_last_fault_run_ms = 0U;
volatile int32_t dbg_last_fault_progress = 0L;
volatile uint32_t dbg_last_motor_run_ms = 0U;
volatile uint8_t  dbg_last_motor_run_dir = 0U;


static volatile uint8_t status_due = 1U;
static uint8_t tx_busy = 0U;
static uint8_t tx_kind = 0U;
static uint8_t seq_no = 0U;
static volatile uint8_t pending_cmd = CMD_STOP;
static uint8_t pending_pwm = 0U;
static uint8_t pending_recovery = 0U;
static volatile uint8_t reverse_previous = CMD_STOP;
static volatile uint8_t calibration_jog = 0U;
static volatile uint8_t first_timed_open = 0U;
static uint8_t closed_db_last = 0U;
static volatile uint8_t limit_trip_pending = 0U;
static volatile uint8_t stall_event = 0U;
static volatile uint8_t stall_from_recovery = 0U;
static uint8_t tx_reserved = 0U;
static uint8_t can_recovery_pending = 0U;
static uint32_t can_recovery_next_ms = 0UL;

static volatile uint32_t motion_start_ms = 0UL;
static volatile uint8_t startup_motion_confirmed = 0U;
static volatile uint32_t last_valid_cmd_ms = 0UL;
static volatile uint32_t last_motor_off_ms = 0UL;
static volatile uint8_t last_motor_dir = CMD_STOP;
static volatile uint32_t last_window_ms = 0UL;
static uint32_t pending_at_ms = 0UL;
static uint32_t last_status_ms = 0UL;
static uint32_t last_position_ms = 0UL;
static uint32_t last_tx_started_ms = 0UL;
static uint32_t limit_changed_ms = 0UL;
static volatile uint32_t window_start_edges = 0UL;
static volatile uint32_t last_observed_edges = 0UL;
static volatile uint32_t last_edge_ms = 0UL;
static volatile uint32_t dbg_motor_start_edges = 0UL;
static uint32_t speed_start_edges = 0UL;
static uint32_t last_speed_ms = 0UL;
static volatile uint8_t encoder_prev_ab = 0U;
static volatile int32_t encoder_open_sign = ENCODER_OPEN_SIGN;
static volatile uint32_t last_valid_raw_edges = 0UL;

/* Recovery state 0=normal, 1=deadtime before reverse, 2=reversing. */
/* For manual direction change, pending_recovery=0 and pending_cmd is set. */

static uint32_t CoreClock_Hz(void)
{
    uint32_t src = (SCG->CSR & SCG_CSR_SCS_MASK) >> SCG_CSR_SCS_SHIFT;
    uint32_t div = ((SCG->CSR & SCG_CSR_DIVCORE_MASK) >> SCG_CSR_DIVCORE_SHIFT) + 1U;
    uint32_t hz = 0U;
    if (src == 3U) hz = 48000000UL;       /* FIRC: default 48 MHz */
    else if (src == 2U) hz = 8000000UL;   /* SOSC fitted 8 MHz crystal */
    else if (src == 1U)
    {
        /* SIRC 2 or 8 MHz; do not silently assume if unsupported. */
        hz = ((SCG->SIRCCFG & SCG_SIRCCFG_RANGE_MASK) != 0U) ?
             8000000UL : 2000000UL;
    }
    else if (src == 6U)
    {
        uint32_t pre = ((SCG->SPLLCFG & SCG_SPLLCFG_PREDIV_MASK) >>
                         SCG_SPLLCFG_PREDIV_SHIFT) + 1U;
        uint32_t mul = ((SCG->SPLLCFG & SCG_SPLLCFG_MULT_MASK) >>
                         SCG_SPLLCFG_MULT_SHIFT) + 16U;
        hz = ((8000000UL / pre) * mul) / 2U;
    }
    return (hz == 0U) ? 0U : (hz / div);
}

static void WDOG_Off_BenchOnly(void)
{
    /* For laboratory debugging only; production requires a real watchdog. */
    WDOG->CNT = 0xD928C520U;
    WDOG->TOVAL = 0x0000FFFFU;
    WDOG->CS = 0x00002100U;
}

/* Count is encoder x4 transitions; dt is measured time in milliseconds. */
static uint32_t Encoder_CountsPerSecond(uint32_t count, uint32_t dt_ms)
{
    if (dt_ms == 0U) return 0U;
    return (uint32_t)(((uint64_t)count * 1000ULL) / dt_ms);
}

static uint32_t Encoder_Rpm(uint32_t count, uint32_t dt_ms)
{
    if (dt_ms == 0U) return 0U;
    /* 11 PPR/channel * quadrature x4 = 44 count/revolution. */
    return (uint32_t)(((uint64_t)count * 60000ULL) / ((uint64_t)ENCODER_X4_CPR * dt_ms));
}

static void Motor_CutOutput(void)
{
    /* May be called from PORTB / SysTick ISR: NO busy-wait or CAN send. */
    FTM0->CONTROLS[3].CnV = 0U;
    PORTD->PCR[1] = PORT_PCR_MUX(1U);
    PTD->PCOR = (1UL << 1U);
    PTE->PCOR = 3UL;
    if (dbg_motor_dir != CMD_STOP)
    {
        dbg_last_motor_run_dir = dbg_motor_dir;
        dbg_last_motor_run_ms = (uint32_t)(dbg_ms - motion_start_ms);
        dbg_last_run_edges = dbg_encoder_edges - dbg_motor_start_edges;
        dbg_last_run_avg_pps = Encoder_CountsPerSecond(dbg_last_run_edges,
                                                       dbg_last_motor_run_ms);
        dbg_last_run_avg_rpm = Encoder_Rpm(dbg_last_run_edges,
                                          dbg_last_motor_run_ms);
        last_motor_dir = dbg_motor_dir;
        last_motor_off_ms = dbg_ms;
    }
    dbg_motor_pwm = 0U;
    dbg_motor_dir = CMD_STOP;
}

static void Motor_Init(void)
{
    PCC->PCCn[PCC_PORTE_INDEX] |= PCC_PCCn_CGC_MASK;
    PCC->PCCn[PCC_PORTD_INDEX] |= PCC_PCCn_CGC_MASK;
    PORTE->PCR[0] = PORT_PCR_MUX(1U);
    PORTE->PCR[1] = PORT_PCR_MUX(1U);
    PTE->PCOR = 3UL;
    PTE->PDDR |= 3UL;
    PORTD->PCR[1] = PORT_PCR_MUX(1U);
    PTD->PCOR = (1UL << 1U);
    PTD->PDDR |= (1UL << 1U);
    PCC->PCCn[PCC_FTM0_INDEX] &= ~PCC_PCCn_CGC_MASK;
    PCC->PCCn[PCC_FTM0_INDEX] = PCC_PCCn_PCS(1U) | PCC_PCCn_CGC_MASK;
    FTM0->SC = 0U;
    FTM0->MODE |= FTM_MODE_WPDIS_MASK;
    FTM0->COMBINE = 0U;
    FTM0->POL = 0U;
    FTM0->OUTMASK = 0U;
    FTM0->CNTIN = 0U;
    FTM0->CNT = 0U;
    FTM0->MOD = PWM_PERIOD;
    FTM0->CONTROLS[3].CnSC = FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK;
    FTM0->CONTROLS[3].CnV = 0U;
    FTM0->SC = FTM_SC_PWMEN3_MASK | FTM_SC_CLKS(3U) | FTM_SC_PS(0U);
    Motor_CutOutput();
}

static void LEDs_Init(void)
{
    PTD->PSOR = LED_GREEN | LED_RED | LED_BLUE;
    PORTD->PCR[0] = PORT_PCR_MUX(1U);
    PORTD->PCR[15] = PORT_PCR_MUX(1U);
    PORTD->PCR[16] = PORT_PCR_MUX(1U);
    PTD->PDDR |= LED_GREEN | LED_RED | LED_BLUE;
    PTD->PSOR = LED_GREEN | LED_RED | LED_BLUE;
}

static void LEDs_Poll(uint32_t now)
{
    uint32_t led_off = 0U, led_on = 0U;
    if (dbg_valid_cmd != 0U && (uint32_t)(now - last_valid_cmd_ms) < 250U)
        led_on |= LED_GREEN;
    else led_off |= LED_GREEN;
    if ((dbg_fault & FAULT_BLOCKING_MASK) != 0U) led_on |= LED_RED;
    else led_off |= LED_RED;
    if (dbg_limit_level == 0U) led_on |= LED_BLUE;
    else led_off |= LED_BLUE;
    PTD->PSOR = led_off;
    PTD->PCOR = led_on;
}

static void Switch_Init(void)
{
    /* PTD13 verified by standalone switch test, NOT onboard BTN1/PTC13. */
    PCC->PCCn[PCC_PORTD_INDEX] |= PCC_PCCn_CGC_MASK;
    PTD->PDDR &= ~CLOSED_MASK;
    PORTD->PCR[CLOSED_PIN] = PORT_PCR_MUX(1U) |
                             PORT_PCR_PE_MASK | PORT_PCR_PS_MASK |
                             PORT_PCR_IRQC(10U); /* falling-edge IRQ */
    PORTD->ISFR = CLOSED_MASK;
    S32_NVIC->IP[(uint32_t)PORTD_IRQn] = 0x40U;
    S32_NVIC->ICPR[(uint32_t)PORTD_IRQn / 32U] =
        1UL << ((uint32_t)PORTD_IRQn % 32U);
    S32_NVIC->ISER[(uint32_t)PORTD_IRQn / 32U] =
        1UL << ((uint32_t)PORTD_IRQn % 32U);
    closed_db_last = ((PTD->PDIR & CLOSED_MASK) == 0U) ? 1U : 0U;
    limit_changed_ms = 0UL;
}

static uint8_t Switch_Pressed(void)
{
    return ((PTD->PDIR & CLOSED_MASK) == 0U) ? 1U : 0U;
}

/* Read the actual PTB3/PTB2 logic levels (A in bit 1, B in bit 0). */
static uint8_t Encoder_ReadAB(void)
{
    uint32_t pins = PTB->PDIR;
    return (uint8_t)((((pins >> ENC_A_PIN) & 1UL) << 1U) |
                      ((pins >> ENC_B_PIN) & 1UL));
}

static void Encoder_Init(void)
{
    PCC->PCCn[PCC_PORTB_INDEX] |= PCC_PCCn_CGC_MASK;
    PTB->PDDR &= ~ENC_MASK;

    /* Use GPIO+PORTB interrupts, not the previously noncounting FTM1 input. */
    PORTB->PCR[ENC_A_PIN] = PORT_PCR_MUX(1U) |
        PORT_PCR_PE_MASK | PORT_PCR_PS_MASK | PORT_PCR_IRQC(11U);
    PORTB->PCR[ENC_B_PIN] = PORT_PCR_MUX(1U) |
        PORT_PCR_PE_MASK | PORT_PCR_PS_MASK | PORT_PCR_IRQC(11U);

    encoder_prev_ab = Encoder_ReadAB();
    dbg_encoder_ab_state = encoder_prev_ab;
    dbg_encoder_pin_levels = encoder_prev_ab;
    PORTB->ISFR = ENC_MASK; /* W1C: discard flags from pin configuration. */

    /* PORTD limit switch remains higher priority (0x40). */
    S32_NVIC->IP[(uint32_t)PORTB_IRQn] = 0x60U;
    S32_NVIC->ICPR[(uint32_t)PORTB_IRQn / 32U] =
        1UL << ((uint32_t)PORTB_IRQn % 32U);
    S32_NVIC->ISER[(uint32_t)PORTB_IRQn / 32U] =
        1UL << ((uint32_t)PORTB_IRQn % 32U);

    dbg_encoder_pcr2 = PORTB->PCR[ENC_B_PIN];
    dbg_encoder_pcr3 = PORTB->PCR[ENC_A_PIN];
}

/* Called on BOTH rising and falling edges of BOTH encoder phases. */
void PORTB_IRQHandler(void)
{
    static const int8_t transition[16] = {
         0, -1,  1,  0,
         1,  0,  0, -1,
        -1,  0,  0,  1,
         0,  1, -1,  0
    };
    uint32_t pending = PORTB->ISFR & ENC_MASK;
    uint8_t prev, curr, changed;
    int8_t step;

    if (pending == 0UL) return;
    PORTB->ISFR = pending; /* Clear only flagged encoder pins (W1C). */
    ++dbg_encoder_irq_count;
    if ((pending & (1UL << ENC_A_PIN)) != 0UL) ++dbg_encoder_irq_a_flags;
    if ((pending & (1UL << ENC_B_PIN)) != 0UL) ++dbg_encoder_irq_b_flags;

    curr = Encoder_ReadAB();
    prev = encoder_prev_ab;
    encoder_prev_ab = curr;
    dbg_encoder_ab_state = curr;
    changed = (uint8_t)(prev ^ curr);

    if ((changed & 2U) != 0U) ++dbg_encoder_a_edges;
    if ((changed & 1U) != 0U) ++dbg_encoder_b_edges;
    dbg_encoder_raw_edges += (uint32_t)((changed & 1U) + ((changed >> 1U) & 1U));

    if (changed == 0U) return;
    if (changed == 3U)
    {
        ++dbg_encoder_invalid; /* Two phases changed between observations. */
        return;
    }

    step = transition[((uint32_t)prev << 2U) | curr];
    if (step != 0)
    {
        dbg_encoder_count += (int32_t)step * encoder_open_sign;
        ++dbg_encoder_edges; /* Verified one-step quadrature transition. */
    }
}

static void Encoder_1ms(uint32_t now)
{
    /* Raw A/B interrupts update counters; 1ms task computes speed. */
    dbg_encoder_pin_levels = Encoder_ReadAB();
    if ((uint32_t)(now - last_speed_ms) >= 100U)
    {
        uint32_t dt_ms = now - last_speed_ms;
        uint32_t count = dbg_encoder_edges - speed_start_edges;
        dbg_encoder_speed_100ms = (count > 65535U) ? 65535U : (uint16_t)count;
        dbg_encoder_pps = Encoder_CountsPerSecond(count, dt_ms);
        dbg_encoder_rpm = Encoder_Rpm(count, dt_ms);
        if (dbg_motor_dir != CMD_STOP && count != 0U)
        {
            dbg_last_nonzero_pps = dbg_encoder_pps;
            dbg_last_nonzero_rpm = dbg_encoder_rpm;
        }
        speed_start_edges = dbg_encoder_edges;
        last_speed_ms = now;
    }

    if (dbg_motor_dir != CMD_STOP)
    {
        dbg_encoder_run_edges = dbg_encoder_edges - dbg_motor_start_edges;
        dbg_encoder_run_ms = (uint32_t)(now - motion_start_ms);
        /* Update run-average every 100 ms; retain the last result after STOP. */
        if ((dbg_encoder_run_ms % 100U) == 0U && dbg_encoder_run_ms != 0U)
        {
            dbg_encoder_run_pps = Encoder_CountsPerSecond(dbg_encoder_run_edges,
                                                            dbg_encoder_run_ms);
            dbg_encoder_run_rpm = Encoder_Rpm(dbg_encoder_run_edges,
                                              dbg_encoder_run_ms);
        }
    }
}

static void CutAndFault(uint8_t fault)
{
    /* Capture the FIRST cause while output is still commanded on. */
    dbg_last_fault_cause = fault;
    dbg_last_fault_motor_dir = dbg_motor_dir;
    dbg_last_fault_run_ms = (uint32_t)(dbg_ms - motion_start_ms);
    dbg_last_fault_progress = dbg_encoder_count - dbg_motor_start_encoder;
    dbg_last_fault_edges = dbg_encoder_edges - dbg_motor_start_edges;
    Motor_CutOutput();
    dbg_state = DOOR_STOPPED;
    dbg_fault |= fault;
    status_due = 1U;
    pending_cmd = CMD_STOP;
    stall_event = 0U;
    dbg_recovery = 0U;
    calibration_jog = 0U;
    first_timed_open = 0U;
}

/* Obstacle (bit 0) is a transient event until a real endpoint is reached. */
static void ClearObstacleAtEndpoint(uint8_t end_state)
{
    if ((dbg_fault & FAULT_OBSTACLE) != 0U)
    {
        dbg_fault &= (uint8_t)~FAULT_OBSTACLE;
        ++dbg_obstacle_clear_count;
        dbg_obstacle_clear_at_state = end_state;
    }
}

/* Transition to CLOSED is validated by stable limit, not encoder alone. */
static void SetClosed(void)
{
    Motor_CutOutput();
    dbg_encoder_count = 0L;
    /* Do not zero cumulative raw/valid edge diagnostics at CLOSED. */
    dbg_homed = 1U;
    dbg_fault &= (uint8_t)~FAULT_UNHOMED;
    if (dbg_recovery == 2U) ++dbg_recovery_close_completed_count;
    ClearObstacleAtEndpoint(DOOR_CLOSED);
    dbg_state = DOOR_CLOSED;
    ++dbg_closed_completed_count;
    dbg_recovery = 0U;
    pending_cmd = CMD_STOP;
    stall_event = 0U;
    calibration_jog = 0U;
    first_timed_open = 0U;
    status_due = 1U;
}

/* OPENED requires reaching the encoder target; a timer alone cannot prove it. */
static void SetOpened(void)
{
    Motor_CutOutput();
    if (dbg_recovery == 2U) ++dbg_recovery_open_completed_count;
    dbg_state = DOOR_OPENED;
    ClearObstacleAtEndpoint(DOOR_OPENED);
    dbg_recovery = 0U;
    pending_cmd = CMD_STOP;
    stall_event = 0U;
    stall_from_recovery = 0U;
    calibration_jog = 0U;
    first_timed_open = 0U;
    ++dbg_open_completed_count;
    status_due = 1U;
}

void PORTD_IRQHandler(void)
{
    if ((PORTD->ISFR & CLOSED_MASK) != 0U)
    {
        PORTD->ISFR = CLOSED_MASK;
        ++dbg_limit_irq_count;
        if (Switch_Pressed() != 0U && dbg_motor_dir == CMD_CLOSE)
        {
            Motor_CutOutput();
            limit_trip_pending = 1U;
            dbg_state = DOOR_STOPPED;
            status_due = 1U;
        }
    }
}

void SysTick_Handler(void)
{
    uint32_t now = ++dbg_ms;
    uint8_t low = Switch_Pressed();
    dbg_limit_level = (low != 0U) ? 0U : 1U;
    Encoder_1ms(now);
    dbg_ftm0_compare_now = (uint16_t)FTM0->CONTROLS[3].CnV;

    /* Backup 1 ms path for CLOSED limit if PORTD interrupt misses. */
    if (dbg_motor_dir == CMD_CLOSE && low != 0U)
    {
        Motor_CutOutput();
        limit_trip_pending = 1U;
        ++dbg_limit_tick_stops;
        dbg_state = DOOR_STOPPED;
        status_due = 1U;
    }
    if (low != closed_db_last)
    {
        closed_db_last = low;
        limit_changed_ms = now;
    }
    if ((uint32_t)(now - limit_changed_ms) >= CLOSED_DEBOUNCE_MS)
    {
        dbg_limit_stable = low;
        if (low != 0U && dbg_motor_dir != CMD_OPEN &&
            (dbg_state == DOOR_UNKNOWN || dbg_state == DOOR_CLOSED ||
             limit_trip_pending != 0U || dbg_state == DOOR_STOPPED ||
             dbg_state == DOOR_CLOSING))
        {
            /* Do NOT home on a bounce while opening. */
            if (dbg_state != DOOR_CLOSED || dbg_homed == 0U)
                SetClosed();
            limit_trip_pending = 0U;
        }
    }

    if (dbg_motor_dir == CMD_STOP) return;

#if REQUIRE_HEARTBEAT
    if (dbg_hb_count == 0U ||
        (uint32_t)(now - dbg_last_hb_ms) > HEARTBEAT_TIMEOUT_MS)
    {
        CutAndFault(FAULT_CAN_LOST);
        ++dbg_can_timeout_count;
        return;
    }
#endif
    if ((uint32_t)(now - motion_start_ms) >= MAX_TRAVEL_MS)
    {
        CutAndFault(FAULT_TRAVEL);
        return;
    }
    if (calibration_jog != 0U && (uint32_t)(now - motion_start_ms) >= JOG_PULSE_MS)
    {
        Motor_CutOutput();
        dbg_state = DOOR_STOPPED;
        calibration_jog = 0U;
        status_due = 1U;
        return;
    }
    /* OPENED = reached a positive encoder target. Never use timeout as
       position confirmation. A missing/unreliable target is a stop fault. */
    if (dbg_motor_dir == CMD_OPEN && CALIBRATION_MODE == 0U)
    {
        int32_t target = (DOOR_TRAVEL_COUNTS > 0L) ? DOOR_TRAVEL_COUNTS :
                         (dbg_open_target_valid != 0U ? dbg_open_target_count : 0L);
        if (target <= 0L)
        {
            CutAndFault(FAULT_TRAVEL);
            return;
        }
        if (dbg_encoder_count >= target)
        {
            /* Reaching the encoder target must also release CLOSED switch. */
            if (low != 0U)
            {
                CutAndFault(FAULT_LIMIT);
                return;
            }
            SetOpened();
            return;
        }
    }
    /* OPEN has its own independent bound, regardless of learned target,
       reversal mode, or first-open flag. Do not claim OPENED on timeout. */
    if (dbg_motor_dir == CMD_OPEN && calibration_jog == 0U &&
        (uint32_t)(now - motion_start_ms) >= OPEN_MAX_RUN_MS)
    {
        ++dbg_open_time_limit_count;
        CutAndFault(FAULT_TRAVEL);
        return;
    }
    if (dbg_motor_dir == CMD_OPEN && low != 0U &&
        (uint32_t)(now - motion_start_ms) >= OPEN_LIMIT_RELEASE_MS)
    {
        CutAndFault(FAULT_LIMIT);
        return;
    }

    /* Compare the accumulated A/B edges with the timestamp.
       The signed position can cancel out even while encoder edges are valid. */
    {
        uint32_t edges = dbg_encoder_edges;
        uint32_t run_edges = edges - dbg_motor_start_edges;
        dbg_start_progress = (run_edges > 0x7FFFFFFFUL) ?
                             0x7FFFFFFFL : (int32_t)run_edges;

        if (startup_motion_confirmed == 0U && calibration_jog == 0U)
        {
            if (run_edges >= (uint32_t)MIN_WINDOW_PROGRESS)
            {
                /* Quadrature A/B wiring can reverse the sign. Learn OPEN's
                   physical-positive direction ONCE, from verified CLOSED=0.
                   Keep using signed position to detect direction afterwards. */
                if (dbg_motor_dir == CMD_OPEN && dbg_homed != 0U &&
                    dbg_encoder_sign_learned == 0U &&
                    dbg_motor_start_encoder == 0L)
                {
                    int32_t signed_delta = dbg_encoder_count - dbg_motor_start_encoder;
                    if (signed_delta >= (int32_t)MIN_WINDOW_PROGRESS ||
                        signed_delta <= -(int32_t)MIN_WINDOW_PROGRESS)
                    {
                        if (signed_delta < 0L)
                        {
                            encoder_open_sign = -encoder_open_sign;
                            dbg_encoder_count = -dbg_encoder_count;
                        }
                        dbg_encoder_open_sign = encoder_open_sign;
                        dbg_encoder_sign_learned = 1U;
                    }
                }
                /* If many transitions cancel direction, do not claim a valid
                   position yet; keep sampling until signed movement appears. */
                if (dbg_motor_dir == CMD_OPEN &&
                    dbg_motor_start_encoder == 0L &&
                    dbg_encoder_sign_learned == 0U)
                {
                    if ((uint32_t)(now - motion_start_ms) >= START_MOTION_VERIFY_MS)
                    {
                        ++dbg_start_no_motion_count;
                        CutAndFault(FAULT_ENCODER);
                    }
                    return;
                }
                startup_motion_confirmed = 1U;
                dbg_start_motion_confirmed = 1U;
                last_observed_edges = edges;
                last_valid_raw_edges = dbg_encoder_raw_edges;
                last_edge_ms = now;
                window_start_edges = edges;
                last_window_ms = now;
            }
            else if ((uint32_t)(now - motion_start_ms) >=
                     START_MOTION_VERIFY_MS)
            {
                ++dbg_start_no_motion_count;
                CutAndFault(FAULT_ENCODER); /* No encoder feedback. */
            }
            return;
        }

        /* A new accepted encoder edge means the shaft is still moving. */
        if (edges != last_observed_edges)
        {
            last_observed_edges = edges;
            last_valid_raw_edges = dbg_encoder_raw_edges;
            last_edge_ms = now;
        }

        if ((uint32_t)(now - last_window_ms) >= STALL_WINDOW_MS)
        {
            uint32_t delta = edges - window_start_edges;
            dbg_last_window_progress = (delta > 0x7FFFFFFFUL) ?
                                       0x7FFFFFFFL : (int32_t)delta;
            dbg_motion_progress = (delta != 0U) ? 1U : 0U;
            window_start_edges = edges;
            last_window_ms = now;
        }

        /* The no-edge timeout uses RECENT movement, not the run-average RPM.
           A single empty 100ms speed sample never causes an obstacle alarm. */
        if (calibration_jog == 0U &&
            (uint32_t)(now - motion_start_ms) >= STALL_GRACE_MS &&
            (uint32_t)(now - last_edge_ms) >= STALL_NO_EDGE_MS)
        {
            /* Raw edges without valid steps indicate decoder/signal trouble,
               not proven mechanical obstruction. Stop without reversing. */
            if (dbg_encoder_raw_edges != last_valid_raw_edges)
            {
                CutAndFault(FAULT_ENCODER);
                return;
            }
            reverse_previous = dbg_motor_dir;
            Motor_CutOutput();
            dbg_state = DOOR_STOPPED;
            stall_event = 1U;
            stall_from_recovery = (dbg_recovery == 2U) ? 1U : 0U;
            status_due = 1U;
            ++dbg_stall_count;
        }
    }
}

static uint8_t Tick_Init1ms(void)
{
    uint32_t hz = CoreClock_Hz();
    uint32_t reload = hz / 1000UL;
    if (hz == 0U || reload == 0U || reload > 0x1000000UL) return 0U;
    dbg_core_hz = hz;
    S32_SysTick->CSR = 0U;
    S32_SysTick->RVR = reload - 1U;
    S32_SysTick->CVR = 0U;
    S32_SysTick->CSR = S32_SysTick_CSR_CLKSOURCE_MASK |
                       S32_SysTick_CSR_TICKINT_MASK |
                       S32_SysTick_CSR_ENABLE_MASK;
    return 1U;
}

static uint8_t LinkGood(uint32_t now)
{
#if REQUIRE_HEARTBEAT
    return (dbg_hb_count != 0U &&
            (uint32_t)(now - dbg_last_hb_ms) <= HEARTBEAT_TIMEOUT_MS) ? 1U : 0U;
#else
    (void)now;
    return 1U;
#endif
}

static void Motor_Run(uint8_t cmd, uint8_t percent, uint32_t now, uint8_t recovery)
{
    uint32_t compare;
    /* Limit and heartbeat must both be verified at the point of actuation. */
    if (LinkGood(now) == 0U) { CutAndFault(FAULT_CAN_LOST); return; }
    if (cmd == CMD_CLOSE && Switch_Pressed() != 0U)
    {
        SetClosed();
        return;
    }
    /* Normal motion and safety reversal: fixed 65% PWM.
       Keep JOG duty separately selectable for mechanical calibration. */
    if (calibration_jog == 0U)
        percent = MOTOR_NORMAL_PWM;
    else
    {
        if (percent > PWM_MAX) percent = PWM_MAX;
        if (percent < PWM_MIN_MOVING) percent = PWM_MIN_MOVING;
    }
    Motor_CutOutput();
    if (cmd == CMD_OPEN)
    {
        PTE->PCOR = 2UL;
        PTE->PSOR = 1UL;
        dbg_state = DOOR_OPENING;
    }
    else
    {
        PTE->PCOR = 1UL;
        PTE->PSOR = 2UL;
        dbg_state = DOOR_CLOSING;
    }
    first_timed_open = 0U; /* OPEN uses target count; timer is only a bound. */
    motion_start_ms = now;
    startup_motion_confirmed = 0U;
    dbg_start_motion_confirmed = 0U;
    dbg_start_progress = 0L;
    dbg_motor_start_encoder = dbg_encoder_count;
    dbg_motor_start_edges = dbg_encoder_edges;
    dbg_encoder_run_edges = 0U;
    dbg_encoder_run_ms = 0U;
    dbg_encoder_run_pps = 0U;
    dbg_encoder_run_rpm = 0U;
    speed_start_edges = dbg_encoder_edges;
    last_speed_ms = now;
    last_observed_edges = dbg_encoder_edges;
    last_valid_raw_edges = dbg_encoder_raw_edges;
    last_edge_ms = now;
    last_window_ms = now;
    window_start_edges = dbg_encoder_edges;
    dbg_last_window_progress = 0L;
    dbg_motion_progress = 0U;
    compare = ((uint32_t)percent * (PWM_PERIOD + 1U)) / 100U;
    if (compare > PWM_PERIOD) compare = PWM_PERIOD;
    dbg_motor_dir = cmd;
    dbg_motor_pwm = percent;
    dbg_recovery = recovery;
    FTM0->CONTROLS[3].CnV = compare;
    PORTD->PCR[1] = PORT_PCR_MUX(2U);
    ++dbg_motor_start_count;
    dbg_motor_start_ms = now;
    dbg_motor_start_direction = cmd;
    dbg_motor_applied_compare = (uint16_t)compare;
    status_due = 1U;
}

static void ScheduleStart(uint8_t cmd, uint8_t duty, uint32_t now, uint8_t recovery)
{
    /* Always enforce a motor-off deadtime before changing direction. */
    Motor_CutOutput();
    dbg_state = DOOR_STOPPED;
    pending_cmd = cmd;
    pending_pwm = duty;
    pending_recovery = recovery;
    pending_at_ms = now;
    dbg_recovery = (recovery != 0U) ? 1U : 0U;
    status_due = 1U;
}

static void OnStall(uint32_t now)
{
    uint8_t old = reverse_previous;
    uint8_t was_recovery = stall_from_recovery;
    stall_event = 0U;
    /* A jam during OPEN may reverse to the known CLOSED switch even before
       the first OPEN endpoint was learned. A jam during CLOSE needs a known
       encoder target for reversing OPEN; otherwise stop and latch fault. */
    if (was_recovery != 0U || dbg_homed == 0U ||
        (old == CMD_CLOSE && DOOR_TRAVEL_COUNTS <= 0L &&
         dbg_open_target_valid == 0U) ||
        CALIBRATION_MODE != 0U)
    {
        CutAndFault(FAULT_ENCODER | FAULT_OBSTACLE);
        return;
    }
    dbg_fault |= FAULT_OBSTACLE;
    if (old == CMD_CLOSE) ScheduleStart(CMD_OPEN, MOTOR_NORMAL_PWM, now, 1U);
    else if (old == CMD_OPEN) ScheduleStart(CMD_CLOSE, MOTOR_NORMAL_PWM, now, 1U);
    else CutAndFault(FAULT_ENCODER);
}

static void Commands_Apply(uint8_t cmd, uint8_t duty, uint32_t now);

/* ========================================================================
 *  APP <> CAN DRIVER boundary. No direct FlexCAN register writes in app.
 *  The six mentor APIs are in Can.c; auxiliary bus-off/mode/diagnostic
 *  services are retained to support ver3 automatic recovery safely.
 * ======================================================================== */
static uint8_t App_TrySendCan(uint16_t id, uint8_t dlc,
                              uint32_t data0, uint32_t data1,
                              uint8_t kind, uint32_t now)
{
    uint8_t buffer[8];
    uint8_t i;
    Can_PduType pdu;
    Std_ReturnType ret;
    for (i = 0U; i < 8U; ++i) {
        uint32_t w = (i < 4U) ? data0 : data1;
        buffer[i] = (uint8_t)(w >> (24U - 8U * (uint32_t)(i & 3U)));
    }
    pdu.swPduHandle = (PduIdType)kind;
    pdu.id = (Can_IdType)id;
    pdu.length = dlc;
    pdu.sdu = buffer;
    ret = Can_Write(CAN_HTH_TX, &pdu);
    if (ret != E_OK) return 0U;
    tx_kind = kind;
    tx_busy = 1U;
    last_tx_started_ms = now;
    return 1U;
}

static void App_CanTxService(uint32_t now)
{
    uint32_t w0, w1;
    if (can_recovery_pending != 0U || LinkGood(now) == 0U) return;
    if (tx_busy != 0U) {
        if ((uint32_t)(now - last_tx_started_ms) >= CAN_TX_GUARD_MS) {
            if (tx_reserved == 0U) ++dbg_tx_stuck_count;
            tx_reserved = 1U; /* Do not overwrite unconfirmed frame */
        }
        return;
    }
    if (tx_reserved != 0U) return;
    /* After physical CAN recovery, the bus-off fault remains latched until
     * CLEAR and UNLOCK, but status TX is permitted so remote can see the fault. */
    if (status_due != 0U || (uint32_t)(now-last_status_ms) >= STATUS_PERIOD_MS) {
        uint8_t flags = 0U;
        if (dbg_limit_stable != 0U) flags |= FLAG_CLOSED_SWITCH;
        if (dbg_homed != 0U) flags |= FLAG_HOMED;
        if (dbg_recovery != 0U) flags |= FLAG_REVERSING;
        if (LinkGood(now) != 0U) flags |= FLAG_HEARTBEAT_OK;
        if (CALIBRATION_MODE != 0U) flags |= FLAG_CALIBRATING;
        if (dbg_motion_progress != 0U) flags |= FLAG_ENCODER_PROGRESS;
        if (dbg_open_estimated != 0U) flags |= FLAG_OPEN_ESTIMATED;
        w0 = ((uint32_t)dbg_state << 24U) | ((uint32_t)dbg_locked << 16U) |
             ((uint32_t)dbg_fault << 8U) | (uint32_t)dbg_motor_pwm;
        w1 = ((uint32_t)flags << 24U) | ((uint32_t)seq_no << 16U);
        if (App_TrySendCan(ID_STATE, 6U, w0, w1, 1U, now) != 0U) {
            last_status_ms = now;
            status_due = 0U;
            ++seq_no;
        }
    } else if ((uint32_t)(now-last_position_ms) >= POSITION_PERIOD_MS) {
        w0 = (uint32_t)dbg_encoder_count;
        w1 = ((uint32_t)dbg_encoder_speed_100ms << 16U) |
             ((uint32_t)dbg_motor_dir << 8U) | (uint32_t)dbg_limit_level;
        if (App_TrySendCan(ID_ENCODER, 8U, w0, w1, 2U, now) != 0U)
            last_position_ms = now;
    }
}

/* Driver-owned ESR/ECR access through diagnostic API. The app remains
 * responsible for deciding to cut motor and latch the security fault. */
static void App_CanSupervise(uint32_t now)
{
    uint32_t esr1, ecr;
    uint8_t bus_off, event;
    if (Can_GetRawDiagnostics(&esr1, &ecr) != E_OK) return;
    dbg_can_esr1_now = esr1;
    dbg_can_ecr_now = ecr;
    if ((esr1 & CAN_ESR1_ACKERR_MASK) != 0U) ++dbg_can_ackerr_seen;
    if ((esr1 & (CAN_ESR1_BIT0ERR_MASK | CAN_ESR1_BIT1ERR_MASK)) != 0U)
        ++dbg_can_biterr_seen;
    if ((esr1 & (CAN_ESR1_FRMERR_MASK | CAN_ESR1_STFERR_MASK |
                 CAN_ESR1_CRCERR_MASK)) != 0U) ++dbg_can_frameerr_seen;
    bus_off = ((esr1 & CAN_ESR1_FLTCONF_MASK) ==
               CAN_ESR1_FLTCONF(2U)) ? 1U : 0U;
    event = ((esr1 & CAN_ESR1_BOFFINT_MASK) != 0U) ? 1U : 0U;
    if ((bus_off != 0U || event != 0U) && can_recovery_pending == 0U) {
        dbg_can_esr1_at_busoff = esr1;
        dbg_can_ecr_at_busoff = ecr;
        dbg_can_busoff_ms = now;
        dbg_can_rx_at_busoff = can_rx_count;
        dbg_can_tx_at_busoff = can_tx_count;
        ++dbg_can_busoff_count;
        CutAndFault(FAULT_BUS_OFF);
        dbg_locked = 1U;
        tx_reserved = 1U;
        can_recovery_pending = 1U;
        dbg_can_recovery_pending = 1U;
        can_recovery_next_ms = now + CAN_RECOVER_RETRY_MS;
    }
}

/* Recovery states: 0 = no recovery, 1 = wait STOPPED then DeInit/Init,
 * 2 = await asynchronous STARTED confirmation. Safety fault stays latched. */
static uint8_t app_can_recovery_phase = 0U;
static uint32_t app_can_start_ms = 0U;
static void App_CanRecover(uint32_t now)
{
    Can_ControllerStateType mode = CAN_CS_UNINIT;
    if (can_recovery_pending == 0U) return;
    if (app_can_recovery_phase == 2U) {
        Can_MainFunction_Mode();
        if (Can_GetControllerMode(CAN_CTRL_0, &mode) == E_OK &&
            mode == CAN_CS_STARTED) {
            tx_busy = 0U;
            tx_reserved = 0U;
            tx_kind = 0U;
            dbg_hb_count = 0U; /* Require a fresh heartbeat post-recovery. */
            dbg_last_hb_ms = 0U;
            status_due = 1U;
            last_status_ms = now;
            last_position_ms = now;
            can_recovery_pending = 0U;
            dbg_can_recovery_pending = 0U;
            app_can_recovery_phase = 0U;
            ++dbg_can_reinit_success;
        } else if ((uint32_t)(now - app_can_start_ms) > CAN_RECOVER_RETRY_MS) {
            /* A stuck mode transition cannot safely be force-deinitialized.
             * Motor remains OFF; a board reset is required if HW never ACKs. */
            ++dbg_can_reinit_failed;
            app_can_recovery_phase = 3U;
        }
        return;
    }
    if (app_can_recovery_phase == 3U ||
        (int32_t)(now - can_recovery_next_ms) < 0) return;

    /* Driver must own bus-off state change and TX cancellation. */
    if (Can_GetControllerMode(CAN_CTRL_0, &mode) != E_OK ||
        mode != CAN_CS_STOPPED) {
        /* Still waiting for Can_MainFunction_BusOff to acknowledge freeze. */
        can_recovery_next_ms = now + CAN_RECOVER_RETRY_MS;
        ++dbg_can_reinit_failed;
        return;
    }
    ++dbg_can_reinit_attempts;
    Can_DeInit();
    if (can_last_status != E_OK || can_deinit_failure != 0U) {
        ++dbg_can_reinit_failed;
        app_can_recovery_phase = 3U;
        return;
    }
    if (Board_CanMcuClockInit() != E_OK) {
        ++dbg_can_reinit_failed;
        app_can_recovery_phase = 3U;
        return;
    }
    Board_CanPortInit();
    Can_Init(&Can_Config);
    if (can_last_status != E_OK ||
        Can_GetControllerMode(CAN_CTRL_0, &mode) != E_OK ||
        mode != CAN_CS_STOPPED ||
        Can_SetControllerMode(CAN_CTRL_0, CAN_CS_STARTED) != E_OK) {
        ++dbg_can_reinit_failed;
        app_can_recovery_phase = 3U;
        return;
    }
    app_can_recovery_phase = 2U;
    app_can_start_ms = now;
}

void CanIf_RxIndication(const Can_HwType *Mailbox,
                        const PduInfoType *PduInfoPtr)
{
    uint32_t now = dbg_ms;
    const uint8_t *data;
    if (can_recovery_pending != 0U || Mailbox == NULL_PTR ||
        PduInfoPtr == NULL_PTR || PduInfoPtr->SduDataPtr == NULL_PTR) return;
    data = PduInfoPtr->SduDataPtr;
    if (Mailbox->Hoh == CAN_HRH_HEARTBEAT && Mailbox->CanId == ID_HEARTBEAT &&
        PduInfoPtr->SduLength == 1U && data[0] == 0xA5U) {
        dbg_last_hb_ms = now;
        ++dbg_hb_count;
    } else if (Mailbox->Hoh == CAN_HRH_COMMAND && Mailbox->CanId == ID_CMD &&
               PduInfoPtr->SduLength == 2U &&
               data[0] <= CMD_JOG_CLOSE && data[1] <= 100U) {
        ++dbg_valid_cmd;
        dbg_last_cmd_word = ((uint32_t)data[0] << 8U) | data[1];
        last_valid_cmd_ms = now;
        Commands_Apply(data[0], data[1], now);
    }
}
void CanIf_TxConfirmation(PduIdType CanTxPduId)
{
    if (tx_busy != 0U && tx_kind == (uint8_t)CanTxPduId) {
        if (CanTxPduId == 1U) ++dbg_status_tx_count;
        tx_busy = 0U;
        tx_reserved = 0U;
    }
}
void CanIf_ControllerModeIndication(uint8 Controller,
                                    Can_ControllerStateType ControllerMode)
{
    (void)Controller;
    (void)ControllerMode;
}
void CanIf_ControllerBusOff(uint8 Controller)
{
    (void)Controller;
    /* Bus-off hardware was confirmed STOPPED and TX cancelled in CAN Driver. */
    tx_busy = 0U;
    tx_reserved = 1U;
    if ((dbg_fault & FAULT_BUS_OFF) == 0U) CutAndFault(FAULT_BUS_OFF);
    dbg_locked = 1U;
    if (can_recovery_pending == 0U) {
        can_recovery_pending = 1U;
        dbg_can_recovery_pending = 1U;
        can_recovery_next_ms = dbg_ms + CAN_RECOVER_RETRY_MS;
    }
}


static void Commands_Apply(uint8_t cmd, uint8_t duty, uint32_t now)
{
    dbg_last_reject = 0U;
    if (cmd == CMD_STOP)
    {
        uint8_t was_moving = (dbg_motor_dir != CMD_STOP) ? 1U : 0U;
        Motor_CutOutput();
        dbg_recovery = 0U;
        pending_cmd = CMD_STOP;
        stall_event = 0U;
        stall_from_recovery = 0U;
        calibration_jog = 0U;
        first_timed_open = 0U;
        if (was_moving != 0U ||
            (dbg_state != DOOR_CLOSED && dbg_state != DOOR_OPENED))
            dbg_state = DOOR_STOPPED;
        status_due = 1U;
        return;
    }
    if (cmd == CMD_LOCK)
    {
        /* Preserve an already confirmed CLOSED/OPENED position. The previous
         * firmware forced STOPPED even when locking a stationary closed door,
         * which prevented the first normal OPEN after UNLOCK.
         */
        uint8_t interrupted = (dbg_motor_dir != CMD_STOP ||
                               pending_cmd != CMD_STOP) ? 1U : 0U;
        Motor_CutOutput();
        dbg_recovery = 0U;
        pending_cmd = CMD_STOP;
        stall_event = 0U;
        stall_from_recovery = 0U;
        calibration_jog = 0U;
        first_timed_open = 0U;
        dbg_locked = 1U;
        if (interrupted != 0U && dbg_state != DOOR_CLOSED &&
            dbg_state != DOOR_OPENED)
            dbg_state = DOOR_STOPPED;
        status_due = 1U;
        return;
    }
    if (cmd == CMD_UNLOCK)
    {
        /* Bus-Off must first be recovered and acknowledged by CLEAR. */
        if ((dbg_fault & FAULT_BUS_OFF) != 0U || can_recovery_pending != 0U)
        {
            dbg_last_reject = 11U;
            return;
        }
        dbg_locked = 0U;
        status_due = 1U;
        return;
    }
    if (cmd == CMD_CLEAR)
    {
        /* Never clear Bus-Off merely because hardware re-entered error-active.
           Require a fresh valid heartbeat after peripheral reinitialization. */
        if (dbg_motor_dir != CMD_STOP) { dbg_last_reject = 1U; return; }
        if (can_recovery_pending != 0U || can_last_status != E_OK ||
            (dbg_can_esr1_now & CAN_ESR1_FLTCONF_MASK) == CAN_ESR1_FLTCONF(2U) ||
            LinkGood(now) == 0U)
        {
            dbg_last_reject = 12U;
            return;
        }
        dbg_fault &= (uint8_t)(FAULT_UNHOMED);
        status_due = 1U;
        return;
    }
    if (dbg_locked != 0U) { dbg_last_reject = 2U; return; }
    if ((dbg_fault & FAULT_BLOCKING_MASK) != 0U)
    {
        dbg_last_reject = 3U;
        return;
    }
    if (LinkGood(now) == 0U) { dbg_last_reject = 4U; return; }
    if (dbg_recovery != 0U) { dbg_last_reject = 5U; return; }

    if (CALIBRATION_MODE != 0U)
    {
        if (cmd != CMD_JOG_OPEN && cmd != CMD_JOG_CLOSE)
        {
            dbg_last_reject = 6U;
            return;
        }
        if (dbg_motor_dir != CMD_STOP || pending_cmd != CMD_STOP)
        {
            dbg_last_reject = 7U;
            return;
        }
        calibration_jog = 1U;
        /* Position check disabled in JOG, but switch and 180ms deadline remain. */
        Motor_Run((cmd == CMD_JOG_OPEN) ? CMD_OPEN : CMD_CLOSE,
                  (duty > JOG_PWM_MAX) ? JOG_PWM_MAX : duty, now, 0U);
        return;
    }

    if (cmd != CMD_OPEN && cmd != CMD_CLOSE) { dbg_last_reject = 8U; return; }
    /* If the physical CLOSED switch is confirmed while stationary, restore
     * CLOSED even if a previous STOP/LOCK caused a STOPPED state.
     * Do not change position in response to an un-debounced signal.
     */
    if (cmd == CMD_OPEN && dbg_motor_dir == CMD_STOP &&
        pending_cmd == CMD_STOP && dbg_limit_stable != 0U &&
        Switch_Pressed() != 0U && dbg_state != DOOR_CLOSED)
        SetClosed();
    /* First uncalibrated OPEN is permitted only from a known CLOSED state.
       Encoder position for a partly open door after STOP/reset is uncertain. */
    if (cmd == CMD_OPEN && DOOR_TRAVEL_COUNTS <= 0L &&
        dbg_open_target_valid == 0U && dbg_state != DOOR_CLOSED)
    {
        dbg_last_reject = 9U;
        return;
    }
    if (cmd == CMD_OPEN && dbg_homed == 0U)
    {
        dbg_fault |= FAULT_UNHOMED;
        dbg_last_reject = 10U;
        status_due = 1U;
        return;
    }
    if (cmd == CMD_CLOSE && Switch_Pressed() != 0U)
    {
        SetClosed();
        return;
    }
    if ((cmd == CMD_OPEN && dbg_state == DOOR_OPENED) ||
        (cmd == CMD_CLOSE && dbg_state == DOOR_CLOSED))
        return;
    if (dbg_motor_dir == cmd) return; /* Duplicates do not reset timer. */
    if (pending_cmd == cmd) return;   /* Duplicates do not reset deadtime. */
    if (dbg_motor_dir != CMD_STOP || pending_cmd != CMD_STOP ||
        (last_motor_dir != CMD_STOP && last_motor_dir != cmd &&
         (uint32_t)(now - last_motor_off_ms) < REVERSAL_WAIT_MS))
        ScheduleStart(cmd, duty, now, 0U);
    else
        Motor_Run(cmd, duty, now, 0U);
}

int main(void)
{
    uint32_t now, last_led_ms = 0U, timeout;
    Can_ControllerStateType mode = CAN_CS_UNINIT;
    WDOG_Off_BenchOnly();
    Motor_Init();
    LEDs_Init();
    Switch_Init();
    Encoder_Init();
    dbg_open_target_count = DOOR_TRAVEL_COUNTS;
    dbg_open_target_valid = (DOOR_TRAVEL_COUNTS > 0L) ? 1U : 0U;
    dbg_open_estimated = (OPEN_TARGET_CALIBRATED == 0U) ? 1U : 0U;
    if (Board_CanMcuClockInit() != E_OK) {
        CutAndFault(FAULT_BUS_OFF);
        for (;;) { }
    }
    Board_CanPortInit();
    Can_Init(&Can_Config);
    if (Can_GetControllerMode(CAN_CTRL_0, &mode) != E_OK ||
        mode != CAN_CS_STOPPED ||
        Can_SetControllerMode(CAN_CTRL_0, CAN_CS_STARTED) != E_OK) {
        CutAndFault(FAULT_BUS_OFF);
        for (;;) { }
    }
    timeout = CAN_HW_TIMEOUT;
    while (timeout != 0U) {
        Can_MainFunction_Mode();
        if (Can_GetControllerMode(CAN_CTRL_0, &mode) == E_OK &&
            mode == CAN_CS_STARTED) break;
        --timeout;
    }
    if (timeout == 0U) {
        CutAndFault(FAULT_BUS_OFF);
        for (;;) { }
    }
    if (Tick_Init1ms() == 0U) {
        CutAndFault(FAULT_TRAVEL);
        for (;;) { }
    }
    for (;;) {
        now = dbg_ms;
        /* Safety ISR handles motor cut immediately on limit/encoder timeout. */
        App_CanSupervise(now);
        Can_MainFunction_BusOff();
        Can_MainFunction_Mode();
        App_CanRecover(now);
        if (can_recovery_pending != 0U) {
            if ((uint32_t)(now-last_led_ms) >= 10U) {
                last_led_ms = now;
                LEDs_Poll(now);
            }
            continue; /* no RX action/TX while CAN recovery pending */
        }
        if (stall_event != 0U && (dbg_fault & FAULT_BLOCKING_MASK) == 0U)
            OnStall(now);
        Can_MainFunction_Read(); /* heartbeat then door command */
        Can_MainFunction_Write(); /* TX confirmation via CanIf callback */
        if (pending_cmd != CMD_STOP &&
            (uint32_t)(now-pending_at_ms) >= REVERSAL_WAIT_MS) {
            uint8_t cmd = pending_cmd, pwm = pending_pwm, rev = pending_recovery;
            pending_cmd = CMD_STOP;
            if ((dbg_fault & FAULT_BLOCKING_MASK) == 0U &&
                dbg_locked == 0U && LinkGood(now) != 0U)
                Motor_Run(cmd, pwm, now, (rev != 0U) ? 2U : 0U);
            else CutAndFault(FAULT_CAN_LOST);
        }
        if ((uint32_t)(now-last_led_ms) >= 10U) {
            last_led_ms = now;
            LEDs_Poll(now);
        }
        App_CanTxService(now);
    }
}

