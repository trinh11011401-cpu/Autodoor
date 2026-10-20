#include "app_config.h"
#include "app_protocol.h"
#include "board.h"
#include "encoder.h"
#include "door.h"

volatile uint8_t dbg_state = DOOR_UNKNOWN;
volatile uint8_t dbg_locked = 1U;
volatile uint8_t dbg_fault = FAULT_POWER_UP | FAULT_UNHOMED;
volatile uint8_t dbg_last_reject = 0U;
volatile uint8_t dbg_motor_dir = CMD_STOP;
volatile uint8_t dbg_motor_pwm = 0U;
volatile uint8_t dbg_homed = 0U;
volatile uint8_t dbg_obstacle_events = 0U;
volatile uint32_t dbg_last_run_ms = 0U;
volatile uint32_t dbg_last_run_counts = 0U;
volatile float dbg_last_run_average_rpm = 0.0f;
volatile float dbg_last_run_output_angle_deg = 0.0f;
volatile uint32_t dbg_last_running_pps = 0U;
volatile uint32_t dbg_last_fault_at_ms = 0U;
volatile uint8_t dbg_last_fault_cause = 0U;
volatile uint32_t dbg_stall_baseline_counts = 0U;
volatile uint32_t dbg_stall_last_counts = 0U;
volatile uint8_t dbg_stall_low_count = 0U;
volatile uint32_t dbg_last_hb_ms = 0U;
volatile uint32_t dbg_hb_count = 0U;
volatile uint8_t dbg_status_dirty = 1U;

static uint32_t motion_start_ms = 0U;
static uint32_t run_start_edges = 0U;
static int32_t run_start_position = 0L;
static uint8_t motion_confirmed = 0U;
static uint8_t recovery_active = 0U;
static uint8_t pending_direction = CMD_STOP;
static uint8_t pending_pwm = 0U;
static uint8_t pending_recovery = 0U;
static uint32_t pending_since_ms = 0U;
static uint32_t stall_sample_ms = 0U;
static int32_t stall_sample_position = 0L;
static uint32_t stall_sample_invalid = 0U;
static uint8_t switch_raw = 0U;
static uint8_t switch_stable = 0U;
static uint32_t switch_changed_ms = 0U;
static uint8_t switch_interrupted_close = 0U;
static uint8_t jog_active = 0U;
static uint8_t last_direction = CMD_STOP;
static uint32_t last_motor_off_ms = 0U;

static int32_t DirectionalProgress(int32_t newer, int32_t older, uint8_t dir)
{
    int32_t change = newer - older;
    return (dir == CMD_CLOSE) ? -change : change;
}

static void StopHardware(uint32_t now)
{
    if (dbg_motor_dir != CMD_STOP)
    {
        EncoderSnapshot e;
        uint32_t elapsed = now - motion_start_ms;
        Encoder_GetSnapshot(&e);
        dbg_last_run_ms = elapsed;
        dbg_last_run_counts = e.total - run_start_edges;
        dbg_last_run_average_rpm = (elapsed != 0U) ?
            ((float)dbg_last_run_counts * 60000.0f) /
            ((float)ENC_CPR * (float)elapsed) : 0.0f;
        dbg_last_run_output_angle_deg =
            (float)dbg_last_run_counts * 360.0f /
            ((float)ENC_CPR * (float)GEAR_RATIO);
        last_direction = dbg_motor_dir;
        last_motor_off_ms = now;
    }
    Board_MotorStop();
    motion_confirmed = 0U;
    dbg_motor_dir = CMD_STOP;
    dbg_motor_pwm = 0U;
}

static void StopAndHold(uint32_t now)
{
    StopHardware(now);
    pending_direction = CMD_STOP;
    pending_recovery = 0U;
    recovery_active = 0U;
    jog_active = 0U;
    if (dbg_state != DOOR_CLOSED && dbg_state != DOOR_OPENED)
        dbg_state = DOOR_STOPPED;
    dbg_status_dirty = 1U;
}

static void FaultStop(uint8_t cause, uint32_t now)
{
    dbg_last_fault_cause = cause;
    dbg_last_fault_at_ms = now;
    StopHardware(now);
    dbg_fault |= cause;
    dbg_state = DOOR_STOPPED;
    pending_direction = CMD_STOP;
    pending_recovery = 0U;
    recovery_active = 0U;
    jog_active = 0U;
    dbg_status_dirty = 1U;
}

static void SetClosed(uint32_t now)
{
    StopHardware(now);
    pending_direction = CMD_STOP;
    pending_recovery = 0U;
    recovery_active = 0U;
    jog_active = 0U;
    Encoder_SetZero();
    dbg_homed = 1U;
    dbg_fault &= (uint8_t)~FAULT_UNHOMED;
    dbg_state = DOOR_CLOSED;
    dbg_status_dirty = 1U;
}

static void StartMotor(uint8_t dir, uint8_t duty, uint32_t now, uint8_t recovery)
{
    EncoderSnapshot e;
    if (Door_LinkOk(now) == 0U) { FaultStop(FAULT_CAN_LOST, now); return; }
    if (dir == CMD_CLOSE && Board_SwitchPressed() != 0U)
    {
        SetClosed(now);
        return;
    }
    if (duty < MOTOR_PWM_MIN || duty > MOTOR_PWM_MAX)
    {
        dbg_last_reject = 8U;
        return;
    }
    StopHardware(now);
    Encoder_GetSnapshot(&e);
    motion_start_ms = now;
    run_start_edges = e.total;
    run_start_position = e.position;
    stall_sample_ms = now;
    stall_sample_position = e.position;
    stall_sample_invalid = e.invalid;
    motion_confirmed = 0U;
    recovery_active = recovery;
    dbg_stall_baseline_counts = 0U;
    dbg_stall_last_counts = 0U;
    dbg_stall_low_count = 0U;
    dbg_motor_dir = dir;
    dbg_motor_pwm = duty;
    dbg_state = (dir == CMD_OPEN) ? DOOR_OPENING : DOOR_CLOSING;
    Board_MotorSet(dir, duty);
    dbg_status_dirty = 1U;
}

static void QueueStart(uint8_t dir, uint8_t duty, uint32_t now, uint8_t recovery)
{
    StopHardware(now);
    dbg_state = DOOR_STOPPED;
    pending_direction = dir;
    pending_pwm = duty;
    pending_recovery = recovery;
    pending_since_ms = now;
    dbg_status_dirty = 1U;
}

static void ObstacleStop(uint32_t now)
{
    uint8_t previous = dbg_motor_dir;
    StopHardware(now);
    dbg_fault |= FAULT_OBSTACLE;
    ++dbg_obstacle_events;
    dbg_status_dirty = 1U;
    if (recovery_active != 0U || dbg_homed == 0U || jog_active != 0U)
    {
        FaultStop(FAULT_OBSTACLE | FAULT_ENCODER, now);
        return;
    }
    /* One reversal only: CLOSE -> calibrated OPEN endpoint; OPEN -> CLOSED switch. */
    if (previous == CMD_CLOSE && DOOR_OPEN_TARGET_COUNTS > 0U)
        QueueStart(CMD_OPEN, MOTOR_RECOVERY_PWM, now, 1U);
    else if (previous == CMD_OPEN)
        QueueStart(CMD_CLOSE, MOTOR_RECOVERY_PWM, now, 1U);
    else
        FaultStop(FAULT_ENCODER, now);
}

void Door_Init(void)
{
    switch_raw = Board_SwitchPressed();
    switch_stable = 0U;
    switch_changed_ms = g_ms;
    Board_MotorStop();
}

void Door_Heartbeat(uint32_t now)
{
    dbg_last_hb_ms = now;
    ++dbg_hb_count;
}

uint8_t Door_LinkOk(uint32_t now)
{
    return (dbg_hb_count != 0U &&
            (uint32_t)(now - dbg_last_hb_ms) <= HEARTBEAT_TIMEOUT_MS) ? 1U : 0U;
}

void Door_BusOff(uint32_t now)
{
    if ((dbg_fault & FAULT_BUS_OFF) == 0U)
    {
        FaultStop(FAULT_BUS_OFF, now);
        dbg_locked = 1U;
    }
}

void Door_Command(uint8_t cmd, uint8_t duty, uint32_t now)
{
    dbg_last_reject = 0U;
    if (cmd == CMD_STOP) { StopAndHold(now); return; }
    if (cmd == CMD_LOCK)
    {
        uint8_t moving = (dbg_motor_dir != CMD_STOP || pending_direction != CMD_STOP);
        StopAndHold(now);
        dbg_locked = 1U;
        if (moving != 0U) dbg_state = DOOR_STOPPED;
        dbg_status_dirty = 1U;
        return;
    }
    if (cmd == CMD_UNLOCK) { dbg_locked = 0U; dbg_status_dirty = 1U; return; }
    if (cmd == CMD_CLEAR_FAULT)
    {
        if (dbg_motor_dir != CMD_STOP || pending_direction != CMD_STOP)
            { dbg_last_reject = 1U; return; }
        /* Bus-off is not cleared in software until the CAN controller recovers. */
        dbg_fault &= FAULT_UNHOMED;
        if (dbg_homed == 0U) dbg_fault |= FAULT_UNHOMED;
        dbg_status_dirty = 1U;
        return;
    }
    if (cmd != CMD_OPEN && cmd != CMD_CLOSE &&
        cmd != CMD_JOG_OPEN && cmd != CMD_JOG_CLOSE)
        { dbg_last_reject = 2U; return; }
    if (dbg_locked != 0U) { dbg_last_reject = 3U; return; }
    if ((dbg_fault & (FAULT_BLOCKING | FAULT_OBSTACLE)) != 0U)
        { dbg_last_reject = 4U; return; }
    if (Door_LinkOk(now) == 0U) { dbg_last_reject = 5U; return; }
    if (recovery_active != 0U || pending_direction != CMD_STOP)
        { dbg_last_reject = 6U; return; }
    if (duty < MOTOR_PWM_MIN || duty > MOTOR_PWM_MAX)
        { dbg_last_reject = 8U; return; }
    if (cmd == CMD_JOG_OPEN || cmd == CMD_JOG_CLOSE)
    {
        if (dbg_motor_dir != CMD_STOP) { dbg_last_reject = 7U; return; }
        jog_active = 1U;
        StartMotor((cmd == CMD_JOG_OPEN) ? CMD_OPEN : CMD_CLOSE,
                   duty > MOTOR_RECOVERY_PWM ? MOTOR_RECOVERY_PWM : duty, now, 0U);
        return;
    }
    if (cmd == CMD_OPEN)
    {
        if (dbg_homed == 0U) { dbg_last_reject = 9U; return; }
        if (dbg_state == DOOR_OPENED) return;
    }
    else if (Board_SwitchPressed() != 0U && switch_stable != 0U)
    {
        SetClosed(now);
        return;
    }
    if (dbg_motor_dir == cmd) return;
    if (dbg_motor_dir != CMD_STOP ||
        (last_direction != CMD_STOP && last_direction != cmd &&
         (uint32_t)(now - last_motor_off_ms) < REVERSE_DEADTIME_MS))
        QueueStart(cmd, duty, now, 0U);
    else
        StartMotor(cmd, duty, now, 0U);
}

static void SafetyMonitor(const EncoderSnapshot *e, uint32_t now)
{
    int32_t since_start = DirectionalProgress(e->position, run_start_position, dbg_motor_dir);
    int32_t step;
    uint32_t dt, invalid_delta, measured, threshold;
    if (motion_confirmed == 0U)
    {
        if (since_start >= (int32_t)START_MIN_COUNTS)
        {
            motion_confirmed = 1U;
            stall_sample_position = e->position;
            stall_sample_invalid = e->invalid;
            stall_sample_ms = now;
        }
        else if ((uint32_t)(now - motion_start_ms) >= START_CONFIRM_MS)
            FaultStop(FAULT_ENCODER, now);
        return;
    }
    dt = now - stall_sample_ms;
    if (dt < STALL_SAMPLE_MS) return;
    step = DirectionalProgress(e->position, stall_sample_position, dbg_motor_dir);
    invalid_delta = e->invalid - stall_sample_invalid;
    stall_sample_position = e->position;
    stall_sample_invalid = e->invalid;
    stall_sample_ms = now;
    if (invalid_delta >= INVALID_SAMPLE_LIMIT &&
        (step <= 0 || invalid_delta > (uint32_t)step / 4U))
    {
        FaultStop(FAULT_ENCODER, now);
        return;
    }
    if (step < 0L) { FaultStop(FAULT_ENCODER, now); return; }
    measured = (uint32_t)step;
    dbg_stall_last_counts = measured;
    if ((uint32_t)(now - motion_start_ms) < STALL_START_GRACE_MS) return;
    if (measured >= MIN_BASELINE_COUNTS)
    {
        if (dbg_stall_baseline_counts == 0U)
            dbg_stall_baseline_counts = measured;
        else if (measured >= dbg_stall_baseline_counts / 2U)
            dbg_stall_baseline_counts = (3U * dbg_stall_baseline_counts + measured) / 4U;
    }
    if (dbg_stall_baseline_counts == 0U) return; /* Not yet calibrated on this run. */
    threshold = (dbg_stall_baseline_counts * STALL_THRESHOLD_PCT) / 100U;
    if (threshold < STALL_MIN_COUNTS) threshold = STALL_MIN_COUNTS;
    if (measured < threshold)
    {
        if (++dbg_stall_low_count >= STALL_LOW_SAMPLES) ObstacleStop(now);
    }
    else dbg_stall_low_count = 0U;
}

void Door_Tick(uint32_t now)
{
    uint8_t raw = Board_SwitchPressed();
    EncoderSnapshot e;
    Encoder_UpdateSpeed(now);
    if (dbg_motor_dir != CMD_STOP && dbg_encoder_counts_per_sec != 0U)
        dbg_last_running_pps = dbg_encoder_counts_per_sec;
    if (raw != switch_raw) { switch_raw = raw; switch_changed_ms = now; }
    if ((uint32_t)(now - switch_changed_ms) >= CLOSED_DEBOUNCE_MS)
    {
        switch_stable = raw;
        if (switch_stable != 0U && dbg_motor_dir != CMD_OPEN &&
            pending_direction != CMD_OPEN &&
            (dbg_state == DOOR_UNKNOWN || dbg_state == DOOR_CLOSING ||
             dbg_state == DOOR_STOPPED || dbg_state == DOOR_CLOSED))
        {
            if (dbg_homed == 0U || dbg_state != DOOR_CLOSED)
                SetClosed(now);
            g_limit_irq_stop = 0U;
            switch_interrupted_close = 0U;
        }
    }
    if (g_limit_irq_stop != 0U)
    {
        g_limit_irq_stop = 0U;
        if (dbg_motor_dir == CMD_CLOSE)
        {
            StopHardware(now);
            dbg_state = DOOR_STOPPED;
            switch_interrupted_close = 1U;
            dbg_status_dirty = 1U;
        }
        else if (dbg_motor_dir == CMD_OPEN &&
                 (uint32_t)(now - motion_start_ms) > OPEN_SWITCH_RELEASE_MS)
        {
            FaultStop(FAULT_LIMIT, now);
            return;
        }
    }
    if (switch_interrupted_close != 0U &&
        (uint32_t)(now - switch_changed_ms) >= CLOSED_DEBOUNCE_MS &&
        switch_stable == 0U)
    {
        switch_interrupted_close = 0U;
        FaultStop(FAULT_LIMIT, now);
        return;
    }
    if (dbg_motor_dir != CMD_STOP)
    {
        uint32_t elapsed = now - motion_start_ms;
        if (Door_LinkOk(now) == 0U) { FaultStop(FAULT_CAN_LOST, now); return; }
        if (elapsed >= MAX_TRAVEL_MS) { FaultStop(FAULT_TRAVEL, now); return; }
        if (dbg_motor_dir == CMD_CLOSE && raw != 0U)
        {
            StopHardware(now);
            dbg_state = DOOR_STOPPED;
            switch_interrupted_close = 1U;
            dbg_status_dirty = 1U;
            return;
        }
        if (dbg_motor_dir == CMD_OPEN && raw != 0U &&
            elapsed >= OPEN_SWITCH_RELEASE_MS)
        {
            FaultStop(FAULT_LIMIT, now);
            return;
        }
        if (jog_active != 0U && elapsed >= 180U)
        {
            StopAndHold(now);
            return;
        }
        Encoder_GetSnapshot(&e);
#if BENCH_ENCODER_TEST
        if (dbg_motor_dir == CMD_OPEN && recovery_active == 0U && jog_active == 0U)
        {
            if (elapsed >= BENCH_RUN_MS)
            {
                StopAndHold(now); /* Timed test does not prove OPENED. */
            }
            return;
        }
#endif
        if (dbg_motor_dir == CMD_OPEN && jog_active == 0U &&
            e.position >= (int32_t)DOOR_OPEN_TARGET_COUNTS)
        {
            if (raw != 0U) { FaultStop(FAULT_LIMIT, now); return; }
            StopHardware(now);
            dbg_state = DOOR_OPENED;
            recovery_active = 0U;
            dbg_status_dirty = 1U;
            return;
        }
        if (jog_active == 0U) SafetyMonitor(&e, now);
        return;
    }
    if (pending_direction != CMD_STOP &&
        (uint32_t)(now - pending_since_ms) >= REVERSE_DEADTIME_MS)
    {
        uint8_t dir = pending_direction, duty = pending_pwm, recover = pending_recovery;
        pending_direction = CMD_STOP;
        pending_recovery = 0U;
        if (dbg_locked == 0U && Door_LinkOk(now) != 0U &&
            (dbg_fault & (FAULT_CAN_LOST | FAULT_BUS_OFF |
                          FAULT_ENCODER | FAULT_TRAVEL | FAULT_LIMIT)) == 0U)
            StartMotor(dir, duty, now, recover);
        else FaultStop(FAULT_CAN_LOST, now);
    }
}

uint8_t Door_StatusFlags(uint32_t now)
{
    uint8_t flags = 0U;
    if (switch_stable != 0U) flags |= STATUS_CLOSED;
    if (dbg_homed != 0U) flags |= STATUS_HOMED;
    if (recovery_active != 0U || pending_recovery != 0U) flags |= STATUS_REVERSING;
    if (Door_LinkOk(now) != 0U) flags |= STATUS_LINK_OK;
#if BENCH_ENCODER_TEST
    flags |= STATUS_BENCH;
#endif
    if (motion_confirmed != 0U) flags |= STATUS_ENCODER_OK;
    flags |= STATUS_TARGET_ESTIMATED; /* Nominal gear ratio; not door-angle calibration. */
    return flags;
}
