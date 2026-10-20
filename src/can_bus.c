#include "S32K144.h"
#include "can_bus.h"

#define TX_MB 0U
#define COMMAND_MB 4U
#define HEARTBEAT_MB 5U
#define MAX_MB 15U
#define INIT_TIMEOUT 1000000UL

volatile uint32_t dbg_can_rx_count = 0U;
volatile uint32_t dbg_can_tx_count = 0U;
volatile uint32_t dbg_can_error_count = 0U;
volatile uint32_t dbg_can_init_step = 0U;

static uint8_t tx_pending = 0U;
static uint32_t tx_started_ms = 0U;

static uint8_t WaitMasked(volatile uint32_t *reg, uint32_t mask, uint8_t wanted_set)
{
    uint32_t timeout = INIT_TIMEOUT;
    while (timeout-- != 0U)
    {
        if (((*reg & mask) != 0U) == (wanted_set != 0U)) return 1U;
    }
    return 0U;
}

static uint8_t ClockInit(void)
{
    if ((SCG->SOSCCSR & SCG_SOSCCSR_SOSCVLD_MASK) != 0U) return 1U;
    SCG->SOSCCSR &= ~SCG_SOSCCSR_SOSCEN_MASK;
    SCG->SOSCDIV = SCG_SOSCDIV_SOSCDIV1(1U) | SCG_SOSCDIV_SOSCDIV2(1U);
    SCG->SOSCCFG = SCG_SOSCCFG_RANGE(2U) | SCG_SOSCCFG_EREFS_MASK;
    SCG->SOSCCSR |= SCG_SOSCCSR_SOSCEN_MASK;
    return WaitMasked(&SCG->SOSCCSR, SCG_SOSCCSR_SOSCVLD_MASK, 1U);
}

static void InitRxMb(uint8_t mb, uint16_t id)
{
    uint32_t b = (uint32_t)mb * 4U;
    CAN0->RAMn[b] = 0U;
    CAN0->RAMn[b + 1U] = (uint32_t)id << 18U;
    CAN0->RAMn[b + 2U] = 0U;
    CAN0->RAMn[b + 3U] = 0U;
    CAN0->RAMn[b] = 0x04000000UL;
    CAN0->IFLAG1 = 1UL << mb;
}

uint8_t Can_Init(void)
{
    uint32_t i;
    dbg_can_init_step = 1U;
    if (ClockInit() == 0U) return 0U;
    PCC->PCCn[PCC_PORTE_INDEX] |= PCC_PCCn_CGC_MASK;
    PORTE->PCR[4] = PORT_PCR_MUX(5U);
    PORTE->PCR[5] = PORT_PCR_MUX(5U);
    PCC->PCCn[PCC_FlexCAN0_INDEX] |= PCC_PCCn_CGC_MASK;
    CAN0->CTRL1 &= ~CAN_CTRL1_CLKSRC_MASK;
    CAN0->MCR &= ~CAN_MCR_MDIS_MASK;
    dbg_can_init_step = 2U;
    if (WaitMasked(&CAN0->MCR, CAN_MCR_LPMACK_MASK, 0U) == 0U) return 0U;
    CAN0->MCR |= CAN_MCR_FRZ_MASK | CAN_MCR_HALT_MASK;
    dbg_can_init_step = 3U;
    if (WaitMasked(&CAN0->MCR, CAN_MCR_FRZACK_MASK, 1U) == 0U) return 0U;
    CAN0->MCR &= ~(CAN_MCR_FDEN_MASK | CAN_MCR_RFEN_MASK | CAN_MCR_MAXMB_MASK);
    CAN0->MCR |= CAN_MCR_MAXMB(MAX_MB) | CAN_MCR_SRXDIS_MASK;
    /* 8 MHz / (1 * 16 TQ) = 500 kbps, same timing as proven CAN0 version. */
    CAN0->CTRL1 = CAN_CTRL1_PRESDIV(0U) | CAN_CTRL1_RJW(2U) |
                  CAN_CTRL1_PSEG1(6U) | CAN_CTRL1_PSEG2(2U) |
                  CAN_CTRL1_PROPSEG(4U);
    CAN0->CTRL1 &= ~CAN_CTRL1_CLKSRC_MASK;
    for (i = 0U; i < 128U; ++i) CAN0->RAMn[i] = 0U;
    CAN0->RXMGMASK = 0xFFFFFFFFUL;
    CAN0->RAMn[0] = 0x08000000UL;
    CAN0->IFLAG1 = 0xFFFFFFFFUL;
    InitRxMb(COMMAND_MB, CAN_ID_COMMAND);
    InitRxMb(HEARTBEAT_MB, CAN_ID_HEARTBEAT);
    CAN0->MCR &= ~CAN_MCR_HALT_MASK;
    dbg_can_init_step = 4U;
    if (WaitMasked(&CAN0->MCR, CAN_MCR_FRZACK_MASK, 0U) == 0U) return 0U;
    if (WaitMasked(&CAN0->MCR, CAN_MCR_NOTRDY_MASK, 0U) == 0U) return 0U;
    dbg_can_init_step = 5U;
    return 1U;
}

static uint8_t ReadMb(uint8_t mb, CanFrame *frame)
{
    uint32_t b = (uint32_t)mb * 4U;
    uint32_t cs, id, w0, w1, dummy;
    uint8_t i, code;
    if ((CAN0->IFLAG1 & (1UL << mb)) == 0U) return 0U;
    cs = CAN0->RAMn[b];
    id = CAN0->RAMn[b + 1U];
    w0 = CAN0->RAMn[b + 2U];
    w1 = CAN0->RAMn[b + 3U];
    dummy = CAN0->TIMER; (void)dummy;
    CAN0->IFLAG1 = 1UL << mb;
    code = (uint8_t)((cs >> 24U) & 0x0FU);
    if ((cs & ((1UL << 21U) | (1UL << 20U))) != 0U ||
        ((cs >> 16U) & 0xFU) > 8U || (code != 2U && code != 6U))
    {
        ++dbg_can_error_count;
        return 0U;
    }
    if (code == 6U) ++dbg_can_error_count; /* RX overrun */
    frame->id = (uint16_t)((id >> 18U) & 0x7FFU);
    frame->dlc = (uint8_t)((cs >> 16U) & 0xFU);
    for (i = 0U; i < frame->dlc; ++i)
    {
        uint32_t w = (i < 4U) ? w0 : w1;
        frame->data[i] = (uint8_t)(w >> (24U - (uint32_t)(i % 4U) * 8U));
    }
    ++dbg_can_rx_count;
    return 1U;
}

uint8_t Can_ReadCommand(CanFrame *frame) { return ReadMb(COMMAND_MB, frame); }
uint8_t Can_ReadHeartbeat(CanFrame *frame) { return ReadMb(HEARTBEAT_MB, frame); }

uint8_t Can_BusOff(void)
{
    return ((CAN0->ESR1 & (3UL << 4U)) == (2UL << 4U)) ? 1U : 0U;
}

void Can_TxPoll(uint32_t now)
{
    if (tx_pending == 0U) return;
    if ((CAN0->IFLAG1 & (1UL << TX_MB)) != 0U)
    {
        CAN0->IFLAG1 = 1UL << TX_MB;
        ++dbg_can_tx_count;
        tx_pending = 0U;
    }
    else if ((uint32_t)(now - tx_started_ms) >= 1000U)
    {
        ++dbg_can_error_count;
        /* Do not overwrite a busy TX MB before CAN reset / recovery. */
        tx_started_ms = now;
    }
}

uint8_t Can_TrySend(const CanFrame *frame, uint32_t now)
{
    uint32_t w0 = 0U, w1 = 0U;
    uint8_t i;
    if (frame->dlc > 8U || frame->id > 0x7FFU ||
        tx_pending != 0U || Can_BusOff() != 0U) return 0U;
    for (i = 0U; i < frame->dlc; ++i)
    {
        if (i < 4U) w0 |= (uint32_t)frame->data[i] << (24U - 8U * i);
        else w1 |= (uint32_t)frame->data[i] << (24U - 8U * (i - 4U));
    }
    CAN0->IFLAG1 = 1UL << TX_MB;
    CAN0->RAMn[0] = 0x08000000UL;
    CAN0->RAMn[1] = (uint32_t)frame->id << 18U;
    CAN0->RAMn[2] = w0;
    CAN0->RAMn[3] = w1;
    CAN0->RAMn[0] = (0xCUL << 24U) | ((uint32_t)frame->dlc << 16U);
    tx_started_ms = now;
    tx_pending = 1U;
    return 1U;
}
