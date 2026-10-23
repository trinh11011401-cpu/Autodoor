#include "S32K144.h"
#include "Can.h"
#include "Board_Can.h"
/* Platform (Mcu/Port replacement for single-file learning). */
Std_ReturnType Board_CanMcuClockInit(void)
{
    uint32 timeout = CAN_HW_TIMEOUT;
    can_step = 1U;
    if ((SCG->SOSCCSR & SCG_SOSCCSR_SOSCVLD_MASK) == 0U) {
        SCG->SOSCCSR &= ~SCG_SOSCCSR_SOSCEN_MASK;
        SCG->SOSCDIV = SCG_SOSCDIV_SOSCDIV1(1U) |
                       SCG_SOSCDIV_SOSCDIV2(1U);
        SCG->SOSCCFG = SCG_SOSCCFG_RANGE(2U) |
                       SCG_SOSCCFG_EREFS_MASK;
        SCG->SOSCCSR |= SCG_SOSCCSR_SOSCEN_MASK;
        while (((SCG->SOSCCSR & SCG_SOSCCSR_SOSCVLD_MASK) == 0U) &&
               (timeout != 0U)) { --timeout; }
        if (timeout == 0U) { can_step = 101U; return E_NOT_OK; }
    }
    PCC->PCCn[PCC_FlexCAN0_INDEX] |= PCC_PCCn_CGC_MASK;
    return E_OK;
}
void Board_CanPortInit(void)
{
    PCC->PCCn[PCC_PORTE_INDEX] |= PCC_PCCn_CGC_MASK;
    PORTE->PCR[4] = PORT_PCR_MUX(5U); /* CAN0 RX */
    PORTE->PCR[5] = PORT_PCR_MUX(5U); /* CAN0 TX */
}

