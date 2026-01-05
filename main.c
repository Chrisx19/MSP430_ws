#include "driverlib.h"
#include "evr.h"

void main (void)
{
    //Stop WDT
    WDT_A_hold(WDT_A_BASE);

    /* Set DCO to 8 MHz */
    CS_setDCOFreq(CS_DCORSEL_1, CS_DCOFSEL_3);
    CS_initClockSignal(CS_ACLK, CS_LFMODOSC_SELECT, CS_CLOCK_DIVIDER_1);
    CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);
    CS_initClockSignal(CS_MCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);

    if (EVR_Init() != EVR_SUCCESS) {
        __bis_SR_register(LPM0_bits); // CPU off
        __no_operation(); // Remain in LPM0
    } else {
        EVR("EVR Initialized.");
    }

     while (1)
    {
    	__bis_SR_register(LPM0_bits); // CPU off
        __no_operation(); // Remain in LPM0
    }
}
