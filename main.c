#include <msp430.h>
#include "driverlib.h"
/*
This code example if want to read real measurement of Source clock such as SMCLK at this case. P3.4 is the output of the clock that can be measured with logic analyzer. 
This code only works for MSP430FR5969, now if different board the need to read its own pins datasheet
*/
int main(void)
{
    WDT_A_hold(WDT_A_BASE);        // Stop watchdog
    PMM_unlockLPM5();         // Unlock GPIO from LPMx.5

    // Set DCO = 1 MHz
    CS_setDCOFreq(CS_DCORSEL_0, CS_DCOFSEL_0);
    // SMCLK = DCO @ 1 MHz, no division
    CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);

    // P3.4 -> SMCLK output
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P3,
                                                GPIO_PIN4,
                                                GPIO_SECONDARY_MODULE_FUNCTION);

    while(1)
    {
        __no_operation();  // SMCLK should appear on P3.4
    }
}
