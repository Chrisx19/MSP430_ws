#include "driverlib.h"
#include <stdbool.h>

volatile bool readyToSendTemp = false;

void Timer_Init(void);

void main (void)
{
    //Stop WDT
    WDT_A_hold(WDT_A_BASE);
    PMM_unlockLPM5();

    // Error R LED
       GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN6);
       GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN6);

//    EVR_Init();
    Timer_Init();
    __enable_interrupt();

    while (1) {
        if (readyToSendTemp == true) {
//            EVR_Debug("LED Stats: 1  PGOOD Stats: 1  PWM Lvl: 255  LED: 0.1A  Temp: 23\n\r");
            GPIO_toggleOutputOnPin(GPIO_PORT_P4, GPIO_PIN6);
            readyToSendTemp = false;
        }
    }
}

// ========== TIMER INIT ==========
void Timer_Init(void)
{
    Timer_A_initUpModeParam timerConfig = {
        .clockSource = TIMER_A_CLOCKSOURCE_ACLK,            // Measured ACLK using LogicA 37675 Hz
        .clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_8,// Calc div to match 5 second interrupt
        .timerPeriod = 23546,                               // Calc period for precise 5 second interrupt                 
        .timerInterruptEnable_TAIE = TIMER_A_TAIE_INTERRUPT_DISABLE,
        .captureCompareInterruptEnable_CCR0_CCIE = TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE, // Need this for CCRO ISR
        .timerClear = TIMER_A_DO_CLEAR,
        .startTimer = false
    };

    Timer_A_initUpMode(TIMER_A2_BASE, &timerConfig);
    Timer_A_startCounter(TIMER_A2_BASE, TIMER_A_UP_MODE);
}

/* 
======================== TIMER_A2 ISR ========================
Use for interval print for telemetry
Interrupt will happen every ~5 seconds
Time_interval = (1 / ACLK) * clk_src_div * ( timer_Period + 1)
Measured by discrete I/O using logic analyzer
*/
#pragma vector=TIMER2_A0_VECTOR
__interrupt void Timer_A2_ISR(void)
{
    readyToSendTemp = true;
}
