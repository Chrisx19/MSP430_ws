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
    // Timer_A0 => 5-second period using ACLK/8 = 4096 Hz
    // 5 seconds * 4096 = 20480 => CCR0 = 20479
    Timer_A_initUpModeParam timerConfig = {
        .clockSource = TIMER_A_CLOCKSOURCE_ACLK,              // Use ACLK (~32 kHz)
        .clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_1,  // Divide by 8
        .timerPeriod = 20479,                                 // 5 seconds - 1
        .timerInterruptEnable_TAIE = TIMER_A_TAIE_INTERRUPT_DISABLE,
        .captureCompareInterruptEnable_CCR0_CCIE = TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE,
        .timerClear = TIMER_A_DO_CLEAR,
        .startTimer = false
    };

    Timer_A_initUpMode(TIMER_A2_BASE, &timerConfig);
    Timer_A_startCounter(TIMER_A2_BASE, TIMER_A_UP_MODE);
}

// ========== TIMER_A0 ISR ==========
//#pragma vector=TIMER0_A0_VECTOR //Timer A0
#pragma vector=TIMER2_A0_VECTOR //Timer_A1
__interrupt void Timer_A2_ISR(void)
{
    // This interrupt occurs every 5 seconds
    readyToSendTemp = true;
}

