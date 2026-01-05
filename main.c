#include "humidity_sensor_sht31.h"
#include "evr.h"
#include "driverlib.h"

static Hum_Sensor_t hum = {0};

void main(void)
{
    //Stop watchdog timer
    WDT_A_hold(WDT_A_BASE);
    PMM_unlockLPM5();

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

    if (Hum_Init(&hum) != HUM_STATUS_SUCCESS) {
        __bis_SR_register(LPM0_bits); // CPU off
        __no_operation(); // Remain in LPM0
    } else {
        EVR("Humidity Initialized.");
    }

    __bis_SR_register(GIE); //enable interrupts

    while(1) {
        if (Hum_Get_Humidity(&hum) != HUM_STATUS_SUCCESS) {
            __bis_SR_register(LPM0_bits); // CPU off
            __no_operation(); // Remain in LPM0
        } else {
            EVR("Humidity: %.02f%", hum.humidity);
        }

        __delay_cycles(8000000); // ~1 second @ 8 MHz
    }
}
