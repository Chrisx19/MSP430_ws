#include "driverlib.h"
#include "ina219.h"

uint8_t RXData;

float currentmA_g = 0;

INA219_config_t *ina = &(INA219_config_t) {
    .txBuffer = {0x00},
    .rxBuffer = {0x00},
    .CALIBRATION_VALUE = 8192,
};

void main (void)
{
    /*
     * Disable the GPIO power-on default high-impedance mode to activate
     * previously configured port settings
     */

    PMM_unlockLPM5();
    //Stop WDT
    WDT_A_hold(WDT_A_BASE);

    //Set DCO frequency to 1MHz
    CS_setDCOFreq(CS_DCORSEL_0,CS_DCOFSEL_0);
    //Set ACLK = VLO with frequency divider of 1
    CS_initClockSignal(CS_ACLK,CS_VLOCLK_SELECT,CS_CLOCK_DIVIDER_1);
    //Set SMCLK = DCO with frequency divider of 1
    CS_initClockSignal(CS_SMCLK,CS_DCOCLK_SELECT,CS_CLOCK_DIVIDER_1);
    //Set MCLK = DCO with frequency divider of 1
    CS_initClockSignal(CS_MCLK,CS_DCOCLK_SELECT,CS_CLOCK_DIVIDER_1);

    //Set P1.0 as an output pin.
    /*
     * Select Port 1
     * Set Pin 0 as output
     */
    GPIO_setAsOutputPin(
        GPIO_PORT_P1,
        GPIO_PIN0
    );

    EUSCI_B_I2C_clearInterrupt(EUSCI_B0_BASE,
        EUSCI_B_I2C_TRANSMIT_INTERRUPT0 +
        EUSCI_B_I2C_BYTE_COUNTER_INTERRUPT
        );

    //Enable master Receive interrupt
    EUSCI_B_I2C_enableInterrupt(EUSCI_B0_BASE,
        EUSCI_B_I2C_RECEIVE_INTERRUPT0 +
        EUSCI_B_I2C_TRANSMIT_INTERRUPT0 +
        EUSCI_B_I2C_BYTE_COUNTER_INTERRUPT
        );

    INA_Init(ina);

     while (1)
    {
//         currentmA_g = getCurrent_mA(ina);
//         __delay_cycles(2000000);

    }
}



