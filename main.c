#include "wdt_a.h"
#include "ina219.h"

float currentmA_g = 0;

void main (void)
{
    //Stop WDT
    WDT_A_hold(WDT_A_BASE);
    i2cInit();
    __bis_SR_register(GIE);  // Enables global interrupts
    //Set P1.0 as an output pin.
    /*
     * Select Port 1
     * Set Pin 0 as output
     */
    GPIO_setAsOutputPin(
        GPIO_PORT_P1,
        GPIO_PIN0
    );

    INA_Init();

     while (1)
    {
         currentmA_g = getCurrent_mA();
         GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0);
         __delay_cycles(2000000);
    }
}
