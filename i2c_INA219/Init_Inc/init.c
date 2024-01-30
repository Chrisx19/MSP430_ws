/*
 * init.c
 *
 *  Created on: Jan 27, 2024
 *      Author: chris
 */
#include <msp430.h>
#include "init.h"

void disableInit(void) {
    // Disable the GPIO power-on default high-impedance mode to activate
    // previously configured port settings
    PM5CTL0 &= ~LOCKLPM5;
    WDTCTL = WDTPW | WDTHOLD;   // stop watchdog timer
}
