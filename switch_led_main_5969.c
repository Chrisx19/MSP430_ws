#include <msp430.h>
#include <stdint.h>


/**
 * main.c
 */

void init(void) {
    PM5CTL0 &= ~LOCKLPM5;
    WDTCTL = WDTPW | WDTHOLD;   // stop watchdog timer
}


void switchInit(void)
{
    P1DIR &= ~0x02; //bit2
    P1REN |= 0x02; //enable pull up or pulldown
    P1OUT |= 0x02; //pull up, so its always on or '1'
}

int main(void)
{
    init();
    switchInit();

    // Configure P1.0 as output for LED1
    P1DIR |= 0x01;  // //BIT0
    P4DIR |= 0x40; //0100 0000 Set P4.6 to output direction

    while(1) {
        if ((P1IN & 0x02) != 0x02) {
            P1OUT ^= 0x01;
            P4OUT ^= 0x40;
        } else {
            P1OUT = P1OUT;
            P4OUT = P4OUT;
        }
    }
}
