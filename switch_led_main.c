#include <msp430.h> 


/**
 * main.c
 */
int main(void)
{
    PM5CTL0 &= ~LOCKLPM5;
	WDTCTL = WDTPW | WDTHOLD;	// stop watchdog timer

    // Configure P1.1 as input for the button
    P1DIR &= ~BIT1;  // Set P1.1 as input
    P4DIR &= ~BIT5;  // Set P4.5 as input

    P1REN |= 0x02;   // Enable pull-up/pull-down resistor
    P4REN |= 0x20;
    P1OUT |= 0x02;   // Select pull-up mode
    P4OUT |= 0x20;

    // Configure P1.0 as output for LED1
    P1DIR |= 0x01;  // //BIT0
    P4DIR |= 0x40; //0100 0000 Set P4.6 to output direction

    while(1) {
        if((P1IN & BIT1) != BIT1) {  // Check if button is pressed
            P1OUT |= 0x01;    // Turn on LED1

        }
        else if (!(P4IN & 0x20)) {
            P4OUT |= 0x40;    //BIT6
        } else {
            P1OUT &= ~0x01;   // Turn off LED1
            P4OUT &= ~0x40;
        }
    }
	return 0;
}
