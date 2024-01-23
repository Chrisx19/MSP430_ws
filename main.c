#include <msp430.h> 


/**
 * main.c
 */
int main(void)
{
    PM5CTL0 &= ~LOCKLPM5;
	WDTCTL = WDTPW | WDTHOLD;	// stop watchdog timer

	//Setting P1 as output config
	P1DIR |= 0x01;
	P4DIR |= 0x01;
	P1OUT |= 0x01;
	P4OUT |= 0x01;

	
	return 0;
}
