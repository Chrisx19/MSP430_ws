#include <msp430.h> 


/**
 * main.c
 */
int main(void)
{
    PM5CTL0 &= ~LOCKLPM5;
	WDTCTL = WDTPW | WDTHOLD;	// stop watchdog timer

	//Setting P1 for 0th bit as output config
	P1DIR |= 0x01;
	//Setting P4 for 0th bit as output config
	P4DIR |= 0x01;

	//Setting 0th LED of port 1 and port 4 to high
	P1OUT |= 0x01;
	P4OUT |= 0x01;

	
	return 0;
}
