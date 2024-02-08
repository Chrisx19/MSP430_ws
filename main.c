#include <msp430.h> 
#define ACLK 0x0100 // Timer ACLK source
#define UP 0x0010 // Timer UP mode
/**
 * main.c
 */
int main(void)
{
    PM5CTL0 &= ~LOCKLPM5;
	WDTCTL = WDTPW | WDTHOLD;	// stop watchdog timer
	
     P1DIR |= BIT0; // Set red LED as an output
     P1OUT &= ~BIT0; // Start with red LED off

     P1DIR &= ~BIT2;
     P1REN |= BIT2;

     P1IE |= BIT2;
     P1IES |= BIT2;

     P1IFG = 0x00;
     __bis_SR_register(GIE);
     while(1); // Wait here for interrupt
}

//***********************************************************************
//* Port 1 Interrupt Service Routine
//***********************************************************************
#pragma vector=PORT1_VECTOR
__interrupt void Port_1(void)
{

    unsigned long delay; // Wait for bouncing to end
    //more accurate if checked how long we have to wait on oscilloscope
    for(delay=0;delay<20000;delay=delay+1);

    P1OUT = P1OUT ^ BIT0; // Toggle the red LED with every
    // button push
    P1IFG &= ~(BIT2); // Clear P1.1 interrupt flag
}
