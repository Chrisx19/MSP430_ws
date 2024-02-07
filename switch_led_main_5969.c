#include <msp430.h>
#define ACLK 0x0100 // Timer_A ACLK source
#define UP 0x0010 // Timer_A UP mode
main()
{
    PM5CTL0 &= ~LOCKLPM5;
    WDTCTL = WDTPW | WDTHOLD;   // stop watchdog timer
    P1DIR |= BIT0; // Green LED is on Port 9, bit 7 (P9.7)
    P4DIR |= BIT0; // Green LED is on Port 9, bit 7 (P9.7)
    P1DIR &= ~BIT2; // Ensure P1.1 button is an input and
    // P1.0 is an output
    P1REN |= BIT2;
    P1OUT |= BIT2; // P1.1 button needs a pull-up resistor
    TA0CCR0 = 40000; // 40000 * 25us = 1000000us = 1second
    TA0CTL |= ACLK | UP; // Set ACLK, UP mode
    TA0CCTL0 = CCIE; // Enable interrupt for Timer_0
 __bis_SR_register(GIE);
     while(1) // Keep looping forever
     {
         if((BIT2 & P1IN) == 0) // Is P1.1 button pushed?
         {
             P4OUT |= BIT0; // Turn on the green LED (P9.7)
         } else {
             P4OUT &= ~BIT0; // Turn off the green LED (P9.7)
         }
     }
}
//************************************************************************
// Timer0 Interrupt Service Routine
//************************************************************************
#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer0_ISR (void)
{
 P1OUT ^= BIT0; // Toggle red LED on P1.0
}
