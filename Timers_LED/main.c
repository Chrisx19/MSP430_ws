#include <msp430.h> 


unsigned int timerCount = 0;    // Keep track of timer ticks

/**
 * main.c
 */
int main(void)
{
    PM5CTL0 &= ~LOCKLPM5;
    WDTCTL = WDTPW | WDTHOLD;   // stop watchdog timer

    P1DIR |= 0x01;  // Set direction of P1.0 to out

    TA0CTL |= 0x0100;    // Choose ACLK
    TA0CTL |= 0x0010;    // Count up mode
    TA0CTL |= 0x0004;    // Clear the timer;

    TA0CCTL0 = 0x0010;          // Enable interrupt in capture and control register
    TA0CCR0 = 4096;             // Set capture and control threshold to 1/8 of 32kHz clock

    __bis_SR_register(LPM0_bits + GIE);       // Enter LPM0, enable interrupts
}


#pragma vector=TIMER0_A0_VECTOR
__interrupt void TIMER0_A0_ISR(void)
{
    timerCount++;

    // Timer is 1/8 of a second, so count up to 8 for 1 second toggle
    if (timerCount >= 8)
    {
        timerCount = 0;

        P1OUT ^= 0x01;
    }
}
