#include <msp430.h>
#include <stdint.h>

/**
 * main.c
 */

void init(void);

uint8_t timerCount = 0;

void clkInit(void)
{
    TA0CTL |= 0x0100; //0000 0001 0000 0000 SETTING ACLK ~ TASSEL
    TA0CTL |= 0x0010; //0000 0001 0001 0000 SETTING UP MODE which correspond to TAXCCR0
    TA0CTL |= 0x0004; //0000 0001 0001 0100 CLEAR; automatic reset when counter is up max

    TA0CCR0 = 4096; //Set 32768/8=4096 to set the pulse faster; for Capture & control threshold
    TA0CCTL0 |= 0x0010; //enables ISR on capture & compare mode

}

int main(void)
{
    init();
    clkInit();

    // Configure P1.0 as output for LED1
    P1DIR |= 0x01;  // //BIT0
    P4DIR |= 0x40; //0100 0000 Set P4.6 to output direction

//    while(1) {
//        if((P1IN & BIT1) == 0x00) {  // Check if button is pressed
//            P1OUT |= 0x01;    // Turn on LED1
//
//        }
//        else if (!(P4IN & 0x20)) {
//            P4OUT |= 0x40;    //BIT6
//        } else {
//            P1OUT &= ~0x01;   // Turn off LED1
//            P4OUT &= ~0x40;
//        }
//    }
    __bis_SR_register(LPM0_bits + GIE);
}

void init(void) {
    PM5CTL0 &= ~LOCKLPM5;
    WDTCTL = WDTPW | WDTHOLD;   // stop watchdog timer
}

#pragma vector=TIMER0_A0_VECTOR
__interrupt void TIMER0_A0_ISR(void)
{
    timerCount++;
    if (timerCount >= 8)  {
        timerCount = 0;
        P1OUT ^= 0x01;
        P4OUT ^= 0x40;
    }
}
