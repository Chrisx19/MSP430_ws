#include <msp430.h>
#include <stdint.h>

uint8_t timerCount = 0;
int code = 0;
/**
 * main.c
 */

void init(void) {
    PM5CTL0 &= ~LOCKLPM5;
    WDTCTL = WDTPW | WDTHOLD;   // stop watchdog timer
}


void clkInit(void)
{
    TA0CTL |= 0x0100; //0000 0001 0000 0000 SETTING ACLK ~ TASSEL
    TA0CTL |= 0x0010; //0000 0001 0001 0000 SETTING UP MODE which correspond to TAXCCR0
    TA0CTL |= 0x0004; //0000 0001 0001 0100 CLEAR; automatic reset when counter is up max

    TA0CCR0 = 4096; //Set 32768/8=4096 to set the pulse faster; for Capture & control threshold
    TA0CCTL0 |= 0x0010; //enables ISR on capture & compare mode

}

void switchInit(void)
{
    P1DIR &= ~0x04; //bit2
    P1REN |= 0x04; //enable pull up or pulldown
    P1OUT |= 0x04; //pull up, so its always on or '1'
}

int main(void)
{
    init();
    clkInit();

    // Configure P1.0 as output for LED1
    P1DIR |= 0x01;  // //BIT0
    P4DIR |= 0x01; //0100 0000 Set P4.6 to output direction

    __bis_SR_register(LPM0_bits + GIE);
}



#pragma vector=TIMER0_A0_VECTOR
__interrupt void TIMER0_A0_ISR(void)
{
    timerCount++;
    if ((P1IN & 0x04) != 0x04) {
        code ^= 1;
    }

    if(code) {
        if (timerCount >= 8)  {
                timerCount = 0;
                P1OUT ^= 0x01;
                P4OUT ^= 0x01;
        }
    }


}
