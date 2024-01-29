#include <msp430.h>


volatile unsigned char RXData;


int main(void)
{
  WDTCTL = WDTPW | WDTHOLD;


  // Configure GPIO
  P1OUT &= ~BIT0;                           // Clear P1.0 output latch
  P1DIR |= BIT0;                            // For LED
  P1SEL1 |= BIT6 | BIT7;                    // I2C pins


  // Disable the GPIO power-on default high-impedance mode to activate
  // previously configured port settings
  PM5CTL0 &= ~LOCKLPM5;


  // Configure USCI_B0 for I2C mode
  UCB0CTLW0 |= UCSWRST;                     // Software reset enabled
  UCB0CTLW0 |= UCMODE_3 | UCMST | UCSYNC;   // I2C mode, Master mode, sync
  UCB0CTLW1 |= UCASTP_2;                    // Automatic stop generated
                                            // after UCB0TBCNT is reached
//  UCB0BRW = 0x0008;                         // baudrate = SMCLK / 8
  UCB0BRW = 10;
  UCB0TBCNT = 0x0005;                       // number of bytes to be received
  UCB0I2CSA = 0x0077;                       // Slave address
  UCB0CTL1 &= ~UCSWRST;
  UCB0IE |= UCRXIE | UCNACKIE | UCBCNTIE;


  while (1)
  {
    __delay_cycles(2000);
    while (UCB0CTL1 & UCTXSTP);             // Ensure stop condition got sent
    UCB0CTL1 |= UCTXSTT;                    // I2C start condition


    __bis_SR_register(LPM0_bits | GIE);     // Enter LPM0 w/ interrupt
  }
}

//pg 843
#pragma vector = USCI_B0_VECTOR
__interrupt void USCI_B0_ISR(void)
{
  switch(UCB0IV)
  {
    case USCI_NONE:          break;         // Vector 0: No interrupts
    case USCI_I2C_UCALIFG:   break;         // Vector 2: ALIFG
    case USCI_I2C_UCNACKIFG:                // Vector 4: NACKIFG
      UCB0CTL1 |= UCTXSTT;                  // I2C start condition
      break;
    case USCI_I2C_UCSTTIFG:  break;         // Vector 6: STTIFG
    case USCI_I2C_UCSTPIFG:  break;         // Vector 8: STPIFG
    case USCI_I2C_UCRXIFG3:  break;         // Vector 10: RXIFG3
    case USCI_I2C_UCTXIFG3:  break;         // Vector 12: TXIFG3
    case USCI_I2C_UCRXIFG2:  break;         // Vector 14: RXIFG2
    case USCI_I2C_UCTXIFG2:  break;         // Vector 16: TXIFG2
    case USCI_I2C_UCRXIFG1:  break;         // Vector 18: RXIFG1
    case USCI_I2C_UCTXIFG1:  break;         // Vector 20: TXIFG1
    case USCI_I2C_UCRXIFG0:                 // Vector 22: RXIFG0
      RXData = UCB0RXBUF;                   // Get RX data
      __bic_SR_register_on_exit(LPM0_bits); // Exit LPM0
      break;
    case USCI_I2C_UCTXIFG0:  break;         // Vector 24: TXIFG0
    case USCI_I2C_UCBCNTIFG:                // Vector 26: BCNTIFG
      P1OUT ^= BIT0;                        // Toggle LED on P1.0
      break;
    case USCI_I2C_UCCLTOIFG: break;         // Vector 28: clock low timeout
    case USCI_I2C_UCBIT9IFG: break;         // Vector 30: 9th bit
    default: break;
  }
}
