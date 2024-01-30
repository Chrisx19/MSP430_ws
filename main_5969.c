#include <msp430.h>

#define SCL BIT7
#define SDA BIT6

void i2cInit(void){
    // Configure GPIO
    P1SEL1 |= SCL;
    P1SEL1 |= SDA;

    // USCI_B0
    UCB0CTLW0 |= UCSWRST;                     // Software reset enabled
    UCB0CTLW0 |= UCSSEL_3;                    // SMCLK
    UCB0CTLW0 |= UCMODE_3 | UCMST;            // I2C mode, Master mode
    UCB0CTLW0 |= UCTR;                        // Transmit Mode
    UCB0CTLW1 |= UCASTP_2;                    // Automatic stop generated

    UCB0BRW = 10;                             // Baudrate
    UCB0TBCNT = 0x0001;                       // number of bytes to send
    UCB0I2CSA = 0x40;                         // Slave address
    UCB0CTLW0 &= ~UCSWRST;
    UCB0IE |= UCTXIE;
    __enable_interrupt();
}

void Transmit(void);
void Receive(void);

int main(void)
{
  // Disable the GPIO power-on default high-impedance mode to activate
  // previously con   figured port settings
  PM5CTL0 &= ~LOCKLPM5;
  WDTCTL = WDTPW | WDTHOLD;

  P1OUT &= ~BIT0;                           // Clear P1.0 output latch
  P1DIR |= BIT0;                            // For LED

  i2cInit();

  while (1)
  {
      UCB0CTLW0 |= UCTXSTT;                    // I2C start condition
    __delay_cycles(2000);

  }
}

//pg 843
#pragma vector = USCI_B0_VECTOR
__interrupt void USCI_B0_ISR(void)
{
//  switch(UCB0IV)
//  {
//    case USCI_NONE:          break;         // Vector 0: No interrupts
//    case USCI_I2C_UCALIFG:   break;         // Vector 2: ALIFG
//    case USCI_I2C_UCNACKIFG:                // Vector 4: NACKIFG
//        UCB0CTLW0 |= UCTXSTT;                  // I2C start condition
//      break;
//    case USCI_I2C_UCSTTIFG:  break;         // Vector 6: STTIFG
//    case USCI_I2C_UCSTPIFG:  break;         // Vector 8: STPIFG
//    case USCI_I2C_UCRXIFG3:  break;         // Vector 10: RXIFG3
//    case USCI_I2C_UCTXIFG3:  break;         // Vector 12: TXIFG3
//    case USCI_I2C_UCRXIFG2:  break;         // Vector 14: RXIFG2
//    case USCI_I2C_UCTXIFG2:  break;         // Vector 16: TXIFG2
//    case USCI_I2C_UCRXIFG1:  break;         // Vector 18: RXIFG1
//    case USCI_I2C_UCTXIFG1:  break;         // Vector 20: TXIFG1
//    case USCI_I2C_UCRXIFG0:                 // Vector 22: RXIFG0
//      RXData = UCB0RXBUF;                   // Get RX data
//      __bic_SR_register_on_exit(LPM0_bits); // Exit LPM0
//      break;
//    case USCI_I2C_UCTXIFG0:           // Vector 24: TXIFG0
        UCB0TXBUF = 0xBB;
//        break;
//    case USCI_I2C_UCBCNTIFG:                // Vector 26: BCNTIFG
//      P1OUT ^= BIT0;                        // Toggle LED on P1.0
//      break;
//    case USCI_I2C_UCCLTOIFG: break;         // Vector 28: clock low timeout
//    case USCI_I2C_UCBIT9IFG: break;         // Vector 30: 9th bit
//    default: break;
//  }
}

void Transmit(void){
    while (UCB0CTL1 & UCTXSTP);             // Ensure stop condition got sent
    UCB0CTL1 |= UCTR + UCTXSTT;             // I2C TX, start condition
    __bis_SR_register(CPUOFF + GIE);        // Enter LPM0 w/ interrupts
}
void Receive(void){
   UCB0CTL1 &= ~UCTR ;                  // Clear UCTR
   while (UCB0CTL1 & UCTXSTP);             // Ensure stop condition got sent
   UCB0CTL1 |= UCTXSTT;                    // I2C start condition
   while (UCB0CTL1 & UCTXSTT);             // Start condition sent?
   UCB0CTL1 |= UCTXSTP;                    // I2C stop condition
   __bis_SR_register(CPUOFF + GIE);        // Enter LPM0 w/ interrupts
}
