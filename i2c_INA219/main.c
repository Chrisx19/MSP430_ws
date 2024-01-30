#include <msp430.h>
#include <string.h>
#include "ina219.h"

#ifdef __MSP430FR4133__
    #define SCL BIT3
    #define SDA BIT2
#elif defined(__MSP430FR5969__)
    #define SCL BIT7
    #define SDA BIT6
#endif


volatile uint8_t txBuffer[2];
volatile uint8_t txByteCount = 0;  // Counter for transmission bytes
volatile uint8_t rxData[2];

void i2cInit(void){
    // Configure GPIO
    P5SEL0 |= SCL;
    P5SEL0 |= SDA;

    // USCI_B0
    UCB0CTLW0 |= UCSWRST;                     // Software reset enabled
    UCB0CTLW0 |= UCSSEL_3;                    // SMCLK
    UCB0CTLW0 |= UCMODE_3 | UCMST;            // I2C mode, Master mode
    UCB0CTLW0 |= UCTR;                        // Transmit Mode
    UCB0CTLW1 |= UCASTP_2;                    // Automatic stop generated

    UCB0BRW = 10;                             // Baudrate
    UCB0TBCNT = sizeof(txBuffer);             // number of bytes to send
    UCB0I2CSA = INA219_ADDRESS;               // Slave address
    UCB0CTLW0 &= ~UCSWRST;
    UCB0IE |= UCTXIE;
    __enable_interrupt();
}

void transmitRegister(uint16_t const config) {
    uint8_t const TX_SIZE = sizeof(txBuffer)/sizeof(uint8_t);
    memset((uint8_t *)txBuffer, 0x00, TX_SIZE);
    txBuffer[0] = (config >> 8) & 0xff;  // MSB
    txBuffer[1] = config & 0xff;         // LSB
    txByteCount = 0;                     // Reset byte counter
    UCB0CTLW0 |= UCTXSTT;                // I2C TX, start condition
}

//not done
void Receive(void){
   UCB0CTL1 &= ~UCTR ;                  // Clear UCTR
   while (UCB0CTL1 & UCTXSTP);             // Ensure stop condition got sent
   UCB0CTL1 |= UCTXSTT;                    // I2C start condition
   while (UCB0CTL1 & UCTXSTT);             // Start condition sent?
   UCB0CTL1 |= UCTXSTP;                    // I2C stop condition
   __bis_SR_register(CPUOFF + GIE);        // Enter LPM0 w/ interrupts
}

int main(void)
{
  // Disable the GPIO power-on default high-impedance mode to activate
  // previously con   figured port settings
  PM5CTL0 &= ~LOCKLPM5;
  WDTCTL = WDTPW | WDTHOLD;

  P1OUT &= ~BIT0;                           // Clear P1.0 output latch
  P1DIR |= BIT0;                            // For LED

  i2cInit();
  uint16_t const config =   SHUNT_BUS_CONT | SADC_12BIT_532US | BADC_12BIT_532US |
                                      PG_GAIN_1_40mV | BRNG_FSR_16V;

  uint16_t const calibrationValue = 8192;

  transmitRegister(config);
  transmitRegister(calibrationValue);

  while (1)
  {


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
        UCB0CTLW0 |= UCTXSTT;               // I2C start condition
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
      break;
    case USCI_I2C_UCTXIFG0:                 // Vector 24: TXIFG0
        P1OUT ^= BIT0;
        if (txByteCount <= 2) {
            while (UCB0CTL1 & UCTXSTP);             // Ensure stop condition was sent
            UCB0TXBUF = txBuffer[txByteCount++];    // Send first byte
            while (!(UCB0IFG & UCTXIFG));           // Wait for TX buffer ready
            UCB0TXBUF = txBuffer[txByteCount];      // Send second byte
        } else {
            while (!(UCB0IFG & UCTXIFG));           // Wait for transmission complete
            UCB0CTLW0 |= UCTXSTP;                   // Send I2C stop condition
            UCB0IFG &= ~UCTXIFG;                    // Clear TX flag
        }
        break;
    case USCI_I2C_UCBCNTIFG:                // Vector 26: BCNTIFG
      P1OUT ^= BIT0;                        // Toggle LED on P1.0
      break;
    case USCI_I2C_UCCLTOIFG: break;         // Vector 28: clock low timeout
    case USCI_I2C_UCBIT9IFG: break;         // Vector 30: 9th bit
    default: break;
  }
}
