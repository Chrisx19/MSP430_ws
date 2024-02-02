#include <msp430.h>
#include "ina219.h"
#include "init.h"

uint8_t receiveFlag_g = 0;
uint8_t slavePointerReg_g = 0;
int l;

float currentmA_g = 0;

INA219_config_t *ina = &(INA219_config_t) {
    .txBuffer = {0x00},
    .rxBuffer = {0x00},
    .CALIBRATION_VALUE = 8192,
};

int main(void)
{
  P1OUT &= ~BIT0;  // Clear P1.0 output latch
  P1DIR |= BIT0;   // For LED

  disableInit();
  i2cInit();
  __enable_interrupt();  // Enables global interrupts
  INA_Init(ina);

  while (1) {
    currentmA_g = getCurrent_mA(ina);
    P1OUT ^= BIT0;
    __delay_cycles(2000000);
  }
}

// pg 843
#pragma vector = USCI_B0_VECTOR
__interrupt void USCI_B0_ISR(void)
{
  switch (UCB0IV) {
    case USCI_I2C_UCRXIFG0:  // Vector 22: RXIFG0
      ina->rxBuffer[ina->rxByteCount++] = UCB0RXBUF;
      if (ina->rxByteCount >= RX_SIZE) {
          ina->rxByteCount = 0;
      }
      break;
    // TODO: Need to update this
    case USCI_I2C_UCTXIFG0:  // Vector 24: TXIFG0
      if (receiveFlag_g == 0) {
        if (ina->txByteCount < TX_SIZE) {
          while (UCB0CTLW0 & UCTXSTP);               // Ensure stop condition was sent

          UCB0TXBUF = ina->txBuffer[ina->txByteCount++];  // Send first byte
          for (l=0;l < (TX_SIZE - 1); l++) {
              while ((UCB0IFG & UCTXIFG) == 0);
              UCB0TXBUF = ina->txBuffer[ina->txByteCount++];
          }
        }
      } else {
        while (UCB0CTLW0 & UCTXSTP);                // Ensure stop condition was sent
        UCB0TXBUF = slavePointerReg_g;              // Send the register address
        receiveFlag_g = 0;
      }
      break;
    default:
      break;
  }
}
