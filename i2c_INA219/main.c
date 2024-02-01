#include <msp430.h>
#include <string.h>
#include "init.h"
#include "ina219.h"

#define TX_SIZE 3
#define RX_SIZE 2
#define CURRENT_THRESHOLD 30

#ifdef __MSP430FR4133__
    #define SCL_0 BIT3
    #define SDA_0 BIT2
#elif defined(__MSP430FR5969__)
    #define SCL_0 BIT7
    #define SDA_0 BIT6
#endif

typedef struct {
    volatile uint8_t txBuffer[TX_SIZE];
    volatile uint8_t rxBuffer[RX_SIZE];
    uint16_t const CALIBRATION_VALUE; //0x2000
}INA219_CONFIG_t;

uint8_t txByteCount = 0;
uint8_t rxByteCount = 0;
uint8_t slavePointerReg_g = 0;
uint8_t receiveFlag_g = 0;

float currentmA_g = 0;

INA219_CONFIG_t *ina = &(INA219_CONFIG_t){
    .txBuffer = {0x00},
    .rxBuffer = {0x00},
    .CALIBRATION_VALUE = 8192,
};

void i2cInit(void);
void INA219TransmitRegister(uint8_t const reg, uint16_t const config, uint8_t const size);
void INA219ReceiveRegister(uint8_t const readRegister);
void setCalibration_16V_400mA(void);

uint16_t getCurrent_raw(void)
{
    uint16_t rawData_g = 0x0000;
    TransmitRegister(INA219_REG_CALIBRATION, ina->CALIBRATION_VALUE, TX_SIZE);

    ReceiveRegister(INA219_REG_CURRENT);
    rawData_g = ((ina->rxBuffer[0] << 8) | ina->rxBuffer[1]);
    return rawData_g;
}

float getCurrent_mA(void)
{
    uint32_t const CURRENT_DIVIDER_MILLI_AMP = 20;
    float valueDec = getCurrent_raw();

    if (valueDec < CURRENT_THRESHOLD) {
        valueDec = 0;
    } else {
        valueDec /= CURRENT_DIVIDER_MILLI_AMP;
    }
    return valueDec;
}

int main(void)
{
  P1OUT &= ~BIT0;                           // Clear P1.0 output latch
  P1DIR |= BIT0;                            // For LED

  disableInit();
  i2cInit();
  __enable_interrupt();                // Enables global interrupts

  setCalibration_16V_400mA();

  while (1)
  {
      currentmA_g = getCurrent_mA();
      P1OUT ^= BIT0;
      __delay_cycles(2000000);
  }
}

/* pg 843 */
#pragma vector = USCI_B0_VECTOR
__interrupt void USCI_B0_ISR(void)
{
  switch(UCB0IV)
  {
    case USCI_I2C_UCRXIFG0:                 // Vector 22: RXIFG0
        ina->rxBuffer[rxByteCount++] = UCB0RXBUF;
        if ( rxByteCount >= RX_SIZE ) {
            rxByteCount = 0;
        }
        break;
    /* TODO: Need to update this  */
    case USCI_I2C_UCTXIFG0:                 // Vector 24: TXIFG0
        if (receiveFlag_g == 0) {
            if (txByteCount < TX_SIZE) {
                while (UCB0CTLW0 & UCTXSTP);         // Ensure stop condition was sent
                UCB0TXBUF = ina->txBuffer[txByteCount++]; // Send first byte
                while ((UCB0IFG & UCTXIFG) == 0);    // Wait for TX buffer ready
                UCB0TXBUF = ina->txBuffer[txByteCount++]; // Send second byte
                while ((UCB0IFG & UCTXIFG) == 0);    // Wait for TX buffer ready
                UCB0TXBUF = ina->txBuffer[txByteCount];   // Send third byte
            }
        } else {
            while (UCB0CTLW0 & UCTXSTP);             // Ensure stop condition was sent
            UCB0TXBUF = slavePointerReg_g;             // Send the register address
            receiveFlag_g = 0;
        }
        break;
    default: break;
  }
}

void INA219TransmitRegister(uint8_t const reg, uint16_t const config, uint8_t const size)
{
    receiveFlag_g = 0;
    UCB0CTLW0 |= UCTR;                   // Transmit Mode
    memset((uint8_t *)ina->txBuffer, 0x00, size);

    UCB0TBCNT = size;                   // number of bytes to send
    ina->txBuffer[0] = reg;
    ina->txBuffer[1] = (config >> 8) & 0xff;  // MSB
    ina->txBuffer[2] = config & 0xff;         // LSB
    txByteCount = 0;                     // Reset byte counter
    UCB0CTLW0 |= UCTXSTT;                // I2C TX, start condition
    __delay_cycles(100);
}

void INA219ReceiveRegister(uint8_t const readRegister)
{
    receiveFlag_g = 1;
    UCB0CTLW0 |= UCTR;
    UCB0TBCNT = 1;
    slavePointerReg_g = readRegister;
    UCB0CTLW0 |= UCTXSTT;
    __delay_cycles(100);
//==========================================================================================//
    UCB0CTLW0 &= ~UCTR;
    UCB0TBCNT = 2;
    UCB0CTLW0 |= UCTXSTT;

    while ((UCB0IFG & UCSTPIFG) == 0);
    UCB0IFG &= ~UCSTPIFG;
}

/* pg 845 */
void i2cInit(void)
{
    /* Configure GPIO */
#ifdef __MSP430FR4133__
    P5SEL0 |= SCL_0;
    P5SEL0 |= SDA_0;
#elif defined(__MSP430FR5969__)
    P1SEL1 |= SCL_0;
    P1SEL1 |= SDA_0;
#endif
    /* USCI_B0 */
    UCB0CTLW0 |= UCSWRST;                // Software reset enabled
    UCB0CTLW0 |= UCSSEL_3;               // SMCLK
    UCB0BRW = 10;                        // Baudrate Prescaler
    UCB0CTLW0 |= UCMODE_3 | UCMST;       // I2C mode, Master mode
    UCB0I2CSA = INA219_ADDRESS;          // Slave address

    UCB0TBCNT = 1;                       // number of bytes to send
    UCB0CTLW1 |= UCASTP_2;               // Automatic stop generated

    UCB0CTLW0 &= ~UCSWRST;               // Software reset disabled

    UCB0IE |= UCTXIE | UCRXIE;           // Enable ISR for Tx, and Rx; This goes to ISR routines
}

void setCalibration_16V_400mA(void)
{
    TransmitRegister(INA219_REG_CALIBRATION, ina->CALIBRATION_VALUE, TX_SIZE);

    uint16_t const CONFIG =   SHUNT_BUS_CONT | SADC_12BIT_532US | BADC_12BIT_532US |
                                        PG_GAIN_1_40mV | BRNG_FSR_16V;
    TransmitRegister(INA219_REG_CONFIG, CONFIG, TX_SIZE);
}
