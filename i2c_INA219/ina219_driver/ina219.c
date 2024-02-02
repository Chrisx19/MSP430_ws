#include "ina219.h"

extern uint8_t receiveFlag_g;
extern uint8_t slavePointerReg_g;

void INA219TransmitRegister(INA219_config_t *ina, uint8_t const reg, uint16_t const config,
                      uint8_t const size)
{
    receiveFlag_g = 0;
    UCB0CTLW0 |= UCTR;  // Transmit Mode
    memset((uint8_t *)ina->txBuffer, 0x00, size);

    UCB0TBCNT = size;  // number of bytes to send
    ina->txBuffer[0] = reg;
    ina->txBuffer[1] = (config >> 8) & 0xff;  // MSB
    ina->txBuffer[2] = config & 0xff;         // LSB
    ina->txByteCount = 0;                          // Reset byte counter
    UCB0CTLW0 |= UCTXSTT;                     // I2C TX, start condition
    __delay_cycles(100);
}

void INA219ReceiveRegister(uint8_t readRegister)
{
    receiveFlag_g = 1;
    UCB0CTLW0 |= UCTR;
    UCB0TBCNT = 1;
    slavePointerReg_g = readRegister;
    UCB0CTLW0 |= UCTXSTT;
    __delay_cycles(100);

    UCB0CTLW0 &= ~UCTR; //flip mode to Rx
    UCB0TBCNT = 2;
    UCB0CTLW0 |= UCTXSTT;

    while ((UCB0IFG & UCSTPIFG) == 0);
    UCB0IFG &= ~UCSTPIFG;
}

static uint16_t getCurrent_raw(INA219_config_t *ina)
{
  uint16_t rawData_g = 0x0000;
  INA219TransmitRegister(ina, INA219_REG_CALIBRATION, ina->CALIBRATION_VALUE, TX_SIZE);

  INA219ReceiveRegister(INA219_REG_CURRENT);
  rawData_g = ((ina->rxBuffer[0] << 8) | ina->rxBuffer[1]);
  return rawData_g;
}

float getCurrent_mA(INA219_config_t *ina)
{
  uint32_t const CURRENT_DIVIDER_MILLI_AMP = 20;
  float valueDec = getCurrent_raw(ina);

  if (valueDec < CURRENT_THRESHOLD) {
    valueDec = 0;
  } else {
    valueDec /= CURRENT_DIVIDER_MILLI_AMP;
  }
  return valueDec;
}

static void setCalibration_16V_400mA(INA219_config_t *ina)
{
  INA219TransmitRegister(ina, INA219_REG_CALIBRATION, ina->CALIBRATION_VALUE, TX_SIZE);

  uint16_t const CONFIG = SHUNT_BUS_CONT | SADC_12BIT_532US | BADC_12BIT_532US |
                          PG_GAIN_1_40mV | BRNG_FSR_16V;
  INA219TransmitRegister(ina, INA219_REG_CONFIG, CONFIG, TX_SIZE);
}

void INA_Init(INA219_config_t *ina)
{
    ina->txByteCount = 0;
    ina->rxByteCount = 0;

    setCalibration_16V_400mA(ina);
}

// pg 845
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
  UCB0CTLW0 |= UCSWRST;           // Software reset enabled
  UCB0CTLW0 |= UCSSEL_3;          // SMCLK
  UCB0BRW = 10;                   // Baudrate Prescaler
  UCB0CTLW0 |= UCMODE_3 | UCMST;  // I2C mode, Master mode
  UCB0I2CSA = INA219_ADDRESS;     // Slave address

  UCB0TBCNT = 1;          // number of bytes to send
  UCB0CTLW1 |= UCASTP_2;  // Automatic stop generated

  UCB0CTLW0 &= ~UCSWRST;  // Software reset disabled

  UCB0IE |= UCTXIE | UCRXIE;  // Enable ISR for Tx, and Rx; This goes to ISR routines
}


