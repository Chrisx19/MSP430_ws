#include "ina219.h"

void INA219TransmitRegister(INA219_config_t *ina, uint8_t const reg, uint16_t const config,
                      uint8_t const size)
{
    //Set Master in transmit mode
    EUSCI_B_I2C_setMode(EUSCI_B0_BASE, EUSCI_B_I2C_TRANSMIT_MODE);
    //Enable I2C Module to start operations
    EUSCI_B_I2C_enable(EUSCI_B0_BASE);
}

void INA219ReceiveRegister(uint8_t const readRegister)
{
    //Set Master in receive mode
    EUSCI_B_I2C_setMode(EUSCI_B0_BASE, EUSCI_B_I2C_RECEIVE_MODE);
    //Enable I2C Module to start operations
    EUSCI_B_I2C_enable(EUSCI_B0_BASE);
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

void i2cInit(void)
{
    EUSCI_B_I2C_initMasterParam param = {0};
    param.selectClockSource = EUSCI_B_I2C_CLOCKSOURCE_SMCLK;
    param.i2cClk = CS_getSMCLK();
    param.dataRate = EUSCI_B_I2C_SET_DATA_RATE_100KBPS;
    param.byteCounterThreshold = 2;
    param.autoSTOPGeneration = EUSCI_B_I2C_SEND_STOP_AUTOMATICALLY_ON_BYTECOUNT_THRESHOLD;
    EUSCI_B_I2C_initMaster(EUSCI_B0_BASE, &param);

    //Specify slave address
    EUSCI_B_I2C_setSlaveAddress(EUSCI_B0_BASE, INA219_ADDRESS);

    // Configure Pins for I2C
    //Set P1.6 and P1.7 as Secondary Module Function Input.
    /*
    * Select Port 1
    * Set Pin 6, 7 to input Secondary Module Function, (UCB0SIMO/UCB0SDA, UCB0SOMI/UCB0SCL).
    */
    GPIO_setAsPeripheralModuleFunctionInputPin(
        GPIO_PORT_P1,
        GPIO_PIN6 + GPIO_PIN7,
        GPIO_SECONDARY_MODULE_FUNCTION
    );
}

