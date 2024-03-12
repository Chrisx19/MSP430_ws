#include "ina219.h"

INA219_config_t *ina = &(INA219_config_t) {
    .txBuffer = { 0x00 },
    .rxBuffer = { 0x00 },
    .txByteCount = 0,
    .rxByteCount = 0,
    .CALIBRATION_VALUE = 8192
};

static uint16_t getCurrent_raw()
{
  uint16_t rawData = 0x0000;
  INA219TransmitRegister(INA219_REG_CALIBRATION, ina->CALIBRATION_VALUE);
  __delay_cycles(50);
  INA219ReceiveRegister(INA219_REG_CURRENT);
  rawData = ((ina->rxBuffer[0] << 8) | ina->rxBuffer[1]);
  return rawData;
}

float getCurrent_mA()
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

static void setCalibration_16V_400mA()
{
  INA219TransmitRegister(INA219_REG_CALIBRATION, ina->CALIBRATION_VALUE);
  uint16_t const CONFIG = SHUNT_BUS_CONT | SADC_12BIT_532US | BADC_12BIT_532US |
                          PG_GAIN_1_40mV | BRNG_FSR_16V;
  INA219TransmitRegister(INA219_REG_CONFIG, CONFIG);
}

void INA_Init()
{
    setCalibration_16V_400mA();
}

void i2cInit()
{
    /*
     * Disable the GPIO power-on default high-impedance mode to activate
     * previously configured port settings
     */
    PMM_unlockLPM5();
    // Configure Pins for I2C
    /*
    * Select Port 5
    * Set Pin 2, 3 to input with function, (UCB0SIMO/UCB0SDA, UCB0SOMI/UCB0SCL).
    */
    GPIO_setAsPeripheralModuleFunctionInputPin(
        GPIO_PORT_P5,
        GPIO_PIN2 + GPIO_PIN3,
        GPIO_PRIMARY_MODULE_FUNCTION
    );

    EUSCI_B_I2C_initMasterParam *param = &(EUSCI_B_I2C_initMasterParam) {
        .selectClockSource = EUSCI_B_I2C_CLOCKSOURCE_SMCLK,
        .i2cClk = CS_getSMCLK(),
        .dataRate = EUSCI_B_I2C_SET_DATA_RATE_100KBPS,
        .byteCounterThreshold = 3,
        .autoSTOPGeneration = EUSCI_B_I2C_SEND_STOP_AUTOMATICALLY_ON_BYTECOUNT_THRESHOLD
    };

    EUSCI_B_I2C_initMaster(EUSCI_B0_BASE, param);

    EUSCI_B_I2C_setSlaveAddress(EUSCI_B0_BASE, INA219_ADDRESS);

    EUSCI_B_I2C_enable(EUSCI_B0_BASE); //Enables the I2C in order the ISR to work

    EUSCI_B_I2C_clearInterrupt( EUSCI_B0_BASE,
                                EUSCI_B_I2C_RECEIVE_INTERRUPT0);
    EUSCI_B_I2C_enableInterrupt(EUSCI_B0_BASE,
                                EUSCI_B_I2C_RECEIVE_INTERRUPT0);

    EUSCI_B_I2C_clearInterrupt( EUSCI_B0_BASE,
                                EUSCI_B_I2C_TRANSMIT_INTERRUPT0);
    EUSCI_B_I2C_enableInterrupt(EUSCI_B0_BASE,
                                EUSCI_B_I2C_TRANSMIT_INTERRUPT0);
}

void INA219TransmitRegister(uint8_t const readRegister, uint16_t const config)
{
    //Set Master in transmit mode
    EUSCI_B_I2C_setMode(EUSCI_B0_BASE, EUSCI_B_I2C_TRANSMIT_MODE);
    memset((uint8_t *)ina->txBuffer, 0x00, 2);
    ina->txBuffer[0] = (config >> 8) & 0xff;  // MSB
    ina->txBuffer[1] = config & 0xff;         // LSB
    ina->txByteCount = 0;
    EUSCI_B_I2C_masterSendMultiByteStart(EUSCI_B0_BASE, readRegister);
}

void INA219ReceiveRegister(uint8_t const readRegister)
{
    //Set Master in transmit mode
    EUSCI_B_I2C_masterSendSingleByte(EUSCI_B0_BASE, readRegister);
    while (EUSCI_B_I2C_isBusBusy(EUSCI_B0_BASE)) ;
    //Delay between changes
    __delay_cycles(50);
    //Receive Area
    HWREG16(EUSCI_B0_BASE + OFS_UCBxTBCNT) = 2; //Changed to 2 since want to only Rx 2 bytes
    ina->rxByteCount = 0;
    EUSCI_B_I2C_masterReceiveStart(EUSCI_B0_BASE);
}

#pragma vector=USCI_B0_VECTOR
__interrupt
void USCIB0_ISR(void)
{
    switch (__even_in_range(UCB0IV, USCI_I2C_UCBIT9IFG))
    {
        case USCI_NONE:             // No interrupts break;
            break;
        case USCI_I2C_UCALIFG:      // Arbitration lost
            break;
        case USCI_I2C_UCNACKIFG:    // NAK received (master only)
            break;
        case USCI_I2C_UCSTTIFG:     // START condition detected with own address (slave mode only)
            break;
        case USCI_I2C_UCSTPIFG:     // STOP condition detected (master & slave mode)
            break;
        case USCI_I2C_UCRXIFG3:     // RXIFG3
            break;
        case USCI_I2C_UCTXIFG3:     // TXIFG3
            break;
        case USCI_I2C_UCRXIFG2:     // RXIFG2
            break;
        case USCI_I2C_UCTXIFG2:     // TXIFG2
            break;
        case USCI_I2C_UCRXIFG1:     // RXIFG1
            break;
        case USCI_I2C_UCTXIFG1:     // TXIFG1
            break;
        case USCI_I2C_UCRXIFG0:     // RXIFG0
            ina->rxBuffer[ina->rxByteCount] = EUSCI_B_I2C_masterReceiveMultiByteNext(EUSCI_B0_BASE);
            ina->rxByteCount++;
            if (ina->rxByteCount >= RX_SIZE) {
                ina->rxByteCount = 0;
            }
            break; // Vector 24: RXIFG0 break;
        case USCI_I2C_UCTXIFG0:     // TXIFG0
            if(ina->txByteCount < TX_SIZE) {
                EUSCI_B_I2C_masterSendMultiByteNext(EUSCI_B0_BASE, ina->txBuffer[ina->txByteCount]);
                ina->txByteCount++;
            } else {
                ina->txByteCount = 0;
            }
            break;
        case USCI_I2C_UCBCNTIFG:    // Byte count limit reached (UCBxTBCNT)
            break;
        case USCI_I2C_UCCLTOIFG:    // Clock low timeout - clock held low too long
            break;
        case USCI_I2C_UCBIT9IFG:    // Generated on 9th bit of a transmit (for debugging)
            break;
        default:
            break;
    }
}
