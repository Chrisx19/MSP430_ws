#include <mb85rs64.h>

MB85RS64_t *fram_g = &(MB85RS64_t) {0};

static MB85RS64_Error_t FRAM_SPI_Init(void)
{
    PMM_unlockLPM5(); //unlock all pins

    //Chip Select
    GPIO_setAsOutputPin(
        GPIO_PORT_P1,
        GPIO_PIN3
    );

    // Chip Select will be high in idle
    GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN3);

    //Set DCO frequency to max DCO setting
    CS_setDCOFreq(CS_DCORSEL_0,CS_DCOFSEL_3);
    //Select DCO as the clock source for SMCLK with no frequency divider
    CS_initClockSignal(CS_SMCLK,CS_DCOCLK_SELECT,CS_CLOCK_DIVIDER_1);

    // SPI_Clock
    GPIO_setAsPeripheralModuleFunctionInputPin(
        GPIO_PORT_P2,
        GPIO_PIN2,
        GPIO_SECONDARY_MODULE_FUNCTION
    );

    /*
    * Select Port 1
    * Set Pin 6, 7 to input Secondary Module Function, (UCB0TXD/UCB0SIMO, UCB0RXD/UCB0SOMI).
    */
    GPIO_setAsPeripheralModuleFunctionInputPin(
        GPIO_PORT_P1,
        GPIO_PIN6 + GPIO_PIN7,
        GPIO_SECONDARY_MODULE_FUNCTION
    );

    EUSCI_B_SPI_initMasterParam *param = &(EUSCI_B_SPI_initMasterParam) {
        .selectClockSource    = EUSCI_B_SPI_CLOCKSOURCE_SMCLK,
        .clockSourceFrequency = CS_getSMCLK(),
        .desiredSpiClock      = 500000,
        .msbFirst             = EUSCI_B_SPI_MSB_FIRST,
        .clockPhase           = EUSCI_B_SPI_PHASE_DATA_CHANGED_ONFIRST_CAPTURED_ON_NEXT,
        .clockPolarity        = EUSCI_B_SPI_CLOCKPOLARITY_INACTIVITY_LOW,
        .spiMode              = EUSCI_B_SPI_4PIN_UCxSTE_ACTIVE_LOW
    };

    if (param == NULL) {
        return MB85RS64_ERR_SPI_FAILURE;
    }

    EUSCI_B_SPI_initMaster(EUSCI_B0_BASE, param);

    EUSCI_B_SPI_select4PinFunctionality(EUSCI_B0_BASE,
                                        EUSCI_B_SPI_ENABLE_SIGNAL_FOR_4WIRE_SLAVE);

    //Enable SPI module
    EUSCI_B_SPI_enable(EUSCI_B0_BASE);

    EUSCI_B_SPI_clearInterrupt(EUSCI_B0_BASE,
                               EUSCI_B_SPI_RECEIVE_INTERRUPT);
    // Enable USCI_B0 RX interrupt
    EUSCI_B_SPI_enableInterrupt(EUSCI_B0_BASE,
                                EUSCI_B_SPI_RECEIVE_INTERRUPT);

    EUSCI_B_SPI_clearInterrupt(EUSCI_B0_BASE,
                               EUSCI_B_SPI_TRANSMIT_INTERRUPT);
    // Enable USCI_B0 TX interrupt
    EUSCI_B_SPI_enableInterrupt(EUSCI_B0_BASE,
                                EUSCI_B_SPI_TRANSMIT_INTERRUPT);

    //Wait for slave to initialize
    __delay_cycles(100);
    return MB85RS64_ERR_SUCCESS;
}

MB85RS64_Error_t MB85RS64_Init(MB85RS64_t *fram)
{
    MB85RS64_Error_t retVal = MB85RS64_ERR_SUCCESS;
    fram = fram_g;
    if (fram == NULL) {
        return MB85RS64_ERR_FAILURE;
    } else {

        if (FRAM_SPI_Init() != MB85RS64_ERR_SUCCESS) {
            return retVal;
        }
        memset(fram->txBuffer, 0x00, TX_SIZE);
        fram->rxRaw = 0;
        fram->txCounter = 0;
        fram->rxCounter = 0;
    }
    return retVal;
}

static void MB85RS64_Enable(void)
{
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN3);
}

static void MB85RS64_Disable(void)
{
    GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN3);
}

static MB85RS64_Error_t  MB85RS64_Transmit(uint8_t const *data, uint8_t const size)
{
    if (data == NULL) {
        return MB85RS64_ERR_FAILURE;
    }
    unsigned int i = 0;
    MB85RS64_Enable();
    for(i=0; i<size; i++) {
        EUSCI_B_SPI_transmitData(EUSCI_B0_BASE, data[i]);
    }
    __delay_cycles(1);
    MB85RS64_Disable();
    return MB85RS64_ERR_SUCCESS;
}

MB85RS64_Error_t MB85RS64_WriteEnableLatch(bool latchEn)
{
    MB85RS64_Error_t retVal = MB85RS64_ERR_SUCCESS;

    uint8_t cmd = 0;
    if (latchEn == true) {
        cmd = OPCODE_WREN;
    } else {
        cmd = OPCODE_WRDI;
    }

    retVal |= MB85RS64_Transmit(&cmd, 1);
    return retVal;
}

MB85RS64_Error_t MB85RS64_Write(MB85RS64_t *fram, uint16_t address, uint8_t const data)
{
    MB85RS64_Error_t retVal = MB85RS64_ERR_SUCCESS;
    fram->txBuffer[0] = OPCODE_WRITE;
    fram->txBuffer[1] = (address >> 8) & 0xFF;
    fram->txBuffer[2] = address & 0xFF;
    fram->txBuffer[3] = data;
    retVal |= MB85RS64_Transmit(fram->txBuffer, 4);
    return retVal;
}

#pragma vector=USCI_B0_VECTOR
__interrupt void USCI_B0_ISR(void) {
    switch(__even_in_range(UCB0IV, USCI_SPI_UCTXIFG)) {
        case USCI_NONE: break;
        case USCI_SPI_UCRXIFG: // RX interrupt
            break;
        case USCI_SPI_UCTXIFG: // TX interrupt
            break;
        default: break;
    }
}
