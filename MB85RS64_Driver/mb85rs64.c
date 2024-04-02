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
        .desiredSpiClock      = 1000000, //1Mhz
        .msbFirst             = EUSCI_B_SPI_MSB_FIRST,
        .clockPhase           = EUSCI_B_SPI_PHASE_DATA_CAPTURED_ONFIRST_CHANGED_ON_NEXT,
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

    //Wait for slave to initialize
    __delay_cycles(100);
//    __bis_SR_register(GIE);
    return MB85RS64_ERR_SUCCESS;
}

static void MB85RS64_Enable(void)
{
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN3);
}

static void MB85RS64_Disable(void)
{
    GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN3);
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
        memset(fram->rxBuffer, 0x00, RX_SIZE);
        fram->txCounter = 0;
        fram->rxCounter = 0;
        if (MB85RS64_GetDeviceID(fram) != MB85RS64_ERR_SUCCESS) {
            return MB85RS64_ERR_FAILURE;
        }

        if (fram->manufactureID == 0x04) {
            printf("Fujitsu\r\n");
        }
    }
    return retVal;
}

static MB85RS64_Error_t  MB85RS64_Transmit(uint8_t const *data, uint8_t const size)
{
    if (data == NULL) {
        return MB85RS64_ERR_FAILURE;
    }
    unsigned int i = 0;
    MB85RS64_Enable();
    for(i=0; i<size; i++) {
        while (!(UCB0IFG & UCTXIFG));
        EUSCI_B_SPI_transmitData(EUSCI_B0_BASE, data[i]);
    }
    MB85RS64_Disable();
    return MB85RS64_ERR_SUCCESS;
}

static MB85RS64_Error_t MB85RS64_TransmitReceive(MB85RS64_t *fram, uint8_t const *txData, uint8_t const txSize)
{
    unsigned int i = 0;
    MB85RS64_Enable();
    for(i=0; i<txSize; i++) {
        while (!(UCB0IFG & UCTXIFG));
        EUSCI_B_SPI_transmitData(EUSCI_B0_BASE, txData[i]);
        fram->rxBuffer[i] = EUSCI_B_SPI_receiveData(EUSCI_B0_BASE);
    }
    MB85RS64_Disable();
    return MB85RS64_ERR_SUCCESS;
}

MB85RS64_Error_t MB85RS64_GetDeviceID(MB85RS64_t *fram)
{
    MB85RS64_Error_t retVal = MB85RS64_ERR_SUCCESS;
    fram->txBuffer[0] = OPCODE_RDID;
    fram->txBuffer[1] = 0x11;   //DUMMY_DATA
    fram->txBuffer[2] = 0x22;   //DUMMY_DATA
    fram->txBuffer[3] = 0x33;   //DUMMY_DATA
    fram->txBuffer[4] = 0x33;   //DUMMY_DATA

    MB85RS64_TransmitReceive(fram, fram->txBuffer, 5);

    if (fram->rxBuffer[2] == 0x7F) {
        // Device with continuation code (0x7F) in their second byte
        // Manu ( 1 byte)  - 0x7F - Product (2 bytes)
        fram->manufactureID = (fram->rxBuffer[1]);
        fram->productID = (fram->rxBuffer[3] << 8) + fram->rxBuffer[4];
    } else {
        // Device without continuation code
        // Manu ( 1 byte)  - Product (2 bytes)
        fram->manufactureID = (fram->rxBuffer[1]);
        fram->productID = (fram->rxBuffer[2] << 8) + fram->rxBuffer[3];
    }

    return retVal;
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
    retVal = MB85RS64_Transmit(&cmd, 1);
    return retVal;
}

MB85RS64_Error_t MB85RS64_Write(MB85RS64_t *fram, uint16_t address, uint8_t const data)
{
    MB85RS64_Error_t retVal = MB85RS64_ERR_SUCCESS;
    fram->txBuffer[0] = OPCODE_WRITE;
    fram->txBuffer[1] = address >> 8;
    fram->txBuffer[2] = address & 0xFF;
    fram->txBuffer[3] = data;
    retVal = MB85RS64_Transmit(fram->txBuffer, 4);
    return retVal;
}

MB85RS64_Error_t MB85RS64_Read(MB85RS64_t *fram, uint16_t address, uint8_t *readData)
{
    if (fram == NULL || readData == NULL) {
        return MB85RS64_ERR_NULL_FAILURE;
    }
    MB85RS64_Error_t retVal = MB85RS64_ERR_SUCCESS;
    uint8_t const DUMMY_DATA = 0x99;
    fram->txBuffer[0] = OPCODE_READ;
    fram->txBuffer[1] = (address >> 8) & 0xFF;
    fram->txBuffer[2] = address & 0xFF;
    fram->txBuffer[3] = DUMMY_DATA;
    retVal = MB85RS64_TransmitReceive(fram, fram->txBuffer, 4);
    *readData = fram->rxBuffer[3];
    return retVal;
}
