#include "mcp3008.h"

MCP3008_t *mcp_g = &(MCP3008_t) {0};

static MCP3008_Error_t SPI_Init(void)
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
        return MCP3008_ERR_SPI_FAILURE;
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
    __bis_SR_register(GIE);
    return MCP3008_ERR_SUCCESS;
}

MCP3008_Error_t MCP3008_Init(MCP3008_t *mcp)
{
    MCP3008_Error_t retVal = MCP3008_ERR_SUCCESS;
    mcp = mcp_g;
    if (mcp == NULL) {
        return MCP3008_ERR_FAILURE;
    } else {

        if (SPI_Init() != MCP3008_ERR_SUCCESS) {
            return retVal;
        }
        mcp->txBuffer = 0;
        memset(mcp->rxBuffer, 0x00, RX_SIZE);
        mcp->txCounter = 0;
        mcp->rxCounter = 0;
    }
    return retVal;
}

static MCP3008_Error_t MCP3008_Enable(MCP3008_t *mcp)
{
    mcp = mcp_g;
    if (mcp == NULL) {
        return MCP3008_ERR_FAILURE;
    } else {
        GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN3);
    }
    return MCP3008_ERR_SUCCESS;
}

static MCP3008_Error_t MCP3008_Disable(MCP3008_t *mcp)
{
    mcp = mcp_g;
    if (mcp == NULL) {
        return MCP3008_ERR_FAILURE;
    } else {
        GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN3);
    }
    return MCP3008_ERR_SUCCESS;
}

static MCP3008_Error_t  MCP3008_TransmitRegister(MCP3008_t *mcp, uint8_t const data)
{
    mcp = mcp_g;
    if (mcp == NULL) {
        return MCP3008_ERR_FAILURE;
    }

    if (data > 0xFF) {
        return MCP3008_ERR_FAILURE;
    }

    mcp->txBuffer = data;
    EUSCI_B_SPI_transmitData(EUSCI_B0_BASE, mcp->txBuffer);
    return MCP3008_ERR_SUCCESS;
}

static MCP3008_Error_t MCP3008_ReceiveRegister(MCP3008_t *mcp)
{
    mcp = mcp_g;
    const uint8_t DUMMY_DATA = 0x69;
    if (mcp == NULL) {
        return MCP3008_ERR_FAILURE;
    }
    EUSCI_B_SPI_transmitData(EUSCI_B0_BASE, DUMMY_DATA); //Dummy data
    EUSCI_B_SPI_receiveData(EUSCI_B0_BASE);
    mcp->rxBuffer[1] = mcp->rxRaw;
    return MCP3008_ERR_SUCCESS;
}

static MCP3008_Error_t MCP3008_TransmitReceiveRegister(MCP3008_t *mcp, uint8_t const data)
{
    mcp = mcp_g;
    if (mcp == NULL) {
        return MCP3008_ERR_FAILURE;
    }

    if (data > 0xFF) {
        return MCP3008_ERR_FAILURE;
    }
    mcp->txBuffer = data;
    EUSCI_B_SPI_transmitData(EUSCI_B0_BASE, mcp->txBuffer);
    EUSCI_B_SPI_receiveData(EUSCI_B0_BASE);
    mcp->rxBuffer[0] = mcp->rxRaw;
    return MCP3008_ERR_SUCCESS;
}

int MCP3008_ReadChannel_1(MCP3008_t *mcp)
{
    mcp = mcp_g;
    const uint8_t CH0_CONFIG = MCP3008_SINGLE_ENDED | MCP3008_CHANNEL_0;
    int adcVal = 0;
    MCP3008_Error_t retVal = MCP3008_ERR_SUCCESS;
    retVal |= MCP3008_Enable(mcp);
    retVal |= MCP3008_TransmitRegister(mcp, MCP3008_START_BIT);
    retVal |= MCP3008_TransmitReceiveRegister(mcp, CH0_CONFIG);
    retVal |= MCP3008_ReceiveRegister(mcp);
    retVal |= MCP3008_Disable(mcp);

    if (retVal != MCP3008_ERR_SUCCESS) {
        return 0;
    }

    uint16_t adcMSB = (mcp->rxBuffer[0] & 0x03) << 8;
    uint8_t adcLSB = mcp->rxBuffer[1];
    adcVal = adcMSB | adcLSB;
    return adcVal;
}

#pragma vector=USCI_B0_VECTOR
__interrupt void USCI_B0_ISR(void) {
    switch(__even_in_range(UCB0IV, USCI_SPI_UCTXIFG)) {
        case USCI_NONE: break;
        case USCI_SPI_UCRXIFG: // RX interrupt
            mcp_g->rxRaw = EUSCI_B_SPI_receiveData(EUSCI_B0_BASE);
            break;
        case USCI_SPI_UCTXIFG: // TX interrupt
            break;
        default: break;
    }
}