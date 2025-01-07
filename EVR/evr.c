/*
 * evr.c
 *
 *  Created on: Dec 30, 2024
 *      Author: clazo
 */
#include "evr.h"
#include "msp430.h"
#include "driverlib.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdbool.h>

#define UART_CMD_SIZE 32
#define UART_TRANSMIT_SIZE 128

typedef struct {
    volatile char uartBuffer[UART_CMD_SIZE];
    volatile uint16_t uartIndex;
    volatile bool commandReady;
} UART_ISR_Context_t;

static volatile UART_ISR_Context_t g_uartIsrContext = {0};

EVR_Status EVR_Init(EVR_t *evr)
{
    // Configure UART pins (P2.6 -> TXD, P2.5 -> RXD)
    GPIO_setAsPeripheralModuleFunctionInputPin(
        GPIO_PORT_P2,
        GPIO_PIN6 | GPIO_PIN5,
        GPIO_SECONDARY_MODULE_FUNCTION
    );

    // UART configuration parameters set to 115200 baud
    // Datasheet to configure baudrate and UART
    // https://www.ti.com/lit/ug/slau367p/slau367p.pdf?ts=1706206110916&ref_url=https%253A%252F%252Fwww.ti.com%252Fproduct%252FMSP430FR5969
    EUSCI_A_UART_initParam uartConfig = {
        EUSCI_A_UART_CLOCKSOURCE_SMCLK,    // Clock source: SMCLK
        8,                                 // Clock prescaler
        0,                                 // First modulation stage
        0xD6,                              // Second modulation stage; 0.6667 (Pg 779)
        EUSCI_A_UART_NO_PARITY,            // No parity
        EUSCI_A_UART_LSB_FIRST,            // LSB first
        EUSCI_A_UART_ONE_STOP_BIT,         // One stop bit
        EUSCI_A_UART_MODE,                 // UART mode
    };

    if (EUSCI_A_UART_init(EUSCI_A1_BASE, &uartConfig) == STATUS_FAIL) {
        return EVR_ERROR;
    }

    // Enable UART module and interrupts
    EUSCI_A_UART_enable(EUSCI_A1_BASE);
    EUSCI_A_UART_clearInterrupt(EUSCI_A1_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
    EUSCI_A_UART_enableInterrupt(EUSCI_A1_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);

    __enable_interrupt();

    return EVR_SUCCESS;
}

EVR_Status EVR(const char *format, ...) 
{
    if (format == NULL) {
        return EVR_NULL_ERROR;
    }

    EVR_Status ret = EVR_SUCCESS;
    char buffer[UART_TRANSMIT_SIZE] = {0};
    va_list args;

    // Start variadic arguments processing
    va_start(args, format);

    // Format the string into the buffer
    if (vsnprintf(buffer, UART_TRANSMIT_SIZE, format, args) >= UART_TRANSMIT_SIZE) {
        ret = EVR_ERROR; // Buffer overflow error
    }

    // End variadic arguments processing
    va_end(args);

    if (ret == EVR_SUCCESS) {
        const char *message = buffer;
        while (*message) {
            EUSCI_A_UART_transmitData(EUSCI_A1_BASE, *message++);
            while (!EUSCI_A_UART_getInterruptStatus(EUSCI_A1_BASE, EUSCI_A_UART_TRANSMIT_INTERRUPT_FLAG));
        }
    }

    return ret;
}

static void resetBuffer() 
{
    memset((char *)g_uartIsrContext.uartBuffer, 0, UART_CMD_SIZE);
    g_uartIsrContext.uartIndex = 0;
}

EVR_Status EVR_GetCommand(char *outBuffer)
{
    if (outBuffer == NULL) {
        return EVR_NULL_ERROR;
    }

    EVR_Status ret = EVR_SUCCESS;

    if (g_uartIsrContext.commandReady == true) {
        memcpy((void *)outBuffer, (const void *)g_uartIsrContext.uartBuffer, UART_CMD_SIZE);
        (void) resetBuffer();
        g_uartIsrContext.commandReady = false;
    } 

    return ret;
}

// UART ISR Pg 784
// https://www.ti.com/lit/ug/slau367p/slau367p.pdf?ts=1706206110916&ref_url=https%253A%252F%252Fwww.ti.com%252Fproduct%252FMSP430FR5969
#pragma vector=USCI_A1_VECTOR
__interrupt void USCI_A1_ISR(void) {
    switch (__even_in_range(UCA1IV,18)) {
        case USCI_NONE:
            break;
        case USCI_UART_UCRXIFG: /* Receive ISR */
        {
            char c = EUSCI_A_UART_receiveData(EUSCI_A1_BASE);
            if (c == '\n' || c == '\r') {
                g_uartIsrContext.uartBuffer[g_uartIsrContext.uartIndex] = '\0';
                g_uartIsrContext.commandReady = true; 
            } else {
                g_uartIsrContext.uartBuffer[g_uartIsrContext.uartIndex++] = c;
            }
            break;
        }     
        case USCI_UART_UCTXIFG: /* Transmit ISR */
            break;
        case USCI_UART_UCSTTIFG:
            break;
        case USCI_UART_UCTXCPTIFG:
            break;
        default:
            break;
    }
}
