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

static EVR_Status User_Interface_UART_Init(EUSCI_A_UART_initParam *uartConfig, uint16_t baseChannel)
{
    if (EUSCI_A_UART_init(baseChannel, uartConfig) == STATUS_FAIL) {
        return EVR_ERROR;
    }

    EUSCI_A_UART_enable(baseChannel);
    EUSCI_A_UART_clearInterrupt(baseChannel, EUSCI_A_UART_RECEIVE_INTERRUPT);
    EUSCI_A_UART_enableInterrupt(baseChannel, EUSCI_A_UART_RECEIVE_INTERRUPT);

    return EVR_SUCCESS;
}

static EVR_Status Debug_Interface_UART_Init(EUSCI_A_UART_initParam *uartConfig, uint16_t baseChannel)
{
    if (EUSCI_A_UART_init(baseChannel, uartConfig) == STATUS_FAIL) {
        return EVR_ERROR;
    }

    EUSCI_A_UART_enable(baseChannel);

    return EVR_SUCCESS;
}

EVR_Status EVR_Init()
{
    // Configure user & debug UART pins
    GPIO_setAsPeripheralModuleFunctionInputPin(
        GPIO_PORT_P2,
        GPIO_PIN0 | GPIO_PIN1 |GPIO_PIN6 | GPIO_PIN5,
        GPIO_SECONDARY_MODULE_FUNCTION
    );

    // UART configuration parameters set to 115200 baud
    // Datasheet to configure baudrate and UART
    // https://www.ti.com/lit/ug/slau367p/slau367p.pdf?ts=1706206110916&ref_url=https%253A%252F%252Fwww.ti.com%252Fproduct%252FMSP430FR5969
    EUSCI_A_UART_initParam uartUserConfig = {
        .selectClockSource = EUSCI_A_UART_CLOCKSOURCE_SMCLK,    // Clock source: SMCLK
        .clockPrescalar = 8,                                    // Clock prescaler
        .firstModReg = 0,                                       // First modulation stage
        .secondModReg = 0xD6,                                   // Second modulation stage; 0.6667 (Pg 779)
        .parity = EUSCI_A_UART_NO_PARITY,                       // No parity
        .msborLsbFirst = EUSCI_A_UART_LSB_FIRST,                // LSB first
        .numberofStopBits = EUSCI_A_UART_ONE_STOP_BIT,          // One stop bit
        .uartMode = EUSCI_A_UART_MODE,                          // UART mode
    };

    if ( User_Interface_UART_Init(&uartUserConfig, EUSCI_A1_BASE) != EVR_SUCCESS ) {
        return EVR_ERROR;
    }

    if ( Debug_Interface_UART_Init(&uartUserConfig, EUSCI_A0_BASE) != EVR_SUCCESS ) {
        return EVR_ERROR;
    }

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

EVR_Status EVR_Debug(const char *format, ...) 
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
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, *message++);
            while (!EUSCI_A_UART_getInterruptStatus(EUSCI_A0_BASE, EUSCI_A_UART_TRANSMIT_INTERRUPT_FLAG));
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

// #pragma vector=USCI_A0_VECTOR
// __interrupt void USCI_A0_ISR(void) {
//     switch (__even_in_range(UCA1IV,18)) {
//         case USCI_NONE:
//             break;
//         case USCI_UART_UCRXIFG: /* Receive ISR */
//         {
//             char c = EUSCI_A_UART_receiveData(EUSCI_A0_BASE);
//             if (c == '\n' || c == '\r') {
//                 g_uartIsrContext.uartBuffer[g_uartIsrContext.uartIndex] = '\0';
//                 g_uartIsrContext.commandReady = true; 
//             } else {
//                 g_uartIsrContext.uartBuffer[g_uartIsrContext.uartIndex++] = c;
//             }
//             break;
//         }     
//         case USCI_UART_UCTXIFG: /* Transmit ISR */
//             break;
//         case USCI_UART_UCSTTIFG:
//             break;
//         case USCI_UART_UCTXCPTIFG:
//             break;
//         default:
//             break;
//     }
// }
