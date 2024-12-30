#include <msp430.h>
#include <driverlib.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdarg.h>
#include <stdbool.h>

#define UART_BUFFER_SIZE 128

// UART configuration parameters
// Datasheet to configure baudrate and UART
// https://www.ti.com/lit/ug/slau367p/slau367p.pdf?ts=1706206110916&ref_url=https%253A%252F%252Fwww.ti.com%252Fproduct%252FMSP430FR5969
EUSCI_A_UART_initParam uartConfig = {
    EUSCI_A_UART_CLOCKSOURCE_SMCLK,    // Clock source: SMCLK
    103,                              // Clock prescaler
    0,                                // First modulation stage
    0xDD,                             // Second modulation stage; 0.7503 (Pg 779)
    EUSCI_A_UART_NO_PARITY,           // No parity
    EUSCI_A_UART_LSB_FIRST,           // LSB first
    EUSCI_A_UART_ONE_STOP_BIT,        // One stop bit
    EUSCI_A_UART_MODE,                // UART mode
};

typedef enum {
    EVR_SUCCESS = 0,
    EVR_NULL_ERROR,
    EVR_ERROR
} EVR_Code;

typedef struct {
    char command[20];
    char parameters[3][10];
} Command;

volatile char uartBuffer[UART_BUFFER_SIZE] = {0};
volatile uint16_t uartIndex = 0;
volatile bool commandReady = false;

EVR_Code EVR(const char *format, ...) {
    if (format == NULL) {
        return EVR_NULL_ERROR;
    }

    EVR_Code ret = EVR_SUCCESS;
    char buffer[UART_BUFFER_SIZE] = {0};
    va_list args;

    // Start variadic arguments processing
    va_start(args, format);

    // Format the string into the buffer
    if (vsnprintf(buffer, UART_BUFFER_SIZE, format, args) >= UART_BUFFER_SIZE) {
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

void resetBuffer() {
    memset((char *)uartBuffer, 0, UART_BUFFER_SIZE);
    uartIndex = 0;
    commandReady = false;
}

EVR_Code GetCommand(Command *cmd) {
    if (cmd == NULL) {
        return EVR_NULL_ERROR;
    }

    EVR_Code ret = EVR_SUCCESS;

    if (commandReady == true) {
        commandReady = false; // Reset the flag

        // Parse the command
        char *token = strtok((char *)uartBuffer, " ");
        if (token) {
            memcpy(cmd->command, token, sizeof(cmd->command) - 1);
            cmd->command[sizeof(cmd->command) - 1] = '\0';

            uint16_t count = 0;
            while ((token = strtok(NULL, " ")) && count < 3) {
                memcpy(cmd->parameters[count], token, sizeof(cmd->parameters[0]) - 1);
                cmd->parameters[count][sizeof(cmd->parameters[0]) - 1] = '\0';
                count++;
            }

            // Respond with command details
            EVR("Called %s %s %s %s\n\r", cmd->command, count > 0 ? cmd->parameters[0] : "",
                     count > 1 ? cmd->parameters[1] : "",
                     count > 2 ? cmd->parameters[2] : "");
        } else {
            ret = EVR_ERROR;
        }
    }

    return ret;
}


void main(void) {
    // Stop watchdog timer
    WDT_A_hold(WDT_A_BASE);

    // Unlock GPIO functionality
    PMM_unlockLPM5();

    // Configure UART pins (P2.6 -> TXD, P2.5 -> RXD)
    GPIO_setAsPeripheralModuleFunctionInputPin(
        GPIO_PORT_P2,
        GPIO_PIN6 | GPIO_PIN5,
        GPIO_SECONDARY_MODULE_FUNCTION
    );

    // Error R LED
    GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN6);

    if (STATUS_FAIL == EUSCI_A_UART_init(EUSCI_A1_BASE, &uartConfig)) {
        GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN6);
        return;
    }

    // Enable UART module and interrupts
    EUSCI_A_UART_enable(EUSCI_A1_BASE);
    EUSCI_A_UART_clearInterrupt(EUSCI_A1_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
    EUSCI_A_UART_enableInterrupt(EUSCI_A1_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);

    __enable_interrupt();

    EVR("System is running...\n\r");

    while (1) {
        Command cmd = {0};
        GetCommand(&cmd);
    }
}

// UART ISR Pg 784
// https://www.ti.com/lit/ug/slau367p/slau367p.pdf?ts=1706206110916&ref_url=https%253A%252F%252Fwww.ti.com%252Fproduct%252FMSP430FR5969
#pragma vector=USCI_A1_VECTOR
__interrupt void USCI_A1_ISR(void) {
    switch (__even_in_range(UCA1IV,18)) {
        case USCI_NONE:
            break;
        case USCI_UART_UCRXIFG: /* Receive ISR */
            if (uartIndex < UART_BUFFER_SIZE - 1) {
                char receivedCharacter = EUSCI_A_UART_receiveData(EUSCI_A1_BASE);
                if (receivedCharacter == '\n' || receivedCharacter == '\r') {
                    uartBuffer[uartIndex] = '\0'; // Null-terminate the string
                    commandReady = true;         // Set the flag to indicate command is ready
                    uartIndex = 0;               // Reset the buffer index
                } else if (!commandReady) {      // Only append if not processing a command
                    uartBuffer[uartIndex++] = receivedCharacter;
                }
            } else {
                resetBuffer(); // Clear the buffer on overflow
            }
            break;
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
