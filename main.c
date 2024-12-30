#include <msp430.h>
#include <driverlib.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>

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

EVR_Code EVR(const char *format, ...) {
    if (format == NULL) {
        return EVR_NULL_ERROR;
    }

    EVR_Code ret = EVR_SUCCESS;
    char buffer[UART_BUFFER_SIZE];
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

EVR_Code GetCommand(Command *cmd) {
    if (cmd == NULL) {
        return EVR_NULL_ERROR;
    }

    EVR_Code ret = EVR_SUCCESS;
    char buffer[50];
    int index = 0;

    // Read characters until newline or buffer full
    while (index < sizeof(buffer) - 1) {
        while (!EUSCI_A_UART_getInterruptStatus(EUSCI_A1_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG));
        char c = EUSCI_A_UART_receiveData(EUSCI_A1_BASE);

        if (c == '\n' || c == 0x13) { // Ends at carriage return
            buffer[index] = '\0';
            break;
        }
        buffer[index++] = c;
    }

    // Ensure null termination if buffer limit is reached
    buffer[sizeof(buffer) - 1] = '\0';

    // Parse the command
    char *token = strtok(buffer, " ");
    if (token) {

        // Command too long
        if (strlen(token) >= sizeof(cmd->command)) {
            return EVR_ERROR;
        }
        memcpy(cmd->command, token, sizeof(cmd->command) - 1);
        cmd->command[sizeof(cmd->command) - 1] = '\0';

        int count = 0;
        while ((token = strtok(NULL, " ")) && count < 3) {
            char *end = token + strlen(token) - 1;
            while (end >= token && (*end == '\r' || *end == '\n')) {
                *end = '\0';
                end--;
            }

            // Parameter too long
            if (strlen(token) >= sizeof(cmd->parameters[0])) {
                return EVR_ERROR; 
            }

            memcpy(cmd->parameters[count], token, sizeof(cmd->parameters[0]) - 1);
            cmd->parameters[count][sizeof(cmd->parameters[0]) - 1] = '\0';
            count++;
        }

        // Cmd Response
        char response[100];
        snprintf(response, sizeof(response), "Called %s %s %s %s\n\r",
                 cmd->command,
                 count > 0 ? cmd->parameters[0] : "",
                 count > 1 ? cmd->parameters[1] : "",
                 count > 2 ? cmd->parameters[2] : "");
        EVR(response);
    } else {
        ret = EVR_ERROR;
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

    // Initialize UART
    if (STATUS_FAIL == EUSCI_A_UART_init(EUSCI_A1_BASE, &uartConfig)) {
        GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN6);
        return;
    }

    // Enable UART module
    EUSCI_A_UART_enable(EUSCI_A1_BASE);
    EVR("System is running...\n\r");

    while (1) {
        Command cmd = {0};
        GetCommand(&cmd);
    }
}
