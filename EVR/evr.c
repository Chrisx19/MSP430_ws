/*
 * evr.c
 *
 *  Created on: 02/10/25
 *      Author: clazo
 */
#include "evr.h"

#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driverlib.h"
#include "msp430.h"

#define UART_CMD_SIZE      32
#define UART_TRANSMIT_SIZE 128

#define DEBUG_EVR_CHANNEL           EUSCI_A0_BASE
#define USER_EVR_CHANNEL            EUSCI_A1_BASE
#define CMD_CRC_LOCATION_WITH_SPACE (g_uartIsrContext.uartIndex - 5)
#define CMD_CRC_LOCATION            (g_uartIsrContext.uartBuffer + g_uartIsrContext.uartIndex - 4)

typedef struct {
  volatile char    uartBuffer[UART_CMD_SIZE];
  volatile bool    commandReady;
  volatile bool    bufferOverflow;
  volatile uint8_t evrCh;
  volatile uint8_t uartIndex;
} UART_ISR_Context_t;

static volatile UART_ISR_Context_t g_uartIsrContext = {0};

static Evr_Status InterfaceUartInit(EUSCI_A_UART_initParam *uartConfig, const uint16_t *baseChannel)
{
  int uartChannel;
  for (uartChannel = 0; uartChannel < 2; uartChannel++) {
    if (EUSCI_A_UART_init(baseChannel[uartChannel], uartConfig) == STATUS_FAIL) {
      return EVR_ERROR;
    }

    EUSCI_A_UART_enable(baseChannel[uartChannel]);
    EUSCI_A_UART_clearInterrupt(baseChannel[uartChannel], EUSCI_A_UART_RECEIVE_INTERRUPT);
    EUSCI_A_UART_enableInterrupt(baseChannel[uartChannel], EUSCI_A_UART_RECEIVE_INTERRUPT);
  }

  return EVR_SUCCESS;
}

Evr_Status EVR_Init()
{
  /* Configure user & debug UART pins */
  GPIO_setAsPeripheralModuleFunctionInputPin(
      GPIO_PORT_P2, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN6 | GPIO_PIN5, GPIO_SECONDARY_MODULE_FUNCTION);

  // UART configuration parameters set to 115200 baud
  // Datasheet to configure baudrate and UART
  // https://www.ti.com/lit/ug/slau367p/slau367p.pdf?ts=1706206110916&ref_url=https%253A%252F%252Fwww.ti.com%252Fproduct%252FMSP430FR5969
  // 8 Mhz / 115200 = 69.4444; 0.4378(0x55) ~ 0.4444
  EUSCI_A_UART_initParam uartConfig = {
      .selectClockSource = EUSCI_A_UART_CLOCKSOURCE_SMCLK,  // Clock source: SMCLK
      .clockPrescalar    = 69,                              // Clock prescaler
      .firstModReg       = 0x07,                       // First modulation stage 0.4378 (Pg 779)
      .secondModReg      = 0x00,                       // Second modulation stage
      .parity            = EUSCI_A_UART_NO_PARITY,     // No parity
      .msborLsbFirst     = EUSCI_A_UART_LSB_FIRST,     // LSB first
      .numberofStopBits  = EUSCI_A_UART_ONE_STOP_BIT,  // One stop bit
      .uartMode          = EUSCI_A_UART_MODE,          // UART mode
  };

  const uint16_t baseChannels[2] = {USER_EVR_CHANNEL, DEBUG_EVR_CHANNEL};

  if (InterfaceUartInit(&uartConfig, baseChannels) != EVR_SUCCESS) {
    return EVR_ERROR;
  }

  /* Default UART interface EVR on debug mode */
  g_uartIsrContext.evrCh = DEBUG;

  return EVR_SUCCESS;
}

Evr_Status EVR(const char *format, ...)
{
  if (format == NULL) {
    return EVR_NULL_ERROR;
  }

  Evr_Status ret                        = EVR_SUCCESS;
  char       buffer[UART_TRANSMIT_SIZE] = {0};
  va_list    args;

  va_start(args, format);

  /* Format the string into the buffer */
  if (vsnprintf(buffer, UART_TRANSMIT_SIZE, format, args) >= UART_TRANSMIT_SIZE) {
    ret = EVR_ERROR;  // Buffer overflow error
  }

  va_end(args);

  if (ret == EVR_SUCCESS) {
    const char *message = buffer;
    while (*message) {
      EUSCI_A_UART_transmitData(DEBUG_EVR_CHANNEL, *message++);
      while (
          !EUSCI_A_UART_getInterruptStatus(DEBUG_EVR_CHANNEL, EUSCI_A_UART_TRANSMIT_INTERRUPT_FLAG))
        ;
    }

    EUSCI_A_UART_transmitData(DEBUG_EVR_CHANNEL, (uint8_t)'\r');
    EUSCI_A_UART_transmitData(DEBUG_EVR_CHANNEL, (uint8_t)'\n');
  }

  return ret;
}

// UART ISR Pg 784
// https://www.ti.com/lit/ug/slau367p/slau367p.pdf?ts=1706206110916&ref_url=https%253A%252F%252Fwww.ti.com%252Fproduct%252FMSP430FR5969
#pragma vector = USCI_A0_VECTOR
__interrupt void USCI_A0_ISR(void)
{
  switch (__even_in_range(UCA0IV, USCI_UART_UCTXCPTIFG)) {
    case USCI_NONE:
      break;
    case USCI_UART_UCRXIFG: /* Receive ISR */
    {
      char c = EUSCI_A_UART_receiveData(DEBUG_EVR_CHANNEL);
      if (c == '\n' || c == '\r') {
        g_uartIsrContext.uartBuffer[g_uartIsrContext.uartIndex] = '\0';
        g_uartIsrContext.commandReady                           = true;
        g_uartIsrContext.evrCh                                  = DEBUG;
      }
      else {
        // Only add the character if there is room in the buffer.
        if (g_uartIsrContext.uartIndex < (UART_CMD_SIZE - 1)) {
          g_uartIsrContext.uartBuffer[g_uartIsrContext.uartIndex++] = c;
        }
        else {
          g_uartIsrContext.bufferOverflow = true;
        }
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

#pragma vector = USCI_A1_VECTOR
__interrupt void USCI_A1_ISR(void)
{
  switch (__even_in_range(UCA1IV, USCI_UART_UCTXCPTIFG)) {
    case USCI_NONE:
      break;
    case USCI_UART_UCRXIFG: /* Receive ISR */
    {
      char c = EUSCI_A_UART_receiveData(USER_EVR_CHANNEL);
      if (c == '\n' || c == '\r') {
        g_uartIsrContext.uartBuffer[g_uartIsrContext.uartIndex] = '\0';
        g_uartIsrContext.commandReady                           = true;
        g_uartIsrContext.evrCh                                  = USER;
      }
      else {
        // Only add the character if there is room in the buffer.
        if (g_uartIsrContext.uartIndex < (UART_CMD_SIZE - 1)) {
          g_uartIsrContext.uartBuffer[g_uartIsrContext.uartIndex++] = c;
        }
        else {
          g_uartIsrContext.bufferOverflow = true;
        }
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
