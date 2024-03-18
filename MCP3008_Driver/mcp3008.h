// Datasheet: https://cdn-shop.adafruit.com/datasheets/MCP3008.pdf
#ifndef MCP3008_DRIVER_H_
#define MCP3008_DRIVER_H_
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "eusci_b_spi.h"
#include "pmm.h"
#include "gpio.h"
#include "cs.h"

#define RX_SIZE 2

// MCP3008 Command Bits
#define MCP3008_START_BIT          0x01
#define MCP3008_SINGLE_ENDED       0x80
#define MCP3008_DIFFERENTIAL       0x00

// Channel Selection for MCP3008
// For single-ended mode
#define MCP3008_CHANNEL_0          (0x00 << 4)
#define MCP3008_CHANNEL_1          (0x01 << 4)
#define MCP3008_CHANNEL_2          (0x02 << 4)
#define MCP3008_CHANNEL_3          (0x03 << 4)
#define MCP3008_CHANNEL_4          (0x04 << 4)
#define MCP3008_CHANNEL_5          (0x05 << 4)
#define MCP3008_CHANNEL_6          (0x06 << 4)
#define MCP3008_CHANNEL_7          (0x07 << 4)

typedef enum {
    MCP3008_ERR_SUCCESS = 0,
    MCP3008_ERR_FAILURE,
    MCP3008_ERR_SPI_FAILURE,
} MCP3008_Error_t;

typedef struct {
    uint8_t txBuffer;
    uint8_t rxRaw;
    uint8_t rxBuffer[RX_SIZE];
    unsigned int txCounter;
    unsigned int rxCounter;
} MCP3008_t;

MCP3008_Error_t MCP3008_Init(MCP3008_t *mcp);
int MCP3008_ReadChannel_1(MCP3008_t *mcp);

#endif /* MCP3008_DRIVER_H_ */
