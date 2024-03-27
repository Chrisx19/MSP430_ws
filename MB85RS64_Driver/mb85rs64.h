// Datasheet: https://cdn-shop.adafruit.com/datasheets/MB85RS64V-DS501-00015-4v0-E.pdf
#ifndef MB85RS64_DRIVER_H_
#define MB85RS64_DRIVER_H_
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "eusci_b_spi.h"
#include "pmm.h"
#include "gpio.h"
#include "cs.h"

#define TX_SIZE 4

#define FRAM_SR_WRITE_ENABLE_LATCH 0x02
#define FRAM_SR_BLOCK_PROTECT_0    0x04
#define FRAM_SR_BLOCK_PROTECT_1    0x08
#define FRAM_SR_WRITE_PROTECT      0x80

// Device parameters
#define MB85RS64V_MEMORY_SIZE 8192 // Total memory in bytes (64Kbits)
#define MB85RS64V_PAGE_SIZE   256  // Page size in bytes

/** Operation Codes **/
typedef enum {
  OPCODE_WREN  = 0x06,    /* Write Enable Latch */
  OPCODE_WRDI  = 0x04,    /* Reset Write Enable Latch */
  OPCODE_RDSR  = 0x05,    /* Read Status Register */
  OPCODE_WRSR  = 0x01,    /* Write Status Register */
  OPCODE_READ  = 0x03,    /* Read Memory */
  OPCODE_WRITE = 0x02,    /* Write Memory */
  OPCODE_RDID  = 0x9F     /* Read Device ID */
} MB85RS64_Opcodes_t;

typedef enum {
    MB85RS64_ERR_SUCCESS = 0,
    MB85RS64_ERR_FAILURE,
    MB85RS64_ERR_SPI_FAILURE,
    MB85RS64_ERR_WRITE_EN_FAILURE,
} MB85RS64_Error_t;

typedef struct {
    uint8_t txBuffer[TX_SIZE];
    uint8_t rxRaw;
    unsigned int txCounter;
    unsigned int rxCounter;
} MB85RS64_t;

MB85RS64_Error_t MB85RS64_Init(MB85RS64_t *fram);
MB85RS64_Error_t MB85RS64_WriteEnableLatch(bool latchEn);
MB85RS64_Error_t MB85RS64_Write(MB85RS64_t *fram, uint16_t address, uint8_t const data);

#endif /* MB85RS64_DRIVER_H_ */
