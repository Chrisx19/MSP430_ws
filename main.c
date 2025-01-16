#include "driverlib.h"
#include "msp430.h"
#include "EVR/evr.h"

#define CRC_POLY_CCITT_BR 0x1021
#define CRC_SEED 0xFACE

uint16_t CRC_16_Implementation(const uint16_t *data, uint32_t length);

void main (void)
{
    //Stop WDT
    WDT_A_hold(WDT_A_BASE);
    PMM_unlockLPM5();

    EVR_Init();

    uint16_t data[] = {
                            0xFFFF, // MSB SW Version
                            0xAFFF, // LSB SW Version
                            0x0001, // LED status
                            200,    // Brightness level
                            1,      // Power Good
                            4095,   // LED Current
                            23      // Chip Temperature
    };

    uint16_t crcResultReversed = 0;
    uint16_t calcCRC = 0;

    (void)CRC_setSeed(CRC_BASE, CRC_SEED);

    int i;
    for (i=0; i<sizeof(data)/sizeof(uint16_t); i++) {
        (void)CRC_set16BitDataReversed(CRC_BASE, data[i]);
    }

    crcResultReversed = CRC_getResult(CRC_BASE);

    calcCRC = CRC_16_Implementation(data, sizeof(data)/sizeof(uint16_t));

    EVR("CRC Module Prototype...\n\r");

    if (calcCRC == crcResultReversed) {
        EVR("Passed!\n\r");
        EVR("CRC Calculation   = 0x%X\n\r", calcCRC);
        EVR("CRC Reversed Calc = 0x%X\n\r", crcResultReversed);
    } else {
        EVR("Failed\n\r");
    }



    //Enter LPM4, interrupts enabled
    __bis_SR_register(LPM4_bits);
    __no_operation();

}

// Saving this here, working code on calculating CRC16 reversed
uint16_t CRC_16_Implementation(const uint16_t *data, uint32_t length)
{
    // Cast the 16-bit pointer to an 8-bit pointer
    const uint8_t *bytePtr = (const uint8_t *)data;

    uint16_t crc = CRC_SEED;
    int i, j;
    for (i = 0; i < (length*2); i++) {
        crc ^= (bytePtr[i] << 8);
        for (j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = ((crc << 1) ^ CRC_POLY_CCITT_BR);
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}
