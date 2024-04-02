#include <stdio.h>
#include <msp430.h> 
#include <mb85rs64.h>

MB85RS64_t fram = {0};
uint8_t val_1 = 0;
uint8_t val_2 = 0;

int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;   // stop watchdog timer

    if (MB85RS64_Init(&fram) != MB85RS64_ERR_SUCCESS) {
        printf("Initialize Err\r\n");
        return -1;
    }

    printf("Passed Initialized\r\n");
    MB85RS64_WriteEnableLatch(true);
    MB85RS64_Write(&fram, 0x0069, 0x69);
    MB85RS64_WriteEnableLatch(false);

    MB85RS64_WriteEnableLatch(true);
    MB85RS64_Write(&fram, 0x0099, 0x99);
    MB85RS64_WriteEnableLatch(false);

    MB85RS64_Read(&fram, 0x0069, &val_1);
    printf("Read Val = 0x%02X\r\n", val_1);

    MB85RS64_Read(&fram, 0x0099, &val_2);
    printf("Read Val = 0x%02X\r\n", val_2);

    while (1) {

    }
}
