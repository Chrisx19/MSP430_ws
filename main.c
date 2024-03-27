#include <stdio.h>
#include <msp430.h> 
#include <mb85rs64.h>

//volatile int val = 0;
MB85RS64_t fram = {0};
//char txBuff[64] = {0x00};

int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;   // stop watchdog timer

    if (MB85RS64_Init(&fram) != MB85RS64_ERR_SUCCESS) {
        printf("Initialize Err\r\n");
        return -1;
    }
    __bis_SR_register(GIE);

    MB85RS64_WriteEnableLatch(true);
    MB85RS64_Write(&fram, 0x0069, 0x69);
    MB85RS64_WriteEnableLatch(false);

    printf("Passed Initialized\r\n");
    while (1) {

    }
}
