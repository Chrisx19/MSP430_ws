#include <stdio.h>
#include <msp430.h> 
#include "mcp3008.h"


/**
 * hello.c
 */

volatile int val = 0;
MCP3008_t mcp = {0};
char txBuff[64] = {0x00};

int main(void)
{
	WDTCTL = WDTPW | WDTHOLD;	// stop watchdog timer

    if (MCP3008_Init(&mcp) != MCP3008_ERR_SUCCESS) {
        return -1;
    }

    __bis_SR_register(GIE);


    while (1) {
        val = MCP3008_ReadChannel_1(&mcp);
        sprintf(txBuff, "Ch0 Val = %d\r\n", val);
        printf(txBuff);
//        __delay_cycles(500);
    }
}
