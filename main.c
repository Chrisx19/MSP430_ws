#include <msp430.h>
#include "wdt_a.h"
#include "mcp3008.h"
/**
 * main.c
 */
int val = 0;
MCP3008_t mcp = {0};
int main(void)
{
    WDT_A_hold(WDT_A_BASE);	// stop watchdog timer
    MCP3008_Init(&mcp);
    __bis_SR_register(GIE);

    while (1) {
        val = MCP3008_ReadChannel_1(&mcp);
        __delay_cycles(500000);
    }
}
