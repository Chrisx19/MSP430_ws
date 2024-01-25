#include <msp430.h>

void initI2C(void);
void startI2CTransmission(void);

int main(void)
{
    PM5CTL0 &= ~LOCKLPM5;
    WDTCTL = WDTPW | WDTHOLD;  // Stop watchdog timer

    // Initialize I2C communication
    initI2C();

    // Main loop
    while(1)
    {
        // Start I2C transmission
        startI2CTransmission();

        // Enter low-power mode with interrupts enabled
        __bis_SR_register(LPM0_bits + GIE);
    }
}

void initI2C(void)
{
    // Configure I2C pins
    P1SEL1 |= BIT6 + BIT7;  // Select I2C function for P1.6 (SDA) and P1.7 (SCL)
    P1SEL0 &= ~(BIT6 + BIT7);

    // Setup I2C
    UCB0CTLW0 |= UCSWRST;                    // Enable SW reset
    UCB0CTLW0 |= UCMST + UCMODE_3 + UCSYNC;  // I2C Master, synchronous mode
    UCB0CTLW1 = UCASTP_2;                    // Automatic stop generated
                                             // after UCB0TBCNT is reached
    UCB0BRW = 10;                            // Set baud rate
    UCB0CTLW0 &= ~UCSWRST;                   // Clear SW reset, resume operation
    UCB0IE |= UCRXIE + UCTXIE;               // Enable interrupt
}

void startI2CTransmission(void)
{
    UCB0I2CSA = 0x0A;        // Set slave address to 0x0A
    UCB0CTLW0 |= UCTR + UCTXSTT;  // I2C TX, start condition
}

#pragma vector = USCI_B0_VECTOR
__interrupt void USCI_B0_ISR(void)
{
    if (UCB0IFG & UCTXIFG) {
        // Transmit data
        UCB0TXBUF = 0x55;  // Example data to send
    }
    if (UCB0IFG & UCRXIFG) {
        // Receive data (if necessary)
        volatile unsigned char received_data = UCB0RXBUF;
    }

    UCB0CTLW0 |= UCTXSTP;  // Send I2C stop condition
    __bic_SR_register_on_exit(LPM0_bits);  // Exit LPM0
}
