#include <msp430.h>
#include "init.h"
#include "ina219.h"

#define SDA BIT2
#define SCL BIT3

INA219_t my_INA;
int arr[2];

void initI2C(void);
void startI2CTransmission(void);

int main(void)
{
    disableI nit();
    P1DIR |= 0x01;  // //BIT0

    // Initialize I2C communication
    initI2C();
    startI2CTransmission();
    // Main loop
    while(1)
    {
        // Start I2C transmission
//        startI2CTransmission();

    }
}

//Registers pg 649
void initI2C(void)
{
    // Set up I2C pins, assuming P5.2 (SDA) and P5.3 (SCL)
    P5SEL0 |=  SCL;
    P5SEL0 |=  SDA;

//    // Set MSP430 in I2C master mode
    UCB0CTLW0 |= UCSWRST;       // Enable SW reset
    /* I2C Master, synchronous mode  */
    UCB0CTLW0 |= UCSSEL_3;      // SMCLK Clock source
    UCB0BRW = 10;               // Set baud rate ; Prescale 10

    UCB0CTLW0 |= UCSYNC;        // Synchronous mode
    UCB0CTLW0 |= UCMODE_3;      // I2C Mode
    UCB0CTLW0 |= UCMST;         //Master Mode
    UCB0CTLW0 |= UCTR;          //Transmitter Mode
    UCB0I2CSA = INA219_ADDRESS; // Set slave address;; INA219 Address

    //REGISTER 2
    UCB0CTLW1 |= UCASTP_2;      // Automatic stop generated
    UCB0TBCNT = 1;              // # of bytes in packet

    UCB0CTL1 &= ~UCSWRST;                    // Clear SW reset, resume operation;; local enable for TX0
    UCB0IE |= UCTXIE0 + UCNACKIE;            // Enable transmit and NACK interrupt
    __enable_interrupt();
}

void startI2CTransmission(void)
{
    UCB0CTLW0 |= UCTXSTT;  // manual start condition
    __delay_cycles(100);
}

#pragma vector = USCI_B0_VECTOR
__interrupt void USCI_B0_ISR(void)
{
//        const uint16_t config           =   SHUNT_BUS_CONT | SADC_12BIT_532US
//                               | BADC_12BIT_532US | PG_GAIN_8_40mV | BRNG_FSR_32V;
        while (UCB0CTL1 & UCTXSTP);             // Ensure stop condition got sent
        UCB0CTL1 |= UCTR + UCTXSTT;             // I2C TX, start condition

        while (!(UCB0IFG & UCTXIFG));           // Wait for TX buffer ready
        UCB0TXBUF = 0x39;                      // Send first byte

        while (!(UCB0IFG & UCTXIFG));           // Wait for TX buffer ready
        UCB0TXBUF = 0x9F;                      // Send second byte

        while (!(UCB0IFG & UCTXIFG));           // Wait for transmission complete
        UCB0CTL1 |= UCTXSTP;                    // I2C stop condition
        UCB0IFG &= ~UCTXIFG;                    // Clear USCI_B0 TX int flag
        P1OUT ^= 0x01;
//    __bic_SR_register_on_exit(LPM0_bits);  // Exit LPM0
}
