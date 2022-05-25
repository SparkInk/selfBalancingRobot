
/***********************************************
 * Header for I2C protocol
 * 
 * 
 * **********************************************/

#include <msp430.h>

#ifndef USCII2C_H
#define USCII2C_H

    // constants
//    #define SMCLK_DIV_10 10     // SMCLK deviser by 10
    #define SMCLK_DIV_10 100     // SMCLK deviser by 100
    
    // MACROS
    #define SEND_SA(slaveAddr)   UCB1I2CSA = slaveAddr

    #define I2C_RST_ON          UCB1CTL1 |= UCSWRST;
    #define I2C_RST_OFF         UCB1CTL1 &= ~UCSWRST;

    #define MST_TX_MODE         UCB1CTL1 |= UCTR     // Reset is ON; Transmitter Mode; Reset is OFF
    #define MST_RX_MODE         UCB1CTL1 &= ~UCTR   // Reset is ON; Receiver Mode; Reset is OFF
    #define START_BIT           UCB1CTL1 |= UCTXSTT    // Transmit START bit
    #define STOP_BIT            UCB1CTL1 |= UCTXSTP    // Transmit STOP bit
    #define WAIT_STT            while (UCB1CTL1 & UCTXSTT)  // Wait for START bit to go low -> ACK is received
    #define WAIT_TX_IFG         while (!(UCB1IFG & UCTXIFG))  // Wait for TXBUFF to be empty; when it is empty TXIFG == 1;
    #define WAIT_RX_IFG         while (!(UCB1IFG & UCRXIFG))   // Wait RXBUFF to receive a character; when receive a character RXIFG == 1;

    // global variables
    
    // functions' prototypes

    void usciB1I2CInit (unsigned int sclkDiv);

    signed char usciB1MstTx (signed char nBytes, char slaveAddr, char *txBuffer);

    signed char usciB1MstRx (unsigned char nBytes, unsigned char slaveAddr, signed char *rxBuffer);

#endif
