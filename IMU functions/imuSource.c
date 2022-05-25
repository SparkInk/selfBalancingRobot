
/*************************************************
 *  MPU6050 source file 
 * 
 * 
 * Author: Iakov Umrikhin
 * 
 * Date Created: 23.02.2022 (dd::mm::yyyy)
 * Date Modified: 14.03.2022 (dd::mm::yyyy)
 * 
 * **********************************************
 * 
 * ************************************************/

#include <msp430.h>
#include "usciI2C.h"
#include "imuHeader.h"

/*********************************************************************
 * Name: usciB0I2CInit 
 * 
 * Description: usciB0I2cInit initialises I2C protocol on MSP4305529LP
 * 
 * Arguments: sclkDiv   -  sub-muster clock deviser
 * 
 * Return: none, for it owes you nothing 
 * 
 * Author: Iakov Umrikhin
 * Date Created: 23.02.2022 (dd::mm::yyyy)
 * Date Modified: 28.02.2022 (dd::mm::yyyy)
 * ******************************************************************/

void usciB1I2CInit (unsigned int sclkDiv) {
    UCB1CTL1 |= UCSSEL_2 | UCSWRST;     // SMCLK; reset ON

    // Configure Registers
    // Control registers
    UCB1CTL0 |= UCMST | UCMODE_3 | UCSYNC;  // Master Mode; I2C mode; synchronous mode
    UCB1CTL0 &= ~(UCA10 | UCSLA10 | UCMM);   // Own address: 7 bit; Slave address: 7 bit; Single-master mode

    // I2C own address register
    UCB1I2COA = 0;  // do not respond to general call; own address = 0;

    // Leave reset condition
    UCB1CTL1 &= ~UCSWRST;

    // Configure Ports
    // Use only P4.2 (SCL) and P4.1 (SDA)
    P4SEL |= BIT2 | BIT1;

    // Clock divider
    UCB1BR0 = sclkDiv & 0x00ff;
    UCB1BR1 = sclkDiv >> 8;
}
/*********************************************************************
 
/*********************************************************************
 * Name: mpu6050SendByte
 * 
 * Description: usciB0MstTx sends a string of bytes to PixyCam v2.
 *              It is little-endian byte ordering, i.e. LSB first;
 * 
 * Arguments: nBytes    -   a number of bytes in a string to be sent
 *            txBuffer  -   a string of bytes
 *            slaveAddr -   the address of the slave; not a register address
 *            restart   -   0: write to an address without issuing a restart bit;
 *                          1: issue a restart bit in a READ sequence
 * 
 * Return: 0 if nBytes have been sent successfully; -1 if it is all in vain
 * 
 * Author: Iakov Umrikhin
 * Date Created: 23.02.2022 (dd::mm::yyyy)
 * Date Modified: 14.03.2022 (dd::mm::yyyy)
 * ******************************************************************/

signed char IMUSendByte(unsigned char slaveAddr, unsigned char regAddr, unsigned char txBuffer, unsigned char restart) {

    volatile signed char result = -1;
    volatile unsigned char txCounter = 0;

    // Configure as a transmitter
    SEND_SA(slaveAddr);
    MST_TX_MODE;
    // Initialise START bit
    START_BIT;

    // send a register's address
    WAIT_TX_IFG;
    UCB1TXBUF = regAddr;

    if (!restart) { // single byte write

        // send data
        WAIT_TX_IFG;
        UCB1TXBUF = txBuffer;
        result = 0;
    }


    if (!stopBit && !restart) {
        WAIT_TX_IFG;
        STOP_BIT;
    }

    stopBit = 0;

    // wait for 1 ms
    WAIT_1ms;
    
    return result;
}

/*********************************************************************
 * Name: mpu6050SendBurst
 * 
 * Description: usciB0MstTx sends a string of bytes to PixyCam v2.
 *              It is little-endian byte ordering, i.e. LSB first;
 * 
 * Arguments: nBytes    -   a number of bytes in a string to be sent
 *            txBuffer  -   a string of bytes
 *            slaveAddr -   the address of the slave; not a register address
 * 
 * Return: 0 if nBytes have been sent successfully; -1 if it is all in vain
 * 
 * Author: Iakov Umrikhin
 * Date Created: 23.02.2022 (dd::mm::yyyy)
 * Date Modified: 14.03.2022 (dd::mm::yyyy)
 * ******************************************************************/

signed char IMUSendBurst(signed char nBytes, char slaveAddr, unsigned char regAddr, unsigned char *txBuffer, unsigned char restart) {

    volatile signed char result = -1;
    volatile unsigned char txCounter = 0;

    // Configure as a transmitter
    SEND_SA(slaveAddr);
    MST_TX_MODE;

    // Initialise START bit
    START_BIT;

    // send a register's address
    WAIT_TX_IFG;
    UCB1TXBUF = regAddr;
    
    // send data
    while(txCounter < nBytes && !stopBit && !restart) {
        WAIT_TX_IFG;
        UCB1TXBUF = txBuffer[txCounter];
        txCounter++;
        result = 0;
    }

    if (!stopBit) {
        WAIT_TX_IFG;
        STOP_BIT;

//        IMU_PWR_TRIG;
    }
    stopBit = 0;

    return result;
}
/*********************************************************************
 * Name: mpu6050Read 
 * 
 * Description: 
 * 
 * Arguments: nBytes    -   # of bytes to receive
 *            slaveAddr -   slave address
 *            regAddr   -   address of a register to read from
 *            rxBuffer  -   buffer to store received data
 * 
 * Return: 
 * 
 * Author: Iakov Umrikhin
 * Date Created: 23.02.2022 (dd::mm::yyyy)
 * Date Modified: 28.02.2022 (dd::mm::yyyy)
 * ******************************************************************/

signed char IMURead(unsigned char nBytes, unsigned char slaveAddr, signed char *rxBuffer) {


    volatile signed char result = -1;
    volatile unsigned char rxCounter = 0;

    // Receive mode
    SEND_SA(slaveAddr);
    MST_RX_MODE;

    // START Bit
    START_BIT;
//    WAIT_STT;
    // dummy read; to clear the UCRXIFG;
    rxBuffer[0] = UCB1RXBUF;

    while (rxCounter < nBytes && !stopBit) {
        WAIT_RX_IFG;
        rxBuffer[rxCounter] = UCB1RXBUF;

        if (rxCounter == (nBytes - 1)) {
            WAIT_RX_IFG;
            STOP_BIT;
        }

        rxCounter++;
    }

//    IMU_PWR_TRIG;

    return result;
}
/*********************************************************************
 * Name: readGyro
 *
 * Description: This function returns a pointer to an array with data from a gyroscope;
 *
 * Arguments: axisName  -   a name of a gyroscope's axis to read
 *
 * Return:
 *
 * Author: Iakov Umrikhin
 * Date Created: 04.03.2022 (dd::mm::yyyy)
 * Date Modified: 04.03.2022 (dd::mm::yyyy)
 * ******************************************************************/
signed char *readGyro(volatile unsigned char axisName) {

    volatile signed char *data;

    return data;
}

/************************************************************************************
* Function: dispCnsl
* Description: displays computedAngle on a console;
*              Serves as a replacement to sprintf(), because the latter refuses to
*              fulfill its duties :(
*
* Arguments: computedAngle  -   a number of counts
*
* return: none
* Author: Iakov Umrikhin
* Date Created: 06.02.2022 (dd:mm:yyyy)
* Date Modified: 06.03.2022 (dd::mm::yyyy)
************************************************************************************/
void dispCnsl(signed int computedSpeed) {
    char buffer[8] = 0;
    unsigned char i = 0;
    unsigned char j = 0;

    if (computedSpeed >= 0) {    // positive counts
        while (computedSpeed > 0) {
            volatile int mod;
            mod = computedSpeed % 10;
            buffer[7-j] = mod;    //0th element equals to the last digit of a number
            //usciA1UartTxChar('0' + mod);    // shift a digit by 40 in ASCII table
            computedSpeed /= 10;
            j++;
        }
        for (i = (8-j); i < 8; i++){
            usciA1UartTxChar('0' + buffer[i]);
        }
        i = 0;
        j = 0;
//        usciA1UartTxChar('\r');
//        usciA1UartTxChar('\n');
    }
    if (computedSpeed  < 0) {   // negative counts
        usciA1UartTxChar('-');  // display a negative sign
        computedSpeed *= -1;    // convert from negative to positive
        while (computedSpeed > 0) {
            volatile int mod;
            mod = computedSpeed % 10;   // chunk off the last digit
            buffer[7-j] = mod;    // 0th element equals to the last digit of a number
            //usciA1UartTxChar(mod + '0');
            computedSpeed /= 10;      // remove the last digit from the current number
            j++;
        }
        for (i = (8-j); i < 8; i++){
            usciA1UartTxChar('0' + buffer[i]);
        }
        i = 0;
        j = 0;
//        usciA1UartTxChar('\r');
//        usciA1UartTxChar('\n');
    }

    memset(buffer, 0, 8);   // clear buffer from previous data
}

#pragma vector = USCI_B1_VECTOR
__interrupt void usciB1ISR (void) {

    switch(__even_in_range(UCB1IV, 4)) {
    case 0:
        break;
    case 2:
        break;
    case 4:     // NACK
        UCB1CTL1 |= UCTXSTP;
        stopBit = 1;
        break;
    }
}
