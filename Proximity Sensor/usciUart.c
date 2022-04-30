/*************************************************************************************************
 * ucsiUart.c
 * - C implementation or source file for MSP430 UCSI UART A1
 *
 *  Author: Iakov Umrikhin
 *  Created on: 16.01.2022
 *  Modified: 17.01.2022
 **************************************************************************************************/

#include <msp430.h>
#include "usciUart.h"

#define     TXD_A1          BIT4            //Transmit Data on P4.4
#define     RXD_A1          BIT5            //Recieve Data on P4.5

volatile signed long int rxString[50];  // store commands from the terminal
/************************************************************************************
* Function: ucsiA1UartInit
* - configures UCA1 UART to use SMCLK, no parity, 8 bit data, LSB first, one stop bit
*  BAUD rate = 19.2Kbps with 16xoversampling.
*  assumes SMCLK = 2^20 Hz.
* argument:
* Arguments: none, but baud rate would be useful
*
* return: none
* Author: Greg Scutt
* Date: March 1st, 2017
* Modified: <date of any mods> usually taken care of by rev control
************************************************************************************/
void usciA1UartInit(){

	// READ THIS --> You must use UCSI Control BITS defined in msp430.h !!
	// In fact you must use pre-defined control bits whenever possible. no obscure HEX codes allowed anymore.

	P4SEL |= TXD_A1 | RXD_A1;
	UCA1CTL1 |= UCSWRST; 			// USCI A1  Software Reset Enabled
	//********************

	UCA1CTL1 	|= 	UCSSEL_2; 		// select SMCLK. User is responsible for setting this rate.

	UCA1CTL0	 = 	0; 				// RESET UCA1CTL0 before new configuration
	UCA1CTL0	&=	~UCPEN			// No Parity
				&	~UCMSB			// LSB First
				&	~UC7BIT			// 8 bits of data
				&	~UCSPB			// 1 stop bit
				&	~UCSYNC;		// UART Mode


	UCA1BR1 = 0;// high byte of clock prescaler; (UCA1BR1 * 256 + UCA1BR0) = UCBR1.
	UCA1BR0 = 3;// low byte of clock prescaler; (UCA1BR1 * 256 + UCA1BR0) = UCBR1.
	UCA1MCTL |= UCBRF_7 | UCOS16;// <7:4> = 6 (0xC) - first modulation (BITCLK16); <3:1> = 0 - second modulation (BITCLK); <0> = 0x1 - oversample (enabled: 1; disabled: 0).

	UCA1CTL1 	&= ~UCSWRST; 		//  configured. take state machine out of reset.
	}


/************************************************************************************
* Function: ucsiA1UartTxChar
* - writes a single character to UCA1TXBUF if it is ready
* argument:
* Arguments: txChar - byte to be transmitted
*
* return: none
* Author: Greg Scutt
* Date: March 1st, 2017
* Modified: <date of any mods> usually taken care of by rev control
************************************************************************************/
void usciA1UartTxChar(signed long int txChar) {

	while (!(UCA1IFG & UCTXIFG)); // is this efficient ?
		UCA1TXBUF = txChar;	 // if TXBUFF ready then transmit a byte by writing to it
}


/************************************************************************************
* Function: ucsiA1UartTxString
* - writes a C string of characters, one char at a time to UCA1TXBUF by calling
*   ucsiA1UartTxChar. Stops when it encounters  the NULL character in the string
*   does NOT transmit the NULL character
* argument:
* Arguments: txChar - pointer to char (string) to be transmitted
*
* return: number of characters transmitted
* Author: Greg Scutt
* Date: March 1st, 2017
* Modified: <date of any mods> usually taken care of by rev control
************************************************************************************/
void usciA1UartTxString(unsigned char *txChar){

	// while not a null character
    while (*txChar != '\n') {
        // transmit a character with ucsiA1UartTxChar and increment the pointer
        usciA1UartTxChar(*txChar);
        txChar += 1;    // should shift the pointer to a next character in the string
    }

}
/************************************************************************************
* Function: usciA1UartGets
* -
* argument:
* Arguments: rxString - pointer to an array
*
* return:
* Author: Iakov Umrikhin
* Date: 16.01.2022 (dd.mm.yyyy)
* Modified: <date of any mods> usually taken care of by rev control
************************************************************************************/
char *usciA1UartGets (char * rxString) {
    static int status = -1;
    char *returnValue;
    static unsigned volatile int i = 0; // counter for rxBuffer

   do {   // writing a string in the console
        status = 0;

        while(!(UCRXIFG & UCA1IFG)){
           // wait for a character to be entered
        }

        rxBuffer[i] = UCA1RXBUF;    // move an entered character to rxBuffer
        usciA1UartTxChar(UCA1RXBUF);    // display an entered character
        i++;

    } while (UCA1RXBUF != '\r' && i < BUF_SIZE);

    rxBuffer[i] = '\n'; // puts a new-line character at the end

    i = 0; // updates the counter -> sets it to zero

    usciA1UartTxChar('\n'); // carriage return

    strcpy(rxString, rxBuffer); // copy rxBuffer to rxString
    memset(rxBuffer, 0, BUF_SIZE);  // clear rxBuffer for the next command

    if (status) {
        memset(rxString, 0, 50);
    }
    return rxString;
}


