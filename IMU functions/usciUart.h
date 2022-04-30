/*************************************************************************************************
 * ucsiUart.h
 * - - C interface file for MSP430 UCSI UART A1, A0
 *
 *  Author: Greg Scutt
 *  Created on: March 1, 2017
 *  Modified: Feb 19, 2018
 **************************************************************************************************/

#ifndef USCIUART_H_
#define USCIUART_H_

      #define BUF_SIZE 100
      #define MAX_ARGS 4
//    #define MAX_CMDS 11
//    #define CMD_NARG 3
//    #define CMD2_NARG 1
//    #define CMD3_NARG 2
//    #define CMD4_NARG 0
//    #define CMD5_NARG 4
//    #define CMD6_NARG 1
//    #define CMD7_NARG 0
//    #define CMD8_NARG 0
//    #define CMD9_NARG 1
//    #define CMD10_NARG 0
// command name
//    #define CMD0 "pDir"
//    #define CMD1 "pOut"
//    #define CMD2 "p3Out" // assumes P3OUT configured as an output
//    #define CMD3 "nokLcdDrawScrnLine"
//    #define CMD4 "nokLcdClear"
//    #define CMD5 "nokLcdDrawLine"
//    #define CMD6 "fediHome"
//    #define CMD7 "fediClr"
//    #define CMD8 "fediRead"
//    #define CMD9 "fediDisp"
//    #define CMD10 "fediFw"
// global variables
    char rxBuffer[BUF_SIZE];
    volatile signed long int rxString[50];  // store commands from the terminal
    typedef struct CMD {
        const char* cmdName;
        int nArgs; // number of input arguments for a command
        signed long int args[MAX_ARGS]; // arguments
    }CMD;

// functions
    void usciA1UartInit();

    void usciA1UartTxChar(signed long int txChar);

    void usciA1UartTxString(unsigned char *txChar);

    char *usciA1UartGets(char *rxString);

#endif /* USCIUART_H_ */
