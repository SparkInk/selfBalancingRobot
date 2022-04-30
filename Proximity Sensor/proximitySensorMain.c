
/*********************************************************
 *
 *  This is a test bench for HC-SR04 proximity sensor
 *
 *  Author: Iakov Umrikhin
 *  Date Created: 15.03.2022
 *  Date Modified: 15.03.2022
 ********************************************************/

#include <msp430.h>
#include <stdio.h>
#include "usciUart.h"
#include "hcsr04.h"

volatile unsigned int countDiff;
volatile unsigned char distUpdate = 0;
int main(void)
{
	WDTCTL = WDTPW | WDTHOLD;	// stop watchdog timer

	volatile unsigned int distMeasured;
	volatile unsigned char buffer[15];

	// initialise UART
	usciA1UartInit();

	// initialise P1.4
	pin14Init();

	// initialise timerA0 with CCR3 in the capture mode
	timerA0Init();

	// enable the global interrupt
	__enable_interrupt();

	while (1) {
	    INIT_15us;

	    if (distUpdate) {
	        distMeasured = distance(countDiff);
			sprintf(buffer, "Distance: %.2f \n", distMeasured);
			usciA1UartTxString(buffer);
	        usciA1UartTxChar('\r');
	    }
	}

	return 0;
}

/************************************************
 *              TimerA1 ISR                     *
 *                                              *
 * Used to compute the speed of the flywheel    *
 * with the capture mode                        *
 ***********************************************/

#pragma vector = TIMER0_A1_VECTOR
__interrupt void timerA1ISR(void) {

    static volatile unsigned char ingressCount = 0;
    static volatile unsigned int newCounts;


    //disable interrupt
    TA0CCTL3 &= ~CCIE;
    if (!ingressCount) {    // positive edge triggered
        newCounts = TA0R;
        ingressCount = 1;
    }
    else {  // negative edge triggered
        countDiff = TA0R - newCounts;
        ingressCount = 0;
        distUpdate = 1;
    }

    // clear interrupt flag;
    TA0CCTL3 &= ~CCIFG;

    // enable interrupt
    TA0CCTL3 |= CCIE;
}
