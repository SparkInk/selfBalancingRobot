

/*********************************************************
 *
 *  This is a source file for the HC-SR04 proximity sensor
 *
 *
 *  Author: Iakov Umrikhin
 *
 *  Date Created: 15.03.2022
 *  Date Modified: 15.03.2022
 ********************************************************/

#include <msp430.h>

/******************************************************************
 * Name: timerA0Init
 *
 * Description: Timer A0 is used for the capture mode to
 *              measure the time of the echoed signal;
 *              Channel 3 is used for the capture
 *
 * Arguments:   none
 *
 * Return: none
 *
 * Author: Iakov Umrikhin
 *
 * Date Created: 15.03.2022 (dd:mm:yyyy)
 * Date Modified: 15.03.2022 (dd:mm:yyyy)
 ******************************************************************/
void timerA0Init(void) {
    // for now we use ID_0; needs to be changed in the lab to save power
    TA0CTL |= TASSEL_2 | ID_3 | MC_2 | TACLR;   // SMCLK; division by 8; Continuous mode

    // TA0CCTL1 initialisation
    TA0CCTL3 |= CM_3 | CCIS_0 | SCS | CAP;  // capture pos. and neg. edges; CCI3A; Capture synchronise; Capture mode

    // enable interrupt on CCR1
    TA0CCTL3 |= CCIE;
    TA0EX0 |= TAIDEX_7; // additional division by 8

    // clear all pending interrupt flags
    TA0CCTL3 &= ~CCIFG;
    TA0CTL &= ~TAIFG;

}

/******************************************************************
 * Name: pin14Init
 *
 * Description: Initialises pins P1.4 (capture mode)
 *              and P1.5 (pulse generation)
 *
 * Arguments:   none
 *
 * Return: none
 *
 * Author: Iakov Umrikhin
 *
 * Date Created: 15.03.2022 (dd:mm:yyyy)
 * Date Modified: 15.03.2022 (dd:mm:yyyy)
 ******************************************************************/
void pin14Init(void) {

    P1DIR &= ~(BIT4);
    P1SEL |= BIT4;

    // P1.5 to generate a 15 us signal to initiate the sensor
    P1DIR |= BIT5;
    P1OUT &= ~BIT5; // Low initially
}

/******************************************************************
 * Name: distance
 *
 * Description: Calculates the distance
 *
 * Arguments: time  -  time duration of the echoed signal
 *
 * Return: dist -   measured distance in centimeters
 *
 * Author: Iakov Umrikhin
 *
 * Date Created: 15.03.2022 (dd:mm:yyyy)
 * Date Modified: 15.03.2022 (dd:mm:yyyy)
 ******************************************************************/
unsigned int distance(unsigned int time) {

    volatile unsigned int dist = 0;

    dist = (time / (double)16375) * (16500);

    return dist;
}
