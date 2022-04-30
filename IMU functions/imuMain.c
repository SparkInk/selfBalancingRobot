

/**************************************************************************
 *                             mpu6050TestBench.c
 *
 * This test bench is for MPU6050 sensor.
 *
 * Author: Iakov Umrikhin
 * Date Created: 14.03.2022 (dd:mm:yyyy)
 * Date Modified: 14.03.2022 (dd:mm:yyyy)
 **************************************************************************/

#include <msp430.h>
#include <math.h>
#include "usciI2C.h"
#include "imuHeader.h"
#include "imuCmd.h"
#include "fastClock.h"

int main(void)

{
	WDTCTL = WDTPW | WDTHOLD;	// stop watchdog timer

    volatile unsigned char rxString[50];  // store commands from the termial

	volatile signed char verStr[MPU_BUF_SIZE];

    int cmdIndex;
	IMU_CMD cmdList[MAX_IMU_CMDS];

	// Initialise 25 MHz clock

	fastClkInit();

	// initiallise motor driver
	pwmDriveInit();

	// initialise UART
	usciA1UartInit();

	// initialise I2C on B0
	usciB1I2CInit(SMCLK_DIV_10);

	// initialise CMD list
	initCmdList(cmdList);

	// enable NACK interrupt
	UCB1IFG &= ~UCNACKIFG;
	UCB1IE |= UCNACKIE;
	__enable_interrupt();

	// imuPowerOn
	executeCmd(cmdList, 4);

	// imuReadAcc
	executeCmd(cmdList, 1);

	while (1) {

		usciA1UartGets(rxString);
		cmdIndex = parseCmd(cmdList, rxString);
		executeCmd(cmdList, cmdIndex);
	}
	return 0;
}


// Temperature calculation

//  temperature_raw = ((int)verStr[1] << 8) | (0xff & verStr[2]);
//  temperature_well_done = ((double)temperature_raw /340) + 37;
