
                            /************************************************
                            *  A command module for the MPU6050             *
                            *                                               *
                            *                                               *
                            * Author: Iakov (aka Iasha) Umrikhin            *
                            * Date Created: 15.03.2022                      *
                            * Date Modified: 17.05.2022                     *
                            ************************************************/

#include <msp430.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include "imuCmd.h"
#include "imuHeader.h"
#include "motorControl.h"
#include "quadEncDecode.h"

// Global variables
volatile unsigned char enterConsole;
volatile unsigned char pidChange;

ARROWS arrowKey;

/************************************************************************************
* Function: initCmdList
*
* Description: initialises commands to control the system
*
* Arguments: cmdList - list of commands
*
* Return: none
*
* Author: Iakov (aka Iasha) Umrikhin
* Date Created: 15.03.2022 (dd.mm.yyyy)
* Date Modified: 17.05.2022 (dd.mm.yyyy)
************************************************************************************/
void initCmdList(IMU_CMD* cmdList) {
//  whoAmI
    cmdList[0].cmdName = CMD0_IMU; // initialize the first command name;
    cmdList[0].nArgs = CMD0_IMU_NARG; // initialize num of arguments in first command

// repeat for all remaining valid commands

//  imuReadAcc
    cmdList[1].cmdName = CMD1_IMU;
    cmdList[1].nArgs = CMD1_IMU_NARG;

//  imuReadGyro
    cmdList[2].cmdName = CMD2_IMU;
    cmdList[2].nArgs = CMD2_IMU_NARG;

//  imuReadTemp
    cmdList[3].cmdName = CMD3_IMU;
    cmdList[3].nArgs = CMD3_IMU_NARG;

//  imuPowerOn
    cmdList[4].cmdName = CMD4_IMU;
    cmdList[4].nArgs = CMD4_IMU_NARG;

//  systemLaunch
    cmdList[5].cmdName = CMD5_IMU;
    cmdList[5].nArgs = CMD5_IMU_NARG;

//  imuAccelRange
    cmdList[6].cmdName = CMD6_IMU;
    cmdList[6].nArgs = CMD6_IMU_NARG;

//  imuGyroRange
    cmdList[7].cmdName = CMD7_IMU;
    cmdList[7].nArgs = CMD7_IMU_NARG;

}
/************************************************************************************
* Function: parseCmd
*
* Description: parseCmd parses command (cmdLine) received from the console and
*              stores arguments in cmdList->args[MAX_CMDS]
*
* Arguments: cmdList - list of commands
*            cmdLine - pointer to the request (or command) entered;
*
* Return: the index of a command if it is valid; -1 otherwise;
*
* Author: Iakov (aka Iasha) Umrikhin
* Date: 16.01.2022 (dd.mm.yyyy)
* Modified: 17.01.2022 (dd.mm.yyyy)
************************************************************************************/
int parseCmd(IMU_CMD * cmdList , char * cmdLine) {
    char* token;
    int valid = -1;
    int counter = 0;
    char *pEnd; // to use only for strtol

    token = strtok(cmdLine, " ");

    // validating command name
    int index = validateCmd(cmdList, token); // stores the index of the command name
    if (index != -1) {
        // validating the number of arguments
        while (token != NULL) {
            token = strtok(NULL, " ");
            if (token == 0x0d) break;   // breaks if '\r' is found
            if (token != NULL && isxdigit(*token) && !isalpha(*token)) {    // for numbers only
                // isxdigit returns > 0 if a hex number; return > 0 if true
                cmdList->args[counter] = atoi(token, &pEnd, 16);   // numArg is a string; convert to an integer
                counter++;
            }
            else if (token != NULL & isalpha(*token)) {   // for registers' names
                cmdList->encRegName = token;
            }
        }
        // varifying if nArgs is equal to the actual number of arguments in the data entered
        if (cmdList[index].nArgs == counter) {
            valid = index;
        }
        else {
            valid = -1;
        }
    }
    else {
        valid = -1;
    }

    cmdLine = 0;   //  clear the rxString for the next entry


    return valid;

}
/************************************************************************************
* Function: validateCmd
*
* Description: validateCmd validates the command entered exists in the cmdList
*
* Arguments: cmdList - list of commands
*            cmdName - pointer to a command entered
*
* Return: index of a command
*
* Author: Iakov (aka Iasha) Umrikhin
* Date: 16.01.2022 (dd.mm.yyyy)
* Modified: 17.01.2022 (dd.mm.yyyy)
************************************************************************************/
int validateCmd(IMU_CMD* cmdList, char* cmdName) {

    int i = 0;
    int idx = -1;
    int invalidCmd = 1;

    while (invalidCmd && i < MAX_IMU_CMDS) {
        invalidCmd = strcmp(cmdName, cmdList[i++].cmdName);     // returns 0 if true
    }

    if (!invalidCmd) {
        idx = i - 1;
    }

    return idx;
}
/************************************************************************************
 * Function: executeCMD
 *
 * Description: executeCMD uses cmdIndex to execute a command from cmdList->cmdName[cmdIndex].
 *
 * Arguments: cmdList - list of commands
  *           cmdIndex - integer index to a command from cmdList
 *
 * Return: 0 if it is valid; -1 otherwise;
 *
 * Author: Iakov (aka Iasha) Umrikhin
 * Date: 16.01.2022 (dd.mm.yyyy)
 * Modified: 17.05.2022 (dd.mm.yyyy)
 ************************************************************************************/

 int executeCmd(IMU_CMD * cmdList, int cmdIndex) {

     volatile int status = -1;
     volatile signed char dataIn[MPU_BUF_SIZE];
     volatile unsigned char dispBuff[30];
     volatile signed int temperature_raw;
     volatile signed int temperature_well_done;

     volatile signed int phi;   // accelerometer's angle
     volatile static signed int phiOffset;
     volatile signed int theta; // gyroscope's angle

     volatile static char oneShot = 0;

     // static variables for MPU6050
     volatile static signed int acc_x_raw;
     volatile static signed int acc_y_raw;
     volatile static signed int acc_z_raw;

     volatile static signed int gyro_x_raw;
     volatile static signed int gyro_y_raw;
     volatile static signed int gyro_z_raw;

     IMU_ACC accData;
     IMU_GYRO gyroData;

     // PID gains
     static PID pidGain;

     switch(cmdIndex) {

     case 0:    // whoAmI

         IMUSendByte(MPU6050_ADDR, WHO_AM_I, 0, 1);
         IMURead(1, MPU6050_ADDR, dataIn);

         sprintf(dispBuff, "Slave's address: 0x%x \n", dataIn[0]);
         usciA1UartTxString(dispBuff);
         break;

     case 1:    // imuReadAcc

         while (!enterConsole) {
             UCA1IE |= UCRXIE;
             IMUSendByte(MPU6050_ADDR, ACC_XOUT_H, 0, 1);
             IMURead(6, MPU6050_ADDR, dataIn);

//           store gyro_out
             acc_x_raw = (dataIn[0] << 8) | (0xff & dataIn[1]);
             acc_y_raw = (dataIn[2] << 8) | (0xff & dataIn[3]);
             acc_z_raw = (dataIn[4] << 8) | (0xff & dataIn[5]);

             sprintf(dispBuff, "Acc X: %d  Acc Y:  %d  Acc Z: %d  \r\n",
                                acc_x_raw,
                                            acc_y_raw,
                                                         acc_z_raw);
             usciA1UartTxString(dispBuff);

         }
         // disable interrupt
         UCA1IE &= ~UCRXIE;

//       clear the gyroscope data
         acc_x_raw = 0;
         acc_y_raw = 0;
         acc_z_raw = 0;

//       clear enterConsole
         enterConsole = 0;
         break;


         break;

     case 2:    // imuReadGyro

         while (!enterConsole) {
             UCA1IE |= UCRXIE;
             IMUSendByte(MPU6050_ADDR, GYRO_XOUT_H, 0, 1);
             IMURead(6, MPU6050_ADDR, dataIn);

//           store gyro_out
             gyro_x_raw = (dataIn[0] << 8) | (0xff & dataIn[1]);
             gyro_y_raw = (dataIn[2] << 8) | (0xff & dataIn[3]);
             gyro_z_raw = (dataIn[4] << 8) | (0xff & dataIn[5]);

             sprintf(dispBuff, "Gyro X: %d  Gyro Y:  %d  Gyro Z: %d  \r\n",
                                gyro_x_raw,
                                            gyro_y_raw,
                                                         gyro_z_raw);
             usciA1UartTxString(dispBuff);

         }
         // disable interrupt
         UCA1IE &= ~UCRXIE;

//       clear the gyroscope data
         gyro_x_raw = 0;
         gyro_y_raw = 0;
         gyro_z_raw = 0;

//       clear enterConsole
         enterConsole = 0;
         break;


     case 3:    // imuReadTemp

        while (!enterConsole) {

            UCA1IE |= UCRXIE;

            // read temperature data
            IMUSendByte(MPU6050_ADDR, TEMP_OUT_H, 0, 1);
            IMURead(2, MPU6050_ADDR, dataIn);
        }
         break;

     case 4:    // imuPowerOn

         // set gyroscope range
         IMUSendByte(MPU6050_ADDR, GYRO_CONFIG, GYRO_250, 0);
         usciA1UartTxString("Range +/- 250 deg/s is set \n");
         newLine();

         // set accelerometer range
         IMUSendByte(MPU6050_ADDR, ACCEL_CONFIG, ACCEL_2G, 0);
         usciA1UartTxString("Range +/- 2g is set\n");
         newLine();

         // set the low-pass filter
         IMUSendByte(MPU6050_ADDR, CONFIG, BW_21_ACC, 0);
         usciA1UartTxString("Bandwidth 21 Hz is set\n");
         newLine();

         // set NORMAL power mode
         IMUSendByte(MPU6050_ADDR, MPU_PWR_MGMT_1, NORMAL_MODE, 0);
         usciA1UartTxString("Power On Complete\n");
         newLine();

         break;

     case 5:    // systemLaunch

         // NOTE: when initialising this command
         //       keep the MPU orientation such that Oz points upwards;
         while (!enterConsole && !stopBit) {

             // enable UART interrupt
             UCA1IE |= UCRXIE;

//           use oneShot to set a range for Gyro and Accelerometer
             if (!oneShot) {

//               store raw data
                 IMUSendByte(MPU6050_ADDR, ACC_XOUT_H, 0, 1);
                 IMURead(6, MPU6050_ADDR, dataIn);

                 // store acc_out_raw for the offset
                 acc_x_raw = (dataIn[0] << 8) | (0xff & dataIn[1]);
                 acc_y_raw = (dataIn[2] << 8) | (0xff & dataIn[3]);
                 acc_z_raw = (dataIn[4] << 8) | (0xff & dataIn[5]);

                 phiOffset = atan2(acc_z_raw, acc_x_raw) * 573;

                 // wait to allow the I2C to finish transmission
                 WAIT_1ms;

                 // clear the data-storing buffer
                 memset(dataIn, 0, 10);

//               request the gyro_data
                 IMUSendByte(MPU6050_ADDR, GYRO_XOUT_H, 0, 1);
                 IMURead(6, MPU6050_ADDR, dataIn);

//               store gyro_out for the offset
                 gyro_y_raw = (dataIn[2] << 8) | (0xff & dataIn[3]);

                 // clear the data-storing buffer
                 memset(dataIn, 0, 10);

                 oneShot = 1;

             }

             // delay to 2 ms
             WAIT_2ms;

             IMUSendByte(MPU6050_ADDR, ACC_XOUT_H, 0, 1);
             IMURead(6, MPU6050_ADDR, dataIn);

//               store acc_out_raw for the offset
             accData.x_well_done = (dataIn[0] << 8) | (0xff & dataIn[1]);
             accData.y_well_done = (dataIn[2] << 8) | (0xff & dataIn[3]);
             accData.z_well_done = (dataIn[4] << 8) | (0xff & dataIn[5]);

             // wait to allow the I2C to finish transmission
             WAIT_1ms;

//           request the gyro_data
             IMUSendByte(MPU6050_ADDR, GYRO_XOUT_H, 0, 1);
             IMURead(6, MPU6050_ADDR, dataIn);

//           store the real data (Oy)
             gyroData.y_well_done = (dataIn[2] << 8) | (0xff & dataIn[3]);

//           offset the gyro data (Oy)
             gyroData.y_well_done -= gyro_y_raw;

//           convert gyro angle from bits to deg/s
             theta = mapGyro(gyroData.y_well_done);


//           compute the angle (Oz, Ox); amplified by 10
             phi = atan2(accData.z_well_done, accData.x_well_done) * 573;
             phi -= phiOffset;

//           compute the angular displacement (Oy) due to the acceleration
//           filter phi by means of the complementary filter
             phi = lowPassFilter(phi, theta);

//           correct the angle(Oz, Ox) for the angular displacement

//           send an angle to the motors
             updateMotorSpeed(pidCompute(phi, &pidGain));

//           display the angle(Oz, Ox)
             sprintf(dispBuff,
                     "Angle:   %d  PID:   %d   Gyro(deg/s): %d  \r\n",
                     (phi / 10),
                          pidCompute(phi, &pidGain),
                              theta);
             sprintf(dispBuff, "Angle:   %d \n", (phi / 10));
             usciA1UartTxString(dispBuff);
             newLine();

             memset(dispBuff, 0 ,30);

         }  // while loop end

         // disable interrupt
         UCA1IE &= ~UCRXIE;

//       clear the accelerometer's data and gyroscope's data
         IMU_ACC accData;
         IMU_GYRO gyroData;

//       clear enterConsole
         enterConsole = 0;

         if (stopBit) {
             usciA1UartTxString("You idiot, you've got a NACK \r\n");
         }

//       clear oneShot
         oneShot = 0;
         stopBit = 0;

         break;

     case 6:    // imuAccelRange
         switch(cmdList->args[0]) {   // choose a range
         case 0:    // +/- 2g

             IMUSendByte(MPU6050_ADDR, ACCEL_CONFIG, ACCEL_2G, 0);
             usciA1UartTxString("Range +/- 2g is set\n");
             newLine();

             break;
         case 1:    // +/- 4g

             IMUSendByte(MPU6050_ADDR, ACCEL_CONFIG, ACCEL_4G, 0);
             usciA1UartTxString("Range +/- 4g is set\n");
             newLine();

             break;
         case 2:    // +/- 8g

             IMUSendByte(MPU6050_ADDR, ACCEL_CONFIG, ACCEL_8G, 0);
             usciA1UartTxString("Range +/- 8g is set\n");
             newLine();

             break;
         case 3:    // +/- 16g

             IMUSendByte(MPU6050_ADDR, ACCEL_CONFIG, ACCEL_16G, 0);
             usciA1UartTxString("Range +/- 16g is set\n");
             newLine();

             break;
         }
         break;

     case 7:    // imuGyroRange
         switch(cmdList->args[0]) {
         case 0:    // +/- 250 rad/s

            IMUSendByte(MPU6050_ADDR, GYRO_CONFIG, GYRO_250, 0);
            usciA1UartTxString("Range +/- 250 rad/s is set \n");
            newLine();

            break;
        case 1:     // +/- 500 rad/s

            IMUSendByte(MPU6050_ADDR, GYRO_CONFIG, GYRO_500, 0);
            usciA1UartTxString("Range +/- 500 rad/s is set \n");
            newLine();

            break;
        case 2:     // +/- 1000 rad/s

            IMUSendByte(MPU6050_ADDR, GYRO_CONFIG, GYRO_1000, 0);
            usciA1UartTxString("Range +/- 1000 rad/s is set \n");
            newLine();

            break;
        case 3:     // +/0 2000 rad/s

            IMUSendByte(MPU6050_ADDR, GYRO_CONFIG, GYRO_2000, 0);
            usciA1UartTxString("Range +/- 2000 rad/s is set \n");
            newLine();

            break;
         }
         break;
     }

     return status;  // returns -1 if there is an error; 0 if all good!!

 }
 /************************************************************************************
 * Function: newLine
 *
 * Description: goes to a new line in the console
 *
 * Arguments: none
 *
 * Return: none
 *
 * Author: Iakov (aka Iasha) Umrikhin
 * Date: 19.03.2022 (dd.mm.yyyy)
 * Modified: 19.03.2022 (dd.mm.yyyy)
 ************************************************************************************/

 void newLine(void) {

     usciA1UartTxChar('\r');
     usciA1UartTxChar('\n');

 }

 /************************************************************************************
  * Function: lowPassFilter
  *
  * Description: 
  *
  * Arguments: actualAngle - a current, raw angle from the accelerometer; deg
  *            gyro     -   a mapped angle from the gyroscope; deg/s
  *
  * Return: angle - filtered angle
  *
  * Author: Iakov (aka Iasha) Umrikhin
  * Date: 22.04.2022 (dd.mm.yyyy)
  * Modified: 07.05.2022 (dd.mm.yyyy)
  ************************************************************************************/
 signed int lowPassFilter(signed int actualAngle, signed int gyro) {

    volatile signed int filteredAngle;
    volatile static signed int prevAngle;
    volatile float dT = 0.05;    // time to integrate over is 100 ms
    volatile float gyroAngle;
    volatile float weight = 0.95;

    // calculate the angle from the gyroscope (Oy)
    gyroAngle = (float)gyro * dT;

    // filter the angle
    filteredAngle = weight * (gyroAngle + prevAngle) + (1 - weight) * actualAngle;

    // record the filtered angle 
    prevAngle = filteredAngle;

    return filteredAngle;
 }
 /************************************************************************************
  * Function: pidCompute
  *[
  * Description: computes the PID values for the control loop
  *
  * Arguments: currAngle  -   current robot's angle
  *            pidGain    -   a pointer to a data structure containing PID gains
  *
  * Return: pidResult - a combine result of P, I, and D components
  *
  * Author: Iakov (aka Iasha) Umrikhin
  * Date: 22.04.2022 (dd.mm.yyyy)
  * Modified: 17.05.2022 (dd.mm.yyyy)
  ************************************************************************************/

signed int pidCompute(signed int currAngle, PID *pidGain) {

    volatile static signed int setAngle = 0;
    volatile signed int error;
    volatile static signed int prevError = 0;
    volatile static signed int aforePrevError = 0;
    volatile float dT = 0.05;    // time to integrate over is 50 ms
    volatile static unsigned char oneShot = 1;
    volatile unsigned char pidBuff[20];

    // PID components
    volatile signed int pComp;
    volatile signed int iComp = 0;
    volatile signed int dComp;

    volatile signed int pidResult;
    volatile static signed int pidResultOld = 0;

    if (oneShot){

        pidGain->Kp = 30;
        pidGain->Ki = 0;
        pidGain->Kd = 3.5;

        oneShot = 0;
    }

    // if one of the keys is pressed modify PID gains
    if (pidChange){
        changePID(pidGain);
        pidChange = 0;
    }

    // calculate the error
    error = setAngle - currAngle;

    // compute gains
    // proportional gain
    pComp = (error - prevError);

    // integral gain
    iComp = (error + prevError) >> 1;
    iComp = constrain(iComp, -300, 300);

    // derivative gain
    dComp = (error - 2 * prevError + aforePrevError) / dT;
    dComp = constrainf(dComp, -300, 300);

    // compute the PID output result
    pidResult = pidResultOld + (pidGain->Kp * pComp + pidGain->Ki * iComp * dT + pidGain->Kd * dComp) / 10;

    aforePrevError = prevError;
    prevError = error;
    pidResultOld = pidResult;

    // shut the motors down after 45 degrees; reset static variables
    // angle is amplified by 10
    if (abs(currAngle) > 450) {

        pidResult = 0;

    }
    else {
        pidResult = constrain(pidResult, -500, 500);
        pidResultOld = constrain(pidResultOld, -500, 500);
    }

    return pidResult;
 }

/***************************************************************************************
 * Name: changePID
 * 
 * Description: this function is responsible for changing the PID gains on the fly.
 *              UP arrow increments the desired gain; DOWN arrow decrements.
 *              NUM_1   ->  Kp gain <- incremented by 1
 *              NUM_2   ->  Ki gain <- incremented by 1
 *              NUM_3   ->  Kd gain <- incremented by 5; amplified by 100 to tune 
 *                                     values up to 100th digit
 *              
 * Arguments: pid   -   a pointer to a data structure containing PID gains
 * 
 * Return: Unfortunately, this function is poor, and cannot return anything
 *         at this moment.
 * 
 * Author: Iakov (aka Iasha) Umrikhin
 * 
 * Date Created: 08.05.2022 (dd::mm::yyyy)
 * Date Modified: 17.05.2022 (dd::mm::yyyy)
 * 
 * ************************************************************************************/
void changePID(PID *pid) {

    volatile char pidBuff[50];
    volatile signed int kpOld, kiOld, kdOld;

    clearArrowKeys(&arrowKey);

    // scale up to 100th
    pid->Kd *= 100;

    kpOld = pid->Kp;
    kiOld = pid->Ki;
    kdOld = pid->Kd;


    // print a new line to separate a current routine from the previous ones
    newLine();
    usciA1UartTxString("Choose a gain to tune: 1 - Kp; 2 - Ki; 3 - Kd \r\n");
    newLine();

    while (UCA1RXBUF != 0x73) {     // this is active until the letter S is not pressed

        // enable interrupt
        UCA1IE |= UCRXIE;

        switch (arrowKey.bitPID){
        case 1:     // change Kp gain

            // if the UP key is pressed - increment; if the DOWN key is pressed - decrement
            // else - don't change the variable's value
            pid->Kp = arrowKey.up ? (pid->Kp + 1) : arrowKey.down ? (pid->Kp - 1) : pid->Kp;
            sprintf(pidBuff, "Kp:   %d        \r\n", pid->Kp);
            usciA1UartTxString(pidBuff);

            // to debounce the keypad button; because it needs to increment only by 1
            if (abs(kpOld - pid->Kp) >= 1) {
                clearArrowKeys(&arrowKey);
                kpOld = pid->Kp;
            }
            break;
        case 2:     // change Ki gain
            pid->Ki = arrowKey.up ? (pid->Ki + 1) : arrowKey.down ? (pid->Ki - 1) : pid->Ki;

            sprintf(pidBuff, "Ki:  %d        \r\n", pid->Ki);
            usciA1UartTxString(pidBuff);

            if (abs(kiOld - pid->Ki) >= 1) {
                clearArrowKeys(&arrowKey);
                kiOld = pid->Ki;
            }
            break;
        case 3:     // change Kd gain
            pid->Kd = arrowKey.up ? (pid->Kd + 5): arrowKey.down ? (pid->Kd - 5) : pid->Kd;
            
            sprintf(pidBuff, "Kd (x0.01):   %.0f        \r\n", pid->Kd);
            usciA1UartTxString(pidBuff);

            // to debounce the keypad button; because it needs to increment only by 5
            if (abs(kdOld - pid->Kd) >= 5) {
                clearArrowKeys(&arrowKey);
                kdOld = pid->Kd;
            }
            break;
        }

    }

    // scale back
    pid->Kd /= 100;

    arrowKey.bitPID = 0;

    // print out the result of PID change
    sprintf(pidBuff, "Kp:   %d   Ki:    %d  Kd:     %f.2  \r\n", pid->Kp, pid->Ki, pid->Kd);
    usciA1UartTxString(pidBuff);
    newLine();
}
/***************************************************************************************
 * Name: changePID
 * 
 * Description: clears whatever values each arrow-key variable currently has
 *
 * 
 * Arguments: arrowKey - structure containing bools for arrow keys
 * 
 * Return: You kid me not! Nothing in return
 * 
 * Author: Iakov (aka Iasha) Umrikhin
 * 
 * Date Created: 08.05.2022 (dd::mm::yyyy)
 * Date Modified: 08.05.2022 (dd::mm::yyyy)
 * 
 * ************************************************************************************/
void clearArrowKeys(ARROWS *arrow) {
    arrow->up = 0;
    arrow->down = 0;
}

/***************************************************************************************
 * Name: detectArrowKey
 *
 * Description: detects if one of the arrow keys has been pressed;
 *              if true, assigns 1 to a variable pertaining to the key
 *
 *
 * Arguments: arrowKey - structure containing variables for each arrow key
 *
 * Return: You kid me not! Nothing in return
 *
 * Author: Iakov (aka Iasha) Umrikhin
 *
 * Date Created: 08.05.2022 (dd::mm::yyyy)
 * Date Modified: 08.05.2022 (dd::mm::yyyy)
 *
 * ************************************************************************************/
void detectArrowKey(ARROWS *arrow) {

    switch(UCA1RXBUF){

        case 0x41:              // UP key is pressed
            arrow->up = 1;
            break;
        case 0x42:              // DOWN key is pressed
            arrow->down = 1;
            break;
        case 0x31:              // NUM_1 is pressed
            arrow->bitPID = 1;
            break;
        case 0x32:              // NUM_2 is pressed
            arrow->bitPID = 2;
            break;
        case 0x33:              // NUM_3 is pressed
            arrow->bitPID = 3;
            break;
    }

}
/***************************************************************************************
 * Name: constrain
 *
 * Description: this function constrains values between lowBoundary and hiBoundary
 *              for integers.
 *
 * Arguments: modifiedVal   -   the float-point value to constrain
 *            lowBoundary   -   self-explanatory
 *            highBoundary  -   self-explanatory
 *
 * Return: if modifiedVal > highBoudary -> highBoundary; if modifiedVal < lowBoundary;
 *         otherwise -> modifiedVal. 
 *
 * Author: Iakov (aka Iasha) Umrikhin
 *
 * Date Created: 28.04.2022 (dd::mm::yyyy)
 * Date Modified: 17.05.2022 (dd::mm::yyyy)
 *
 * ************************************************************************************/

signed int constrain(signed int modifiedVar, signed int lowBoundary, signed int highBoundary) {

    if (modifiedVar > highBoundary) {
        modifiedVar = highBoundary;
    }
    else if (modifiedVar < lowBoundary) {
        modifiedVar = lowBoundary;
    }
    return modifiedVar;

}
/***************************************************************************************
 * Name: constrainf
 * 
 * Description: this function constrains values between lowBoundary and highBoundary
 *              for float-point numbers.
 * 
 * Arguments: modifiedVal   -   the float-point value to constrain
 *            lowBoundary   -   self-explanatory
 *            highBoundary  -   self-explanatory
 * 
 * Return: if modifiedVal > highBoudary -> highBoundary; if modifiedVal < lowBoundary;
 *         otherwise -> modifiedVal.
 * 
 * Author: Iakov (aka Iasha) Umrikhin
 * 
 * Date Created: 28.04.2022 (dd::mm::yyyy)
 * Date Modified: 17.05.2022 (dd::mm::yyyy)
 * 
 * ************************************************************************************/

float constrainf(float modifiedVar, signed int lowBoundary, signed int highBoundary) {

    if (modifiedVar > highBoundary) {
        modifiedVar = highBoundary;
    }
    else if (modifiedVar < lowBoundary) {
        modifiedVar = lowBoundary;
    }
    return modifiedVar;

}

/***************************************************************************************
 * Name: mapGyro()
 * 
 * Description: this function convert the gyroscopes data from bits to deg/s
 *              It is amplified by 10, to match with the accelerometer's angles
 *              that are also multiplied by 10.
 *
 * 
 * Arguments: unmappedVal   -   raw data from the gyroscope; Oy axis
 * 
 * Return: amplified by 10 angular speed in deg/s
 * 
 * Author: Iakov (aka Iasha) Umrikhin
 * 
 * Date Created: 29.04.2022 (dd::mm::yyyy)
 * Date Modified: 17.05.2022 (dd::mm::yyyy)
 * 
 * ************************************************************************************/
signed int mapGyro(signed int unmappedVal) {

    // gyro goes from +- 26624 => 250 / 32768 ~= 0.0094
    unmappedVal *=  0.094;      // amplified by 10

    return unmappedVal;
}


/********************************
 *      UART Interrupt          *
 *                              *
 * Used to trigger on ENTER key *
 ********************************/

#pragma vector = USCI_A1_VECTOR
 __interrupt void uartISR (void) {

     // disable interrupts
     UCA1IE &= ~UCRXIE;

     switch(UCA1RXBUF) {
        case 0x70:              // letter P is pressed
            pidChange = 1;
            break;

        case 0x0d:
            enterConsole = 1;   // ENTER key is pressed
            break;
     }

     detectArrowKey(&arrowKey);

     // clear interrupt flags
     UCA1IFG &= ~UCRXIFG;

 }
