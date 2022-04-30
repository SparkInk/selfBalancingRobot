
                            /*************************************
                            *  A command module for the MPU6050  *
                            *                                    *
                            *                                    *
                            * Author: Iakov Umrikhin             *
                            * Date Created: 15.03.2022           *
                            * Date Modified: 15.03.2022          *
                            **************************************/

#include <msp430.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include "imuCmd.h"
#include "imuHeader.h"
#include "motorControl.h"

// Global variables
unsigned char enterConsole;

/************************************************************************************
* Function: initCmdList
*
* Description:
*
* Arguments: cmdList - list of commands
*
* Return: none
*
* Author: Iakov Umrikhin
* Date Created: 15.03.2022 (dd.mm.yyyy)
* Date Modified: 15.03.2022 (dd.mm.yyyy)
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

//  imuSendByte
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
* Author: Iakov Umrikhin
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
* Author: Iakov Umrikhin
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
 * Author: Iakov Umrikhin
 * Date: 16.01.2022 (dd.mm.yyyy)
 * Modified: 21.03.2022 (dd.mm.yyyy)
 ************************************************************************************/

 int executeCmd(IMU_CMD * cmdList, int cmdIndex) {

     volatile int status = -1;
     volatile signed char dataIn[MPU_BUF_SIZE];
     volatile unsigned char dispBuff[30];
     volatile signed int temperature_raw;
     volatile signed int temperature_well_done;

     volatile signed int phi;
     volatile signed int gyroAngleDisp = 0;
     volatile static char oneShot = 0;

     // static variables
     volatile static float acc_x_raw;
     volatile static float acc_y_raw;
     volatile static float acc_z_raw;

     volatile static float gyro_x_raw;
     volatile static float gyro_y_raw;
     volatile static float gyro_z_raw;

     IMU_ACC accData;
     IMU_GYRO gyroData;

     switch(cmdIndex) {

     case 0:    // whoAmI

         IMUSendByte(MPU6050_ADDR, WHO_AM_I, 0, 1);
         IMURead(1, MPU6050_ADDR, dataIn);

         sprintf(dispBuff, "Slave's address: 0x%x \n", dataIn[0]);
         usciA1UartTxString(dispBuff);
         break;

     case 1:    // imuReadAcc
         // NOTE: when initialising this command
         //       keep the MPU orientation such that Oz points upwards;
         while (enterConsole != '\r') {

             UCA1IE |= UCRXIE;  // enable the UART interrupt to stop displaying
                                // accelerometer data

//           use oneShot to set a range for Gyro and Accelerometer
             if (!oneShot) {

//               store raw data
                 IMUSendByte(MPU6050_ADDR, ACC_XOUT_H, 0, 1);
                 IMURead(6, MPU6050_ADDR, dataIn);

                 // store acc_out_raw for the offset
                 acc_x_raw = (dataIn[0] << 8) | (0xff & dataIn[1]);
                 acc_y_raw = (dataIn[2] << 8) | (0xff & dataIn[3]);
                 acc_z_raw = (dataIn[4] << 8) | (0xff & dataIn[5]);

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
             else {
                 // delay to 1 ms
                  WAIT_100ms;

                 IMUSendByte(MPU6050_ADDR, ACC_XOUT_H, 0, 1);
                 IMURead(6, MPU6050_ADDR, dataIn);

//               store acc_out_raw for the offset
                 accData.x_well_done = (dataIn[0] << 8) | (0xff & dataIn[1]);
                 accData.y_well_done = (dataIn[2] << 8) | (0xff & dataIn[3]);
                 accData.z_well_done = (dataIn[4] << 8) | (0xff & dataIn[5]);

//               offset the acc_data
//               accData.x_well_done -= acc_x_raw;
                 accData.y_well_done -= acc_y_raw;
                 accData.z_well_done -= acc_z_raw;

//               request the gyro_data
                 IMUSendByte(MPU6050_ADDR, GYRO_XOUT_H, 0, 1);
                 IMURead(6, MPU6050_ADDR, dataIn);

//               store the real data (Oy)
                 gyroData.y_well_done = (dataIn[2] << 8) | (0xff & dataIn[3]);

//               offset the gyro data (Oy)
                 gyroData.y_well_done -= gyro_y_raw;

//               convert from +- 2000 deg/s to 250 deg/s
                 gyroData.y_well_done = mapGyro(gyroData.y_well_done);

             }

//          compute the angle (Oz, Ox)
            phi = atan2(accData.z_well_done, accData.x_well_done) * 57.3;

//          compute the angular displacement (Oy) due to the acceleration
        //    gyroAngleDisp = gyroData.x_well_done * dT;

//          filter phi by means of the complementary filter            
            phi = lowPassFilter(&accData, phi, gyroData.y_well_done);

//          correct the angle(Oz, Ox) for the angular displacement
            // newPhi = 90 + phi - gyroAngleDisp;



//          send an angle to the motors
            updateMotorSpeed(pidCompute(phi));

 //          display the angle(Oz, Ox)
              sprintf(dispBuff,
                    "Angle: %d    ACC_X: %d   ACC_Y: %d  ACC_Z: %d    \r\n",
                         phi,
                         pidCompute(phi),
                             gyroData.y_well_done,
                                 accData.z_well_done);
              usciA1UartTxString(dispBuff);

              memset(dispBuff, 0 ,30);

         }  // while loop end

//       clear the accelerometer's data and gyroscope's data
         IMU_ACC accData;
         IMU_GYRO gyroData;

//       clear enterConsole
         enterConsole = 0;

//       clear oneShot
         oneShot = 0;

         break;

     case 2:    // imuReadGyro

         while (enterConsole != '\r') {
             UCA1IE |= UCRXIE;
             IMUSendByte(MPU6050_ADDR, GYRO_XOUT_H, 0, 1);
             IMURead(6, MPU6050_ADDR, dataIn);

//           store gyro_out
             gyro_x_raw = (dataIn[0] << 8) | (0xff & dataIn[1]);
             gyro_y_raw = (dataIn[2] << 8) | (0xff & dataIn[3]);
             gyro_z_raw = (dataIn[4] << 8) | (0xff & dataIn[5]);

             sprintf(dispBuff, "Gyro X: %d  Gyro Y:  %d  Gyro Z: %d  \n",
                                gyro_x_raw,
                                            gyro_y_raw,
                                                         gyro_z_raw);
         }

//       clear the gyroscope data
         gyro_x_raw = 0;
         gyro_y_raw = 0;
         gyro_z_raw = 0;

//       clear enterConsole
         enterConsole = 0;
         break;


     case 3:    // imuReadTemp

         IMUSendByte(MPU6050_ADDR, TEMP_OUT_H, 0, 1);
         IMURead(2, MPU6050_ADDR, dataIn);

         break;

     case 4:    // imuPowerOn

         // set gyroscope range
         IMUSendByte(MPU6050_ADDR, GYRO_CONFIG, GYRO_250, 0);
         usciA1UartTxString("Range +/- 250 deg/s is set \n");
         newLine();

         // set accelerometer range
         IMUSendByte(MPU6050_ADDR, ACCEL_CONFIG, ACCEL_16G, 0);
         usciA1UartTxString("Range +/- 16g is set\n");
         newLine();

        //  // set the low-pass filter
        //  IMUSendByte(MPU6050_ADDR, CONFIG, BW_44_ACC, 0);
        //  usciA1UartTxString("Bandwidth 44 Hz is set\n");
        //  newLine();

         // set NORMAL power mode
         IMUSendByte(MPU6050_ADDR, MPU_PWR_MGMT_1, NORMAL_MODE, 0);
         usciA1UartTxString("Power On Complete\n");
         newLine();

         break;

     case 5:    // imuSendByte


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
 * Description: goes to a new line
 *
 * Arguments: none
 *
 * Return: none
 *
 * Author: Iakov Umrikhin
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
  * Arguments: angle - a current, raw angle from the accelerometre
  *
  * Return: angle - filtered angle
  *
  * Author: Iakov Umrikhin
  * Date: 22.04.2022 (dd.mm.yyyy)
  * Modified: 22.04.2022 (dd.mm.yyyy)
  ************************************************************************************/
 signed int lowPassFilter(IMU_GYRO *angle, signed int acctualAngle, signed int gyro) {

    volatile float filteredAngle;
    volatile static signed int prevAngle;
    volatile float dT = 0.001;    // time to integrate over is 10 ms
    volatile float gyroAngle;

    volatile float magnitude;
    volatile float weight;

    // compute the magnitude of the accelerometer in terms of g
    angle->x_well_done = constrain(angle->x_well_done, -2048, 2048);
    angle->z_well_done = constrain(angle->z_well_done, -2048, 2048);

    magnitude = sqrt(pow((angle->x_well_done / (float)2048), 2)  + pow((angle->z_well_done / (float)2048), 2));

    weight = 1 - 5 * fabsf(1 - magnitude);

    if (weight > 1) {
        weight = 1;
    }
    else if (weight < 0) {
        weight = 0;
    }
    weight /= 10;

    // calculate the angle from the gyroscopes
    gyroAngle = gyro * dT;

    // filter the angle;
    
    filteredAngle = (1 - weight) * (prevAngle + gyroAngle) + weight * acctualAngle;

    prevAngle = filteredAngle;

     return filteredAngle;
 }
 /************************************************************************************
  * Function: pidCompute
  *[
  * Description: computes the PID values for the control loop
  *
  * Arguments: currAngle - current robot's angle
  *
  * Return: pidResult - a combine result of P, I, and D components
  *
  * Author: Iakov Umrikhin
  * Date: 22.04.2022 (dd.mm.yyyy)
  * Modified: 22.04.2022 (dd.mm.yyyy)
  ************************************************************************************/

signed int pidCompute(signed int currAngle) {

    volatile signed int setAngle = 0;
    volatile static signed int prevAngle = 0;

    volatile float error;
    volatile static float prevError = 0;
    volatile float dT = 0.1;    // time to integrate over is 1 ms

    // PID components
    volatile static float iComp = 0;
    volatile float dComp;

    volatile signed int pidResult;

    // PID gains
    volatile float Kp = 60;
    volatile float Ki = 15;
    volatile float Kd = 0.7; 

    // calculate the error
    error = setAngle - currAngle;

    // compute gains

    // integral gain
    iComp += error;
    iComp = constrain(iComp, -50, 50);

    // derivative gain; save the current angle value
    dComp = (error - prevError);
    prevError = error;

    // compute the PID output result; scale for the maximum error = 90 deg
    pidResult = (Kp * error + Ki * iComp + Kd * dComp);

    if (abs(currAngle) > 45) {
        pidResult = 0;
    }
    else {
        pidResult = constrain(pidResult, -350, 350);
    }

    return pidResult;
 }

/***************************************************************************************
 * Name: constrain
 * 
 * Descripttion: 
 * 
 * Arguments: 
 * 
 * Return: 
 * 
 * Author: Iakov Umrikhin
 * 
 * Date Created: 28.04.2022 (dd::mm::yyyy)
 * Date Modified: 28.04.2022 (dd::mm::yyyy)
 * 
 * ************************************************************************************/

signed int constrain(signed int modifiedVar, signed int lowBoundary, signed int highBoundary) {

    return (modifiedVar > highBoundary) ? highBoundary : (modifiedVar < lowBoundary) ? lowBoundary : modifiedVar;

}

/***************************************************************************************
 * Name: mapGyro()
 * 
 * Descripttion: 
 * 
 * Arguments: 
 * 
 * Return: 
 * 
 * Author: Iakov Umrikhin
 * 
 * Date Created: 29.04.2022 (dd::mm::yyyy)
 * Date Modified: 29.04.2022 (dd::mm::yyyy)
 * 
 * ************************************************************************************/
signed int mapGyro(signed int unmappedVal) {

    // gyro goes from +- 26624 => 250 / 32768 ~= 0.0094

    return (unmappedVal * 0.0076);
}


/********************************
 *      UART Interrupt          *
 *                              *
 * Used to trigger on ENTER key *
 ********************************/

#pragma vector = USCI_A1_VECTOR
 __interrupt void enterConsISR (void) {

     // disable interrupts
     UCA1IE &= ~UCRXIE;
     enterConsole = 0x0d;   // new line
     UCA1IFG &= ~UCRXIFG;

 }


/************************************************
 * Pseudo code for the motor's direction control:
 * 1. Set the desired angle; Desired angle: 0 deg ->
 *    error = newPhi - setAngle;s
 *    (SET_ANGLE)
 *
 * 2. Send the desired angle to the PID control
 *
 *    Ki, Kp, Kd;
 *    P = error * Kp;
 *    I += error * Ki;
 *    D = ???
 *
 * 3. Actuate motors
 *
 *
 *
 * 4. Record the actual angle
 * 5. Compute the error
 * 6. Repeat
 *
 *
 *
 ************************************************/
