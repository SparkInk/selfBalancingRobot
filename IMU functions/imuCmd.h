/*
 * imuCmd.h
 *
 *
 *
 *
 */

#include <msp430.h>


#ifndef IMUCMD_H_
#define IMUCMD_H_

    #define BUF_IMU_SIZE        100
    #define MAX_IMU_ARGS        2
    #define MAX_IMU_CMDS        8

    #define CMD0_IMU_NARG       0
    #define CMD1_IMU_NARG       0
    #define CMD2_IMU_NARG       0
    #define CMD3_IMU_NARG       0
    #define CMD4_IMU_NARG       0
    #define CMD5_IMU_NARG       0
    #define CMD6_IMU_NARG       1
    #define CMD7_IMU_NARG       1

    // Macros

    // Command names
    #define CMD0_IMU    "whoAmI"
    #define CMD1_IMU    "imuReadAcc"
    #define CMD2_IMU    "imuReadGyro"
    #define CMD3_IMU    "imuReadTemp"
    #define CMD4_IMU    "imuPowerOn"
    #define CMD5_IMU    "systemLaunch"
    #define CMD6_IMU    "imuAccelRange"
    #define CMD7_IMU    "imuGyroRange"
    
    // variables
    typedef struct IMU_CMD {
            const char* cmdName;
            int nArgs; // number of input arguments for a command
            signed long int args[MAX_IMU_ARGS]; // arguments
            char *encRegName;
        }IMU_CMD;

    // accelerometer structure
    typedef struct IMU_ACC {

        volatile signed int x_well_done;
        volatile signed int y_well_done;
        volatile signed int z_well_done;

    }IMU_ACC;

    // gyroscope structure
    typedef struct IMU_GYRO {

        volatile signed int x_well_done;
        volatile signed int y_well_done;
        volatile signed int z_well_done;


    }IMU_GYRO;

    // arrow keys structure
    typedef struct ARROWS {

        volatile unsigned char up, down, left, right, bitPID;

    } ARROWS;

    // PID gains 
    typedef struct PID {
        volatile signed int Kp, Ki;
        volatile float Kd;
    } PID;

    // functions definitions
    void initCmdList(IMU_CMD* cmdList);

    int parseCmd(IMU_CMD * cmdList , char * cmdLine);

    int validateCmd(IMU_CMD* cmdList, char* cmdName);

    int executeCmd(IMU_CMD * cmdList, int cmdIndex);

    void newLine(void);

#endif /* IMUCMD_H_ */
