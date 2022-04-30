

#include <msp430.h>

#ifndef IMUHEADER_H
#define IMUHEADER_H
                        /****************************************************
                         *                                                  *
                         *                MPU6050 Registers                 *
                         *                                                  *
                         ****************************************************/

    //**************************** MPU6050's addresses and control bits ***************************
    #define MPU6050_ADDR            0x68

    /*************************  CONFIGURATION ************************/
    #define CONFIG                  0x1A

    /******************** BANDWIDTH(Hz) ACC and GYRO *****************/
    #define BW_260_ACC              0x0
    #define BW_184_ACC              0x1
    #define BW_94_ACC               0x2
    #define BW_44_ACC               0x3
    #define BW_21_ACC               0x4
    #define BW_10_ACC               0x5
    #define BW_5_ACC                0x6
    
    /***************************   ACC_REG   ************************/
    #define ACC_XOUT_H              0x3B
    #define ACC_XOUT_L              0x3C
    #define ACC_YOUT_H              0x3D
    #define ACC_YOUT_L              0x3E
    #define ACC_ZOUT_H              0x3F
    #define ACC_ZOUT_L              0x40

                      /******    ACC_CONFIG  ******/
    #define ACCEL_CONFIG            0x1C
    #define ACCEL_2G                0x00
    #define ACCEL_4G                0x08
    #define ACCEL_8G                0x10
    #define ACCEL_16G               0x18    // full scale: +- 16g

    /***************************   TEMPERATURE   ************************/
    #define TEMP_OUT_H              0x41
    #define TEMP_OUT_L              0x42

    /***************************   GYRO_REG   ************************/
    #define GYRO_XOUT_H             0x43
    #define GYRO_XOUT_L             0x44
    #define GYR_YOUT_H              0x45
    #define GYR_YOUT_L              0x46
    #define GYR_ZOUT_H              0x47
    #define GYR_ZOUT_L              0x48
                      /******    GYRO_CONFIG  ******/
    #define GYRO_CONFIG             0x1B
    #define GYRO_250                0x00
    #define GYRO_500                0x08
    #define GYRO_1000               0x10
    #define GYRO_2000               0x18
    // Registers to write to or read from
    #define SAMPLE_RATE_DIVIDER     0x19
    #define MPU_PWR_MGMT_1          0x6B
    #define WHO_AM_I                0x75

    // Power management
    #define NORMAL_MODE             0
    //**************************************************************************

    // constants
    #define MAX_BYTES               2
    #define MPU_BUF_SIZE            10
    #define DUMMY_REG               0x00
    #define SET_ANGLE               10

    // MACROS           
    #define IMU_PWR_TRIG            P8OUT &= ~BIT2; P8OUT |= BIT2;
    #define SEND_SA(slaveAddr)      UCB1I2CSA = slaveAddr
    #define WAIT_1ms                __delay_cycles(25000)
    #define WAIT_100ms                __delay_cycles(2500000)
    #define I2C_RST_ON              UCB1CTL1 |= UCSWRST;
    #define I2C_RST_OFF             UCB1CTL1 &= ~UCSWRST;

    #define MST_TX_MODE             UCB1CTL1 |= UCTR     // Reset is ON; Transmitter Mode; Reset is OFF
    #define MST_RX_MODE             UCB1CTL1 &= ~UCTR   // Reset is ON; Receiver Mode; Reset is OFF
    #define START_BIT               UCB1CTL1 |= UCTXSTT    // Transmit START bit
    #define STOP_BIT                UCB1CTL1 |= UCTXSTP    // Transmit STOP bit
    #define WAIT_STT                while (UCB1CTL1 & UCTXSTT)  // Wait for START bit to go low -> ACK is received
    #define WAIT_TX_IFG             while (!(UCB1IFG & UCTXIFG))  // Wait for TXBUFF to be empty; when it is empty TXIFG == 1;
    #define WAIT_RX_IFG             while (!(UCB1IFG & UCRXIFG))   // Wait RXBUFF to receive a character; when receive a character RXIFG == 1;

    #define IMU_PWR_OFF             P8OUT &= ~BIT2;
    #define IMU_PWR_ON              P8OUT |= BIT2;
    // global variables

    unsigned char stopBit;
    // function prototypes
    signed char IMUSendByte(unsigned char slaveAddr, unsigned char regAddr, unsigned char txBuffer, unsigned char restart);

    signed char IMUSendBurst(signed char nBytes, char slaveAddr, unsigned char regAddr, unsigned char *txBuffer, unsigned char restart);
    
    signed char IMURead(unsigned char nBytes, unsigned char slaveAddr, signed char *rxBuffer);

#endif
