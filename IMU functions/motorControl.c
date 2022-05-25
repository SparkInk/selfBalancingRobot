/*
 * motor_control.c
 *
 *  Created on: Jan. 29, 2022
 *	Updated: 22.04.2022 (dd::mm:yyyy)
 *      Author: Administrator
 */
#include <motorControl.h>
#include <msp430.h>
#include <math.h>

void pwmDriveInit(void){
    P2SEL |= BIT4|BIT5;   //Enable timer hardware pins
    P2DIR |= BIT4|BIT5;   // Set them to output

    TA2CTL = TASSEL_2 | MC_1 | ID_0;                    // SMCLK/1(ID_0), SMCLK/8(ID_3), up-mode(MC_1)

    P3DIR |= M1DIR|M2DIR;
    P2DIR |= M1DIR2|M2DIR2;

    TA2CCR0 = PWM_SPEED;
    TA2CCR1 = 0;     // Initialize motor off
    TA2CCR2 = 0;

    TA2CCTL1 |= OUTMOD_7;       // RESET/SET mode
    TA2CCTL2 |= OUTMOD_7;
}

//since the RC remote already works with a range of 200-1800
//we can do 1000 - 1800 800 positive
// and      200 - 1000 -800 negative

// subtract or add 100 as need and multiply by 2 for the value that shall be stored in the PWM timer
/********************************************************************
 * Name: updateMotorSpeed
 * 
 * Description: 
 * 
 * Date created: 29.01.2022 (dd:mm:yyyy)
 * Date modified: 22.04.2022 (dd:mm:yyyy)
 * *****************************************************************/

// void updateMotorSpeed(int m1, int m2){
void updateMotorSpeed(int speed){
    volatile int m1Speed, m2Speed;
    // Give better names to motors
    // instead of m1 and m2

    speed += 1000;


//  motor one 
    if(speed > MOTOR_HIGH_RANGE){
//      initial speed
        m1Speed = (speed - 1000) * 2;
        m2Speed = (speed - 1000) * 2;

        ccwMotor();
    } 
    else if (speed < MOTOR_LOW_RANGE) {
        
        m1Speed = (1000 - speed) * 2;
        m2Speed = (1000 - speed) * 2;

        cwMotor();
    }
    else {
        m1Speed = 0;
        m2Speed = 0;
    }

//  motor two
    // if(m2 > MOTOR_HIGH_RANGE){
    //     m2Speed = (m2 - 1000) * 2;
    //     MOTOR_DIR &= ~M2DIR;     // Set Direction Reverse
    //     MOTOR_DIR2 |= M2DIR2;     // Set direction Forward
    // } 
    // else if (m2 < MOTOR_LOW_RANGE){
    //     m2Speed = (1000 - m2) * 2;
    //     MOTOR_DIR |=  M2DIR;     // Set direction Forward
    //     MOTOR_DIR2 &= ~M2DIR2;     // Set Direction Reverse
    // }
    // else {
    //     m2Speed = 0;
    // }

//  Initialize motor off
    TA2CCR1 = m1Speed;     
    TA2CCR2 = m2Speed;
}


void cwMotor(void) {
        MOTOR_DIR |=  M1DIR;     // Set direction Forward
        MOTOR_DIR &= ~M2DIR;     // Set Direction Reverse

        MOTOR_DIR2 &= ~M1DIR2;     // Set Direction Reverse
        MOTOR_DIR2 |= M2DIR2;     // Set direction Forward
}

void ccwMotor(void) {

        MOTOR_DIR &= ~M1DIR;     // Set Direction Reverse
        MOTOR_DIR |=  M2DIR;     // Set direction Forward

        MOTOR_DIR2 |=  M1DIR2;     // Set direction Forward
        MOTOR_DIR2 &= ~M2DIR2;     // Set Direction Reverse
}
