/*
 * motor_control.h
 *
 *  Created on: Jan. 29, 2022
	Updated: 22.04.2022 (dd::mm:yyyy);
 *      Author: Administrator
 */

#ifndef MOTORCONTROL_H_
#define MOTORCONTROL_H_

//(CPU_SPEED)/15000hZ) = 1667   using clock divisor 1 in order to get more precision
#define PWM_SPEED 1667

#define MOTOR_DIR P3OUT
#define     M1DIR BIT0
#define     M2DIR BIT1
#define MOTOR_DIR2 P2OUT
#define     M1DIR2 BIT6
#define     M2DIR2 BIT3

#define MOTOR_LOW_RANGE 995
#define MOTOR_HIGH_RANGE 1005
// Configures Hardware PWM Signals at the speed defined by PWM_SPEED
// Also configures the direction Pins as required by the existing motor controllers
void pwmDriveInit(void);

// Set Hardware PWM duty Cycle to a current variable
// Call this function every n seconds as required by the PID control loop
void updateMotorSpeed(int speed);


#endif /* MOTORCONTROL_H_ */