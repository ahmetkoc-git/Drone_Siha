/*
 * control.h
 *
 *  Created on: Jan 24, 2026
 *      Author: Ahmet
 */

#ifndef INC_CONTROL_H_
#define INC_CONTROL_H_

#include "main.h"

#define MAX_ROLL_RATE_DPS   150.0f

#define TIMERHZ 500.0f//degisebilir suanlik 500hz
#define DEADBAND 2.0f


typedef struct{
	float motor1;
	float motor2;
	float motor3;
	float motor4;
}motors_t;

void Control_pid_Roll_Rate_Init(void);
float Control_pid_Roll_Rate_Update(float RollFromJoystick,float RollRate_FromGyro,float actualRollAngle);
void Control_pid_Roll_Angle_Init(void);
float Control_pid_Roll_Angle_Update(float actualRollAngle, float RollFromJoystick);

#endif /* INC_CONTROL_H_ */
