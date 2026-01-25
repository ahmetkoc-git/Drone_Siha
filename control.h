/*
 * control.h
 *
 *  Created on: Jan 24, 2026
 *      Author: Ahmet
 */

#ifndef INC_CONTROL_H_
#define INC_CONTROL_H_

#include "main.h"

#define MAX_ROLL_RATE_DPS   200.0f

#define TIMERHZ 500.0f//degisebilir suanlik 500hz
#define DEADBAND 2.0f

void Control_pid_Roll_Rate_Init(void);
float Control_pid_Roll_Rate_Update(float RollRate_FromJoystick,float RollRate_FromGyro);


#endif /* INC_CONTROL_H_ */
