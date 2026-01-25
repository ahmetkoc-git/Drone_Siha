/*
 * control.c
 *
 *  Created on: Jan 24, 2026
 *      Author: Ahmet
 */

#include "control.h"
#include "main.h"
#include "mpu6050.h"
#include "pid.h"


static PID_Controller_t pid_roll_rate;

extern I2C_HandleTypeDef hi2c1;

void Control_pid_Roll_Rate_Init(void){
    pid_roll_rate.kp = 0.02f;
    pid_roll_rate.ki = 0.00f;
    pid_roll_rate.kd = 0.00f;
    pid_roll_rate.integral  = 0.0f;
    pid_roll_rate.prevError = 0.0f;
    pid_roll_rate.output    = 0.0f;
    pid_roll_rate.sampleTime = 1.0f/(float)TIMERHZ;
    pid_roll_rate.minOutput=-1.0f;
    pid_roll_rate.maxOutput = 1.0f;
    pid_roll_rate.maxIntegral=50.0f;
    pid_roll_rate.minIntegral=-50.0f;
    pid_roll_rate.prevActual=0.0f;
    pid_roll_rate.setPoint = 0.0f;
}
float Control_pid_Roll_Rate_Update(float RollRate_FromJoystick,float RollRate_FromGyro){
	float NormalRollFromJoystick = RollRate_FromJoystick/ 30.0f;
	if (NormalRollFromJoystick > 1.0f) NormalRollFromJoystick = 1.0f;
	if (NormalRollFromJoystick < -1.0f) NormalRollFromJoystick = -1.0f;

	float RollRateDps = NormalRollFromJoystick * MAX_ROLL_RATE_DPS;//setpoint degiscek angle pid yazinca setpoint oradan gelecek.

	pid_roll_rate.setPoint = RollRateDps;


	return PID_Compute(&pid_roll_rate, RollRate_FromGyro,DEADBAND);
}
