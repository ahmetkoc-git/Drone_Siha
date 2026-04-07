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
static PID_Controller_t pid_roll_angle;

static PID_Controller_t pid_pitch_rate;

extern I2C_HandleTypeDef hi2c1;

void Control_pid_Roll_Rate_Init(void){
    pid_roll_rate.kp = 2.25f;//2.25 di kp
    pid_roll_rate.ki = 0.00f;
    pid_roll_rate.kd = 0.008f;//0.008 iydir.
    pid_roll_rate.integral  = 0.0f;
    pid_roll_rate.prevError = 0.0f;//reset için
    pid_roll_rate.output    = 0.0f;
    pid_roll_rate.sampleTime = 1.0f/(float)TIMERHZ;
    pid_roll_rate.minOutput=-150.0f;
    pid_roll_rate.maxOutput =150.0f;
    pid_roll_rate.maxIntegral=50.0f;
    pid_roll_rate.minIntegral=-50.0f;
    pid_roll_rate.prevActual=0.0f;
    pid_roll_rate.setPoint = 0.0f;
}
void Control_pid_Roll_Angle_Init(void){
	pid_roll_angle.kp = 0.0f;//5 denenebilir.
	pid_roll_angle.ki = 0.0f;
	pid_roll_angle.kd = 0.0f;
	pid_roll_angle.integral = 0.0f;
	pid_roll_rate.prevError = 0.0f;//reset için
	pid_roll_rate.output    = 0.0f;
	pid_roll_rate.sampleTime = 1.0f/(float)TIMERHZ;
	pid_roll_rate.minOutput=-150.0f;
	pid_roll_rate.maxOutput =150.0f;
	pid_roll_rate.maxIntegral=50.0f;
    pid_roll_rate.minIntegral=-50.0f;
	pid_roll_rate.prevActual=0.0f;
	pid_roll_rate.setPoint = 0.0f;
}
float Control_pid_Roll_Angle_Update(float actualRollAngle, float RollFromJoystick){
	pid_roll_angle.setPoint = RollFromJoystick;
	return PID_Compute(&pid_roll_angle, actualRollAngle, 1);
}
float Control_pid_Roll_Rate_Update(float RollFromJoystick,float RollRate_FromGyro,float actualRollAngle){
	/*float NormalRollFromJoystick = RollRate_FromJoystick/ 30.0f;
	if (NormalRollFromJoystick > 1.0f) NormalRollFromJoystick = 1.0f;
	if (NormalRollFromJoystick < -1.0f) NormalRollFromJoystick = -1.0f;

	float RollRateDps = NormalRollFromJoystick * MAX_ROLL_RATE_DPS;//setpoint degiscek angle pid yazinca setpoint oradan gelecek.

	pid_roll_rate.setPoint = RollRateDps;*/
	pid_roll_rate.setPoint = Control_pid_Roll_Angle_Update(actualRollAngle, RollFromJoystick);

	return PID_Compute(&pid_roll_rate, RollRate_FromGyro,DEADBAND);
}

