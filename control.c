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
    pid_roll_rate.kp = 0.05f;//tunning yapilacak.
    pid_roll_rate.ki = 0.00f;
    pid_roll_rate.kd = 0.00f;
    pid_roll_rate.integral  = 0.0f;
    pid_roll_rate.prevError = 0.0f;
    pid_roll_rate.output    = 0.0f;
    pid_roll_rate.sampleTime = 1.0f/(float)TIMERHZ;
    pid_roll_rate.minOutput=-200.0f;
    pid_roll_rate.maxOutput =200.0f;
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

void Control_pid_Roll_Angle_Init(void){
	//ratenin pid tunningi yapılıp input olarak buradan vercegiz.
}

void Control_pid_pitch_Init(void){
	pid_pitch_rate.kp = 0.02f;//tunning yapilacak.
	pid_pitch_rate.ki = 0.00f;
	pid_pitch_rate.kd = 0.00f;
	pid_pitch_rate.integral = 0.00f;
	pid_pitch_rate.prevError = 0.00f;
	pid_pitch_rate.output = 0.00f;
	pid_pitch_rate.sampleTime = 0.00f;
	pid_pitch_rate.minOutput = 0.00f;
	pid_pitch_rate.maxOutput = 0.00f;
	pid_pitch_rate.maxIntegral = 0.00f;
	pid_pitch_rate.minIntegral = 0.00f;
	pid_pitch_rate.prevActual = 0.00f;
	pid_pitch_rate.setPoint = 0.00f;
}
