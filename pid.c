/*
 * pid.c
 *
 *  Created on: Jan 24, 2026
 *      Author: Ahmet
 */
#include "pid.h"


void PID_Init(PID_Controller_t *pid, float kp, float ki, float kd, float sampleTime, float minOutput, float maxOutput,float maxIntegral,float minIntegral){

	pid->kp =kp;
	pid->ki =ki;
	pid->kd =kd;
	pid->sampleTime = sampleTime;
	pid->minOutput= minOutput;
	pid->maxOutput=maxOutput;
	pid->integral=0.0f;
	pid->prevError=0.0f;
	pid->prevActual=0.0f;
	pid->output=0.0f;
	pid->maxIntegral = 0.0f;
	pid->minIntegral = 0.0f;
}

void PID_SetPoint(PID_Controller_t *pid, float setPoint){
	pid->setPoint = setPoint;
}

float PID_GetSetPoint(PID_Controller_t *pid){

	return pid-> setPoint;

}
float PID_Compute(PID_Controller_t *pid, float actualValue,float Deadband){
	float error= pid-> setPoint - actualValue;

	if (error > -Deadband && error<Deadband) {
		error=0.0f;
	}

	float proportional = pid->kp * error;

	pid->integral += error*pid->sampleTime;
	if (pid->integral > pid->maxIntegral) {
		pid->integral = pid->maxIntegral;
	}else if(pid->integral < pid->minIntegral){
		pid->integral = pid-> minIntegral;
	}
	float integral = pid-> ki * pid->integral;

	float derivative = -pid-> kd * (actualValue - pid->prevActual)/pid->sampleTime;
	pid->prevActual = actualValue;

	pid->output= proportional + integral + derivative;
	if (pid->output > pid->maxOutput) {
			pid->output = pid->maxOutput;
		}else if(pid->output < pid->minOutput){
			pid->output = pid-> minOutput;
		}
	return pid-> output;

}
void PID_Reset(PID_Controller_t *pid){

	pid->integral = 0.0f;
	pid->prevError = 0.0f;
}
float PID_GetKp(PID_Controller_t *pid){
	return pid->kp;
}
float PID_GetKi(PID_Controller_t *pid){
	return pid->ki;
}
float PID_GetKd(PID_Controller_t *pid){
	return pid->kd;
}

void PID_SetKp(PID_Controller_t *pid, float kp){
	pid->kp =kp;
}
void PID_SetKi(PID_Controller_t *pid, float ki){
	pid->ki =ki;
}
void PID_SetKd(PID_Controller_t *pid, float kd){
	pid->kd =kd;
}




