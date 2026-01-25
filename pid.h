/*
 * pid.h
 *
 *  Created on: Jan 24, 2026
 *      Author: Ahmet
 */

#ifndef INC_PID_H_
#define INC_PID_H_



typedef struct{
	float kp;
	float ki;
	float kd;
	float setPoint;
	float integral;
	float prevError;
	float prevActual;
	float output;
	float minOutput;
	float maxOutput;
	float sampleTime;
	float maxIntegral;
	float minIntegral;
}PID_Controller_t;


void PID_Init(PID_Controller_t *pid, float kp, float ki, float kd, float sampleTime, float minOutput, float maxOutput,float maxIntegral,float minIntegral);
void PID_SetPoint(PID_Controller_t *pid, float setPoint);
float PID_GetSetPoint(PID_Controller_t *pid);
float PID_Compute(PID_Controller_t *pid, float actualValue,float Deadband);
void PID_Reset(PID_Controller_t *pid);
float PID_GetKp(PID_Controller_t *pid);
float PID_GetKi(PID_Controller_t *pid);
float PID_GetKd(PID_Controller_t *pid);
void PID_SetKp(PID_Controller_t *pid, float kp);
void PID_SetKi(PID_Controller_t *pid, float ki);
void PID_SetKd(PID_Controller_t *pid, float kd);
#endif /* INC_PID_H_ */
