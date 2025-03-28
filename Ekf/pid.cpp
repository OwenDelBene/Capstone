#include "pid.h"

void PIDInit(PIDController* pid, float kp, float ki, float kd, float tau, float dt, float maxOutput, float minOutput)
{
	pid->integral = 0.0f;
	pid->prevError = 0.0f;
	
	pid->derivative = 0.0f;
	pid->prevMeasurement = 0.0f;

	pid->output = 0.0f;

	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;


	pid->tau = tau;
	pid->T = dt;

	pid->upperLimit = maxOutput;
	pid->lowerLimit = minOutput;


}



float PIDUpdate(PIDController* pid, float setpoint, float measurement)
{
	//Error signal

	float error = setpoint - measurement;

	//Proportional term
	float proportional = pid->kp * error;

	//Integral term	
	pid->integral = pid->integral + 0.5f * pid->ki * pid->T * (error + pid->prevError);

	
	//integrator clamping 

	float limMinI, limMaxI;

	if (pid->upperLimit > proportional)
		limMaxI = pid->upperLimit - proportional;
	else limMaxI = 0.0f;

	if (pid->lowerLimit < proportional)
		limMinI = pid->lowerLimit - proportional;
	else limMinI = 0.0f;

	
	if (pid->integral > limMaxI) pid->integral = limMaxI;


	if (pid->integral < limMinI) pid->integral = limMinI;


	
	//Derivative term
	pid->derivative = (2.0f * pid->kd * (measurement - pid->prevMeasurement) 
			+ (2.0f * pid->tau - pid->T) * pid->derivative) 
			/ (2.0f * pid->tau * pid->T);

	pid->output = proportional + pid->integral + pid->derivative;

	if (pid->output > pid->upperLimit) pid->output = pid->upperLimit;

	if (pid->output < pid->lowerLimit) pid->output = pid->lowerLimit;


	pid->prevError = error;
	pid->prevMeasurement = measurement;
	return pid->output;
}
