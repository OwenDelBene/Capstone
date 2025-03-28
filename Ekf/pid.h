#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

typedef struct{
	float kp;
	float ki;
	float kd;

	//Time constant of low pass filter of derivative term
	float tau;

	//limits on the output
	float lowerLimit;
	float upperLimit;

	//sample time (s)
	float T;


	float integral;
	float prevError;
	float derivative;
	float prevMeasurement;


	float output;


} PIDController;


void PIDInit(PIDController* pid, float kp, float ki, float kd, float tau, float dt, float maxOutput, float minOutput);
float PIDUpdate(PIDController* pid, float setpoint, float measurement);


#endif



