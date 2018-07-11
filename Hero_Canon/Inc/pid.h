#ifndef _PID_H_
#define _PID_H_

typedef struct PID_Regulator_t
{
	float ref;
	float fdb;
	float err[2];
	float output;
	float last_output;
	float kp;
	float ki;
	float kd;
	float KpComponent;
	float KiComponent;
	float KdComponent;
}PID_Regulator_t;

#define ARM_POSITION_PID_DEFAULT \
{\
	0,\
	0,\
	{0,0},\
	0,\
	0,\
	0.0f,\
	0.0f,\
	0.0f,\
	0,\
	0,\
	0,\
}\

#define ARM_SPEED_PID_DEFAULT \
{\
	0,\
	0,\
	{0,0},\
	0,\
	0,\
	200.0f,\
	0.0f,\
	0.0f,\
	0,\
	0,\
	0,\
}\
//Encoder bias = 2500
#define ARM_ENCODER_DEFAULT \
{\
	0,\
	0,\
	0,\
	0,\
	0,\
	0,\
	0,\
	7486,\
	0,\
	0,\
	0,\
	0,\
	0,\
	0,\
	0,\
}\

#define CHASSIS_MOTOR_SPEED_PID_DEFAULT \
{\
	0,\
	0,\
	{0,0},\
	0,\
	0,\
	1.0f,\
	0.0f,\
	0.0f,\
	0,\
	0,\
	0,\
}\


void PID_Calc(PID_Regulator_t *pid);
void PID_Calc_Arm(PID_Regulator_t *PID_Regulator);
#endif
