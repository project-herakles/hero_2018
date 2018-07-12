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
	float Ki_Limit;
	float output_limit;
	void (*Calc)(struct PID_Regulator_t *);
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
	0,\
	0,\
	&PID_Calc_Arm,\
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
	0,\
	0,\
	&PID_Calc_Arm, \
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

#define YAW_ENCODER_DEFAULT \
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

#define PITCH_ENCODER_DEFAULT \
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

#define SHOOT_ENCODER_DEFAULT \
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
	220.0f,\
	0.0f,\
	0.0f,\
	0,\
	0,\
	0,\
	0,\
	5000,\
	&PID_Calc_Step,\
}\


void PID_Calc(PID_Regulator_t *pid);
void PID_Calc_Step(PID_Regulator_t *pid);
void PID_Calc_Arm(PID_Regulator_t *PID_Regulator);
#endif
