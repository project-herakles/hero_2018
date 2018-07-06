#include "controlTask.h"
#include "remoteTask.h"
#include "can.h"
#include "gpio.h"
#include "tim.h"
#include "pid.h"
#include "gun.h"
#include "ramp.h"
#include "math.h"


WorkState_e workState;
frictionState_e fricState = FRIC_STATIC;
uint32_t time_tick_ms = 0;

PID_Regulator_t GMPPositionPID = PITCH_POSITION_PID_DEFAULT;     
PID_Regulator_t GMPSpeedPID = PITCH_SPEED_PID_DEFAULT;
PID_Regulator_t GMYPositionPID = YAW_POSITION_PID_DEFAULT;			
PID_Regulator_t GMYSpeedPID = YAW_SPEED_PID_DEFAULT;

PID_Regulator_t CM1SpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;
PID_Regulator_t CM2SpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;
PID_Regulator_t CM3SpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;
PID_Regulator_t CM4SpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;
PID_Regulator_t ArmPositionPID;
PID_Regulator_t ArmSpeedPID;

PID_Regulator_t ShootPositionPID = SHOOT_POSITION_PID_DEFAULT;      //shoot motor
PID_Regulator_t ShootSpeedPID = SHOOT_SPEED_PID_DEFAULT;

extern volatile Encoder ArmEncoder;
extern volatile Encoder CM1Encoder;
extern volatile Encoder CM2Encoder;
extern volatile Encoder CM3Encoder;
extern volatile Encoder CM4Encoder;
extern volatile Encoder GMYawEncoder;
extern volatile Encoder GMPitchEncoder;
extern volatile Encoder ShootEncoder;
extern Chassis_speed_ref_t chassis_speed_ref;
extern RC_Ctrl_t RC_CtrlData;

uint32_t getCurrentTimeTick(void)
{
	return time_tick_ms;
}
void setWorkState(WorkState_e state)
{
	workState = state;
}

WorkState_e GetWorkState(void)
{
	return workState;
}

void workStateFSM(void)
{
	switch(workState)
	{
		case PREPARE_STATE:
		{
			if(getInputMode() == STOP_MODE)
				setWorkState(STOP_STATE);
			else if(time_tick_ms > PREPARE_TIME_TICK_MS)
				setWorkState(NORMAL_STATE);
		}break;
			
		case NORMAL_STATE:
		{
			if (getInputMode() == STOP_MODE)
				setWorkState(STOP_STATE);
		}break;
			
		case STOP_STATE:
		{
			if(getInputMode() != STOP_MODE)
				setWorkState(PREPARE_STATE);
		}break;
		default:;	
	}
}

float yaw_r,pitch_r=0;
const float YAW_INIT=0,PITCH_INIT=0;

void Gimbal_Control(void)
{
	//RED_LED_ON();
	switch(workState)
	{
		case PREPARE_STATE: // Init state (Yaw,Pitch) = (0,0)
		{
			yaw_r = YAW_INIT;
			pitch_r = PITCH_INIT; // Initial Yaw/Pitch value
			
			GMYPositionPID.ref = yaw_r;
			GMYPositionPID.fdb = GMYawEncoder.ecd_angle;
			//fuzzy test
			if(fabs(GMYPositionPID.ref-GMYPositionPID.fdb)<5.0f)
				PID_Calc_Debug(&GMYPositionPID,0.5,0.0006,0);
			else
				PID_Calc_Debug(&GMYPositionPID,0.3,0.00001,5); // a debug version of PID_Calc for testing parameters (P=0.6,I=0.0003,D=8)
			GMYSpeedPID.ref = GMYPositionPID.output;
			GMYSpeedPID.fdb = GMYawEncoder.filter_rate;
			PID_Calc_Debug(&GMYSpeedPID,100,0,0);
			
			GMPPositionPID.ref = pitch_r;
			GMPPositionPID.fdb = GMPitchEncoder.ecd_angle;			
			PID_Calc_Debug(&GMPPositionPID,0.5,0.004,1.5);
			GMPSpeedPID.ref = GMPPositionPID.output;
			GMPSpeedPID.fdb = GMPitchEncoder.filter_rate;
			PID_Calc_Debug(&GMPSpeedPID,20.0,0.0,0.0);
			set_GM_speed(-GMYSpeedPID.output,-GMPSpeedPID.output);
		}break;
		case NORMAL_STATE:
		{
			GMYPositionPID.ref = yaw_r;
			GMYPositionPID.fdb = GMYawEncoder.ecd_angle;
			if(fabs(GMYPositionPID.ref-GMYPositionPID.fdb)<5.0f)
				PID_Calc_Debug(&GMYPositionPID,0.5,0.0006,0);
			else
				PID_Calc_Debug(&GMYPositionPID,0.3,0.00001,5); // a debug version of PID_Calc for testing parameters (P=0.6,I=0.0003,D=8)

			GMYSpeedPID.ref = GMYPositionPID.output;
			GMYSpeedPID.fdb = GMYawEncoder.filter_rate;
			PID_Calc_Debug(&GMYSpeedPID,100,0.0,0);
			
			GMPPositionPID.ref = pitch_r;
			GMPPositionPID.fdb = GMPitchEncoder.ecd_angle;
			if(fabs(GMPPositionPID.ref-GMPPositionPID.fdb)<3.0f)
				PID_Calc_Debug(&GMPPositionPID,0.6,0.006,0);
			else
				PID_Calc_Debug(&GMPPositionPID,0.5,0.004,1.5);
			
			GMPSpeedPID.ref = GMPPositionPID.output;
			GMPSpeedPID.fdb = GMPitchEncoder.filter_rate;
			PID_Calc_Debug(&GMPSpeedPID,20.0,0.0,0.0);
			
			set_GM_speed(-GMYSpeedPID.output,-GMPSpeedPID.output);
		}break;
		default:
		{
			set_GM_speed(0,0);
		}
	}
}

void CM_Control(void)
{
	/*
	CM1SpeedPID.ref =  (-chassis_speed_ref.forward_back_ref*0.075 + chassis_speed_ref.left_right_ref*0.075 + chassis_speed_ref.rotate_ref)*18;
	CM2SpeedPID.ref = (chassis_speed_ref.forward_back_ref*0.075 + chassis_speed_ref.left_right_ref*0.075 + chassis_speed_ref.rotate_ref)*18;
	CM3SpeedPID.ref = (chassis_speed_ref.forward_back_ref*0.075 - chassis_speed_ref.left_right_ref*0.075 + chassis_speed_ref.rotate_ref)*18;
	CM4SpeedPID.ref = (-chassis_speed_ref.forward_back_ref*0.075 - chassis_speed_ref.left_right_ref*0.075 + chassis_speed_ref.rotate_ref)*18; 
	*/
	
	CM1SpeedPID.ref =  (-chassis_speed_ref.forward_back_ref*0.075 + chassis_speed_ref.left_right_ref*0.075)*18;
	CM2SpeedPID.ref = (chassis_speed_ref.forward_back_ref*0.075 + chassis_speed_ref.left_right_ref*0.075)*18;
	CM3SpeedPID.ref = (chassis_speed_ref.forward_back_ref*0.075 - chassis_speed_ref.left_right_ref*0.075)*18;
	CM4SpeedPID.ref = (-chassis_speed_ref.forward_back_ref*0.075 - chassis_speed_ref.left_right_ref*0.075)*18; 
	
	CM1SpeedPID.fdb = CM1Encoder.filter_rate;
	CM2SpeedPID.fdb = CM2Encoder.filter_rate;
	CM3SpeedPID.fdb = CM3Encoder.filter_rate;
	CM4SpeedPID.fdb = CM4Encoder.filter_rate;
	
	PID_Calc(&CM1SpeedPID);
	PID_Calc(&CM2SpeedPID);
	PID_Calc(&CM3SpeedPID);
	PID_Calc(&CM4SpeedPID);
	
	if(GetWorkState() != NORMAL_STATE)
	{
		set_CM_speed(0,0,0,0);
	}
	else
	{
		//set_CM_speed(CM1SpeedPID.ref*SPEED_OUTPUT_ATTENUATION,CM2SpeedPID.ref*SPEED_OUTPUT_ATTENUATION,CM3SpeedPID.ref*SPEED_OUTPUT_ATTENUATION,CM4SpeedPID.ref*SPEED_OUTPUT_ATTENUATION);
		set_CM_speed(-CM1SpeedPID.output,-CM2SpeedPID.output,-CM3SpeedPID.output,-CM4SpeedPID.output);
	}
}


void FrictionWheelControl(void)
{
  //if(PC_cmd.shoot.fire == 1) //Receive shooting command from odroid
	if(RC_CtrlData.rc.s2 == 1)
	{
		if(fricState==FRIC_STATIC) fricState = FRIC_ACCELERATING;
	}
	else
	{
		fricState = FRIC_STATIC;
	}
	switch(fricState)
	{
		case FRIC_STATIC:
		{
			SetFrictionWheelSpeed(1000);
		}break;
		case FRIC_ACCELERATING: 
		{
			uint16_t fric_output = FrictionRamp(1000,1600,500); // start=1000,end=1600,duration=500ms
			if(fric_output==1600)	
				fricState = FRIC_MAX_SPEED;
			else
				SetFrictionWheelSpeed(fric_output);
		}break;
		case FRIC_MAX_SPEED:
		{
			SetFrictionWheelSpeed(1600);
		}break;
	}
}

void shootMotorControl(void)
{
	//if(PC_cmd.shoot.fire == 1) //Receive shooting command from odroid
	if(RC_CtrlData.rc.s1 == 1)
	{
		ShootSpeedPID.ref = 120; // was 160
		ShootSpeedPID.fdb = ShootEncoder.filter_rate;
		PID_Calc_Debug(&ShootSpeedPID,10,0.002,0); 
		set_Shoot_speed(ShootSpeedPID.output); //Trigger Motor PID here
	}
	else
	{ 
		set_Shoot_speed(0);
		// Trigger Motor stop
	}
}

void Shoot_Control(void)
{
	switch(workState)
	{
		case PREPARE_STATE:
		{
			InitFrictionWheel();
		}break;
		case NORMAL_STATE:
		{
			FrictionWheelControl();
			shootMotorControl();
		}break;
		case STOP_STATE:
		{
			SetFrictionWheelSpeed(1000); // FrictionWheel stops	
			set_Shoot_speed(0);	// Trigger Motor stop
		}break;
		default:
		{
		}
	}
}


void Collect_Control()
{	
	ArmPositionPID.ref = 0;
	ArmPositionPID.fdb = ArmEncoder.ecd_angle;
	PID_Calc(&ArmPositionPID);
	//ArmSpeedPID.ref = ArmPositionPID.output;
	ArmSpeedPID.ref = 0;
	ArmSpeedPID.fdb = ArmEncoder.filter_rate;
	PID_Calc(&ArmSpeedPID);
	//setArmSpeed(-(int16_t)ArmSpeedPID.output);
	//setArmSpeed(-16000);
}
void contorlTaskInit(void)
{	return;
}




void Control_Task(void)
{

	if(time_tick_ms%1000==0)
	{
		RED_LED_TOGGLE();
	}
	time_tick_ms += 1;
	//set_CM_speed(2000,2000,2000,2000);
	workStateFSM();
	
	//step 1: ÔÆÌ¨¿ØÖÆ
	Gimbal_Control();
	Shoot_Control();
	GMShootControl();
	if(time_tick_ms%4 == 0)
	{
		CM_Control();
	}
	Collect_Control();
}
