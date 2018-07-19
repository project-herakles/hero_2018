#include "controlTask.h"
#include "remoteTask.h"
#include "can.h"
#include "gpio.h"
#include "tim.h"
#include "pid.h"
#include "math.h"
#include "imu_task.h"
#include "math.h"
#include "collect.h"
#include "gun.h"


#define RESCUE_CLAW_ENABLE() HAL_GPIO_WritePin(GPIOF,GPIO_PIN_10,GPIO_PIN_SET)
#define RESCUE_CLAW_DISABLE() HAL_GPIO_WritePin(GPIOF,GPIO_PIN_10,GPIO_PIN_RESET)

int32_t Arm_Encoder_Init;

workState_e workState;
CollectMode collectMode;
uint32_t time_tick_ms = 0;
uint32_t start_time_tick_ms = 0; //to record the current time for timeout in collectModeSwitch()
uint8_t countdown = 0; //used for timeout in collectModeSwitch()
uint8_t collect_flag = 0; //set it to 1 to go into collect Mode, only return to 0 when collect is finished
uint8_t rescue_flag = 0;
uint8_t collect_mode_code = 0;
uint8_t claw_in_place = 0;//Variable for human supervision, change by clicking
uint8_t waveLength = 50; 

PID_Regulator_t CM1SpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;
PID_Regulator_t CM2SpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;
PID_Regulator_t CM3SpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;
PID_Regulator_t CM4SpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;
PID_Regulator_t RotatePID = ROTATE_PID_DEFAULT;
PID_Regulator_t GMYPositionPID = YAW_POSITION_PID_DEFAULT;
PID_Regulator_t GMPPositionPID = PITCH_POSITION_PID_DEFAULT;
PID_Regulator_t GMYSpeedPID = YAW_SPEED_PID_DEFAULT;
PID_Regulator_t GMPSpeedPID = PITCH_SPEED_PID_DEFAULT;
PID_Regulator_t ShootPositionPID = SHOOT_POSITION_PID_DEFAULT;
PID_Regulator_t ShootSpeedPID = SHOOT_SPEED_PID_DEFAULT;

extern volatile Encoder ArmEncoder;
extern volatile Encoder GMYawEncoder;
extern volatile Encoder GMPitchEncoder;
extern volatile Encoder CM1Encoder;
extern volatile Encoder CM2Encoder;
extern volatile Encoder CM3Encoder;
extern volatile Encoder CM4Encoder;
extern volatile Encoder ShootEncoder;
extern Chassis_speed_ref_t chassis_speed_ref;
extern RC_Ctrl_t RC_CtrlData;
extern Gimbal_Ref_t Gimbal_Ref;
extern uint8_t imu_cali_flag;
extern imu_attitude_t atti;
extern imu_data_t imu;
extern mpu_data_t mpu_data;
float yaw_offset = 0;
float yaw_speed = 0;
float yaw_speed_offset = 0;

extern uint32_t can_count;

uint32_t getCurrentTimeTick(void)
{
	return time_tick_ms;
}
void setWorkState(workState_e state)
{
	workState = state;
}

workState_e getWorkState(void)
{
	return workState;
}

void workStateFSM(void)
{
	switch(workState)
	{
		case WAKE_STATE:
			if(getInputMode() == STOP_MODE)
				setWorkState(STOP_STATE);
			else if(imu_cali_flag==1)
				setWorkState(PREPARE_STATE);
		case PREPARE_STATE:
			if(getInputMode() == STOP_MODE)
				setWorkState(STOP_STATE);
			else if(time_tick_ms > PREPARE_TIME_TICK_MS)
				setWorkState(NORMAL_STATE);
			break;
			
		case NORMAL_STATE:
			if (getInputMode() == STOP_MODE)
				setWorkState(STOP_STATE);
			else if(collect_flag == 1)
				setWorkState(COLLECT_STATE);
			break;
			
		case COLLECT_STATE:
			if(getInputMode() == STOP_MODE)
				setWorkState(STOP_STATE);
			else if(collect_flag == 0)
				setWorkState(NORMAL_STATE);
			break;
			
		case STOP_STATE:
			if(getInputMode() != STOP_MODE)
			{
				setWorkState(PREPARE_STATE);
				time_tick_ms = 0;
				can_count = 0;
				Gimbal_Ref.yaw_angle_dynamic_ref = 0;
				Gimbal_Ref.pitch_angle_dynamic_ref = 0;
			}
			break;
		default:;	
	}
}

void CM_Control(void)
{
	
	// rotate with stick
	CM1SpeedPID.ref =  (-chassis_speed_ref.forward_back_ref*0.075 + chassis_speed_ref.left_right_ref*0.075 + chassis_speed_ref.rotate_ref*0.075)*18;
	CM2SpeedPID.ref = (chassis_speed_ref.forward_back_ref*0.075 + chassis_speed_ref.left_right_ref*0.075 + chassis_speed_ref.rotate_ref*0.075)*18;
	CM3SpeedPID.ref = (chassis_speed_ref.forward_back_ref*0.075 - chassis_speed_ref.left_right_ref*0.075 + chassis_speed_ref.rotate_ref*0.075)*18;
	CM4SpeedPID.ref = (-chassis_speed_ref.forward_back_ref*0.075 - chassis_speed_ref.left_right_ref*0.075 + chassis_speed_ref.rotate_ref*0.075)*18; 
	
	// rotate with yaw 
	/*
	if(getWorkState()== NORMAL_STATE)
	{
		RotatePID.ref = 0;
		RotatePID.fdb = GMYawEncoder.ecd_angle;
		PID_Calc_Debug(&RotatePID,1,0,0);
		chassis_speed_ref.rotate_ref = RotatePID.output;
	}
	else
	{
		chassis_speed_ref.rotate_ref = 0;
	}
	*/
	/*
	CM1SpeedPID.ref =  (-chassis_speed_ref.forward_back_ref*0.075 + chassis_speed_ref.left_right_ref*0.075 + chassis_speed_ref.rotate_ref)*18;
	CM2SpeedPID.ref = (chassis_speed_ref.forward_back_ref*0.075 + chassis_speed_ref.left_right_ref*0.075 + chassis_speed_ref.rotate_ref)*18;
	CM3SpeedPID.ref = (chassis_speed_ref.forward_back_ref*0.075 - chassis_speed_ref.left_right_ref*0.075 + chassis_speed_ref.rotate_ref)*18;
	CM4SpeedPID.ref = (-chassis_speed_ref.forward_back_ref*0.075 - chassis_speed_ref.left_right_ref*0.075 + chassis_speed_ref.rotate_ref)*18; 
	*/
	
	// no rotate
	/*
	CM1SpeedPID.ref =  (-chassis_speed_ref.forward_back_ref*0.075 + chassis_speed_ref.left_right_ref*0.075)*18;
	CM2SpeedPID.ref = (chassis_speed_ref.forward_back_ref*0.075 + chassis_speed_ref.left_right_ref*0.075)*18;
	CM3SpeedPID.ref = (chassis_speed_ref.forward_back_ref*0.075 - chassis_speed_ref.left_right_ref*0.075)*18;
	CM4SpeedPID.ref = (-chassis_speed_ref.forward_back_ref*0.075 - chassis_speed_ref.left_right_ref*0.075)*18; 
	*/
	
	CM1SpeedPID.fdb = CM1Encoder.filter_rate;
	CM2SpeedPID.fdb = CM2Encoder.filter_rate;
	CM3SpeedPID.fdb = CM3Encoder.filter_rate;
	CM4SpeedPID.fdb = CM4Encoder.filter_rate;
	
	PID_Calc(&CM1SpeedPID);
	PID_Calc(&CM2SpeedPID);
	PID_Calc(&CM3SpeedPID);
	PID_Calc(&CM4SpeedPID);
	
	if(getWorkState() != NORMAL_STATE)
	{
		set_CM_speed(0,0,0,0);
	}
	else
	{
		//set_CM_speed(CM1SpeedPID.ref*SPEED_OUTPUT_ATTENUATION,CM2SpeedPID.ref*SPEED_OUTPUT_ATTENUATION,CM3SpeedPID.ref*SPEED_OUTPUT_ATTENUATION,CM4SpeedPID.ref*SPEED_OUTPUT_ATTENUATION);
		set_CM_speed(CM1SpeedPID.output*SPEED_OUTPUT_ATTENUATION,CM2SpeedPID.output*SPEED_OUTPUT_ATTENUATION,CM3SpeedPID.output*SPEED_OUTPUT_ATTENUATION,CM4SpeedPID.output*SPEED_OUTPUT_ATTENUATION);
	}
}

//ARM_IN:arm is moving in or has been already inside
//ARM_OUT:arm is moving out ro has been already outside
//ARM_REST: output = 0;
//need input(instructions):

void contorlTaskInit(void)
{
    return;
}

void Gimbal_Control(void)
{
	//RED_LED_ON();
	switch(workState)
	{
		case PREPARE_STATE: // Init state (Yaw,Pitch) = (0,0)
		{
			
			GMYPositionPID.ref = 0;
			GMYPositionPID.fdb = GMYawEncoder.ecd_angle;
			//fuzzy test
			if(fabs(GMYPositionPID.ref-GMYPositionPID.fdb)<5.0f)
				PID_Calc_Debug(&GMYPositionPID,1,0.000,0);
			else
				PID_Calc_Debug(&GMYPositionPID,0.5,0.00,5); // a debug version of PID_Calc for testing parameters (P=0.6,I=0.0003,D=8)
			GMYSpeedPID.ref = GMYPositionPID.output;
			GMYSpeedPID.fdb = GMYawEncoder.filter_rate;
			PID_Calc_Debug(&GMYSpeedPID,50,0,0);
			
			GMPPositionPID.ref = 0;
			GMPPositionPID.fdb = GMPitchEncoder.ecd_angle;			
			PID_Calc_Debug(&GMPPositionPID,0.8,0.00001,0);
			GMPSpeedPID.ref = GMPPositionPID.output;
			GMPSpeedPID.fdb = GMPitchEncoder.filter_rate;
			PID_Calc_Debug(&GMPSpeedPID,100.0,0.0,0.0);
			//set_GM_speed(-GMYSpeedPID.output,-GMPSpeedPID.output);
			set_GM_speed(0,-GMPSpeedPID.output);
			
			yaw_offset = atti.yaw;
		}break;
		case NORMAL_STATE:
		{
			// YAW WITH ENCODER
			/*
			GMYPositionPID.ref = 0;
			GMYPositionPID.fdb = GMYawEncoder.ecd_angle;
			if(fabs(GMYPositionPID.ref-GMYPositionPID.fdb)<5.0f)
				PID_Calc_Debug(&GMYPositionPID,1,0.000,0);
			else
				PID_Calc_Debug(&GMYPositionPID,0.5,0.00,5); // a debug version of PID_Calc for testing parameters (P=0.6,I=0.0003,D=8)
			GMYSpeedPID.ref = GMYPositionPID.output;
			GMYSpeedPID.fdb = GMYawEncoder.filter_rate;
			PID_Calc_Debug(&GMYSpeedPID,50,0,0);
			*/
			
			// YAW WITH IMU
			yaw_speed = mpu_data.gz / 16.384f;
			
			GMYPositionPID.ref = Gimbal_Ref.yaw_angle_dynamic_ref;
			/*
			if(time_tick_ms%4000==0)
			{
				if(GMYPositionPID.ref==0)
				{
					GMYPositionPID.ref = 30;
				}
				else
				{
					GMYPositionPID.ref = 0;
				}
			}
			*/
			GMYPositionPID.fdb = atti.yaw - yaw_offset;
			//PID_Calc_Debug(&GMYPositionPID,8,0.00,300);
			
			if(fabs(GMYPositionPID.ref-GMYPositionPID.fdb)<5.0f)
				PID_Calc_Debug(&GMYPositionPID,15,0.004,100);
			else
			{
				//GMYPositionPID.KiComponent = 0; // clear ki if step occurs
				PID_Calc_Debug(&GMYPositionPID,10,0.00,300); // a debug version of PID_Calc for testing parameters (P=0.6,I=0.0003,D=8)
			}
			//GMYSpeedPID.ref = 0;
			GMYSpeedPID.ref = GMYPositionPID.output;
			GMYSpeedPID.fdb = yaw_speed;
			PID_Calc_Debug(&GMYSpeedPID,1,0.0,0);
			
			GMPPositionPID.ref = Gimbal_Ref.pitch_angle_dynamic_ref;
			GMPPositionPID.fdb = GMPitchEncoder.ecd_angle;
			PID_Calc_Debug(&GMPPositionPID,0.4,0.001,0);
			
			if(fabs(GMPPositionPID.ref-GMPPositionPID.fdb)<5.0f)
				PID_Calc_Debug(&GMPPositionPID,1,0.0001,0);
			else
				PID_Calc_Debug(&GMPPositionPID,0.8,0.00001,5);
			
			GMPSpeedPID.ref = GMPPositionPID.output;
			GMPSpeedPID.fdb = GMPitchEncoder.filter_rate;
			PID_Calc_Debug(&GMPSpeedPID,100,0.0,0.0);
			
			//set_GM_speed(-GMYSpeedPID.output,-GMPSpeedPID.output);
			set_GM_speed(0,-GMPSpeedPID.output); //disable yaw
		}break;
		default:
		{
			set_GM_speed(0,0);
		}
	}
}
void FrictionWheelControl(void)
{
  //if(PC_cmd.shoot.fire == 1) //Receive shooting command from odroid
	//if(RC_CtrlData.rc.s2 == 1)
	if(getWorkState() == NORMAL_STATE && RC_CtrlData.rc.s1==1)
	{
		SetFrictionWheelSpeed(1800);
	}
	else
	{
		SetFrictionWheelSpeed(1000);
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



void Control_Loop(void)
{
	if(time_tick_ms%1000==0)
	{
		RED_LED_TOGGLE();
	}
	time_tick_ms += 1;

	workStateFSM();
	Gimbal_Control();
	Shoot_Control();
	GMArmShootControl();
	
	if(time_tick_ms%4==0)
	{
		CM_Control();
	}
	
	//Collect_Mode_Switch();
	if(workState == NORMAL_STATE)
		Collect_Control();
}
