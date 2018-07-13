#include "controlTask.h"
#include "remoteTask.h"
#include "can.h"
#include "gpio.h"
#include "tim.h"
#include "pid.h"
#include "math.h"

#define ARM_Angle_to_Encoder 77116.0f/180.0f
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
PID_Regulator_t GMYPositionPID = YAW_POSITION_PID_DEFAULT;
PID_Regulator_t GMPPositionPID = PITCH_POSITION_PID_DEFAULT;
PID_Regulator_t GMYSpeedPID = YAW_SPEED_PID_DEFAULT;
PID_Regulator_t GMPSpeedPID = PITCH_SPEED_PID_DEFAULT;
PID_Regulator_t ArmPositionPID = ARM_POSITION_PID_DEFAULT;
PID_Regulator_t ArmSpeedPID = ARM_SPEED_PID_DEFAULT;

extern volatile Encoder ArmEncoder;
extern volatile Encoder GMYawEncoder;
extern volatile Encoder GMPitchEncoder;
extern volatile Encoder CM1Encoder;
extern volatile Encoder CM2Encoder;
extern volatile Encoder CM3Encoder;
extern volatile Encoder CM4Encoder;
extern Chassis_speed_ref_t chassis_speed_ref;
extern RC_Ctrl_t RC_CtrlData;
extern Gimbal_Ref_t Gimbal_Ref;

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
				setWorkState(PREPARE_STATE);
			break;
		default:;	
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

void code2mode(uint8_t code,CollectMode *collectMode){
	switch(code)
	{
		case 0:
			collectMode->armMode = ACUTE;
			collectMode->clawMode = CLAW_DISABLE;
			collectMode->dumperMode = REST;
			break;
		case 1:
			collectMode->armMode = OBTUSE;
			collectMode->clawMode = CLAW_DISABLE;
			collectMode->dumperMode = REST;
			break;
		case 2:
			collectMode->armMode = OBTUSE;
			collectMode->clawMode = CLAW_ENABLE;
			collectMode->dumperMode = REST;
			break;
		case 3:
			collectMode->armMode = RIGHT;
			collectMode->clawMode = CLAW_ENABLE;
			collectMode->dumperMode = HOLD;
			break;
		case 4:
			collectMode->armMode = RIGHT;
			collectMode->clawMode = CLAW_DISABLE;
			collectMode->dumperMode = HOLD;
			break;
		case 5:
			collectMode->armMode = ACUTE;
			collectMode->clawMode = CLAW_DISABLE;
			collectMode->dumperMode = HOLD;
			break;
		case 6:
			collectMode->armMode = ACUTE;
			collectMode->clawMode = CLAW_DISABLE;
			collectMode->dumperMode = DUMP;
			break;
	}
	return;
}



//need input(instructions):
void Collect_Control(void)
{	
	if(ArmSpeedPID.ki == 0)
		ArmSpeedPID.KiComponent = 0;
	ArmSpeedPID.fdb = ArmEncoder.filter_rate;
	if(RC_CtrlData.rc.s2 != 1)
	{
		if(RC_CtrlData.rc.s1 == 2)
		{
			if(ArmEncoder.ecd_value < Arm_Encoder_Init - 150 * ARM_Angle_to_Encoder)
			{
				 ArmSpeedPID.ref = 200;
				ArmSpeedPID.kp = 100.0f;
				ArmSpeedPID.ki = 0;
			 PID_Calc_Arm(&ArmSpeedPID);
			}

			else if (ArmEncoder.ecd_value <Arm_Encoder_Init - 3 * ARM_Angle_to_Encoder)
	   {       
				ArmSpeedPID.ref = 200;
				ArmSpeedPID.kp = 100.0f;
	    	ArmSpeedPID.ki = 0;
				PID_Calc_Arm(&ArmSpeedPID);
			}
			else
			{
				ArmSpeedPID.ref = 0;
				ArmSpeedPID.kp = 60.0f;
				ArmSpeedPID.ki = 0;
				//ArmSpeedPID.ki = 0.003;
				PID_Calc_Arm(&ArmSpeedPID);
			}
		}
		else if(RC_CtrlData.rc.s1 == 1)//without box
		{
			
			if(ArmEncoder.ecd_value > Arm_Encoder_Init - 90 * ARM_Angle_to_Encoder)
			{
        ArmSpeedPID.ref = -200;
				ArmSpeedPID.kp = 100.0f;
				ArmSpeedPID.ki = 0;
			 PID_Calc_Arm(&ArmSpeedPID);
			}
			else if (ArmEncoder.ecd_value >Arm_Encoder_Init - 170 * ARM_Angle_to_Encoder)
			{
				ArmSpeedPID.ref = -200;
				ArmSpeedPID.kp = 100.0f;
				ArmSpeedPID.ki = 0;
				PID_Calc_Arm(&ArmSpeedPID);
			}
			else 
			{
				ArmSpeedPID.ref = 0;
				ArmSpeedPID.kp = 80.0f;
				ArmSpeedPID.ki = 0.2;
				PID_Calc_Arm(&ArmSpeedPID);
			}
		}
		else if(RC_CtrlData.rc.s1 == 3)
		{
			ArmSpeedPID.ref = 0;
			ArmSpeedPID.kp = 100.0f;
			ArmSpeedPID.ki = 0;
			
			//PID_Calc_Arm(&ArmSpeedPID);
			ArmSpeedPID.output=0;
		}
	}
	else//with heavy object; grabbing the box 
	{
		if(RC_CtrlData.rc.s1 == 2)//move back
		{
			if(ArmEncoder.ecd_value < Arm_Encoder_Init - 110 * ARM_Angle_to_Encoder)
			{
				 ArmSpeedPID.ref = 60;
				ArmSpeedPID.kp = 120.0f;
				ArmSpeedPID.ki = 5;//to keep the kicomponent; cancel out with the static torque
			 PID_Calc_Arm(&ArmSpeedPID);
			}

			else if (ArmEncoder.ecd_value <Arm_Encoder_Init - 15 * ARM_Angle_to_Encoder)
    	{       
				ArmSpeedPID.ref = 60;
				ArmSpeedPID.kp = 120.0f;
		    ArmSpeedPID.ki = 0;//abandon kicomponent; use kp only because the static torque is changing
				PID_Calc_Arm(&ArmSpeedPID);
			}
			else//output to speed down
			{
				ArmSpeedPID.ref = 0;
				ArmSpeedPID.kp = 100.0f;
				ArmSpeedPID.ki = 0;
				PID_Calc_Arm(&ArmSpeedPID);
			}
		}
		else if(RC_CtrlData.rc.s1 == 1)//move forward
		{
			if(ArmEncoder.ecd_value > Arm_Encoder_Init - 90 * ARM_Angle_to_Encoder)
			{
			  ArmSpeedPID.ref = -150;
				ArmSpeedPID.kp = 120.0f;
				ArmSpeedPID.ki = 0;
			 PID_Calc_Arm(&ArmSpeedPID);
			}
			else if (ArmEncoder.ecd_value >Arm_Encoder_Init - 170 * ARM_Angle_to_Encoder)//Almost there
			{
				ArmSpeedPID.ref = -150;
				ArmSpeedPID.kp = 120.0f;
				ArmSpeedPID.ki = 0;
				PID_Calc_Arm(&ArmSpeedPID);
			}
			else //when grabbing & lifting: hope the arm will stay still
			{
				ArmSpeedPID.ref = 0;
				ArmSpeedPID.kp = 120.0f;
				ArmSpeedPID.ki = 7;
				PID_Calc_Arm(&ArmSpeedPID);
			}
		}
		else if(RC_CtrlData.rc.s1 == 3)
		{
			ArmSpeedPID.ref = 0;
			ArmSpeedPID.kp = 100.0f;
			ArmSpeedPID.ki = 0;
			ArmSpeedPID.output = 0;
		}
	}
	
	//set output limit
	if(ArmSpeedPID.output>16384)
		ArmSpeedPID.output = 16384;
	else if(ArmSpeedPID.output<-16384)
		ArmSpeedPID.output = -16384;
  setArmSpeed(ArmSpeedPID.output);
}

void contorlTaskInit(void)
{
    Arm_Encoder_Init = ArmEncoder.ecd_value;	
}


void CollectModeSwitch(void)
{	
	switch(collectMode.dumperMode)
	{
		case REST:
			waveLength = 45;
			break;
		case DUMP:
			waveLength = 12;
			break;
		case HOLD:
			waveLength = 30;
			break;
	}
	
	switch(collect_mode_code)
	{
		case 0:
			if(collect_flag==1)
				collect_mode_code++;
			break;
		case 1:
			if(claw_in_place==1)
			{
				collect_mode_code++;
				claw_in_place=0;
				countdown = 1;
			}
			break;
		case 2:
			if(countdown==1)
			{
				start_time_tick_ms = getCurrentTimeTick();
				countdown=0;
			}
			if(getCurrentTimeTick()-start_time_tick_ms > 2000)
			{
				collect_mode_code++;
				countdown=1;
			}
			break;
		case 3:
			if(countdown==1)
			{
				start_time_tick_ms = getCurrentTimeTick();
				countdown=0;
			}
			if(getCurrentTimeTick()-start_time_tick_ms > 2000)
			{
				collect_mode_code++;
				countdown=1;
			}
			break;
		case 4:
			if(countdown==1)
			{
				start_time_tick_ms = getCurrentTimeTick();
				countdown=0;
			}
			if(getCurrentTimeTick()-start_time_tick_ms > 2000)
			{
				collect_mode_code++;
				countdown=1;
			}
			break;
		case 5:
			if(countdown==1)
			{
				start_time_tick_ms = getCurrentTimeTick();
				countdown=0;
			}
			if(getCurrentTimeTick()-start_time_tick_ms > 2000)
			{
				collect_mode_code++;
				countdown=1;
			}
			break;
		case 6:
			if(countdown==1)
			{
				start_time_tick_ms = getCurrentTimeTick();
				countdown=0;
			}
			if(getCurrentTimeTick()-start_time_tick_ms > 2000)
			{
				collect_mode_code++;
				collect_flag = 0;
			}
			break;
	}
	code2mode(collect_mode_code,&collectMode);
}




void CollectClawControl(void)
{	
	if(RC_CtrlData.rc.s2 == 1)
		RESCUE_CLAW_ENABLE();
	else
		RESCUE_CLAW_DISABLE();
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
				PID_Calc_Debug(&GMYPositionPID,1,0.001,0);
			else
				PID_Calc_Debug(&GMYPositionPID,0.5,0.00,5); // a debug version of PID_Calc for testing parameters (P=0.6,I=0.0003,D=8)
			GMYSpeedPID.ref = GMYPositionPID.output;
			GMYSpeedPID.fdb = GMYawEncoder.filter_rate;
			PID_Calc_Debug(&GMYSpeedPID,50,0,0);
			
			GMPPositionPID.ref = 0;
			GMPPositionPID.fdb = GMPitchEncoder.ecd_angle;			
			PID_Calc_Debug(&GMPPositionPID,0.0,0.000,0);
			GMPSpeedPID.ref = GMPPositionPID.output;
			GMPSpeedPID.fdb = GMPitchEncoder.filter_rate;
			PID_Calc_Debug(&GMPSpeedPID,50.0,0.0,0.0);
			set_GM_speed(-GMYSpeedPID.output,-GMPSpeedPID.output);
		}break;
		case NORMAL_STATE:
		{
			GMYPositionPID.ref = 0;
			GMYPositionPID.fdb = GMYawEncoder.ecd_angle;
			PID_Calc_Debug(&GMYPositionPID,1,0,0);
			/*
			if(fabs(GMYPositionPID.ref-GMYPositionPID.fdb)<5.0f)
				PID_Calc_Debug(&GMYPositionPID,0.0,0.000,0);
			else
				PID_Calc_Debug(&GMYPositionPID,0.0,0.0,0.0); // a debug version of PID_Calc for testing parameters (P=0.6,I=0.0003,D=8)
			*/
			//PID_Smart(&GMYPositionPID,10); // cope with non-linear inteval
			
			GMYSpeedPID.ref = GMYPositionPID.output;
			GMYSpeedPID.fdb = GMYawEncoder.filter_rate;
			PID_Calc_Debug(&GMYSpeedPID,100,0.0,0);
			
			GMPPositionPID.ref = Gimbal_Ref.pitch_angle_dynamic_ref;
			GMPPositionPID.fdb = GMPitchEncoder.ecd_angle;
			//PID_Calc_Debug(&GMPPositionPID,0.4,0.001,0);
			
			if(fabs(GMPPositionPID.ref-GMPPositionPID.fdb)<5.0f)
				PID_Calc_Debug(&GMPPositionPID,1,0.001,0);
			else
				PID_Calc_Debug(&GMPPositionPID,0.6,0.0001,5);
			
			GMPSpeedPID.ref = GMPPositionPID.output;
			GMPSpeedPID.fdb = GMPitchEncoder.filter_rate;
			PID_Calc_Debug(&GMPSpeedPID,100,0.0,0.0);
			
			set_GM_speed(-GMYSpeedPID.output,-GMPSpeedPID.output);
		}break;
		default:
		{
			set_GM_speed(0,0);
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
	GMShootControl();
	
	if(time_tick_ms%4==0)
	{
		CM_Control();
	}
	/*
	Collect_Control();
	CollectClawControl();
	*/
}
