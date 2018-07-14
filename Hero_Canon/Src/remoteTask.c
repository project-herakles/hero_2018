#include "remoteTask.h"
#include "controlTask.h"
#include "tim.h"
#include "gpio.h"
#include "pid.h"

Chassis_speed_ref_t chassis_speed_ref;
RC_Ctrl_t RC_CtrlData;
InputMode_e InputMode;
Gimbal_Ref_t Gimbal_Ref;


void remoteTaskInit(void);

void setInputMode(Remote *rc)
{
	if(rc->s2 == 1)
	{
		InputMode = REMOTE_MODE;
	}
	else if(rc->s2 == 3)
	{
		InputMode = MOUSE_KEY_MODE;
	}
	else if(rc->s2 == 2)
	{
		InputMode = STOP_MODE;
	}	
}

InputMode_e getInputMode(void)
{
	return InputMode;
}

void remoteDataProcess(uint8_t* pData){
	if(pData == NULL)
		return;
	RC_CtrlData.rc.ch0 = ((int16_t)pData[0] | ((int16_t)pData[1] << 8)) & 0x07FF; 
  RC_CtrlData.rc.ch1 = (((int16_t)pData[1] >> 3) | ((int16_t)pData[2] << 5)) & 0x07FF;
  RC_CtrlData.rc.ch2 = (((int16_t)pData[2] >> 6) | ((int16_t)pData[3] << 2) |
                       ((int16_t)pData[4] << 10)) & 0x07FF;
  RC_CtrlData.rc.ch3 = (((int16_t)pData[4] >> 1) | ((int16_t)pData[5]<<7)) & 0x07FF;
    
  RC_CtrlData.rc.s1 = ((pData[5] >> 4) & 0x000C) >> 2;
  RC_CtrlData.rc.s2 = ((pData[5] >> 4) & 0x0003);

  RC_CtrlData.mouse.x = ((int16_t)pData[6]) | ((int16_t)pData[7] << 8);
  RC_CtrlData.mouse.y = ((int16_t)pData[8]) | ((int16_t)pData[9] << 8);
  RC_CtrlData.mouse.z = ((int16_t)pData[10]) | ((int16_t)pData[11] << 8);    

  RC_CtrlData.mouse.press_l = pData[12];
  RC_CtrlData.mouse.press_r = pData[13];
 
  RC_CtrlData.key.v = ((int16_t)pData[14]);
	
	setInputMode(&RC_CtrlData.rc);
	
	switch(getInputMode())
	{
		case REMOTE_MODE:
			Remote_Control(&RC_CtrlData.rc);
			break;
		case MOUSE_KEY_MODE:
			MouseKey_Control(&RC_CtrlData.mouse,&RC_CtrlData.key);
			break;
		case STOP_MODE:
			break;
		default:;
	}
}

void Remote_Control(Remote *rc)
{
	if(getWorkState()!= STOP_STATE && getWorkState()!= PREPARE_STATE)
	{
		chassis_speed_ref.forward_back_ref = (rc->ch1 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * STICK_TO_CHASSIS_SPEED_REF_FACT;
    chassis_speed_ref.left_right_ref   = (rc->ch0 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * STICK_TO_CHASSIS_SPEED_REF_FACT; 
		Gimbal_Ref.yaw_angle_dynamic_ref -= (rc->ch2 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * STICK_TO_YAW_ANGLE_INC_FACT;
		Gimbal_Ref.pitch_angle_dynamic_ref += (rc->ch3 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * STICK_TO_PITCH_ANGLE_INC_FACT;
	}
}

void MouseKey_Control(Mouse* mouse,Key* key)
{
	static int16_t forward_back_speed = 0;
	static int16_t left_right_speed = 0;
	if(getWorkState()!=PREPARE_STATE && getWorkState()!=STOP_STATE)
	{
		if(key->v & 0x40) // key: shift
		{
			forward_back_speed =  HIGH_FORWARD_BACK_SPEED;
			left_right_speed = HIGH_LEFT_RIGHT_SPEED;
		}
		else
		{
			forward_back_speed =  NORMAL_FORWARD_BACK_SPEED;
			left_right_speed = NORMAL_LEFT_RIGHT_SPEED;
		}
		if(key->v & 0x01)  // key: w
		{
			chassis_speed_ref.forward_back_ref = forward_back_speed;
		}
		else if(key->v & 0x02) //key: s
		{
			chassis_speed_ref.forward_back_ref = -forward_back_speed;
		}
		else
		{
			chassis_speed_ref.forward_back_ref = 0;
		}
		
		if(key->v & 0x04)  // key: d
		{
			chassis_speed_ref.left_right_ref = -left_right_speed;
		}
		else if(key->v & 0x08) //key: a
		{
			chassis_speed_ref.left_right_ref = left_right_speed;
		}
		else
		{
			chassis_speed_ref.left_right_ref = 0;
		}
	}
	/*
	VAL_LIMIT(mouse->x,-100,100);
	chassis_speed_ref.rotate_ref = mouse->x;
  */	
}
