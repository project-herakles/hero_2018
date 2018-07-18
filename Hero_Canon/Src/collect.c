#include "collect.h"
#include "controlTask.h"
#include "pid.h"
#include "can.h"
#include "remoteTask.h"
#include "stm32f4xx.h"
#include "lifter.h"
#include "claw.h"
extern uint32_t time_tick_ms;
static uint32_t delay_start_time;
Collect_State_t collectState;
Arm_Mode_t Arm_Mode;
uint8_t collectFlag = 0;
uint8_t alignCPflag = 0;
uint8_t errorFlag = 0;
uint8_t grabCPflag = 0;
uint8_t releaseCPflag = 0;
uint8_t ARM_OUT_InPos;
uint8_t ARM_IN_InPos;
PID_Regulator_t ArmPositionPID = ARM_POSITION_PID_DEFAULT;
PID_Regulator_t ArmSpeedPID = ARM_SPEED_PID_DEFAULT;
extern volatile Encoder ArmEncoder;
extern RC_Ctrl_t RC_CtrlData;

void Arm_Control(uint8_t heavy)
{	
	if(ArmSpeedPID.ki == 0)
		ArmSpeedPID.KiComponent = 0;
	ArmSpeedPID.fdb = ArmEncoder.filter_rate;
	if(heavy == 0)//without box
	{
		if(Arm_Mode == ARM_IN)
		{
			if(ArmEncoder.ecd_value < ArmEncoder.ecd_bias - 150 * ARM_Angle_to_Encoder)
			{
				 ArmSpeedPID.ref = 200;
				ArmSpeedPID.kp = 100.0f;
				ArmSpeedPID.ki = 0;
			 PID_Calc_Arm(&ArmSpeedPID);
				ARM_IN_InPos = 0;
			}

			else if (ArmEncoder.ecd_value <ArmEncoder.ecd_bias - 3 * ARM_Angle_to_Encoder)
	   {       
				ArmSpeedPID.ref = 200;
				ArmSpeedPID.kp = 100.0f;
	    	ArmSpeedPID.ki = 0;
				PID_Calc_Arm(&ArmSpeedPID);
				ARM_IN_InPos = 0;
			}
			else
			{
				ArmSpeedPID.ref = 0;
				ArmSpeedPID.kp = 60.0f;
				ArmSpeedPID.ki = 0;
				//ArmSpeedPID.ki = 0.003;
				PID_Calc_Arm(&ArmSpeedPID);
				ARM_IN_InPos = 1;
			}
		}
		else if(Arm_Mode == ARM_OUT )
		{
			if(ArmEncoder.ecd_value > ArmEncoder.ecd_bias - 90 * ARM_Angle_to_Encoder)
			{
        ArmSpeedPID.ref = -200;
				ArmSpeedPID.kp = 100.0f;
				ArmSpeedPID.ki = 0;
			 PID_Calc_Arm(&ArmSpeedPID);
				ARM_OUT_InPos = 0;
			}
			else if (ArmEncoder.ecd_value >ArmEncoder.ecd_bias - 170 * ARM_Angle_to_Encoder)
			{
				ArmSpeedPID.ref = -200;
				ArmSpeedPID.kp = 100.0f;
				ArmSpeedPID.ki = 0;
				PID_Calc_Arm(&ArmSpeedPID);
				ARM_OUT_InPos = 0;
			}
			else 
			{
				ArmSpeedPID.ref = 0;
				ArmSpeedPID.kp = 80.0f;
				ArmSpeedPID.ki = 0.2;
				PID_Calc_Arm(&ArmSpeedPID);
				ARM_OUT_InPos = 1;
			}
		}
		else if(Arm_Mode == ARM_REST)
		{
			ArmSpeedPID.ref = 0;
			ArmSpeedPID.kp = 100.0f;
			ArmSpeedPID.ki = 0;
			
			//PID_Calc_Arm(&ArmSpeedPID);
			ArmSpeedPID.output=0;
		}
	}
	else if (heavy == 1)//with heavy object; grabbing the box 
	{
		if(Arm_Mode == ARM_IN)//move back
		{
			if(ArmEncoder.ecd_value < ArmEncoder.ecd_bias - 110 * ARM_Angle_to_Encoder)
			{
				 ArmSpeedPID.ref = 60;
				ArmSpeedPID.kp = 120.0f;
				ArmSpeedPID.ki = 5;//to keep the kicomponent; cancel out with the static torque
			 PID_Calc_Arm(&ArmSpeedPID);
				ARM_IN_InPos = 0;
			}

			else if (ArmEncoder.ecd_value <ArmEncoder.ecd_bias - 15 * ARM_Angle_to_Encoder)
    	{       
				ArmSpeedPID.ref = 60;
				ArmSpeedPID.kp = 120.0f;
		    ArmSpeedPID.ki = 0;//abandon kicomponent; use kp only because the static torque is changing
				PID_Calc_Arm(&ArmSpeedPID);
				ARM_IN_InPos = 0;
			}
			else//output to speed down
			{
				ArmSpeedPID.ref = 0;
				ArmSpeedPID.kp = 100.0f;
				ArmSpeedPID.ki = 0;
				PID_Calc_Arm(&ArmSpeedPID);
				ARM_IN_InPos = 1;
			}
		}
		else if(Arm_Mode == ARM_OUT)//move forward
		{
			if(ArmEncoder.ecd_value > ArmEncoder.ecd_bias - 90 * ARM_Angle_to_Encoder)
			{
			  ArmSpeedPID.ref = -150;
				ArmSpeedPID.kp = 120.0f;
				ArmSpeedPID.ki = 0;
			 PID_Calc_Arm(&ArmSpeedPID);
				ARM_OUT_InPos = 0;
			}
			else if (ArmEncoder.ecd_value >ArmEncoder.ecd_bias - 170 * ARM_Angle_to_Encoder)//Almost there
			{
				ArmSpeedPID.ref = -150;
				ArmSpeedPID.kp = 120.0f;
				ArmSpeedPID.ki = 0;
				PID_Calc_Arm(&ArmSpeedPID);
				ARM_OUT_InPos = 0;
			}
			else //when grabbing & lifting: hope the arm will stay still
			{
				ArmSpeedPID.ref = 0;
				ArmSpeedPID.kp = 120.0f;
				ArmSpeedPID.ki = 7;
				PID_Calc_Arm(&ArmSpeedPID);
				ARM_OUT_InPos = 1;
			}
		}
		else if(Arm_Mode == ARM_REST)
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


void Collect_Mode_Switch(void)
{
	switch(collectState)
	{
		case COLLECT_REST:
		{
			if(collectFlag==1)
			{
				collectState = COLLECT_ALIGN;
				collectFlag=0;
			}
		}break;
		case COLLECT_ALIGN:
		{
			if(alignCPflag==1)
			{
				collectState = COLLECT_GRAB;
				alignCPflag = 0;
			}
		}break;
		case COLLECT_GRAB:
		{
			if(grabCPflag==1)
			{
				collectState = COLLECT_POUR;
				grabCPflag = 0;
				delay_start_time = getCurrentTimeTick();
			}
		}break;
		case COLLECT_POUR:
		{
			if(getCurrentTimeTick()-delay_start_time > POUR_DELAY)
			{
				collectState = COLLECT_RELEASE;
			}
		}break;
		case COLLECT_RELEASE:
		{
			if(releaseCPflag==1)
			{
				collectState = COLLECT_REST;
				releaseCPflag = 0;
			}
		}break;
		default:
		{
		}
	}
}

void Collect_Control(void)
{
	switch(collectState)
	{
		case COLLECT_REST:
		{
			Arm_Mode = ARM_IN;
			Arm_Control(WITHOUT_BOX);
			CLAW_DISABLE();
			if(getCurrentHeight()!=0)
			{
				Lift_setHeight(HEIGHT0);
			}
		}break;
		case COLLECT_ALIGN:
		{
			Arm_Mode = ARM_IN;
			Arm_Control(WITHOUT_BOX);
			CLAW_DISABLE();
		}break;
		case COLLECT_GRAB:
		{
			Arm_Mode = ARM_OUT;
			Arm_Control(WITHOUT_BOX);
			if(ARM_OUT_InPos==1)
			{
				CLAW_ENABLE();
			}
		}break;
		case COLLECT_POUR:
		{
			Arm_Mode = ARM_IN;
			Arm_Control(WITH_BOX);
			if(ARM_IN_InPos==1)
			{
				
			}
		}break;
		case COLLECT_RELEASE:
		{
			Arm_Mode = ARM_OUT;
			Arm_Control(WITH_BOX);
			if(ARM_OUT_InPos==1)
			{
				CLAW_DISABLE();
			}
		}break;
		default:
		{
		}
	}
}
