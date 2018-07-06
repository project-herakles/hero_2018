#include "ramp.h"
#include "controlTask.h"

void getRampedRef(PID_Regulator_t* pid,float step_ref,float slope)
{
	if(fabs(pid->ref-step_ref)>1.0f)
		(pid->ref-step_ref>1.0f) ? (pid->ref -= slope) : (pid->ref += slope);
	else
		pid->ref = step_ref;
}

static uint8_t rampFlag = 0;
static uint32_t fric_start_time = 0;
//This function ramps the input for friction wheel
//start is the inital PWM value, end is the end PWM value, duration is in ms
uint16_t FrictionRamp(uint16_t start,uint16_t end,uint32_t duration)
{
	if(rampFlag==0)
	{
		fric_start_time = getCurrentTimeTick();
		rampFlag = 1;
		return start;
	}
	else
	{
		if(getCurrentTimeTick()-fric_start_time > duration)
		{
			rampFlag = 0;
			return end;
		}
		else
		{
			return start+(end-start)*(getCurrentTimeTick()-fric_start_time)/duration;
		}
	}
	
}
