#include "lifter.h"
#include "stepper.h"
#include "controlTask.h"
#include "gpio.h"

extern Stepper_Regulator_t stepper_left;
extern Stepper_Regulator_t stepper_right;
const float pi = 3.1415926;
uint32_t lift_start_time = 0;
uint16_t currentHeight = 0;
uint8_t setHeightFlag = 0;

void Lift_init(void)
{
	stepper_init(&stepper_left);
	stepper_init(&stepper_right);
}

void Lift_stop(void)
{
	stepper_disable(&stepper_left);
	stepper_disable(&stepper_right);
}
float degree2turn;
void Lift_setHeight(uint16_t mm)
{
	if(setHeightFlag==0) // calls rotate function once
	{
		lift_start_time = getCurrentTimeTick();
		if(currentHeight < mm)
		{
			degree2turn = (mm-currentHeight)/RADIUS*(180.0f/pi);
			stepper_rotate(&stepper_left,CLOCKWISE,(mm-currentHeight)/RADIUS*(180.0f/pi));
			stepper_rotate(&stepper_right,COUNTER_CLOCKWISE,(mm-currentHeight)/RADIUS*(180.0f/pi));
			setHeightFlag = 1;
		}
		else if(currentHeight > mm)
		{
			degree2turn = (mm-currentHeight)/RADIUS*(180.0f/pi);
			//Lift_hold(); // going downwards. Lift up the frame a bit to ensure successful unlatch
			stepper_rotate(&stepper_left,COUNTER_CLOCKWISE,(currentHeight-mm)/RADIUS*(180.0f/pi));
			stepper_rotate(&stepper_right,CLOCKWISE,(currentHeight-mm)/RADIUS*(180.0f/pi));
			setHeightFlag = 1;
		}
	}
	
	if(getCurrentTimeTick()-lift_start_time > LATCH_DELAY)
	{
		LIFT_LATCHED();
	}
	else
	{
		LIFT_UNLATCH();
	}
	
	if(stepper_left.pulses==0 && stepper_right.pulses==0)
	{
		setHeightFlag = 0;
		currentHeight = mm;
	}
}

void Lift_hold(void)
{
	Lift_setHeight(currentHeight+5);
}
void Lift_idle(void)
{
}

uint16_t getCurrentHeight(void)
{
	return currentHeight;
}