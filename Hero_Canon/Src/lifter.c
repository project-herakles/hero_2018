#include "lifter.h"
#include "stepper.h"

extern Stepper_Regulator_t stepper_left;
extern Stepper_Regulator_t stepper_right;
uint16_t currentHeight = 0;

void Lift_init(void)
{
}
void Lift_setHeight(uint16_t mm)
{
}
void Lift_hold(void)
{
}
void Lift_idle(void)
{
}

uint16_t getCurrentHeight(void)
{
	return currentHeight;
}