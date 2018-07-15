#include "gun.h"

void gun_init(void)
{
	HAL_TIM_PWM_Start(&htim12,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim12,TIM_CHANNEL_2);
	//SET_PWM1(2000);//Throttle full
	//SET_PWM2(2000);
	//HAL_Delay(2000);
	SET_PWM1(1000);//Throttle lowest
	SET_PWM2(1000);
	HAL_Delay(3000);
	SET_PWM1(1500);
	SET_PWM2(1500);
	HAL_Delay(5000);
	SET_PWM1(1000);//Throttle lowest
	SET_PWM2(1000);
	//InitFrictionWheel();
}

