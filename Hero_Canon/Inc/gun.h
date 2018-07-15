#ifndef __GUN_H_
#define __GUN_H_
#include "tim.h"

#define SET_PWM1(x) __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, x)
#define SET_PWM2(x) __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, x)

#define InitFrictionWheel()     \
        SET_PWM1(1000);             \
        SET_PWM2(1000);
#define SetFrictionWheelSpeed(x) \
        SET_PWM1(x);                \
        SET_PWM2(x);

void gun_init(void);

#endif
